// SPDX-License-Identifier: GPL-2.0-or-later OR MIT
/*
 * Regulator driver for the PWM controlled voltage regulators of HiSilicon STB
 * SoCs
 *
 * Copyright (c) 2023 David Yang
 */

#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rational.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

/*
 * Duty cycle + 1 and period + 1 are counted in periods of the oscillator
 * clock (usually 24 MHz), and a larger duty cycle means a lower output
 * voltage.
 */
#define HISTB_DUTY		GENMASK(31, 16)
#define HISTB_PERIOD		GENMASK(15, 0)
/* 24000 kHz / 4094 ~= 6 kHz, the lowest switching frequency in use */
#define HISTB_COUNTER_MAX	0xfff
#define HISTB_PERIOD_MIN	FIELD_PREP(HISTB_PERIOD, 2)
#define HISTB_DUTY_ZERO		FIELD_PREP(HISTB_DUTY, 1)
#define HISTB_DUTY_FULL		FIELD_PREP(HISTB_DUTY, 2)

struct histb_regulator {
	struct regulator_desc desc;
	void __iomem *base;
	int min_uV;
	int max_uV;
};

static int histb_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct histb_regulator *reg = rdev_get_drvdata(rdev);
	u32 val, duty, period;
	u64 uV;

	val = readl_relaxed(reg->base);
	period = FIELD_GET(HISTB_PERIOD, val);
	duty = FIELD_GET(HISTB_DUTY, val);

	/* an empty counter range leaves the output high */
	if (duty <= 1 || period <= 1)
		return reg->max_uV;

	duty--;
	period--;
	if (duty >= period)
		return reg->min_uV;

	uV = (u64)(reg->max_uV - reg->min_uV) * duty;

	return reg->max_uV - DIV_ROUND_CLOSEST_ULL(uV, period);
}

static int histb_regulator_set_voltage(struct regulator_dev *rdev, int min_uV,
				       int max_uV, unsigned int *selector)
{
	struct histb_regulator *reg = rdev_get_drvdata(rdev);
	unsigned long duty, period;
	u32 val;

	if (min_uV >= reg->max_uV) {
		val = HISTB_DUTY_ZERO | HISTB_PERIOD_MIN;
	} else if (min_uV <= reg->min_uV) {
		val = HISTB_DUTY_FULL | HISTB_PERIOD_MIN;
	} else {
		rational_best_approximation(reg->max_uV - min_uV,
					    reg->max_uV - reg->min_uV,
					    HISTB_COUNTER_MAX, HISTB_COUNTER_MAX,
					    &duty, &period);
		val = FIELD_PREP(HISTB_DUTY, duty + 1) |
		      FIELD_PREP(HISTB_PERIOD, period + 1);
	}

	writel_relaxed(val, reg->base);

	return 0;
}

static const struct regulator_ops histb_regulator_ops = {
	.get_voltage = histb_regulator_get_voltage,
	.set_voltage = histb_regulator_set_voltage,
};

static int histb_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct histb_regulator *reg;
	struct regulator_init_data *init_data;
	struct regulator_config config = {};
	struct regulator_dev *rdev;

	reg = devm_kzalloc(dev, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return -ENOMEM;

	init_data = of_get_regulator_init_data(dev, dev->of_node, &reg->desc);
	if (!init_data)
		return -ENOMEM;

	reg->min_uV = init_data->constraints.min_uV;
	reg->max_uV = init_data->constraints.max_uV;
	if (reg->min_uV <= 0 || reg->max_uV <= reg->min_uV) {
		dev_err(dev, "invalid voltage range (%d..%d uV)\n",
			reg->min_uV, reg->max_uV);
		return -EINVAL;
	}

	reg->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(reg->base))
		return PTR_ERR(reg->base);

	reg->desc.name = dev_name(dev);
	reg->desc.type = REGULATOR_VOLTAGE;
	reg->desc.ops = &histb_regulator_ops;
	reg->desc.owner = THIS_MODULE;
	reg->desc.continuous_voltage_range = true;

	config.dev = dev;
	config.init_data = init_data;
	config.driver_data = reg;
	config.of_node = dev->of_node;

	rdev = devm_regulator_register(dev, &reg->desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "failed to register %s\n", reg->desc.name);

	return 0;
}

static const struct of_device_id histb_regulator_of_match[] = {
	{ .compatible = "hisilicon,histb-volt" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, histb_regulator_of_match);

static struct platform_driver histb_regulator_driver = {
	.probe = histb_regulator_probe,
	.driver = {
		.name = "histb-regulator",
		.of_match_table = histb_regulator_of_match,
	},
};
module_platform_driver(histb_regulator_driver);

MODULE_DESCRIPTION("HiSilicon STB PWM voltage regulator driver");
MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");
