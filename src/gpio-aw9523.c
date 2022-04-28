/*
 * AW9523B 16 bit I/O ports with LED driver
 *
 * Copyright (C) 2020 Markus Zehnder <zehnder@live.com>
 * Derived from Android drivers found on the web and rewritten for Linux.
 * Interrupt handling derived from drivers/gpio/gpio-pcf857x driver.
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/leds.h>

#include "gpio-aw9523.h"

#define AW_DRV_NAME    "aw9523b-gpio"
#define AW_DRV_VERSION "20210129-03"

/* at the moment only one chip type */
#define AW_GPIO_MASK	  0x00FF
#define AW9523B_TYPE	  0x1000
#define AW9523B_TYPE_MASK 0xF000

// clang-format off
static const struct i2c_device_id aw9523b_id[] = {
	{ "aw9523b", 16 | AW9523B_TYPE },
	{}
};
MODULE_DEVICE_TABLE(i2c, aw9523b_id);
// clang-format on

static const struct of_device_id aw9523b_of_table[] = {
	{ .compatible = "awinic,aw9523b" },
	{}
};
MODULE_DEVICE_TABLE(of, aw9523b_of_table);

/*-------------------------------------------------------------------------*/

static int aw9523_read_reg(struct aw9523b *aw, u8 reg, unsigned int *val)
{
	return regmap_read(aw->regmap, reg, val);
}

static int aw9523_write_reg(struct aw9523b *aw, u8 reg, u8 val)
{
	return regmap_write(aw->regmap, reg, val);
}

static int aw_set_mask(struct aw9523b *aw, unsigned int reg, unsigned int mask,
		       bool enabled)
{
	u16 val = enabled ? 0xff : 0x00;
	return regmap_update_bits(aw->regmap, reg, mask, val);
}

static int aw_set_bit(struct aw9523b *aw, unsigned int reg, unsigned int pin,
		      bool enabled)
{
	u16 mask = BIT(pin);
	return aw_set_mask(aw, reg, mask, enabled);
}

/*-------------------------------------------------------------------------*/

static int aw9523_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct aw9523b *aw = gpiochip_get_data(chip);
	u8		reg_num;
	int		ret;

	if (offset < 8) {
		reg_num = REG_GPIO_CFG_P0;
	} else {
		reg_num = REG_GPIO_CFG_P1;
		offset	= offset % 8;
	}

	AW_DEBUG(aw->dev, "Set input: offset=%d, reg=0x%02x\n", offset,
		 reg_num);

	mutex_lock(&aw->lock);
	ret = aw_set_bit(aw, reg_num, offset, true);
	mutex_unlock(&aw->lock);

	if (ret < 0)
		dev_err(aw->dev,
			"Failed setting direction input, reg=0x%02x, offset=%d, ret=%d\n",
			reg_num, offset, ret);

	return ret;
}

static int aw9523_direction_output(struct gpio_chip *chip, unsigned offset,
				   int val)
{
	struct aw9523b *aw = gpiochip_get_data(chip);
	u8		reg_num_out, reg_num_cfg;
	int		ret;

	if (offset <= 7) {
		reg_num_out = REG_GPIO_OUT_P0;
		reg_num_cfg = REG_GPIO_CFG_P0;
	} else {
		reg_num_out = REG_GPIO_OUT_P1;
		reg_num_cfg = REG_GPIO_CFG_P1;
		offset	    = offset >> 8;
	}

	AW_DEBUG(
		aw->dev,
		"Set output direction: offset=%d, out_reg=0x%02x, val=0x%02x, cfg_reg=0x%02x\n",
		offset, reg_num_out, val, reg_num_cfg);

	mutex_lock(&aw->lock);
	ret = aw_set_bit(aw, reg_num_out, offset, val);
	if (ret == 0) {
		ret = aw_set_bit(aw, reg_num_cfg, offset, true);
	}
	mutex_unlock(&aw->lock);

	if (ret < 0)
		dev_err(aw->dev,
			"Failed setting direction output, reg=0x%02x, offset=%d, ret=%d\n",
			reg_num_out, offset, ret);

	return ret;
}

static int aw9523_gpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	struct aw9523b *aw = gpiochip_get_data(chip);
	u8		reg_num;
	unsigned int	reg_val;
	int		ret;
	int		gpio_bits;

	AW_DEBUG(aw->dev, "%s: enter, offset=%d\n", __func__, offset);

	mutex_lock(&aw->lock);

	if (offset < 8) {
		reg_num	  = REG_GPIO_IN_P0;
		gpio_bits = offset;
	} else {
		reg_num	  = REG_GPIO_IN_P1;
		gpio_bits = offset - 8;
	}

	/* TODO regmap read bit function? */
	ret = aw9523_read_reg(aw, reg_num, &reg_val);

	mutex_unlock(&aw->lock);

	if (ret < 0) {
		/* not much we can do here */
		return 0;
	}
	AW_DEBUG(aw->dev, "%s: value=0x%02x\n", __func__, reg_val);

	return (reg_val & (1 << gpio_bits)) ? 1 : 0;
}

static void aw9523_gpio_set_value(struct gpio_chip *chip, unsigned offset,
				  int val)
{
	struct aw9523b *aw = gpiochip_get_data(chip);
	u8		reg_num;
	int		ret;
	unsigned mask;

	AW_DEBUG(aw->dev, "%s: enter, offset=%d, value=%d\n", __func__, offset,
		 val);

	if (offset < 8) {
		reg_num = REG_GPIO_OUT_P0;
	} else {
		reg_num = REG_GPIO_OUT_P1;
		offset	= offset >> 8;
	}

	mask = BIT(offset);

	mutex_lock(&aw->lock);
	ret = aw_set_mask(aw, reg_num, mask, !!val);
	mutex_unlock(&aw->lock);

	/* TODO check if regmap already prints error messages */
	if (ret < 0)
		dev_err(aw->dev,
			"Failed setting gpio value, reg=0x%02x, offset=%d, mask=0x02%x, ret=%d\n",
			reg_num, offset, mask, ret);
}

static int aw9523_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct aw9523b *aw = gpiochip_get_data(chip);

	if (aw->led_data && aw->led_data->led_mask & (0x1 << offset)) {
		dev_warn(aw->dev, "gpio offset %d is reserved for LED driver\n",
			 offset);
		return -EINVAL;
	}

	return 0;
}

/* For testing only */
static int aw9523_gpio_set_config(struct gpio_chip *chip, unsigned int offset,
				  unsigned long config)
{
	struct aw9523b *aw __maybe_unused = gpiochip_get_data(chip);

	AW_DEBUG(aw->dev, "Set gpio config: offset=%d, config=0x%04lx\n",
		 offset, config);

	return -ENOTSUPP;
}

/*-------------------------------------------------------------------------*/

static irqreturn_t aw9523_irq_handler(int irq, void *data)
{
	struct aw9523b *aw     = data;
	unsigned int	status = 0, status_p0, status_p1;
	unsigned long	change, i;
	int		ret;

	AW_DEBUG(aw->dev, "%s: enter, irq=%d, old_status=0x%04x\n", __func__,
		 irq, aw->status);

	/* TODO mutex around aw9523_read_reg required? */
	mutex_lock(&aw->lock);

	/* Reading REG_GPIO_IN_P0 / P1 clears the IRQ */
	/* Reading REG_GPIO_IN_P0 & P1 with regmap_bulk_read misses P1 changes! */
	ret = aw9523_read_reg(aw, REG_GPIO_IN_P0, &status_p0);
	if (ret >= 0)
		ret = aw9523_read_reg(aw, REG_GPIO_IN_P1, &status_p1);

	if (ret >= 0) {
		status	   = (status_p1 << 8) | status_p0;
		change	   = (aw->status ^ status) & aw->irq_enabled;
		aw->status = status;
	}

	mutex_unlock(&aw->lock);

	if (ret < 0) {
		dev_err(aw->dev, "Error reading P0 or P1 status. Error: %d",
			ret);
		return IRQ_HANDLED;
	}

	AW_DEBUG(aw->dev, "%s: p0=0x%02x, p1=0x%02x, change=0x%04lx\n",
		 __func__, status_p0, status_p1, change);

	for_each_set_bit (i, &change, aw->chip.ngpio) {
		unsigned int nestedirq =
			irq_find_mapping(aw->chip.irq.domain, i);
		AW_DEBUG(aw->dev, "%s: handle nested irq=%d, bit=%ld\n",
			 __func__, nestedirq, i);
		if (nestedirq)
			handle_nested_irq(nestedirq);
	}

	AW_DEBUG(aw->dev, "%s: exit\n", __func__);

	return IRQ_HANDLED;
}

/* TODO untested! */
static int aw9523_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct aw9523b *aw  = irq_data_get_irq_chip_data(data);
	int		ret = 0;

	AW_DEBUG(aw->dev, "%s: enter, on=%d\n", __func__, on);

	if (aw->irq_parent) {
		ret = irq_set_irq_wake(aw->irq_parent, on);
		if (ret) {
			dev_info(aw->dev,
				 "irq %u doesn't support irq_set_wake\n",
				 aw->irq_parent);
			aw->irq_parent = 0;
		}
	}
	return ret;
}

/* FIXME shouldn't we write that to REG_INT_P0 / P1? */
static void aw9523_irq_enable(struct irq_data *data)
{
	struct aw9523b *aw = irq_data_get_irq_chip_data(data);

	aw->irq_enabled |= (1 << data->hwirq);

	AW_DEBUG(aw->dev, "%s: enter, hwirq=%ld, irq_enabled=0x%04x\n",
		 __func__, data->hwirq, aw->irq_enabled);
}

/* FIXME shouldn't we write that to REG_INT_P0 / P1? */
static void aw9523_irq_disable(struct irq_data *data)
{
	struct aw9523b *aw = irq_data_get_irq_chip_data(data);

	aw->irq_enabled &= ~(1 << data->hwirq);

	AW_DEBUG(aw->dev, "%s: enter, hwirq=%ld, irq_enabled=0x%04x\n",
		 __func__, data->hwirq, aw->irq_enabled);
}

static void aw9523_irq_bus_lock(struct irq_data *data)
{
	struct aw9523b *aw = irq_data_get_irq_chip_data(data);

	AW_DEBUG(aw->dev, "%s: enter\n", __func__);

	mutex_lock(&aw->lock);
}

static void aw9523_irq_bus_sync_unlock(struct irq_data *data)
{
	struct aw9523b *aw = irq_data_get_irq_chip_data(data);

	AW_DEBUG(aw->dev, "%s: enter\n", __func__);

	mutex_unlock(&aw->lock);
}

/*-------------------------------------------------------------------------*/

static int aw9523_set_led_imax(struct aw9523b *aw, aw9523_imax_t imax)
{
	int	     ret;
	unsigned int reg_val;

	if (imax > E_VAL_MAX) {
		dev_warn(aw->dev, "Invalid LED Imax value %d: setting to 1/4\n",
			 imax);
		imax = ONE_QUARTER;
	}

	ret = aw9523_read_reg(aw, REG_CTL, &reg_val);
	if (ret < 0)
		return ret;

	/* TODO regmap bit functions? */
	reg_val &= ~0x03;
	reg_val |= (imax & 0x03);

	AW_DEBUG(aw->dev,
		 "Setting LED Imax value %d. Control register: 0x%02x\n", imax,
		 ret);
	return aw9523_write_reg(aw, REG_CTL, reg_val);
}

static int aw9523_set_dim_by_mask(struct aw9523b *aw, unsigned int mask,
				  unsigned int brightness)
{
	int ret	   = 0;
	int i;
	u8 *adjust = aw->led_data ? aw->led_data->brightness_adjust : NULL;

	AW_DEBUG(aw->dev, "%s: enter\n", __func__);

	mutex_lock(&aw->lock);

	AW_DEBUG(aw->dev, "%s: inside mutex\n", __func__);
	for (i = 0; i < AW9523B_MAX_LEDS; i++) {
		if (mask & (0x1 << i)) {
			/* map led mask to output register */
			u8 reg = i < 8 ? REG_LED_DIM4_P0_0 + i :
					 i > 11 ?
					 REG_LED_DIM12_P1_4 + (i - 12) :
					 REG_LED_DIM0_P1_0 + (i - 8);
			u8 value = brightness;
			if (adjust && adjust[i] > 0 && brightness > 0) {
				value = brightness * adjust[i] / 100;
				AW_DEBUG(aw->dev,
					 "Adjusted led %d brightness to %d\n",
					 i, value);
			}
			AW_DEBUG(aw->dev, "%s: write brightness led %d\n",
				 __func__, i);
			ret = aw9523_write_reg(aw, reg, value);
			if (ret < 0) {
				return ret;
			}
		}
	}
	mutex_unlock(&aw->lock);
	AW_DEBUG(aw->dev, "%s: exit\n", __func__);

	return ret;
}

int aw9523_brightness_set(struct led_classdev *led_cdev,
			  enum led_brightness  brightness)
{
	struct aw9523b_led_dev *led_dev =
		container_of(led_cdev, struct aw9523b_led_dev, cdev);

	AW_DEBUG(led_dev->parent->dev, "%s: enter\n", __func__);

	if (brightness > led_dev->max_brightness) {
		brightness = led_dev->max_brightness;
	}

	AW_DEBUG(led_dev->parent->dev, "Set LED mask 0x%02x brightness: %d\n",
		 led_dev->led_mask, brightness);

	led_dev->cdev.brightness = brightness;
	led_dev->cur_brightness	 = brightness;

	return aw9523_set_dim_by_mask(led_dev->parent, led_dev->led_mask,
				      brightness);
}

enum led_brightness aw9523_brightness_get(struct led_classdev *led_cdev)
{
	struct aw9523b_led_dev *led_dev =
		container_of(led_cdev, struct aw9523b_led_dev, cdev);
	return led_dev->cur_brightness;
}

static int aw9523_led_chip_init(struct aw9523b *aw)
{
	struct aw9523b_led *led = aw->led_data;
	int		    ret;
	int			i;

	/* led_mask = led outputs as 1. Chip: 0 = LED mode, 1 = GPIO mode */
	ret = aw9523_write_reg(aw, REG_LED_MODE_P0, ~(led->led_mask & 0xFF));
	if (ret < 0)
		return ret;
	ret = aw9523_write_reg(aw, REG_LED_MODE_P1,
			       ~((led->led_mask >> 8) & 0xFF));
	if (ret < 0)
		return ret;

	ret = aw9523_set_led_imax(aw, led->imax);
	if (ret < 0)
		return ret;

	for (i = 0; i < led->count; i++) {
		ret = aw9523_set_dim_by_mask(aw, led->led_devs[i].led_mask,
					     led->led_devs[i].cur_brightness);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static void aw9523_led_free_resources(struct aw9523b *aw)
{
	int i;

	AW_DEBUG(aw->dev, "%s: enter\n", __func__);

	if (aw->led_data) {
		for (i = 0; i < aw->led_data->count; i++) {
			AW_DEBUG(aw->dev, "%s: led_classdev_unregister %d\n",
				 __func__, i);
			devm_led_classdev_unregister(
				aw->dev, &aw->led_data->led_devs[i].cdev);
		}
		devm_kfree(aw->dev, aw->led_data);
	}

	AW_DEBUG(aw->dev, "%s: exit\n", __func__);
}

static int aw9523_parse_led_devs(struct device_node *led_node,
				 struct aw9523b *aw, struct aw9523b_led *led)
{
	struct aw9523b_led_dev *led_devs = led->led_devs;
	struct device_node *	child;
	u32			led_count;
	u32 *			led_array;
	u32 *			adjust_array;
	int			i = 0;
	int			j;
	int			ret;

	for_each_child_of_node (led_node, child) {
		adjust_array = led_array = NULL;

		led_devs[i].cdev.name =
			of_get_property(child, "label", NULL) ?: child->name;
		led_devs[i].cdev.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);

		ret = of_property_count_u32_elems(child, "aw9523,leds");
		if (ret < 0 || ret > AW9523B_MAX_LEDS) {
			dev_err(aw->dev,
				"aw9523,leds property missing or invalid, idx = %d\n",
				i);
			return ret;
		}
		led_count = ret;

		led_array = kzalloc(sizeof(u32) * led_count, GFP_KERNEL);
		if (led_array == NULL) {
			dev_err(aw->dev, "%s: memory allocation failed\n",
				__func__);
			return -ENOMEM;
		}
		ret = of_property_read_u32_array(child, "aw9523,leds",
						 led_array, led_count);
		if (ret < 0) {
			dev_err(aw->dev,
				"dts parse aw9523,leds array failed, idx = %d\n",
				i);
			goto fail;
		}

		/* optional brightness adjustment factor (per led) */
		if (of_property_count_u32_elems(
			    child, "aw9523,leds_bright_adjust") == led_count) {
			adjust_array =
				kzalloc(sizeof(u32) * led_count, GFP_KERNEL);
			if (adjust_array == NULL) {
				dev_err(aw->dev,
					"%s: memory allocation failed\n",
					__func__);
				ret = -ENOMEM;
				goto fail;
			}

			ret = of_property_read_u32_array(
				child, "aw9523,leds_bright_adjust",
				adjust_array, led_count);
			if (ret < 0) {
				dev_err(aw->dev,
					"aw9523,leds_bright_adjust array property parsing failed, idx = %d\n",
					i);
				goto fail;
			}
		}
		for (j = 0; j < led_count; j++) {
			u32 led_idx = led_array[j];
			if (led_idx >= AW9523B_MAX_LEDS) {
				dev_err(aw->dev,
					"dts error: aw9523,leds index %d unavailable, idx =  %d\n",
					led_idx, i);
				goto fail;
			}

			led_devs[i].led_mask |= (0x01 << led_idx);

			if (adjust_array && (adjust_array[j] != 100) &&
			    (adjust_array[j] < 255)) {
				AW_DEBUG(aw->dev,
					 "Brightness adjustment led %d: %d%%\n",
					 led_idx, adjust_array[j]);
				led->brightness_adjust[led_idx] =
					adjust_array[j];
			}
		}

		ret = of_property_read_u32(child, "aw9523,default_brightness",
					   &led_devs[i].cur_brightness);
		if (ret < 0) {
			dev_err(aw->dev,
				"aw9523,default_brightness property missing or invalid, idx = %d\n",
				i);
			goto fail;
		}
		ret = of_property_read_u32(child, "aw9523,max_brightness",
					   &led_devs[i].max_brightness);
		if (ret < 0 || led_devs[i].max_brightness > 255) {
			dev_err(aw->dev,
				"aw9523,max_brightness property missing or invalid, idx = %d\n",
				i);
			goto fail;
		}

		/* update main led mask */
		led->led_mask |= led_devs[i].led_mask;

		AW_DEBUG(
			aw->dev,
			"aw9523 led device %d: %s, trigger=%s, led# = %d, mask = 0x%02x\n",
			i, led_devs[i].cdev.name,
			led_devs[i].cdev.default_trigger, led_count,
			led_devs[i].led_mask);

		kfree(led_array);
		kfree(adjust_array);
		i++;
	}

	AW_DEBUG(aw->dev, "Parsed %d led device(s). Main led_mask = 0x%02x\n",
		 i, led->led_mask);

	/* register led classdev */
	for (i = 0; i < led->count; i++) {
		led_devs[i].cdev.max_brightness = led_devs[i].max_brightness;
		led_devs[i].cdev.brightness_set_blocking =
			aw9523_brightness_set;
		led_devs[i].cdev.brightness_get = aw9523_brightness_get;
		led_devs[i].parent		= aw;
		ret = devm_led_classdev_register(aw->dev, &led_devs[i].cdev);
		if (ret) {
			dev_err(aw->dev,
				"led classdev register failed, idx = %d\n", i);
			for (i--; i >= 0; i--) {
				devm_led_classdev_unregister(aw->dev,
							     &led_devs[i].cdev);
			}
			return ret;
		}
	}

	return 0;

fail:
	kfree(led_array);
	kfree(adjust_array);
	return ret;
}

static int aw9523_led_init(struct aw9523b *aw, struct device_node *led_node)
{
	unsigned int	    prop_val;
	int		    ret = 0;
	int			i;
	struct aw9523b_led *led = NULL;

	AW_DEBUG(aw->dev, "Initializing LED feature...\n");

	led = devm_kzalloc(aw->dev, sizeof(struct aw9523b_led), GFP_KERNEL);
	if (led == NULL) {
		dev_err(aw->dev, "%s: memory allocation failed\n", __func__);
		return -ENOMEM;
	}

	ret = of_property_read_u32(led_node, "aw9523,default_imax", &prop_val);
	if (ret < 0) {
		dev_err(aw->dev, "Missing property aw9523,default_imax\n");
		goto fail_led;
	}
	led->imax = prop_val;
	if (led->imax > E_VAL_MAX) {
		dev_err(aw->dev,
			"Invalid 'aw9523,default_imax' value %d. Allowed: %d..%d\n",
			led->imax, E_VAL_MIN, E_VAL_MAX);
		ret = -EINVAL;
		goto fail_led;
	}

	led->count = of_get_child_count(led_node);
	if (led->count < 1 || led->count > AW9523B_MAX_LEDS) {
		dev_err(aw->dev,
			"Invalid number of led child nodes: %d. Allowed: 1..%d\n",
			led->count, AW9523B_MAX_LEDS);
		ret = -EINVAL;
		goto fail_led;
	}
	AW_DEBUG(aw->dev, "LED Imax = %d, LED count = %d\n", led->imax,
		 led->count);

	led->led_devs =
		devm_kzalloc(aw->dev,
			     sizeof(struct aw9523b_led_dev) * led->count,
			     GFP_KERNEL);
	if (led->led_devs == NULL) {
		dev_err(aw->dev, "%s: memory allocation failed\n", __func__);
		ret = -ENOMEM;
		goto fail_led;
	}

	ret = aw9523_parse_led_devs(led_node, aw, led);
	if (ret)
		goto fail_led_devs;

	aw->led_data = led;

	ret = aw9523_led_chip_init(aw);
	if (ret)
		goto fail_led_class;

	return 0;

fail_led_class:
	for (i = 0; i < led->count; i++) {
		devm_led_classdev_unregister(aw->dev, &led->led_devs[i].cdev);
	}
fail_led_devs:
	devm_kfree(aw->dev, led->led_devs);
fail_led:
	devm_kfree(aw->dev, led);
	aw->led_data = NULL;
	return ret;
}

/*-------------------------------------------------------------------------*/

static int aw9523_probe(struct i2c_client *	    client,
			const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct aw9523b *    aw;
	struct device_node *led_node = NULL;
	int		    ret	     = 0;
	unsigned int reg_val;

	dev_dbg(&client->dev, "Probing version %s\n", AW_DRV_VERSION);

	if (!(IS_ENABLED(CONFIG_OF) && np)) {
		dev_err(&client->dev,
			"No device tree data for '%s'! Platform data not supported.\n",
			client->name);
		return -EINVAL;
	}

	aw = devm_kzalloc(&client->dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	aw->dev	   = &client->dev;
	aw->regmap = devm_regmap_init_i2c(client, &aw9523b_regmap);
	if (IS_ERR(aw->regmap)) {
		dev_err(aw->dev, "regmap initialization failed.\n");
		devm_kfree(aw->dev, aw);
		return PTR_ERR(aw->regmap);
	}

	i2c_set_clientdata(client, aw);

	aw->reset_gpio = devm_gpiod_get(aw->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset_gpio))
		return PTR_ERR(aw->reset_gpio);
	gpiod_set_consumer_name(aw->reset_gpio, "aw9523b-reset");

	aw->chip.label	= AW_DRV_NAME;
	aw->chip.parent = aw->dev;
	aw->chip.owner	= THIS_MODULE;

	aw->chip.request	  = aw9523_gpio_request;
	aw->chip.direction_input  = aw9523_direction_input;
	aw->chip.direction_output = aw9523_direction_output;
	aw->chip.get		  = aw9523_gpio_get_value;
	aw->chip.set		  = aw9523_gpio_set_value;
	aw->chip.set_config	  = aw9523_gpio_set_config,

	aw->chip.base	   = -1; /* allocate in free space */
	aw->chip.ngpio	   = id->driver_data & AW_GPIO_MASK;
	aw->chip.can_sleep = true; /* TODO verify sleep */

	aw->chip.of_node = np; /* TODO research */

	/* TODO aw->chip.dbg_show = aw9523_dbg_show; */

	mutex_init(&aw->lock);

	AW_DEBUG(aw->dev, "Testing chip identification...\n");

	ret = aw9523_read_reg(aw, REG_ID, &reg_val);
	if (ret >= 0 && reg_val != AW9523B_ID) {
		dev_err(aw->dev, "Expected chip id 0x%02x but got: 0x%02x\n",
			AW9523B_ID, reg_val);
		ret = -EIO;
	}
	if (ret < 0)
		goto fail_pre_gpiochip;

	AW_DEBUG(aw->dev, "Initializing...\n");

	/* Reset the aw9523b */
	ret = aw9523_write_reg(aw, REG_RESET, 0x00);
	if (ret < 0)
		goto fail_pre_gpiochip;

	/* Switch P0 to Push-Pull Mode if configured*/
	if (of_property_read_bool(np, "aw9523,p0-output-push-pull")) {
		AW_DEBUG(aw->dev, "Setting P0 output to push-pull mode\n");
		ret = aw9523_read_reg(aw, REG_CTL, &reg_val);
		if (ret < 0)
			goto fail_pre_gpiochip;
		/* TODO regmap bit set function? */
		reg_val |= 0x10;
		ret = aw9523_write_reg(aw, REG_CTL, reg_val);
		if (ret < 0)
			goto fail_pre_gpiochip;
	}

	led_node = of_get_child_by_name(client->dev.of_node, "aw9523,led");
	if (led_node) {
		ret = aw9523_led_init(aw, led_node);
		if (ret) {
			dev_err(aw->dev, "LED feature init failed \n");
			goto fail_pre_gpiochip;
		}
	}

	ret = devm_gpiochip_add_data(aw->dev, &aw->chip, aw);
	if (ret < 0) {
		dev_err(aw->dev, "Failed to add gpiochip data\n");
		goto fail_pre_gpiochip;
	}

	/* Enable irqchip if we have an interrupt */
	if (client->irq) {
		AW_DEBUG(aw->dev, "Setting up irq handling...\n");

		aw->irqchip.name		= "aw9523b";
		aw->irqchip.irq_enable		= aw9523_irq_enable;
		aw->irqchip.irq_disable		= aw9523_irq_disable;
		aw->irqchip.irq_set_wake	= aw9523_irq_set_wake;
		aw->irqchip.irq_bus_lock	= aw9523_irq_bus_lock;
		aw->irqchip.irq_bus_sync_unlock = aw9523_irq_bus_sync_unlock;

		/* TODO verify handle_level_irq. Alternative: handle_edge_irq */
		ret = gpiochip_irqchip_add_nested(&aw->chip, &aw->irqchip, 0,
						  handle_level_irq,
						  IRQ_TYPE_NONE);
		if (ret) {
			dev_err(aw->dev, "Cannot add irqchip\n");
			goto fail;
		}

		/* TODO IRQF_SHARED required?
		 * TODO How to use pre-defined IRQ_TYPE from device tree?
		 * docs: When requesting an interrupt without specifying a IRQF_TRIGGER,
         * the setting should be assumed to be "as already configured", which
         * may be as per machine or firmware initialisation.
		 */
		ret = devm_request_threaded_irq(
			aw->dev, client->irq, NULL, aw9523_irq_handler,
			IRQF_ONESHOT | IRQF_TRIGGER_LOW | IRQF_SHARED,
			devm_kasprintf(aw->dev, GFP_KERNEL, "aw9523b.%s",
				       dev_name(&client->dev)),
			aw);
		if (ret)
			goto fail;

		gpiochip_set_nested_irqchip(&aw->chip, &aw->irqchip,
					    client->irq);
		aw->irq_parent = client->irq;

		/* TODO wakeup from standby test. What is required?
		device_init_wakeup(aw->dev, 1);
		*/
	}

	return 0;

fail:
	devm_gpiochip_remove(aw->dev, &aw->chip);

fail_pre_gpiochip:
	mutex_destroy(&aw->lock);
	aw9523_led_free_resources(aw);

	/* TODO required or taken care of automatically? I.e. when client->dev is destroyed? */
	devm_kfree(aw->dev, aw);

	/* Note: failed probe() call needs to return a negative code! */
	return ret;
}

static int aw9523_remove(struct i2c_client *client)
{
	struct aw9523b *aw = i2c_get_clientdata(client);

	AW_DEBUG(&client->dev, "%s: enter\n", __func__);

	/* FIXME remove parent interrupt line configuration! But how?
	   modprobe doesn't work anymore after rmmod and loading the driver a 2nd time:
		irq: type mismatch, failed to map hwirq-28 for gpio@44e07000!
	*/
	if (aw->irq_parent > 0)
		devm_free_irq(&client->dev, aw->irq_parent, aw);

	/* This will trigger a callback from the led driver to brightness_set to switch off leds! */
	aw9523_led_free_resources(aw);

	AW_DEBUG(&client->dev, "%s: devm_gpiochip_remove...\n", __func__);
	devm_gpiochip_remove(&client->dev, &aw->chip);

	AW_DEBUG(&client->dev, "%s: mutex_destroy...\n", __func__);
	mutex_destroy(&aw->lock);

	AW_DEBUG(&client->dev, "%s: devm_kfree...\n", __func__);
	devm_kfree(&client->dev, aw);

	AW_DEBUG(&client->dev, "%s: Removed\n", __func__);

	return 0;
}

static struct i2c_driver aw9523_driver = {
	.driver = {
		.name = AW_DRV_NAME,
		.of_match_table = of_match_ptr(aw9523b_of_table),
	},
	.probe    = aw9523_probe,
	.remove	  = aw9523_remove,
	.id_table = aw9523b_id,
};

static int __init aw9523_init(void)
{
	return i2c_add_driver(&aw9523_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(aw9523_init);

static void __exit aw9523_exit(void)
{
	i2c_del_driver(&aw9523_driver);
}
module_exit(aw9523_exit);

MODULE_AUTHOR("Markus Zehnder <zehnder@live.com>");
MODULE_DESCRIPTION("GPIO expander and LED driver for AW9523B");
MODULE_LICENSE("GPL");
