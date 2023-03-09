/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MCP98XX_MCP98XX_H
#define ZEPHYR_DRIVERS_SENSOR_MCP98XX_MCP98XX_H

#ifndef ZEPHYR_DRIVERS_SENSOR_MCP98XX_MCP98XX_H_
#define ZEPHYR_DRIVERS_SENSOR_MCP98XX_MCP98XX_H_

#include <errno.h>

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define MCP98XX_REG_CONFIG		0x01
#define MCP98XX_REG_UPPER_LIMIT		0x02
#define MCP98XX_REG_LOWER_LIMIT		0x03
#define MCP98XX_REG_CRITICAL		0x04
#define MCP98XX_REG_TEMP_AMB		0x05

/* 16 bits control configuration and state.
 *
 * * Bit 0 controls alert signal output mode
 * * Bit 1 controls interrupt polarity
 * * Bit 2 disables upper and lower threshold checking
 * * Bit 3 enables alert signal output
 * * Bit 4 records alert status
 * * Bit 5 records interrupt status
 * * Bit 6 locks the upper/lower window registers
 * * Bit 7 locks the critical register
 * * Bit 8 enters shutdown mode
 * * Bits 9-10 control threshold hysteresis
 */
#define MCP98XX_CFG_ALERT_MODE_INT	BIT(0)
#define MCP98XX_CFG_ALERT_ENA		BIT(3)
#define MCP98XX_CFG_ALERT_STATE		BIT(4)
#define MCP98XX_CFG_INT_CLEAR		BIT(5)

/* 16 bits are used for temperature and state encoding:
 * * Bits 0..11 encode the temperature in a 2s complement signed value
 *   in Celsius with 1/16 Cel resolution
 * * Bit 12 is set to indicate a negative temperature
 * * Bit 13 is set to indicate a temperature below the lower threshold
 * * Bit 14 is set to indicate a temperature above the upper threshold
 * * Bit 15 is set to indicate a temperature above the critical threshold
 */
#define MCP98XX_TEMP_SCALE_CEL		16 /* signed */
#define MCP98XX_TEMP_SIGN_BIT		BIT(12)
#define MCP98XX_TEMP_ABS_MASK		((uint16_t)(MCP98XX_TEMP_SIGN_BIT - 1U))
#define MCP98XX_TEMP_LWR_BIT		BIT(13)
#define MCP98XX_TEMP_UPR_BIT		BIT(14)
#define MCP98XX_TEMP_CRT_BIT		BIT(15)

#define MCP98XX_REG_CONFIG		0x01
#define MCP98XX_REG_CONFIG_SHDN		(1<<8)

#if CONFIG_MCP98XX_CHIP_MCP9808
#define MCP98XX_REG_RESOLUTION      0x08
#elif CONFIG_MCP98XX_CHIP_MCP9844
#define MCP98XX_REG_RESOLUTION      0x09
#else
#error No MCP98xx chiptype selected.
#endif

struct mcp98xx_data {
	uint16_t reg_val;

#ifdef CONFIG_MCP98XX_TRIGGER
	struct gpio_callback alert_cb;

	const struct device *dev;

	struct sensor_trigger trig;
	sensor_trigger_handler_t trigger_handler;
#endif

#ifdef CONFIG_MCP98XX_TRIGGER_OWN_THREAD
	struct k_sem sem;
#endif

#ifdef CONFIG_MCP98XX_TRIGGER_GLOBAL_THREAD
	struct k_work work;
#endif
};

struct mcp98xx_config {
	struct i2c_dt_spec i2c;
	uint8_t resolution;
#ifdef CONFIG_MCP98XX_TRIGGER
	struct gpio_dt_spec int_gpio;
#endif /* CONFIG_MCP98XX_TRIGGER */
};

int mcp98xx_reg_read(const struct device *dev, uint8_t reg, uint16_t *val);
int mcp98xx_reg_write_16bit(const struct device *dev, uint8_t reg,
			    uint16_t val);
int mcp98xx_reg_write_8bit(const struct device *dev, uint8_t reg,
			   uint8_t val);

#ifdef CONFIG_MCP98XX_TRIGGER
int mcp98xx_attr_set(const struct device *dev, enum sensor_channel chan,
		     enum sensor_attribute attr,
		     const struct sensor_value *val);
int mcp98xx_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);
int mcp98xx_setup_interrupt(const struct device *dev);
#endif /* CONFIG_MCP98XX_TRIGGER */

/* Encode a signed temperature in scaled Celsius to the format used in
 * register values.
 */
static inline uint16_t mcp98xx_temp_reg_from_signed(int temp)
{
	/* Get the 12-bit 2s complement value */
	uint16_t rv = temp & MCP98XX_TEMP_ABS_MASK;

	if (temp < 0) {
		rv |= MCP98XX_TEMP_SIGN_BIT;
	}
	return rv;
}

/* Decode a register temperature value to a signed temperature in
 * scaled Celsius.
 */
static inline int mcp98xx_temp_signed_from_reg(uint16_t reg)
{
	int rv = reg & MCP98XX_TEMP_ABS_MASK;

	if (reg & MCP98XX_TEMP_SIGN_BIT) {
		/* Convert 12-bit 2s complement to signed negative
		 * value.
		 */
		rv = -(1U + (rv ^ MCP98XX_TEMP_ABS_MASK));
	}
	return rv;
}

#endif /* ZEPHYR_DRIVERS_SENSOR_MCP98XX_MCP98XX_H_ */


#endif // ZEPHYR_DRIVERS_SENSOR_MCP98XX_MCP98XX_H
