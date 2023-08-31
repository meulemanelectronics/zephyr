/*
 * Copyright (c) 2023 Sensorfy B.V.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AD7156_AD7156_H_
#define ZEPHYR_DRIVERS_SENSOR_AD7156_AD7156_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

/* AD7156 registers */
#define AD7156_REG_STATUS 		0x00 /* Status */
#define AD7156_REG_CH1_DATA_MSB 		0x01 /* Ch 1 Data MSB */
#define AD7156_REG_CH1_DATA_LSB 		0x02 /* Ch 1 Data LSB */
#define AD7156_REG_CH2_DATA_MSB 		0x03 /* Ch 2 Data MSB */
#define AD7156_REG_CH2_DATA_LSB 		0x04 /* Ch 2 Data LSB */
#define AD7156_REG_CH1_AVG_MSB 		0x05 /* Ch 1 Average MSB */
#define AD7156_REG_CH1_AVG_LSB 		0x06 /* Ch 1 Average LSB */
#define AD7156_REG_CH2_AVG_MSB 		0x07 /* Ch 2 Average MSB */
#define AD7156_REG_CH2_AVG_LSB 		0x08 /* Ch 2 Average LSB */
#define AD7156_REG_CH1_SENS_THRSH 	0x09 /* Ch 1 Sensitivity/ Ch1 Threshold MSB */
#define AD7156_REG_CH1_TMO_THRSH 	0x0A /* Ch 1 Timeout/ Ch 1 Threshold LSB */
#define AD7156_REG_CH1_SETUP 		0x0B /* Ch 1 Setup  */
#define AD7156_REG_CH2_SENS_THRSH 	0x0C /* Ch 2 Sensitivity/ Ch2 Threshold MSB */
#define AD7156_REG_CH2_TMO_THRSH 	0x0D /* Ch 2 Timeout/ Ch 2 Threshold LSB */
#define AD7156_REG_CH2_SETUP 		0x0E /* Ch 2 Setup  */
#define AD7156_REG_CONFIG 		0x0F /* Configuration */
#define AD7156_REG_PWR_DWN_TMR 		0x10 /* Power-Down Timer */
#define AD7156_REG_CH1_CAPDAC 		0x11 /* Ch 1 CAPDAC */
#define AD7156_REG_CH2_CAPDAC 		0x12 /* Ch 2 CAPDAC */
#define AD7156_REG_SERIAL_N3 		0x13 /* Serial Number 3 */
#define AD7156_REG_SERIAL_N2 		0x14 /* Serial Number 2 */
#define AD7156_REG_SERIAL_N1 		0x15 /* Serial Number 1 */
#define AD7156_REG_SERIAL_N0 		0x16 /* Serial Number 0  */
#define AD7156_REG_CHIP_ID 		0x17 /* Chip ID */

/* AD7156_REG_STATUS definition */
#define AD7156_STATUS_PWR_DWN		BIT(7)
#define AD7156_STATUS_DAC_STEP2		BIT(6)
#define AD7156_STATUS_OUT2			BIT(5)
#define AD7156_STATUS_DAC_STEP1		BIT(4)
#define AD7156_STATUS_OUT1			BIT(3)
#define AD7156_STATUS_C1_C2 		BIT(2)
#define AD7156_STATUS_RDY2 			BIT(1)
#define AD7156_STATUS_RDY1 			BIT(0)

/* AD7156_REG_SETUP definition */
#define AD7156_SETUP_RANGE_MASK 	((0x3) << 6)
#define AD7156_SETUP_RANGE(x) 		(((x)&0x3) << 6)
#define AD7156_SETUP_HYST2			BIT(4)
#define AD7156_SETUP_THR2(x) 		((x)&0xF)

/* AD7156_REG_CONFIG definition */
#define AD7156_CONFIG_THR_MASK 		(7u << 5)
#define AD7156_CONFIG_THR_FIXED 	BIT(7)
#define AD7156_CONFIG_THR_MD(x) 	(((x)&0x3) << 5)
#define AD7156_CONFIG_EN_CH1 		BIT(4)
#define AD7156_CONFIG_EN_CH2 		BIT(3)
#define AD7156_CONFIG_MD_MASK 		0x7
#define AD7156_CONFIG_MD(x) 		((x)&AD7156_CONFIG_MD_MASK)

/* AD7156_REG_PWR_DWN_TMR definition */
#define AD7156_PWR_DWN_TMR_TIMEOUT(x) 	(((x)&0x3F) | (1 << 6))

/* AD7156_REG_CAPDAC definition */
#define AD7156_CAPDAC_DAC_EN 		(1 << 7)
#define AD7156_CAPDAC_DAC_AUTO 		(1 << 6)
#define AD7156_CAPDAC_DAC_VAL_MASK 	(0x3F)
#define AD7156_CAPDAC_DAC_VAL(x) 	((x)&AD7156_CAPDAC_DAC_VAL_MASK)

/* AD7156 default chip ID */
#define AD7156_DEFAULT_ID 		0x88

/* AD7156 reset command */
#define AD7156_RESET_CMD 		0xBF

/* AD7156 channels*/
#define AD7156_CHANNEL1 		1
#define AD7156_CHANNEL2 		2

enum sensor_attribute_ad7156 {
	/* Channel state/enable */
	SENSOR_ATTR_AD7156_EN = SENSOR_ATTR_PRIV_START,
	/* Threshold mode */
	SENSOR_ATTR_AD7156_THR_MODE,
	/* Threshold (only applicable for fixed mode) */
	SENSOR_ATTR_AD7156_THR,
	/* Sensitivity (only applicable for adaptive mode) */
	SENSOR_ATTR_AD7156_SENS,
	/* CAPDAC mode */
	SENSOR_ATTR_AD7156_CAPDAC_MODE,
};

enum ad7156_powermode {
	AD7156_CONV_MODE_IDLE = 0,
	AD7156_CONV_MODE_CONT_CONV = 1,
	AD7156_CONV_MODE_SINGLE_CONV = 2,
	AD7156_CONV_MODE_PWR_DWN = 3,
};

enum ad7156_range {
	AD7156_CDC_RANGE_2_PF = 0, // 2pF input range.
	AD7156_CDC_RANGE_0_5_PF = 1, // 0.5pF input range.
	AD7156_CDC_RANGE_1_PF = 2, // 1pF input range.
	AD7156_CDC_RANGE_4_PF = 3, // 4pF input range.
};

enum ad7156_thr_mode {
	AD7156_THR_MODE_NEGATIVE = 0,
	AD7156_THR_MODE_POSITIVE = 1,
	AD7156_THR_MODE_IN_WINDOW = 2,
	AD7156_THR_MODE_OUT_WINDOW = 3,
};

enum ad7156_thr_fixed {
	AD7156_ADAPTIVE_THRESHOLD = 0,
	AD7156_FIXED_THRESHOLD = 1,
};

struct ad7156_ch_config {
	bool state;
	enum ad7156_range range;
	enum ad7156_thr_mode threshold_mode;
	enum ad7156_thr_fixed threshold_fixed;
	float sensitivity;
#ifdef CONFIG_AD7156_TRIGGER
	gpio_pin_t int_pin;
	gpio_flags_t int_flags;
	const char *int_name;
#endif
};

struct ad7156_ch_data {
	uint16_t raw_data;
	float pf_data;
};


struct ad7156_data {
	struct ad7156_ch_data channel1;
	struct ad7156_ch_data channel2;
#ifdef CONFIG_AD7156_TRIGGER
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t th_handler;
	struct sensor_trigger th_trigger;

	const struct device *dev;

#if defined(CONFIG_AD7156_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_AD7156_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_AD7156_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_AD7156_TRIGGER */
};

struct ad7156_dev_config {
	struct i2c_dt_spec i2c;
	struct ad7156_ch_config ch1;
	struct ad7156_ch_config ch2;
};


#ifdef CONFIG_AD7156_TRIGGER
int ad7156_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int ad7156_init_interrupt(const struct device *dev);
#endif /* CONFIG_AD7156_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_AD7156_AD7156_H_ */
