/*
 * Copyright (c) 2023 Sensorfy B.V.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad7156

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>

#include "ad7156.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(AD7156, CONFIG_SENSOR_LOG_LEVEL);

/*******************************************************************************
 * @brief Resets the device.
 *
 * @param dev - The device structure.
 *
 * @return None.
*******************************************************************************/
static void ad7156_reset(const struct device *dev)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;

	i2c_reg_write_byte_dt(&cfg->i2c, AD7156_RESET_CMD, 1);
}

/*******************************************************************************
 * @brief Sets the converter mode of operation.
 *
 * @param dev      - The device structure.
 * @param pwr_mode - Mode of operation option.
 *		    Example: AD7156_CONV_MODE_IDLE - Idle
 *                           AD7156_CONV_MODE_CONT_CONV  - Continuous conversion
 *                           AD7156_CONV_MODE_SINGLE_CONV - Single conversion
 *                           AD7156_CONV_MODE_PWR_DWN - Power-down
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_set_power_mode(const struct device *dev, enum ad7156_powermode pwr_mode)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	ret = i2c_reg_update_byte_dt(&cfg->i2c, AD7156_REG_CONFIG,
				  AD7156_CONFIG_MD_MASK, AD7156_CONFIG_MD(pwr_mode));

	return ret;
}

/*******************************************************************************
 * @brief Enables or disables conversion on the selected channel.
 *
 * @param dev         - The device structure.
 * @param channel     - Channel option.
 *                      Example: AD7156_CHANNEL1
 *                               AD7156_CHANNEL2
 * @param enable_conv - The state of channel activity.
 *                      Example: 0 - disable conversion on selected channel.
 *                               1 - enable conversion on selected channel.
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_channel_state(const struct device *dev, uint8_t channel, uint8_t enable_conv)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint8_t channel_mask, value_mask = 0;
	channel_mask = (channel & AD7156_CHANNEL1) ? AD7156_CONFIG_EN_CH1 : 0;
	channel_mask |= (channel & AD7156_CHANNEL2) ? AD7156_CONFIG_EN_CH2 : 0;
	value_mask = (enable_conv) ? channel_mask : 0;

	ret = i2c_reg_update_byte_dt(&cfg->i2c, AD7156_REG_CONFIG, channel_mask,
				  value_mask);

	return ret;
}

/*******************************************************************************
 * @brief Sets the CAPDAC offset of the specified channel.
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param range   - Input range option.
 *                  value between 0 and 0x3F (steps of 200fF)
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_set_capdac_range(const struct device *dev, uint8_t channel, uint8_t range)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint8_t reg_address = (channel == AD7156_CHANNEL1) ? AD7156_REG_CH1_CAPDAC : AD7156_REG_CH2_CAPDAC;
	ret = i2c_reg_update_byte_dt(&cfg->i2c, reg_address,
				  AD7156_CAPDAC_DAC_VAL_MASK, AD7156_CAPDAC_DAC_VAL(range));

	return ret;
}

/*******************************************************************************
 * @brief Gets the CAPDAC offset of the specified channel.
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param range   - Reference for the capacitive input offset. (steps of 200fF)
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_get_capdac_range(const struct device *dev, uint8_t channel, uint8_t *range)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint8_t reg_address = (channel == AD7156_CHANNEL1) ? AD7156_REG_CH1_CAPDAC : AD7156_REG_CH2_CAPDAC;
	ret = i2c_reg_read_byte_dt(&cfg->i2c, reg_address, range);

	*range = AD7156_CAPDAC_DAC_VAL(*range);

	return ret;
}

/*******************************************************************************
 * @brief Sets the input range of the specified channel.
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param range   - Input range option.
 *                  Example: AD7156_CDC_RANGE_2_PF   - 2pF input range.
 *                           AD7156_CDC_RANGE_0_5_PF - 0.5pF input range.
 *                           AD7156_CDC_RANGE_1_PF   - 1pF input range.
 *                           AD7156_CDC_RANGE_4_PF   - 4pF input range.
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_set_range(const struct device *dev, uint8_t channel, enum ad7156_range range)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint8_t reg_address = (channel == AD7156_CHANNEL1) ? AD7156_REG_CH1_SETUP : AD7156_REG_CH2_SETUP;
	ret = i2c_reg_update_byte_dt(&cfg->i2c, reg_address,
				  AD7156_SETUP_RANGE_MASK, AD7156_SETUP_RANGE(range));

	return ret;
}

/*******************************************************************************
 * @brief Reads the range bits from the device and returns the range in pF.
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param val     - Reference for the capacitive input range(pF).
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_get_range(const struct device *dev, uint32_t channel, float *val)
{
	if (val == NULL) {
		return -1;
	}
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;
	uint8_t setup_reg = 0;

	uint8_t reg_address = (channel == AD7156_CHANNEL1) ? AD7156_REG_CH1_SETUP : AD7156_REG_CH2_SETUP;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, reg_address, &setup_reg);

	if (ret) {
		return ret;
	}

	setup_reg = (setup_reg & AD7156_SETUP_RANGE_MASK) >> 6;
	switch (setup_reg) {
	case AD7156_CDC_RANGE_2_PF:
		*val = 2.0;
		break;
	case AD7156_CDC_RANGE_0_5_PF:
		*val = 0.5;
		break;
	case AD7156_CDC_RANGE_1_PF:
		*val = 1.0;
		break;
	case AD7156_CDC_RANGE_4_PF:
		*val = 4.0;
		break;
	}

	return ret;
}

/*******************************************************************************
 * @brief Selects the threshold mode of operation.
 *
 * @param dev       - The device structure.
 * @param thr_mode  - Output comparator mode.
 *                   Example: AD7156_THR_MODE_NEGATIVE
 *                            AD7156_THR_MODE_POSITIVE
 *                            AD7156_THR_MODE_IN_WINDOW
 *                            AD7156_THR_MODE_OUT_WINDOW
 * @param thr_fixed - Selects the threshold mode.
 *                   Example: AD7156_ADAPTIVE_THRESHOLD
 *                            AD7156_FIXED_THRESHOLD
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_set_threshold_mode(const struct device *dev, enum ad7156_thr_mode thr_mode,
				     enum ad7156_thr_fixed thr_fixed)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	ret = i2c_reg_update_byte_dt(
		&cfg->i2c, AD7156_REG_CONFIG, AD7156_CONFIG_THR_MASK,
		(AD7156_CONFIG_THR_FIXED * thr_fixed) | (AD7156_CONFIG_THR_MD(thr_mode)));

	return ret;
}

/*******************************************************************************
 * @brief Writes to the threshold register when threshold fixed mode is enabled.
 *
 * @param dev      - The device structure.
 * @param channel  - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param p_fthr   - The threshold value in picofarads(pF). The value must not be
 *                  out of the selected input range.
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_set_threshold(const struct device *dev, uint8_t channel, float p_fthr)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint8_t thr_reg_address =
		(channel == AD7156_CHANNEL1) ? AD7156_REG_CH1_SENS_THRSH : AD7156_REG_CH2_SENS_THRSH;
	uint16_t raw_thr = 0;
	float range = 0;

	ret = ad7156_get_range(dev, channel, &range);
	if (ret) {
		return ret;
	}

	raw_thr = (uint16_t)((p_fthr * 0xA000 / range) + 0x3000);
	if (raw_thr > 0xD000) {
		raw_thr = 0xD000;
	} else if (raw_thr < 0x3000) {
		raw_thr = 0x3000;
	}

	ret = i2c_burst_write_dt(&cfg->i2c, thr_reg_address, raw_thr, 2);

	return ret;
}

/*******************************************************************************
 * @brief Writes a value(pF) to the sensitivity register. This functions
 * should be used when adaptive threshold mode is selected.
 * In adaptive threshold mode, the output comparator threshold
 * is set as a defined distance (sensitivity) above the data average,
 *
 * @param dev            - The device structure.
 * @param channel        - Channel option.
 *                        Example: AD7156_CHANNEL1
 *                                 AD7156_CHANNEL2
 * @param p_fsensitivity - The sensitivity value in picofarads(pF).
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_set_sensitivity(const struct device *dev, uint8_t channel, float p_fsensitivity)
{
	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint8_t sensitivity_reg_addr = 0;
	uint16_t raw_sensitivity = 0;
	float range = 0;

	sensitivity_reg_addr =
		(channel == AD7156_CHANNEL1) ? AD7156_REG_CH1_SENS_THRSH : AD7156_REG_CH2_SENS_THRSH;
	ret = ad7156_get_range(dev, channel, &range);
	if (ret) {
		return ret;
	}

	raw_sensitivity = (uint16_t)(p_fsensitivity * 0xA00 / range);
	raw_sensitivity = (raw_sensitivity << 4) & 0x0FF0;

	ret = i2c_burst_write_dt(&cfg->i2c, sensitivity_reg_addr, raw_sensitivity, 2);

	return ret;
}

/*******************************************************************************
 * @brief Reads a 12-bit sample from the selected channel.
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param sample  - buffer with conversion result
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_read_channel_data(const struct device *dev, uint8_t channel, uint16_t *sample)
{
	if (sample == NULL) {
		return -1;
	}

	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint8_t reg_data[2] = { 0, 0 };
	uint8_t ch_address = (channel == AD7156_CHANNEL1) ? AD7156_REG_CH1_DATA_MSB : AD7156_REG_CH2_DATA_MSB;

	ret = i2c_burst_read_dt(&cfg->i2c, ch_address, reg_data, 2);
	if (ret) {
		return ret;
	}

	*sample = (reg_data[0] << 8) + reg_data[1];

	return ret;
}

/*******************************************************************************
 * @brief Waits for a finished CDC conversion and reads a 12-bit sample from
 *        the selected channel.
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param sample  - buffer with conversion result
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_wait_read_channel_data(const struct device *dev, uint8_t channel,
					 uint16_t *sample)
{
	if (sample == NULL) {
		return -1;
	}

	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint8_t reg_data[2] = { 0, 0 };
	uint8_t status = 0;
	uint8_t ch_rdy_mask = 0;
	uint8_t ch_address = 0;

	if (channel == AD7156_CHANNEL1) {
		ch_rdy_mask = AD7156_STATUS_RDY1;
		ch_address = AD7156_REG_CH1_DATA_MSB;
	} else {
		ch_rdy_mask = AD7156_STATUS_RDY2;
		ch_address = AD7156_REG_CH2_DATA_MSB;
	}

	do {
		ret = i2c_reg_read_byte_dt(&cfg->i2c, AD7156_REG_STATUS, &status);
		if (ret) {
			return ret;
		}
	} while ((status & ch_rdy_mask) != 0);

	ret = i2c_burst_read_dt(&cfg->i2c, ch_address, reg_data, 2);
	if (ret) {
		return ret;
	}
	*sample = (reg_data[0] << 8) + reg_data[1];

	return ret;
}

/*******************************************************************************
 * @brief Reads a sample the selected channel and converts the data to
 *        picofarads(pF).
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param result  - Conversion result form the selected channel as picofarads(pF).
 *
 * @return i2c result.
*******************************************************************************/
static int ad7156_read_channel_capacitance(const struct device *dev, uint8_t channel, float *result)
{
	if (result == NULL) {
		return -1;
	}

	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint16_t raw_ch = 0;
	float ch_range = 0;

	ret = ad7156_get_range(dev, channel, &ch_range);
	if (ret) {
		return ret;
	}

	ret = ad7156_read_channel_data(dev, channel, &raw_ch);
	if (ret) {
		return ret;
	}

	if (raw_ch < 0x3000) {
		raw_ch = 0x3000;
	} else if (raw_ch > 0xD000) {
		raw_ch = 0xD000;
	}
	*result = (((raw_ch)-0x3000) * ch_range) / 0xA000;

	// add offset from capdac
	uint8_t offset = 0;
	ad7156_get_capdac_range(dev, channel, &offset);
	*result = *result + (offset * 0.2f);

	return ret;
}

/*******************************************************************************
 * @brief Waits for a finished CDC conversion the selected channel, reads a
 *        sample and converts the data to picofarads(pF).
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *                  Example: AD7156_CHANNEL1
 *                           AD7156_CHANNEL2
 * @param result  - Conversion result form the selected channel as picofarads(pF).
 *
 * @return i2c result.
*******************************************************************************/
float ad7156_wait_read_channel_capacitance(const struct device *dev, uint8_t channel, float *result)
{
	if (result == NULL) {
		return -1;
	}

	struct ad7156_data *drv_data = dev->data;
	const struct ad7156_dev_config *cfg = dev->config;
	int ret = 0;

	uint16_t raw_ch = 0;
	float ch_range = 0;

	ret = ad7156_get_range(dev, channel, &ch_range);
	if (ret) {
		return ret;
	}

	ret = ad7156_wait_read_channel_data(dev, channel, &raw_ch);
	if (ret) {
		return ret;
	}

	if (raw_ch < 0x3000) {
		raw_ch = 0x3000;
	} else if (raw_ch > 0xD000) {
		raw_ch = 0xD000;
	}
	*result = (((raw_ch)-0x3000) * ch_range) / 0xA000;

	// add offset from capdac
	uint8_t offset = 0;
	ad7156_get_capdac_range(dev, channel, &offset);
	*result = *result + (offset * 0.2f);

	return ret;
}

/******************************************************************************/
static int ad7156_attr_get(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    struct sensor_value *val)
{
	int result;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_CAP_1 && chan != SENSOR_CHAN_CAP_2) {
		return -ENOTSUP;
	}

	switch ((int) attr) {
	default:
		return -ENOTSUP;
	}

	return result;
}

static int ad7156_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	const struct ad7156_dev_config *cfg = dev->config;
	uint8_t val8, reg = 0U;
	uint16_t rate;
	int64_t value;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_CAP_1 && chan != SENSOR_CHAN_CAP_2) {
		LOG_DBG("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		enum ad7156_range range_;
		switch (val->val1)
		{
		case 0:
			range_ = AD7156_CDC_RANGE_0_5_PF;
			break;
		case 1:
			range_ = AD7156_CDC_RANGE_1_PF;
			break;
		case 2:
			range_ = AD7156_CDC_RANGE_2_PF;
			break;
		case 4:
			range_ = AD7156_CDC_RANGE_4_PF;
			break;
		default:
			return -EINVAL;
		}
		return ad7156_set_range(dev, chan, range_);

	case SENSOR_ATTR_OFFSET:
		return ad7156_set_capdac_range(dev, chan, val->val1);

	case SENSOR_ATTR_CONFIGURATION:
		return ad7156_set_power_mode(dev, val->val1);
	case SENSOR_ATTR_AD7156_EN:
	case SENSOR_ATTR_AD7156_THR_MODE:
	case SENSOR_ATTR_AD7156_THR:
	case SENSOR_ATTR_AD7156_SENS:
	case SENSOR_ATTR_AD7156_CAPDAC_MODE:


	default:
		return -ENOTSUP;
	}

	return 0;
}

static int ad7156_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ad7156_data *drv_data = dev->data;
	int ret = 0;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_CAP_1 && chan != SENSOR_CHAN_CAP_2) {
		return -ENOTSUP;
	}

	switch (chan) {
	case SENSOR_CHAN_CAP_1:
		ret = ad7156_wait_read_channel_capacitance(dev, AD7156_CHANNEL1,
						      &drv_data->channel1.pf_data);
		break;

	case SENSOR_CHAN_CAP_2:
		ret = ad7156_wait_read_channel_capacitance(dev, AD7156_CHANNEL2,
						      &drv_data->channel2.pf_data);
		break;

	default:
		ret = ad7156_wait_read_channel_capacitance(dev, AD7156_CHANNEL1,
						      &drv_data->channel1.pf_data);
		if (ret) {
			return ret;
		}
		ret = ad7156_wait_read_channel_capacitance(dev, AD7156_CHANNEL2,
						      &drv_data->channel2.pf_data);
		break;
	}
	return ret;
}

static int ad7156_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct ad7156_data *drv_data = dev->data;
	int32_t value;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_CAP_1 && chan != SENSOR_CHAN_CAP_2) {
		return -ENOTSUP;
	}

	switch (chan) {
	case SENSOR_CHAN_CAP_1:
		return sensor_value_from_double(val, drv_data->channel1.pf_data);
	case SENSOR_CHAN_CAP_2:
		return sensor_value_from_double(val, drv_data->channel2.pf_data);
	default:
		break;
	}

	return 0;
}

static const struct sensor_driver_api ad7156_driver_api = {
	.attr_get = ad7156_attr_get,
	.attr_set = ad7156_attr_set,
	.sample_fetch = ad7156_sample_fetch,
	.channel_get = ad7156_channel_get,
#ifdef CONFIG_AD7156_TRIGGER
	.trigger_set = ad7156_trigger_set,
#endif
};


static int ad7156_probe(const struct device *dev)
{
	const struct ad7156_dev_config *cfg = dev->config;
	uint8_t value;
	int ret = 0;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, AD7156_REG_CHIP_ID, &value);
	if (ret) {
		return ret;
	}

	if (value != AD7156_DEFAULT_ID) {
		return -ENODEV;
	}

	ret = i2c_reg_update_byte_dt(&cfg->i2c, AD7156_REG_CONFIG,
				  AD7156_CONFIG_MD_MASK,
				  AD7156_CONFIG_MD(AD7156_CONV_MODE_CONT_CONV));
	if (ret) {
		return ret;
	}

	ret = ad7156_channel_state(dev, (AD7156_CHANNEL1 & AD7156_CHANNEL2), true);
	if (ret) {
		return ret;
	}

	ret = ad7156_set_range(dev, AD7156_CHANNEL1, AD7156_CDC_RANGE_4_PF);
	if (ret) {
		return ret;
	}

	ret = ad7156_set_range(dev, AD7156_CHANNEL2, AD7156_CDC_RANGE_4_PF);

#ifdef CONFIG_AD7156_TRIGGER
	if (cfg->int_gpio.port) {
		ret = ad7156_init_interrupt(dev);
		if (ret < 0) {
			LOG_ERR("Failed to initialize ad7156 interrupt!");
			return ret;
		}
	}
#endif

	return ret;
}

static int ad7156_init(const struct device *dev)
{
	const struct ad7156_dev_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -EINVAL;
	}
	return ad7156_probe(dev);
}

#define AD7156_DEFINE(inst)								\
	static struct ad7156_data ad7156_data_##inst;					\
	static const struct ad7156_dev_config ad7156_config_##inst = {		\
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
											\
	IF_ENABLED(CONFIG_AD7156_TRIGGER,						\
		   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),))	\
	};										\
											\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ad7156_init, NULL, &ad7156_data_##inst,	\
			      &ad7156_config_##inst, POST_KERNEL,			\
			      CONFIG_SENSOR_INIT_PRIORITY, &ad7156_driver_api);	\

DT_INST_FOREACH_STATUS_OKAY(AD7156_DEFINE)
