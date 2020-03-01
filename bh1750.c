
#include <device.h>
#include <drivers/i2c.h>
#include "stm32f1xx_ll_i2c.h"
#include <drivers/sensor.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <kernel.h>

#include "bh1750.h"


#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(BH1750);

#define DT_INST_0_TI_BH1750_BUS_NAME "I2C_1"
#define DT_INST_0_TI_BH1750_LABEL "BH1750"

#define UNCONFIGURED 0
      // Measurement at 1 lux resolution. Measurement time is approx 120ms.
#define CONTINUOUS_HIGH_RES_MODE 0x10


static int BH1750_reg_read(struct device *dev, u8_t mode, u16_t *val)
{
	struct bh1750_data *drv_data = dev->driver_data;
	const struct bh1750_dev_config *cfg = dev->config->config_info;
    
	u8_t res = 0;
	u8_t b[1], read[2];
	b[0] = mode; 
    if (i2c_write(drv_data->i2c, b, 1, cfg->i2c_addr) < 0) 
    {
		res = -EIO;
    }
    if (i2c_read(drv_data->i2c, read, 2, cfg->i2c_addr) < 0) 
    {
		res = -EIO;
    }
    else LOG_HEXDUMP_DBG(read, 2, "[AM] bh1750");

    *val = sys_be16_to_cpu(*(u16_t*) &read[0]);
	return res;
}

static int BH1750_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct bh1750_data *drv_data = dev->driver_data;
	u16_t value;
	int rc;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL ||
			chan == SENSOR_CHAN_LIGHT);

	/* clear sensor values */
	drv_data->light = 0;

	/* Get the most recent temperature measurement */
	rc = BH1750_reg_read(dev, CONTINUOUS_HIGH_RES_MODE, &value);
	if (rc < 0) {
		LOG_ERR("%s: Failed to read from TEMP register!",
			DT_INST_0_TI_BH1750_LABEL);
		return rc;
	}
	/* store measurements to the driver */
	drv_data->light = value;
	
	return 0;
}

//const float BH1750_CONV_FACTOR = 1.2 = *12 / 10;

static int BH1750_channel_get(struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bh1750_data *drv_data = dev->driver_data;
	s32_t tmp;

	if (chan == SENSOR_CHAN_LIGHT)	
	{
		tmp = (s16_t)drv_data->light * 12;
		val->val1 = tmp / 10; /* lux */
		val->val2 = tmp % 10;
		return 0;
	}
	else return -ENOTSUP; 
}

static const struct sensor_driver_api BH1750_driver_api = {
	.sample_fetch = BH1750_sample_fetch,
	.channel_get = BH1750_channel_get
};

#define __HAL_RCC_I2C1_FORCE_RESET()       (RCC->APB1RSTR |= (RCC_APB1RSTR_I2C1RST))
#define __HAL_RCC_I2C1_RELEASE_RESET()       (RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST))

static int BH1750_init(struct device *dev)
{
	struct bh1750_data *drv_data = dev->driver_data;

	/* Bind to the I2C bus that the sensor is connected */
	drv_data->i2c = device_get_binding(DT_INST_0_TI_BH1750_BUS_NAME);
	if (!drv_data->i2c) {
		LOG_ERR("Cannot bind to %s device!", DT_INST_0_TI_BH1750_BUS_NAME);
		return -EINVAL;
	}
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    while (LL_I2C_IsActiveFlag_BUSY(I2C1)) {
        LL_I2C_Disable(I2C1);
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        LL_I2C_Enable(I2C1);
        LOG_ERR("LL_I2C_IsActiveFlag_BUSY");
        k_sleep(1000);
	}
    if (i2c_configure(drv_data->i2c, I2C_SPEED_FAST | I2C_MODE_MASTER) < 0)
    {
        LOG_ERR("i2c_configure");
    }

	/* Check the Device ID */
	// rc = BH1750_device_id_check(dev);
	// if (rc < 0) {
		// return rc;
	// }

	return 0;
}

static struct bh1750_data BH1750_data;

static const struct bh1750_dev_config BH1750_config = {
	.i2c_addr = 0x23,
};

DEVICE_AND_API_INIT(bh1750, DT_INST_0_TI_BH1750_LABEL, BH1750_init,
		    &BH1750_data, &BH1750_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &BH1750_driver_api);
