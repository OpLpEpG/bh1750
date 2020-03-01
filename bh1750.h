/*
 * Copyright (c) 2019 Centaur Analytics, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_
#define ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_


struct bh1750_data {
	struct device *i2c;
	u16_t light;
};

struct bh1750_dev_config {
	u16_t i2c_addr;
};

#endif /*  ZEPHYR_DRIVERS_SENSOR_BH1750_BH1750_H_ */
