/* Copyright (c) 2015, Pixelworks, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "dsi_iris6_api.h"
#include "dsi_iris6_i3c.h"
#include "dsi_iris6_log.h"

#define REG_NUM  (12)
static uint32_t op_addr[REG_NUM] = {
0xf000007c, 0xf000003c, 0xf000003c, 0xf000003c, 0xf000003c,
0xf11d1100, 0xf11400a0, 0xf11400a4, 0xf11400a8, 0xf11400ac,
0xf1140000, 0xf114100c
};

static uint32_t op_val[REG_NUM] = {
0x00000000, 0x00000003, 0x00000007, 0x0000000f, 0x0000001F,
0x00000002, 0x00000003, 0x40320cad, 0x2fba33c7, 0x36d6fcd5,
0x0280914c, 0x0001011f
};

static uint32_t op_delay[REG_NUM] = {
0, 1, 1, 1, 0, 0,
0, 0, 0, 0, 0, 0
};

static uint32_t checksum[3] = {0x40320cad, 0x2fba33c7, 0x36d6fcd5};

static int _iris_loopback_config(void)
{
	int ret = 0;
	int i;

	for (i = 0; i < REG_NUM; i++) {
		ret = iris_ioctl_i2c_write(op_addr[i], op_val[i]);
		if (ret) {
			IRIS_LOGE("%s(), error, i = %d, ret = %d.", __func__, i, ret);
			return ret;
		}

		if (op_delay[i])
			mdelay(op_delay[i]);
	}

	return ret;
}

int iris_loopback_validate(void)
{
	int ret = 0;
	int i = 0;
	uint32_t statis[3] = {0};
	uint32_t bist_ctrl = 0;

	IRIS_LOGI("%s(%d), start.", __func__, __LINE__);

	iris_reset();
	mdelay(100);

	if (iris_platform_get() == 0) {
		ret = iris_ioctl_i2c_write(0xF0000100, 0x0000EE00);
		if (ret) {
			IRIS_LOGE("%s(%d), i3c filter set fail.", __func__, __LINE__);
			return 1;
		}
	}

	ret = _iris_loopback_config();
	if (ret) {
		IRIS_LOGE("%s(%d), i2c configure loopback fail.", __func__, __LINE__);
		return 1;
	}

	mdelay(100);
	for (i = 0; i < 3; i++) {
		iris_ioctl_i2c_read(0xf11400d8 + i*4, &statis[i]);
		if (ret) {
			IRIS_LOGE("%s(%d), i2c read statis fail.", __func__, __LINE__);
			return 2;
		}
	}

	IRIS_LOGI("%s(%d), statis = 0x%x 0x%x 0x%x.", __func__, __LINE__,
			statis[0], statis[1], statis[2]);

	ret = iris_ioctl_i2c_read(0xf114100c, &bist_ctrl);
	if (ret) {
		IRIS_LOGE("%s(%d), i2c read bist ctrl fail.", __func__, __LINE__);
		return 2;
	}

	IRIS_LOGI("%s(%d), bist_ctrl = 0x%x.", __func__, __LINE__, bist_ctrl);

	if ((statis[0] == checksum[0]) && (statis[1] == checksum[1]) && (statis[2] == checksum[2])) {
		if ((bist_ctrl & 0x10100) == 0x100) {
			ret = 0;
			IRIS_LOGI("%s(%d), loopback validate success.", __func__, __LINE__);
		} else {
			ret = 4;
			IRIS_LOGE("%s(%d), bist ctrl status error.", __func__, __LINE__);
		}
	} else {
		ret = 3;
		IRIS_LOGE("%s(%d), statis not equal to checksum.", __func__, __LINE__);
	}

	IRIS_LOGI("%s(%d), end.", __func__, __LINE__);

	return ret;

}
