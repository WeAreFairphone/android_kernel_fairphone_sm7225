/* Copyright (C) 2021 Tcl Corporation Limited */
/*
################################################################################
# se47xx family of image sensors
# Copyright Zebra Technologies 2020
#
# * This program is free software; you can redistribute it and/or modify
# * it under the terms of the GNU General Public License version 2 and
# * only version 2 as published by the Free Software Foundation.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
#
################################################################################
*/

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <linux/i2c-dev.h>
#include <cam_sensor_cmn_header.h>
#include "cam_sensor_core.h"
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"
#include "cam_sensor_soc.h"


#define SCANNER_SENSOR_NAME "se47xx"

//#define SCANNER_KLOGD

#define TAG SCANNER_SENSOR_NAME
#define I2C_INTERVAL (10)

#define SE47XX_SIZE_WVGA				0
#define SE47XX_I2C_DEVICE_ADDR			0x5c
#define SE47XX_IMG_RES					0x5c
#define SE47XX_IMG_RES_RB2				0x01
#define SE47XX_IMG_RES_RB4				0x02
#define SE47XX_IMG_RES_CB2				0x03
#define SE47XX_IMG_RES_CB4				0x04
#define SE47XX_ACQ					    0x58
#define SE47XX_ACQ_ON					0x01
#define SE47XX_ACQ_OFF					0x00
#define SE47XX_AIM					    0x55
#define SE47XX_AIM_ON					0x01
#define SE47XX_AIM_OFF					0x00
#define SE47XX_AIM_DURING_EXPOSURE  	0x56
#define SE47XX_ILLUM					0x59
#define SE47XX_ILLUM_ON					0x01
#define SE47XX_ILLUM_OFF				0x00
#define SE47XX_AUTO_POWER				0x74
#define SE47XX_AUTO_POWER_EN			0x01
#define SE47XX_TIME_TO_LOW_POWER		0x75
#define SE47XX_PICKLIST_MODE			0x7b //Scanning Specific Barcode.
#define SE47XX_LCD_MODE					0x82 //Scanning Bar code On LCD.
#define SE47XX_FRAME_RATE				0x5E // FPS
#define SE47XX_PING					    0x7A
#define SE47XX_IMG_CAP_MODE				0x73
#define SE47XX_IMG_CAP_MODE_ON			0x01
#define SE47XX_IMG_CAP_MODE_OFF			0x00

#define SE47XX_GET_PARAM                0x70

//BlockBuster return status values
#define SE47XX_RETURN_ACK                   0x80 //Bit 7 Command Processed
#define SE47XX_RETURN_SP1                   0x01
#define SE47XX_RETURN_NAK                   0x82
#define SE47XX_RETURN_CKSM_ERROR            0x84
#define SE47XX_RETURN_AIM_POWER_FAILURE     0x88
#define SE47XX_RETURN_THERMAL_FAILURE       0x90
#define SE47XX_RETURN_INTERNAL_I2C_FAILURE  0x20
#define SE47XX_RETURN_SP6                   0x40
#define SE47XX_RETURN_UNUSUAL_ACK           0xA2 //unusual ack

//#define USE_I2C_DELAY

static int is_opened_by_SDL = 0;	// A flag to indicate if scan engine is opened by SDL.
struct file_operations sdl_control_fops;

static struct cam_sensor_ctrl_t* se47xx_misc_wa = NULL;
extern int32_t cam_handle_mem_ptr(uint64_t handle, struct cam_sensor_ctrl_t *s_ctrl);
extern void cam_sensor_query_cap(struct cam_sensor_ctrl_t *s_ctrl,
	struct  cam_sensor_query_cap *query_cap);

#define BUF_SIZE 20
static char g_camstatus[BUF_SIZE] = "none";
static struct mutex g_camstatus_mutex;

static int scan_exist_status = 0;
static int g_is_scan_mode = 0;
void set_scan_mode (int value)
{
	g_is_scan_mode = (value) ? 1 : 0;
}
EXPORT_SYMBOL(set_scan_mode);

static int is_scan_mode (void)
{
	return g_is_scan_mode;
}
EXPORT_SYMBOL(is_scan_mode);

#ifdef USE_I2C_DELAY
static unsigned get_tick_count (void)
{
	struct timeval gettick;
	do_gettimeofday(&gettick);
	return gettick.tv_sec * 1000 + gettick.tv_usec / 1000;
}
#endif

int32_t cam_se47xx_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg);

static long cam_sensor_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl =
		v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_se47xx_driver_cmd(s_ctrl, arg);
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid ioctl cmd: %d", cmd);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int cam_sensor_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_sensor_ctrl_t *s_ctrl =
		v4l2_get_subdevdata(sd);

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "s_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	cam_sensor_shutdown(s_ctrl);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return 0;
}

static struct v4l2_subdev_core_ops cam_sensor_subdev_core_ops = {
	.ioctl = cam_sensor_subdev_ioctl,
	.s_power = cam_sensor_power,
};

static struct v4l2_subdev_ops cam_sensor_subdev_ops = {
	.core = &cam_sensor_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_sensor_internal_ops = {
	.close = cam_sensor_subdev_close,
};

static int cam_sensor_init_subdev_params(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	s_ctrl->v4l2_dev_str.internal_ops =
		&cam_sensor_internal_ops;
	s_ctrl->v4l2_dev_str.ops =
		&cam_sensor_subdev_ops;
	strlcpy(s_ctrl->device_name, SCANNER_SENSOR_NAME,
		sizeof(s_ctrl->device_name));
	s_ctrl->v4l2_dev_str.name =
		s_ctrl->device_name;
	s_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	s_ctrl->v4l2_dev_str.ent_function =
		CAM_SENSOR_DEVICE_TYPE;
	s_ctrl->v4l2_dev_str.token = s_ctrl;

	rc = cam_register_subdev(&(s_ctrl->v4l2_dev_str));
	if (rc) {
		CAM_ERR(CAM_SENSOR,
			"%s :cam_register_subdev failed, rc(%d)", s_ctrl->device_name, rc);
    }

	return rc;
}

static struct miscdevice sdl_control_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sdl_control",
	.fops = &sdl_control_fops,
};

static uint8_t se47xx_calc_checksum (uint8_t *data, uint16_t count)
{
	uint16_t sum = 0;
	uint16_t i;

	for (i = 0; i < count; i++)
		sum += data[i];

	sum = ~sum + 1;

	return (sum & 0xff);
}

static int se47xx_send_command (struct i2c_client *client, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;

	if (!client) {
		pr_err("Invalid argument! (client=%p)", client);
		return -1;
	}

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	return i2c_transfer(client->adapter, &msg, 1);
}

static int se47xx_get_response (struct i2c_client *client, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;

	if (!client) {
		pr_err("Invalid argument! (client=%p)", client);
		return -1;
	}

	msg.addr = client->addr;
	msg.flags = I2C_M_RD;
	msg.len = len;
	msg.buf = data;

	return i2c_transfer(client->adapter, &msg, 1);
}

static int se47xx_send_command0 (struct i2c_client *client, uint8_t command, uint8_t *response)
{
	int rc;
	uint8_t data[2];

	data[0] = command;
	data[1] = se47xx_calc_checksum(data, 1);

	rc = se47xx_send_command(client, data, 2);
	if (rc < 0) {
		mdelay(100);
		rc = se47xx_send_command(client, data, 2);
		if (rc < 0) {
			pr_err("se47xx_send_command() failed!");
			return rc;
		}
	}
	rc = se47xx_get_response(client, data, 2);
	if (rc < 0) {
		mdelay(100);
		rc = se47xx_get_response(client, data, 2);
		if (rc < 0) {
			pr_err("se47xx_get_response() failed!");
			return rc;
		}
	}
	if (command != data[0]) {
		pr_err("command and response mismatch!");
		return -1;
	}
	if (response != NULL) {
		*response = data[1];
	}

	return 0;
}
//it is not used during decoding
#if 0
static int se47xx_send_command1 (struct i2c_client *client, uint8_t command, uint8_t value, uint8_t *response)
{
	int rc;
	uint8_t data[3];

	data[0] = command;
	data[1] = value;
	data[2] = se47xx_calc_checksum(data, 2);
	rc = se47xx_send_command(client, data, 3);
	if (rc < 0) {
		mdelay(100);
		rc = se47xx_send_command(client, data, 3);
		if (rc < 0) {
			pr_err("se47xx_send_command() failed!");
			return rc;
		}
	}
	rc = se47xx_get_response(client, data, 2);
	if (rc < 0) {
		mdelay(100);
		rc = se47xx_get_response(client, data, 2);
		if (rc < 0) {
			pr_err("se47xx_get_response() failed!");
			return rc;
		}
	}

	if (command != data[0]) {
		pr_err("command and response mismatch!");
		return -1;
	}
	if (response != NULL) {
		*response = data[1];
	}

	return 0;
}
#endif

static int cam_se47xx_match_id(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint8_t response = 0;
	struct cam_camera_slave_info *slave_info;

	slave_info = &(s_ctrl->sensordata->slave_info);

	if (!slave_info) {
		CAM_ERR(CAM_SENSOR, " failed: %pK",
			 slave_info);
		return -EINVAL;
	}

	rc = se47xx_send_command0(s_ctrl->io_master_info.client, SE47XX_PING, &response);
	if (rc < 0) {
		pr_err("se47xx_send_command0(SE47XX_PING) failed!");
		return rc;
	}
	if (SE47XX_RETURN_ACK == response) {
		scan_exist_status = 1;
        se47xx_misc_wa = s_ctrl;
        dev_set_drvdata(sdl_control_device.this_device, s_ctrl->io_master_info.client);
		pr_err("se47xx match probe success\n");
		return 0;
	} else {
		scan_exist_status = 0;
		pr_err("se47xx match id failed!");
		return -ENODEV;
	}
}

static int32_t se4750_mipi_i2c_rxdata(struct  i2c_client *client,
	unsigned char *rxdata, int data_length)
{
	int32_t rc = 0;
	uint16_t saddr = client->addr;
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = data_length,
			.buf   = rxdata,
		},
	};
	rc = i2c_transfer(client->adapter, msgs, 1);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata failed 0x%x", saddr);
	return rc;
}

static int32_t se4750_mipi_i2c_txdata(struct i2c_client *client,
				unsigned char *txdata, int length)
{
	int32_t rc = 0;
	uint16_t saddr =  client->addr;
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	rc = i2c_transfer(client->adapter, msg, 1);
	if (rc < 0)
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata faild 0x%x rc=%d", saddr,rc);
	return rc;
}

static int se47xx_streamOn(struct  i2c_client *client){
	int rc = 0;
	int i = 0;
	unsigned char streamOn_enableHW[3]={0x84,0x01,0x7b};
	unsigned char streamOn_enableMIPI[3]={0x86,0x03,0x77};
	unsigned char streamOn_enableAimOn[3]={0x55,0x01,0xaa};
	unsigned char streamOn_enableIllumOn[3]={0x59,0x01,0xa6};
	unsigned char streamOn_enableAcqOn[3]={0x58,0x01,0xa7};
	unsigned char respbuf[2] = {0};

	rc = se4750_mipi_i2c_txdata(client,streamOn_enableHW,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableHW failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableHW[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableHW success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

	rc = se4750_mipi_i2c_txdata(client,streamOn_enableMIPI,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableMIPI failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableMIPI[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableMIPI success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

	rc = se4750_mipi_i2c_txdata(client,streamOn_enableAimOn,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableAimOn failed rc=%d",rc);
		goto ERR_EXIT;
	}
	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableAimOn[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableAimOn success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

	rc = se4750_mipi_i2c_txdata(client,streamOn_enableIllumOn,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableIllumOn failed rc=%d",rc);
		goto ERR_EXIT;
	}
	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableIllumOn[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableIllumOn success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

	rc = se4750_mipi_i2c_txdata(client,streamOn_enableAcqOn,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableAcqOn failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableAcqOn[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableAcqOn success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

ERR_EXIT:
	return rc;
}

static int se47xx_streamOff(struct  i2c_client *client){
	int rc = 0;
	int i = 0;
	int retry = 10;
//	unsigned char streamOff_AimOff[3]={0x55,0x00,0xab};
//	unsigned char streamOff_IllumOff[3]={0x59,0x00,0xa7};
	unsigned char streamOff_AcqOff[3]={0x58,0x00,0xa8};
	unsigned char respbuf[2] = {0};
#if 0
	rc = se4750_mipi_i2c_txdata(client,streamOff_AimOff,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOff_AimOff failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOff_AimOff[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOff_AimOff success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

		rc = se4750_mipi_i2c_txdata(client,streamOff_IllumOff,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOff_IllumOff failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOff_IllumOff[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOff_IllumOff success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

#endif
	for(i=0; i < retry ; i++){
		rc = se4750_mipi_i2c_txdata(client,streamOff_AcqOff,3);
		if(rc > 0)
		{
			break;
		}
		else
		{
			msleep(1);
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOff_AcqOff failed rc=%d retry=%d", rc, i);
		}
	}
	if (i >= retry)
	{
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOff_AcqOff failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0; i < retry ; i++){

		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOff_AcqOff[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOff_AcqOff success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

ERR_EXIT:
	return rc;
}

static int streamOn_cmds(struct  i2c_client *client){
	int rc = 0;
	int i = 0;
	unsigned char streamOn_enableHW[3]={0x84,0x01,0x7b};
	unsigned char streamOn_enableMIPI[3]={0x86,0x03,0x77};
	unsigned char streamOn_enableAimOn[3]={0x55,0x01,0xaa};
	unsigned char streamOn_enableIllumOn[3]={0x59,0x01,0xa6};
	unsigned char streamOn_enableAcqOn[3]={0x58,0x01,0xa7};
	unsigned char respbuf[2] = {0};

	rc = se4750_mipi_i2c_txdata(client,streamOn_enableHW,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableHW failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableHW[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableHW success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}


	rc = se4750_mipi_i2c_txdata(client,streamOn_enableMIPI,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableMIPI failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableMIPI[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableMIPI success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}


	rc = se4750_mipi_i2c_txdata(client,streamOn_enableAimOn,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableAimOn failed rc=%d",rc);
		goto ERR_EXIT;
	}
	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableAimOn[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableAimOn success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}



	rc = se4750_mipi_i2c_txdata(client,streamOn_enableIllumOn,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableIllumOn failed rc=%d",rc);
		goto ERR_EXIT;
	}
	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableIllumOn[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableIllumOn success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}


	rc = se4750_mipi_i2c_txdata(client,streamOn_enableAcqOn,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOn_enableAcqOn failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOn_enableAcqOn[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOn_enableAcqOn success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}


ERR_EXIT:
	return rc;
}


static int streamOff_cmds(struct  i2c_client *client){
	int rc = 0;
	int i = 0;
	int retry = 10;
//	unsigned char streamOff_AimOff[3]={0x55,0x00,0xab};
//	unsigned char streamOff_IllumOff[3]={0x59,0x00,0xa7};
	unsigned char streamOff_AcqOff[3]={0x58,0x00,0xa8};
	unsigned char respbuf[2] = {0};
#if 0
	rc = se4750_mipi_i2c_txdata(client,streamOff_AimOff,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOff_AimOff failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOff_AimOff[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOff_AimOff success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

		rc = se4750_mipi_i2c_txdata(client,streamOff_IllumOff,3);
	if(rc < 0){
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOff_IllumOff failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0;i<10;i++){
		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOff_IllumOff[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOff_IllumOff success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

#endif
	for(i=0; i < retry ; i++){
		rc = se4750_mipi_i2c_txdata(client,streamOff_AcqOff,3);
		if(rc > 0)
		{
			break;
		}
		else
		{
			msleep(1);
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOff_AcqOff failed rc=%d retry=%d", rc, i);
		}
	}
	if (i >= retry)
	{
		CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_txdata streamOff_AcqOff failed rc=%d",rc);
		goto ERR_EXIT;
	}

	for(i=0; i < retry ; i++){

		rc = se4750_mipi_i2c_rxdata(client,respbuf,2);
		if(rc > 0 &&  respbuf[0] == streamOff_AcqOff[0] && respbuf[1] == 0x80){
			CAM_ERR(CAM_SENSOR,"se4750_mipi_i2c_rxdata streamOff_AcqOff success! i =%d",i);
			break;
		}else{
			msleep(1);
		}
	}

ERR_EXIT:
	return rc;
}


static void cam_sensor_update_req_mgr(
	struct cam_sensor_ctrl_t *s_ctrl,
	struct cam_packet *csl_packet)
{
	struct cam_req_mgr_add_request add_req;

	add_req.link_hdl = s_ctrl->bridge_intf.link_hdl;
	add_req.req_id = csl_packet->header.request_id;
	CAM_ERR(CAM_SENSOR, " Rxed Req Id: %lld",
		csl_packet->header.request_id);
	add_req.dev_hdl = s_ctrl->bridge_intf.device_hdl;
	add_req.skip_before_applying = 0;
	if (s_ctrl->bridge_intf.crm_cb &&
		s_ctrl->bridge_intf.crm_cb->add_req)
		s_ctrl->bridge_intf.crm_cb->add_req(&add_req);

	CAM_ERR(CAM_SENSOR, " add req to req mgr: %lld",
			add_req.req_id);
}

static void cam_sensor_release_stream_rsc(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int rc;

	i2c_set = &(s_ctrl->i2c_data.streamoff_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Streamoff settings");
	}

	i2c_set = &(s_ctrl->i2c_data.streamon_settings);
	if (i2c_set->is_settings_valid == 1) {
		i2c_set->is_settings_valid = -1;
		rc = delete_request(i2c_set);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed while deleting Streamon settings");
	}
}

static void cam_sensor_release_per_frame_resource(
	struct cam_sensor_ctrl_t *s_ctrl)
{
	struct i2c_settings_array *i2c_set = NULL;
	int i, rc;

	if (s_ctrl->i2c_data.per_frame != NULL) {
		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(s_ctrl->i2c_data.per_frame[i]);
			if (i2c_set->is_settings_valid == 1) {
				i2c_set->is_settings_valid = -1;
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"delete request: %lld rc: %d",
						i2c_set->request_id, rc);
			}
		}
	}
}

static int32_t cam_sensor_i2c_pkt_parse(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int32_t rc = 0;
	uintptr_t generic_ptr;
	struct cam_control *ioctl_ctrl = NULL;
	struct cam_packet *csl_packet = NULL;
	struct cam_cmd_buf_desc *cmd_desc = NULL;
	struct cam_buf_io_cfg *io_cfg = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	size_t len_of_buff = 0;
	size_t remain_len = 0;
	uint32_t *offset = NULL;
	struct cam_config_dev_cmd config;
	struct i2c_data_settings *i2c_data = NULL;

	ioctl_ctrl = (struct cam_control *)arg;

	if (ioctl_ctrl->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_SENSOR, "Invalid Handle Type");
		return -EINVAL;
	}

	if (copy_from_user(&config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(config)))
		return -EFAULT;

	rc = cam_mem_get_cpu_buf(
		config.packet_handle,
		&generic_ptr,
		&len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Failed in getting the packet: %d", rc);
		return rc;
	}

	remain_len = len_of_buff;
	if ((sizeof(struct cam_packet) > len_of_buff) ||
		((size_t)config.offset >= len_of_buff -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_SENSOR,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buff);
		rc = -EINVAL;
		goto end;
	}

	remain_len -= (size_t)config.offset;
	csl_packet = (struct cam_packet *)(generic_ptr +
		(uint32_t)config.offset);

	if ((csl_packet == NULL) || cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_SENSOR, "Invalid packet params");
		rc = -EINVAL;
		goto end;

	}

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG &&
		csl_packet->header.request_id <= s_ctrl->last_flush_req
		&& s_ctrl->last_flush_req != 0) {
		CAM_ERR(CAM_SENSOR,
			"reject request %lld, last request to flush %u",
			csl_packet->header.request_id, s_ctrl->last_flush_req);
		rc = -EINVAL;
		goto end;
	}

	if (csl_packet->header.request_id > s_ctrl->last_flush_req)
		s_ctrl->last_flush_req = 0;

	i2c_data = &(s_ctrl->i2c_data);
	CAM_DBG(CAM_SENSOR, "Header OpCode: %d", csl_packet->header.op_code);
	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG: {
		i2c_reg_settings = &i2c_data->init_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG: {
		i2c_reg_settings = &i2c_data->config_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMON: {
		if (s_ctrl->streamon_count > 0)
			goto end;

		s_ctrl->streamon_count = s_ctrl->streamon_count + 1;
		i2c_reg_settings = &i2c_data->streamon_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_STREAMOFF: {
		if (s_ctrl->streamoff_count > 0)
			goto end;

		s_ctrl->streamoff_count = s_ctrl->streamoff_count + 1;
		i2c_reg_settings = &i2c_data->streamoff_settings;
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_READ: {
		i2c_reg_settings = &(i2c_data->read_settings);
		i2c_reg_settings->request_id = 0;
		i2c_reg_settings->is_settings_valid = 1;

		CAM_DBG(CAM_SENSOR, "number of IO configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_SENSOR, "No I/O configs to process");
			goto end;
		}

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_SENSOR, "I/O config is invalid(NULL)");
			goto end;
		}
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed Update packets without linking");
			goto end;
		}

		i2c_reg_settings =
			&i2c_data->per_frame[csl_packet->header.request_id %
				MAX_PER_FRAME_ARRAY];
		CAM_DBG(CAM_SENSOR, "Received Packet: %lld req: %lld",
			csl_packet->header.request_id % MAX_PER_FRAME_ARRAY,
			csl_packet->header.request_id);
		if (i2c_reg_settings->is_settings_valid == 1) {
			CAM_ERR(CAM_SENSOR,
				"Already some pkt in offset req : %lld",
				csl_packet->header.request_id);
			/*
			 * Update req mgr even in case of failure.
			 * This will help not to wait indefinitely
			 * and freeze. If this log is triggered then
			 * fix it.
			 */
			cam_sensor_update_req_mgr(s_ctrl, csl_packet);
			goto end;
		}
		break;
	}
	case CAM_SENSOR_PACKET_OPCODE_SENSOR_NOP: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_ACQUIRE)) {
			CAM_WARN(CAM_SENSOR,
				"Rxed NOP packets without linking");
			goto end;
		}

		cam_sensor_update_req_mgr(s_ctrl, csl_packet);
		goto end;
	}
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Packet Header");
		rc = -EINVAL;
		goto end;
	}

	offset = (uint32_t *)&csl_packet->payload;
	offset += csl_packet->cmd_buf_offset / 4;
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	rc = cam_sensor_i2c_command_parser(&s_ctrl->io_master_info,
			i2c_reg_settings, cmd_desc, 1, io_cfg);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "Fail parsing I2C Pkt: %d", rc);
		goto end;
	}

	if ((csl_packet->header.op_code & 0xFFFFFF) ==
		CAM_SENSOR_PACKET_OPCODE_SENSOR_UPDATE) {
		i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		cam_sensor_update_req_mgr(s_ctrl, csl_packet);
	}

end:
	return rc;
}

int32_t cam_se47xx_driver_cmd(struct cam_sensor_ctrl_t *s_ctrl,
	void *arg)
{
	int rc = 0, pkt_opcode = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_sensor_power_ctrl_t *power_info =
		&s_ctrl->sensordata->power_info;
	if (!s_ctrl || !arg) {
		CAM_ERR(CAM_SENSOR, "s_ctrl is NULL");
		return -EINVAL;
	}

	CAM_ERR(CAM_SENSOR, "wuyq enter %s type:0x%x", __func__, cmd->op_code);
	if (cmd->op_code != CAM_SENSOR_PROBE_CMD) {
		if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
			CAM_ERR(CAM_SENSOR, "Invalid handle type: %d",
				cmd->handle_type);
			return -EINVAL;
		}
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	switch (cmd->op_code) {
	case CAM_SENSOR_PROBE_CMD: {
		if (s_ctrl->is_probe_succeed == 1) {
			CAM_ERR(CAM_SENSOR,
				"Already Sensor Probed in the slot");
			break;
		}

		if (cmd->handle_type ==
			CAM_HANDLE_MEM_HANDLE) {
			rc = cam_handle_mem_ptr(cmd->handle, s_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "Get Buffer Handle Failed");
				goto release_mutex;
			}
		} else {
			CAM_ERR(CAM_SENSOR, "Invalid Command Type: %d",
				 cmd->handle_type);
			rc = -EINVAL;
			goto release_mutex;
		}

		/* Parse and fill vreg params for powerup settings */
		rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_setting,
			s_ctrl->sensordata->power_info.power_setting_size);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for PUP rc %d",
				 rc);
			goto free_power_settings;
		}

		/* Parse and fill vreg params for powerdown settings*/
		rc = msm_camera_fill_vreg_params(
			&s_ctrl->soc_info,
			s_ctrl->sensordata->power_info.power_down_setting,
			s_ctrl->sensordata->power_info.power_down_setting_size);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR,
				"Fail in filling vreg params for PDOWN rc %d",
				 rc);
			goto free_power_settings;
		}

		/* Power up and probe sensor */
		rc = cam_sensor_power_up(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "power up failed");
			goto free_power_settings;
		}

		/* Match sensor ID */
		rc = cam_se47xx_match_id(s_ctrl);
		if (rc < 0) {
			cam_sensor_power_down(s_ctrl);
			msleep(20);
			goto free_power_settings;
		}

		CAM_INFO(CAM_SENSOR,
			"Probe success,slot:%d,slave_addr:0x%x,sensor_id:0x%x",
			s_ctrl->soc_info.index,
			s_ctrl->sensordata->slave_info.sensor_slave_addr,
			s_ctrl->sensordata->slave_info.sensor_id);

		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "fail in Sensor Power Down");
			goto free_power_settings;
		}
		/*
		 * Set probe succeeded flag to 1 so that no other camera shall
		 * probed on this slot
		 */
		s_ctrl->is_probe_succeed = 1;
		s_ctrl->sensor_state = CAM_SENSOR_INIT;
	}
		break;
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev sensor_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		if ((s_ctrl->is_probe_succeed == 0) ||
			(s_ctrl->sensor_state != CAM_SENSOR_INIT)) {
			CAM_WARN(CAM_SENSOR,
				"Not in right state to aquire %d， probe %d",
				s_ctrl->sensor_state, s_ctrl->is_probe_succeed);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (s_ctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_SENSOR, "Device is already acquired");
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = copy_from_user(&sensor_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(sensor_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed Copying from user");
			goto release_mutex;
		}

		bridge_params.session_hdl = sensor_acq_dev.session_handle;
		bridge_params.ops = &s_ctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = s_ctrl;

		sensor_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		s_ctrl->bridge_intf.device_hdl = sensor_acq_dev.device_handle;
		s_ctrl->bridge_intf.session_hdl = sensor_acq_dev.session_handle;

		CAM_DBG(CAM_SENSOR, "Device Handle: %d",
			sensor_acq_dev.device_handle);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&sensor_acq_dev,
			sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}

        /* for preview debug */
		if (!is_opened_by_SDL) {
            rc = cam_sensor_power_up(s_ctrl);
            if (rc < 0) {
                CAM_ERR(CAM_SENSOR, "Sensor Power up failed");
                goto release_mutex;
            }
		}

		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
		s_ctrl->last_flush_req = 0;
		CAM_INFO(CAM_SENSOR,
			"CAM_ACQUIRE_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	case CAM_RELEASE_DEV: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_START)) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to release : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		if (s_ctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_SENSOR,
				"Device [%d] still active on link 0x%x",
				s_ctrl->sensor_state,
				s_ctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

        /* for preview debug */
		if (is_opened_by_SDL) {
			is_opened_by_SDL = 0;
        }

		rc = cam_sensor_power_down(s_ctrl);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Sensor Power Down failed");
			goto release_mutex;
		}

		cam_sensor_release_per_frame_resource(s_ctrl);
		cam_sensor_release_stream_rsc(s_ctrl);
		if (s_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_SENSOR,
				"Invalid Handles: link hdl: %d device hdl: %d",
				s_ctrl->bridge_intf.device_hdl,
				s_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(s_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR,
				"failed in destroying the device hdl");
		s_ctrl->bridge_intf.device_hdl = -1;
		s_ctrl->bridge_intf.link_hdl = -1;
		s_ctrl->bridge_intf.session_hdl = -1;

		s_ctrl->sensor_state = CAM_SENSOR_INIT;
		CAM_INFO(CAM_SENSOR,
			"CAM_RELEASE_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
		s_ctrl->streamon_count = 0;
		s_ctrl->streamoff_count = 0;
		s_ctrl->last_flush_req = 0;
	}
		break;
	case CAM_QUERY_CAP: {
		struct  cam_sensor_query_cap sensor_cap;

		cam_sensor_query_cap(s_ctrl, &sensor_cap);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&sensor_cap, sizeof(struct  cam_sensor_query_cap))) {
			CAM_ERR(CAM_SENSOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
	}
	CAM_ERR(CAM_SENSOR, "wuyq  %s type:CAM_QUERY_CAP", __func__);
		break;

	case CAM_START_DEV: {
		if ((s_ctrl->sensor_state == CAM_SENSOR_INIT) ||
			(s_ctrl->sensor_state == CAM_SENSOR_START)) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to start : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

        //paul don't send commands if it's opened by SDL
		if (is_opened_by_SDL) {
			//msleep(100);
			CAM_INFO(CAM_SENSOR, "CAM_START_DEV with is_opened_by_SDL = %d", is_opened_by_SDL);
		} else {
            if ( 1 == 1 )
                rc = se47xx_streamOn(s_ctrl->io_master_info.client);
            else
			    rc = streamOn_cmds(s_ctrl->io_master_info.client);

        }

		s_ctrl->sensor_state = CAM_SENSOR_START;
		CAM_INFO(CAM_SENSOR,
			"CAM_START_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	case CAM_STOP_DEV: {
		if (s_ctrl->sensor_state != CAM_SENSOR_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_SENSOR,
			"Not in right state to stop : %d",
			s_ctrl->sensor_state);
			goto release_mutex;
		}

		//paul don't send commands if it's opened by SDL
		if (is_opened_by_SDL) 		{
			//msleep(100);
			CAM_INFO(CAM_SENSOR, "CAM_START_DEV with is_opened_by_SDL = %d", is_opened_by_SDL);
		} else {
            if (1 == 1)
			    rc = se47xx_streamOff(s_ctrl->io_master_info.client);
            else
			    rc = streamOff_cmds(s_ctrl->io_master_info.client);

        }

		cam_sensor_release_per_frame_resource(s_ctrl);
		s_ctrl->last_flush_req = 0;
		s_ctrl->sensor_state = CAM_SENSOR_ACQUIRE;
		CAM_INFO(CAM_SENSOR,
			"CAM_STOP_DEV Success, sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	case CAM_CONFIG_DEV: {
		rc = cam_sensor_i2c_pkt_parse(s_ctrl, arg);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "Failed i2c pkt parse: %d", rc);
			goto release_mutex;
		}
        if (s_ctrl->sensordata->slave_info.sensor_id == 0x80) {
            rc = 0;
            break;
        }
		if (s_ctrl->i2c_data.init_settings.is_settings_valid &&
			(s_ctrl->i2c_data.init_settings.request_id == 0)) {

			pkt_opcode =
				CAM_SENSOR_PACKET_OPCODE_SENSOR_INITIAL_CONFIG;
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				pkt_opcode);

			if ((rc == -EAGAIN) &&
			(s_ctrl->io_master_info.master_type == CCI_MASTER)) {
				/* If CCI hardware is resetting we need to wait
				 * for sometime before reapply
				 */
				CAM_WARN(CAM_SENSOR,
					"Reapplying the Init settings due to cci hw reset");
				usleep_range(1000, 1010);
				rc = cam_sensor_apply_settings(s_ctrl, 0,
					pkt_opcode);
			}
			s_ctrl->i2c_data.init_settings.request_id = -1;

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply init settings rc= %d",
					rc);
				delete_request(&s_ctrl->i2c_data.init_settings);
				goto release_mutex;
			}
			rc = delete_request(&s_ctrl->i2c_data.init_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"Fail in deleting the Init settings");
				goto release_mutex;
			}
		}

		if (s_ctrl->i2c_data.config_settings.is_settings_valid &&
			(s_ctrl->i2c_data.config_settings.request_id == 0)) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_CONFIG);

			s_ctrl->i2c_data.config_settings.request_id = -1;

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply config settings");
				delete_request(
					&s_ctrl->i2c_data.config_settings);
				goto release_mutex;
			}
			rc = delete_request(&s_ctrl->i2c_data.config_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"Fail in deleting the config settings");
				goto release_mutex;
			}
			s_ctrl->sensor_state = CAM_SENSOR_CONFIG;
		}

		if (s_ctrl->i2c_data.read_settings.is_settings_valid) {
			rc = cam_sensor_apply_settings(s_ctrl, 0,
				CAM_SENSOR_PACKET_OPCODE_SENSOR_READ);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"cannot apply read settings");
				delete_request(
					&s_ctrl->i2c_data.read_settings);
				goto release_mutex;
			}
			rc = delete_request(
				&s_ctrl->i2c_data.read_settings);
			if (rc < 0) {
				CAM_ERR(CAM_SENSOR,
					"Fail in deleting the read settings");
				goto release_mutex;
			}
		}

		CAM_DBG(CAM_SENSOR,
			"CAM_CONFIG_DEV done sensor_id:0x%x,sensor_slave_addr:0x%x",
			s_ctrl->sensordata->slave_info.sensor_id,
			s_ctrl->sensordata->slave_info.sensor_slave_addr);
	}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid Opcode: %d", cmd->op_code);
		rc = -EINVAL;
		goto release_mutex;
	}

release_mutex:
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	return rc;
}

static ssize_t device_sysfs_se47xx_camstatus_show (struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
	char *s = buf;
	mutex_lock(&g_camstatus_mutex);
	s += sprintf(buf, g_camstatus);
	mutex_unlock(&g_camstatus_mutex);
	return s - buf + 1;
}

static ssize_t device_sysfs_se47xx_camstatus_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
	char lbuf[BUF_SIZE];
	mutex_lock(&g_camstatus_mutex);
	if (count <= 0 || count >= sizeof(g_camstatus)) {
		goto exit_camstatus_store;
	}
	memcpy(lbuf, buf, count);
	lbuf[count] = 0;
	if (lbuf[0] == '#' && strcmp(g_camstatus, lbuf + 1) == 0) {
		strcpy(g_camstatus, "none");
	} else if (lbuf[0] != '#' && strcmp(g_camstatus, "none") == 0) {
		strcpy(g_camstatus, lbuf);
	}

exit_camstatus_store:
	mutex_unlock(&g_camstatus_mutex);
	return count;
}

static DEVICE_ATTR(camstatus, 0664, device_sysfs_se47xx_camstatus_show, device_sysfs_se47xx_camstatus_store);

static ssize_t device_sysfs_se47xx_is_scan_mode_show (struct device* dev,
    struct device_attribute *attr,
    char *buf)
{
	return sprintf(buf, "%d\n", g_is_scan_mode);
}

static DEVICE_ATTR(is_scan_mode, S_IRUGO,
    device_sysfs_se47xx_is_scan_mode_show,
    NULL);

static ssize_t device_sysfs_se47xx_scan_exist_status_show (struct device* dev,
    struct device_attribute *attr,
    char *buf)
{
	return sprintf(buf, "%d\n", scan_exist_status);
}

static DEVICE_ATTR(scan_exist_status, S_IRUGO,
    device_sysfs_se47xx_scan_exist_status_show,
    NULL);

struct kobject *device_se47xx_kobj = NULL;
static int device_sysfs_init (void)
{
	int rc = 0;

	device_se47xx_kobj = kobject_create_and_add("se47xx", NULL);
	if (device_se47xx_kobj == NULL) {
		pr_err("kobject_creat_and_add(/sys/se47xx/) failed!");
		return -ENOMEM;
	}

	rc = sysfs_create_file(device_se47xx_kobj,&dev_attr_is_scan_mode.attr);
	if (rc) {
		pr_err("sysfs_creat_file(/sys/se47xx/is_scan_mode) failed!");
		goto destroy_kobj;
	}

	rc = sysfs_create_file(device_se47xx_kobj, &dev_attr_camstatus.attr);
	if (rc) {
		pr_err("sysfs_create_file('/sys/se47xx/camstatus') failed");
		goto destroy_is_scanstatus_mode;
	}

	rc = sysfs_create_file(device_se47xx_kobj, &dev_attr_scan_exist_status.attr);
	if (rc) {
		pr_err("sysfs_create_file('/sys/se47xx/scan_exist_status') failed");
		goto destroy_scan_exist_status_mode;
	}

	mutex_init(&g_camstatus_mutex);

	return rc;

destroy_scan_exist_status_mode:
	sysfs_remove_file(device_se47xx_kobj, &dev_attr_camstatus.attr);
destroy_is_scanstatus_mode:
	sysfs_remove_file(device_se47xx_kobj, &dev_attr_is_scan_mode.attr);
destroy_kobj:
	kobject_put(device_se47xx_kobj);
	return rc;
}

static void device_sysfs_deinit (void)
{
	mutex_destroy(&g_camstatus_mutex);
  	sysfs_remove_file(device_se47xx_kobj, &dev_attr_scan_exist_status.attr);
	sysfs_remove_file(device_se47xx_kobj, &dev_attr_camstatus.attr);
	sysfs_remove_file(device_se47xx_kobj, &dev_attr_is_scan_mode.attr);
	kobject_put(device_se47xx_kobj);
}

static int sdl_control_open (struct inode* node, struct file* file)
{
	int rc = 0;
	if (se47xx_misc_wa)
	{
		CAM_ERR(CAM_SENSOR, "se47xx_misc_wa is not null");
		rc = cam_sensor_power_up(se47xx_misc_wa);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "power up failed");
			//goto free_power_settings;
		}
	}
	is_opened_by_SDL = 1;
	CAM_INFO(CAM_SENSOR, "SDL open %s:%d is_opened_by_SDL=%d\n",__func__,__LINE__, is_opened_by_SDL);
	return 0;
}

//#define MIMIC_ENGINE_RESPONSE	1
#ifdef MIMIC_ENGINE_RESPONSE
static int mimic_cmd_response = 0;
static int last_cmd_op_code = 0;
#endif

static long sdl_control_ioctl (struct file* file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client*			pClient;
	struct i2c_rdwr_ioctl_data	I2CData;
	struct i2c_msg				I2CMsg;
	u8 __user*					pData;
	long						lRetVal;

	//pr_err("yanhao open %s:%d \n",__func__,__LINE__);

	if ((cmd != I2C_RDWR) || !arg )
	{
		return(-EINVAL);
	}


	pClient =dev_get_drvdata(sdl_control_device.this_device);
    //pClient = file->private_data->sensor_i2c_client->client;
	//CAM_ERR(CAM_SENSOR, "yanhao open %s:%d client.addr = 0x%x \n",__func__,__LINE__,pClient->addr);

	// Copy data structure argument from user-space
	if ( copy_from_user(&I2CData, (struct i2c_rdwr_ioctl_data __user*) arg, sizeof(I2CData)) )
	{
		return(-EFAULT);
	}

	// Only allow one message at a time
	if ( I2CData.nmsgs != 1 )
	{
		return(-EINVAL);
	}

	// Copy the message structure from user-space
	if ( copy_from_user(&I2CMsg, I2CData.msgs, sizeof(struct i2c_msg)) )
	{
		return(-EFAULT);
	}

	lRetVal = 0;
	// Only allow transfers to the SE4750, limit the size of the message and don't allow received length changes
	if ( (I2CMsg.addr != SE47XX_I2C_DEVICE_ADDR) || (I2CMsg.len > 256) || (I2CMsg.flags & I2C_M_RECV_LEN) )
	{
		return(-EINVAL);
	}

	// Map the data buffer from user-space
	pData = (u8 __user*) I2CMsg.buf;
	I2CMsg.buf = memdup_user(pData, I2CMsg.len);
	if ( IS_ERR(I2CMsg.buf) )
	{
		return(PTR_ERR(I2CMsg.buf));
	}
    if (!(I2CMsg.flags & I2C_M_RD))
    {
        print_hex_dump(KERN_ERR, "sdl_control_ioctl: w:", DUMP_PREFIX_NONE, 16, 1, I2CMsg.buf, I2CMsg.len, true);
#ifdef MIMIC_ENGINE_RESPONSE
		if (I2CMsg.buf[0] == 0x77 || I2CMsg.buf[0] == 0x58)
		{
			mimic_cmd_response = 1;
			last_cmd_op_code = I2CMsg.buf[0];
			kfree(I2CMsg.buf);
			return I2CMsg.len;
		}
#endif
	}
	else
	{
#ifdef MIMIC_ENGINE_RESPONSE
		if (mimic_cmd_response)
		{
			mimic_cmd_response = 0;
			I2CMsg.buf[0] = last_cmd_op_code;
			I2CMsg.buf[1] = 0x80;
			I2CMsg.len = 2;
			if ( copy_to_user(pData, I2CMsg.buf, I2CMsg.len) )
			{
				lRetVal = -EFAULT;
			}
			kfree(I2CMsg.buf);
			return I2CMsg.len;
		}
#endif
	}
	// Perform the I2C transfer
	lRetVal = i2c_transfer(pClient->adapter, &I2CMsg, 1);
	if ( (lRetVal >= 0) && (I2CMsg.flags & I2C_M_RD) )
	{
		print_hex_dump(KERN_ERR, "sdl_control_ioctl: r:", DUMP_PREFIX_NONE, 16, 1, I2CMsg.buf, I2CMsg.len, true);
		// Successful read, copy data to user-space
		if ( copy_to_user(pData, I2CMsg.buf, I2CMsg.len) )
		{
			lRetVal = -EFAULT;
		}
	}
	kfree(I2CMsg.buf);
	return lRetVal;
}

static int sdl_control_release (struct inode* node, struct file* file)
{
	CAM_ERR(CAM_SENSOR,"enter");
	return 0;
}

struct file_operations sdl_control_fops =
{
	.owner = THIS_MODULE,
	.unlocked_ioctl = sdl_control_ioctl,
	.open = sdl_control_open,
	.release = sdl_control_release,
};

static int cam_se47xx_driver_i2c_remove(struct i2c_client *client)
{
	int                        i;
	struct cam_sensor_ctrl_t  *s_ctrl = i2c_get_clientdata(client);
	struct cam_hw_soc_info    *soc_info;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "sensor device is NULL");
		return 0;
	}

	CAM_INFO(CAM_SENSOR, "i2c remove invoked");
	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	cam_sensor_shutdown(s_ctrl);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
	soc_info = &s_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	kfree(s_ctrl->i2c_data.per_frame);
	v4l2_set_subdevdata(&(s_ctrl->v4l2_dev_str.sd), NULL);
	kfree(s_ctrl);

	return 0;
}

int32_t cam_sensor_apply_request_nop(struct cam_req_mgr_apply_request *apply)
{
	CAM_INFO(CAM_SENSOR,"%s Enter", __func__);
    return 0;
}

int32_t cam_sensor_flush_request_nop(struct cam_req_mgr_flush_request *flush_req)
{
	CAM_INFO(CAM_SENSOR,"%s Enter", __func__);
    return 0;
}

static int32_t se47xx_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
	int32_t rc = 0;
    int i = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct cam_hw_soc_info   *soc_info = NULL;

	CAM_ERR(CAM_SENSOR,"%s Enter", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_SENSOR,
			"%s :i2c_check_functionality failed", client->name);
		return -EFAULT;
	}

	/* Create barcode control structure */
	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR,
			"%s :kzalloc for s_ctrl acquire mem failed, rc(%d)", client->name, rc);
		return -ENOMEM;
    }

    /* Set the barcode private data */
	i2c_set_clientdata(client, s_ctrl);

	s_ctrl->io_master_info.client = client;
	soc_info = &s_ctrl->soc_info;
	soc_info->dev = &client->dev;
	soc_info->dev_name = client->name;

	/* Initialize sensor device type */
	s_ctrl->of_node = client->dev.of_node;
	s_ctrl->io_master_info.master_type = I2C_MASTER;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->last_flush_req = 0;

    /* Parse dt data for init*/
	rc = cam_sensor_parse_dt(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR,
			"%s :cam_sensor_parse_dt failed, rc(%d)", client->name, rc);
		goto free_s_ctrl;
	}

    /* Set ioctl interface for up layer */
	rc = cam_sensor_init_subdev_params(s_ctrl);
	if (rc) {
		CAM_ERR(CAM_SENSOR,
			"%s :cam_sensor_init_subdev_params failed, rc(%d)", client->name, rc);
		goto free_s_ctrl;
    }

	s_ctrl->i2c_data.per_frame =
		(struct i2c_settings_array *)
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (s_ctrl->i2c_data.per_frame == NULL) {
		CAM_ERR(CAM_SENSOR,
			"%s :kzalloc for per_frame acquire mem failed, rc(%d)", client->name, rc);
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(s_ctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamon_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamoff_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(s_ctrl->i2c_data.per_frame[i].list_head));

	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.link_hdl = -1;
	s_ctrl->bridge_intf.ops.get_dev_info = cam_sensor_publish_dev_info;
	s_ctrl->bridge_intf.ops.link_setup = cam_sensor_establish_link;
	s_ctrl->bridge_intf.ops.apply_req = cam_sensor_apply_request_nop;
	s_ctrl->bridge_intf.ops.flush_req = cam_sensor_flush_request_nop;

	s_ctrl->sensordata->power_info.dev = soc_info->dev;

    /* Registering sdl_control */
	CAM_INFO(CAM_SENSOR, "Registering sdl_control");
	//se47xx_misc_wa = s_ctrl;
	rc = misc_register(&sdl_control_device);
	if (rc) {
		CAM_ERR(CAM_SENSOR,
			"%s :misc_register failed, rc(%d)", client->name, rc);
	} else {
		device_sysfs_init();
	}

	return rc;
unreg_subdev:
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
free_s_ctrl:
	kfree(s_ctrl);
	return rc;
}

static int se47xx_i2c_remove (struct i2c_client *client)
{
	CAM_ERR(CAM_SENSOR,"enter");
	cam_se47xx_driver_i2c_remove(client);
	device_sysfs_deinit();
	CAM_ERR(CAM_SENSOR,"exit");
	return 0;
}

static const struct i2c_device_id se47xx_i2c_id[] = {
	{ SCANNER_SENSOR_NAME, (kernel_ulong_t)NULL },
	{ }
};

static const struct of_device_id se47xx_dt_match[] = {
	{ .compatible = "zebra,se47xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, se47xx_dt_match);

static struct i2c_driver cam_se47xx_driver_i2c = {
	.id_table = se47xx_i2c_id,
	.probe = se47xx_i2c_probe,
	.remove = se47xx_i2c_remove,
	.driver = {
		.name = SCANNER_SENSOR_NAME,
		.of_match_table = se47xx_dt_match,
	},
};


static int __init se47xx_init_module (void)
{
	CAM_ERR(CAM_SENSOR,"++");
	return i2c_add_driver(&cam_se47xx_driver_i2c);
}

static void __exit se47xx_exit_module (void)
{
	CAM_ERR(CAM_SENSOR,"++");
	return i2c_del_driver(&cam_se47xx_driver_i2c);
}

module_init(se47xx_init_module);
module_exit(se47xx_exit_module);
MODULE_DESCRIPTION("Zebra SE47XX 2D scanner driver");
MODULE_LICENSE("GPL v2");
