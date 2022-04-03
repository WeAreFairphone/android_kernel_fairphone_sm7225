// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/dma-contiguous.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

#define      default_gain_X 0.075
#define      default_gain_Y 0.05
#define      FW_VERSION_MAJOR 0x0001
#define      FW_VERSION_MINOR 0x002B
#define OIS_FW_FILE "/vendor/etc/camera/cm401_fw.txt"


typedef struct REGSETTING{
	uint16_t reg ;
	uint16_t val ;
}REGSETTING ;

typedef struct REGSETTING_1{
	uint16_t reg ;
	uint32_t val ;
}REGSETTING_1 ;


static int32_t gyro_offset_X = 0;
static int32_t gyro_offset_Y = 0;
static int32_t gyro_offset_X_check = -1;
static int32_t gyro_offset_Y_check = -1;
static int32_t ois_reg_value = -1;


static int calibration_status = 0;
static int ois_status = 0;
static int ois_init_status = 0;

extern float gyro_gain_X;
extern float gyro_gain_Y;
static int32_t decrease_gain_X = 0;
static int32_t decrease_gain_Y = 0;

int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;

	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	if (ois_acq_dev.device_handle <= 0) {
		CAM_ERR(CAM_OIS, "Can not create device handle");
		return -EFAULT;
	}
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc)
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;

	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

static int cam_ois_gyro_calibration(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	int32_t                            rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;
  	uint32_t                           fw_size;
	uint32_t cmd_adress=0,cmd_data=0;
	uint32_t c=0,d=0;
	float target_gain_X = 0.0;
	float target_gain_Y = 0.0;

	const REGSETTING cml_ois_gyro_calibration[]= {
		//gyro cali mode
		{0x9b2c ,0x0002} ,//0[write]enter calibration mode
		{0x9b28 ,0x2000} ,//1[read]check mode
		{0x9b2a ,0x0001} ,//2[write]
		{0x9b28 ,0x2001} ,//3[read]calibration done
		{0x9fb0 ,0x8001} ,//4[read]gyro offset calibration result
		{0x9fb6 ,0x0000} ,//5[read]gyro offset X
		{0x9fb8 ,0x0000} ,//6[read]gyro offset Y
		//save mode
		{0x9b2c ,0x0006} ,//7[write]store mode
		{0x9b28 ,0x6000} ,//8[read]
		{0x0220 ,0xc0d4} ,//9[write]code pt off
		{0x9b2a ,0x0001} ,//10[write]
		{0x9b28 ,0x6001} ,//11[read]store done
		{0x0220 ,0x0000} ,//12[write]code pt on

		{0x0018 ,0x0001} ,//13[write]
		{0x9E18 ,0x0002} ,//14[write]
		{0x0024 ,0x0001} ,//15[write]
		{0x9fb2 ,0x0000} ,//16[read]
		{0x9fb4 ,0x0000} ,//17[read]
	};

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	total_bytes = sizeof(cml_ois_gyro_calibration[0]);

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;

	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *	total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		return -ENOMEM;
	}
	//[Begin]enter gyro cali mode
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (page_address(page));

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[13].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[13].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	CAM_ERR(CAM_OIS, "write 0x0018 -> 0x0001");

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[14].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[14].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	CAM_ERR(CAM_OIS, "write 0x9E18 -> 0x0002");

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[15].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[15].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	CAM_ERR(CAM_OIS, "write 0x0024 -> 0x0001");

	mdelay(50);
	if (decrease_gain_X == 0 && decrease_gain_Y == 0)
	{
		target_gain_X = gyro_gain_X - default_gain_X;
		target_gain_Y = gyro_gain_Y - default_gain_Y;
		c = (int) (target_gain_X*8192);
		d = (int) (target_gain_Y*8192);

		if (c<=11264 && c>=5939 && d<=11060 && d>=6144)//0.725-1.375,0.75-1.35
		{
			i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[16].reg;
			i2c_reg_setting.reg_setting[0].reg_data = c;
			i2c_reg_setting.reg_setting[0].delay = 1;
			i2c_reg_setting.reg_setting[0].data_mask = 0;
			CAM_ERR(CAM_OIS, "write 0x9fb2 -> 0x%x(default offset)",c);
			rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

			mdelay(50);
			i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[17].reg;
			i2c_reg_setting.reg_setting[0].reg_data = d;
			i2c_reg_setting.reg_setting[0].delay = 1;
			i2c_reg_setting.reg_setting[0].data_mask = 0;
			CAM_ERR(CAM_OIS, "write 0x9fb4 -> 0x%x(default offset)",d);
			rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
		}
		else
			CAM_ERR(CAM_OIS, "invalid x(0x%x) or y(0x%x) gain",c,d);
	}
	else
	{
		target_gain_X = gyro_gain_X - (float) decrease_gain_X/1000;
		target_gain_Y = gyro_gain_Y - (float) decrease_gain_Y/1000;
		c = (int) (target_gain_X*8192);
		d = (int) (target_gain_Y*8192);
		
		if (c<=11264 && c>=5939 && d<=11060 && d>=6144)//0.725-1.375,0.75-1.35
		{
			i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[16].reg;
			i2c_reg_setting.reg_setting[0].reg_data = c;
			i2c_reg_setting.reg_setting[0].delay = 1;
			i2c_reg_setting.reg_setting[0].data_mask = 0;
			CAM_ERR(CAM_OIS, "write 0x9fb2 -> 0x%x(manual)",c);
			rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

			mdelay(50);
			i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[17].reg;
			i2c_reg_setting.reg_setting[0].reg_data = d;
			i2c_reg_setting.reg_setting[0].delay = 1;
			i2c_reg_setting.reg_setting[0].data_mask = 0;
			CAM_ERR(CAM_OIS, "write 0x9fb4 -> 0x%x(manual)",d);
			rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
		}
		else
			CAM_ERR(CAM_OIS, "invalid x(0x%x) or y(0x%x) gain",c,d);
	}

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[0].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[0].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2c -> 0x0002");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	mdelay(50);
	cmd_adress = cml_ois_gyro_calibration[5].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9fb6 -> 0x%x",cmd_data);
		
	mdelay(50);
	cmd_adress = cml_ois_gyro_calibration[6].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9fb8 -> 0x%x",cmd_data);
		
	mdelay(50);
	cmd_adress = cml_ois_gyro_calibration[1].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9b28 -> 0x%x",cmd_data);

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[2].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[2].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2a -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);


	mdelay(3000);
	cmd_adress = cml_ois_gyro_calibration[3].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9b28 -> 0x%x",cmd_data);

	if (cmd_data == cml_ois_gyro_calibration[3].val)
	{
		CAM_ERR(CAM_OIS, "read {0x9b28 ,0x2001} success");
	}
	
	mdelay(50);
	cmd_adress = cml_ois_gyro_calibration[4].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9fb0 -> 0x%x",cmd_data);

	mdelay(3000);
	cmd_adress = cml_ois_gyro_calibration[5].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9fb6 -> 0x%x",cmd_data);
	gyro_offset_X = cmd_data;

						
	mdelay(50);
	cmd_adress = cml_ois_gyro_calibration[6].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9fb8 -> 0x%x",cmd_data);
	gyro_offset_Y = cmd_data;
						

	CAM_ERR(CAM_OIS,"ois calibration op end");
	//[End]enter gyro cali mode

	//[Begin]enter save mode
	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[7].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[7].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2c -> 0x0006");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write {0x9b2a ,0x0001} failed %d", rc);
	}

	mdelay(50);
	cmd_adress = cml_ois_gyro_calibration[8].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9b28 -> 0x%x",cmd_data);


	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[9].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[9].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0220 -> 0xc0d4");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write 0x0220 -> 0xc0d4 failed %d", rc);
	}

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[10].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[10].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2a -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write 0x9b2a -> 0x0001 failed %d", rc);
	}

	mdelay(50);
	cmd_adress = cml_ois_gyro_calibration[11].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS,"read 0x9b28 -> 0x%x",cmd_data);

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_gyro_calibration[12].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_gyro_calibration[12].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0220 -> 0x0000");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write 0x0220 -> 0x0000 failed %d", rc);
	}
	CAM_ERR(CAM_OIS,"ois save gyro offset end");
	//[End]enter save mode
	
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),	page, fw_size);
	page = NULL;

    return rc;
}


static int cam_ois_gyro_offset_check(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                            rc = 0;
	uint32_t cmd_adress=0,cmd_data=0;
	
	const REGSETTING cml_ois_gyro_offset_check[]= {
		{0x9fb6 ,0x0000} ,//0[read]gyro offset X
		{0x9fb8 ,0x0000} ,//1[read]gyro offset Y
	};

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	
	mdelay(50);
	cmd_adress = cml_ois_gyro_offset_check[0].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) 
	{
		CAM_ERR(CAM_OIS, "read 0x9fb6  failed: %d",rc);
	}
	else
	{
		CAM_ERR(CAM_OIS,"read 0x9fb6 -> 0x%x",cmd_data);
		gyro_offset_X_check = cmd_data;
	}

	
	mdelay(50);
	cmd_adress = cml_ois_gyro_offset_check[1].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) 
	{
		CAM_ERR(CAM_OIS, "read 0x9fb8  failed: %d",rc);
	}
	else
	{
		CAM_ERR(CAM_OIS,"read 0x9fb8 -> 0x%x",cmd_data);
		gyro_offset_Y_check = cmd_data;
	}
	
    return rc;
}


static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);

	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.prog;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	page = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.coeff;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);

release_firmware:
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	release_firmware(fw);

	return rc;
}

const REGSETTING cml_ois_control[]= {
	{0x9b2c ,0x0001} ,//[0]
	{0x9b2a ,0x0001} ,//[1]
	{0x9b28 ,0x1001} ,//[2]
	{0x9b2a ,0x0003} ,//[3]enter idle low power mode
	{0x9b28 ,0x1003} ,//[4]

//ois init before sr test
	{0x0018 ,0x0001} ,//[5]
	{0x9e18 ,0x0002} ,//[6]
	{0x0024 ,0x0001} ,//[7]
	{0x8820 ,0x0028} ,//[8]
	{0x9b2a ,0x0002} ,//[9]
};

const REGSETTING_1 cml_ois_fw[]= {
	{0x302c ,0x000012b7} ,//[0]
};


static int cam_ois_init(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	int32_t                            rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;
  	uint32_t                           fw_size;
	
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	total_bytes = sizeof(cml_ois_control[0]);

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;

	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *	total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (page_address(page));

/*************			OIS init BEGIN	                ******************/
	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[5].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[5].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0018 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[6].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[6].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9e18 -> 0x0002");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[7].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[7].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0024 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[8].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[8].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x8820 -> 0x0028");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[9].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[9].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2a -> 0x0002");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	mdelay(50);
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),	page, fw_size);
	page = NULL;

	return rc;
}


ssize_t ois_init_before_sr_test_show(struct device *dev, struct device_attribute *attr, char *buf){
	
	return sprintf(buf, "%u\n", ois_init_status);

}

ssize_t ois_init_before_sr_test_store(struct device *dev,  struct device_attribute *attr, const char *buf, size_t count){
	struct cam_ois_ctrl_t *o_ctrl = NULL;
	char cmd_buf[32];
	uint32_t cmd_adress=0,cmd_data=0;
	char flag;
	int rc = 0;

	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	memset(cmd_buf,0,32);
	o_ctrl = platform_get_drvdata(pdev);


	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return count;
	}
	//cpy user cmd to kernel 0x:0x:r  0x:0x:w
	strcpy(cmd_buf,buf);
	sscanf(cmd_buf,"%x:%x:%c",&cmd_adress,&cmd_data,&flag);

	if ((flag == 'w') && (cmd_adress == 0x0018) && (cmd_data == 0x0001))
	{
		CAM_ERR(CAM_OIS, "prepare ois write:adress=0x%x,data=0x%x",cmd_adress,cmd_data);
		rc = cam_ois_init(o_ctrl);
		if (rc == 0)
			ois_init_status = 1;
		else 
			ois_init_status = 0;
	}
	
	return count;
}

static int cam_cml_ois_enable(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	int32_t                            rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;
  	uint32_t                           fw_size;
	

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	total_bytes = sizeof(cml_ois_control[0]);
		
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;

	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *	total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		return -ENOMEM;
	}
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (page_address(page));
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[0].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[0].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2c -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write {0x9b2c ,0x0001} failed %d", rc);

	}
	
	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[1].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[1].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2a -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write {0x9b2a ,0x0001} failed %d", rc);
	}
	mdelay(50);
	
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),	page, fw_size);
	page = NULL;

    return rc;
}

static int cam_cml_ois_disable(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	int32_t                            rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;
  	uint32_t                           fw_size;
	

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	total_bytes = sizeof(cml_ois_control[0]);
		
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;

	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *	total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		return -ENOMEM;
	}
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (page_address(page));
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[0].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[0].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2c -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write {0x9b2c ,0x0001} failed %d", rc);

	}
	
	mdelay(50);
	i2c_reg_setting.reg_setting[0].reg_addr = cml_ois_control[3].reg;
	i2c_reg_setting.reg_setting[0].reg_data = cml_ois_control[3].val;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9b2a -> 0x0000");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write {0x9b2a ,0x0000} failed %d", rc);
	}
	mdelay(50);
	
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),	page, fw_size);
	page = NULL;

    return rc;
}

static int32_t get_size_from_file(const char *filename, uint64_t* size)
{
	struct kstat stat;
	mm_segment_t fs;
	int rc = 0;

	stat.size = 0;

	fs = get_fs();
	set_fs(KERNEL_DS);

	rc = vfs_stat(filename,&stat);
	if(rc < 0)
	{
		pr_err("vfs_stat(%s) failed, rc = %d\n",filename,rc);
		rc = -1;
		goto END;
	}

	*size = stat.size;
	END:
	set_fs(fs);
	return rc;
}

static int read_file_into_buffer(const char *filename, uint8_t* data, uint32_t size)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	int rc;

	fp = filp_open(filename,O_RDONLY,S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR(fp)) {
		pr_err("open(%s) failed\n", filename);
		return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	pos = 0;
	rc = vfs_read(fp, data, size, &pos);

	set_fs(fs);
	filp_close(fp, NULL);

	return rc;
}

static uint32_t change_byte_to_dword_data(uint8_t* pBuffer,int count,int size)
{
	uint32_t read_data=0;
	uint32_t tmp[8]={0};
	int i=0;

	for(i=0;i<size;i++)
	{
		if (pBuffer[count+i]<58 && pBuffer[count+i] >47)
			pBuffer[count+i] = pBuffer[count+i] - 48;
		else if(pBuffer[count+i] < 103 && pBuffer[count+i]>96)
			pBuffer[count+i] = pBuffer[count+i] -87;

		tmp[i]= pBuffer[count+i];
	}
	read_data = ((tmp[6]<<28)&0xF0000000) | 
				((tmp[7]<<24)&0x0F000000) |
				((tmp[4]<<20)&0x00F00000) |
				((tmp[5]<<16)&0x000F0000) | 
				((tmp[2]<<12)&0x0000F000) | 
				((tmp[3]<<8)&0x00000F00) | 
				((tmp[0]<<4)&0x000000F0) | 
				((tmp[1]<<0)&0x0000000F);
	
	return read_data;
}


static int cam_cml_ois_fw_upgrade(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint16_t                           total_bytes_1 = 0;
	int32_t                            rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting_1;
	struct page                       *page = NULL;
	struct page                       *page_1 = NULL;
  	uint32_t                           fw_size,fw_size_1;
	unsigned short addr= 0;
	uint32_t csH=0,csL=0;
    uint32_t mcs_checksum_flash=0,if_checksum_flash=0;
	int get_size = 0;
	uint64_t size;
	uint8_t* pBuffer;
	int i= 0;
	uint32_t read_data=0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = get_size_from_file(OIS_FW_FILE, &size);
	CAM_ERR(CAM_OIS, "size 0x%x",size);
	if (rc != 0)
		CAM_ERR(CAM_OIS, "get ois fw file size failed.");

	pBuffer = kzalloc(size, GFP_KERNEL);

	if (pBuffer) {
		get_size = read_file_into_buffer(OIS_FW_FILE, pBuffer, size);
		CAM_ERR(CAM_OIS, "get_size 0x%x",get_size);
		if(get_size < 0)
			kfree(pBuffer);
	}else{
		CAM_ERR(CAM_OIS,"alloc buffer fail\n");
	}
	
#if 1	
	total_bytes = sizeof(cml_ois_control[0]);
	total_bytes_1 = sizeof(cml_ois_fw[0]);
	
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;

	i2c_reg_setting_1.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting_1.data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	i2c_reg_setting_1.size = total_bytes_1;
	i2c_reg_setting_1.delay = 0;

	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *	total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		return -ENOMEM;
	}

	fw_size_1 = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *	total_bytes_1) >> PAGE_SHIFT;
	page_1 = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),fw_size_1, 0, GFP_KERNEL);
	if (!page_1) {
		CAM_ERR(CAM_OIS, "Failed in allocating page_1 i2c_array");
		return -ENOMEM;
	}
	
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (page_address(page));
	i2c_reg_setting_1.reg_setting = (struct cam_sensor_i2c_reg_array *) (page_address(page_1));
	
	/* cm4x1_MCS_download begin*/
	mdelay(1);
	i2c_reg_setting.reg_setting[0].reg_addr = 0x0020;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0020 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	
	mdelay(1);
	i2c_reg_setting.reg_setting[0].reg_addr = 0x0024;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0000;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0024 -> 0x0000");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	mdelay(1);
	i2c_reg_setting.reg_setting[0].reg_addr = 0x0220;
	i2c_reg_setting.reg_setting[0].reg_data = 0xC0D4;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0220 -> 0xC0D4");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	
	mdelay(1);
	i2c_reg_setting.reg_setting[0].reg_addr = 0x3000;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0000;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x3000 -> 0x0000");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	mdelay(1);

	addr = 0x8000;//MCS_START_ADDRESS
	for (i=0; i < 16; i++)
    {
        //erase address
		i2c_reg_setting.reg_setting[0].reg_addr = 0x3008;
		i2c_reg_setting.reg_setting[0].reg_data = addr;
		i2c_reg_setting.reg_setting[0].delay = 1;
		i2c_reg_setting.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x3008 -> 0x%x",addr);
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

		mdelay(1);
        //erase sector 2kbyte
        i2c_reg_setting.reg_setting[0].reg_addr = 0x300C;
		i2c_reg_setting.reg_setting[0].reg_data = 0x0002;
		i2c_reg_setting.reg_setting[0].delay = 1;
		i2c_reg_setting.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x300C -> 0x0002");
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
        addr += 0x800; //2kbyte
        mdelay(5);
    }
	
	/* flash fw write */
	addr = 0x8000;//MCS_START_ADDRESS
	for(i=0;i<65536;i+=8){
		read_data=change_byte_to_dword_data(pBuffer,i,8);
		//CAM_ERR(CAM_OIS, "read_data[%d]:0x%x,",i/4,read_data);

		i2c_reg_setting.reg_setting[0].reg_addr = 0x3028;
		i2c_reg_setting.reg_setting[0].reg_data = addr;
		i2c_reg_setting.reg_setting[0].delay = 1;
		i2c_reg_setting.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x3028 -> 0x%x",addr);
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
		
        addr += 0x4; //2kbyte

		/* program sequential write 2K byte */
		i2c_reg_setting_1.reg_setting[0].reg_addr = 0x302C;
		i2c_reg_setting_1.reg_setting[0].reg_data = read_data;
		i2c_reg_setting_1.reg_setting[0].delay = 1;
		i2c_reg_setting_1.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x302C -> 0x%x",read_data);
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting_1);
	}

	i2c_reg_setting.reg_setting[0].reg_addr = 0x3048;
	i2c_reg_setting.reg_setting[0].reg_data = 0x8000;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x3048 -> 0x8000");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	i2c_reg_setting.reg_setting[0].reg_addr = 0x304C;
	i2c_reg_setting.reg_setting[0].reg_data = 0x2000;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x304C -> 0x2000");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	mdelay(1);
	
	i2c_reg_setting.reg_setting[0].reg_addr = 0x3050;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x3050 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	mdelay(1);
	
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),0x3054,&csH,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS, "csH : 0x%x",csH);
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),0x3058,&csL,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS, "csL : 0x%x",csL);
    mcs_checksum_flash = (csH << 16) + csL;
    //if(mcs_checksum_criteria != mcs_checksum_flash)
    //{
    //    msg = CHECKSUM_ERROR;
    //    log_tprintf(1, _T("mcs checksum fail, mcs_checksum_criteria: 0x%08X, mcs_checksum_flash: 0x%08X"), mcs_checksum_criteria, mcs_checksum_flash);
        //code protection on
    //    RamWriteA(0x0220, 0x0000);
    //    return msg;
    //}
	CAM_ERR(CAM_OIS, "mcs_checksum_flash : 0x%08X",mcs_checksum_flash);

	i2c_reg_setting.reg_setting[0].reg_addr = 0x0018;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0018 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	mdelay(10);

	/* cm4x1_MCS_download end*/

	/* cm4x1_IF_download begin*/
	
    //chip enable
    i2c_reg_setting.reg_setting[0].reg_addr = 0x0020;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0020 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

    //stanby mode(MCU off)
    i2c_reg_setting.reg_setting[0].reg_addr = 0x0024;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0000;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0024 -> 0x0000");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

    //code protection
    i2c_reg_setting.reg_setting[0].reg_addr = 0x0220;
	i2c_reg_setting.reg_setting[0].reg_data = 0xC0D4;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0220 -> 0xC0D4");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

    //select if flash
    i2c_reg_setting.reg_setting[0].reg_addr = 0x3000;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x3000 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

    mdelay(1);
    addr = 0x0000;
    for (i=0; i < 4; i++)
    {
        //erase address
		i2c_reg_setting.reg_setting[0].reg_addr = 0x3008;
		i2c_reg_setting.reg_setting[0].reg_data = addr;
		i2c_reg_setting.reg_setting[0].delay = 1;
		i2c_reg_setting.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x3008 -> 0x%x",addr);
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

        //erase page 512 byte
        i2c_reg_setting.reg_setting[0].reg_addr = 0x300C;
		i2c_reg_setting.reg_setting[0].reg_data = 0x0001;
		i2c_reg_setting.reg_setting[0].delay = 1;
		i2c_reg_setting.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x300C -> 0x0001");
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
        addr += 0x200; //512 byte
        mdelay(5);
    }
    addr = 0x0000;
    for (i = 65536; i < 69632; i+=8)
    {
	    /* program sequential write 2K byte */
		read_data=change_byte_to_dword_data(pBuffer,i,8);
		//CAM_ERR(CAM_OIS, "read_data[%d]:0x%x,",i/4,read_data);
		
		i2c_reg_setting.reg_setting[0].reg_addr = 0x3028;
		i2c_reg_setting.reg_setting[0].reg_data = addr;
		i2c_reg_setting.reg_setting[0].delay = 1;
		i2c_reg_setting.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x3028 -> 0x%x",addr);
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
		addr += 0x4;
		
		/* program sequential write 2K byte */
		i2c_reg_setting_1.reg_setting[0].reg_addr = 0x302C;
		i2c_reg_setting_1.reg_setting[0].reg_data = read_data;
		i2c_reg_setting_1.reg_setting[0].delay = 1;
		i2c_reg_setting_1.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x302C -> 0x%x",read_data);
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting_1);
    }

    /* Checksum calculation for fw data */

    /* Set the checksum area */
	i2c_reg_setting.reg_setting[0].reg_addr = 0x3048;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0000;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x3048 -> 0x0000");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	i2c_reg_setting.reg_setting[0].reg_addr = 0x304C;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0200;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x304C -> 0x0200");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);

	i2c_reg_setting.reg_setting[0].reg_addr = 0x3050;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x3050 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
    mdelay(1);
	
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),0x3054,&csH,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS, "csH : 0x%x",csH);
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),0x3058,&csL,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS, "csL : 0x%x",csL);
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),0x3058,&csL,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS, "csL : 0x%x",csL);

    if_checksum_flash = (csH << 16) + csL;
    //if(if_checksum_criteria != if_checksum_flash)
    //{
    //    msg = CHECKSUM_ERROR;
    //    log_tprintf(1, _T("if checksum fail, if_checksum_criteria: 0x%08X, if_checksum_flash: 0x%08X"), if_checksum_criteria, if_checksum_flash);
        //code protection on
    //    RamWriteA(0x0220, 0x0000);
    //    return msg;
    //}
    
	CAM_ERR(CAM_OIS, "if_checksum_flash : 0x%08X",if_checksum_flash);

	i2c_reg_setting.reg_setting[0].reg_addr = 0x0018;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x0018 -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
	mdelay(10);
	
	/* cm4x1_IF_download end*/
	kfree(pBuffer);
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),	page, fw_size);
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),	page_1, fw_size_1);
	page = NULL;
	page_1 = NULL;
	
#endif
    return rc;
}


ssize_t ois_reg_show(struct device *dev, struct device_attribute *attr, char *buf){
	
	return sprintf(buf, "0x%x\n", ois_reg_value);

}

ssize_t ois_reg_store(struct device *dev,  struct device_attribute *attr, const char *buf, size_t count){

	struct cam_ois_ctrl_t *o_ctrl = NULL;
	char cmd_buf[32];
	uint32_t cmd_adress=0,cmd_data=0,read_data=0;
	char flag;
	int rc = 0;
	int i = 0;
	uint32_t fw_version_major = 0;
	uint32_t fw_version_minor = 0;

	uint16_t                           total_bytes = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;
  	uint32_t                           fw_size;
	
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	memset(cmd_buf,0,32);
	o_ctrl = platform_get_drvdata(pdev);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return count;
	}

	total_bytes = sizeof(cml_ois_control[0]);
		
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;

	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *	total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		return -ENOMEM;
	}
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (page_address(page));
	
	//cpy user cmd to kernel 0x:0x:r/w
	strcpy(cmd_buf,buf);
	sscanf(cmd_buf,"%x:%x:%c",&cmd_adress,&cmd_data,&flag);

	if (flag == 'r')
	{
		mdelay(50);
		rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&read_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc < 0) 
		{
			CAM_ERR(CAM_OIS, "read %x  failed: %d",cmd_adress,rc);
		}
		else
		{
			CAM_ERR(CAM_OIS,"read %x -> 0x%x",cmd_adress,read_data);
			ois_reg_value = read_data;
		}
		mdelay(50);
	}
	else if (flag == 'w')
	{
		mdelay(50);
		i2c_reg_setting.reg_setting[0].reg_addr = cmd_adress;
		i2c_reg_setting.reg_setting[0].reg_data = cmd_data;
		i2c_reg_setting.reg_setting[0].delay = 1;
		i2c_reg_setting.reg_setting[0].data_mask = 0;
		CAM_ERR(CAM_OIS, "write 0x%x -> 0x%x",cmd_adress,cmd_data);
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_reg_setting);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write 0x%x -> 0x%x failed %d",cmd_adress,cmd_data,rc);
		
		}
		mdelay(50);
	}
	else if (flag == 'u')//upgrade fw 
	{
		mdelay(10);
		cmd_adress = 0x9b08;
		rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&fw_version_major,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
		CAM_ERR(CAM_OIS, "read 0x9b08 -> 0x%x",fw_version_major);
		
		mdelay(10);
		cmd_adress = 0x9b0a;
		rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&fw_version_minor,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
		CAM_ERR(CAM_OIS, "read 0x9b0a -> 0x%x",fw_version_minor);
		if (fw_version_major == FW_VERSION_MAJOR && fw_version_minor == FW_VERSION_MINOR)
		{
			CAM_ERR(CAM_OIS, "The fw version of ois is already up to date,fw_version:0x%x-0x%x ",fw_version_major,fw_version_minor);
		}
		else
		{
			CAM_ERR(CAM_OIS, "The fw version of ois is old,current fw_version:0x%x-0x%x ,target fw_version:0x%x-0x%x.Now begin to upgrate!!!",fw_version_major,fw_version_minor,FW_VERSION_MAJOR,FW_VERSION_MINOR);
			rc = cam_cml_ois_fw_upgrade(o_ctrl);

			mdelay(10);
			cmd_adress = 0x9b08;
			rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&fw_version_major,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
			CAM_ERR(CAM_OIS, "read 0x9b08 -> 0x%x",fw_version_major);

			mdelay(10);
			cmd_adress = 0x9b0a;
			rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&fw_version_minor,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
			CAM_ERR(CAM_OIS, "read 0x9b0a -> 0x%x",fw_version_minor);
			if (fw_version_major == FW_VERSION_MAJOR && fw_version_minor == FW_VERSION_MINOR)
			{
				CAM_ERR(CAM_OIS, "upgrade success!!!");
			}
			else
				CAM_ERR(CAM_OIS, "upgrade failed!!!");
			
		}
	}
	else
	{
		if (flag == 'k')
		{
			while(i < 3000)
			{
				mdelay(10);
				cmd_adress = 0x9bc0;
				rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&read_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
				pr_err("yjw cam_camera_cci_i2c_read_seq line%d: addr = 0x%x ,Data: 0x%x \n",__LINE__,cmd_adress,read_data);
				cmd_adress = 0x9c00;
				rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&read_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
				pr_err("yjw cam_camera_cci_i2c_read_seq line%d: addr = 0x%x ,Data: 0x%x \n",__LINE__,cmd_adress,read_data);
				cmd_adress = 0x9bc2;
				rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&read_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
				pr_err("yjw cam_camera_cci_i2c_read_seq line%d: addr = 0x%x ,Data: 0x%x \n",__LINE__,cmd_adress,read_data);
				cmd_adress = 0x9c02;
				rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&read_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
				pr_err("yjw cam_camera_cci_i2c_read_seq line%d: addr = 0x%x ,Data: 0x%x \n",__LINE__,cmd_adress,read_data);
				i++;
			}
		}
	}

	return count;
}

static char ois_read_cmd_buf[32];

ssize_t ois_position_data_show(struct device *dev, struct device_attribute *attr, char *buf){
	
	strcpy(buf,ois_read_cmd_buf);
	return sizeof(ois_read_cmd_buf);
}

ssize_t ois_position_data_store(struct device *dev,  struct device_attribute *attr, const char *buf, size_t count){

	struct cam_ois_ctrl_t *o_ctrl = NULL;
	int32_t 						   rc = 0;
	char cmd_buf[5];
	int32_t flag;
	uint32_t cmd_adress=0,cmd_data=0;
	uint32_t position_data[5];
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	memset(cmd_buf,0,5);
	memset(ois_read_cmd_buf,0,sizeof(ois_read_cmd_buf));
	o_ctrl = platform_get_drvdata(pdev);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return count;
	}
	strcpy(cmd_buf,buf);
	sscanf(cmd_buf,"%d",&flag);

	if (flag == 1) {
		cmd_adress = 0x9CB8;//POSITION_ESTIMATE_X
		rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "ois Failed: random read I2C settings: %d",rc);
			return count;
		} 
		else{
	        CAM_DBG(CAM_OIS,"ois read::address: 0x%x  reg_data: 0x%x",cmd_adress,cmd_data);
			position_data[0] = cmd_data;
	    }

		cmd_adress = 0x9CBA;//POSITION_ESTIMATE_Y
		rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "ois Failed: random read I2C settings: %d",rc);
			return count;
		} 
		else{
	        CAM_DBG(CAM_OIS,"ois read::address: 0x%x  reg_data: 0x%x",cmd_adress,cmd_data);
			position_data[1] = cmd_data;
	    }
		sprintf(ois_read_cmd_buf,"%.4x%.4x\n",position_data[0],position_data[1]);

		CAM_DBG(CAM_OIS,"ois kernel read::position_X 0x%x,position_Y 0x%x",position_data[0],position_data[1]);
	}

	
	return count;
}

ssize_t ois_gain_set_show(struct device *dev, struct device_attribute *attr, char *buf){
	
	return sprintf(buf, "%d,%d\n", decrease_gain_X,decrease_gain_Y);
}

ssize_t ois_gain_set_store(struct device *dev,  struct device_attribute *attr, const char *buf, size_t count){

	struct cam_ois_ctrl_t *o_ctrl = NULL;
	char cmd_buf[32];
	char flag;
	int32_t x_gain=0,y_gain=0;
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	memset(cmd_buf,0,32);
	o_ctrl = platform_get_drvdata(pdev);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return count;
	}
	//cpy user cmd to kernel x_gain:y_gain:w
	strcpy(cmd_buf,buf);
	sscanf(cmd_buf,"%d:%d:%c",&x_gain,&y_gain,&flag);

	if (flag == 'w')
	{
		CAM_ERR(CAM_OIS, "x_gain %d,y_gain %d",x_gain,y_gain);
		decrease_gain_X = x_gain;
		decrease_gain_Y = y_gain;
	}
	
	return count;
}


ssize_t ois_status_show(struct device *dev, struct device_attribute *attr, char *buf){

	struct cam_ois_ctrl_t *o_ctrl = NULL;
	uint32_t cmd_adress=0,cmd_data=0;
	int rc = 0;
	
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	o_ctrl = platform_get_drvdata(pdev);

	cmd_adress = cml_ois_control[2].reg;
	rc = camera_io_dev_read(&(o_ctrl->io_master_info),cmd_adress,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);

	if (cmd_data == cml_ois_control[2].val) {
		ois_status = 1;
		CAM_ERR(CAM_OIS,"read 0x9b28 -> 0x%x",cmd_data);
	}
	else if (cmd_data == cml_ois_control[4].val) {
		ois_status = 2;
		CAM_ERR(CAM_OIS,"read 0x9b28 -> 0x%x",cmd_data);
	}
	else {
		ois_status = 0;
		CAM_ERR(CAM_OIS,"read 0x9b28 -> 0x%x",cmd_data);
	}
	
	return sprintf(buf, "%u\n", ois_status);

}

ssize_t ois_status_store(struct device *dev,  struct device_attribute *attr, const char *buf, size_t count){

	struct cam_ois_ctrl_t *o_ctrl = NULL;
	char cmd_buf[32];
	uint32_t cmd_adress=0,cmd_data=0;
	char flag;
	
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	memset(cmd_buf,0,32);
	o_ctrl = platform_get_drvdata(pdev);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return count;
	}
	//cpy user cmd to kernel 0x:0x:r  0x:0x:w
	strcpy(cmd_buf,buf);
	sscanf(cmd_buf,"%x:%x:%c",&cmd_adress,&cmd_data,&flag);

/*************			OIS enable BEGIN	                ******************/
	if ((flag == 'w') && (cmd_adress == 0x9b2a) && (cmd_data == 0x0001))
	{
		cam_cml_ois_enable(o_ctrl);

	}
/*************			OIS enable END	    				******************/

/*************			OIS disable BEGIN					******************/
	if ((flag == 'w') && (cmd_adress == 0x9b2a) && (cmd_data == 0x0003))
	{
		cam_cml_ois_disable(o_ctrl);
	}
/*************			OIS disable BEGIN					******************/

	return count;
}


ssize_t ois_gyro_cali_data_show(struct device *dev, struct device_attribute *attr, char *buf){
	
	return sprintf(buf, "%u\n", calibration_status);

}

ssize_t ois_gyro_cali_data_store(struct device *dev,  struct device_attribute *attr, const char *buf, size_t count){

	struct cam_ois_ctrl_t *o_ctrl = NULL;
	char cmd_buf[32];
	uint32_t cmd_adress=0,cmd_data=0;
	char flag;
	int gyro_cali_flag=0,check_gyro_offset=0;
	int rc = 0;

	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	memset(cmd_buf,0,32);
	o_ctrl = platform_get_drvdata(pdev);


	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return count;
	}
	//cpy user cmd to kernel 0x:0x:r  0x:0x:w
	strcpy(cmd_buf,buf);
	sscanf(cmd_buf,"%x:%x:%c",&cmd_adress,&cmd_data,&flag);

	if(flag=='w'){
		CAM_ERR(CAM_OIS, "ois write:adress=0x%x,data=0x%x",cmd_adress,cmd_data);
		if ((cmd_adress == 0x9b2c) && (cmd_data == 0x0002))//gyro offset calibration mode
		{
			gyro_cali_flag = 1;
		}
		else
		{
			gyro_cali_flag = 0;
		}
		if (gyro_cali_flag)
		{
			rc = cam_ois_gyro_calibration(o_ctrl);
		}
		CAM_ERR(CAM_OIS, "cam_ois_gyro_calibration end");
	}

	if (flag=='r')
	{
		CAM_ERR(CAM_OIS, "ois read:adress1=0x%x,adress2=0x%x",cmd_adress,cmd_data);
		if ((cmd_adress == 0x9fb6) && (cmd_data == 0x9fb8))//check gyro_offset
		{
			check_gyro_offset = 1;
		}
		else
		{
			check_gyro_offset = 0;
		}
		if (check_gyro_offset)
		{
			rc = cam_ois_gyro_offset_check(o_ctrl);
		}
		CAM_ERR(CAM_OIS, "cam_ois_gyro_calibration end");
	}
	if ((gyro_offset_X == gyro_offset_X_check) && (gyro_offset_Y == gyro_offset_Y_check))
		calibration_status = 1;
	
	return count;
}

/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		return -EINVAL;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_OIS, "Invalid packet params");
		return -EINVAL;
	}


	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				return rc;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				return -EINVAL;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				return -EINVAL;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					return rc;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					return rc;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					return rc;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					return rc;
				}
			}
			break;
			}
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				return rc;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if ((rc == -EAGAIN) &&
			(o_ctrl->io_master_info.master_type == CCI_MASTER)) {
			CAM_WARN(CAM_OIS,
				"CCI HW is restting: Reapplying INIT settings");
			usleep_range(1000, 1010);
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_init_data);
		}
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Cannot apply Init settings: rc = %d",
				rc);
			goto pwr_dwn;
		}

		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			}
		}

		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			return rc;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			return rc;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_READ: {
		struct cam_buf_io_cfg *io_cfg;
		struct i2c_settings_array i2c_read_settings;

		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to read OIS: %d",
				o_ctrl->cam_ois_state);
			return rc;
		}
		CAM_DBG(CAM_OIS, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_OIS, "No I/O configs to process");
			rc = -EINVAL;
			return rc;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_OIS, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			return rc;
		}

		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS read pkt parsing failed: %d", rc);
			return rc;
		}

		rc = cam_sensor_i2c_read_data(
			&i2c_read_settings,
			&o_ctrl->io_master_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "cannot read data rc: %d", rc);
			delete_request(&i2c_read_settings);
			return rc;
		}

		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Failed in deleting the read settings");
			return rc;
		}
		break;
	}
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		return -EINVAL;
	}

	if (!rc)
		return rc;
pwr_dwn:
	cam_ois_power_down(o_ctrl);
	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
		rc = cam_ois_power_down(o_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;
	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
