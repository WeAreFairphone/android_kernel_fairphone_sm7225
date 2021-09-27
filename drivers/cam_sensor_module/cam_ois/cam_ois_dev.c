// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2018, 2020, The Linux Foundation. All rights reserved.
 */

#include "cam_ois_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_ois_soc.h"
#include "cam_ois_core.h"
#include "cam_debug_util.h"
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>
#include <linux/dma-contiguous.h>

struct cam_ois_ctrl_t *o_ctrl_vsync = NULL;
int64_t timestamp_ois;
uint8_t *position_X_Y_timestamps=NULL;
uint8_t data_for_vsync[28] = {0};


typedef struct REGSETTING{
	uint16_t reg ;
	uint16_t val ;
}REGSETTING ;

const REGSETTING ois_fw_control[]= {
	{0x9b2c ,0x0001} ,//[0]
};



static long cam_ois_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int                       rc     = 0;
	struct cam_ois_ctrl_t *o_ctrl = v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_ois_driver_cmd(o_ctrl, arg);
		break;
	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

static int cam_ois_subdev_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_ois_ctrl_t *o_ctrl =
		v4l2_get_subdevdata(sd);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "o_ctrl ptr is NULL");
			return -EINVAL;
	}

	mutex_lock(&(o_ctrl->ois_mutex));
	o_ctrl->open_cnt++;
	CAM_DBG(CAM_OIS, "OIS open count %d", o_ctrl->open_cnt);
	mutex_unlock(&(o_ctrl->ois_mutex));

	return 0;
}

static int cam_ois_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_ois_ctrl_t *o_ctrl =
		v4l2_get_subdevdata(sd);

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "o_ctrl ptr is NULL");
			return -EINVAL;
	}

	mutex_lock(&(o_ctrl->ois_mutex));
	if (o_ctrl->open_cnt <= 0) {
		mutex_unlock(&(o_ctrl->ois_mutex));
		return -EINVAL;
	}
	o_ctrl->open_cnt--;
	CAM_DBG(CAM_OIS, "OIS open count %d", o_ctrl->open_cnt);
	if (o_ctrl->open_cnt == 0)
		cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));

	return 0;
}

static int32_t cam_ois_update_i2c_info(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_ois_i2c_info_t *i2c_info)
{
	struct cam_sensor_cci_client        *cci_client = NULL;

	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		cci_client = o_ctrl->io_master_info.cci_client;
		if (!cci_client) {
			CAM_ERR(CAM_OIS, "failed: cci_client %pK",
				cci_client);
			return -EINVAL;
		}
		cci_client->cci_i2c_master = o_ctrl->cci_i2c_master;
		cci_client->sid = (i2c_info->slave_addr) >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = i2c_info->i2c_freq_mode;
	}

	return 0;
}

#ifdef CONFIG_COMPAT
static long cam_ois_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_OIS,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_ois_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc) {
			CAM_ERR(CAM_OIS,
				"Failed in ois suddev handling rc %d",
				rc);
			return rc;
		}
		break;
	default:
		CAM_ERR(CAM_OIS, "Invalid compat ioctl: %d", cmd);
		rc = -EINVAL;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_OIS,
				"Failed to copy from user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

static const struct v4l2_subdev_internal_ops cam_ois_internal_ops = {
	.open  = cam_ois_subdev_open,
	.close = cam_ois_subdev_close,
};

static struct v4l2_subdev_core_ops cam_ois_subdev_core_ops = {
	.ioctl = cam_ois_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_ois_init_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_ois_subdev_ops = {
	.core = &cam_ois_subdev_core_ops,
};

static int cam_ois_init_subdev_param(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;

	o_ctrl->v4l2_dev_str.internal_ops = &cam_ois_internal_ops;
	o_ctrl->v4l2_dev_str.ops = &cam_ois_subdev_ops;
	strlcpy(o_ctrl->device_name, CAM_OIS_NAME,
		sizeof(o_ctrl->device_name));
	o_ctrl->v4l2_dev_str.name = o_ctrl->device_name;
	o_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	o_ctrl->v4l2_dev_str.ent_function = CAM_OIS_DEVICE_TYPE;
	o_ctrl->v4l2_dev_str.token = o_ctrl;

	rc = cam_register_subdev(&(o_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_OIS, "fail to create subdev");

	return rc;
}

ssize_t ois_timestamps_position_show(struct device *dev, struct device_attribute *attr, char *buf){

	int i = 0, len = 0;

	for( i = 0; i < 28; i++)
		len += sprintf(buf + len, "0x%x,", data_for_vsync[i]);
	len +=sprintf(buf + len , "\n");

	return len;

}

DEVICE_ATTR(ois_gyro_cali_data, 0664, ois_gyro_cali_data_show, ois_gyro_cali_data_store);
DEVICE_ATTR(ois_position_data, 0664, ois_position_data_show, ois_position_data_store);
DEVICE_ATTR(ois_status, 0664, ois_status_show, ois_status_store);
DEVICE_ATTR(ois_reg, 0664, ois_reg_show, ois_reg_store);
DEVICE_ATTR(ois_init_before_sr_test, 0664, ois_init_before_sr_test_show, ois_init_before_sr_test_store);
DEVICE_ATTR(ois_gain_set, 0664, ois_gain_set_show, ois_gain_set_store);
DEVICE_ATTR(ois_timestamps_position_data, 0664, ois_timestamps_position_show, NULL);


static int cml_vsync_pinctrl_init(struct cam_ois_ctrl_t *o_ctrl)
{
	int retval = 0;

	/* Get pinctrl if target uses pinctrl */
	o_ctrl->vsync_pinctrl = devm_pinctrl_get((o_ctrl->pdev->dev.parent));
	if (IS_ERR_OR_NULL(o_ctrl->vsync_pinctrl)) {
		retval = PTR_ERR(o_ctrl->vsync_pinctrl);
		CAM_ERR(CAM_OIS, "Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}
	
	o_ctrl->pin_default = pinctrl_lookup_state(o_ctrl->vsync_pinctrl, "default");
	if (IS_ERR_OR_NULL(o_ctrl->pin_default)) {
		retval = PTR_ERR(o_ctrl->pin_default);
		CAM_ERR(CAM_OIS, "Failed to look up default state\n");
		goto err_pinctrl_lookup;
	}

	return retval;

err_pinctrl_lookup:
	devm_pinctrl_put(o_ctrl->vsync_pinctrl);
err_pinctrl_get:
	o_ctrl->vsync_pinctrl = NULL;
	return retval;
}

irqreturn_t interrupt_ois_vsync_irq(int irq, void *dev)
{
	timestamp_ois = ktime_get_real_ns();
	queue_work(o_ctrl_vsync->ois_vsync_wq, &o_ctrl_vsync->ois_vsync_work);

	return IRQ_HANDLED;
}

static void ois_vsync_function(struct work_struct *work)
{
	uint16_t                           total_bytes = 0;
	int32_t                            rc = 0;
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;
  	uint32_t                           fw_size;
	uint32_t cmd_data=0;

	if (!o_ctrl_vsync) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return;
	}
	total_bytes = sizeof(ois_fw_control[0]);
		
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;

	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *	total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl_vsync->soc_info.dev)),fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		return;
	}
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (page_address(page));
	
	i2c_reg_setting.reg_setting[0].reg_addr = 0x9DFC;
	i2c_reg_setting.reg_setting[0].reg_data = 0x0A04;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9DFC -> 0x0A04");
	rc = camera_io_dev_write(&(o_ctrl_vsync->io_master_info), &i2c_reg_setting);

	i2c_reg_setting.reg_setting[0].reg_addr = 0x9B2C;
	i2c_reg_setting.reg_setting[0].reg_data = 0X0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9B2C -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl_vsync->io_master_info), &i2c_reg_setting);

	i2c_reg_setting.reg_setting[0].reg_addr = 0x9B2A;
	i2c_reg_setting.reg_setting[0].reg_data = 0X0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9B2A -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl_vsync->io_master_info), &i2c_reg_setting);
	
	rc = camera_io_dev_read(&(o_ctrl_vsync->io_master_info),0x9B28,&cmd_data,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	CAM_ERR(CAM_OIS, "read 0x9B28 -> 0x%x",cmd_data);

	i2c_reg_setting.reg_setting[0].reg_addr = 0x9DFE;
	i2c_reg_setting.reg_setting[0].reg_data = 0X0001;
	i2c_reg_setting.reg_setting[0].delay = 1;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	CAM_ERR(CAM_OIS, "write 0x9DFE -> 0x0001");
	rc = camera_io_dev_write(&(o_ctrl_vsync->io_master_info), &i2c_reg_setting);
	mdelay(1);
	
	rc = camera_io_dev_read_seq(&(o_ctrl_vsync->io_master_info),0x9DAC,position_X_Y_timestamps,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD,80);

	data_for_vsync[0] = position_X_Y_timestamps[6];
	data_for_vsync[1] = position_X_Y_timestamps[7];//6X
	data_for_vsync[2] = position_X_Y_timestamps[8];
	data_for_vsync[3] = position_X_Y_timestamps[9];//6Y
	data_for_vsync[4] = position_X_Y_timestamps[22];
	data_for_vsync[5] = position_X_Y_timestamps[23];//7X
	data_for_vsync[6] = position_X_Y_timestamps[24];
	data_for_vsync[7] = position_X_Y_timestamps[25];//7Y
	data_for_vsync[8] = position_X_Y_timestamps[38];
	data_for_vsync[9] = position_X_Y_timestamps[39];//8X
	data_for_vsync[10] = position_X_Y_timestamps[40];
	data_for_vsync[11] = position_X_Y_timestamps[41];//8Y
	data_for_vsync[12] = position_X_Y_timestamps[54];
	data_for_vsync[13] = position_X_Y_timestamps[55];//9X
	data_for_vsync[14] = position_X_Y_timestamps[56];
	data_for_vsync[15] = position_X_Y_timestamps[57];//9Y
	data_for_vsync[16] = position_X_Y_timestamps[70];
	data_for_vsync[17] = position_X_Y_timestamps[71];//10X
	data_for_vsync[18] = position_X_Y_timestamps[72];
	data_for_vsync[19] = position_X_Y_timestamps[73];//10Y
	
	data_for_vsync[20]  = (uint8_t)(timestamp_ois&0x00000000000000FF);
	data_for_vsync[21]  = (uint8_t)((timestamp_ois&0x000000000000FF00)>>8);
	data_for_vsync[22]  = (uint8_t)((timestamp_ois&0x0000000000FF0000)>>16);
	data_for_vsync[23]  = (uint8_t)((timestamp_ois&0x00000000FF000000)>>24);
	data_for_vsync[24]  = (uint8_t)((timestamp_ois&0x000000FF00000000)>>32);
	data_for_vsync[25]  = (uint8_t)((timestamp_ois&0x0000FF0000000000)>>40);
	data_for_vsync[26] = (uint8_t)((timestamp_ois&0x00FF000000000000)>>48);
	data_for_vsync[27] = (uint8_t)((timestamp_ois&0xFF00000000000000)>>56);
	
	cma_release(dev_get_cma_area((o_ctrl_vsync->soc_info.dev)),	page, fw_size);
	page = NULL;

}

static int cam_ois_i2c_driver_probe(struct i2c_client *client,
	 const struct i2c_device_id *id)
{
	int                          rc = 0;
	struct cam_ois_ctrl_t       *o_ctrl = NULL;
	struct cam_ois_soc_private  *soc_private = NULL;

	if (client == NULL || id == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args client: %pK id: %pK",
			client, id);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_OIS, "i2c_check_functionality failed");
		goto probe_failure;
	}

	o_ctrl = kzalloc(sizeof(*o_ctrl), GFP_KERNEL);
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "kzalloc failed");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, o_ctrl);

	o_ctrl->soc_info.dev = &client->dev;
	o_ctrl->soc_info.dev_name = client->name;
	o_ctrl->ois_device_type = MSM_CAMERA_I2C_DEVICE;
	o_ctrl->io_master_info.master_type = I2C_MASTER;
	o_ctrl->io_master_info.client = client;

	soc_private = kzalloc(sizeof(struct cam_ois_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto octrl_free;
	}

	o_ctrl->soc_info.soc_private = soc_private;
	rc = cam_ois_driver_soc_init(o_ctrl);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: cam_sensor_parse_dt rc %d", rc);
		goto soc_free;
	}

	rc = cam_ois_init_subdev_param(o_ctrl);
	if (rc)
		goto soc_free;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
	o_ctrl->open_cnt = 0;

	return rc;

soc_free:
	kfree(soc_private);
octrl_free:
	kfree(o_ctrl);
probe_failure:
	return rc;
}

static int cam_ois_i2c_driver_remove(struct i2c_client *client)
{
	int                             i;
	struct cam_ois_ctrl_t          *o_ctrl = i2c_get_clientdata(client);
	struct cam_hw_soc_info         *soc_info;
	struct cam_ois_soc_private     *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	CAM_INFO(CAM_OIS, "i2c driver remove invoked");
	soc_info = &o_ctrl->soc_info;

	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));

	soc_private =
		(struct cam_ois_soc_private *)soc_info->soc_private;
	power_info = &soc_private->power_info;

	kfree(o_ctrl->soc_info.soc_private);
	v4l2_set_subdevdata(&o_ctrl->v4l2_dev_str.sd, NULL);
	kfree(o_ctrl);

	return 0;
}

static int32_t cam_ois_platform_driver_probe(
	struct platform_device *pdev)
{
	int32_t                         rc = 0;
	struct cam_ois_ctrl_t          *o_ctrl = NULL;
	struct cam_ois_soc_private     *soc_private = NULL;

	o_ctrl = kzalloc(sizeof(struct cam_ois_ctrl_t), GFP_KERNEL);
	if (!o_ctrl)
		return -ENOMEM;

	o_ctrl->soc_info.pdev = pdev;
	o_ctrl->pdev = pdev;
	o_ctrl->soc_info.dev = &pdev->dev;
	o_ctrl->soc_info.dev_name = pdev->name;

	o_ctrl->ois_device_type = MSM_CAMERA_PLATFORM_DEVICE;

	o_ctrl->io_master_info.master_type = CCI_MASTER;
	o_ctrl->io_master_info.cci_client = kzalloc(
		sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!o_ctrl->io_master_info.cci_client)
		goto free_o_ctrl;

	soc_private = kzalloc(sizeof(struct cam_ois_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_cci_client;
	}
	o_ctrl->soc_info.soc_private = soc_private;
	soc_private->power_info.dev  = &pdev->dev;

	INIT_LIST_HEAD(&(o_ctrl->i2c_init_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_calib_data.list_head));
	INIT_LIST_HEAD(&(o_ctrl->i2c_mode_data.list_head));
	mutex_init(&(o_ctrl->ois_mutex));
	rc = cam_ois_driver_soc_init(o_ctrl);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: soc init rc %d", rc);
		goto free_soc;
	}
	
	rc = cml_vsync_pinctrl_init(o_ctrl);
	if (!rc && o_ctrl->vsync_pinctrl) {
		rc = pinctrl_select_state(
				o_ctrl->vsync_pinctrl,
				o_ctrl->pin_default);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Can't select pinctrl state\n");
		}
	}

	o_ctrl->ois_vsync_wq= create_singlethread_workqueue("ois_vsync_wq");
	INIT_WORK(&o_ctrl->ois_vsync_work, ois_vsync_function);
	queue_work(o_ctrl->ois_vsync_wq, &o_ctrl->ois_vsync_work);

	rc = request_irq(gpio_to_irq(o_ctrl->irq_gpio), interrupt_ois_vsync_irq, IRQ_TYPE_EDGE_RISING , "ois_vsync_work", &o_ctrl->soc_info.dev);
	if(rc < 0)
	{
	
		CAM_ERR(CAM_OIS, "request_irq failed\n");
		goto exit_free_gpio;
	}
    enable_irq_wake(gpio_to_irq(o_ctrl->irq_gpio));

	
	rc = cam_ois_init_subdev_param(o_ctrl);
	if (rc)
		goto free_soc;

	if ((device_create_file(&pdev->dev, &dev_attr_ois_gyro_cali_data))  ||
		(device_create_file(&pdev->dev, &dev_attr_ois_position_data))   ||
		(device_create_file(&pdev->dev, &dev_attr_ois_status))			||
		(device_create_file(&pdev->dev, &dev_attr_ois_reg))				||
		(device_create_file(&pdev->dev, &dev_attr_ois_timestamps_position_data))				||
		(device_create_file(&pdev->dev, &dev_attr_ois_gain_set))			||
		(device_create_file(&pdev->dev, &dev_attr_ois_init_before_sr_test)))
	{
		CAM_ERR(CAM_OIS, "creat ois device_create_file failed rc=%d", rc);
	}

	rc = cam_ois_update_i2c_info(o_ctrl, &soc_private->i2c_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed: to update i2c info rc %d", rc);
		goto unreg_subdev;
	}
	o_ctrl->bridge_intf.device_hdl = -1;

	platform_set_drvdata(pdev, o_ctrl);
	o_ctrl->cam_ois_state = CAM_OIS_INIT;
	o_ctrl->open_cnt = 0;
	
	o_ctrl_vsync = o_ctrl;
	position_X_Y_timestamps = (uint8_t *)
		vzalloc(80 * sizeof(uint8_t));
	timestamp_ois = 0;

	return rc;
	
exit_free_gpio:
	gpio_free(o_ctrl->irq_gpio);
unreg_subdev:
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));
free_soc:
	kfree(soc_private);
free_cci_client:
	kfree(o_ctrl->io_master_info.cci_client);
free_o_ctrl:
	kfree(o_ctrl);
	return rc;
}

static int cam_ois_platform_driver_remove(struct platform_device *pdev)
{
	int                             i;
	struct cam_ois_ctrl_t          *o_ctrl;
	struct cam_ois_soc_private     *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info         *soc_info;

	o_ctrl = platform_get_drvdata(pdev);
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "ois device is NULL");
		return -EINVAL;
	}

	device_remove_file(&pdev->dev, &dev_attr_ois_gyro_cali_data);
	device_remove_file(&pdev->dev, &dev_attr_ois_status);
	device_remove_file(&pdev->dev, &dev_attr_ois_position_data);
	device_remove_file(&pdev->dev, &dev_attr_ois_reg);
	device_remove_file(&pdev->dev, &dev_attr_ois_timestamps_position_data);
	device_remove_file(&pdev->dev, &dev_attr_ois_gain_set);
	device_remove_file(&pdev->dev, &dev_attr_ois_init_before_sr_test);
	CAM_ERR(CAM_OIS, " device_remove_file node");
	
	CAM_INFO(CAM_OIS, "platform driver remove invoked");
	soc_info = &o_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	kfree(o_ctrl->soc_info.soc_private);
	kfree(o_ctrl->io_master_info.cci_client);
	platform_set_drvdata(pdev, NULL);
	v4l2_set_subdevdata(&o_ctrl->v4l2_dev_str.sd, NULL);
	kfree(o_ctrl);

	return 0;
}

static const struct of_device_id cam_ois_dt_match[] = {
	{ .compatible = "qcom,ois" },
	{ }
};


MODULE_DEVICE_TABLE(of, cam_ois_dt_match);

static struct platform_driver cam_ois_platform_driver = {
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = cam_ois_dt_match,
	},
	.probe = cam_ois_platform_driver_probe,
	.remove = cam_ois_platform_driver_remove,
};
static const struct i2c_device_id cam_ois_i2c_id[] = {
	{ "msm_ois", (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver cam_ois_i2c_driver = {
	.id_table = cam_ois_i2c_id,
	.probe  = cam_ois_i2c_driver_probe,
	.remove = cam_ois_i2c_driver_remove,
	.driver = {
		.name = "msm_ois",
	},
};

static struct cam_ois_registered_driver_t registered_driver = {
	0, 0};

static int __init cam_ois_driver_init(void)
{
	int rc = 0;

	rc = platform_driver_register(&cam_ois_platform_driver);
	if (rc) {
		CAM_ERR(CAM_OIS, "platform_driver_register failed rc = %d",
			rc);
		return rc;
	}

	registered_driver.platform_driver = 1;

	rc = i2c_add_driver(&cam_ois_i2c_driver);
	if (rc) {
		CAM_ERR(CAM_OIS, "i2c_add_driver failed rc = %d", rc);
		return rc;
	}

	registered_driver.i2c_driver = 1;
	return rc;
}

static void __exit cam_ois_driver_exit(void)
{
	if (registered_driver.platform_driver)
		platform_driver_unregister(&cam_ois_platform_driver);

	if (registered_driver.i2c_driver)
		i2c_del_driver(&cam_ois_i2c_driver);
}

module_init(cam_ois_driver_init);
module_exit(cam_ois_driver_exit);
MODULE_DESCRIPTION("CAM OIS driver");
MODULE_LICENSE("GPL v2");
