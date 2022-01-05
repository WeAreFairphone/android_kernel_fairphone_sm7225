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


extern uint64_t time_stamp_val_for_cm401;
static struct cam_ois_dev g_cam_ois_dev;

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

DEVICE_ATTR(ois_gyro_cali_data, 0664, ois_gyro_cali_data_show, ois_gyro_cali_data_store);
DEVICE_ATTR(ois_status, 0664, ois_status_show, ois_status_store);
DEVICE_ATTR(ois_reg, 0664, ois_reg_show, ois_reg_store);
DEVICE_ATTR(ois_init_before_sr_test, 0664, ois_init_before_sr_test_show, ois_init_before_sr_test_store);
DEVICE_ATTR(ois_gain_set, 0664, ois_gain_set_show, ois_gain_set_store);


static int cml_vsync_pinctrl_init(struct cam_ois_ctrl_t *o_ctrl)
{
	int retval = 0;
	struct cam_ois_dev *pois_dev = o_ctrl->pois_dev;

	/* Get pinctrl if target uses pinctrl */
	pois_dev->vsync_pinctrl = devm_pinctrl_get((o_ctrl->pdev->dev.parent));
	if (IS_ERR_OR_NULL(pois_dev->vsync_pinctrl)) {
		retval = PTR_ERR(pois_dev->vsync_pinctrl);
		CAM_ERR(CAM_OIS, "Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	pois_dev->pin_default = pinctrl_lookup_state(pois_dev->vsync_pinctrl, "default");
	if (IS_ERR_OR_NULL(pois_dev->pin_default)) {
		retval = PTR_ERR(pois_dev->pin_default);
		CAM_ERR(CAM_OIS, "Failed to look up default state\n");
		goto err_pinctrl_lookup;
	}

	return retval;

err_pinctrl_lookup:
	devm_pinctrl_put(pois_dev->vsync_pinctrl);
err_pinctrl_get:
	pois_dev->vsync_pinctrl = NULL;
	return retval;
}

irqreturn_t interrupt_ois_vsync_irq(int irq, void *dev)
{
	struct cam_ois_dev             *pois_dev = (struct cam_ois_dev *)dev;

	if (pois_dev->dev_state == CAM_OIS_DEV_START)
	{
		pois_dev->need_read = 1;
		wake_up_interruptible(&pois_dev->queue);
	}

	return IRQ_HANDLED;
}

enum SETTING_INDEX {
	SETTING_ADDR,
	SETTING_DATA,
	SETTING_DELAY,
	SETTING_MASK,
};

static struct cam_sensor_i2c_reg_array ois_data_reader_setting[] =
{
	{0x9DFC, 0x0A04, 1, 0},
	{0x9B2C, 0x0001, 1, 0},
	{0x9B2A, 0x0001, 1, 0},
	{0x9DFE, 0x0000, 1, 0},
};

static int regDataOffset[] =
{
	6, 22, 38, 54, 70, 86, 102, 118, 134, 150
};

static struct cam_sensor_i2c_reg_array ois_data_reader_setting_for_buffer1[] =
{
	{0x9DFE, 0x0001, 1, 0},
};

static int32_t cam_ois_reg_setting_init(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t rc = 0, i = 0, item_num = 0;
	struct cam_sensor_i2c_reg_array *p_reg_array = NULL;
	struct cam_ois_dev *pois_dev = o_ctrl->pois_dev;
	pois_dev->reg_page_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
					(sizeof(ois_data_reader_setting) /
					sizeof(ois_data_reader_setting[0]))) >> PAGE_SHIFT;
	pois_dev->reg_setting_page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
						pois_dev->reg_page_size, 0, GFP_KERNEL);
	if (!pois_dev->reg_setting_page) {
		rc = -ENOMEM;
	}

	p_reg_array = (struct cam_sensor_i2c_reg_array *) (page_address(pois_dev->reg_setting_page));
	item_num = sizeof(ois_data_reader_setting) / sizeof(ois_data_reader_setting[0]);

	for (i = 0; i < item_num; i++) {
		p_reg_array[i].reg_addr = ois_data_reader_setting[i].reg_addr;
		p_reg_array[i].reg_data = ois_data_reader_setting[i].reg_data;
		p_reg_array[i].delay = ois_data_reader_setting[i].delay;
		p_reg_array[i].data_mask = ois_data_reader_setting[i].data_mask;
	}
	pois_dev->i2c_reg_setting.reg_setting = p_reg_array;
	pois_dev->i2c_reg_setting.addr_type   = CAMERA_SENSOR_I2C_TYPE_WORD;
	pois_dev->i2c_reg_setting.data_type   = CAMERA_SENSOR_I2C_TYPE_WORD;
	pois_dev->i2c_reg_setting.size        = item_num;
	pois_dev->i2c_reg_setting.delay       = 0;

	pois_dev->i2c_reg_setting_for_buffer0.reg_setting = ois_data_reader_setting_for_buffer1;
	pois_dev->i2c_reg_setting_for_buffer0.addr_type   = CAMERA_SENSOR_I2C_TYPE_WORD;
	pois_dev->i2c_reg_setting_for_buffer0.data_type   = CAMERA_SENSOR_I2C_TYPE_WORD;
	pois_dev->i2c_reg_setting_for_buffer0.size        = 1;
	pois_dev->i2c_reg_setting_for_buffer0.delay       = 0;

	return rc;
}

static void cam_ois_reg_setting_uninit(struct cam_ois_ctrl_t *o_ctrl)
{
	struct cam_ois_dev *pois_dev = o_ctrl->pois_dev;
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		pois_dev->reg_setting_page,
		pois_dev->reg_page_size);
	pois_dev->reg_setting_page = NULL;
	pois_dev->reg_page_size    = 0;
}

static int32_t reverse_byte(struct cam_ois_dev *pois_dev)
{
	int32_t rc = 0,i=0;
	struct cam_ois_ctrl_t *o_ctrl = pois_dev->o_ctrl;
	uint8_t tmp_data=0;

	if (!pois_dev || !o_ctrl) {
		CAM_ERR(CAM_OIS, "read failed: pois_dev: %p o_ctrl:%p", pois_dev, o_ctrl);
		return -EINVAL;
	}

	for (i = 0;i<160;i+=2)
	{
		tmp_data = pois_dev->reg_data_buffer[i];
		pois_dev->reg_data_buffer[i] = pois_dev->reg_data_buffer[i+1];
		pois_dev->reg_data_buffer[i+1] = tmp_data;
	}
	return rc;
}

static int32_t load_ois_data(struct cam_ois_dev *pois_dev)
{
	int32_t rc = 0;
	struct cam_ois_ctrl_t *o_ctrl = pois_dev->o_ctrl;
	struct timespec64 ts;

	if (!pois_dev || !o_ctrl) {
		CAM_ERR(CAM_OIS, "read failed: pois_dev: %p o_ctrl:%p", pois_dev, o_ctrl);
		return -EINVAL;
	}
	mutex_lock(&(o_ctrl->ois_mutex));
	camera_io_dev_write_continuous(&o_ctrl->io_master_info, &pois_dev->i2c_reg_setting, 0);
	mdelay(1); /* for safe read data */
	get_monotonic_boottime64(&ts);
	pois_dev->load_ois_timestamp = time_stamp_val_for_cm401;
	camera_io_dev_read_seq(&(o_ctrl->io_master_info),0x9DAC, pois_dev->reg_data_buffer,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD,80);
	camera_io_dev_write(&o_ctrl->io_master_info,&pois_dev->i2c_reg_setting_for_buffer0);
	mdelay(1); /* for safe read data */
	camera_io_dev_read_seq(&(o_ctrl->io_master_info),0x9DAC, &pois_dev->reg_data_buffer[80],CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD,80);
	reverse_byte(pois_dev);
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
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
    struct cam_ois_dev             *pois_dev = NULL;

	o_ctrl = kzalloc(sizeof(struct cam_ois_ctrl_t), GFP_KERNEL);
	if (!o_ctrl)
		return -ENOMEM;

	o_ctrl->soc_info.pdev = pdev;
	o_ctrl->pdev = pdev;
	o_ctrl->pois_dev = &g_cam_ois_dev;
	pois_dev         = o_ctrl->pois_dev;
	pois_dev->o_ctrl = o_ctrl;
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

	/*init ois data reader configuration*/
	rc = cam_ois_reg_setting_init(o_ctrl);
	if (rc) {
		goto free_soc;
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
		goto free_reg_array;
	}

	rc = cml_vsync_pinctrl_init(o_ctrl);
	if (!rc && pois_dev->vsync_pinctrl) {
		rc = pinctrl_select_state(
				pois_dev->vsync_pinctrl,
				pois_dev->pin_default);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Can't select pinctrl state\n");
		}
	}

	rc = request_irq(gpio_to_irq(pois_dev->irq_gpio), interrupt_ois_vsync_irq, IRQ_TYPE_EDGE_RISING , "ois_vsync_intf", pois_dev);
	if(rc < 0)
	{
		CAM_ERR(CAM_OIS, "request_irq failed\n");
		goto exit_free_gpio;
	}
	enable_irq_wake(gpio_to_irq(pois_dev->irq_gpio));

	rc = cam_ois_init_subdev_param(o_ctrl);
	if (rc)
		goto free_reg_array;

	if ((device_create_file(&pdev->dev, &dev_attr_ois_gyro_cali_data)) ||
		(device_create_file(&pdev->dev, &dev_attr_ois_status))			||
		(device_create_file(&pdev->dev, &dev_attr_ois_reg))				||
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
	return rc;

exit_free_gpio:
	gpio_free(pois_dev->irq_gpio);
unreg_subdev:
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));
free_soc:
	kfree(soc_private);
free_reg_array:
    cam_ois_reg_setting_uninit(o_ctrl);
free_cci_client:
	kfree(o_ctrl->io_master_info.cci_client);
free_o_ctrl:
	kfree(o_ctrl);
    pois_dev->o_ctrl = NULL;
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
	device_remove_file(&pdev->dev, &dev_attr_ois_reg);
	device_remove_file(&pdev->dev, &dev_attr_ois_gain_set);
	device_remove_file(&pdev->dev, &dev_attr_ois_init_before_sr_test);
	CAM_ERR(CAM_OIS, " device_remove_file node");

	cam_ois_reg_setting_uninit(o_ctrl);

	CAM_INFO(CAM_OIS, "platform driver remove invoked");
	soc_info = &o_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);
	o_ctrl->pois_dev->o_ctrl = NULL;
	o_ctrl->pois_dev = NULL;

	mutex_lock(&(o_ctrl->ois_mutex));
	cam_ois_shutdown(o_ctrl);
	mutex_unlock(&(o_ctrl->ois_mutex));
	cam_unregister_subdev(&(o_ctrl->v4l2_dev_str));

	soc_private =(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
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

static int c_ois_open(struct inode *inode,struct file *filp)
{
	struct cam_ois_dev *pois_dev = &g_cam_ois_dev;
	mutex_lock(&pois_dev->dev_mutex);
	filp->private_data = (void *)pois_dev;
	if (CAM_OIS_DEV_UNINIT != g_cam_ois_dev.dev_state)
		g_cam_ois_dev.dev_state = CAM_OIS_DEV_START;
	mutex_unlock(&pois_dev->dev_mutex);
	return 0;
}

static int c_ois_release(struct inode *inode,struct file *filp)
{
	struct cam_ois_dev *pois_dev = (struct cam_ois_dev *)filp->private_data;
	mutex_lock(&pois_dev->dev_mutex);
	if (CAM_OIS_DEV_UNINIT != g_cam_ois_dev.dev_state)
		g_cam_ois_dev.dev_state = CAM_OIS_DEV_STOP;
	filp->private_data = NULL;
	mutex_unlock(&pois_dev->dev_mutex);
	return 0;
}

static ssize_t c_ois_read(struct file *filp,char *buf,size_t len,loff_t *off)
{
	struct cam_ois_dev *pois_dev = (struct cam_ois_dev *)filp->private_data;
	int rc = 0, i = 0, data_num = 0;
	if (!pois_dev || len < 28) {
		return -EINVAL;
	}
	if (wait_event_interruptible(pois_dev->queue, pois_dev->need_read != 0))
		return -ERESTARTSYS;
	pois_dev->need_read = 0;

	rc = load_ois_data(pois_dev);
	if (rc) {
		goto error_handle;
	}
	/*OIS data protocol:
	 * |1-byte| 8-byte  |    N-byte      |
	 * |  N   |timestamp|   x,y data     |
	 * */
	data_num = 10; //we just read 5-data now
	if (copy_to_user(&buf[0], (void *)&data_num, 1))
	{
		rc = -EFAULT;
		goto error_handle;
	}
	rc += 1;

	if (copy_to_user(&buf[rc], &pois_dev->load_ois_timestamp, sizeof(uint64_t))) {
		rc = -EFAULT;
		goto error_handle;
	}
	rc += sizeof(uint64_t);

	for (i = 0; i < data_num; i++)
	{
		if (copy_to_user(&buf[rc], (void *)(&pois_dev->reg_data_buffer[regDataOffset[i]]), sizeof(uint32_t))) {
			rc = -EFAULT;
			goto error_handle;
		}
		rc += sizeof(uint32_t);
	}

error_handle:
	return rc;
}

static ssize_t c_ois_write(struct file *filp,const char *buf,size_t len,loff_t *off)
{
	return 0;
}

static __poll_t c_ois_poll(struct file *filp, poll_table *wait)
{
	struct cam_ois_dev *pois_dev = (struct cam_ois_dev *)filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &pois_dev->queue, wait);
	if (pois_dev->need_read) {
		mask |= EPOLLIN;
	}

	return mask;
}
static const struct file_operations cois_fops =
{
	.read = c_ois_read,
	.write = c_ois_write,
	.open = c_ois_open,
	.release = c_ois_release,
	.poll = c_ois_poll,
};

static int __init cam_ois_driver_init(void)
{
	int rc = 0;
	dev_t ois_dev = 0;
	memset(&g_cam_ois_dev, 0, sizeof(g_cam_ois_dev));

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

	ois_dev = MKDEV(g_cam_ois_dev.ois_major, 0);
#define OISDEV_NAME "OISCDEV"
	if (g_cam_ois_dev.ois_major) {
		rc = register_chrdev_region(ois_dev, 1, OISDEV_NAME);
	} else {
		rc = alloc_chrdev_region(&ois_dev, 0, 1, OISDEV_NAME);
		g_cam_ois_dev.ois_major = MAJOR(ois_dev);
	}

	if (rc < 0) {
		return rc;
	}

	cdev_init(&g_cam_ois_dev.devm, &cois_fops);
	g_cam_ois_dev.devm.owner = THIS_MODULE;
	rc = cdev_add(&g_cam_ois_dev.devm, ois_dev, 1);
	if (rc) {
		CAM_ERR(CAM_OIS, "Error %d adding cm4_ois0 device", rc);
	} else {
		g_cam_ois_dev.ois_class = class_create(THIS_MODULE, "cm4_ois0");
		device_create(g_cam_ois_dev.ois_class, NULL, ois_dev, NULL, "cm4_ois0");
	}

	mutex_init(&(g_cam_ois_dev.dev_mutex));
	init_waitqueue_head(&g_cam_ois_dev.queue);
	g_cam_ois_dev.reg_data_buffer = (uint8_t *)vzalloc(160 * sizeof(uint8_t));
	g_cam_ois_dev.dev_state = CAM_OIS_DEV_INIT;
	return rc;
}

static void __exit cam_ois_driver_exit(void)
{
	if (g_cam_ois_dev.reg_data_buffer)
	{
		vfree(g_cam_ois_dev.reg_data_buffer);
		g_cam_ois_dev.reg_data_buffer = NULL;
	}
	device_destroy(g_cam_ois_dev.ois_class, MKDEV(g_cam_ois_dev.ois_major, 0));
	class_destroy(g_cam_ois_dev.ois_class);
	cdev_del(&g_cam_ois_dev.devm);
	unregister_chrdev_region(MKDEV(g_cam_ois_dev.ois_major, 0), 1);
	if (registered_driver.platform_driver)
		platform_driver_unregister(&cam_ois_platform_driver);

	if (registered_driver.i2c_driver)
		i2c_del_driver(&cam_ois_i2c_driver);
}

module_init(cam_ois_driver_init);
module_exit(cam_ois_driver_exit);
MODULE_DESCRIPTION("CAM OIS driver");
MODULE_LICENSE("GPL v2");
