/* Copyright (C) 2019 Tcl Corporation Limited */
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/err.h>


static int board_id0_gpio=-1;
static int board_id1_gpio=-1;
static int board_id2_gpio=-1;
static char hw_version_status =-1;


static ssize_t hw_version_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	int id0=-1,id1=-1,id2=-1;

	id0=gpio_get_value(board_id0_gpio);
	pr_err(" %s board_id0 return is=%d ", __func__,id0);
	id1=gpio_get_value_cansleep(board_id1_gpio);
	pr_err(" %s board_id1 return is=%d ", __func__,id1);
	id2=gpio_get_value_cansleep(board_id2_gpio);
	pr_err(" %s board_id2 return is=%d ", __func__,id2);

	hw_version_status =
				(id0 << 0) |
				(id1 << 1) |
				(id2 << 2);


	return sprintf(buf, "%u\n", hw_version_status);
}


int hw_version_get(void)
{
	int id0=-1,id1=-1,id2=-1;
	int hw_version=-1;

	id0=gpio_get_value(board_id0_gpio);
	pr_err("board_id0 return is=%d ", id0);
	id1=gpio_get_value_cansleep(board_id1_gpio);
	pr_err("board_id1 return is=%d ", id1);
	id2=gpio_get_value_cansleep(board_id2_gpio);
	pr_err("board_id2 return is=%d ", id2);

	hw_version =(id0 << 0) |(id1 << 1) |(id2 << 2);
	pr_err("hw_version =%x", hw_version);

	return hw_version;
}



static struct class_attribute board_detect_value =
	__ATTR(version, 0665, hw_version_show, NULL);


static int board_detect_creat_file(void)
{
	int ret;
	struct class *board_detect_class;

	/* board_detect create (/<sysfs>/class/board_id) */
	board_detect_class = class_create(THIS_MODULE, "board_id");
	if (IS_ERR(board_detect_class)) {
		ret = PTR_ERR(board_detect_class);
		printk(KERN_ERR "board_detect_class : couldn't create info\n");
	}
	if (class_create_file(board_detect_class, &board_detect_value)	) {

		printk(KERN_ERR " couldn't create version\n");
		}
	 pr_err("%s  success \n",__func__);

	return 0;

}


static int board_detect_probe (struct platform_device * pdev)
{

	int rc = 0;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;

	pr_err("%s start \n",__func__);
	board_id0_gpio=of_get_named_gpio(pdev->dev.of_node,"qcom,board-id0-gpio", 0);
	rc = gpio_request(board_id0_gpio, "qcom,board-id0-gpio");
	if (rc) {
				pr_err("  board_id0_gpio request gpio=%d failed, rc=%d\n", board_id0_gpio,rc);
				goto err_board_id0_gpio;
			}

	board_id1_gpio=of_get_named_gpio(pdev->dev.of_node,"qcom,board-id1-gpio", 0);
	rc = gpio_request(board_id1_gpio, "qcom,board-id1-gpio");
	if (rc) {
				pr_err("  board_id1_gpio request gpio=%d failed, rc=%d\n", board_id1_gpio,rc);
				goto err_board_id1_gpio;
			}

	board_id2_gpio=of_get_named_gpio(pdev->dev.of_node,"qcom,board-id2-gpio", 0);
	rc = gpio_request(board_id2_gpio, "qcom,board-id2-gpio");
	if (rc) {
				pr_err("  board_id2_gpio request gpio=%d failed, rc=%d\n", board_id2_gpio,rc);
				goto err_board_id2_gpio;;
			}

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		pr_err("board_detect: Failed to get pinctrl\n");
		return PTR_ERR(pinctrl);
	}
	pin_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR_OR_NULL(pin_default)) {
		pr_err("board_detect: Failed to look up default state\n");
		return PTR_ERR(pinctrl);
	}
	rc = pinctrl_select_state(pinctrl, pin_default);
	if (rc) {
		pr_err("board_detect: Can't select pinctrl state\n");
		return rc;
	}
	pr_err("%s---id0_gpio=%d---id1_gpio=%d---id2_gpio=%d----\n",__func__,board_id0_gpio,board_id1_gpio,board_id2_gpio);

	rc = gpio_direction_input(board_id0_gpio);
	rc = gpio_direction_input(board_id1_gpio);
	rc = gpio_direction_input(board_id2_gpio);

	rc=board_detect_creat_file();

	pr_err("%s finished \n",__func__);

	return rc;

err_board_id0_gpio:
    gpio_free(board_id0_gpio);
	return	-ENODEV;

err_board_id1_gpio:
    gpio_free(board_id1_gpio);
	return	-ENODEV;

err_board_id2_gpio:
    gpio_free(board_id2_gpio);
	return	-ENODEV;
}



static int board_detect_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id board_match_table[] = {
	{ .compatible = "qcom,fp4-board-id",},
	{ },
};

static struct platform_driver board_detect_driver = {
	.probe = board_detect_probe,
	.remove = board_detect_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "board_id_detect",
		.of_match_table = board_match_table,
	},
};

static int __init board_detect_driver_init(void)
{
	int err;

	pr_err("board detect init start \n");
	err = platform_driver_register(&board_detect_driver);
	if (err) {
		pr_err("platform_driver_register() failed! (err=%d)", err);
		return err;
	}

	return 0;
}

static void __exit board_detect_driver_exit(void)
{
	platform_driver_unregister(&board_detect_driver);
}

module_init(board_detect_driver_init);
module_exit(board_detect_driver_exit);
MODULE_LICENSE("GPL v2");
