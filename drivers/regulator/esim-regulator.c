#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

struct regulator *regulator_l5e = NULL;
const char *regulator_name = "esim_regulator";


static int regulator_status = 0;


static ssize_t esim_power_store(struct class *class,
				struct class_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	if ((!strncmp(buf,"1",1))) {
	    ret = regulator_set_voltage(regulator_l5e, 1810000, 1810000); 
		   regulator_status = 1;
		}
	else {
	    ret = regulator_set_voltage(regulator_l5e, 0, 0); 
		   regulator_status = 0;
		}

	return count;

}
static ssize_t esim_power_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", regulator_status);
}





static struct class_attribute esim_regulator_power =
	__ATTR(esim_power, 0665, esim_power_show, esim_power_store);






static int esim_regulator_creat_file(void)
{
	int ret;
	struct class *esim_regulator_class;

	/* esim_regulator create (/<sysfs>/class/esim_regulator) */
	esim_regulator_class = class_create(THIS_MODULE, "esim_regulator");
	if (IS_ERR(esim_regulator_class)) {
		ret = PTR_ERR(esim_regulator_class);
		printk(KERN_ERR "esim_regulator_class: couldn't create esim_regulator\n");
	}
	if (class_create_file(esim_regulator_class, &esim_regulator_power)) {

			pr_err("%s: esim_regulator_class: couldn't create sub file node\n", __func__);
		}

	return 0;

}


static int esim_regulator_probe(struct platform_device *pdev)
{
	int ret = 0;

	regulator_l5e = regulator_get(&pdev->dev, regulator_name);
		if (IS_ERR(regulator_l5e)) {
			printk("regulator_get regulator fail\n");
			//return -EINVAL;
		}
		else {
	    	ret = regulator_set_voltage(regulator_l5e, 1810000, 1810000); 
	    		if (ret)
	    		{
	        		printk("Could not set to 1.81v.\n");
	    		}
		}

	ret = esim_regulator_creat_file();

	return ret;

}


static int esim_regulator_remove(struct platform_device *pdev)
{


	return 0;
}


static const struct of_device_id esim_regulator_dt_match[] = {
	{.compatible = "qcom,esim-power"},
	{}
};

MODULE_DEVICE_TABLE(of, esim_regulator_dt_match);

static struct platform_driver esim_regulator_driver = {
	.probe = esim_regulator_probe,
	.remove = esim_regulator_remove,
	.shutdown = NULL,
	.driver = {
		.name = "esim-power",
		.of_match_table = esim_regulator_dt_match,
	},
};

static int esim_regulator_register_driver(void)
{
	return platform_driver_register(&esim_regulator_driver);
}



static int __init esim_regulator_init(void)
{

	int ret = 0;

	ret = esim_regulator_register_driver();
	if (ret) {
		pr_err("esim_regulator_driver() failed!\n");
		return ret;
	}

	return ret;

}

module_init(esim_regulator_init);

static void __exit esim_regulator_exit(void)
{
}
module_exit(esim_regulator_exit);

MODULE_DESCRIPTION("Get esim power");
MODULE_LICENSE("GPL v2");

