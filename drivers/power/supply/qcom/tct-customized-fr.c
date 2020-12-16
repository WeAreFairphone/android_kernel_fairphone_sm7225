#define pr_fmt(fmt) "[SMBLIB5]: " fmt

#include <linux/init.h>
#include <linux/power_supply.h>
#include "smb5-lib.h"
#if defined(CONFIG_DRM)
#include <linux/msm_drm_notify.h>
#if defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#endif
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/platform_device.h>
#include <linux/iio/consumer.h>

#define smblib_err(chg, fmt, ...)		\
	pr_err_ratelimited("%s: " fmt, \
		__func__, ##__VA_ARGS__)
#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_err_ratelimited("%s: " fmt, \
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug_ratelimited("%s: " fmt, \
				__func__, ##__VA_ARGS__);	\
	} while (0)

/************** battery current thermal for screen on or off ******************/
/* smb5-lib.h qpnp-smb5.c add some patch */
#if defined(CONFIG_DRM)
#if defined(CONFIG_DRM_PANEL)
static int smblib_check_panel_dt(struct smb_charger *chg)
{
	int i;
	int count;
	struct device_node *node, *np;
	struct drm_panel *panel;

	np = of_find_compatible_node(NULL, NULL, "qcom,msm-notifier");
	if (!np) {
		smblib_err(chg, "failed to find qcom,msm-notifier node\n");
		return -ENODEV;
	}
	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			chg->active_panel = panel;
			return 0;
		}
	}

	return -ENODEV;
}
#endif
static int smblib_drm_notifier_callback(struct notifier_block *self,
										unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank = NULL;
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(self, struct smb_charger, fb_nb);

	if (!evdata) {
		smblib_err(chg, "evdata is null\n");
		return 0;
	}

	if (event != MSM_DRM_EVENT_BLANK) {
		smblib_dbg(chg, PR_MISC, "DRM event(%lu) do not need process\n", event);
		return 0;
	}

	if (evdata->data && event == MSM_DRM_EVENT_BLANK && chg) {
		blank = evdata->data;
		pval.intval = chg->system_temp_level;
		switch (*blank) {
		case MSM_DRM_BLANK_UNBLANK:
			chg->is_screen_on = true;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		case MSM_DRM_BLANK_POWERDOWN:
			chg->is_screen_on = false;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		default:
			smblib_dbg(chg, PR_MISC, "DRM blank(%d) do not need process\n", *blank);
			break;
		}
	}

	return 0;

}

#elif defined(CONFIG_FB)
static int smblib_fb_notifier_callback(struct notifier_block *self,
										unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(self, struct smb_charger, fb_nb);

	if (!evdata) {
		smblib_err(chg, "evdata is null\n");
		return 0;
	}

	if (event != FB_EVENT_BLANK) {
		smblib_dbg(chg, PR_MISC, "FB event(%lu) do not need process\n", event);
		return 0;
	}

	if (evdata->data && event == FB_EVENT_BLANK && chg) {
		blank = evdata->data;
		pval.intval = chg->system_temp_level;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			chg->is_screen_on = true;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		case FB_BLANK_POWERDOWN:
			chg->is_screen_on = false;
			smblib_set_prop_system_temp_level(chg, &pval);
			break;
		default:
			smblib_dbg(chg, PR_MISC, "FB blank(%d) do not need process\n", *blank);
			break;
		}
	}

	return 0;
}
#endif

static int __init tct_thermal_init(void)
{
	struct power_supply	*batt_psy = NULL;
	struct smb_charger *chg = NULL;
	int rc = 0;
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		return -ENODEV;

	chg = power_supply_get_drvdata(batt_psy);
	if (chg) {
		chg->is_screen_on = true;
#if defined(CONFIG_DRM)
		chg->fb_nb.notifier_call = smblib_drm_notifier_callback;
#if defined(CONFIG_DRM_PANEL)
		smblib_check_panel_dt(chg);
		if (chg->active_panel) {
			smblib_err(chg, "find active_panel\n");
			rc = drm_panel_notifier_register(chg->active_panel, &chg->fb_nb);
			if (rc < 0) {
				smblib_err(chg, "Couldn't register DRM_PANEL notifier rc = %d\n", rc);
				goto err_notifier_register;
			}
		} else {
			smblib_err(chg, "Couldn't find active_panel\n");
			goto err_notifier_register;
		}
#else
		rc = msm_drm_register_client(&chg->fb_nb);
		if (rc < 0) {
			smblib_err(chg, "Couldn't register DRM notifier rc = %d\n", rc);
			goto err_notifier_register;
		}
#endif
#elif defined(CONFIG_FB)
		chg->fb_nb.notifier_call = smblib_fb_notifier_callback;
		rc = fb_register_client(&chg->fb_nb);
		if (rc < 0) {
			smblib_err(chg, "Couldn't register FB notifier rc = %d\n", rc);
			goto err_notifier_register;
		}
#endif
	} else {
		pr_err("[%s]chg is null\n", __func__);
		return -ENODATA;
	}

	return 0;
err_notifier_register:
	chg->is_screen_on = false;
	return rc;
}
late_initcall(tct_thermal_init);

/************ print charger and battery log for hw test every 10s *************/
static int g_log_en = 0;
static unsigned int g_alarm_time = 10;
static struct delayed_work print_tct_charge_worker;
static struct alarm fg_log_thread_alarm;
static struct smb_charger *g_qcom_chg;

static const char * const power_supply_status_text[] = {
	"Unknown", "Charging", "Discharging", "Not charging", "Full"
};

static const char * const power_supply_charge_type_text[] = {
	"Unknown", "N/A", "Trickle", "Fast", "Taper"
};

static const char * const power_supply_health_text[] = {
	"Unknown", "Good", "Overheat", "Dead", "Over voltage",
	"Unspecified failure", "Cold", "Watchdog timer expire",
	"Safety timer expire",
	"Warm", "Cool", "Hot"
};

void print_log_for_fg(void)
{
	union power_supply_propval Vbat = {0,}, Ibat = {0,}, batt_temp = {0,}, soc = {0,};
	union power_supply_propval Vbus = {0,}, status = {0,}, health = {0,}, Vph = {0,}, thermal_icl = {0,};
	union power_supply_propval charging_type = {0,}, Ibus = {0,}, Isns = {0,};
	union power_supply_propval cp_temp = {0,};
	int v_int_ext = 0, i_int_ext = 0;

	if (g_qcom_chg->batt_psy) {
		power_supply_get_property(g_qcom_chg->batt_psy, POWER_SUPPLY_PROP_STATUS, &status);
		power_supply_get_property(g_qcom_chg->batt_psy, POWER_SUPPLY_PROP_HEALTH, &health);
		power_supply_get_property(g_qcom_chg->batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &Vbat);
		power_supply_get_property(g_qcom_chg->batt_psy, POWER_SUPPLY_PROP_CAPACITY, &soc);
		power_supply_get_property(g_qcom_chg->batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &Ibat);
		power_supply_get_property(g_qcom_chg->batt_psy, POWER_SUPPLY_PROP_TEMP, &batt_temp);
		power_supply_get_property(g_qcom_chg->batt_psy, POWER_SUPPLY_PROP_CHARGE_TYPE, &charging_type);
	}

	if (g_qcom_chg->usb_psy) {
		power_supply_get_property(g_qcom_chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &Vbus);
		power_supply_get_property(g_qcom_chg->usb_psy, POWER_SUPPLY_PROP_INPUT_CURRENT_NOW, &Ibus);
		power_supply_get_property(g_qcom_chg->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_VPH, &Vph);
		power_supply_get_property(g_qcom_chg->usb_psy, POWER_SUPPLY_PROP_THERM_ICL_LIMIT, &thermal_icl);
	}

	if (g_qcom_chg->cp_psy) {
		power_supply_get_property(g_qcom_chg->cp_psy, POWER_SUPPLY_PROP_CP_ISNS, &Isns);
		power_supply_get_property(g_qcom_chg->cp_psy, POWER_SUPPLY_PROP_CP_DIE_TEMP, &cp_temp);
	}

	if (g_qcom_chg->iio.v_i_int_ext_chan) {
		iio_read_channel_processed_two(g_qcom_chg->iio.v_i_int_ext_chan, &v_int_ext, &i_int_ext);
	}

	printk(KERN_ERR "%s: status:%s,health:%s,voltage:%dmV,capacity:%d,current:%dmA,temperature:%d,chgvoltage:%dmV,charging_type:%s,Ibus:%d,Thermal_icl:%d,VPH:%d,Isns:%d,cp_temp:%d,die_temp:%d,connector_temp:%d,skin_temp=%d,I_pmic=%d,I_smb=%d\n",
			__func__, power_supply_status_text[status.intval],
			power_supply_health_text[health.intval],
			Vbat.intval/1000, soc.intval, Ibat.intval/1000, batt_temp.intval,
			Vbus.intval/1000, power_supply_charge_type_text[charging_type.intval],
			Ibus.intval/1000, thermal_icl.intval/1000, Vph.intval/1000, Isns.intval/1000,
			cp_temp.intval, g_qcom_chg->die_temp, g_qcom_chg->connector_temp, g_qcom_chg->skin_temp,
			-i_int_ext/1000, Ibat.intval/1000 + i_int_ext/1000);
}

static void print_tct_charge_work(struct work_struct *work)
{
	print_log_for_fg();
}

static ssize_t enable_show(struct device *dev, struct device_attribute
							*attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_log_en);
}

static ssize_t enable_store(struct device *dev, struct device_attribute
							*attr, const char *buf, size_t count)
{
	int val;
	if (kstrtos32(buf, 0, &val))
		return -EINVAL;

	g_log_en = val;
	if (g_log_en)
		alarm_start_relative(&fg_log_thread_alarm,
			ns_to_ktime((unsigned long long)(g_alarm_time) * NSEC_PER_SEC));
	else
		alarm_cancel(&fg_log_thread_alarm);

	return count;
}
static DEVICE_ATTR_RW(enable);

static ssize_t alarm_time_show(struct device *dev, struct device_attribute
									*attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_alarm_time);
}

static ssize_t alarm_time_store(struct device *dev, struct device_attribute
									*attr, const char *buf, size_t count)
{
	int val;
	if (kstrtos32(buf, 0, &val))
		return -EINVAL;

	g_alarm_time = val;

	return count;
}
static DEVICE_ATTR_RW(alarm_time);

static struct attribute *log_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_alarm_time.attr,
	NULL,
};
ATTRIBUTE_GROUPS(log);

static enum alarmtimer_restart fg_log_thread_func(struct alarm *alarm, ktime_t now)
{
	queue_delayed_work(private_chg_wq, &print_tct_charge_worker, 0);
	alarm_forward_now(&fg_log_thread_alarm,
		ns_to_ktime((unsigned long long)(g_alarm_time) * NSEC_PER_SEC));
	return ALARMTIMER_RESTART;
}

static void fg_log_thread_init(void)
{
	alarm_init(&fg_log_thread_alarm, ALARM_REALTIME, fg_log_thread_func);
}

static int __init tct_charger_battery_log_init(void)
{
	int rc = 0;
	struct platform_device *log_pdev = NULL;
	struct power_supply	*batt_psy = NULL;
	struct smb_charger *chg = NULL;
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy)
		return -ENODEV;
	chg = power_supply_get_drvdata(batt_psy);
	if (!chg) {
		pr_err("[%s]chg is null\n", __func__);
		return -ENODATA;
	}
	g_qcom_chg = chg;

	log_pdev = platform_device_register_data(NULL, "tct_charge_log",
											PLATFORM_DEVID_NONE, chg, sizeof(*chg));
	if (IS_ERR(log_pdev)) {
		pr_err("Failed to register tct_charge_log platform device\n");
		rc = PTR_ERR(log_pdev);
		goto err_pdev_register;
	}

	rc = sysfs_create_groups(&log_pdev->dev.kobj, log_groups);
	if (rc < 0) {
		pr_err("Couldn't create sysfs files rc=%d\n", rc);
		goto err_group;
	}

	fg_log_thread_init();

	INIT_DELAYED_WORK(&print_tct_charge_worker, print_tct_charge_work);
	if (g_log_en)
		alarm_start_relative(&fg_log_thread_alarm,
			ns_to_ktime((unsigned long long)(g_alarm_time) * NSEC_PER_SEC));

	return 0;
err_group:
	platform_device_unregister(log_pdev);
err_pdev_register:
	return rc;
}
late_initcall(tct_charger_battery_log_init);
