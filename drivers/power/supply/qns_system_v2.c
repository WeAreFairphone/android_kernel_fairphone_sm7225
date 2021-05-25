/*
 * qns_system.c version 2.0
 * Qnovo QNS wrapper implementation. Compatible with kernel 4.14.
 * Copyright (C) 2014 Qnovo Corp
 * Miro Zmrzli <miro@qnovocorp.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/alarmtimer.h>

// Set the sign (-1 or 1) so that /sys/class/qns/current_now returns negative 
// values while discharging and positive while charging.
#define READ_CURRENT_SIGN	(1)

#define CHARGE_CURRENT_PROP	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX
#define CHARGE_VOLTAGE_PROP POWER_SUPPLY_PROP_VOLTAGE_MAX

#define IBATMANAME		"battery"

#define QNS_OK		0
#define QNS_ERROR	(-1)

struct qns_data
{
	struct power_supply * battery_psy;
	struct alarm alarm;
	bool alarm_inited;
	int alarm_value;
	struct wakeup_source *wakelock;
	bool wakelock_inited;
	bool wakelock_held;
	struct wakeup_source *charge_wakelock;
	bool charge_wakelock_inited;
	bool charge_wakelock_held;
	int options;
};

static struct qns_data data;

static bool qns_has_psy(void)
{
	if(data.battery_psy == NULL)
	{
		data.battery_psy = power_supply_get_by_name(IBATMANAME);
		if(data.battery_psy == NULL)
		{
			pr_info("QNS: ERROR: unable to get " IBATMANAME);
			return false;
		}
	}
	return true;
}

static int qns_set_ibat(int ibatmA)
{
	union power_supply_propval propVal = {ibatmA * 1000,};

	pr_info("QNS: new charge current:%d mA", ibatmA);

	if(!qns_has_psy())
	{
		pr_info("QNS: ERROR: unable to get " IBATMANAME ". Can't set the current!");
		return QNS_ERROR;
	}

	if(power_supply_set_property(data.battery_psy,
			CHARGE_CURRENT_PROP, &propVal) != 0)
	{
		pr_info("QNS: ERROR: unable to set charging current! Does " IBATMANAME " have "
				"POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT property?");
		return QNS_ERROR;
	}
	return QNS_OK;
}

static int qns_set_vbat(int vbatmV)
{
	union power_supply_propval propVal = {vbatmV * 1000,};

	pr_info("QNS: new charge voltage:%d mV", vbatmV);

	if(!qns_has_psy())
	{
		pr_info("QNS: ERROR: unable to get " IBATMANAME ". Can't set the voltage!");
		return QNS_ERROR;
	}

	if(power_supply_set_property(data.battery_psy,
			CHARGE_VOLTAGE_PROP, &propVal) != 0)
	{
		pr_info("QNS: ERROR: unable to set charging voltage! Does " IBATMANAME " have "
				"POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE property?");
		return QNS_ERROR;
	}
	return QNS_OK;
}

static bool qns_is_charging(void)
{
	union power_supply_propval propVal = {0, };

	if(!qns_has_psy())
	{
		pr_info("QNS: ERROR: unable to get " IBATMANAME ". Can't read charging state!");
		return false;
	}

	if(power_supply_get_property(data.battery_psy, POWER_SUPPLY_PROP_STATUS,
			&propVal) != 0)
	{
		pr_info("QNS: ERROR: unable to read charger properties! Does " IBATMANAME " have "
				"POWER_SUPPLY_PROP_STATUS property?");
		return false;
	}

	return propVal.intval == POWER_SUPPLY_STATUS_CHARGING;
}

static int qns_get_scvt(int *soc, int *c, int *v, int *tx10)
{
	/*
	  	soc in %
		c in ma
		v in mv
		t in 0.1 deg c
	*/
	union power_supply_propval ret = {0,};
	int retVal = QNS_OK;

	if(!qns_has_psy())
	{
		pr_info("QNS: ERROR: unable to get " IBATMANAME ". Can't read soc/c/v/t!");
		retVal = QNS_ERROR;
	}

	if (data.battery_psy)
	{
		if(c != NULL)
		{
			if(power_supply_get_property(data.battery_psy,
					POWER_SUPPLY_PROP_CURRENT_NOW, &ret) != 0)
			{
				pr_info("QNS: ERROR: unable to read battery property POWER_SUPPLY_PROP_CURRENT_NOW");
				*c = 0;
				retVal = QNS_ERROR;
			}
			else
				*c = READ_CURRENT_SIGN * ret.intval / 1000;
		}

		if(v != NULL)
		{
			if(power_supply_get_property(data.battery_psy,
					POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret) != 0)
			{
				pr_info("QNS: ERROR: unable to read battery property POWER_SUPPLY_PROP_VOLTAGE_NOW");
				*v = 0;
				retVal = QNS_ERROR;
			}
			else
				*v = ret.intval / 1000;
		}

		if(tx10 != NULL)
		{
			if(power_supply_get_property(data.battery_psy,
					POWER_SUPPLY_PROP_TEMP, &ret) != 0)
			{
				pr_info("QNS: ERROR: unable to read battery property POWER_SUPPLY_PROP_TEMP");
				*tx10 = 0;
				retVal = QNS_ERROR;
			}
			else
				*tx10 = ret.intval;
		}

		if(soc != NULL)
		{
			if(power_supply_get_property(data.battery_psy,
					POWER_SUPPLY_PROP_CAPACITY, &ret) != 0)
			{
				pr_info("QNS: ERROR: unable to read battery property POWER_SUPPLY_PROP_CAPACITY");
				*soc = 0;
				retVal = QNS_ERROR;
			}
			else
				*soc = ret.intval;
		}
	}
	else
	{
		pr_info("QNS: battery power supply is not registered yet.");
		if(c != NULL) *c = 0;
		if(v != NULL) *v = 4000;
		if(tx10 != NULL) *tx10 = 250;
		if(soc != NULL) *soc = 50;
		retVal = QNS_ERROR;
	}
	return retVal;
}

static int qns_get_fcc(int *fcc, int *design)
{
	union power_supply_propval ret = {0,};
	int retVal = QNS_OK;

	if(!qns_has_psy())
	{
		pr_info("QNS: ERROR: unable to get " IBATMANAME ". Can't read fcc/design!");
		return QNS_ERROR;
	}

	if(fcc != NULL)
	{
		if(power_supply_get_property(data.battery_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL, &ret) != 0)
		{
			pr_info("QNS: ERRROR: unable to read battery POWER_SUPPLY_PROP_CHARGE_FULL property.");
			*fcc = 0;
			retVal = QNS_ERROR;
		}
		else
			*fcc = ret.intval / 1000;
	}
	if(design != NULL)
	{
		if(power_supply_get_property(data.battery_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret) != 0)
		{
			pr_info("QNS: ERROR: unable to read battery POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN property.");
			*design = 0;
			retVal = QNS_ERROR;
		}
		else
			*design = ret.intval / 1000;
	}
	
	return retVal;
}

static int qns_get_battery_type(const char **battery_type)
{
	union power_supply_propval ret = {0,};
	int retVal = QNS_OK;
	
	if(!qns_has_psy())
	{
		pr_info("QNS: ERROR: unable to get " IBATMANAME ". Can't read battery_type!");
		*battery_type = "Unknown";
		return QNS_ERROR;
	}

	if(battery_type != NULL)
	{
		if(power_supply_get_property(data.battery_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &ret) != 0)
		{
			pr_info("QNS: ERRROR: unable to read battery POWER_SUPPLY_PROP_BATTERY_TYPE property.");
			*battery_type = "Unknown";
			retVal = QNS_ERROR;
		}
		else
			*battery_type = ret.strval;
	}
	
	return retVal;
}

/* charging_state handlers */

static ssize_t charging_state_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", qns_is_charging() ? 1 : 0);
}

static CLASS_ATTR_RO(charging_state);


/* current_now handlers */

static int c, v;

static ssize_t current_now_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	qns_get_scvt(NULL, &c, &v, NULL);
	return scnprintf(buf, PAGE_SIZE, "%d\n", c);
}

static CLASS_ATTR_RO(current_now);


/* voltage handlers */

static ssize_t voltage_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", v);
}

static CLASS_ATTR_RO(voltage);


/* temp handlers */

static ssize_t temp_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	int t;
	qns_get_scvt(NULL, NULL, NULL, &t);
	return scnprintf(buf, PAGE_SIZE, "%d\n", t);
}

static CLASS_ATTR_RO(temp);


/* fcc handlers */

static ssize_t fcc_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	int t = 0;
	qns_get_fcc(&t, NULL);
	return scnprintf(buf, PAGE_SIZE, "%d\n", t);
}

static CLASS_ATTR_RO(fcc);


/* design handlers */

static ssize_t design_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	int t = 0;
	qns_get_fcc(NULL, &t);
	return scnprintf(buf, PAGE_SIZE, "%d\n", t);
}

static CLASS_ATTR_RO(design);


/* soc handlers */

static ssize_t soc_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	int t = 0;
	qns_get_scvt(&t, NULL, NULL, NULL);
	return scnprintf(buf, PAGE_SIZE, "%d\n", t);
}

static CLASS_ATTR_RO(soc);


/* battery_type handlers */

static ssize_t battery_type_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	const char *battery_type;
	qns_get_battery_type(&battery_type);
	return scnprintf(buf, PAGE_SIZE, "%s\n", battery_type);
}

static CLASS_ATTR_RO(battery_type);


/* charge_current handlers */

static ssize_t charge_current_store(struct class *class, struct class_attribute *attr,
			      const char *buf, size_t count)
{
	int val = 0, ret = -EINVAL;
	
	ret = kstrtoint(buf, 10, &val);

	if (!ret && (val > 0))
	{
		qns_set_ibat(val);
		return count;
	}

	return -EINVAL;
}

static CLASS_ATTR_WO(charge_current);


/* charge_voltage handlers */

static ssize_t charge_voltage_store(struct class *class, struct class_attribute *attr,
			      const char *buf, size_t count)
{
	int val = 0, ret = -EINVAL;
	
	ret = kstrtoint(buf, 10, &val);

	if (!ret && (val > 0))
	{
		qns_set_vbat(val);
		return count;
	}

	return -EINVAL;
}

static CLASS_ATTR_WO(charge_voltage);


/* options handlers */

static ssize_t options_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", data.options);
}

static ssize_t options_store(struct class *class, struct class_attribute *attr,
			      const char *buf, size_t count)
{
	int val = 0, ret = -EINVAL;
	
	ret = kstrtoint(buf, 10, &val);
	
	if (!ret && (val >= 0))
	{
		data.options = val;
		return count;
	}

	return -EINVAL;
}

static CLASS_ATTR_RW(options);


/* alarm handlers */

static enum alarmtimer_restart qns_alarm_handler(struct alarm * alarm, ktime_t now)
{
	pr_info("QNS: ALARM! System wakeup!");
	__pm_stay_awake(data.wakelock);
	data.wakelock_held = true;
	data.alarm_value = 1;
	return ALARMTIMER_NORESTART;
}

enum alarm_values
{
	CHARGE_WAKELOCK = -4,
	CHARGE_WAKELOCK_RELEASE = -3,
	HANDLED = -2,
	CANCEL = -1,
	IMMEDIATE = 0,
};

static ssize_t alarm_show(struct class *class, struct class_attribute *attr,
			     char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", data.alarm_value);
}

static ssize_t alarm_store(struct class *class, struct class_attribute *attr,
			      const char *buf, size_t count)
{
	int val = 0, ret = -EINVAL;
	ktime_t next_alarm;
	
	ret = kstrtoint(buf, 10, &val);
	if(!data.wakelock_inited)
	{
		data.wakelock=wakeup_source_register(NULL, "QnovoQNS");
		data.wakelock_inited = true;
	}

	if(!data.charge_wakelock_inited)
	{
		data.charge_wakelock=wakeup_source_register(NULL, "QnovoQNS");
		data.charge_wakelock_inited = true;
	}
	if (!ret)
	{
		if(val == CHARGE_WAKELOCK)
		{
			if(!data.charge_wakelock_held)
			{
				pr_info("QNS: Alarm: acquiring charge_wakelock via CHARGE_WAKELOCK");
				__pm_stay_awake(data.charge_wakelock);
				data.charge_wakelock_held = true;
			}
		}
		else if(val == CHARGE_WAKELOCK_RELEASE)
		{
			if(data.charge_wakelock_held)
			{
				pr_info("QNS: Alarm: releasing charge_wakelock via CHARGE_WAKELOCK_RELEASE");
				__pm_relax(data.charge_wakelock);
				data.charge_wakelock_held = false;
			}
		}
		else if(val == HANDLED)
		{
			if(data.wakelock_held)
			{
				pr_info("QNS: Alarm: releasing wakelock via HANDLED");
				__pm_relax(data.wakelock);
			}
			data.alarm_value = 0;
			data.wakelock_held = false;
		}
		else if(val == CANCEL)
		{
			if(data.alarm_inited)
			{
				alarm_cancel(&data.alarm);
			}
			data.alarm_value = 0;
			if(data.wakelock_held)
			{
				pr_info("QNS: Alarm: releasing wakelock via CANCEL");
				__pm_relax(data.wakelock);
			}
			data.wakelock_held = false;
		}
		else if(val == IMMEDIATE)
		{
			if(!data.wakelock_held)
			{
				pr_info("QNS: Alarm: acquiring wakelock via IMMEDIATE");
				__pm_stay_awake(data.wakelock);
				data.wakelock_held = true;
			}
		}
		else if(val > 0)
		{
			if(!data.alarm_inited)
			{
				alarm_init(&data.alarm, ALARM_REALTIME, qns_alarm_handler);
				data.alarm_inited = true;
			}
			
			next_alarm = ktime_set(val, 0);
			alarm_start_relative(&data.alarm, next_alarm);

			if(data.wakelock_held)
			{
				pr_info("QNS: Alarm: releasing wakelock via alarm>0");
				__pm_relax(data.wakelock);
			}
			data.alarm_value = 0;
			data.wakelock_held = false;
		}
		return count;
	}
	
	return -EINVAL;
}

static CLASS_ATTR_RW(alarm);


static struct attribute *qns_class_attrs[] = {
	&class_attr_charging_state.attr,
	&class_attr_current_now.attr,
	&class_attr_voltage.attr,
	&class_attr_temp.attr,
	&class_attr_fcc.attr,
	&class_attr_design.attr,
	&class_attr_soc.attr,
	&class_attr_battery_type.attr,
	&class_attr_charge_current.attr,
	&class_attr_charge_voltage.attr,
	&class_attr_options.attr,
	&class_attr_alarm.attr,
	NULL,
};

ATTRIBUTE_GROUPS(qns_class);

static struct class qns_class =
{
	.name = "qns",
	.owner = THIS_MODULE,
	.class_groups = qns_class_groups
};

static int qns_init(void)
{
	memset(&data, 0, sizeof(data));
	data.options = -1;
	
	class_register(&qns_class);
	return 0;
}

static void qns_exit(void)
{
	if(data.wakelock_held)
		__pm_relax(data.wakelock);
	
	data.wakelock_held = false;
	wakeup_source_unregister(data.wakelock);
	
	if(data.charge_wakelock_held)
		__pm_relax(data.charge_wakelock);

	data.charge_wakelock_held = false;
	wakeup_source_unregister(data.charge_wakelock);
	
	class_unregister(&qns_class);
}

module_init(qns_init);
module_exit(qns_exit);

MODULE_AUTHOR("Miro Zmrzli <miro@qnovocorp.com>");
MODULE_DESCRIPTION("QNS System Driver v2");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("QNS");
