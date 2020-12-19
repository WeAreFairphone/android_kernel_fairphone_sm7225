/* Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, The Linux Foundation. All rights reserved.
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
// jason.kim 2015.11.12 update driver
//   Fix failure often to read from battery through HDQ
//   Tidy up codes that update and check parameters of battery
//   Add reporting current
//   Add writing to battery through HDQ

#include <linux/ctype.h>
#include <linux/init.h>
#include "bq27541-hdq-gpio.h"
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/timer.h>
//#include <linux/mfd/pmic8058.h>
//#include <linux/regulator/pmic8058-regulator.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/kernel.h>
//#include <emkit/emkit.h>
#include <linux/spinlock.h>
//#include <linux/msm-charger.h>
#include <linux/sort.h>
//#define ZXZ_BQ27541
 #define DEBUG_BQ
 //#define DEBUG_HDQ
 //#define DEBUG_UPDATE_VALUE
 //#define DEBUG_INIT_VALUE
#define NOT_USE_HDQ_TEMPERATURE  // modefied by robin.cho. charger and HDQ has a gap for temperature. For remove it, i get temperature from charger.


#define USE_REMAP_TABLE // jason.kim 2015.11.23 use remap table
// #define USE_SW_TUNE  // jason.kim 2015.11.27 use increase 100%, 0% ranges
#define USE_SERIAL_32_BYTES // jason.kim 2015.12.08 use only 12 bytes for the serial number
#ifdef USE_SERIAL_32_BYTES

typedef struct _serial_number
{
	unsigned char length_SN;
	char SN[13];
	unsigned char pad1;
	unsigned char length_ModelName;
	char ModelName[10];
	unsigned char pad2;
	unsigned char length_MF_date;
	char MF_date[4];	
}PM85_SERIAL;
#endif
//+ jason.kim 2015.11.24 dump registers
#define BQ27541_REG_CNTL		0x00 /* Control Register */
#define BQ27541_REG_AR		    0x02 /* At rate */
#define BQ27541_REG_UFSOC		0x04 /* Unfiltered SOC */
#define BQ27541_REG_TEMP		0x06 /* Temperature */
#define BQ27541_REG_VOLT		0x08 /* Voltage */
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC		    0x0C /* Nominal available capacity */
#define BQ27541_REG_FAC		    0x0E /* Full available capacity */
#define BQ27541_REG_RM		    0x10 /* Remaining available capacity */
#define BQ27541_REG_FCC		    0x12 /* Full charge capacity */
#define BQ27541_REG_AI		    0x14 /* Average current */
#define BQ27541_REG_TTE		    0x16 /* Time to empty */
#define BQ27541_REG_FFCC		0x18 /* Filtered FCC */
#define BQ27541_REG_SI		    0x1A /* Standby current */
#define BQ27541_REG_UFFCC       0x1C /* Unfiltered FCC */
#define BQ27541_REG_MLI         0x1E /* Max load current */
#define BQ27541_REG_UFRM        0x20 /* Unfiltered RM */
#define BQ27541_REG_FRM         0x22 /* Filtered RM */
#define BQ27541_REG_AP          0x24 /* Average power */

#define BQ27541_REG_INTTEMP     0x28 /* Internal temperature */
#define BQ27541_REG_CC          0x2A /* Cycle count */
#define BQ27541_REG_SOC         0x2C /* State of charge */
#define BQ27541_REG_SOH         0x2E /* State of health */

#define BQ27541_REG_PCHG        0x34 /* Passed charge */
#define BQ27541_REG_DOD0        0x36 /* Depth of discharge during the most recent OCV reading */
#define BQ27541_REG_SDSG        0x38 /* Self-discharge current */
//- jason.kim 2015.11.24 dump registers

//+ jason.kim 2015.12.07 serial number
#define BQ27541_REG_DFCLS       0x3E /* DataFlashClass */
#define BQ27541_REG_DFBLK       0x3F /* DataFlashBlock */
#define BQ27541_REG_ADF			0x40
#define BQ27541_REG_DFDCKS		0x60
#define BQ27541_REG_DFDCNTL		0x61

#define BQ27541_FLAG_CS_QEN         BIT(0)
#define BQ27541_FLAG_CS_VOK         BIT(1)
#define BQ27541_FLAG_CS_RUP_DIS     BIT(2)
#define BQ27541_FLAG_CS_LDMD        BIT(3)
#define BQ27541_FLAG_CS_SLEEP       BIT(4)
#define BQ27541_FLAG_CS_FULLSLEEP   BIT(5)
#define BQ27541_FLAG_CS_HIBERNATE   BIT(6)
#define BQ27541_FLAG_CS_SHUTDWN     BIT(7)
#define BQ27541_FLAG_CS_HDQHOSTIN   BIT(8)
#define BQ27541_FLAG_CS_RSVD        BIT(9)
#define BQ27541_FLAG_CS_BCA         BIT(10)
#define BQ27541_FLAG_CS_CCA         BIT(11)
#define BQ27541_FLAG_CS_CALMODE     BIT(12)
#define BQ27541_FLAG_CS_SS          BIT(13)
#define BQ27541_FLAG_CS_FAS         BIT(14)
#define BQ27541_FLAG_CS_SE          BIT(15)
//- jason.kim 2015.12.07 serial number

static DEFINE_MUTEX(battery_mutex);

// host to bq27541
#define tBreak	   	400	// HDQ Break Time (min 190us)
#define tBR		    100	// HDQ Break Recovery Time (min 40us)
#define tHW1        20    // Host sends 1 time (0.5~50us)
#define tHW0        115    // Host sends 0 time (86~145us)
#define tCYCH        230    // Host bit window timing (min 190us)

// bq27541 to host
#define tDW1	    41  // Slave sends 1 time (32~50us)
#define tDW0	    113	// Slave sends 0 time (80~145us)
#define tCYCD_T_ONE 60  // FIRST TEST POINT IN CYCD time (max tDW1 50 + alpha)
#define tCYCD_T_TWO 145 // SECOND TEST POINT AFTER FIRST (max tDW0 145 + alpha)
#define tRND        210 // Turnaround time
#define tCYCD       250	// The max of Cycle time of device
#define tRSPS_HIGH	150

#define tTO		500	// Time-Out Bit Receiption (500us)

#define HDQ_ERROR 0xFFFF // Time-Out Error Condition
#define BitCount 8 //8 bit

#define BATTERY_FUEL_GAUGER_SEC         10 // 10 secs
#define BATTERY_FUEL_GAUGER_SEC_LOW     5 // 5 secs
#define TIMER_HDQ 	400		//About 230us.
#define BUFFERSIZE  32			//# of bytes for Tx & Rx buffers
#define ATRATE_MA   -100		// USER CONFIG: AtRate setting (mA)
#define DEFAULT_CAPACITY    (50)
#define _KELVIN		2731
#define TEMP_DEFAULT        (273 + _KELVIN)    // 27.3
#define TEMP_LIMIT_LOW      (-250 + _KELVIN)   // -25
#define TEMP_LIMIT_HIGH     (750 + _KELVIN)    // 75C


#define NEED_SN_READ 0xBC
//#define DUMMY_GAUGE
//#define _FAIL_CNT // [robin.cho] save fail count and read count
struct bq27541_device_info;
struct bq27541_access_methods {
	int (*read)(struct bq27541_device_info *di, u8 reg);
	int (*write)(struct bq27541_device_info *di, u8 reg, u8 val);
};

struct bq27541_reg_cache {
	int temperature;
	int time_to_empty;
	//int time_to_empty_avg;
	//int time_to_full;
	int charge_full;
    int cycle_count;
    int remaining_life;
	int capacity;
	int energy;
	int flags;
	//int power_avg;
	int health;

	int voltage_now;
	int current_now;
	int charge_now;
};
enum bq27541_chip { BQ27541};

/**
 * struct bq27541_device_data - Platform-dependent data for hdq-gpio
 * @pin: GPIO pin to use
 * @is_open_drain: GPIO pin is configured as open drain
 */
struct bq27541_device_info {
	const char *name;
	unsigned int pin;
	unsigned int is_open_drain:1;
	struct device		*dev;
	int 		id;
	enum bq27541_chip	chip;
	struct bq27541_reg_cache cache;
	int charge_design_full;
	unsigned long last_update;
	struct delayed_work battery_work;
	struct power_supply *bat;
	spinlock_t   hdq_lock;
	// struct mutex lock;
	struct mutex poll_lock; // jason.kim 2015.10.15
	struct bq27541_access_methods bus;
	int battery_exist;
	int battery_init;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pinctrl_state_active;
    // jason.kim 2015.12.07 serial number
	char serial_no[33];
	int serial_len;
#ifdef _FAIL_CNT
	int fail_count;
	int read_count;
	int id_fail_count;
	int Last_fail_bat_id;
#endif
/* PLATFORM]-add by T2MNB.ZXZ,add hotswap feature,2018/06/27,defect-6502875,Begin */
    int poll_interval;
    int poll_enable;
    bool hot_swap;
	int test_temp_enable; // jason.kim 2016.6.2
	int test_temp; // jason.kim 2016.3.25 SW temperature control

	// jason.kim 2016.7.12 Test battery capacity
	int test_cap_enable;
	int test_cap;
	int test_health_enable;
	int test_health;

	// jason.kim 2016.7.18 Test battery voltage
	int test_voltage_enable;
	int test_voltage;
/* PLATFORM]-add by T2MNB.ZXZ,add hotswap feature,2018/06/27,defect-6502875,End */

};

unsigned char *DeviceName;
/*Add begin Defect-6099797, for HDQ charge safe time */
int BatteryDesignCap =0;
/*Add end Defect-6099797, for HDQ charge safe time */

/*
 *set gpio as input/output
 *bit = nozero,input,usually bit = 1
 *bit = 0,output,and default value = 1
*/
static inline void hdq_gpio_dir(struct bq27541_device_info *pdata, u8 bit)
{
	if (bit)
		gpio_direction_input(pdata->pin);
	else
		gpio_direction_output(pdata->pin, 1);
}

/* Issue break pulse to the device
 * Local function to send a HDQ break-transmission for providing a safe
 * communication start.
 */
static inline void gpio_hdq_break(struct bq27541_device_info *pdata)
{
	gpio_set_value(pdata->pin, 0);
	udelay(tBreak);
	gpio_set_value(pdata->pin, 1);
	udelay(tBR);
}

/*
 *bit = 0,HDQ = 0;which means tHW0 LOW and tCYCH-tHW0 High
 *bit = 1,HDQ = 1;Which means tHW1 LOW and tCYCH-tHW1 High
 */
static inline void gpio_hdq_bit_Write(struct bq27541_device_info *pdata, u8 bit)
{
	int tTime;
	if(bit)
		tTime = tHW1;
	else
		tTime = tHW0;

	gpio_set_value(pdata->pin, 0);
	udelay(tTime);

	gpio_set_value(pdata->pin, 1);
	udelay(tCYCH-tTime);
}

//+ jason.kim 2015.12.01 tidy codes up
//    Error rates verified with 2 units is around 3 times for around 12 hours
static void gpio_hdq_byte_Write(struct bq27541_device_info *pdata, u8 val)
{
    int i;
	for(i=0; i<8; i++) {
		gpio_hdq_bit_Write(pdata, val & (1<<i));
    }
}

static int gpio_hdq_byte_read(struct bq27541_device_info *pdata)
{
    int i;
    int bit;
    int val = 0;
    ktime_t start, now;

    for(i=0; i<8; i++) {
        // wait for bit is high
        start =  ktime_get_real();
        while(gpio_get_value(pdata->pin)) {
            now = ktime_get_real();
            if (ktime_to_us(ktime_sub(now, start)) > tCYCD) {
#ifdef 	_FAIL_CNT
                if(pdata->fail_count!=0xFFFFFFFF)
                   pdata->fail_count++;
#endif
                dev_err(pdata->dev, "%s: error\n", __func__);
                return -1;
            }
        }
        udelay(tCYCD_T_ONE);
        bit = gpio_get_value(pdata->pin) ? 1 : 0;
        val = val | (bit << i);
        udelay(tCYCD_T_TWO-tCYCD_T_ONE);
    }
#ifdef 	_FAIL_CNT
        if(pdata->read_count!=0xFFFFFFFF)
	   pdata->read_count++;
#endif

    return val;
}

static int hdq_gpio_rw(struct bq27541_device_info *pdata, int write, u8 reg, u8 val)
{
	unsigned long flags;
    int value = 0;
    int t_rnd;

	spin_lock_irqsave(&pdata->hdq_lock, flags);

	hdq_gpio_dir(pdata, 0);// output reg
	gpio_hdq_break(pdata);

    gpio_hdq_byte_Write(pdata, write ? (reg | 0x80) : (reg & 0x7f));

    if(write) {
	    udelay(tCYCH);
        gpio_hdq_byte_Write(pdata, val);
        t_rnd = tRND;

    } else {
        hdq_gpio_dir(pdata, 1);// input to read
        value = gpio_hdq_byte_read(pdata);
        t_rnd = tRND + tCYCD - tCYCD_T_ONE - tCYCD_T_TWO;
    }

    spin_unlock_irqrestore(&pdata->hdq_lock, flags);
    udelay(t_rnd); // jason.kim 2016.9.29 Fixes timing for bq27542

#ifdef DEBUG_HDQ
    if(value < 0)
        printk("%s,%d,%d\n",__FUNCTION__,__LINE__, val);
#endif
    return value;
}
//- jason.kim 2015.12.01 tidy codes up

#if 0 
//sort for decrease
static int compare(const void *lhs, const void *rhs) {
    int lhs_integer = *(const int *)(lhs);
    int rhs_integer = *(const int *)(rhs);

    if (lhs_integer < rhs_integer) return -1;
    if (lhs_integer > rhs_integer) return 1;
    return 0;
}
#endif

static inline int hdq_gpio_read(struct bq27541_device_info *di, u8 reg)
{
#if 0
	int try_cnt = 5;
	int temp_value[5]={0,};
	int i;
	for(i=0;i<try_cnt;i++)
	{
		temp_value[i] = hdq_gpio_rw(di, 0, reg, 0);
		//mdelay(5);
	}
	sort(temp_value, sizeof(temp_value)/sizeof(int), sizeof(int), &compare, NULL);
	//dev_err(di->dev, "temp_value[0]0x%x temp_value[1]0x%x temp_value[2]0x%x temp_value[3]0x%x temp_value[4]0x%x  return temp_value[3] 0x%x",temp_value[0],temp_value[1],temp_value[2],temp_value[3],temp_value[4],temp_value[3]);
	return temp_value[3];
#else
	return hdq_gpio_rw(di, 0, reg, 0);
#endif
}

static inline int hdq_gpio_write(struct bq27541_device_info *di, u8 reg, u8 val)
{
    return hdq_gpio_rw(di, 1, reg, val);
}

/*
 * Common code for BQ27541 devices
 */
static int bq27541_write8(struct bq27541_device_info *di, u8 reg, u8 val)
{
	return di->bus.write(di, reg, val);
}

static int bq27541_write16(struct bq27541_device_info *di, u8 reg, u16 val)
{
    int ret;
	ret = bq27541_write8(di, reg, val & 0xff);
	if(ret >= 0)
        ret = bq27541_write8(di, reg+1, val >> 8);
	return ret;
}

static int bq27541_read8(struct bq27541_device_info *di, u8 reg)
{
	return di->bus.read(di, reg);
}

static int bq27541_read16 (struct bq27541_device_info *di, u8 reg)
{
    int upper, lower, val;

    upper = bq27541_read8(di, reg+1);
    if(upper < 0)
        return upper;
    lower = bq27541_read8(di, reg);
    if(lower < 0)
        return lower;

    val = upper << 8 | lower;

#ifdef DEBUG_HDQ
    printk("Upper 0x%x, lower 0x%x, ret = 0x%x\n", upper, lower, val);
#endif
    return val;
}

static int bq27541_pinctrl_init(struct bq27541_device_info *di)
{
    int ret;

    di->pinctrl = devm_pinctrl_get(di->dev);
    if (IS_ERR_OR_NULL(di->pinctrl)) {
        ret = PTR_ERR(di->pinctrl);
        goto err_pinctrl_get;
    }

    di->pinctrl_state_active = pinctrl_lookup_state(di->pinctrl, "bq27541_hdq");
    if (IS_ERR_OR_NULL(di->pinctrl_state_active)) {
        ret = PTR_ERR(di->pinctrl_state_active);
        goto err_pinctrl_lookup;
    }

    ret = pinctrl_select_state(di->pinctrl, di->pinctrl_state_active);
    if (ret) {
        return ret;
    }

    return 0;

err_pinctrl_lookup:
    devm_pinctrl_put(di->pinctrl);

err_pinctrl_get:
    di->pinctrl = NULL;

    return ret;
}

//+ jason.kim 2015.12.07 serial number
static int set_mfg_block(struct bq27541_device_info *di, int sealed)
{
	u8 block = 1;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	if (sealed) {
		block = 2;
	} else {
		int dfd_cntl_status = 0;
        dfd_cntl_status = bq27541_read8(di, BQ27541_REG_DFDCNTL);
		if (dfd_cntl_status != 0) {
            bq27541_write8(di, BQ27541_REG_DFDCNTL, 0);
		}
        bq27541_write8(di, BQ27541_REG_DFCLS, 58);
		block = 1;
	}

    bq27541_write8(di, BQ27541_REG_DFBLK, block);
	msleep(1);
	return 0;
}

#ifdef USE_SERIAL_32_BYTES
#define SERIAL_BYTES     32
#if 0
static int is_good_serial(char *serial)
{
    int i;
    for(i=0; i<SERIAL_BYTES; i++) {
        if(!isalnum(serial[i]))
            return 0;
    }

    return 1;
}
#endif
#endif

static void bq27541_battery_read_serial(struct bq27541_device_info *di)
{
    int flags;
    int data;
	u8 sum = 0;
	int sealed = 0;
	int serial_len;
	int i;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    bq27541_write16(di, BQ27541_REG_CNTL, BQ27541_SUBCMD_CTNL_STATUS);
    udelay(66);
    flags = bq27541_read16(di, BQ27541_REG_CNTL);
//	di->serial_len = 0;
	sealed = (flags & BQ27541_FLAG_CS_SS) == BQ27541_FLAG_CS_SS;
	if (flags < 0 || set_mfg_block(di, sealed)) {
	    dev_err(di->dev, "set_mfg_block failed\n");
		return;
	}

#ifdef USE_SERIAL_32_BYTES
    serial_len = SERIAL_BYTES;
	sum = 0;
#else
    serial_len = bq27541_read8(di, BQ27541_REG_ADF);
	sum = serial_len;
#endif
	if (serial_len <= 0 || serial_len > 32) {
	    dev_err(di->dev, "serial_len %d is not valid\n", serial_len);
		return;
	}

	memset(di->serial_no, 0, sizeof(di->serial_no));
	for (i = 0; i < serial_len; i++) {
#ifdef USE_SERIAL_32_BYTES
        data = bq27541_read8(di, BQ27541_REG_ADF + i);
#else
        data = bq27541_read8(di, BQ27541_REG_ADF + i + 1);
#endif
//		dev_err(di->dev, "## data 0x%x \n", data);

        if(data < 0) {
	        dev_info(di->dev, "Read %ith failed\n", i);
            return;
        }
        di->serial_no[i] = (char)data;
		sum += di->serial_no[i];
	}

    data = bq27541_read8(di, BQ27541_REG_DFDCKS);
    sum += data;
	if (data < 0 || sum != 0xFF) {
	    dev_err(di->dev, "check-sum 0x%x data 0x%x error \n", sum , data);
		return;
	}
#ifdef USE_SERIAL_32_BYTES
	if( di->serial_no[0]<=0)
	{
	    dev_err(di->dev, "serial number:error ::di->serial_no[0] 0x%x  di->serial_len 0x%x\n",di->serial_no[0],di->serial_len );
		return;
		
	}
#endif
	di->serial_len = serial_len;
}

static int bq27541_battery_write_serial(struct bq27541_device_info *di, const char* serial_no)
{
	u8 sum = 0;
	int sealed = 0;
	int serial_len = strlen(serial_no);
    u8 buf_w[32], buf_v[32];
	int i;
	int flags = 0;
    int data;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	if (serial_len && serial_no[serial_len-1] == 0x0a) {
		serial_len = serial_len - 1;
	}

#ifdef USE_SERIAL_32_BYTES
    if(serial_len != SERIAL_BYTES) {
	    dev_err(di->dev, "serial_len %d(%d) is not valid\n", serial_len, SERIAL_BYTES);
        return 0;
    }
#endif

	if (serial_len <= 0 || serial_len > 32) {
	    dev_err(di->dev, "serial_len %d is not valid\n", serial_len);
		return 0;
	}

    bq27541_write16(di, BQ27541_REG_CNTL, BQ27541_SUBCMD_CTNL_STATUS);
    udelay(66);
    flags = bq27541_read16(di, BQ27541_REG_CNTL);
	sealed = (flags & BQ27541_FLAG_CS_SS) == BQ27541_FLAG_CS_SS;
	if (flags < 0 || set_mfg_block(di, sealed)) {
	    dev_err(di->dev, "set_mfg_block failed\n");
		return 0;
	}

#ifdef USE_SERIAL_32_BYTES
	memset(buf_w, 0, sizeof(buf_w));
    memcpy(buf_w, serial_no, serial_len);
	dev_info(di->dev, "Write serial [%s] to battery\n", buf_w);
#else
    // prepare data
	memset(buf_w, 0, sizeof(buf_w));
    buf_w[0] = serial_len;
    memcpy(buf_w+1, serial_no, serial_len);
	dev_info(di->dev, "Write serial [%s] to battery\n", buf_w+1);
#endif

    // write it to battery
	for (i = 0; i < sizeof(buf_w); i++) {
        bq27541_write8(di, BQ27541_REG_ADF+i, buf_w[i]);
		sum += buf_w[i];
	}
	sum = 0xff - sum;

    // read it back
	memset(buf_v, 0, sizeof(buf_w));
	for (i = 0; i < sizeof(buf_w); i++) {
        data = bq27541_read8(di, BQ27541_REG_ADF + i);
        if(data < 0) {
	        dev_info(di->dev, "Read %ith failed\n", i);
            return 0;
        }
        buf_v[i] = (char)data;
	}

    // check it
    if(memcmp(buf_w, buf_v, sizeof(buf_w))) {
	    dev_err(di->dev, "Verification failed\n");
		return 0;
    }
    // write check-sum to battery
	dev_info(di->dev, "Check-sum=0x%x\n", sum);
    bq27541_write8(di, BQ27541_REG_DFDCKS, sum);
	msleep(200);
	return serial_len;
}
//- jason.kim 2015.12.07 serial number

//+ jason.kim 2015.12.08 dump MFG-B
#define DUMP_COL 16
static void dump_bin(u8 *buf, int size)
{
    int i, j;
    int lc;
    char line[DUMP_COL];
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	pr_info("========================\n");
    for(lc=0, i=0; i<size; i++) {
        pr_cont( "%02X ", buf[i]);
        line[lc++] = buf[i];

        if((i+1)%DUMP_COL == 0) {
            lc=0;
            for(j=0; j<DUMP_COL; j++) {
                if(line[j] >= ' ' && line[j] <= '~')
                    pr_cont( "%c", line[j]);
                else
                    pr_cont( ".");
            }
	        pr_cont("\n");
        }
    }
	pr_info("========================\n");
}

static void bq27541_battery_dump_mfg_B(struct bq27541_device_info *di)
{
    int flags;
    int data;
	u8 sum = 0;
	int sealed = 0;
    char serial_no[32];
	int serial_len;
	int i;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    bq27541_write16(di, BQ27541_REG_CNTL, BQ27541_SUBCMD_CTNL_STATUS);
    udelay(66);
    flags = bq27541_read16(di, BQ27541_REG_CNTL);
	di->serial_len = 0;
	sealed = (flags & BQ27541_FLAG_CS_SS) == BQ27541_FLAG_CS_SS;
	if (flags < 0 || set_mfg_block(di, sealed)) {
	    dev_err(di->dev, "set_mfg_block failed\n");
		return;
	}

    serial_len = 32;
	for (i = 0; i < serial_len; i++) {
        data = (char)bq27541_read8(di, BQ27541_REG_ADF + i);
        if(data < 0) {
	        dev_info(di->dev, "Read %ith failed\n", i);
            return;
        }
        serial_no[i] = (char)data;
		sum += serial_no[i];
	}
    data = bq27541_read8(di, BQ27541_REG_DFDCKS);
    if(data < 0) {
        dev_info(di->dev, "Read check-sum failed\n");
        return;
    }

    dump_bin((u8 *)serial_no, 32);
	dev_info(di->dev, "Check sum=0x%x(0x%x)\n", sum, data);
}
//- jason.kim 2015.12.08 dump MFG-B

//+ jason.kim 2016.01.20 Check and update battery parameters
static int bq27541_get_sealing(struct bq27541_device_info *di)
{
    int flags;
	int sealed;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    bq27541_write16(di, BQ27541_REG_CNTL, BQ27541_SUBCMD_CTNL_STATUS);
    udelay(66);
    flags = bq27541_read16(di, BQ27541_REG_CNTL);
	dev_info(di->dev, "flags 0x%X\n", flags);
	sealed = (flags & BQ27541_FLAG_CS_SS) == BQ27541_FLAG_CS_SS;
	if (flags < 0) {
	    dev_err(di->dev, "Failed to read BQ27541_REG_CNTL\n");
        return flags;
	}
    return sealed;
}

static int bq27541_set_sealing(struct bq27541_device_info *di, int sealing)
{
	int sealed;
    int ret;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    if (sealing) {
        bq27541_write16(di, BQ27541_REG_CNTL, BQ27541_SUBCMD_SEALED);
        msleep(200);
        bq27541_write16(di, BQ27541_REG_CNTL, BQ27541_SUBCMD_CTNL_STATUS);
        udelay(66);
    } else {
        bq27541_write16(di, BQ27541_REG_CNTL, 0x0414);
        bq27541_write16(di, BQ27541_REG_CNTL, 0x3672);
        msleep(100);
        bq27541_write16(di, BQ27541_REG_CNTL, BQ27541_SUBCMD_CTNL_STATUS);
        udelay(66);
    }

    sealed = bq27541_get_sealing(di);
    ret = (sealed == sealed) ? 0 : -1;
    if(ret < 0)
        dev_err(di->dev, "Failed to set sealing(%d)\n", sealing);

    return ret;
}

static int bq27541_rw_block(struct bq27541_device_info *di, int subcmd, int offset)
{
    int ret;

    	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    ret = bq27541_write8(di, BQ27541_REG_DFCLS, subcmd);
    if (ret >= 0)
        ret = bq27541_write8(di, BQ27541_REG_DFBLK, offset / 32);
    if (ret >= 0)
        ret = bq27541_write8(di, BQ27541_REG_DFDCNTL, 0);
    return ret;
}

static int bq27541_rw_flash(struct bq27541_device_info *di, int subcmd, int offset, u8 *buf, int bytes, int write)
{
    int ret;
    int i;
    	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    ret = bq27541_rw_block(di, subcmd, offset);
    if (ret < 0)
        return ret;

    // i must be less then 32
    for(i = offset%32; i < (offset%32 + bytes); i++) {
        if(write) {
            ret = bq27541_write8(di, BQ27541_REG_ADF + i, *buf++);
            if (ret < 0)
                return ret;
        } else {
            ret = bq27541_read8(di, BQ27541_REG_ADF + i);
            if (ret < 0)
                return ret;
            *buf++ = (u8)ret;
        }
    }
    return 0;
}

static int bq27541_read_flash16(struct bq27541_device_info *di, int subcmd, int offset)
{
    int ret;
    u8 buf[2];
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    ret = bq27541_rw_flash(di, subcmd, offset, buf, 2, 0);
    if (ret < 0)
        return ret;

    return ((int)buf[0] << 8) | buf[1];
}

__attribute__ ((unused)) static int bq27541_write_flash16(struct bq27541_device_info *di, int subcmd, int offset, int data)
{
    u8 buf[2];
    buf[0] = (data >> 8) & 0xff;
    buf[1] = data & 0xff;

    return bq27541_rw_flash(di, subcmd, offset, buf, 2, 1);
}

static int bq27541_battery_reset_param(struct bq27541_device_info *di)
{
    int ret;
    u8 buf[34] = {
        0x0A,0xF0,0x00,0x0A,0x05,0x00,0x32,0x01,0xC2,0x14,0x14,0x00,0x08,0x09,0xF6,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    };
    u8 sum;
    int i;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    ret = bq27541_set_sealing(di, 0); // unseal
    if (ret < 0)
        return ret;

    bq27541_rw_flash(di, 68, 0, buf, 32, 1);
    sum = 0;
    for (i=0; i<32; i++) {
        sum += buf[i];
    }
    sum = 0xff - sum;
    bq27541_write8(di, BQ27541_REG_DFDCKS, sum);
    msleep(200);
    bq27541_rw_flash(di, 68, 0, buf, 32, 0);
    dump_bin(buf, 32);
    di->battery_init = 0;

    ret = bq27541_set_sealing(di, 1); // seal
    if (ret < 0)
        return ret;
    return 0;
}

static int bq27541_battery_init_param(struct bq27541_device_info *di)
{
    int ret;
    int h_current = 0;
    int h_voltage = 0;
    u8 buf[34];
    u8 sum;
    int i;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    ret = bq27541_set_sealing(di, 0); // unseal
    if (ret < 0)
        return ret;

    h_current = bq27541_read_flash16(di, 68, 11);
    h_voltage = bq27541_read_flash16(di, 68, 13);
    dev_info(di->dev, "Current %d, Voltage %d\n", h_current, h_voltage);
    if(h_current != 0 || h_voltage != 0) {
        bq27541_rw_flash(di, 68, 0, buf, 32, 0);
        dump_bin(buf, 32);
        buf[11] = 0;
        buf[12] = 0;
        buf[13] = 0;
        buf[14] = 0;
        bq27541_rw_flash(di, 68, 0, buf, 32, 1);
        sum = 0;
        for (i=0; i<32; i++) {
            sum += buf[i];
        }
        sum = 0xff - sum;
        bq27541_write8(di, BQ27541_REG_DFDCKS, sum);
        msleep(200);
        h_current = bq27541_read_flash16(di, 68, 11);
        h_voltage = bq27541_read_flash16(di, 68, 13);
        if(h_current == 0 && h_voltage == 0)
            di->battery_init = 1;
        dev_info(di->dev, "Current %d, Voltage %d\n", h_current, h_voltage);
    } else {
        di->battery_init = 1;
    }
    bq27541_rw_flash(di, 68, 0, buf, 32, 0);
    dump_bin(buf, 32);
    ret = bq27541_set_sealing(di, 1); // seal
    if (ret < 0)
        ret = bq27541_set_sealing(di, 1); // seal
    if (ret < 0)
        ret = bq27541_set_sealing(di, 1); // seal

    return ret;
}
//- jason.kim 2016.01.20 Check and update battery parameters

static int bq27541_simple_value(int value,
	union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}
#if 0
static int bq27541_s16_value(int value, union power_supply_propval *val)
{
	if (value < 0)
		return value;

    val->intval = (int)((s16)value);
	return 0;
}
#endif
static int bq27541_battery_read_id(struct bq27541_device_info *di)
{
    int retry = 5;
    int value;
#ifdef DEBUG_BQ
	printk("zxz   -------  %s ,line=%d \n", __FUNCTION__,__LINE__);
		dev_err(di->dev, "zxz   bq27541_battery_read_id\n");
#endif
#if 0
        bq27541_write16(di, bq27541CMD_CNTL_LSB, BQ27541_SUBCMD_DEVCIE_TYPE); // this code removed by robin.cho
        value = bq27541_read16(di, bq27541CMD_CNTL_LSB);
	retry=0;
#else
    do {
        bq27541_write16(di, bq27541CMD_CNTL_LSB, BQ27541_SUBCMD_DEVCIE_TYPE);
        value = bq27541_read16(di, bq27541CMD_CNTL_LSB);
		//mdelay(100);

    } while((value !=BQ27542_DEVICE_ID) && retry--);
    #endif
#ifdef DEBUG_BQ
		dev_err(di->dev, "zxz   bq27541_battery_read_id , value=%x ,retry=%d \n",value,retry);
	printk("zxz   -------  %s ,line=%d  ,value=%x \n", __FUNCTION__,__LINE__,value);
#endif
    return value;
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27541_battery_read_current(struct bq27541_device_info *di)
{
	int curr;
	curr = bq27541_read16(di, bq27541CMD_AI_LSB);
	if(curr < 0){
        dev_err(di->dev, "error reading curr\n");
	}
    return curr;
}

/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27541_battery_read_cc(struct bq27541_device_info *di)
{
	int cyct;

	cyct = bq27541_read16(di, bq27541CMD_CC_LSB);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}
#if 0

/*
 * Read a power avg register.
 * Return < 0 if something fails.
 */
static int bq27541_battery_read_pwr_avg(struct bq27541_device_info *di, u8 reg)
{
	int tval;

	tval = bq27541_read16(di, reg, reg+1);
	if (tval < 0) {
		dev_err(di->dev, "error reading power avg rgister  %02x: %d\n",
			reg, tval);
		return tval;
	}
	return tval;
}
#endif

/*
 * Higher versions of the chip like BQ27425 and BQ27500
 * differ from BQ27000 and BQ27200 in calculation of certain
 * parameters. Hence we need to check for the chip type.
 */
static bool bq27xxx_is_chip_version_higher(struct bq27541_device_info *di)
{

	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	if (di->chip == BQ27541)
		return true;
	return false;
}

/*Remove by T2M Q.Z for PR1003449
* Do not use status detection from bq27541.
* Use smb1357 status detection.
*/
/*
static int bq27541_battery_status(struct bq27541_device_info *di,
	union power_supply_propval *val)
{
	int status;

	if(di->battery_exist == 0)
		status = POWER_SUPPLY_STATUS_UNKNOWN;
	else{
		if (bq27xxx_is_chip_version_higher(di)) {
			if (di->cache.flags & BQ27541_FLAG_FC)
				status = POWER_SUPPLY_STATUS_FULL;
			else if (di->cache.flags & BQ27541_FLAG_DSC)
				status = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				status = POWER_SUPPLY_STATUS_CHARGING;
		}
	}
	val->intval = status;

	return 0;
}
*/

/*
* get FLAGS from bq27541. many marker bits in the FLAGS.
* FC means full-charged . OTC means Over-Temperature and so on.
*/
static int bq27541_get_battery_flags(struct bq27541_device_info *di)
{
    int flags;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    flags = bq27541_read16(di, bq27541CMD_FLAGS_LSB);
    if (flags < 0){
        dev_err(di->dev, "error reading flags\n");
    }

    return flags;
}

static int bq27541_battery_read_volt(struct bq27541_device_info *di)
{
	int volt;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	dev_err(di->dev, "zxz  %s , Line = %d\n",__FUNCTION__,__LINE__);
	#endif
	volt = bq27541_read16(di, bq27541CMD_VOLT_LSB);
	if (volt < 0) {
        dev_err(di->dev, "error reading voltage\n");
        return volt;
	}

	return volt *1000;
}

/*
 * Return a battery charge value in µAh
 * Or < 0 if something fails.
 */
static int bq27541_battery_read_charge(struct bq27541_device_info *di, u8 reg)
{
	int charge;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	dev_err(di->dev, "zxz  %s , Line =%d \n",__FUNCTION__,__LINE__);
	#endif
	charge = bq27541_read16(di, reg);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge \n");
		return charge;
	}

	if (bq27xxx_is_chip_version_higher(di))
		charge *= 1000;

	return charge;
}

/*
 * Return the battery FullChargeCapacity in mAh
 * Or < 0 if something fails.
 */
static inline int bq27541_battery_read_fcc(struct bq27541_device_info *di)
{
	return bq27541_battery_read_charge(di, bq27541CMD_FCC_LSB);
}


/*
 * Return the battery Nominal available capaciy in µAh
 * Or < 0 if something fails.
 */
static int bq27541_battery_read_nac(struct bq27541_device_info *di)
{
	int charge;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	dev_err(di->dev, "zxz  %s , Line =%d \n",__FUNCTION__,__LINE__);
	#endif
	charge = bq27541_read16(di, bq27541CMD_NAC_LSB);
	if (charge < 0) {
		dev_err(di->dev, "error reading NAC\n");
		return charge;
	}

    return charge * 1000;
}

/*
 * Return the battery Relative State-of-Charge
 * Or = 0xff if something fails.
 */
static int bq27541_battery_read_rsoc(struct bq27541_device_info *di)
{
	int rsoc;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	dev_err(di->dev, "zxz  %s , Line =%d \n",__FUNCTION__,__LINE__);
	#endif
	rsoc = bq27541_read16(di, bq27541CMD_SOC_LSB);
	if (rsoc < 0){
        dev_err(di->dev, "error reading State-of-Charge:CAPACITY\n");
        return rsoc;
	}

	return rsoc;
}

/*
 * Return the battery Available energy in µWh
 * Or < 0 if something fails.
 */
static int bq27541_battery_read_energy(struct bq27541_device_info *di)
{
	int ae;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	dev_err(di->dev, "zxz  %s , Line =%d \n",__FUNCTION__,__LINE__);
	#endif
	ae = bq27541_read16(di, bq27541CMD_AE_LSB);
	if (ae < 0) {
		dev_err(di->dev, "error reading available energy\n");
		return ae;
	}

	if (di->chip == BQ27541)
		ae *= 1000;

	return ae;
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
// START removed by chase.jang(2018.04.18)
// unsed function
#if 0
static int bq27541_battery_read_health(struct bq27541_device_info *di)
{
	int tval;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	dev_err(di->dev, "zxz  %s , Line = %d\n",__FUNCTION__,__LINE__);
	#endif
	tval = bq27541_read16(di, bq27541CMD_FLAGS_LSB);
	if (tval < 0) {
        dev_err(di->dev, "error reading health\n");
        return tval;
	}

	if (tval & BQ27541_FLAG_SOCF)
		tval = POWER_SUPPLY_HEALTH_DEAD;
	else if (tval & BQ27541_FLAG_OTC)
		tval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (tval & BQ27541_FLAG_OTD) // for overtemperature when discharging
		tval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		tval = POWER_SUPPLY_HEALTH_GOOD;
	return tval;
}
#endif
// END

static int get_health_from_qpnp_smbcharger(struct bq27541_device_info *di)
{
	struct power_supply * bms_supply = NULL;
	union power_supply_propval prop = {0,};
	/*
		char *str_power_supply[]=
	{
		"POWER_SUPPLY_HEALTH_UNKNOWN",
		"POWER_SUPPLY_HEALTH_GOOD",
		"POWER_SUPPLY_HEALTH_OVERHEAT",
		"POWER_SUPPLY_HEALTH_DEAD",
		"POWER_SUPPLY_HEALTH_OVERVOLTAGE",
		"POWER_SUPPLY_HEALTH_UNSPEC_FAILURE",
		"POWER_SUPPLY_HEALTH_COLD",
		"POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE",
		"POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE",
		"POWER_SUPPLY_HEALTH_WARM",
		"POWER_SUPPLY_HEALTH_COOL",
	};*/
/*
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
		en = (strcmp(prop.strval, UNKNOWN_BATT_TYPE) != 0
				|| chip->charge_unknown_battery)
			&& (strcmp(prop.strval, LOADING_BATT_TYPE) != 0);
		vote(chip->battchg_suspend_votable,
				BATTCHG_UNKNOWN_BATTERY_EN_VOTER, !en, 0);
		*/

	bms_supply = power_supply_get_by_name("battery");
	if(bms_supply == NULL)
	{
		 dev_err(di->dev, "robin.cho]can't get battery-SO return goood status----------\n");
		 return POWER_SUPPLY_HEALTH_GOOD;
	}
	else
	{
		power_supply_get_property(bms_supply,POWER_SUPPLY_PROP_HEALTH, &prop);
		//dev_err(di->dev, "robin.cho]  health status %s\n",str_power_supply[prop.intval]);
	}
	return prop.intval;
}

/*
 * Return the battery temperature in tenths of degree Kelvin
 * Or < 0 if something fails.
 */
 
static int bq27541_battery_read_temperature(struct bq27541_device_info *di)
{

#ifdef NOT_USE_HDQ_TEMPERATURE
		struct power_supply * bms_supply = NULL;
		union power_supply_propval prop = {0,};
		bms_supply = power_supply_get_by_name("bms");
		if(bms_supply == NULL)
		{
			 dev_err(di->dev, "robin.cho]can't get bms-SO Defaut temp 27.3C----------\n");
			 return TEMP_DEFAULT;
		}
		else
		{
			power_supply_get_property(bms_supply,POWER_SUPPLY_PROP_TEMP, &prop);
			// dev_err(di->dev, "robin.cho]can get bms temp %d----------\n",prop.intval);
		}
		return prop.intval+_KELVIN;

#else
	return bq27541_read16(di,bq27541CMD_TEMP_LSB);
#endif
}

/*
 * Return the battery design capacity in mAh
 * Or < 0 if something fails.
 */
static int bq27541_battery_read_dcap(struct bq27541_device_info *di)
{
	int ilmd;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	dev_err(di->dev, "zxz  %s , Line =%d \n",__FUNCTION__,__LINE__);
	#endif
	if (bq27xxx_is_chip_version_higher(di))
		ilmd = bq27541_read16(di, bq27541CMD_DCAP_LSB);

	if (ilmd < 0) {
		dev_err(di->dev, "error reading DESIGN CAPACITY\n");
		return ilmd;
	}

	if (bq27xxx_is_chip_version_higher(di))
		ilmd *= 1000;
	return ilmd;
}

/*
 * Return remaining life (unit : %)
 */
static int bq27541_battery_read_remaining_life(struct bq27541_device_info *di)
{
    int remaining_life = 0;

    remaining_life = bq27541_read16(di, bq27541CMD_SOH_LSB);
    if ((remaining_life < 0) || (remaining_life > 100))
        dev_err(di->dev, "error reading remaining_life\n");

    return remaining_life;
}
// jason.kim 2015.11.23 use remap table
#ifdef USE_REMAP_TABLE
static int bq27541_battery_capacity(struct bq27541_device_info *di,
	union power_supply_propval *val)
{
    const char remap_capacity[101] = {
	    0,   0,   0,   0,   1,   3,   4,   5,   7,   8,
	    9,  10,  12,  14,  15,  16,  18,  19,  20,  22,
	    23,  24,  26,  27,  28,  30,  31,  32,  34,  35,
	    36,  38,  39,  41,  42,  43,  45,  46,  47,  49,
	    51,  52,  53,  54,  55,  56,  57,  58,  59,  60,
	    61,  62,  63,  64,  65,  66,  67,  68,  69,  70,
	    71,  72,  73,  74,  75,  76,  77,  78,  79,  80,
	    81,  82,  83,  84,  85,  86,  87,  88,  90,  90,
	    91,  91,  92,  92,  93,  93,  94,  94,  95,  95,
	    96,  96,  97,  97,  98,  99,  99, 100, 100, 100,
	    100
    };
    int rsoc = di->cache.capacity;

	if (rsoc < 0 || rsoc > 100)
		return -1;

    rsoc = remap_capacity[rsoc];
    val->intval = rsoc;
  //  printk("%s: rsoc %d => %d\n", __func__, di->cache.capacity, rsoc); // MODIFIED by liquan.zhou, 2018-09-08,BUG-6904042
	return 0;
}
#elif defined(USE_SW_TUNE)
// jason.kim 2015.11.21 fake capacity
#define CAPACITY_0      4
#define CAPACITY_100    97
#define CAPACITY_CUT    (CAPACITY_0 + (100 - CAPACITY_100 + 1))
static int bq27541_battery_capacity(struct bq27541_device_info *di,
	union power_supply_propval *val)
{
    int rsoc = di->cache.capacity;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
		dev_err(di->dev, "zxz  %s , Line = %d\n",__FUNCTION__,__LINE__);
	#endif
	if (rsoc < 0)
		return rsoc;

    if(rsoc >= CAPACITY_100) {
        rsoc = 100;
    } else {
        rsoc = rsoc * (100+CAPACITY_CUT) / 100 - CAPACITY_0;
        if(rsoc < 0)
            rsoc = 0;
        if(rsoc > 100)
            rsoc = 100;
    }

    val->intval = rsoc;
    printk("%s: rsoc %d => %d\n", __func__, di->cache.capacity, rsoc);
    return 0;
}
#else
// jason.kim 2015.11.27 Reports original capacity
static int bq27541_battery_capacity(struct bq27541_device_info *di,
	union power_supply_propval *val)
{
    int rsoc = di->cache.capacity;
    	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
			dev_err(di->dev, "zxz  %s , Line = %d\n",__FUNCTION__,__LINE__);
	#endif
    val->intval = rsoc;
    return rsoc;
}
#endif

static int bq27541_battery_capacity_level(struct bq27541_device_info *di,
	union power_supply_propval *val)
{
	int level;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	dev_err(di->dev, "zxz  %s , Line = %d\n",__FUNCTION__,__LINE__);
	#endif
	if (bq27xxx_is_chip_version_higher(di)){
		if (di->cache.flags & BQ27541_FLAG_FC)
			level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (di->cache.flags & BQ27541_FLAG_SOC1)
			level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else if (di->cache.flags & BQ27541_FLAG_SOCF)
			level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else
			level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}

	val->intval = level;

	return 0;
}

/*
 * Read a time register.
 * Return < 0 if something fails.
 */
static int bq27541_battery_read_time(struct bq27541_device_info *di, u8 reg)
{
	int tval;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	tval = bq27541_read16(di, reg);
	if (tval < 0) {
		dev_err(di->dev, "error reading time register\n");
		return tval;
	}
	return tval;
}


// jason.kim 2015.11.12
#define CAPACITY_0_CHECK_COUNT 4 // [2018/05/28 robin.cho]Add for issue #12070
static void bq27541_update(struct bq27541_device_info *di)
{


    struct bq27541_reg_cache cache = {0, };
    int battery_exist;
    int value;
    int value2;
    int bat_id;
    static int CAPACITY_0_check=0;//[2018/05/28 robin.cho]Add for issue #12070
    if(di->cache.flags == 0)
 	di->cache.flags = 0x100;

    cache = di->cache;

#ifdef DEBUG_BQ
	printk("zxz   -------  %s ,line=%d \n", __FUNCTION__,__LINE__);
#endif

#ifdef DEBUG_BQ
	dev_err(di->dev, "zxz  111 bq27541_update\n");
#endif
	bat_id=0;
	bat_id = bq27541_battery_read_id(di);
    battery_exist = (bat_id == BQ27542_DEVICE_ID);
    #ifdef DEBUG_BQ
	dev_err(di->dev, "zxz 222  bq27541_update\n");
#endif


#if 1
	/*get HDQ flags*/
	value = bq27541_get_battery_flags(di);
	if(value >= 0) {
        cache.flags = value;
    }
    #ifdef DEBUG_BQ
    	dev_err(di->dev, "zxz 333  bq27541_update\n");
#endif
#endif

#ifdef DEBUG_BQ
    	dev_err(di->dev, "zxz  444 bq27541_update\n");
   #endif
    if(battery_exist != di->battery_exist) {
		di->battery_exist = battery_exist;
        if(di->battery_exist) {
            printk("%s: Battery is detected\n", __func__);
        } else {
            printk("%s: Battery is not detected\n", __func__);
#ifdef _FAIL_CNT
			if(di->id_fail_count!=0xFFFFFFFF)
			di->id_fail_count++;
			di->Last_fail_bat_id=bat_id;
#endif
	        di->serial_len = NEED_SN_READ; // jason.kim 2015.12.07 srial number
            di->battery_init = 0;
            di->poll_interval = BATTERY_FUEL_GAUGER_SEC_LOW;
            goto battery_not_exist;
        }
    }
#ifdef DEBUG_BQ
	printk("zxz   -------  %s ,line=%d , battery_exist=%d  \n", __FUNCTION__,__LINE__,battery_exist);
#endif
    // jason.kim 2015.12.07 srial number
    if(battery_exist) {
    /*Defect-6099592 no need read battery serial number*/
#if 1
        if(di->serial_len==NEED_SN_READ) {
            bq27541_battery_read_serial(di);
            //dev_info(di->dev, "Serial=%s\n", di->serial_len ? di->serial_no : "unknown");
        }
#endif
        // jason.kim 2016.01.20 Check and update battery parameters
        if(!di->battery_init) {
            bq27541_battery_init_param(di);
        }
    }
	/*get HDQ flags*/
	value = bq27541_get_battery_flags(di);
	if(value >= 0) {
        cache.flags = value;
    }
#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.flags = 0x%x\n",__FUNCTION__,__LINE__,cache.flags);
	    	dev_err(di->dev, "zxz  5555 bq27541_update, cache.flags=%d\n",cache.flags);
#endif

	/*get voltage*/
	value = bq27541_battery_read_volt(di);
	if( (value >= 0) && (value < 4400000) ) {
		cache.voltage_now = value;
	}

#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.voltage_now = %d\n",__FUNCTION__,__LINE__,cache.voltage_now);
		    	dev_err(di->dev, "zxz  6666 bq27541_update, cache.voltage_now=%d\n",cache.voltage_now);

#endif

	/*get current*/
	value = bq27541_battery_read_current(di);
	if(value >= 0) {
        cache.current_now = value;
    }
#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.voltage_now = %d\n",__FUNCTION__,__LINE__,cache.current_now);
	dev_err(di->dev, "zxz  77777 bq27541_update, cache.current_now=%d\n",cache.current_now);
#endif

    // jason.kim 2015.11.27 check capacity error
	/*capacity*/
	value = bq27541_battery_read_rsoc(di);
	if(value >= 0) {
	   msleep(10);
           value2 = bq27541_battery_read_rsoc(di);
	    if( value2 >=0 && abs(value2 - value) <= 1 && value >= 0 && value<=100 ) {
          // +(2018.05.28 robin.cho)
	  //  [issue #12070] if battery gauge chip reporting as 0 percent,we check it 4 times because gauge chip sometime wrong value reporting
	         if( value == 0 ) {
	             ++CAPACITY_0_check;
#ifdef DEBUG_UPDATE_VALUE
		         dev_info(di->dev, "robin.cho]%s,%d,CAPACITY_0_check = %d value = %d value2 = %d\n",__FUNCTION__,__LINE__,CAPACITY_0_check,value,value2);
#endif
                 if(CAPACITY_0_check >= CAPACITY_0_CHECK_COUNT ) {
	                 CAPACITY_0_check = 0;
	                 cache.capacity = value;
	             }
	        }else {
	            CAPACITY_0_check = 0;
                cache.capacity = value;
	        }
	        //-(2018.05.28 robin.cho)
	    }
       }
#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.capacity = %d\n",__FUNCTION__,__LINE__,cache.capacity);
	dev_err(di->dev, "zxz  8888 bq27541_update, cache.capacity=%d\n",cache.capacity);
#endif

	/*get available energy*/
	value = bq27541_battery_read_energy(di);
	if(value >= 0){
		cache.energy = value;
	}
#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.energy = %d\n",__FUNCTION__,__LINE__,cache.energy);
	dev_err(di->dev, "zxz  9999 bq27541_update, cache.energy=%d\n",cache.energy);
#endif

	/*get nominal available capacity*/
	value = bq27541_battery_read_nac(di);
	if(value >= 0){
		cache.charge_now = value;
	}
#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.charge_now = %d\n",__FUNCTION__,__LINE__,cache.charge_now);
	dev_err(di->dev, "zxz  AAAAA bq27541_update, cache.charge_now=%d\n",cache.charge_now);
#endif
#if 0
	/*get health*/
	value = bq27541_battery_read_health(di);
	if(value >= 0){
		cache.health = value;
	}
#endif
	cache.health = get_health_from_qpnp_smbcharger(di); // get battery health status smbcharger


#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.health = %d\n",__FUNCTION__,__LINE__,cache.health);
	dev_err(di->dev, "zxz  BBBB bq27541_update, cache.health=%d\n",cache.health);
#endif

    // jason.kim 2015.11.27 check temperature error
	/*get temperature*/
	value = bq27541_battery_read_temperature(di);
	if(value >= 0) {
	    msleep(10);
	    value2 = bq27541_battery_read_temperature(di);
        if( abs(value2-value) <= 10 && value >= TEMP_LIMIT_LOW && value <= TEMP_LIMIT_HIGH ) {
            cache.temperature = value;
        } else {
		    if(di->cache.temperature == 0)
                cache.temperature = TEMP_DEFAULT;
        }
	}
#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.temperature = %d\n",__FUNCTION__,__LINE__,cache.temperature);
	dev_err(di->dev, "zxz  CCCC bq27541_update, cache.temperature=%d\n",cache.temperature);
#endif

	/*get fcc*/
	value = bq27541_battery_read_fcc(di);
	if(value >= 0 ){
		cache.charge_full = value;
	}
#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.charge_full = %d\n",__FUNCTION__,__LINE__,cache.charge_full);
	dev_err(di->dev, "zxz  DDDD bq27541_update, cache.charge_full=%d\n",cache.charge_full);
#endif

    /* time to empty */
	value =	bq27541_battery_read_time(di,bq27541CMD_TTE_LSB);
	if(value >= 0) {
		cache.time_to_empty = value;
	}
#ifdef DEBUG_UPDATE_VALUE
	printk("%s,%d,cache.time_to_empty = %d\n",__FUNCTION__,__LINE__,cache.time_to_empty);
	dev_err(di->dev, "zxz  EEEE bq27541_update, cache.time_to_empty=%d\n",cache.time_to_empty);
#endif

	/*Modified by T2M Q.Z
	* For 3000mah/4000mah change.
	*/

battery_not_exist:
	if(!battery_exist){
		cache.capacity = DEFAULT_CAPACITY;
/*Modify-Begin ,task-5373326 ,T2M zxz,2017/10/20, power off when use fake battery*/
		cache.voltage_now = 3800000;
/*Modify-End ,task-5373326 ,T2M zxz,2017/10/20, power off when use fake battery*/
		cache.energy = 0;
		cache.charge_now = 0;
		cache.health = POWER_SUPPLY_HEALTH_UNKNOWN;
/*Add-BEGIN by T2M.xcm. 2015.5.15 1003151 power off auto.*/
		cache.temperature = TEMP_DEFAULT;
//Add-END by T2M.xcm
		cache.charge_full = 0;
		cache.time_to_empty = 0;
	}
    value = bq27541_battery_read_cc(di);
    if(value >= 0) {
        cache.cycle_count = value;
    }
#ifdef DEBUG_UPDATE_VALUE
    dev_info(di->dev, "%s,%d,cycle_count = %d\n",__FUNCTION__,__LINE__, cache.cycle_count);
#endif
	value = bq27541_battery_read_remaining_life(di);
    if(value >= 0) {
        cache.remaining_life = value;
    }
#ifdef DEBUG_UPDATE_VALUE
    dev_info(di->dev, "%s,%d,remaining_life = %d\n",__FUNCTION__,__LINE__, cache.remaining_life);
#endif

#ifdef DUMMY_GAUGE
		cache.capacity = DEFAULT_CAPACITY;
/*Modify-Begin ,task-5373326 ,T2M zxz,2017/10/20, power off when use fake battery*/
		cache.voltage_now = 3800000;
/*Modify-End ,task-5373326 ,T2M zxz,2017/10/20, power off when use fake battery*/
		cache.energy = 0;
		cache.charge_now = 0;
		cache.health = POWER_SUPPLY_HEALTH_UNKNOWN;
/*Add-BEGIN by T2M.xcm. 2015.5.15 1003151 power off auto.*/
		cache.temperature = TEMP_DEFAULT;
//Add-END by T2M.xcm
		cache.charge_full = 0;
		cache.time_to_empty = 0;

#endif
#ifdef _FAIL_CNT
	dev_err(di->dev, "zxz battery_exist %d cache.capacity %d cache.voltage_now %d cache.charge_now %d cache.temperature %d fail_count %d read_count %d ,id_fail_count %d Last_fail_bat_id 0x%x\n",battery_exist,cache.capacity,cache.voltage_now,cache.charge_now,cache.temperature,di->fail_count,di->read_count,di->id_fail_count,di->Last_fail_bat_id);
#endif
	if(memcmp(&di->cache, &cache, sizeof(cache)) != 0){
		di->cache = cache;
		power_supply_changed(di->bat);
	}
}

static void bq27541_battery_poll(struct work_struct *work)
{
	struct bq27541_device_info *di =
		container_of(work, struct bq27541_device_info, battery_work.work);

#ifdef DEBUG_BQ
	printk("++%s, %d\n",__FUNCTION__ , __LINE__);
      dev_err(di->dev, "zxz  bq27541_battery_poll  Begin  \n");

#endif
#ifdef ZXZ_BQ27541
      dev_err(di->dev, "zxz  bq27541_battery_poll  Begin  \n");
#endif
    // jason.kim 2015.10.15 for canceling the polling
	mutex_lock(&di->poll_lock);

    if (di->poll_enable) {
        if (!di->hot_swap)
            bq27541_update(di);
        schedule_delayed_work(&di->battery_work, di->poll_interval * HZ);
	}
	mutex_unlock(&di->poll_lock);

#ifdef DEBUG_BQ
    dev_info(di->dev, "--%s, %d\n",__FUNCTION__ , __LINE__);
#endif
#ifdef ZXZ_BQ27541
	dev_err(di->dev, "zxz  bq27541_battery_poll  End \n");
#endif
}

/* PLATFORM]-add by T2MNB.ZXZ,add hotswap feature,2018/06/27,defect-6502875,Begin */
static void bq27541_battery_poll_start(struct bq27541_device_info *di, int ptime)
{
    mutex_lock(&di->poll_lock);
    di->poll_enable = 1;
    mutex_unlock(&di->poll_lock);
    dev_info(di->dev, "%s: schedule_delayed_work\n", __func__);
    schedule_delayed_work(&di->battery_work, ptime * HZ);
}

static void bq27541_battery_poll_stop(struct bq27541_device_info *di)
{
    mutex_lock(&di->poll_lock);
    di->poll_enable = 0;
    mutex_unlock(&di->poll_lock);
    if (delayed_work_pending(&di->battery_work)) {
        dev_info(di->dev, "%s: cancel_delayed_work\n", __func__);
        cancel_delayed_work_sync(&di->battery_work);
    }
}

/* PLATFORM]-add by T2MNB.ZXZ,add hotswap feature,2018/06/27,defect-6502875,End */


static int bq27541_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	switch (prop) {
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int bq27541_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);

	if (di->cache.flags < 0)
	{
		#ifdef DEBUG_BQ
		printk("%s, %d ,return error !\n", __FUNCTION__,__LINE__);
		#endif
		return -ENODEV;
	}

	switch (psp) {
    case POWER_SUPPLY_PROP_PRESENT: // jason.kim 2015.10.15 report the battery present (POWER_SUPPLY_PROP_PRESENT)
        if (di->test_cap_enable)
            val->intval = 1;
        else
            val->intval = di->battery_exist;
        break;


	/*Remove by T2M Q.Z for PR1003449
	* Do not use status detection from bq27541.
	* Use smb1357 status detection.
	*/
	//case POWER_SUPPLY_PROP_STATUS:
	//	ret = bq27541_battery_status(di, val);
	//	break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		if(di->battery_exist)
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		else
			val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        if (di->test_voltage_enable)
    		ret = bq27541_simple_value(di->test_voltage, val);
		else
			ret = bq27541_simple_value(di->cache.voltage_now, val);

		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		//ret = bq27541_s16_value(di->cache.current_now, val);
		val->intval = (int)(s16)bq27541_battery_read_current(di);
        val->intval *= 1000; // uA
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
        if (di->test_cap_enable) {
            val->intval = di->test_cap;
        } else {
            ret = bq27541_battery_capacity(di, val);
        }
        if (val->intval < 20 || di->cache.current_now <= 0) // low battery or charging
            di->poll_interval = BATTERY_FUEL_GAUGER_SEC_LOW;
        else
            di->poll_interval = BATTERY_FUEL_GAUGER_SEC;
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = bq27541_battery_capacity_level(di, val);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
        if (di->cache.time_to_empty == 65535)
            val->intval = -1; // jason.kim 2015.11.16 for suppressing debug message
        else
            ret = bq27541_simple_value(di->cache.time_to_empty * 60, val);
		break;

	//case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	//	ret = bq27541_simple_value(di->cache.time_to_empty_avg, val);
	//	break;

	//case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
	//	ret = bq27541_simple_value(di->cache.time_to_full, val);
	//	break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = bq27541_simple_value(di->charge_design_full, val);
		break;

	//case POWER_SUPPLY_PROP_CYCLE_COUNT:
	//	ret = bq27541_simple_value(di->cache.cycle_count, val);
	//	break;

	//case POWER_SUPPLY_PROP_POWER_AVG:
	//	ret = bq27541_simple_value(di->cache.power_avg, val);
	//	break;

	case POWER_SUPPLY_PROP_TEMP:
		if (di->test_temp_enable) {
		   ret = bq27541_simple_value(di->test_temp, val);
		}
		else
		{
		   ret = bq27541_simple_value(di->cache.temperature, val);
		}
		val->intval -= _KELVIN; //  swich kelvin to Celsius
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = bq27541_simple_value(di->cache.charge_now, val);
		break;

	case POWER_SUPPLY_PROP_ENERGY_NOW:
		ret = bq27541_simple_value(di->cache.energy, val);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (di->test_health_enable)
			val->intval =  di->test_health;
		else
			ret = bq27541_simple_value(di->cache.health, val);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = bq27541_simple_value(di->cache.charge_full, val);
		break;
	default:
		return -EINVAL;
	}


#ifdef DEBUG_BQ
	printk("%s ,line=%d ,prop=%d ; val->intval=%d\n", __FUNCTION__,__LINE__,psp,val->intval);
#endif


	return ret;

}

static int bq27541_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0, update_psy = 0;
    struct bq27541_device_info *di = power_supply_get_drvdata(psy);
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	switch (prop) {
		case POWER_SUPPLY_PROP_CAPACITY:
			di->cache.capacity = val->intval;
			update_psy = 1;
			break;
		default:
			rc = -EINVAL;
	}

	if (!rc && update_psy)
		power_supply_changed(di->bat);
	return rc;
}

//+ jason.kim 2015.11.24 dump registers
static ssize_t show_dump_regs(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
	
    int value;
    char *_buf = buf;

    bq27541_battery_poll_stop(di);

    value = bq27541_read16(di, BQ27541_REG_CNTL);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_CNTL, value);
    value = bq27541_read16(di, BQ27541_REG_AR);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_AR, value);
    value = bq27541_read16(di, BQ27541_REG_UFSOC);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_UFSOC, value);
    value = bq27541_read16(di, BQ27541_REG_TEMP);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_TEMP, value);
    value = bq27541_read16(di, BQ27541_REG_VOLT);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_VOLT, value);
    value = bq27541_read16(di, BQ27541_REG_FLAGS);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_FLAGS, value);
    value = bq27541_read16(di, BQ27541_REG_NAC);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_NAC, value);
    value = bq27541_read16(di, BQ27541_REG_FAC);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_FAC, value);
    value = bq27541_read16(di, BQ27541_REG_RM);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_RM, value);
    value = bq27541_read16(di, BQ27541_REG_FCC);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_FCC, value);
    value = bq27541_read16(di, BQ27541_REG_AI);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_AI, value);
    value = bq27541_read16(di, BQ27541_REG_TTE);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_TTE, value);
    value = bq27541_read16(di, BQ27541_REG_FFCC);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_FFCC, value);
    value = bq27541_read16(di, BQ27541_REG_SI);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_SI, value);
    value = bq27541_read16(di, BQ27541_REG_UFFCC);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_UFFCC, value);
    value = bq27541_read16(di, BQ27541_REG_MLI);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_MLI, value);
    value = bq27541_read16(di, BQ27541_REG_UFRM);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_UFRM, value);
    value = bq27541_read16(di, BQ27541_REG_FRM);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_FRM, value);
    value = bq27541_read16(di, BQ27541_REG_AP);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_AP, value);

    value = bq27541_read16(di, BQ27541_REG_INTTEMP);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_INTTEMP, value);
    value = bq27541_read16(di, BQ27541_REG_CC);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_CC, value);
    value = bq27541_read16(di, BQ27541_REG_SOC);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_SOC, value);
    value = bq27541_read16(di, BQ27541_REG_SOH);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_SOH, value);

    value = bq27541_read16(di, BQ27541_REG_PCHG);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_PCHG, value);
    value = bq27541_read16(di, BQ27541_REG_DOD0);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_DOD0, value);
/*Add begin Defect-6099797, for HDQ charge safe time */
    value = bq27541_read16(di, 0x3c);
    buf += sprintf(buf, "%02X %04X\n", 0x3c, value);
/*Add end Defect-6099797, for HDQ charge safe time */
    value = bq27541_read16(di, BQ27541_REG_SDSG);
    buf += sprintf(buf, "%02X %04X\n", BQ27541_REG_SDSG, value);

    bq27541_battery_poll_start(di, 0);
	return (buf-_buf);
}

//+ jason.kim 2015.12.07 serial number
static ssize_t store_serial_no(struct device *dev,  struct device_attribute *attr, const char *buf,  size_t count)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	if (!di->battery_exist)
        return 0;

    bq27541_battery_poll_stop(di);

    bq27541_battery_write_serial(di, buf);
    bq27541_battery_read_serial(di);
    dev_info(di->dev, "Serial=%s\n", di->serial_len ? di->serial_no : "unknown");

    bq27541_battery_poll_start(di, 0);
    return count;
}

static ssize_t show_serial_no(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);

#ifdef USE_SERIAL_32_BYTES
  printk("%s######\n", __func__);

	if(di->serial_len == NEED_SN_READ)
	{
		return sprintf(buf, "%s\n","unknown");
	}
	else
	{
	
		PM85_SERIAL *serial_number;
		serial_number = (PM85_SERIAL *)di->serial_no;
		
        printk( "%s::%d######di->serial_len 0x%x\n", __func__,__LINE__,di->serial_len);
		if(di->serial_len != NEED_SN_READ)
		{
//			char str_serial[20]={0,};
//			char str_manufacture_date[20]={0,};
			printk("%s::%d  :di->serial_len 0x%x ###### SN : %s MF %s\n", __func__,__LINE__,di->serial_len ,serial_number->SN,serial_number->MF_date);	

//			memcpy((void *)str_serial , (void *)serial_number->SN,serial_number->length_SN);
//			memcpy((void *)str_manufacture_date , (void *)serial_number->MF_date,serial_number->length_MF_date);
//			printk( "%s::%d###### SN : %s MF %s\n", __func__,__LINE__,str_serial,str_manufacture_date);		
			return sprintf(buf,"%s-%s\n",serial_number->SN,serial_number->MF_date);

		
		}else
		{
			return sprintf(buf,"%s\n","unknown");

		}
    //	return sprintf(buf, "%s\n", (di->serial_len && is_good_serial(di->serial_no)) ? di->serial_no : "unknown");
	}
#else
	if(di->serial_len == NEED_SN_READ)
	{
		return sprintf(buf, "%s\n","unknown");
	}
	else
	{
  	  return sprintf(buf, "%s\n", di->serial_len ? di->serial_no : "unknown");
	}
#endif
}
//+ jason.kim 2015.12.07 serial number

//+ jason.kim 2015.12.08 dump MFG-B
static ssize_t show_mfg_b(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
		#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
    bq27541_battery_dump_mfg_B(di);
    return 0;
}
//- jason.kim 2015.12.08 dump MFG-B

// jason.kim 2016.01.20 Check and update battery parameters
static ssize_t show_reset_battery(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);

    bq27541_battery_poll_stop(di);
    bq27541_battery_reset_param(di);
    bq27541_battery_poll_start(di, 0);
    return 0;
}

static ssize_t show_battery_remaining_life(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);

    return sprintf(buf, "%d\n", di->cache.remaining_life);
}
static ssize_t show_battery_cycle_count(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);

    return sprintf(buf, "%d\n", di->cache.cycle_count);
}
static ssize_t show_real_soc(struct device *dev, struct device_attribute *attr, char *buf) //  it add for stress test tool
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);

    return sprintf(buf, "%d", di->cache.capacity);
}

//+ jason.kim 2016.3.24 test temperature control
static ssize_t store_test_temp(struct device *dev,  struct device_attribute *attr, const char *buf,  size_t count)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
    int param, ret;
    ret = kstrtoint(buf, 10, &param);
    if (ret < 0)
        return count;

    // jason.kim 2016.6.2
    if (param == 1000) {
        di->test_temp_enable = 0;
        di->test_temp = 300+ _KELVIN;
    } else if (param == 1001) {
        di->test_temp_enable = 1;
        di->test_temp = 300+_KELVIN;
    } else
        di->test_temp = param + _KELVIN;
        dev_info(di->dev, "temp=%d\n", di->test_temp - _KELVIN);
    return count;
}

static ssize_t show_test_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
    return sprintf(buf, "test_temp %d\n", di->test_temp - _KELVIN);
}
//- jason.kim 2016.3.24 test temperature control

//+ jason.kim 2016.7.12 Test battery capacity
static ssize_t store_test_cap(struct device *dev,  struct device_attribute *attr, const char *buf,  size_t count)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
    int param, ret;
    ret = kstrtoint(buf, 10, &param);
    if (ret < 0)
        return count;

    // jason.kim 2016.6.2
    if (param == 1000) {
        di->test_cap_enable = 0;
        di->test_cap = 50;
    } else if (param == 1001) {
        di->test_cap_enable = 1;
        di->test_cap = 50;
    } else
        di->test_cap = param;

    dev_info(di->dev, "test_cap =%d\n", di->test_cap);
    return count;
}

static ssize_t show_test_cap(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
    return sprintf(buf, "test_cap %d\n", di->test_cap);
}
//- jason.kim 2016.7.12 Test battery capacity

//+ jason.kim 2016.7.18 Test battery voltage
static ssize_t store_test_voltage(struct device *dev,  struct device_attribute *attr, const char *buf,  size_t count)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
    int param, ret;
    ret = kstrtoint(buf, 10, &param);
    if (ret < 0)
        return count;

    // jason.kim 2016.6.2
    if (param == 1000) {
        di->test_voltage_enable = 0;
        di->test_voltage = 3700 * 1000;
    } else if (param == 1001) {
        di->test_voltage_enable = 1;
        di->test_voltage = 3700 * 1000;
    } else
        di->test_voltage = param * 1000;
    dev_info(di->dev, "test_voltage =%d\n", di->test_voltage / 1000);
    return count;
}

static ssize_t show_test_voltage(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
    return sprintf(buf, "test_voltage %d\n", di->test_voltage / 1000);
}
//- jason.kim 2016.7.18 Test battery voltage

static ssize_t store_test_health(struct device *dev,  struct device_attribute *attr, const char *buf,  size_t count)
{
	char *str_power_supply[]=
	{
		"POWER_SUPPLY_HEALTH_UNKNOWN",
		"POWER_SUPPLY_HEALTH_GOOD",
		"POWER_SUPPLY_HEALTH_OVERHEAT",
		"POWER_SUPPLY_HEALTH_DEAD",
		"POWER_SUPPLY_HEALTH_OVERVOLTAGE",
		"POWER_SUPPLY_HEALTH_UNSPEC_FAILURE",
		"POWER_SUPPLY_HEALTH_COLD",
		"POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE",
		"POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE",
		"POWER_SUPPLY_HEALTH_WARM",
		"POWER_SUPPLY_HEALTH_COOL",
	};

	struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
    int param, ret;
    ret = kstrtoint(buf, 10, &param);
    if (ret < 0)
        return count;

    // jason.kim 2016.6.2
    if (param == 1000) {
        di->test_health_enable = 0;
        di->test_health = POWER_SUPPLY_HEALTH_GOOD;
    } else if (param == 1001) {
         di->test_health_enable = 1;
        di->test_health = POWER_SUPPLY_HEALTH_GOOD;
    } else
    {
    	if(param >10)
    	{
   		 	dev_info(di->dev, "Error input param %d \n", param);
       		return count;
    	}
		else
        	di->test_health = param;
    }

    dev_info(di->dev, "Enable:: %d  test_health =%d [%s]\n",  di->test_health_enable ,di->test_health ,str_power_supply[di->test_health]);
    return count;
}

static ssize_t show_test_health(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *str_power_supply[]=
	{
		"POWER_SUPPLY_HEALTH_UNKNOWN",
		"POWER_SUPPLY_HEALTH_GOOD",
		"POWER_SUPPLY_HEALTH_OVERHEAT",
		"POWER_SUPPLY_HEALTH_DEAD",
		"POWER_SUPPLY_HEALTH_OVERVOLTAGE",
		"POWER_SUPPLY_HEALTH_UNSPEC_FAILURE",
		"POWER_SUPPLY_HEALTH_COLD",
		"POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE",
		"POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE",
		"POWER_SUPPLY_HEALTH_WARM",
		"POWER_SUPPLY_HEALTH_COOL",
	};
    struct power_supply *psy = dev_get_drvdata(dev);
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
    return sprintf(buf, "Enable:: %d test_health =%d [%s]\n", di->test_health_enable , di->test_health ,str_power_supply[di->test_health]);
}



static struct device_attribute bq27541_battery_attrs[] = {
	__ATTR(dump_regs, 0444, show_dump_regs, NULL), // jason.kim 2015.03.02 dump all registers
	__ATTR(serial_no, 0664, show_serial_no, store_serial_no), // jason.kim 2015.12.07 serial number
	__ATTR(mfg_b, 0444, show_mfg_b, NULL), // jason.kim 2015.12.08 dump MFG-B
	__ATTR(reset, 0444, show_reset_battery, NULL), // jason.kim 2016.01.20 Check and update battery parameters
    __ATTR(remaining_life, S_IRUGO, show_battery_remaining_life, NULL),
    __ATTR(cycle_count, S_IRUGO, show_battery_cycle_count, NULL),
    __ATTR(real_soc, 0444, show_real_soc, NULL), // [robin.cho] it add for strees test tool:show real SOC
    __ATTR(test_temp, 0664, show_test_temp, store_test_temp), // jason.kim 2016.3.25 SW temperature control
    __ATTR(test_cap, 0664, show_test_cap, store_test_cap), // jason.kim 2016.7.12 Test battery capacity
    __ATTR(test_voltage, 0664, show_test_voltage, store_test_voltage), // jason.kim 2016.7.18 Test battery voltage
    __ATTR(test_health, 0664, show_test_health, store_test_health), // robin.cho
};

static int bq27541_create_attrs(struct device *dev)
{
	unsigned int i;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	for (i = 0; i < ARRAY_SIZE(bq27541_battery_attrs); i++)
		if (device_create_file(dev, &bq27541_battery_attrs[i]))
			goto bq27541_create_attrs_failed;

	return 0;

bq27541_create_attrs_failed:
	dev_err(dev, "failed creating bq27541 battery attrs.\n");
	while (i--)
		device_remove_file(dev, &bq27541_battery_attrs[i]);

	return -EIO;
}

static void bq27541_remove_attrs(struct device *dev)
{
	unsigned int i;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	for (i = 0; i < ARRAY_SIZE(bq27541_battery_attrs); i++)
		device_remove_file(dev, &bq27541_battery_attrs[i]);
}
//- jason.kim 2015.11.24 dump registers



/* PLATFORM]-add by T2MNB.ZXZ,add hotswap feature,2018/06/27,defect-6502875,Begin */
void bq27541_poll_stop(struct power_supply *psy)
{
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
	bq27541_battery_poll_stop(di);
}

int bq27541_read_rsoc(struct power_supply *psy)
{
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
	return bq27541_battery_read_rsoc(di);
}

int bq27541_read_temp(struct power_supply *psy)
{
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);
	return bq27541_battery_read_temperature(di);
}



//void bq24086_set_lowbattery(bool low_battery);
void bq27541_set_hot_swap(struct power_supply *psy, int low_battery)
{
	struct bq27541_device_info *di = power_supply_get_drvdata(psy);

	//bq24086_set_lowbattery(low_battery);
	if (low_battery) {
		di->hot_swap = true;
        bq27541_battery_poll_stop(di);
        di->battery_exist = 0;
        di->serial_len = NEED_SN_READ;
        //di->serial_retry = 0;
        di->battery_init = 0;
	   di->poll_interval = BATTERY_FUEL_GAUGER_SEC_LOW;
        power_supply_changed(di->bat);
	} else {
		di->hot_swap = false;
        bq27541_battery_poll_start(di, 0);
	}

}
/* PLATFORM]-add by T2MNB.ZXZ,add hotswap feature,2018/06/27,defect-6502875,End */

static const struct power_supply_desc batt_psy_desc = {
	.name = "bq27541_battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = bq27541_battery_props,
	.num_properties = ARRAY_SIZE(bq27541_battery_props),
	.get_property = bq27541_battery_get_property,
	.set_property = bq27541_battery_set_property,
	.property_is_writeable = bq27541_battery_is_writeable,
};

static int bq27541_powersupply_init(struct bq27541_device_info *di)
{

	struct power_supply_config batt_cfg = {};
	int ret = 0;
	
    #ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
    #endif

	batt_cfg.drv_data = di;
	batt_cfg.of_node = di->dev->of_node;
	di->bat = devm_power_supply_register(di->dev,
					   &batt_psy_desc,
					   &batt_cfg);
	if (IS_ERR(di->bat)) {
		pr_err("Couldn't register battery power supply bq27541 !\n");
		return PTR_ERR(di->bat);
	}

	#ifdef DEBUG_BQ
	printk("%s,%d\n",__FUNCTION__,__LINE__);
	#endif

	#ifdef _FAIL_CNT
	di->read_count=0;
	di->fail_count=0;
	di->id_fail_count=0;
	di->Last_fail_bat_id=0;
	#endif

	
	INIT_DELAYED_WORK(&di->battery_work, bq27541_battery_poll);
	// mutex_init(&di->lock);

    // jason.kim 2015.11.24 dump registers
    ret = bq27541_create_attrs(di->dev);
    if (ret) {
        dev_err(di->dev, "fail to create main sysfs: %d\n", ret);
        return ret;
    }
	return 0;
}

static int hdq_parse_dt(struct platform_device *pdev,
			struct bq27541_device_info *pdata)
{
    //int rc;
	struct device_node *np = pdev->dev.of_node;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	pdata->pin = of_get_named_gpio(np,"qcom,hdq-gpio", 0);
	if(!pdata->pin){
		dev_err(&pdev->dev, "Unable to get hdq_gpio\n");
		return -ENODEV;
	}

	pdev->dev.platform_data = pdata;

	return 0;
}

static int bq27541_probe(struct platform_device *pdev)
{
	int err;
	int value;
	struct bq27541_device_info *di = pdev->dev.platform_data;
	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif
	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&pdev->dev, "failed to allocate device info data\n");
		return  -ENOMEM;
	}
#ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe 00000000000 \n", __FUNCTION__,__LINE__);
#endif
	platform_set_drvdata(pdev, di); // jason.kim 2015.10.15

	/*hdq parse dt*/
	err = hdq_parse_dt(pdev,di);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to parse DT\n");
		return err;
	}
#ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe 11111111 \n", __FUNCTION__,__LINE__);
#endif
	/*request gpio*/
	if(gpio_is_valid(di->pin)){
		err = gpio_request(di->pin, "hdq");
		if (err) {
			dev_err(&pdev->dev, "gpio_request (pin) failed\n");
			goto free_gpio;
		}
	}
#ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe 22222222222 ---di->pin =%d \n", __FUNCTION__,__LINE__,di->pin);
#endif
	mutex_init(&di->poll_lock); // jason.kim 2015.10.15
	spin_lock_init(&di->hdq_lock);

	/* timer */
	//my_timer->data = 0xff;
	//my_timer->function = &timer_function;
	//my_timer->expires = jiffies + HZ/10000;
	//init_timer(my_timer);

	/*fill the struct*/
	di->battery_exist = 0;
	di->dev = &pdev->dev;
	di->chip = BQ27541;
	di->bus.read = &hdq_gpio_read;
	di->bus.write = &hdq_gpio_write;
	di->poll_interval = BATTERY_FUEL_GAUGER_SEC;
#ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe 3333333 \n", __FUNCTION__,__LINE__);
#endif

    err=bq27541_pinctrl_init(di);
	if(err)
	dev_err(di->dev, " bq27541_pinctrl_init failed \n");

	err = bq27541_powersupply_init(di);
	if (err)
		goto err_free;
#ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe 4444444444444 \n", __FUNCTION__,__LINE__);
#endif
#ifdef DEBUG_INIT_VALUE
    dev_info(di->dev, "Initial capacity %d, current %d\n",
        bq27541_battery_read_rsoc(di),
        (int)((s16)bq27541_battery_read_current(di)));
#endif
#ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe 555555555 \n", __FUNCTION__,__LINE__);
#endif

/*Add begin Defect-6099797, for HDQ charge safe time */
    value = bq27541_battery_read_dcap(di);
	if(value >= 0)
        di->charge_design_full = value;
        BatteryDesignCap=di->charge_design_full;
	printk(" %s,%d,BatteryDesignCap = %d\n",__FUNCTION__,__LINE__,BatteryDesignCap);
/*Add end Defect-6099797, for HDQ charge safe time */

	di->serial_len = NEED_SN_READ;


    bq27541_battery_poll_start(di, 0);
	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);


#ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe success !!!!!!\n", __FUNCTION__,__LINE__);
#endif

	return 0;

 err_free:
	 kfree(di);
	 #ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe FAIL !!!!!!\n", __FUNCTION__,__LINE__);
	#endif
	 return err;

 free_gpio:
	gpio_free(di->pin);
	#ifdef DEBUG_BQ
	printk("%s ,line=%d ,probe FAIL !!!!!!\n", __FUNCTION__,__LINE__);
	#endif
	return err;
}

// jason.kim 2015.10.15
static int bq27541_remove(struct platform_device *pdev)
{
	struct bq27541_device_info *di = dev_get_drvdata(&pdev->dev);

#ifdef DEBUG_BQ
    printk("%s,%d\n",__FUNCTION__,__LINE__);
#endif
    bq27541_battery_poll_stop(di);

    bq27541_remove_attrs(di->dev); // jason.kim 2015.11.24 dump registers
	power_supply_unregister(di->bat);
	// mutex_destroy(&di->lock);
	mutex_destroy(&di->poll_lock);
	gpio_free(di->pin);

	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

static int bq27541_battery_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bq27541_device_info *di = platform_get_drvdata(pdev);

	dev_info(dev, "++%s\n", __func__);

    bq27541_battery_poll_stop(di);

	dev_info(dev, "--%s\n", __func__);
	return 0;
}


static int bq27541_battery_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bq27541_device_info *di = platform_get_drvdata(pdev);

	#ifdef DEBUG_BQ
	printk("%s, %d,  \n",__FUNCTION__ , __LINE__);
	#endif


/* PLATFORM]-add by T2MNB.ZXZ,add hotswap feature,2018/06/27,defect-6502875 */
	if(!di->hot_swap) {
		bq27541_battery_poll_start(di, 0);
	}
	dev_info(dev, "--%s\n", __func__);
	return 0;
}
//-- jason.kim 2015.10.15

static const struct dev_pm_ops bq27541_battery_pm_ops = {
		 .suspend		 = bq27541_battery_suspend,
		 .resume		 = bq27541_battery_resume,
};

static struct of_device_id hdq_gpio_dt_ids[] = {
	{ .compatible = "qcom,bq27541-fuel-hdq" },
	{},
};
MODULE_DEVICE_TABLE(of, hdq_gpio_dt_ids);

static struct platform_driver bq27541_driver = {
	.driver = {
		.name	= "bq27541",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(hdq_gpio_dt_ids),
		.pm = &bq27541_battery_pm_ops,
	},
	.probe = bq27541_probe,
	.remove = bq27541_remove,
};

module_platform_driver(bq27541_driver);

MODULE_DESCRIPTION("GPIO HDQ driver");
MODULE_AUTHOR("AaronZeng <qiang.zeng@T2mobile.com>");
MODULE_LICENSE("GPL");
