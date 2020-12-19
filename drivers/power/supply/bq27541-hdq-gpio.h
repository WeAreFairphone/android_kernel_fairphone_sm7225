/*
 * hdq-gpio interface to platform code
 *
 * Copyright (C) 2014 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */
#ifndef _LINUX_BQ27541_HDQ_GPIO_H
#define _LINUX_BQ27541_HDQ_GPIO_H
	
#define DRIVER_VERSION			"1.1.0"
	/* Bq27541 standard/extended data commands */
#define bq27541CMD_CNTL_LSB  0x00
#define bq27541CMD_CNTL_MSB  0x01
#define bq27541CMD_AR_LSB	 0x02
#define bq27541CMD_AR_MSB	 0x03
#define bq27541CMD_ARTTE_LSB 0x04
#define bq27541CMD_ARTTE_MSB 0x05
#define bq27541CMD_TEMP_LSB  0x06
#define bq27541CMD_TEMP_MSB  0x07
#define bq27541CMD_VOLT_LSB  0x08
#define bq27541CMD_VOLT_MSB  0x09
#define bq27541CMD_FLAGS_LSB 0x0A
#define bq27541CMD_FLAGS_MSB 0x0B
#define bq27541CMD_NAC_LSB	 0x0C
#define bq27541CMD_NAC_MSB	 0x0D
#define bq27541CMD_FAC_LSB	 0x0E
#define bq27541CMD_FAC_MSB	 0x0F
#define bq27541CMD_RM_LSB	 0x10
#define bq27541CMD_RM_MSB	 0x11
#define bq27541CMD_FCC_LSB	 0x12
#define bq27541CMD_FCC_MSB	 0x13
#define bq27541CMD_AI_LSB	 0x14
#define bq27541CMD_AI_MSB	 0x15
#define bq27541CMD_TTE_LSB	 0x16
#define bq27541CMD_TTE_MSB	 0x17
#define bq27541CMD_TTF_LSB	 0x18
#define bq27541CMD_TTF_MSB	 0x19
#define bq27541CMD_SI_LSB	 0x1A
#define bq27541CMD_SI_MSB	 0x1B
#define bq27541CMD_STTE_LSB  0x1C
#define bq27541CMD_STTE_MSB  0x1D
#define bq27541CMD_MLI_LSB	 0x1E
#define bq27541CMD_MLI_MSB	 0x1F
#define bq27541CMD_MLTTE_LSB 0x20
#define bq27541CMD_MLTTE_MSB 0x21
#define bq27541CMD_AE_LSB	 0x22
#define bq27541CMD_AE_MSB	 0x23
#define bq27541CMD_AP_LSB	 0x24
#define bq27541CMD_AP_MSB	 0x25
#define bq27541CMD_TTECP_LSB 0x26
#define bq27541CMD_TTECP_MSB 0x27
#define bq27541CMD_RSVD_LSB  0x28
#define bq27541CMD_RSVD_MSB  0x29
#define bq27541CMD_CC_LSB	 0x2A
#define bq27541CMD_CC_MSB	 0x2B
#define bq27541CMD_SOC_LSB	 0x2C
#define bq27541CMD_SOC_MSB	 0x2D
#define bq27541CMD_SOH_LSB	 0x2E
#define bq27541CMD_DCAP_LSB  0x3C
#define bq27541CMD_DCAP_MSB  0x3D
#define bq27541CMD_DFCLS	 0x3E
#define bq27541CMD_DFBLK	 0x3F
#define bq27541CMD_ADF		 0x40
#define bq27541CMD_ACKSDFD	 0x54
#define bq27541CMD_DFDCKS	 0x60
#define bq27541CMD_DFDCNTL	 0x61
#define bq27541CMD_DNAMELEN  0x62
#define bq27541CMD_DNAME	 0x63
	
#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_SOCF		BIT(1) /* State-of-Charge threshold final */
#define BQ27541_FLAG_SOC1		BIT(2) /* State-of-Charge threshold 1 */
#define BQ27541_FLAG_FC 		BIT(9)
#define BQ27541_FLAG_OTD		BIT(14)
#define BQ27541_FLAG_OTC		BIT(15)
	
#define BQ27541_CS_SS			BIT(13)
#define BQ27541_CS_DLOGEN		BIT(15)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER       0x0002
#define BQ27541_SUBCMD_HW_VER       0x0003
#define BQ27541_SUBCMD_DF_CSUM      0x0004
#define BQ27541_SUBCMD_PREV_MACW    0x0007
#define BQ27541_SUBCMD_CHEM_ID      0x0008
#define BQ27541_SUBCMD_BD_OFFSET    0x0009
#define BQ27541_SUBCMD_INT_OFFSET   0x000a
#define BQ27541_SUBCMD_CC_VER       0x000b
#define BQ27541_SUBCMD_OCV          0x000c
#define BQ27541_SUBCMD_BAT_INS      0x000d
#define BQ27541_SUBCMD_BAT_REM      0x000e
#define BQ27541_SUBCMD_SET_HIB      0x0011
#define BQ27541_SUBCMD_CLR_HIB      0x0012
#define BQ27541_SUBCMD_SET_SLP      0x0013
#define BQ27541_SUBCMD_CLR_SLP      0x0014
#define BQ27541_SUBCMD_FCT_RES      0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED       0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE     0x0040
#define BQ27541_SUBCMD_RESET        0x0041

#define BQ27542_DEVICE_ID 0x542

#include <linux/power_supply.h>

static enum power_supply_property bq27541_battery_props[] = {
	//POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT, // jason.kim 2015.10.15
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	//POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	//POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	//POWER_SUPPLY_PROP_POWER_AVG,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
};

#if 0
int bq27541_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val);

void bq27541_external_power_changed(struct power_supply *psy);
#endif


#endif /* _LINUX_HDQ_GPIO_H */

