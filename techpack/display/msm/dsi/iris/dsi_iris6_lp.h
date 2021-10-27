#ifndef _DSI_IRIS_LP_H_
#define _DSI_IRIS_LP_H_
#include "dsi_iris6_i3c.h"

/* option IDs */
#define ID_SYS_DPG_CTRL 0xF0
#define ID_SYS_MPG_CTRL 0xF1
#define ID_SYS_ABP_CTRL 0xF2
#define ID_SYS_ULPS_CTRL 0xF3
#define ID_SYS_ALT_CTRL0 0xF4
#define ID_SYS_DMA_TRIG_CTRL 0xF6

#define ID_SYS_MPG_OFF 0x06

#define ID_MIPI_ENTER_DBP 0xD0
#define ID_MIPI_EXIT_DBP 0xD1
#define ID_MIPI_BYPASS_CTRL_REG 0xD0
#define ID_MIPI_BYPASS_CTRL_DMA 0xE2

#define ID_DMA_HDR 0xE2

/* regs */
#define IRIS_RUN_STATUS 0xf1100204
#define IRIS_TOPPMU_STATUS 0xf0000064

enum iris_pmu_domain {
	MIPI_PWR = (0x1 << 1),
	PQ_PWR = (0x1 << 2),
	HDR_PWR = (0x1 << 3),
	HDR_COLOR_PWR = (0x1 << 4),
};

enum iris_ulps_sel {
	ULPS_NONE = 0x0,
	ULPS_MAIN = 0x1,
};

/* init iris low power*/
void iris_lp_enable_pre(void);
void iris_lp_enable_post(void);

void iris_lp_init(void);

/* dynamic power gating set */
void iris_dynamic_power_set(bool enable, bool chain);

/* dynamic power gating get */
bool iris_dynamic_power_get(void);

/* power on & off HDR domain
 *   type: 0 -- power off HDR & HDR_COLOR
 *         1 -- power on HDR, power off HDR_COLOER
 *         2 -- power on HDR & HDR_COLOR
 */
int iris_pmu_hdr_set(int type, bool chain);

/* send one wired commands via GPIO */
void iris_one_wired_cmd_send(struct dsi_panel *panel, int cmd);

int iris_one_wired_cmd_init(struct dsi_panel *panel);

int iris_abyp_switch_proc(struct dsi_display *display, int mode, bool blocking);

void iris_ulps_set(bool enable, bool chain);

bool iris_ulps_enable_get(void);

int iris_dbg_init_lp(struct dsi_display *display);

int iris_lp_abyp_enter(bool blocking);

int iris_lp_abyp_exit(bool blocking);

void iris_lp_setting_off(void);

/* Get Iris lightup opt */
int iris_lightup_opt_get(void);

int iris_lightup_exit_abyp(bool one_wired, bool reset);

void iris_dbp_switch(bool enter);

void iris_dpg_event(bool start, bool chain);

void iris_force_abyp_mode(uint8_t mode);
void iris_dsi_recover(void);
void iris_dump_regs(u32 regs[], u32 len);

#endif // _DSI_IRIS_LP_H_
