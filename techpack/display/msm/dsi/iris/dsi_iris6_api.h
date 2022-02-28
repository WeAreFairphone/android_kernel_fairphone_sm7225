#ifndef _DSI_IRIS_API_H_
#define _DSI_IRIS_API_H_

#include "dsi_display.h"
#include "dsi_iris6_def.h"


int iris_parse_param(struct device_node *of_node, struct dsi_panel *panel);
void iris_init(struct dsi_display *display, struct dsi_panel *panel);
void iris_deinit(void);
int iris_pt_send_panel_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset);
int iris_abyp_send_panel_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset);
int iris_enable(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds);
int iris_disable(struct dsi_panel *panel, struct dsi_panel_cmd_set *off_cmds);
void iris_prepare(void);
void iris_send_cont_splash(uint32_t type);
int iris_get_abyp_mode_blocking(void);
int iris_get_abyp_mode_nonblocking(void);
int iris_switch(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *switch_cmds,
		struct dsi_mode_info *mode_info);

int iris_operate_conf(struct msm_iris_operate_value *argp);
int iris_operate_tool(struct msm_iris_operate_value *argp);

int iris_vc_id_get(int type);
int iris_status_get(struct dsi_display_ctrl *ctrl, struct dsi_panel *panel);
int iris_esd_ctrl_get(void);
int iris_platform_get(void);
int iris_hdr_enable_get(void);
bool iris_dspp_dirty(void);
int iris_update_backlight(u32 bl_lvl);
int iris_prepare_for_kickoff(void *phys_enc);
int iris_kickoff(void *phys_enc);

int iris_request_gpio(void);
int iris_reset(void);
int iris_reset_off(void);
int iris_update_dc_brightness(void);
int iris_dc_on_off_pending(void);
void iris_init_tm_points_lut(void);
void iris_dma_ch1_trigger(bool en, bool chain);
int iris_clk_enable(bool enable);
int iris_vdd_enable(int enable);

#endif // _DSI_IRIS_API_H_
