#ifndef _DSI_IRIS_PQ_H_
#define _DSI_IRIS_PQ_H_

enum {
	IRIS_LCE_GRAPHIC = 0x00,
	IRIS_LCE_VIDEO,
};

enum {
	IRIS_COLOR_TEMP_OFF = 0x00,
	IRIS_COLOR_TEMP_MANUL,
	IRIS_COLOR_TEMP_AUTO,
};

enum {
	IRIS_MAGENTA_GAIN_TYPE = 0,
	IRIS_RED_GAIN_TYPE,
	IRIS_YELLOW_GAIN_TYPE,
	IRIS_GREEN_GAIN_TYPE,
	IRIS_BLUE_GAIN_TYPE,
	IRIS_CYAN_GAIN_TYPE,
};

#define IP_OPT_MAX				20

void iris_set_skip_dma(bool skip);

void iris_pq_parameter_init(void);

void iris_dpp_cmcsc_level_set(u32 *csc_value);

void iris_dpp_demo_window_set(u8 enable, u8 owAndGamma, u32 xWindow, u32 yWindow);

void iris_dpp_3dlut_send(u8 lut_optid, uint8_t chain);

void iris_dpp_3dlut_interpolation(u8 enable, u32 interSrc, u32 lutgain, bool bcommit);

void iris_dpp_3dlut_gain(u32 lutgain, bool bcommit);

void iris_dpp_fingerDisplay_set(u8 enable, u8 shapeInFill, u32 radius, u32 position, u32 fillcolor);

void iris_dpp_fadeinout_enable(u8 enable);

void iris_dpp_fadeinout_step(u8 enable, u32 fadestep);

void iris_dpp_gammamode_set(u32 gammmode, u32 gammaIndex);

void iris_cm_colortemp_mode_set(u32 mode, bool bcommit);

void iris_cm_color_temp_set(void);

int iris_dcDimming_backlight_set(u32 *values);

void iris_dcDimming_enable(u8 enable);

void iris_dcDimming_set(u32 rGain, u32 gGain, u32 bGain);

void iris_scurve_enable_set(u32 level);

void iris_dpp_path_set(u32 path_mux);

void iris_dport_disable(bool bdisable, uint8_t chain);

void iris_dpp_digitalBypass_metaEn(u8 metaEn);   //metaEn = 0, or 1
u32 iris_cm_ratio_set_for_iic(void);

void iris_cm_color_gamut_pre_set(u32 source_switch);

void iris_cm_color_gamut_set(u32 level, bool bcommit);

void iris_dpp_gamma_set(void);

void iris_dbc_level_set(u32 level);

void iris_pwm_freq_set(u32 value);

void iris_pwm_enable_set(bool enable);

void iris_dbc_bl_user_set(u32 value);

void iris_dbc_led0d_gain_set(u32 value);

void iris_reading_mode_set(u32 *csc_value);

void iris_pp_datapath_set(bool bEn);

void iris_dbc_compenk_set(u8 lut_table_index);
void iris_sdr2hdr_level_set(u32 level, bool bcommit);
void iris_panel_nits_set(u32 bl_ratio, bool bSystemRestore, int level);
void iris_scaler_filter_update(u8 scaler_type, u32 level);

void iris_init_ipopt_t(void);
//void iris_hdr_csc_prepare(void);
void iris_hdr_csc_complete(int step);
void iris_hdr_csc_frame_ready(void);

int32_t iris_update_ip_opt(uint8_t ip,
		uint8_t opt_id, uint8_t chain);

int iris_dbg_init_pq(struct dsi_display *display);

u8 iris_get_dbc_lut_index(void);

struct iris_setting_info *iris_get_setting(void);

void iris_set_HDR10_YCoCg(bool val);

bool iris_get_debug_cap(void);

void iris_set_debug_cap(bool val);

bool iris_get_dportdis_debug(void);

void iris_set_dportdis_debug(bool val);

void iris_set_sdr2hdr_mode(u8 val);

void iris_quality_setting_off(void);

struct iris_ambient_info *iris_get_ambient_lut(void);

void iris_scaler_gamma_enable(bool lightup_en, u32 level);

void iris_clear_init_trigger(void);

bool iris_sdr2hdr_valid(u32 level);

void iris_hdr_ai_enable(bool bEn);
void iris_hdr_ai_input_al(u32 al_value);
void iris_hdr_ai_input_panel_nits(u32 nits_value);
void iris_hdr_ai_input_bl(u32 bl_value);
void iris_hdr_ai_ctrl_tm_y(bool bEn);
void iris_hdr_ai_ctrl_tm_k(bool bEn);
void iris_hdr_ai_ctrl_cam_gain(bool bEn);
void iris_hdr_ai_ctrl_graphic_weight(bool bEn);
void iris_hdr_ai_ctrl_fade(u32 FadeEn);
void iris_hdr_ai_ctrl_s_curve(bool bEn);
void iris_al_enable(bool enable);
void iris_lux_set(u32 level);
void iris_sdr2hdr_maxcll_set(u32 maxcll);
void iris_cm_csc_restore(bool commit);
void iris_sdr2hdr_lce(u32 value);
void iris_sdr2hdr_scurve(u32 value);
void iris_sdr2hdr_tf_coef_set(u32 value);
void iris_sdr2hdr_ftc(u32 value);
struct msmfb_iris_tm_points_info *iris_get_tm_points_info(void);
void iris_update_tm_lut(void);
void iris_sdr2hdr_kb_switch(u32 value);
void iris_sdr2hdr_kb_switch_pre(void);
#endif // _DSI_IRIS_PQ_H_
