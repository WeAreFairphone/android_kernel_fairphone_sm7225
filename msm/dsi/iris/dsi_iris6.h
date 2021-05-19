#ifndef _DSI_IRIS_H_
#define _DSI_IRIS_H_


#define IRIS3_CHIP_VERSION	0x6933
#define IRIS5_CHIP_VERSION	0x6935
#define IRIS6_CHIP_VERSION	0x6936

struct iris_ambient_info {
	uint32_t lux;
	uint32_t bl_ratio;
	void *payload;
};

enum {
	IRIS_DTSI_PIP_IDX,
	IRIS_LUT_PIP_IDX,
	IRIS_PIP_IDX_CNT
};

enum {
	IRIS_IP_START = 0x00,
	IRIS_IP_SYS = IRIS_IP_START,
	IRIS_IP_RX = 0x01,
	IRIS_IP_TX = 0x02,
	IRIS_IP_PWIL = 0x03,
	IRIS_IP_DPORT = 0x04,
	IRIS_IP_DTG = 0x05,
	IRIS_IP_PWM = 0x06,
	IRIS_IP_DSC_DEN = 0x07,
	IRIS_IP_DSC_ENC = 0x08,
	IRIS_IP_SDR2HDR = 0x09,
	IRIS_IP_SDR2HDR_2 = 0x0a,
	IRIS_IP_SCALER1D = 0x0b,
	IRIS_IP_PEAKING = 0x0c,
	IRIS_IP_LCE = 0x0d,
	IRIS_IP_DPP = 0x0e,
	IRIS_IP_DBC = 0x0f,
	IRIS_IP_EXT = 0x10,
	IRIS_IP_DMA = 0x11,

	IRIS_IP_END,
	IRIS_IP_CNT = IRIS_IP_END
};

struct iris_pq_setting {
	u32 cmcolortempmode:2;
	u32 cmcolorgamut:4;
	u32 graphicdet:1;
	u32 alenable:1;
	u32 dbc:2;
	u32 demomode:3;
	u32 sdr2hdr:4;
	u32 readingmode:3;
};

struct quality_setting {
	struct iris_pq_setting pq_setting;
	u32 cctvalue;
	u32 colortempvalue;
	u32 luxvalue;
	u32 maxcll;
	u32 source_switch;
	u32 al_bl_ratio;
	u32 system_brightness;
	u32 min_colortempvalue;
	u32 max_colortempvalue;
	u32 dspp_dirty;
	u32 scurvelevel;
	u32 dimmingEnable;
	u32 sdr2hdr_lce;
	u32 sdr2hdr_scurve;
	u32 sdr2hdr_tf_coef;
	u32 sdr2hdr_ftc;
	u32 dportdisable;
	u32 ai_ambient;
	u32 ai_backlight;
	u32 ai_auto_en;
	u32 ai_kb_switch;
	u32 lut3d_gain;
};

struct iris_setting_info {
	struct quality_setting quality_cur;
	struct quality_setting quality_def;
};

#define PANEL_BL_MAX_RATIO 10000

enum iris_config_type {
	IRIS_CHIP_VERSION = 33,      // 0x0 : IRIS2, 0x1 : IRIS2-plus, 0x2 : IRIS3-lite
	IRIS_LUX_VALUE = 34,
	IRIS_CCT_VALUE = 35,
	IRIS_READING_MODE = 36,

	IRIS_CM_COLOR_TEMP_MODE = 39,
	IRIS_CM_COLOR_GAMUT = 40,
	IRIS_GRAPHIC_DET_ENABLE = 43,
	IRIS_AL_ENABLE = 44,			//AL means ambient light
	IRIS_DBC_LEVEL = 45,
	IRIS_DEMO_MODE = 46,
	IRIS_SDR2HDR = 47,
	IRIS_COLOR_TEMP_VALUE = 48,
	IRIS_HDR_MAXCLL = 49,
	IRIS_CM_COLOR_GAMUT_PRE = 51,
	IRIS_DBC_LCE_POWER = 52,
	IRIS_PP_DATA_PATH = 53,
	IRIS_DYNAMIC_POWER_CTRL = 54,
	IRIS_DMA_LOAD = 55,
	IRIS_ANALOG_BYPASS_MODE = 56,
	IRIS_PANEL_TYPE = 57,
	IRIS_DPP_ONLY = 59,
	IRIS_HDR_PANEL_NITES_SET = 60,
	IRIS_BLC_PWM_ENABLE = 68,
	IRIS_DBC_LED_GAIN = 69,
	IRIS_SCALER_FILTER_LEVEL = 70,
	IRIS_CCF1_UPDATE = 71,
	IRIS_CCF2_UPDATE = 72,
	IRIS_FW_UPDATE = 73,
	IRIS_HUE_SAT_ADJ = 74,
	IRIS_SCALER_PP_FILTER_LEVEL = 76,
	IRIS_CSC_MATRIX = 75,
	IRIS_CONTRAST_DIMMING = 80,
	IRIS_S_CURVE = 81,
	IRIS_DC_DIMMING = 87,
	IRIS_HDR_PREPARE = 90,
	IRIS_HDR_COMPLETE = 91,
	IRIS_MCF_DATA = 92,
	IRIS_CLEAR_TRIGGER = 88,
	IRIS_PANEL_NITS = 99,

	IRIS_DBG_TARGET_REGADDR_VALUE_GET = 103,
	IRIS_DBG_TARGET_REGADDR_VALUE_SET = 105,
	IRIS_DBG_KERNEL_LOG_LEVEL = 106,
	IRIS_DBG_SEND_PACKAGE = 107,
	IRIS_DBG_TARGET_REGADDR_VALUE_SET2 = 112,
	IRIS_DEBUG_CAP = 113,
	IRIS_WORK_MODE = 126,   //[15-8]: tx mode, [7-0]: rx mode
	IRIS_WAIT_VSYNC = 132,

	IRIS_DPP_DEMO_WINDOW = 139,
	IRIS_FINGER_DISPLAY = 140,
	IRIS_DPP_FADE_INOUT = 141,
	IRIS_DPP_PATH_MUX = 142,
	IRIS_GAMMA_MODE = 143,

	IRIS_SDR2HDR_AI_ENALE = 145,
	IRIS_SDR2HDR_AI_INPUT_AMBIENTLIGHT = 146,
	IRIS_SDR2HDR_AI_INPUT_PANEL_NITS = 147,
	IRIS_SDR2HDR_AI_INPUT_BACKLIGHT = 148,
	IRIS_SDR2HDR_AI_CTRL_TM_Y = 149,
	IRIS_SDR2HDR_AI_CTRL_TM_K = 150,
	IRIS_SDR2HDR_AI_CTRL_CAM_GAIN = 151,
	IRIS_SDR2HDR_AI_CTRL_S_CURVE = 152,
	IRIS_SDR2HDR_AI_CTRL_GRAPHIC_WEIGHT = 153,
	IRIS_SDR2HDR_AI_CTRL_FADE = 154,
	IRIS_SDR2HDR_LCE = 155,
	IRIS_SDR2HDR_SCURVE = 156,
	IRIS_SDR2HDR_TF_COEF = 157,
	IRIS_SDR2HDR_FTC = 158,
	IRIS_DPORT_DISABLE = 159,
	IRIS_DPORT_DIS_DEBUG = 160,
	IRIS_HDR10PLUS = 161,
	IRIS_SDR2HDR_AI_SWITCH = 162,
	IRIS_DPP_3DLUT_GAIN = 163,
	IRIS_SDR2HDR_AI_SWITCH_PRE = 165,
	IRIS_CONFIG_TYPE_MAX
};

enum SDR2HDR_CASE {
	SDR2HDR_Bypass = 0,
	HDR10In_ICtCp,
	HDR10PLUS,
	HDR_CASE1,
	HLG_HDR,
	SDR2HDR_CASE5_1,
	SDR2HDR_CASE5_2,
	SDR2HDR_CASE5_3,
	SDR2HDR_CASE5_4,
	SDR2HDR_CASE5_5,
	SDR2HDR_CASE5_6,
	SDR2HDR_CASE5_AI,
	SDR2HDR_CASE5_8,
	SDR2HDR_CASE5_9,
	SDR2HDR_CASE5_10,
	SDR2HDR_CASE5_11,
};

enum HDR_POWER {
	HDR_CAM_POWER_OFF = 0,
	HDR_POWER_ON,
	HDR_CAM_POWER_ON,
};

enum SDR2HDR_LUT_GAMMA_INDEX {
	SDR2HDR_LUT_GAMMA_120 = 0,
	SDR2HDR_LUT_GAMMA_106 = 1,
	SDR2HDR_LUT_GAMMA_102 = 2,
	SDR2HDR_LUT_GAMMA_100 = 3,
	SDR2HDR_LUT_GAMMA_MAX
};

enum iris_chip_version {
	IRIS2_VER = 0,
	IRIS2PLUS_VER,
	IRIS3LITE_VER,
	IRIS5_VER,
	IRIS6_VER,
	IRISSOFT_VER,
	IRIS5DUAL_VER,
	IRIS6DUAL_VER,
	UNKNOWN_VER,
	IRIS_DEFAULT_MODE_PT = 31,
};

enum AI_CASE {
	AI_AMBIENT_BACKLIGHT_DISABLE = 0,
	AI_AMBIENT_ENABLE,
	AI_BACKLIGHT_ENABLE,
	AI_AMBIENT_BACKLIGHT_ENABLE,
};

struct iris_update_ipopt {
	uint8_t ip;
	uint8_t opt_old;
	uint8_t opt_new;
	uint8_t chain;
};

struct iris_update_regval {
	uint8_t ip;
	uint8_t opt_id;
	uint32_t mask;
	uint32_t value;
};

struct iris_lp_ctrl {
	bool dynamic_power;
	bool ulps_lp;
	bool dbp_mode;	  // 0: pt mode; 1: dbp mode
	int qsync_mode;  // 0: disable qync; 1: enable qsync
	int abyp_lp;
	// bit [0]: iris esd check, [1]: panel esd check, [2]: recovery, [3]: print more information
	//     [4]: force trigger
	int esd_ctrl;
	uint32_t esd_cnt_iris;
	uint32_t esd_cnt_panel;
	int esd_ctrl_backup;
};

struct iris_vc_ctrl {
	bool vc_enable;
	uint8_t to_iris_vc_id;
	uint8_t to_panel_hs_vc_id;
	uint8_t to_panel_lp_vc_id;
};

struct iris_abyp_ctrl {
	bool abyp_disable;
	bool abyp_failed;
	uint16_t pending_mode; // pending_mode is accessed by SDEEncoder and HWBinder.
	struct mutex abyp_mutex;
};

struct ocp_header {
	u32 header;
	u32 address;
};

enum iris_path_sel {
	PATH_DSI = 0,
	PATH_I2C = 1,
};

struct msmfb_iris_tm_points_info {
	void *lut_lutx_payload;
	void *lut_luty_payload;
};

#endif // _DSI_IRIS_H_
