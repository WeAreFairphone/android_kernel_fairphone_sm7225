#include <video/mipi_display.h>
#include <sde_encoder_phys.h>
#include "dsi_iris6_api.h"
#include "dsi_iris6_lightup.h"
#include "dsi_iris6_lightup_ocp.h"
#include "dsi_iris6_lp.h"
#include "dsi_iris6_pq.h"
#include "dsi_iris6_lut.h"
#include "dsi_iris6_ioctl.h"
#include "dsi_iris6_log.h"

static bool iris_HDR10;
static bool iris_HDR10_YCoCg;
static bool shadow_iris_HDR10;
static bool shadow_iris_HDR10_YCoCg;
static bool iris_yuv_datapath;
static bool iris_capture_ctrl_en;
static bool iris_debug_cap;
static u8 iris_dbc_lut_index;
static u8 iris_sdr2hdr_mode;
static struct iris_setting_info iris_setting;
static bool iris_skip_dma;
extern int iris_frc_dma_disable;
static u32 iris_min_color_temp;
static u32 iris_max_color_temp;
static u32 iris_min_x_value;
static u32 iris_max_x_value;
static u8 iris_sdr2hdr_lut_index;
static u32 iris_sdr2hdr_current_level;
static u32 iris_lce_tf_coef;
static u32 iris_scurve_tf_coef;
static u32 iris_tm_tf_coef;
static bool iris_dportdis_debug;
static u32 orig_sdr2hdr_level;

#ifndef MIN
#define  MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

#define IRIS_CCT_MIN_VALUE		2500
#define IRIS_CCT_MAX_VALUE		11000
#define IRIS_CCT_STEP			25
#define IRIS_X_6500K			3128
#define IRIS_X_7500K			2991
#define IRIS_X_7700K			2969
#define IRIS_X_2500K			4637
#define IRIS_LAST_BIT_CTRL	1

static u32 iris_lce_level[][4] = {
	{90, 90, 120, 180},			//LCE_GAMMAGAIN_DARK
	{320, 450, 768, 900},		//LCE_GAMMAUPPER_DARK
	{20, 30, 44, 64},			//LCE_AHECLIP_DARK_UPPER
	{80, 100, 120, 130},		//LCE_AHECLIP_BRIGHT_UPPER
	{512, 490, 450, 300},		//LCE_AHECLIP_MID_EDGE_L
	{32, 54, 64, 68},			//LCE_AHECLIP_MID_UPPER
};

static u32 iris_scurve_level[][3] = {
	{800, 1000, 1228},		//SCURVE_MID
	{280, 332, 358},			//SCURVE_LOW_GAMMA
	{280, 282, 332},			//SCURVE_HIGH_GAMMA
};

static u32 iris_de_ftc[5] = {
	128, 110, 124, 130, 132,
};

static u32 iris_ai_break_Y[7] = {
	0x02460129, 0x062C04CC, 0x09A2083E, 0x0D5A0BFB, 0x0FDF0E8C, 0x0FFF0FFF, 0x00000FFF,
};

static uint32_t lut_y[15] = {};
static uint32_t lut_x[15] = {};
static struct msmfb_iris_tm_points_info iris_tm_points_lut;

/*range is 2500~11000*/
static u32 iris_color_x_buf[] = {
	4637, 4626, 4615, 4603,
	4591, 4578, 4565, 4552,
	4538, 4524, 4510, 4496,
	4481, 4467, 4452, 4437,
	4422, 4407, 4392, 4377,
	4362, 4347, 4332, 4317,
	4302, 4287, 4272, 4257,
	4243, 4228, 4213, 4199,
	4184, 4170, 4156, 4141,
	4127, 4113, 4099, 4086,
	4072, 4058, 4045, 4032,
	4018, 4005, 3992, 3980,
	3967, 3954, 3942, 3929,
	3917, 3905, 3893, 3881,
	3869, 3858, 3846, 3835,
	3823, 3812, 3801, 3790,
	3779, 3769, 3758, 3748,
	3737, 3727, 3717, 3707,
	3697, 3687, 3677, 3668,
	3658, 3649, 3639, 3630,
	3621, 3612, 3603, 3594,
	3585, 3577, 3568, 3560,
	3551, 3543, 3535, 3527,
	3519, 3511, 3503, 3495,
	3487, 3480, 3472, 3465,
	3457, 3450, 3443, 3436,
	3429, 3422, 3415, 3408,
	3401, 3394, 3388, 3381,
	3375, 3368, 3362, 3356,
	3349, 3343, 3337, 3331,
	3325, 3319, 3313, 3307,
	3302, 3296, 3290, 3285,
	3279, 3274, 3268, 3263,
	3258, 3252, 3247, 3242,
	3237, 3232, 3227, 3222,
	3217, 3212, 3207, 3202,
	3198, 3193, 3188, 3184,
	3179, 3175, 3170, 3166,
	3161, 3157, 3153, 3149,
	3144, 3140, 3136, 3132,
	3128, 3124, 3120, 3116,
	3112, 3108, 3104, 3100,
	3097, 3093, 3089, 3085,
	3082, 3078, 3074, 3071,
	3067, 3064, 3060, 3057,
	3054, 3050, 3047, 3043,
	3040, 3037, 3034, 3030,
	3027, 3024, 3021, 3018,
	3015, 3012, 3009, 3006,
	3003, 3000, 2997, 2994,
	2991, 2988, 2985, 2982,
	2980, 2977, 2974, 2971,
	2969, 2966, 2963, 2961,
	2958, 2955, 2953, 2950,
	2948, 2945, 2943, 2940,
	2938, 2935, 2933, 2930,
	2928, 2926, 2923, 2921,
	2919, 2916, 2914, 2912,
	2910, 2907, 2905, 2903,
	2901, 2899, 2896, 2894,
	2892, 2890, 2888, 2886,
	2884, 2882, 2880, 2878,
	2876, 2874, 2872, 2870,
	2868, 2866, 2864, 2862,
	2860, 2858, 2856, 2854,
	2853, 2851, 2849, 2847,
	2845, 2844, 2842, 2840,
	2838, 2837, 2835, 2833,
	2831, 2830, 2828, 2826,
	2825, 2823, 2821, 2820,
	2818, 2817, 2815, 2813,
	2812, 2810, 2809, 2807,
	2806, 2804, 2803, 2801,
	2800, 2798, 2797, 2795,
	2794, 2792, 2791, 2789,
	2788, 2787, 2785, 2784,
	2782, 2781, 2780, 2778,
	2777, 2776, 2774, 2773,
	2772, 2770, 2769, 2768,
	2766, 2765, 2764, 2763,
	2761, 2760, 2759, 2758,
	2756, 2755, 2754, 2753,
	2751, 2750, 2749, 2748,
	2747, 2745, 2744, 2743,
	2742, 2741, 2740, 2739,
	2737,
};

static long nCSCCoffValue[18];
static u16 *iris_crstk_coef_buf;

u8 iris_get_dbc_lut_index(void)
{
	return iris_dbc_lut_index;
}

struct iris_setting_info *iris_get_setting(void)
{
	return &iris_setting;
}

void iris_set_HDR10_YCoCg(bool val)
{
	shadow_iris_HDR10_YCoCg = val;
}

void iris_set_sdr2hdr_mode(u8 val)
{
	iris_sdr2hdr_mode = val;
}

int iris_hdr_enable_get(void)
{
	if (iris_get_cfg()->valid < PARAM_PARSED)
		return 0;

	if (iris_HDR10_YCoCg)
		return 2;
	else if (iris_HDR10)
		return 1;
	else if (iris_setting.quality_cur.pq_setting.sdr2hdr != SDR2HDR_Bypass)
		return 100;
	else
		return 0;
}

bool iris_dspp_dirty(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (pqlt_cur_setting->dspp_dirty > 0) {
		IRIS_LOGI("DSPP is dirty");
		pqlt_cur_setting->dspp_dirty--;
		return true;
	}

	return false;
}

void iris_quality_setting_off(void)
{
	iris_setting.quality_cur.al_bl_ratio = 0;
	iris_sdr2hdr_current_level = 0;
	if (iris_setting.quality_cur.pq_setting.sdr2hdr != SDR2HDR_Bypass) {
		iris_setting.quality_cur.pq_setting.sdr2hdr = SDR2HDR_Bypass;
		iris_sdr2hdr_level_set(SDR2HDR_Bypass, false);
	}

	if (iris_setting.quality_cur.pq_setting.cmcolorgamut != 0) {
		iris_setting.quality_cur.pq_setting.cmcolorgamut = 0;
		iris_cm_color_gamut_set(
			iris_setting.quality_cur.pq_setting.cmcolorgamut, false);
	}

	if (iris_setting.quality_cur.pq_setting.cmcolortempmode != 0) {
		iris_setting.quality_cur.pq_setting.cmcolortempmode = 0;
		iris_cm_colortemp_mode_set(
			iris_setting.quality_cur.pq_setting.cmcolortempmode, false);
	}
	iris_cm_csc_restore(false);

	iris_capture_ctrl_en = false;
	iris_skip_dma = false;
	iris_sdr2hdr_mode = 0;
}

bool iris_get_debug_cap(void)
{
	return iris_debug_cap;
}

void iris_set_debug_cap(bool val)
{
	iris_debug_cap = val;
}

bool iris_get_dportdis_debug(void)
{
	return iris_dportdis_debug;
}

void iris_set_dportdis_debug(bool val)
{
	iris_dportdis_debug = val;
}

void iris_set_skip_dma(bool skip)
{

	IRIS_LOGE("skip_dma=%d", skip);
	iris_skip_dma = skip;

	/*
	if (iris_skip_dma && iris_dynamic_power_get()) {
		len = iris_init_update_ipopt_t(IRIS_IP_SYS, 0x10, 0x10, 0);
		iris_update_pq_opt(true);
	}
	*/
}

static int iris_end_pq(void)
{
	int len = 0;

	if (!iris_skip_dma) {
		if (iris_dynamic_power_get())
			iris_dpg_event(0, 0);
		else
			len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe4, 0xe4, 0);
		iris_update_pq_opt(true);
	}
	return len;
}

static int iris_end_dpp(bool bcommit)
{
	int len = 0;

	if (!iris_skip_dma) {
		len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe3, 0xe3, 0);
		iris_update_pq_opt(bcommit);
	}
	return len;
}

static int iris_end_dport(void)
{
	int len = 0;

	if (!iris_skip_dma) {
		if (iris_dynamic_power_get())
			iris_dpg_event(0, 0);
		else
			len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe7, 0xe7, 0);
		iris_update_pq_opt(true);
	}
	return len;
}

static int iris_start_hdr(void)
{
	int len = 0;
	struct iris_update_regval regval;
	if (iris_dynamic_power_get())
		iris_dpg_event(1, 1);

	regval.ip = IRIS_IP_SDR2HDR;
	regval.opt_id = 0xfb;
	regval.mask = 0x00000003;
	regval.value = 0x00000000;

	iris_update_bitmask_regval_nonread(&regval, false);

	len = iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	return len;
}

static int iris_end_hdr(bool bcommit)
{
	int len = 0;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (!iris_skip_dma) {
		if (iris_dynamic_power_get()) {
			iris_dpg_event(0, 0);
		} else {
			if (pqlt_cur_setting->pq_setting.dbc != 0
				|| pqlt_cur_setting->pq_setting.sdr2hdr != 0)
				len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 0);
			else
				len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe4, 0xe4, 0);
		}
		iris_update_pq_opt(bcommit);
	}
	return len;
}

static int iris_start_dbc(void)
{
	int len = 0;
	if (iris_dynamic_power_get())
		iris_dpg_event(1, 1);
	return len;
}

static int iris_end_dbc(void)
{
	int len = 0;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (!iris_skip_dma) {
		if (iris_dynamic_power_get()) {
			iris_dpg_event(0, 0);
		} else {
			if (pqlt_cur_setting->pq_setting.dbc != 0
				|| pqlt_cur_setting->pq_setting.sdr2hdr != 0)
				len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe5, 0xe5, 0);
			else
				len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe4, 0xe4, 0);
		}
		iris_update_pq_opt(true);
	}
	return len;
}

static int iris_start_demo(void)
{
	int len = 0;
	if (!iris_skip_dma) {
		if (iris_dynamic_power_get())
			iris_dpg_event(1, 1);
	}
	return len;
}

static int iris_end_demo(void)
{
	int len = 0;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (!iris_skip_dma) {
		if (iris_dynamic_power_get()) {
			iris_dpg_event(0, 0);
		} else {
			if (pqlt_cur_setting->pq_setting.dbc != 0
				|| pqlt_cur_setting->pq_setting.sdr2hdr != 0)
				len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 1);
			else
				len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe4, 0xe4, 1);
		}
		len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe3, 0xe3, 0);
		iris_update_pq_opt(true);
	}
	return len;
}

void iris_init_ipopt_t(void)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (i = 0; i < pcfg->timing[pcfg->cur_timing].ip_opt_cnt; i++)
		pcfg->timing[pcfg->cur_timing].pq_update_cmd.update_ipopt_array[i].ip = 0xff;

	pcfg->timing[pcfg->cur_timing].pq_update_cmd.array_index = 0;
}

void iris_crst_coef_check(const u8 *fw_data, size_t fw_size)
{
	u32 len = 0;

	if (fw_size < (CRSTK_COEF_SIZE * CRSTK_COEF_GROUP))
		IRIS_LOGE("%s, fw_size is wrong, should be = %d bytes", __func__, CRSTK_COEF_SIZE * CRSTK_COEF_GROUP);
	if (iris_crstk_coef_buf == NULL) {
		len = (CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP;
		iris_crstk_coef_buf = kzalloc(len, GFP_KERNEL);
	}
	if (!iris_crstk_coef_buf) {
		IRIS_LOGE("%s:failed to alloc mem iris_crstk_coef_buf:%p",
			__func__, iris_crstk_coef_buf);
		return;
	}
	memcpy(&iris_crstk_coef_buf[0], (fw_data), ((CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP));
}
static u32 iris_color_temp_x_get(u32 index)
{
	return iris_color_x_buf[index];
}

void iris_pq_parameter_init(void)
{
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 index;

	if (pqlt_cur_setting->pq_setting.sdr2hdr
			== SDR2HDR_Bypass)
		iris_yuv_datapath = false;
	else
		iris_yuv_datapath = true;

	if (pcfg->valid < PARAM_PARSED) {
		IRIS_LOGW("%s(), doesn't parse iris param", __func__);
		return;
	}

	iris_dbc_lut_index = 0;
	iris_sdr2hdr_current_level = 0;
	if (pcfg->panel->panel_mode == DSI_OP_VIDEO_MODE)
		iris_debug_cap = true;

	iris_min_color_temp = pcfg->min_color_temp;
	iris_max_color_temp = pcfg->max_color_temp;

	index = (iris_min_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_min_x_value = iris_color_temp_x_get(index);

	index = (iris_max_color_temp-IRIS_CCT_MIN_VALUE)/IRIS_CCT_STEP;
	iris_max_x_value = iris_color_temp_x_get(index);

	iris_dportdis_debug = false;
	IRIS_LOGD("%s, iris_min_x_value=%d, iris_max_x_value = %d", __func__, iris_min_x_value, iris_max_x_value);
}

static void iris_hdr_power_datapath_set(void)
{
	int len;
	struct iris_update_regval regval;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	u32 power_level;

	switch (pqlt_cur_setting->pq_setting.sdr2hdr) {
	case SDR2HDR_Bypass:
		power_level = HDR_CAM_POWER_OFF;
		break;
	case SDR2HDR_CASE5_1:
	case SDR2HDR_CASE5_2:
	case SDR2HDR_CASE5_3:
	case SDR2HDR_CASE5_4:
	case SDR2HDR_CASE5_5:
	case SDR2HDR_CASE5_6:
	case SDR2HDR_CASE5_AI:
	case SDR2HDR_CASE5_8:
	case SDR2HDR_CASE5_9:
	case SDR2HDR_CASE5_10:
	case SDR2HDR_CASE5_11:
	case HDR_CASE1:
	case HDR10PLUS:
	case HLG_HDR:
		power_level = HDR_POWER_ON;
		break;
	default:
		power_level = HDR_POWER_ON;
		break;
	}

	if (pqlt_cur_setting->pq_setting.sdr2hdr == 0
		&& pqlt_cur_setting->pq_setting.dbc > 0)
		power_level = HDR_POWER_ON;

	if (power_level == HDR_CAM_POWER_OFF) {
		// set pwil pp_en
		regval.ip = IRIS_IP_PWIL;
		regval.opt_id = 0xfc;
		regval.mask = 0x00000004;
		if (power_level > 0)
			regval.value = 0x00000004;
		else
			regval.value = 0x00000000;
		iris_update_bitmask_regval_nonread(&regval, false);
		len = iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

		//power off hdr domain
		iris_pmu_hdr_set(0, 1);
	} else {
		//power on hdr domain
		if (iris_dynamic_power_get())
			iris_pmu_hdr_set(power_level, 1);
		else
			iris_pmu_hdr_set(power_level, 0);

		// set pwil pp_en
		regval.ip = IRIS_IP_PWIL;
		regval.opt_id = 0xfc;
		regval.mask = 0x00000004;
		if (power_level > 0)
			regval.value = 0x00000004;
		else
			regval.value = 0x00000000;
		iris_update_bitmask_regval_nonread(&regval, false);
		len = iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);
	}
}

static int iris_cm_csc_para_set(uint8_t chain, uint32_t csc_ip, uint32_t *csc_value)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	int len = 0;
	struct iris_ip_opt *psopt;

	if (csc_value == NULL) {
		IRIS_LOGE("csc value is empty");
		return 0;
	}

	ip = csc_ip;
	opt_id = 0x40;
	psopt = iris_find_ip_opt(ip, opt_id);
	if (psopt == NULL) {
		IRIS_LOGE("can not find ip = %02x opt_id = %02x", ip, opt_id);
		return 1;
	}

	data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
	IRIS_LOGI("csc: csc0=0x%x, csc1=0x%x, csc2=0x%x, csc3=0x%x, csc4=0x%x",
		csc_value[0], csc_value[1], csc_value[2], csc_value[3], csc_value[4]);
	val = csc_value[0];
	val &= 0x7fff7fff;
	data[3] = val;
	val = csc_value[1];
	val &= 0x7fff7fff;
	data[4] = val;
	val = csc_value[2];
	val &= 0x7fff7fff;
	data[5] = val;
	val = csc_value[3];
	val &= 0x7fff7fff;
	data[6] = val;
	val = csc_value[4];
	val &= 0x00007fff;
	data[7] = val;

	len = iris_update_ip_opt(ip, opt_id, chain);

	return len;
}

static int iris_cm_csc_coefOffset_set(uint8_t chain, uint32_t csc_ip, uint32_t *csc_value)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	int len = 0;
	struct iris_ip_opt *psopt;

	if (csc_value == NULL) {
		IRIS_LOGE("csc value is empty");
		return 0;
	}

	ip = csc_ip;
	opt_id = 0x40;
	psopt = iris_find_ip_opt(ip, opt_id);
	if (psopt == NULL) {
		IRIS_LOGE("can not find ip = %02x opt_id = %02x", ip, opt_id);
		return 1;
	}

	data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
	IRIS_LOGI("csc: csc0=0x%x, csc1=0x%x, csc2=0x%x, csc3=0x%x, csc4=0x%x",
		csc_value[0], csc_value[1], csc_value[2], csc_value[3], csc_value[4]);
	val = csc_value[0];
	val &= 0x7fff7fff;
	data[3] = val;
	val = csc_value[1];
	val &= 0x7fff7fff;
	data[4] = val;
	val = csc_value[2];
	val &= 0x7fff7fff;
	data[5] = val;
	val = csc_value[3];
	val &= 0x7fff7fff;
	data[6] = val;
	val = csc_value[4];
	val &= 0x00007fff;
	data[7] = val;
	val = csc_value[5]; //offset0
	val &= 0x000fffff;
	data[8] = val;
	val = csc_value[6]; //offset1
	val &= 0x000fffff;
	data[9] = val;
	val = csc_value[7]; //offset2
	val &= 0x000fffff;
	data[10] = val;

	len = iris_update_ip_opt(ip, opt_id, chain);

	return len;
}

void iris_dpp_cmcsc_level_set(u32 *csc_value)
{
	int len;

	len = iris_cm_csc_para_set(1, IRIS_IP_DPP, csc_value);
	len = iris_end_dpp(true);
	IRIS_LOGI("%s csc len=%d", "dpp", len);
}

int iris_dcDimming_backlight_set(u32 *values)
{
	struct iris_update_regval regval;
	int len;
	int rc = 0;
	uint32_t bl_lvl = values[3];
	uint32_t dimmingGain = 0;
	uint32_t  *payload = NULL;
	uint32_t dimmingEnable = values[4];

	if (bl_lvl > 0xffff) {
		IRIS_LOGE("invalid backlight params\n");
		return -EINVAL;
	}

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x60;    //DIM_CTRL/FD
	regval.mask = 0x00000001;
	regval.value = dimmingEnable?0x1:0x0;

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x60, 0x60, 0x1);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x61, 2);
	dimmingGain = ((values[1] << 16) | values[0]);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x61, 2, dimmingGain);
	dimmingGain = values[2];
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x61, 3, dimmingGain);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x61, 0x61, 0x01);

	//rc = iris_update_backlight(bl_lvl);
	len = iris_end_dpp(true);
	return rc;
}

void iris_dcDimming_enable(u8 enable)
{
	struct iris_update_regval regval;
	int len;

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x60;    //DIM_CTRL
	regval.mask = 0x00000001;
	regval.value = enable?1:0;

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x60, 0x60, 0x01);
	len = iris_end_dpp(true);
}

void iris_dcDimming_set(u32 rGain, u32 gGain, u32 bGain)
{
	//struct iris_update_regval regval;
	uint32_t  *payload = NULL;
	int len;
	uint32_t dimmingGain = 0;


	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x61, 2);
	dimmingGain = ((gGain << 16) | rGain);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x61, 2, dimmingGain);
	dimmingGain = bGain;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x61, 3, dimmingGain);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x61, 0x61, 0x01);

	len = iris_end_dpp(true);
}

void iris_dpp_path_set(u32 path_mux)
{
	struct iris_update_regval regval;
	int len;


	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x10;    //DPP_2
	regval.mask = 0x000000ff;
	regval.value = path_mux;

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x10, 0x10, 0x01);

	len = iris_end_dpp(true);
}

void iris_dpp_digitalBypass_metaEn(u8 metaEn)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x10;    //DPP_2
	regval.mask = 0x00000100;
	regval.value = (metaEn?1:0) << 8;

	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x10, 0x10, 0x1);

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0xa0; //gc_ctrl
	regval.mask = 0x800;
	regval.value = 0x800;
	if (pcfg->panel->panel_mode == DSI_OP_VIDEO_MODE)
		regval.value = metaEn ? 0x800 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xa0, 0xa0, 0x1);

	iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe3, 0xe3, 0x0);
	iris_update_pq_opt(true);

	IRIS_LOGI("metaEn: %d", metaEn);
}

void iris_scurve_enable_set(u32 level)
{
	u32 locallevel;
	u32 scurvelevel;
	int len;
	u8 enable = 0;
	uint32_t  *payload = NULL;


	if (level > 0) {
		enable = 1;
		scurvelevel = (0x72 + (level - 1));   //0x72, sCurve
	} else {
		scurvelevel = 0x71; //0x71, basical one
	}
	len = iris_update_ip_opt(IRIS_IP_DPP, scurvelevel, 0x01);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x70, 2);

	if (enable)
		locallevel = payload[0] | 0x5;
	else
		locallevel = payload[0] & (~0x1);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x70, 2, locallevel);
	len = iris_update_ip_opt(IRIS_IP_DPP, 0x70, 0x01);

	len = iris_end_dpp(true);
	IRIS_LOGI("scurve level=%d, len=%d", level, len);
}

int iris_cm_ratio_set(uint8_t chain)
{
	u32 index;
	u32 index_default;
	u32 xvalue;
	u32 xvalue_default;
	u32 ratio;
	u32 value = 0;
	u32 value_default;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	int len;
	struct iris_ip_opt *psopt;
	uint32_t *data = NULL;
	int i;
	//struct iris_cfg *pcfg = iris_get_cfg();
	uint16_t coefBuff_start = 0;

	//csc coef has 54 values + cct has 3 values.
	coefBuff_start = pqlt_cur_setting->pq_setting.cmcolorgamut*(CRSTK_COEF_SIZE/2 + CCT_VALUE_SIZE/2);
	iris_min_color_temp = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2];
	value_default = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2 + 1];
	iris_max_color_temp = iris_crstk_coef_buf[coefBuff_start+CRSTK_COEF_SIZE/2 + 2];

	if ((iris_min_color_temp == 0) || (iris_min_color_temp < IRIS_CCT_MIN_VALUE)
			|| (iris_min_color_temp > IRIS_CCT_MAX_VALUE))
		iris_min_color_temp = 2500;
	if ((iris_max_color_temp == 0) || (iris_max_color_temp < IRIS_CCT_MIN_VALUE)
			|| (iris_max_color_temp > IRIS_CCT_MAX_VALUE))
		iris_max_color_temp = 11000;
	if (value_default == 0)
		value_default = 6500;


	if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL)
		value = pqlt_cur_setting->colortempvalue;
	else if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_AUTO)
		value = pqlt_cur_setting->cctvalue;
	else {
		value = value_default;

	}

	if (value > iris_max_color_temp)
		value = iris_max_color_temp;
	else if (value < iris_min_color_temp)
		value = iris_min_color_temp;
	index = (value - IRIS_CCT_MIN_VALUE)/25;
	xvalue = iris_color_temp_x_get(index);


	if (value_default > iris_max_color_temp)
		value_default = iris_max_color_temp;
	else if (value_default < iris_min_color_temp)
		value_default = iris_min_color_temp;

	index_default = (value_default - IRIS_CCT_MIN_VALUE)/25;
	xvalue_default = iris_color_temp_x_get(index_default);
	IRIS_LOGI("cm color temperature default CCT=%d, xvalue_default = %d\n", value_default, xvalue_default);
	IRIS_LOGI("min_cct = %d, max_cct = %d\n", iris_min_color_temp, iris_max_color_temp);

	if (1 /*pcfg->lut_mode == SINGLE_MODE*/) {
		psopt = iris_find_ip_opt(IRIS_IP_DPP, 0x32);  //csc2' coef
		if (!psopt) {
			IRIS_LOGE("can not find ip=%x id=0x32", IRIS_IP_DPP);
			return -EINVAL;
		}
		data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
		if ((xvalue >= iris_max_x_value) && (xvalue < xvalue_default)) {
			ratio = ((xvalue - iris_max_x_value)*10000)/(xvalue_default - iris_max_x_value);
			IRIS_LOGD("ratio:%d, xvalue: %d, iris_max_x_value: %d", ratio, xvalue, iris_max_x_value);
			for (i = 0; i < 18; i++) {
				nCSCCoffValue[i] = (iris_crstk_coef_buf[coefBuff_start+18+i] * ratio +
							(10000 - ratio)*iris_crstk_coef_buf[coefBuff_start+i])/10000;
			}

		} else if ((xvalue <=  iris_min_x_value) && (xvalue >= xvalue_default)) {
			ratio = ((xvalue - xvalue_default)*10000)/(iris_min_x_value - xvalue_default);
			IRIS_LOGD("ratio:%d, xvalue: %d, iris_min_x_value: %d", ratio, xvalue, iris_min_x_value);
			for (i = 0; i < 18; i++) {
				nCSCCoffValue[i] = (iris_crstk_coef_buf[coefBuff_start+36+i]*ratio +
							iris_crstk_coef_buf[coefBuff_start+18+i]*(10000-ratio))/10000;
			}
		}

		data[2] = nCSCCoffValue[1] << 16 | nCSCCoffValue[0];
		data[3] = nCSCCoffValue[3] << 16 | nCSCCoffValue[2];
		data[4] = nCSCCoffValue[5] << 16 | nCSCCoffValue[4];
		data[5] = nCSCCoffValue[7] << 16 | nCSCCoffValue[6];
		data[6] = 0x0000 << 16 | nCSCCoffValue[8];
		data[7] = nCSCCoffValue[10] << 16 | nCSCCoffValue[9];
		data[8] = nCSCCoffValue[12] << 16 | nCSCCoffValue[11];
		data[9] = nCSCCoffValue[14] << 16 | nCSCCoffValue[13];
		data[10] = nCSCCoffValue[16] << 16 | nCSCCoffValue[15];
		data[11] = 0x0000 << 16 | nCSCCoffValue[17];
		len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x32, 0x32, chain);

		psopt = iris_find_ip_opt(IRIS_IP_DPP, 0x33);  //csc2' coef
		if (!psopt) {
			IRIS_LOGE("could not find ip=%d opt_id=0x33", IRIS_IP_DPP);
			return -EINVAL;
		}
		data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
		data[2] = nCSCCoffValue[1] << 16 | nCSCCoffValue[0];
		data[3] = nCSCCoffValue[3] << 16 | nCSCCoffValue[2];
		data[4] = nCSCCoffValue[5] << 16 | nCSCCoffValue[4];
		data[5] = nCSCCoffValue[7] << 16 | nCSCCoffValue[6];
		data[6] = 0x0000 << 16 | nCSCCoffValue[8];
		len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x33, 0x33, chain);
	}

	IRIS_LOGD("cm color temperature value=%d", value);
	return len;
}

u32 iris_cm_ratio_set_for_iic(void)
{
	u32 tablesel;
	u32 index;
	u32 xvalue;
	u32 ratio;
	u32 value;
	u32 regvalue = 0;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	value = pqlt_cur_setting->colortempvalue;

	if (value > iris_max_color_temp)
		value = iris_max_color_temp;
	else if (value < iris_min_color_temp)
		value = iris_min_color_temp;
	index = (value - IRIS_CCT_MIN_VALUE)/25;
	xvalue = iris_color_temp_x_get(index);

	if (xvalue == iris_min_x_value) {
		tablesel = 0;
		regvalue = tablesel | 0x02;
	} else if ((xvalue < iris_min_x_value) && (xvalue >= IRIS_X_7700K)) {
		tablesel = 0;
		ratio = ((xvalue - IRIS_X_7700K)*16383)/(iris_min_x_value - IRIS_X_7700K);
		regvalue = tablesel | (ratio<<16);
	} else if ((xvalue >= iris_max_x_value) && (xvalue < IRIS_X_7700K)) {
		tablesel = 1;
		ratio = ((xvalue - iris_max_x_value)*16383)/(IRIS_X_7700K - iris_max_x_value);
		regvalue = tablesel | (ratio<<16);
	}

	IRIS_LOGI("cm color temperature value=%d", value);

	return regvalue;
}

void iris_cm_colortemp_mode_set(u32 mode, bool bcommit)
{
	int len = 0;
	uint32_t  *payload = NULL;
	uint32_t csc_ctrl = 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x30, 2);
	csc_ctrl = (mode == 0)?0x00000020:0x00000011;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x30, 2, csc_ctrl);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x30, 0x30, 0x01);

	//if (mode > IRIS_COLOR_TEMP_OFF)
	len = iris_cm_ratio_set(0x01);

	len = iris_end_dpp(bcommit);
	IRIS_LOGD("cm color temperature mode=%d, len=%d", mode, len);
}

void iris_cm_color_temp_set(void)
{
	int len;
	/*struct quality_setting *pqlt_cur_setting = & iris_setting.quality_cur;*/

	/*if(pqlt_cur_setting->pq_setting.cmcolorgamut == 0) {*/

	len = iris_cm_ratio_set(0x01);
	len = iris_end_dpp(true);

	/*}*/
	IRIS_LOGD("%s, len = %d",  __func__, len);
}

void iris_cm_color_gamut_pre_set(u32 source_switch)
{
	struct iris_update_regval regval;
	int len;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	/*add protection for source and scene switch at the same time*/
	if (source_switch == 3)
		source_switch = 1;
	pqlt_cur_setting->source_switch = source_switch;

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x50;
	regval.mask = 0x00000302;
	regval.value = 0x0000000;

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x50, 0x50, 0x01);

	len = iris_end_dpp(true);

	IRIS_LOGD("source switch = %d, len=%d", source_switch, len);
}


void iris_dport_disable(bool bdisable, uint8_t chain)
{
	uint32_t  *payload = NULL;
	int len;
	uint32_t dport_ctrl1 = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (iris_dportdis_debug) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPORT, 0xf0, 4);
		dport_ctrl1 = payload[0] & 0xfffffffe;
		if (bdisable == 0)
			dport_ctrl1 |= 0x1;
		iris_set_ipopt_payload_data(IRIS_IP_DPORT, 0xf0, 4, dport_ctrl1);
		len = iris_update_ip_opt(IRIS_IP_DPORT, 0xf0, chain);
		len = iris_init_update_ipopt_t(IRIS_IP_DPORT, 0x80, 0x80, 0x01);
		len = iris_end_dport();
		IRIS_LOGD("dport_disable=%d, len=%d", bdisable, len);
		pcfg->dport_is_disable = bdisable ? true : false;
		if (bdisable == false)
			pcfg->frame_kickoff_count[1] = pcfg->frame_kickoff_count[0];
	}
}

void iris_dpp_apl_enable(bool enable, uint8_t chain)
{
	uint32_t  *payload = NULL;
	int len;
	uint32_t apl_ctrl = 0;
	uint32_t csc_coef_ctrl = 0;
	//enable/disable apl_ctrl
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x21, 2);
	apl_ctrl = payload[0] & 0xffffffcf;
	if (enable)
		apl_ctrl = apl_ctrl | 0x3 << 4;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x21, 2, apl_ctrl);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x21, 0x21, chain);

	//csc_coef_ctrl, CSC2_COEF_UPDATE_EN = 1 to enable
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x31, 2);
	csc_coef_ctrl = payload[0] & 0xfffffffe;
	if (enable)
		csc_coef_ctrl = csc_coef_ctrl | 0x1;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x31, 2, csc_coef_ctrl);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x31, 0x31, chain);

}

void iris_dpp_3dlut_send(u8 lut_optid, uint8_t chain)
{
	int len;
	u8 lut_type = CM_LUT;

	len = iris_init_update_ipopt_t(lut_type, lut_optid, lut_optid, chain);

	if (chain == 0)
		len = iris_end_dpp(true);

	IRIS_LOGI("3dlut send  optid: 0x%x, len=%d", lut_optid, len);
}

////dpp 3d lut interpolation
void iris_dpp_3dlut_interpolation(u8 enable, u32 interSrc, u32 lutgain, bool bcommit)
{
	struct iris_update_regval regval;
	uint32_t  *payload = NULL;
	int len;
	uint32_t lut3d_interp1;
	uint32_t lut3d_interp3;

	iris_dport_disable(0x1, 0x1);
	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x50;
	if (enable == 0) {  //3dlut bypass
		regval.mask = 0x00000023;
		regval.value = 0x0000020;
	} else {
		regval.mask = 0x000023;
		regval.value = 0x3;
	}
	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x50, 0x50, 0x01);

	if (enable == 1) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2);
		lut3d_interp1 = payload[0] & 0xffff0000;
		lut3d_interp1 = lut3d_interp1 | 0x1 | (interSrc << 8);
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2, lut3d_interp1);
		len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x51, 0x51, 0x01);

		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2);
		lut3d_interp3 = payload[0] & 0xffff0000;
		lut3d_interp3 = lut3d_interp3 | lutgain;
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
		len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x52, 0x52, 0x01);
	} else {
		payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2);
		lut3d_interp1 =  payload[0] & 0xfffffffe;
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2, lut3d_interp1);
		len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x51, 0x51, 0x01);
	}

	len = iris_end_dpp(bcommit);
	IRIS_LOGD("3dlut interpolation, enable=%d, len=%d", enable, len);
}

void iris_dpp_3dlut_gain(u32 lutgain, bool bcommit)
{
	uint32_t  *payload = NULL;
	int len;
	uint32_t lut3d_interp3;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2);
	lut3d_interp3 = payload[0] & 0xffff0000;
	lut3d_interp3 = lut3d_interp3 | lutgain;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x52, 2, lut3d_interp3);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x52, 0x52, 0x01);
	len = iris_end_dpp(bcommit);
	IRIS_LOGD("3dlut interpolation, gain=0x%x, len=%d", lutgain, len);
}

void iris_cm_color_gamut_set(u32 level, bool bcommit)
{
	struct iris_update_regval regval;
	int len;
	u32 gammalevel;
	uint32_t  *payload = NULL;
	uint32_t gammactrl = 0;
	uint32_t gammamode = 0;
	uint32_t currentmode;
	bool apl = 0;
	uint32_t lut3d_interp1;
	u8 aplstatus_value = iris_get_firmware_aplstatus_value();

	iris_dport_disable(0x1, 0x1);
	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x50;
	//regval.mask = 0x00000f22;

	if (level == 0) {  //3dlut bypass
		regval.mask = 0x00000023;
		regval.value = 0x0000020;
	} else {
		regval.mask = 0x00001c23;
		regval.value = 0x0 | ((level) << 10);
	}

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x50, 0x50, 0x01);

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2);
	lut3d_interp1 =  payload[0] & 0xfffffffe;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x51, 2, lut3d_interp1);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x51, 0x51, 0x01);

	apl = (aplstatus_value & (0x1 << level))?1:0;
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2);
	currentmode = payload[0] & 0x7;
	if (apl == 0) {
		gammactrl = payload[0] & 0xff0;
		gammalevel = 0x00+level;
		if (level == 4 || level == 5)
			gammalevel = 0x00+level;
	} else {
		gammamode = 2;
		gammactrl = ((payload[0]&0xff0) | gammamode | (0x1 << 3));
		//gammalevel = 0x20+level;
		gammalevel = 0xa0+level;
	}

	IRIS_LOGI("aplstauts: 0x%x, gammamode: %d, gammactrl= 0x%x", aplstatus_value, gammamode, gammactrl);
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);
	iris_dpp_apl_enable(apl, 0x01);
	iris_update_ip_opt(GAMMA_LUT, gammalevel, 0x01);

	len = iris_cm_ratio_set(0x01);
	/*do not generate lut table for source switch.*/
	/*if (pqlt_cur_setting->source_switch == 0)*/

	len = iris_end_dpp(bcommit);
	IRIS_LOGD("cm color gamut=%d, len=%d", level, len);
}

void iris_dpp_gamma_set(void)
{
	int len;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	u32 gammalevel;

	if (pqlt_cur_setting->pq_setting.cmcolortempmode
			== IRIS_COLOR_TEMP_OFF)
		gammalevel = 0; /*use liner gamma if cm lut disable*/
	else
		gammalevel = pqlt_cur_setting->pq_setting.cmcolorgamut + 1;

	len = iris_update_ip_opt(GAMMA_LUT, gammalevel, 0x01);

	len = iris_end_dpp(true);
}

void iris_dpp_gammamode_set(u32 gammamode, u32 gammaIndex)
{
//struct iris_update_regval regval;
	int len = 0;
	uint32_t  *payload = NULL;
	uint32_t currentmode = 0;
	uint32_t gammactrl = 0;

	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2);
	currentmode = payload[0] & 0x7;

	if ((gammamode == 0) && (currentmode != gammamode)) {
		//regval.value = 0;
		gammactrl = payload[0] & 0xff8;
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		len = iris_update_ip_opt(GAMMA_LUT, 0x00, 0x01);
	} else if (gammamode == 1) {
		gammactrl = ((payload[0]&0xfc0) | gammamode | (gammaIndex << 4));
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		if (currentmode != gammamode) {
			len = iris_update_ip_opt(GAMMA_LUT, 0x10, 0x01);
		}
	} else if (gammamode == 2) {
		gammactrl = ((payload[0]&0xfc0) | gammamode | (gammaIndex << 4));
		iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x20, 2, gammactrl);
		len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x20, 0x20, 0x01);

		if (currentmode != gammamode) {
			len = iris_update_ip_opt(GAMMA_LUT, 0x20, 0x01);
		}
	}

	len = iris_end_dpp(true);
}

/*enable: 1, demo window; 2: enable gamma_en*/
void iris_dpp_demo_window_set(u8 enable, u8 owAndGamma, u32 xWindow, u32 yWindow)
{
	int len = 0;
	uint32_t  *payload = NULL;
	uint32_t regvalue;
	IRIS_LOGD("%s, X:0x%x Y:0x%x, enable,out[%d, %d]", __func__, xWindow, yWindow, enable, owAndGamma);

	len = iris_start_demo();
	// RESERVED
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x63, 4);
	IRIS_LOGD("DPP< 0xE0, pos 4, 0x%x", payload[0]);
	// payload[0]: WND_CTRL
	//bit0: WND_EN,  bit1: out_en; bit 2: gamma_en;
	if (enable == 2)
		regvalue = 0x5;
	else
		regvalue = (enable != 0 ? 1:0);
	regvalue |= (owAndGamma << 1);  //owAndGamma: 0x1: out_en, 0x2: gamma_en; 0x3: out_en&gamm_en
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x63, 2, regvalue);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xD0, 2, regvalue);
	regvalue = xWindow;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x63, 3, regvalue);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xD0, 3, regvalue);
	regvalue = yWindow;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x63, 4, regvalue);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xD0, 4, regvalue);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x63, 0x63, 0x01);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xD0,  0x01);

	len = iris_end_demo();

	IRIS_LOGI("%s, X[0x%x], Y[0x%x], enable: %d len=%d", __func__, xWindow, yWindow, enable, len);

}

//shapeInFill: bit2~5, 0x0~0xf. [5:2]->[DIM_FIRST_EN, FILL_RPLACE_EN, FILL_INSIDE, FILL_SHAPE]
void iris_dpp_fingerDisplay_set(u8 enable, u8 shapeInFill, u32 radius, u32 position, u32 fillcolor)
{
	int len = 0;
	uint32_t  *payload = NULL;
	struct iris_update_regval regval;
	uint32_t regvalue = 0;
	IRIS_LOGD("iris_finger_display set radius:0x%x position:0x%x, fillcolor:0x%x", radius, position, fillcolor);

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x60;    //DIM_CTRL
	regval.mask = 0x0000003e;
	regvalue = ((enable?1:0) << 1 | (shapeInFill << 2));
	regval.value = regvalue;
	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x60, 0x60, 0x1);

	// RESERVED
	payload = iris_get_ipopt_payload_data(IRIS_IP_DPP, 0x62, 2);
	// payload[0]: eclipse, [1]: fill center, [2]: fillcolor
	IRIS_LOGD("dpp, 0x62, radius: 0x%x", payload[0]);
	regvalue = radius;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x62, 2, regvalue);
	regvalue = position;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x62, 3, regvalue);
	regvalue = fillcolor;
	iris_set_ipopt_payload_data(IRIS_IP_DPP, 0x62, 4, regvalue);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x62, 0x62, 0x01);

	len = iris_end_dpp(true);

	IRIS_LOGI("iris_finger_dislay_set set radius:0x%x position:0x%x, fillcolor:0x%x", radius, position, fillcolor);
}

void iris_dpp_fadeinout_enable(u8 enable)
{
	struct iris_update_regval regval;
	int len;

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x54;    //FADE_CTRL
	regval.mask = 0x00000001;
	regval.value = enable?1:0;

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x54, 0x54, 0x01);

	len = iris_end_dpp(true);
}

void iris_dpp_fadeinout_step(u8 enable, u32 fadestep)
{
	struct iris_update_regval regval;
	int len;
	uint32_t regvalue = 0;

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x54;    //FADE_CTRL
	regval.mask = 0x0000ffff;
	regvalue = ((fadestep<<4) | (enable?1:0));
	regval.value = regvalue;

	IRIS_LOGI("fadestep: %d, eanble: %d, regvalue: 0x%x", fadestep, enable, regvalue);
	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DPP, 0x54, 0x54, 0x01);

	len = iris_end_dpp(true);
}

void iris_dbc_level_set(u32 level)
{
	u32 locallevel;
	u32 dbcenable;
	int len;
	u8 localindex;

	len = iris_start_dbc();

	iris_hdr_power_datapath_set();

	dbcenable = (level > 0) ? 0x01:0x00;
	locallevel = 0x10 | level;

	iris_dbc_lut_index ^= 1;

	IRIS_LOGD("send A/B  %d", iris_dbc_lut_index);

	localindex = 0x20 | iris_dbc_lut_index;
	iris_update_ip_opt(DBC_LUT, DBC_OFF + level, 0x01);
	iris_init_update_ipopt_t(DBC_LUT, CABC_DLV_OFF + level, CABC_DLV_OFF + level, 0x01);

	iris_init_update_ipopt_t(IRIS_IP_DBC, localindex, localindex, 0x01);
	len = iris_update_ip_opt(IRIS_IP_DBC, dbcenable, 0x01);
	/*len = iris_update_ip_opt(popt,  IP_OPT_MAX, IRIS_IP_DBC, locallevel, skiplast);*/
	len = iris_end_dbc();

	IRIS_LOGD("dbc level=%d, len=%d", level, len);
}

void iris_reading_mode_set(u32 *csc_value)
{
	int len;

	len = iris_cm_csc_coefOffset_set(0x01, IRIS_IP_DPP, csc_value);

	len = iris_end_dpp(true);

	IRIS_LOGI("%s csc len=%d", "dpp", len);
}

void iris_pp_datapath_set(bool bEn)
{
	int len;
	u32 pwil_datapath;
	uint32_t  *payload = NULL;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_PWIL, 0xFC, 2);
	pwil_datapath = (payload[0] & (~0x4)) | ((bEn == true)?0x00000004:0x00000000);
	IRIS_LOGD("pwil_datapath=%d", pwil_datapath);
	iris_set_ipopt_payload_data(IRIS_IP_PWIL, 0xFC, 2, pwil_datapath);
	len = iris_update_ip_opt(IRIS_IP_PWIL, 0xFC, 0x01);

	len = iris_end_pq();
	IRIS_LOGD("pp_en=%d, len=%d", bEn, len);
}

static bool iris_AI_valid(u32 level)
{
	bool isValid = false;

	switch (level) {
	case SDR2HDR_CASE5_AI:
		isValid = true;
		break;
	default:
		isValid = false;
		break;
	}
	return isValid;
}

static bool iris_tm_valid(u32 level)
{
	bool isValid = false;

	switch (level) {
	case HDR10PLUS:
	case HDR_CASE1:
	case HLG_HDR:
	case SDR2HDR_CASE5_AI:
		isValid = true;
		break;
	default:
		isValid = false;
		break;
	}
	return isValid;
}

void iris_clear_init_trigger(void)
{
	int len;
	//bool skiplast = 0;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (iris_tm_valid(pqlt_cur_setting->pq_setting.sdr2hdr)) {
		len = iris_init_update_ipopt_t(IRIS_IP_SYS, 0x20, 0x20, 0);
		iris_update_pq_opt(true);
	}
}

bool iris_sdr2hdr_valid(u32 level)
{
	bool isValid = false;

	switch (level) {
	case SDR2HDR_Bypass:
	case HDR10PLUS:
	case HDR_CASE1:
	case HLG_HDR:
	case SDR2HDR_CASE5_1:
	case SDR2HDR_CASE5_2:
	case SDR2HDR_CASE5_3:
	case SDR2HDR_CASE5_4:
	case SDR2HDR_CASE5_5:
	case SDR2HDR_CASE5_6:
	case SDR2HDR_CASE5_AI:
	case SDR2HDR_CASE5_8:
	case SDR2HDR_CASE5_9:
	case SDR2HDR_CASE5_10:
	case SDR2HDR_CASE5_11:
		isValid = true;
		break;
	default:
		isValid = false;
		break;
	}
	return isValid;
}

void iris_al_enable(bool enable)
{
	int len;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	len = iris_start_hdr();
	if (enable == true) {
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
			0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 2, 0x000ed412c);    //adj_low=300, adj_high=-300
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
			0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 3, 0x00000320);    //adj_mid = 1000;
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
			0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 10, pqlt_cur_setting->luxvalue);
	} else {
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
			0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 2, 0);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
			0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 3, 0);
	}
	len = iris_init_update_ipopt_t(
		IRIS_IP_SDR2HDR, 0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr,
		0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	len = iris_end_hdr(true);

	IRIS_LOGD("al enable =%d", enable);
}

void iris_lux_set(u32 level)
{
	int len;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	len = iris_start_hdr();
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
		0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 10, level);
	len = iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, 0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr,
		0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("lux value =%d", level);
}

static void iris_sdr2hdr_lut_set(u32 level)
{
	u32 lut_level;
	uint32_t  *payload = NULL;
	u32 mask = 0x00000040;
	bool lut_switch = true;

	if (level == SDR2HDR_Bypass) {
		iris_sdr2hdr_lut_index = 0;
		iris_sdr2hdr_current_level = level;
		lut_switch = false;
	}

	if (level == iris_sdr2hdr_current_level)
		lut_switch = false;

	if (level == HDR10PLUS || level == HDR_CASE1) {
		if (iris_sdr2hdr_current_level == HDR_CASE1
			|| iris_sdr2hdr_current_level == HDR10PLUS) {
			iris_sdr2hdr_current_level = level;
			lut_switch = false;
		}
	}

	if ((level <= SDR2HDR_CASE5_11) && (level >= SDR2HDR_CASE5_1)) {
		if ((iris_sdr2hdr_current_level <= SDR2HDR_CASE5_11)
			&& (iris_sdr2hdr_current_level >= SDR2HDR_CASE5_1)) {
			iris_sdr2hdr_current_level = level;
			lut_switch = false;
		}
	}
	iris_sdr2hdr_current_level = level;

	switch (level) {
	case HDR10PLUS:
	case HDR_CASE1:
		lut_level = 0x10;
		break;
	case HLG_HDR:
		lut_level = 0x12;
		break;
	case SDR2HDR_CASE5_1:
	case SDR2HDR_CASE5_2:
	case SDR2HDR_CASE5_3:
	case SDR2HDR_CASE5_4:
	case SDR2HDR_CASE5_5:
	case SDR2HDR_CASE5_6:
	case SDR2HDR_CASE5_AI:
	case SDR2HDR_CASE5_8:
	case SDR2HDR_CASE5_9:
	case SDR2HDR_CASE5_10:
	case SDR2HDR_CASE5_11:
		lut_level = 0x11;
		break;
	default:
		lut_level = 0x11;
		break;
	}
	if (lut_switch == true) {
		iris_sdr2hdr_lut_index ^= 1;
		if (iris_sdr2hdr_lut_index == 1)
			lut_level += 0x08;
		iris_update_ip_opt(SDR2HDR_LUT, lut_level, 0x01);
	}
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 4);
	if (iris_sdr2hdr_lut_index == 0)
		payload[0] &= ~mask;
	else
		payload[0] |= mask;
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 4, payload[0]);
}

void iris_sdr2hdr_maxcll_set(u32 maxcll)
{
	u32 level;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	level = 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr;
	if (iris_tm_valid(pqlt_cur_setting->pq_setting.sdr2hdr)) {
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
			level, 49, maxcll);
		iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x50 + level, 0x01);
	}
}

static void iris_hdr_ai_enable_mask(bool bEn)
{
	u32 ai_ctrl;
	uint32_t  *payload = NULL;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x00, 2);
	ai_ctrl = (payload[0] & (~0x10000)) | ((bEn == true)?0x00010000:0x00000000);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x00, 2, ai_ctrl);
}

static void iris_hdr_ai_enable_replaceY(void)
{
	uint32_t  *payload = NULL;
	int i;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	uint32_t level;

	level = 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr;
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 11);
	for (i = 0; i < 7; i++) {
		payload[i] = iris_ai_break_Y[i];
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, level, 11+i, payload[i]);
	}
}

void iris_sdr2hdr_level_set(u32 level, bool bcommit)
{
	int len;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	bool dma_sent = false;
	struct iris_update_regval regval;

	len = iris_start_hdr();
	iris_hdr_power_datapath_set();

	if ((level <= HLG_HDR) && (level >= HDR10In_ICtCp)) {
		shadow_iris_HDR10 = true;
	} else {
		shadow_iris_HDR10 = false;
		shadow_iris_HDR10_YCoCg = false;
	}
	iris_sdr2hdr_lut_set(level);

	len = iris_update_ip_opt(IRIS_IP_SDR2HDR, level, 0x01);

	if (level != SDR2HDR_Bypass) {
		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x10 + level, 0x01);

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x20 + level, 0x01);

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x30 + level, 0x01);

		if (iris_AI_valid(level))
			iris_hdr_ai_enable_replaceY();

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x40 + level, 0x01);

		if (level == HLG_HDR || level == HDR10PLUS) {
			len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x50 + level, 0x01);
		}

		regval.ip = IRIS_IP_SDR2HDR;
		regval.opt_id = 0xfb;
		regval.mask = 0x00000003;
		if (level == HLG_HDR || level == HDR10PLUS)
			regval.value = 0x00000001;
		else if (orig_sdr2hdr_level == SDR2HDR_Bypass && level > 0)
			regval.value = 0x00000002;
		else
			regval.value = 0x00000000;

		iris_update_bitmask_regval_nonread(&regval, false);

		len = iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x60 + level, 0x01);

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x70 + level, 0x01);

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x90 + level, 0x01);

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xa0 + level, 0x01);

		if (pqlt_cur_setting->pq_setting.alenable == true) {
			iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
				0xb0 + level, 2, 0x000ed412c);                   //adj_low=300, adj_high=-300
			iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
				0xb0 + level, 3, 0x00000320);                   //adj_mid=1000
		} else {
			iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
				0xb0 + level, 2, 0);
			iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
				0xb0 + level, 3, 0);
		}

		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR,
			0xb0 + level, 10, pqlt_cur_setting->luxvalue);
		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xb0 + level, 0x01);

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xc0 + level, 0x01);

		if (iris_AI_valid(level))
			iris_hdr_ai_enable_mask(true);
		else
			iris_hdr_ai_enable_mask(false);
		len = iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x00, 0x01);

		len = iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x10, 0x01);
	}

	len = iris_end_hdr(bcommit);

	if (!iris_skip_dma) {
		dma_sent = true;
	}

	if (dma_sent) {
		IRIS_LOGI("AP csc prepare.");
		iris_HDR10 = shadow_iris_HDR10;
		iris_HDR10_YCoCg = shadow_iris_HDR10_YCoCg;
		pqlt_cur_setting->dspp_dirty = 1;
	}
	orig_sdr2hdr_level = level;
	IRIS_LOGD("sdr2hdr level =%d", level);
}

void iris_hdr_ai_enable(bool bEn)
{
	int len;
	u32 ai_ctrl;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x00, 2);
	ai_ctrl = (payload[0] & (~0x10000)) | ((bEn == true)?0x00010000:0x00000000);
	IRIS_LOGD("hdr ai enable =%d\n", ai_ctrl);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x00, 2, ai_ctrl);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x00, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_en=%d, len=%d\n", bEn, len);
}

void iris_hdr_ai_input_al(u32 al_value)
{
	int len;
	u32 ai_input_al_value;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	al_value = al_value >> 2;
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x10, 2);
	ai_input_al_value = (payload[0] & (~0x0000ffff)) | (al_value & 0x0000ffff);
	IRIS_LOGD("hdr ai input al =%d\n", al_value);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x10, 2, ai_input_al_value);

	if (!iris_AI_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	len = iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x10, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_input_al_value=%d, len=%d\n", al_value, len);
}

void iris_hdr_ai_input_panel_nits(u32 nits_value)
{
	int len;
	u32 ai_input_nits_value;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	nits_value = nits_value << 16;
	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x10, 2);
	ai_input_nits_value = (payload[0] & (~0xffff0000)) | (nits_value & 0xffff0000);
	IRIS_LOGD("hdr ai input panel nits =%d\n", nits_value);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x10, 2, ai_input_nits_value);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x10, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_input_panel_nits=%d, len=%d\n", nits_value, len);
}

void iris_hdr_ai_input_bl(u32 bl_value)
{
	int len;
	u32 ai_input_bl_value;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x10, 3);
	ai_input_bl_value = (payload[0] & (~0x0000ffff)) | (bl_value & 0x0000ffff);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x10, 3, ai_input_bl_value);

	if (!iris_AI_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	len = iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x10, 0x01);

	len = iris_end_hdr(true);
}

void iris_hdr_ai_ctrl_tm_y(bool bEn)
{
	int len;
	u32 ai_tm_y;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr, 48);
	ai_tm_y = (payload[0] & (~0x10)) | ((bEn == true)?0x0000010:0x00000000);
	IRIS_LOGD("hdr ai tm y =%d\n", ai_tm_y);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr, 48, ai_tm_y);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_tm_y=%d, len=%d\n", bEn, len);
}

void iris_hdr_ai_ctrl_tm_k(bool bEn)
{
	int len;
	u32 ai_tm_k;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr, 48);
	ai_tm_k = (payload[0] & (~0x20)) | ((bEn == true)?0x0000020:0x00000000);
	IRIS_LOGD("hdr ai tm y =%d\n", ai_tm_k);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr, 48, ai_tm_k);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_tm_y=%d, len=%d\n", bEn, len);
}

void iris_hdr_ai_ctrl_cam_gain(bool bEn)
{
	int len;
	u32 ai_cam_gain;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x70 + pqlt_cur_setting->pq_setting.sdr2hdr, 2);
	ai_cam_gain = (payload[0] & (~0x10000)) | ((bEn == true)?0x00010000:0x00000000);
	IRIS_LOGD("hdr ai cam gain =%d\n", ai_cam_gain);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x70 + pqlt_cur_setting->pq_setting.sdr2hdr, 2, ai_cam_gain);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x70 + pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_cam_gain=%d, len=%d\n", bEn, len);
}

void iris_hdr_ai_ctrl_graphic_weight(bool bEn)
{
	int len;
	u32 ai_graphic_weight;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x00 + pqlt_cur_setting->pq_setting.sdr2hdr, 3);
	ai_graphic_weight = (payload[0] & (~0x20)) | ((bEn == true)?0x00000020:0x00000000);
	IRIS_LOGD("hdr ai graphic weight  =%d\n", ai_graphic_weight);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x00 + pqlt_cur_setting->pq_setting.sdr2hdr, 3, ai_graphic_weight);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x00 + pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_graphic_weight=%d, len=%d\n", bEn, len);
}

void iris_hdr_ai_ctrl_fade(u32 FadeEn)
{
	int len;
	u32 ai_fade_ctrl;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	FadeEn = FadeEn & 0x0000001f;
	FadeEn = FadeEn << 25;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x00 + pqlt_cur_setting->pq_setting.sdr2hdr, 3);
	ai_fade_ctrl = (payload[0] & (~0x3e000000)) | FadeEn;
	IRIS_LOGD("hdr ai fade control  =%d\n", ai_fade_ctrl);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0x00 + pqlt_cur_setting->pq_setting.sdr2hdr, 3, ai_fade_ctrl);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0x00 + pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_fade_ctrl=%d, len=%d\n", FadeEn, len);
}

void iris_hdr_ai_ctrl_s_curve(bool bEn)
{
	int len;
	u32 ai_s_curve;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 4);
	ai_s_curve = (payload[0] & (~0x00000800)) | ((bEn == true)?0x00000800:0x00000000);
	IRIS_LOGD("hdr ai s curve =%d\n", ai_s_curve);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, 0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 4, ai_s_curve);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR, 0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("hdr_ai_s_curve=%d, len=%d\n", bEn, len);
}

void iris_pwm_freq_set(u32 value)
{
	int len;
	struct iris_update_regval regval;
	u32 regvalue = 0;

	regvalue = 1000000 / value;
	regvalue = (27000000 / 1024) / regvalue;

	regval.ip = IRIS_IP_PWM;
	regval.opt_id = 0xfd;
	regval.mask = 0xffffffff;
	regval.value = regvalue;

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_PWM, 0xfd, 0xfd, 0);

	iris_update_pq_opt(true);

	IRIS_LOGD("%s, blc_pwm freq=%d", __func__, regvalue);
}

void iris_pwm_enable_set(bool enable)
{
	int len;

	len = iris_update_ip_opt(IRIS_IP_PWM, enable, 0);

	iris_update_pq_opt(true);
	IRIS_LOGD("%s, blc_pwm enable=%d", __func__, enable);
}

void iris_dbc_bl_user_set(u32 value)
{
	int len;
	struct iris_update_regval regval;
	u32 regvalue = 0;

	if (iris_setting.quality_cur.pq_setting.dbc == 0)
		return;

	len = iris_start_dbc();
	regvalue = (value * 0xfff) / 0xff;

	regval.ip = IRIS_IP_DBC;
	regval.opt_id = 0xfd;
	regval.mask = 0xffffffff;
	regval.value = regvalue;

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DBC, 0xfd, 0xfd, 0x01);

	len = iris_end_dbc();

	IRIS_LOGD("%s,bl_user value=%d", __func__, regvalue);
}

void iris_dbc_led0d_gain_set(u32 value)
{
	int len;
	struct iris_update_regval regval;

	len = iris_start_dbc();
	regval.ip = IRIS_IP_DBC;
	regval.opt_id = 0xfc;
	regval.mask = 0xffffffff;
	regval.value = value;

	iris_update_bitmask_regval_nonread(&regval, false);
	len = iris_init_update_ipopt_t(IRIS_IP_DBC, regval.opt_id, regval.opt_id, 0x01);

	len = iris_end_dbc();

	IRIS_LOGD("%s,dbc_led0d value=%d", __func__, value);
}

void iris_panel_nits_set(u32 bl_ratio, bool bSystemRestore, int level)
{
	char led_pwm1[3] = {MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x0, 0x0};
	char hbm_data[2] = {MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x0};
	struct dsi_cmd_desc backlight_cmd = {
		{0, MIPI_DSI_DCS_LONG_WRITE, 0, 0, 0, sizeof(led_pwm1), led_pwm1, 0, NULL}, 1, 1};
	struct dsi_cmd_desc hbm_cmd = {
		{0, MIPI_DSI_DCS_SHORT_WRITE_PARAM, 0, 0, 0, sizeof(hbm_data), hbm_data, 0, NULL}, 1, 1};

	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &backlight_cmd,
	};
	u32 bl_lvl;
	struct iris_cfg *pcfg = iris_get_cfg();

	/* Don't control panel's brightness when sdr2hdr mode is 3 */
	if (iris_sdr2hdr_mode == 3)
		return;

	if (bSystemRestore)
		bl_lvl = iris_setting.quality_cur.system_brightness;
	else
		bl_lvl = bl_ratio * pcfg->panel_dimming_brightness / PANEL_BL_MAX_RATIO;

	if (pcfg->panel->bl_config.bl_max_level > 255) {
		if (pcfg->panel->bl_config.bl_inverted_dbv) {
			led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
			led_pwm1[2] = (unsigned char)(bl_lvl >> 8);
		} else {
			led_pwm1[1] = (unsigned char)(bl_lvl >> 8);
			led_pwm1[2] = (unsigned char)(bl_lvl & 0xff);
		}
	} else {
		led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
		backlight_cmd.msg.tx_len = 2;
	}
	iris_pt_send_panel_cmd(pcfg->panel, &cmdset);

	// Support HBM for different panels.
	hbm_data[1] = (bSystemRestore) ? pcfg->panel_hbm[0] : pcfg->panel_hbm[1];
	cmdset.cmds = &hbm_cmd;
	if (pcfg->panel_hbm[0] != pcfg->panel_hbm[1])
		iris_pt_send_panel_cmd(pcfg->panel, &cmdset);
	IRIS_LOGI("panel_nits: bl_lvl=0x%x, hbm=0x%x, restore=%d", bl_lvl, hbm_data[1], bSystemRestore);
}


void iris_scaler_filter_update(u8 scaler_type, u32 level)
{
	int len;
	uint8_t ip;

	if (scaler_type == SCALER_PP) {
		ip = SCALER1D_PP_LUT;
	} else {
		IRIS_LOGE("This scaler type SCALER_INPUT doesn't exist.");
		return;
	}

	len = iris_init_update_ipopt_t(ip, level, level, 0);

	iris_update_pq_opt(true);

	IRIS_LOGD("scaler filter level=%d", level);
}

/*
void iris_scaler_gamma_enable(bool lightup_en, u32 level)
{
	int len;
	bool skiplast = 0;

	if (lightup_en == false) {
		len = iris_start_hdr(&skiplast);
	}

	len = iris_update_ip_opt(IRIS_IP_DPP, level, skiplast);

	if (lightup_en == false)
		len = iris_end_pq();
	else {
		if (!iris_dynamic_power_get())
			len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 0);
	}

	iris_update_pq_opt(true);


	IRIS_LOGE("gamma enable=%d", level);
}

void iris_hdr_csc_prepare(void)
{
	int len;
	bool skiplast = 0;

	if (iris_capture_ctrl_en == false) {
		IRIS_LOGI("iris csc prepare.");
		iris_capture_ctrl_en = true;
		if (!iris_dynamic_power_get() && !iris_skip_dma)
			skiplast = 1;

		len = iris_init_update_ipopt_t(IRIS_IP_PWIL, 0x52, 0x52, skiplast);
		if (!iris_dynamic_power_get() && !iris_skip_dma)
			len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 0);

		iris_update_pq_opt(true);
	}
}
*/

int iris_kickoff(void *phys_enc)
{
	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (phys_encoder == NULL)
		return -EFAULT;
	if (phys_encoder->connector == NULL)
		return -EFAULT;

	c_conn = to_sde_connector(phys_encoder->connector);
	if (c_conn == NULL)
		return -EFAULT;

	display = c_conn->display;
	if (display == NULL)
		return -EFAULT;

	if (pcfg->valid < PARAM_PARSED) {
		IRIS_LOGW("%s(), doesn't parse iris param", __func__);
		return 0;
	}

	pcfg->frame_kickoff_count[0]++;
	complete(&pcfg->frame_ready_completion);
	return 0;
}

void iris_hdr_csc_complete(int step)
{
	int len;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;

	if (step == 0 || step == 1 || step == 3 || step == 4) {
		IRIS_LOGI("AP csc prepare.");
		iris_HDR10 = shadow_iris_HDR10;
		iris_HDR10_YCoCg = shadow_iris_HDR10_YCoCg;
		iris_setting.quality_cur.dspp_dirty = 1;
		if (step == 4)
			return;
	} else if (step == 5 || step == 6) {
		struct iris_cfg *pcfg = iris_get_cfg();

		IRIS_LOGI("Wait frame ready.");
		reinit_completion(&pcfg->frame_ready_completion);
		if (!wait_for_completion_timeout(&pcfg->frame_ready_completion,
						 msecs_to_jiffies(50)))
			IRIS_LOGE("%s: timeout waiting for frame ready", __func__);
	}

	IRIS_LOGI("AP csc complete.");

	if (!iris_dynamic_power_get()) {
		if (pqlt_cur_setting->pq_setting.dbc != 0
			|| pqlt_cur_setting->pq_setting.sdr2hdr != 0) {
			len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 1);
			len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe5, 0xe5, 1);
		} else {
			len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe4, 0xe4, 1);
		}
	} else {
		iris_dpg_event(0, 1);
	}
	len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe7, 0xe7, 1);
	len = iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe3, 0xe3, 0);
	iris_update_pq_opt(true);
	IRIS_LOGI("iris csc complete.");

}


int32_t  iris_update_ip_opt(uint8_t ip, uint8_t opt_id, uint8_t chain)
{
	int i = 0;
	uint8_t old_opt;
	int32_t cnt = 0;

	struct iris_pq_ipopt_val *pq_ipopt_val = iris_get_cur_ipopt_val(ip);

	if (pq_ipopt_val == NULL) {
		IRIS_LOGI("can not get pq ipot val ip = %02x, opt_id = %02x",
					ip, opt_id);
		return 1;
	}

	if (0 /*ip == IRIS_IP_EXT*/)	{
		if ((opt_id & 0xe0) == 0x40) {  /*CM LUT table*/
			for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
				if (((opt_id & 0x1f)%CM_LUT_GROUP)
						== (((pq_ipopt_val->popt[i]) & 0x1f)
								% CM_LUT_GROUP)) {
					old_opt = pq_ipopt_val->popt[i];
					pq_ipopt_val->popt[i] = opt_id;
					cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
					return cnt;
				}
			}
		} else if (((opt_id & 0xe0) == 0x60)
				|| ((opt_id & 0xe0) == 0xa0)
				|| ((opt_id & 0xe0) == 0xe0)) {
			/*SDR2HDR LUT table and gamma table*/
			for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
				if ((opt_id & 0xe0)
						== ((pq_ipopt_val->popt[i]) & 0xe0)) {
					old_opt = pq_ipopt_val->popt[i];
					pq_ipopt_val->popt[i] = opt_id;
					cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
					return cnt;
				}
			}
		} else { /*DBC LUT table*/
			for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
				if (((opt_id & 0xe0)
					== ((pq_ipopt_val->popt[i]) & 0xe0))
					&& (((pq_ipopt_val->popt[i]) & 0x1f) != 0)) {

					old_opt = pq_ipopt_val->popt[i];
					pq_ipopt_val->popt[i] = opt_id;

					cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
					return cnt;
				}
			}
		}
	}

	if (ip == DBC_LUT) {
		for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
			old_opt = pq_ipopt_val->popt[i];
			/*init lut can not change*/
			if (old_opt < CABC_DLV_OFF && (old_opt & 0x7f) != 0) {
				pq_ipopt_val->popt[i] = opt_id;
				cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
				return cnt;
			}
		}
	}

	if (ip == CM_LUT) {
		for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
			if ((opt_id % CM_LUT_GROUP)
					== (pq_ipopt_val->popt[i]
							% CM_LUT_GROUP)) {
				old_opt = pq_ipopt_val->popt[i];
				pq_ipopt_val->popt[i] = opt_id;
				cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
				return cnt;
			}
		}
	}

	if (ip == GAMMA_LUT) {
		old_opt  = pq_ipopt_val->popt[i];
		pq_ipopt_val->popt[i] = opt_id;
		cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
		return cnt;
	}
	for (i = 0; i < pq_ipopt_val->opt_cnt; i++) {
		if ((opt_id & 0xf0) == ((pq_ipopt_val->popt[i]) & 0xf0)) {
			old_opt  = pq_ipopt_val->popt[i];
			pq_ipopt_val->popt[i] = opt_id;
			cnt = iris_init_update_ipopt_t(ip, old_opt, opt_id, chain);
			return cnt;
		}
	}
	return 1;
}

static ssize_t iris_pq_config_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg_log {
		uint8_t type;
		char *str;
	};

	struct iris_cfg_log arr[] = {
					{IRIS_DBC_LEVEL, "IRIS_DBC_LEVEL"},
					{IRIS_DEMO_MODE, "IRIS_DEMO_MODE"},
					{IRIS_SDR2HDR, "IRIS_SDR2HDR"},
					{IRIS_DYNAMIC_POWER_CTRL, "IRIS_DYNAMIC_POWER_CTRL"},
					{IRIS_GRAPHIC_DET_ENABLE, "IRIS_GRAPHIC_DET_ENABLE"},
					{IRIS_HDR_MAXCLL, "IRIS_HDR_MAXCLL"},
					{IRIS_ANALOG_BYPASS_MODE, "IRIS_ANALOG_BYPASS_MODE"},
					{IRIS_CM_COLOR_TEMP_MODE, "IRIS_CM_COLOR_TEMP_MODE"},
					{IRIS_CM_COLOR_GAMUT, "IRIS_CM_COLOR_GAMUT"},
					{IRIS_CM_COLOR_GAMUT_PRE, "IRIS_CM_COLOR_GAMUT_PRE"},
					{IRIS_CCT_VALUE, "IRIS_CCT_VALUE"},
					{IRIS_COLOR_TEMP_VALUE, "IRIS_COLOR_TEMP_VALUE"},
					{IRIS_AL_ENABLE, "IRIS_AL_ENABLE"},
					{IRIS_LUX_VALUE, "IRIS_LUX_VALUE"},
					{IRIS_READING_MODE, "IRIS_READING_MODE"},
					{IRIS_CHIP_VERSION, "IRIS_CHIP_VERSION"},
					{IRIS_PANEL_TYPE, "IRIS_PANEL_TYPE"},
					{IRIS_SDR2HDR_AI_ENALE, "IRIS_SDR2HDR_AI_ENALE"},
					{IRIS_SDR2HDR_AI_INPUT_AMBIENTLIGHT, "IRIS_SDR2HDR_AI_INPUT_AMBIENTLIGHT"},
					{IRIS_SDR2HDR_AI_INPUT_PANEL_NITS, "IRIS_SDR2HDR_AI_INPUT_PANEL_NITS"},
					{IRIS_SDR2HDR_AI_INPUT_BACKLIGHT, "IRIS_SDR2HDR_AI_INPUT_BACKLIGHT"},
					{IRIS_SDR2HDR_AI_CTRL_TM_Y, "IRIS_SDR2HDR_AI_CTRL_TM_Y"},
					{IRIS_SDR2HDR_AI_CTRL_TM_K, "IRIS_SDR2HDR_AI_CTRL_TM_K"},
					{IRIS_SDR2HDR_AI_CTRL_CAM_GAIN, "IRIS_SDR2HDR_AI_CTRL_CAM_GAIN"},
					{IRIS_SDR2HDR_AI_CTRL_S_CURVE, "IRIS_SDR2HDR_AI_CTRL_S_CURVE"},
					{IRIS_SDR2HDR_AI_CTRL_GRAPHIC_WEIGHT, "IRIS_SDR2HDR_AI_CTRL_GRAPHIC_WEIGHT"},
					{IRIS_SDR2HDR_AI_CTRL_FADE, "IRIS_SDR2HDR_AI_CTRL_FADE"},
				};
	u32 type;
	u32 value;
	int i = 0;
	uint32_t cfg_val = 0;
	int len = ARRAY_SIZE(arr);

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	type = (val & 0xffff0000) >> 16;
	value = (val & 0xffff);
	iris_configure(type, value);

	for (i = 0; i < len; i++) {
		iris_configure_get(arr[i].type, 1, &cfg_val);
		IRIS_LOGE("%s: %d", arr[i].str, cfg_val);
	}

	return count;
}

static const struct file_operations iris_pq_config_fops = {
	.open = simple_open,
	.write = iris_pq_config_write,
};


int iris_dbg_init_pq(struct dsi_display *display)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}
	if (debugfs_create_file("iris_pq_config", 0644, pcfg->dbg_root, display,
				&iris_pq_config_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
			__FILE__, __LINE__);
		return -EFAULT;
	}
	return 0;
}

int iris_update_backlight(u32 bl_lvl)
{
	int rc = 0;
	struct iris_cfg *pcfg = NULL;
	struct mipi_dsi_device *dsi;
	struct dsi_panel *panel;
	char led_pwm1[3] = {MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x0, 0x0};
	struct dsi_cmd_desc backlight_cmd = {
		{0, MIPI_DSI_DCS_LONG_WRITE, 0, 0, 0, sizeof(led_pwm1), led_pwm1, 0, NULL}, 1, 1};

	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &backlight_cmd,
	};

	pcfg = iris_get_cfg();
	panel = pcfg->panel;
	dsi = &panel->mipi_device;

	iris_setting.quality_cur.system_brightness = bl_lvl;

	if (panel->bl_config.bl_max_level > 255) {
		if (panel->bl_config.bl_inverted_dbv) {
			led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
			led_pwm1[2] = (unsigned char)(bl_lvl >> 8);
		} else {
			led_pwm1[1] = (unsigned char)(bl_lvl >> 8);
			led_pwm1[2] = (unsigned char)(bl_lvl & 0xff);
		}
	} else {
		led_pwm1[1] = (unsigned char)(bl_lvl & 0xff);
		backlight_cmd.msg.tx_len = 2;
	}

	if (iris_get_abyp_mode_blocking() == IRIS_PT_MODE)
		rc = iris_pt_send_panel_cmd(panel, &cmdset);
	else
		rc = mipi_dsi_dcs_set_display_brightness(dsi, bl_lvl);

	if (panel->bl_config.bl_inverted_dbv)
		bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));
	iris_setting.quality_cur.ai_backlight = ((u32)pcfg->panel_nits*bl_lvl)/panel->bl_config.bl_max_level;
	if (iris_setting.quality_cur.ai_auto_en == AI_BACKLIGHT_ENABLE
		|| iris_setting.quality_cur.ai_auto_en == AI_AMBIENT_BACKLIGHT_ENABLE)
		iris_hdr_ai_input_bl(iris_setting.quality_cur.ai_backlight);

	return rc;
}

void iris_cm_csc_restore(bool bcommit)
{
	int i;
	uint32_t *data = NULL;
	struct iris_ip_opt *psopt;

	psopt = iris_find_ip_opt(IRIS_IP_DPP, 0x40);
	if (!psopt) {
		IRIS_LOGE("could not find ip=%x, opt_id=0x40", IRIS_IP_DPP);
		return;
	}

	data = (uint32_t *)psopt->cmd[0].msg.tx_buf;
	for (i = 3; i < 11; i++)
		data[i] = 0;

	data[3] = 0x08000000;
	data[5] = 0x08000000;
	data[6] = 0x00000800;

	iris_update_ip_opt(IRIS_IP_DPP, 0x40, 1);

	iris_end_dpp(bcommit);
}

void iris_sdr2hdr_lce(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = 0x10 + pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 3);
	if (value != 0)
		payload[0] &= 0xffffefff;
	else
		payload[0] |= 0x00001000;
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 3, payload[0]);
	iris_init_update_ipopt_t(
		IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
		pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	if (value > 0) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 20);
		payload[0] = (payload[0] & 0xff00ffff) | (iris_lce_level[0][value-1] << 16);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 20, payload[0]);

		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 19, iris_lce_level[1][value-1]);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 11);
		payload[0] = (payload[0] & 0xc00fffff) | (iris_lce_level[2][value-1] << 20);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 11, payload[0]);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 12);
		payload[0] = (payload[0] & 0xc00fffff) | (iris_lce_level[3][value-1] << 20);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 12, payload[0]);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 13);
		payload[0] = (payload[0] & 0xfffffc00) | iris_lce_level[4][value-1];
		payload[0] = (payload[0] & 0xc00fffff) | (iris_lce_level[5][value-1] << 20);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 13, payload[0]);
	}
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);

	iris_end_hdr(true);
	IRIS_LOGD("lce_level = %d", value);
}

void iris_sdr2hdr_scurve(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = 0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 3);
	if (value != 0)
		payload[0] &= 0xfffeffff;
	else
		payload[0] |= 0x00010000;
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr, 3, payload[0]);
	iris_init_update_ipopt_t(
		IRIS_IP_SDR2HDR, pqlt_cur_setting->pq_setting.sdr2hdr,
		pqlt_cur_setting->pq_setting.sdr2hdr, 0x01);

	if (value > 0) {
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 6, iris_scurve_level[0][value-1]);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 5);
		payload[0] = (payload[0] & 0xfffff000) | iris_scurve_level[1][value-1];
		payload[0] = (payload[0] & 0xff000fff) | (iris_scurve_level[2][value-1] << 12);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 5, payload[0]);
		iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);
	}

	iris_end_hdr(true);
	IRIS_LOGD("scurve_level = %d", value);
}

void iris_sdr2hdr_tf_coef_set(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 lce_option = 0x10 + pqlt_cur_setting->pq_setting.sdr2hdr;
	u32 scurve_option = 0xb0 + pqlt_cur_setting->pq_setting.sdr2hdr;
	u32 tm_option = 0x40 + pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	iris_start_hdr();

	if (value == 0) {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, lce_option, 25);
		payload[0] = (payload[0] & 0xfc007fff) | (iris_lce_tf_coef << 15);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, lce_option, 25, payload[0]);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, scurve_option, 4);
		payload[0] = (payload[0] & 0xfffff800) | iris_scurve_tf_coef;
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, scurve_option, 4, payload[0]);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, tm_option, 47);
		payload[0] = (payload[0] & 0xf800ffff) | (iris_tm_tf_coef << 16);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, tm_option, 47, payload[0]);
	} else {
		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, lce_option, 25);
		iris_lce_tf_coef = (payload[0] >> 15) & 0x7ff;
		payload[0] = (payload[0] & 0xfc007fff) | (1024 << 15);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, lce_option, 25, payload[0]);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, scurve_option, 4);
		iris_scurve_tf_coef = payload[0] & 0x7ff;
		payload[0] = (payload[0] & 0xfffff800) | 1024;
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, scurve_option, 4, payload[0]);

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, tm_option, 47);
		iris_tm_tf_coef = (payload[0] >> 16) & 0x7ff;
		payload[0] = (payload[0] & 0xf800ffff) | (1024 << 16);
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, tm_option, 47, payload[0]);
	}
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, lce_option, lce_option, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, scurve_option, scurve_option, 0x01);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, tm_option, tm_option, 0x01);

	iris_end_hdr(true);
	IRIS_LOGD("lce_level = %d", value);
}

void iris_sdr2hdr_ftc(u32 value)
{
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 option = 0xc0 + pqlt_cur_setting->pq_setting.sdr2hdr;

	if (!iris_sdr2hdr_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 19);
	payload[0] = (payload[0] & 0xf803ffff) | (iris_de_ftc[value] << 18);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 19, payload[0]);
	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);

	iris_end_hdr(true);
	IRIS_LOGD("ftc_level = %d", value);
}

void iris_sdr2hdr_kb_switch_pre(void)
{
	int len;
	u32 ai_ctrl;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (!iris_AI_valid(pqlt_cur_setting->pq_setting.sdr2hdr))
		return;

	len = iris_start_hdr();

	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x00, 2);
	ai_ctrl = payload[0] & (~0x10000);
	iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x00, 2, ai_ctrl);
	len = iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x00, 0x01);

	len = iris_end_hdr(true);
	IRIS_LOGD("kb switch pre\n");
}

void iris_sdr2hdr_kb_switch(u32 value)
{
	int len;
	u32 ai_ctrl;
	uint32_t  *payload = NULL;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	iris_update_ip_opt(SDR2HDR_LUT, 0x60 + value, 0x01);

	if (iris_AI_valid(pqlt_cur_setting->pq_setting.sdr2hdr)) {
		iris_update_ip_opt(SDR2HDR_LUT, 0x70 + value, 0x01);

		len = iris_start_hdr();

		payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x00, 2);
		ai_ctrl = payload[0] | 0x10000;
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR_2, 0x00, 2, ai_ctrl);
		len = iris_update_ip_opt(IRIS_IP_SDR2HDR_2, 0x00, 0x01);

		len = iris_end_hdr(true);
	} else {
		iris_update_ip_opt(SDR2HDR_LUT, 0x70 + value, iris_skip_dma);

		if (!iris_skip_dma)
			iris_update_pq_opt(true);
	}
	IRIS_LOGD("kb siwtch = %d", value);
}

void iris_init_tm_points_lut(void)
{
	iris_tm_points_lut.lut_lutx_payload = &lut_x;
	iris_tm_points_lut.lut_luty_payload = &lut_y;
}

void iris_update_tm_lut(void)
{
	uint32_t  *payload = NULL;
	u32 option = 0x52;
	struct iris_update_regval regval;
	struct quality_setting *pqlt_cur_setting = &iris_setting.quality_cur;
	int i;


	payload = iris_get_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 2);
	for (i=0; i<7; i++)
		payload[i] = lut_x[2*i] + (lut_x[2*i+1] << 16);
	payload[7] = lut_x[14];
	for (i=0; i<7; i++)
		payload[8+i] = lut_y[2*i] + (lut_y[2*i+1] << 16);
	payload[15] = lut_y[14];

	for (i=0; i<16; i++)
		iris_set_ipopt_payload_data(IRIS_IP_SDR2HDR, option, 2+i, payload[i]);

	if (pqlt_cur_setting->pq_setting.sdr2hdr != HDR10PLUS)
		return;

	iris_start_hdr();

	iris_init_update_ipopt_t(IRIS_IP_SDR2HDR, option, option, 0x01);

	regval.ip = IRIS_IP_SDR2HDR;
	regval.opt_id = 0xfb;
	regval.mask = 0x00000003;
	regval.value = 0x00000001;
	iris_update_bitmask_regval_nonread(&regval, false);

	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	iris_end_hdr(true);
}

struct msmfb_iris_tm_points_info *iris_get_tm_points_info(void)
{
	return &iris_tm_points_lut;
}
