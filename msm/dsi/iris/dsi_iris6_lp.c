#include <video/mipi_display.h>
#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include "dsi_drm.h"
#include <sde_encoder.h>
#include <sde_encoder_phys.h>
#include "dsi_iris6_lightup.h"
#include "dsi_iris6_lightup_ocp.h"
#include "dsi_iris6_lp.h"
#include "dsi_iris6_pq.h"
#include "dsi_iris6_gpio.h"
#include "dsi_iris6_log.h"
#include "dsi_iris6_api.h"
#include "sde_trace.h"
#include <linux/semaphore.h>

/* lp debug option (for debug only)
 * bit[0]: 0 -- N/A, 1 -- N/A
 * bit[1]: 0 -- N/A, 1 -- for debug when check top pmu status timeout, dump registers
 * bit[2]: 0 -- N/A, 1 -- for debug, force abyp_status_gpio = 0 when PT->ABYP
 * bit[3]: 0 -- N/A, 1 -- for debug, force abyp_status_gpio = 1 when ABYP->PT
 * bit[4]: 0 -- N/A, 1 -- for debug, force hdr_power_on_flag = 1
 * bit[9]: 0 -- N/A, 1 -- force disable ulps
 */
static int debug_lp_opt;

/* abyp light up debug option (for debug only, need panel off/on to take effect)
 * bit[0]: 0 -- light up with pre-load, 1 -- light up without pre-load
 * bit[1]: N/A
 * bit[2]: N/A
 * bit[3]: 0 -- N/A, 1 -- force abyp during panel switch
 * bit[4]: 0 -- stay on ABYP, 1 -- stay on PT.
 * bit[5]: 0 -- N/A, 1 -- enable ABYP debug.
 * bit[6]: 0 -- N/A, 1 -- enable ABYP debug.
 */
static int debug_on_opt;

static int lp_hdr_power_type;
static uint8_t iris_abyp_mode;
static struct semaphore abyp_sem;
static ktime_t lp_ktime0;
static int _esd_tolerant_cnt;
#define FRAME_INTERVAL_RATIO (1.5)

static u32 _abyp_regs[] = {
	0xf1100004,
	0xf110000c,
	0xf110002c,
	0xf1100034,
	0xf1100204,
	0xf110040c,
	0xf1100420,
	0xf1100440,
	0xf1100444,
	0xf1100450,
	0xf1100454,
	0xf1101454,
	0xf1101460,
	0xf1101464,
	0xf1101468,
	0xf110146c,
	0xf1101470,
	0xf0000038,
	0xf000003c,
	0xf0000040,
	0xf0000064,
	0xf0000068,
	0xf000006c,
	0xf00000d0,
	0xf000028c,
	0xf0000290,
	0xf0010004,
	0xf0010014,
	0xf0010018,
	0xf001ffe4,
	0xf1400070,
	0xf1400074,
	0xf1400078,
	0xf141ffe4,
};

static u32 _esd_regs[] = {
	0xf1100004,
	0xf110000c,
	0xf1100034,
	0xf1100204,
	0xf0000038,
	0xf000003c,
	0xf0000040,
	0xf1140000,
	0xf1140004,
	0xf1140008,
	0xf114000c,
	0xf1140130,
	0xf1140138,
	0xf115ffe4,
	0xf10dffe4,
	0xf119ffe4,
	0xf11dffe4,
	0xf125ffe4,
	0xf129ffe4,
	0xf131ffe4,
	0xf139ffe4,
	0xf13dffe4,
	0xf141ffe4,
	0xf149ffe4,
	0xf161ffe4,
};

static void _iris_init_abyp_check_work(void);
static void _iris_abp_ctrl_init(bool chain);
static void _iris_dbp_init(bool enable, bool chain);
static void _iris_extra_dma_trigger(bool chain);

void iris_lp_init(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	if (pcfg->iris_default_mode_pt)
	{
		debug_on_opt |= 0x10;
	}
	iris_abyp_mode = IRIS_PT_MODE;
	if (pcfg->valid < PARAM_PARSED)
		iris_abyp_mode = IRIS_ABYP_MODE;
#ifdef IRIS_ABYP_LIGHTUP
	iris_abyp_mode = IRIS_ABYP_MODE;
#endif

	pcfg->read_path = PATH_I2C; //PATH_I2C;

	pcfg->abyp_ctrl.pending_mode = MAX_MODE;
	mutex_init(&pcfg->abyp_ctrl.abyp_mutex);
	sema_init(&abyp_sem, 1);
	_iris_init_abyp_check_work();
}

void iris_lp_enable_pre(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (debug_on_opt & 0x1)
		iris_abyp_mode = IRIS_ABYP_MODE;

#ifdef IRIS_ABYP_LIGHTUP
	iris_abyp_mode = IRIS_ABYP_MODE;
#endif

	pcfg->abyp_ctrl.abyp_failed = false;

	_esd_tolerant_cnt = 0;

	if (pcfg->valid < PARAM_PARSED)
		return;
	iris_init_one_wired();
}

/* init iris low power */
void iris_lp_enable_post(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (pcfg->valid < PARAM_PARSED)
		return;

	IRIS_LOGI("lp dpg:%d, ulps:%d, abyp:%d qsync:%d dpp_only:%d esd_ctrl:%d, read_path:%d",
			  pcfg->lp_ctrl.dynamic_power, pcfg->lp_ctrl.ulps_lp,
			  pcfg->lp_ctrl.abyp_lp, pcfg->lp_ctrl.qsync_mode,
			  pcfg->dpp_only_enable, pcfg->lp_ctrl.esd_ctrl, pcfg->read_path);

	_iris_abp_ctrl_init(1);

	lp_hdr_power_type = 2;	//to make hdr power type is different
	iris_ulps_set(pcfg->lp_ctrl.ulps_lp, 1);
	iris_dynamic_power_set(pcfg->lp_ctrl.dynamic_power, 1);
	if (pcfg->cont_splash_status == 0) {
		iris_pmu_hdr_set(0, 1); //set hdr power off
		_iris_dbp_init(pcfg->dpp_only_enable, 1);
		_iris_extra_dma_trigger(0);
	} else {
		iris_pmu_hdr_set(0, 0); //set hdr power off
		_iris_dbp_init(pcfg->dpp_only_enable, 0);
		_iris_extra_dma_trigger(0);
	}
}

/*== PMU related APIs ==*/

/* dynamic power gating set */
void iris_dynamic_power_set(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	IRIS_LOGI("%s: %d, chain %d", __func__, enable, chain);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ALT_CTRL0;
	regval.mask = 0x200000; //MIPI_CMD_SEL -- HDR auto dma load
	regval.value = (enable ? 0x200000 : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = 0xe5;
	regval.mask = 0x1;
	regval.value = (enable ? 0x1 : 0x0);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	if (pcfg->lp_ctrl.qsync_mode) {
		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf6;
		regval.mask = 0xc00;
		regval.value = (enable ? 0x0 : 0x800);
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);
	}

	if (pcfg->lp_ctrl.dbp_mode && enable) {
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_OFF, ID_SYS_MPG_OFF, 0x1);
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_CTRL, ID_SYS_MPG_CTRL, 0x1);
		if (lp_hdr_power_type > 0) {
			// set pwil pp_en
			regval.ip = IRIS_IP_PWIL;
			regval.opt_id = 0xfc;
			regval.mask = 0x00000004;
			regval.value = 0x00000004;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);
			iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 0x1);
		}
	}

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_DPG_CTRL;
	regval.mask = 0x00000031; //DYG_EN
	regval.value = (enable ? 0x31 : 0x30);
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(true);

	pcfg->lp_ctrl.dynamic_power = enable;
}

/* dynamic power gating get */
bool iris_dynamic_power_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	return pcfg->lp_ctrl.dynamic_power;
}

void iris_dpg_event(bool start, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;
	uint8_t last;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s: %s chain %d", __func__, start ? "start" : "end", chain);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = start ? 0x10 : 0x11;
	if (pcfg->lp_ctrl.qsync_mode && pcfg->lp_ctrl.dynamic_power)
		last = 0x1;
	else
		last = chain;
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, last);

	if (pcfg->lp_ctrl.qsync_mode && pcfg->lp_ctrl.dynamic_power) {
		// set dma trig gen mode
		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf6;
		regval.mask = 0xc00;
		regval.value = start ? 0x800 : 0x0;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain ? 1 : 0);
	}
}

/* power on & off HDR domain
 *   type: 0 -- power off HDR & HDR_COLOR
 *         1 -- power on HDR, power off HDR_COLOER
 *         2 -- power on HDR & HDR_COLOR
 */
int iris_pmu_hdr_set(int type, bool chain)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s: type %d, cur type %d, chain %d", __func__, type, lp_hdr_power_type, chain);

	if (type < 0 || type > 2) {
		IRIS_LOGW("%s: type %d is wrong.", __func__, type);
		return 1;
	}

	if (lp_hdr_power_type == type) {
		IRIS_LOGI("%s: same type %d", __func__, type);
		return 2;
	}

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xe6; //power protection
	regval.mask = 0x7;
	if (pcfg->lp_ctrl.dbp_mode)
		regval.value = 0x4;
	else if (type > 0)
		regval.value = 0x6;
	else
		regval.value = 0x5;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	if (!pcfg->lp_ctrl.dynamic_power) {
		regval.ip = IRIS_IP_DMA;
		regval.opt_id = 0xe8; //pd11 power protection
		regval.mask = 0x3;
		if (type > 0)
			regval.value = 0x3;
		else
			regval.value = 0x1;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);
	}

	if (pcfg->lp_ctrl.qsync_mode && pcfg->lp_ctrl.dynamic_power) {
		// set pwil pb_req
		regval.ip = IRIS_IP_PWIL;
		regval.opt_id = 0xfd;
		regval.mask = 0x00020000;
		if (type > 0)
			regval.value = 0x00020000;
		else
			regval.value = 0x00000000;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf5; //pmu ctrl
		regval.mask = 0x200;  //HDR_IDLE_MASK_EN
		if (type > lp_hdr_power_type)
			regval.value = 0x200;
		else
			regval.value = 0x0;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);
	}

	lp_hdr_power_type = type;

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_MPG_CTRL;
	regval.mask = 0x18;
	if (lp_hdr_power_type == 2)
		regval.value = 0x18;
	else if (lp_hdr_power_type == 1)
		regval.value = 0x8;
	else
		regval.value = 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(true);

	return 0;
}

static void _iris_extra_dma_trigger(bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	IRIS_LOGI("%s chain %d", __func__, chain);

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xe3;
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(true);
}

void iris_ulps_set(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (debug_lp_opt & 0x200) {
		IRIS_LOGI("not set ulps: %d", enable);
		return;
	}
	IRIS_LOGI("ulps set: %d, chain %d", enable, chain);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ULPS_CTRL;
	regval.mask = 0x40;
	regval.value = enable ? 0x40 : 0x0;
	pcfg->lp_ctrl.ulps_lp = enable;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(true);
}

bool iris_ulps_enable_get(void)
{
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	IRIS_LOGI("ulps ap:%d, iris:%d",
			  pcfg->display->panel->ulps_feature_enabled, pcfg->lp_ctrl.ulps_lp);

	if (pcfg->display->panel->ulps_feature_enabled && pcfg->lp_ctrl.ulps_lp)
		return true;
	else
		return false;
}

/*== Analog bypass related APIs ==*/
static void _iris_abp_ctrl_init(bool chain)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;

	pcfg = iris_get_cfg();

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ABP_CTRL;
	regval.mask = 0x0CC00000;
	if (pcfg->lp_ctrl.abyp_lp == 1)
		regval.value = 0x00800000;
	else if (pcfg->lp_ctrl.abyp_lp == 2)
		regval.value = 0x00400000;
	else
		regval.value = 0x00000000;

	if (pcfg->dpp_only_enable)
		regval.value = 0x00C00000;

	/* w/o dbp, digital_bypass_i2a_en = 1 */
	if (!pcfg->dpp_only_enable)
		regval.value |= 0x04000000;
	/* Set digital_bypass_a2i_en/digital_bypass_i2a_en = 1 for video mode */
	if (pcfg->panel->panel_mode == DSI_OP_VIDEO_MODE)
		regval.value |= 0x0C000000;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(true);
}

void iris_dma_ch1_trigger(bool en, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();
	if (pcfg->cont_splash_status == 0)
		return;

	if (iris_get_abyp_mode_blocking() != IRIS_PT_MODE)
		return;

	IRIS_LOGI("%s,%d: %s [%i]", __func__, __LINE__, en ? "true" : "false", chain);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = 0xf6;
	regval.mask = 0xc00;
	regval.value = en ? 0x0 : 0x800;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);
	if (!chain)
		iris_update_pq_opt(true);
}

// to get correct iris chip abyp status, should call this API.
int iris_get_abyp_mode_blocking(void)
{
	uint8_t mode;
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	down(&abyp_sem);
	mode = iris_abyp_mode;
	up(&abyp_sem);
	return mode;
}

// only for get current status quickly
int iris_get_abyp_mode_nonblocking(void)
{
	return iris_abyp_mode;
}

void iris_force_abyp_mode(uint8_t mode)
{
	IRIS_LOGD("%s mode:%d", __func__, mode);
	down(&abyp_sem);
	if (mode == IRIS_PT_MODE || mode == IRIS_ABYP_MODE)
		iris_abyp_mode = mode;
	else
		IRIS_LOGE("%s not support mode:%d", __func__, mode);
	up(&abyp_sem);
}

/* Switch ABYP by GRCP commands
 * enter_abyp: true -- Enter ABYP, false -- Exit ABYP
*/
static void _iris_send_grcp_abyp(bool enter_abyp)
{
	if (enter_abyp) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 4);
		IRIS_LOGI("%s, Enter ABYP.", __func__);
	} else {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 5);
		IRIS_LOGI("%s, Exit ABYP.", __func__);
	}
}

static int _iris_set_max_return_size(void)
{
	int rc;
	struct iris_cfg *pcfg;
	static char max_pktsize[2] = {0x01, 0x00}; /* LSB tx first, 2 bytes */
	static struct dsi_cmd_desc pkt_size_cmd = {
		{0, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, MIPI_DSI_MSG_REQ_ACK, 0, 0,
		 sizeof(max_pktsize), max_pktsize, 0, NULL},
		1,
		0};
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &pkt_size_cmd,
	};

	pcfg = iris_get_cfg();

	IRIS_LOGD("%s", __func__);

	rc = iris_dsi_send_cmds(pcfg->panel, cmdset.cmds, cmdset.count,
							cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);
	if (rc)
		IRIS_LOGE("failed to send max return size packet, rc=%d", rc);

	return rc;
}

static int _iris_lp_check_gpio_status(int cnt, int target_status)
{
	int i;
	int abyp_status_gpio;

	if (cnt <= 0) {
		IRIS_LOGE("invalid param, cnt is %d", cnt);
		return -EINVAL;
	}

	IRIS_LOGD("%s, cnt = %d, target_status = %d", __func__, cnt, target_status);

	/* check abyp gpio status */
	for (i = 0; i < cnt; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGD("%s, %d, ABYP status: %d.", __func__, i, abyp_status_gpio);
		if (abyp_status_gpio == target_status)
			break;
		udelay(3 * 1000);
	}

	return abyp_status_gpio;
}

void _iris_abyp_stop(void)
{
	int i = 0;
	if (debug_on_opt & 0x20) {
		while (1) {
			IRIS_LOGI("ABYP switch stop here! cnt: %d", i++);
			msleep(3000);
			if (!(debug_on_opt & 0x20)) {
				IRIS_LOGE("ABYP switch stop Exit!");
				break;
			}
		}
	} else {
		IRIS_LOGI("ABYP debug option not enable.");
	}
}

int iris_lp_abyp_enter(bool blocking)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;
	int abyp_status_gpio, toler_cnt;
	int rc = 0;
	int fps, frame_interval;

	pcfg = iris_get_cfg();
	fps = pcfg->panel->cur_mode->timing.refresh_rate;
	if (fps != 0)
		frame_interval = 1000/fps + 1;
	else
		frame_interval = 1000/60 + 1;
	IRIS_LOGI("%s, frame_interval is %d ms", __func__, frame_interval);
	IRIS_LOGI("Enter abyp mode start, blocking: %d", blocking);

	if (pcfg->rx_mode == 1) {
		if (pcfg->lp_ctrl.qsync_mode && pcfg->lp_ctrl.dynamic_power) {
			/* change dma ch1 trigger src sel to SW manual mode */
			regval.ip = IRIS_IP_SYS;
			regval.opt_id = 0xf6;
			regval.mask = 0xc00;
			regval.value = 0x800;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

			/* set rx pb req */
			regval.ip = IRIS_IP_RX;
			regval.opt_id = 0xe4;
			regval.mask = 0x3;
			regval.value = 0x2;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);
		}

		/* mask dpg wakeup src */
		regval.ip = IRIS_IP_SYS;
		regval.opt_id = 0xf0;
		regval.mask = 0x30;
		regval.value = 0x0;
		iris_update_bitmask_regval_nonread(&regval, false);
		iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x0);
		iris_update_pq_opt(true);
	}

	lp_ktime0 = ktime_get();

	toler_cnt = 3;

enter_abyp_begin:
	/* send enter analog bypass */
	SDE_ATRACE_BEGIN("iris_abyp_enter_cmd");
	iris_send_one_wired_cmd(IRIS_ENTER_ANALOG_BYPASS);
	SDE_ATRACE_END("iris_abyp_enter_cmd");

	/*get the gpio status in 2-3 frames*/
	abyp_status_gpio = _iris_lp_check_gpio_status(frame_interval*FRAME_INTERVAL_RATIO, 1);
	if (debug_lp_opt & 0x04) {
		// for error process debug
		debug_lp_opt &= ~0x04;
		abyp_status_gpio = 0;
	}
	if (abyp_status_gpio == 1) {
		iris_abyp_mode = IRIS_ABYP_MODE;
	} else {
		if (toler_cnt > 0) {
			IRIS_LOGE("Enter abyp failed, %d, toler_cnt = %d", __LINE__, toler_cnt);
			iris_reset();
			iris_lightup_exit_abyp(true, true);
			iris_preload();
			toler_cnt--;
			goto enter_abyp_begin;
		} else {
			_iris_abyp_stop();
			pcfg->abyp_ctrl.abyp_failed = true;
			IRIS_LOGE("Enter abyp failed, %d, toler_cnt = %d", __LINE__, toler_cnt);
			rc = -1;
			return rc;
		}
	}

	IRIS_LOGI("Enter abyp spend time %d us",
			(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));

	pcfg->abyp_ctrl.abyp_failed = false;
	IRIS_LOGI("Enter abyp done");
	return rc;
}

static void _iris_proc_timing_switch(struct iris_cfg *pcfg)
{
	ktime_t ktime = 0;

	if (pcfg->cur_timing == pcfg->prev_timing)
		return;

	if (IRIS_IF_LOGI())
		ktime = ktime_get();

	SDE_ATRACE_BEGIN("_iris_proc_timing_switch");
	iris_send_mode_switch_pkt();
	SDE_ATRACE_END("_iris_proc_timing_switch");

	if (IRIS_IF_LOGI())
		IRIS_LOGI("%s() takes %d us.", __func__,
				(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime));
}

/*
 * @Description: check dphy status and revise it
 * @param: void
 * @return: 0 is sucess, -1 is stale data, -2 is i2c failure
 */
static int _iris_dphy_itf_check(void)
{
	int ret = 0;
	u32 dphy_itf_ctrl, sw_pwr_ctrl, value;
	uint32_t *data = NULL;

	dphy_itf_ctrl = 0xF1100000;
	sw_pwr_ctrl = 0xF000003C;

	data = iris_get_ipopt_payload_data(IRIS_IP_SYS, ID_SYS_MPG_CTRL, 2);
	if (!data) {
		IRIS_LOGE("%s, can not find the dest ip[%d] opt[%d]", __func__, IRIS_IP_SYS, ID_SYS_MPG_CTRL);
		return -1;
	}
	IRIS_LOGD("%s, sw_pwr_ctrl is 0x%08x", __func__, data[0]);

	ret = iris_ioctl_i2c_read(dphy_itf_ctrl, &value);
	if (ret) {
		IRIS_LOGW("%s, read dphy_itf_ctrl fail, ret = %d", __func__, ret);
		return -2;
	}

	if (value & 0x10000000) {
		IRIS_LOGE("%s, dphy_itf_ctrl is 0x%08x", __func__, value);
		iris_ioctl_i2c_write(sw_pwr_ctrl, 0x1);
		iris_ioctl_i2c_write(sw_pwr_ctrl, data[0]);
	}

	return 0;
}

int iris_lp_abyp_exit(bool blocking)
{
	struct iris_cfg *pcfg;
	struct iris_update_regval regval;
	int abyp_status_gpio;
	int toler_cnt = 3;
	int rc = 0;
	int fps, frame_interval;

	pcfg = iris_get_cfg();
	fps = pcfg->panel->cur_mode->timing.refresh_rate;
	if (fps != 0)
		frame_interval = 1000/fps + 1;
	else
		frame_interval = 1000/60 + 1;
	IRIS_LOGI("%s, frame_interval is %d ms", __func__, frame_interval);
	IRIS_LOGI("Exit abyp mode start, blocking: %d", blocking);

	lp_ktime0 = ktime_get();
	/* exit analog bypass */
	iris_send_one_wired_cmd(IRIS_EXIT_ANALOG_BYPASS);
	SDE_ATRACE_BEGIN("iris_abyp_exit_cmd");
	if (pcfg->rx_mode == 1) {
		udelay(1000);
		_iris_set_max_return_size();
	}
	SDE_ATRACE_END("iris_abyp_exit_cmd");
	//mutex_lock(&pcfg->abyp_ctrl.abyp_mutex);

exit_abyp_loop:
	/*get the gpio status in 2-3 frames*/
	abyp_status_gpio = _iris_lp_check_gpio_status(frame_interval*FRAME_INTERVAL_RATIO, 0);
	if (abyp_status_gpio == 0)
		iris_abyp_mode = IRIS_PT_MODE;

	/* restore dpg wakeup src value */
	regval.ip = IRIS_IP_SYS;
	regval.opt_id = 0xf0;
	regval.mask = 0x30;
	regval.value = 0x30;
	iris_update_bitmask_regval_nonread(&regval, false);

	if (debug_lp_opt & 0x04) {
		// for error process debug
		debug_lp_opt &= ~0x04;
		abyp_status_gpio = 1;
	}

	rc = _iris_dphy_itf_check();

	if (abyp_status_gpio != 0 || rc == -2) {
		if (toler_cnt-- == 0) {
			IRIS_LOGE("Exit abyp failed, %d", __LINE__);
			_iris_abyp_stop();
			return -1;
		}
		IRIS_LOGW("Exit abyp with preload, %d", __LINE__);
		iris_dump_regs(_abyp_regs, ARRAY_SIZE(_abyp_regs));
		iris_reset();
		iris_lightup_exit_abyp(true, true);
		iris_preload();
		goto exit_abyp_loop;
	} else {
		if (pcfg->lp_ctrl.abyp_lp == 2) {
			IRIS_LOGI("abyp light up iris");
			iris_lightup(pcfg->panel, NULL);
			IRIS_LOGI("Light up time %d us",
				(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
		} else {
			_iris_proc_timing_switch(pcfg);
		}
	}


	IRIS_LOGI("Exit abyp spend time %d us",
		(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));

	pcfg->abyp_ctrl.abyp_failed = false;
	IRIS_LOGI("Exit abyp done");
	return rc;
}

static void _iris_abyp_check_work(struct work_struct *work)
{
	struct iris_cfg *pcfg = container_of(work, struct iris_cfg, abyp_check_work);
	struct iris_update_regval regval;
	int i;
	int abyp_status_gpio;
	bool switch_ok = false;
	bool pt2abyp = false;

	/* check abyp gpio status */
	for (i = 0; i < 50; i++) {
		abyp_status_gpio = iris_check_abyp_ready();
		IRIS_LOGD("%s, %d, status: %d.", __func__, i, abyp_status_gpio);
		if (iris_abyp_mode == IRIS_PT_TO_ABYP_MODE) {
			pt2abyp = true;
			if (abyp_status_gpio == 1) {
				iris_abyp_mode = IRIS_ABYP_MODE;
				switch_ok = true;
				break;
			}
		} else if(iris_abyp_mode == IRIS_ABYP_TO_PT_MODE) {
			pt2abyp = false;
			if (abyp_status_gpio == 0) {
				iris_abyp_mode = IRIS_PT_MODE;
				switch_ok = true;
				/* restore dpg wakeup src value */
				regval.ip = IRIS_IP_SYS;
				regval.opt_id = 0xf0;
				regval.mask = 0x30;
				regval.value = 0x30;
				iris_update_bitmask_regval_nonread(&regval, false);
				break;
			}
		} else {
			IRIS_LOGI("Not switch mode: %d", iris_abyp_mode);
			break;
		}
		udelay(3 * 1000);
	}
	up(&abyp_sem);
	IRIS_LOGI("ABYP switch time %d us",
		(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(lp_ktime0));
	IRIS_LOGI("%s abyp %s, status %d", pt2abyp ? "Enter" : "Exit",
			  switch_ok ? "done" : "failed", iris_abyp_mode);
	if (!switch_ok) {
		mutex_lock(&pcfg->panel->panel_lock);
		pcfg->abyp_ctrl.abyp_failed = true;
		iris_dump_regs(_abyp_regs, ARRAY_SIZE(_abyp_regs));
		_iris_abyp_stop();
		mutex_unlock(&pcfg->panel->panel_lock);
	} else {
		pcfg->abyp_ctrl.abyp_failed = false;
	}
}

static void _iris_init_abyp_check_work(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();
	INIT_WORK(&pcfg->abyp_check_work, _iris_abyp_check_work);
}

int iris_abyp_switch_proc(struct dsi_display *display, int mode, bool blocking)
{
	int rc = 0;
	struct iris_cfg *pcfg;
	bool hdr_power_on_flag = !!(mode & 0x80);

	pcfg = iris_get_cfg();

	if (pcfg->rx_mode != pcfg->tx_mode) {
		IRIS_LOGE("abyp can't be supported! rx_mode != tx_mode!");
		return -1;
	}

	if (pcfg->abyp_ctrl.abyp_disable) {
		IRIS_LOGE("gpio is not setting for abypass");
		return -1;
	}

	if ((mode & 0x7F) == iris_abyp_mode) {
		IRIS_LOGW("%s same mode:%d!", __func__, mode);
		return rc;
	}
	if (debug_lp_opt & 0x80)
		blocking = true;
	if (debug_lp_opt & 0x10)
		hdr_power_on_flag = true;
	if ((mode & 0x7F) == IRIS_ABYP_MODE) {
		if (hdr_power_on_flag) {
			IRIS_LOGE("%s, hdr power is on, do sys reset and preload", __func__);
			if (debug_lp_opt & 0x02) {
				iris_dump_regs(_abyp_regs, ARRAY_SIZE(_abyp_regs));
				iris_dump_regs(_esd_regs, ARRAY_SIZE(_esd_regs));
			}
			iris_dsi_recover();
			iris_lightup(pcfg->panel, NULL);
		} else {
			SDE_ATRACE_BEGIN("iris_abyp_enter");
			rc = iris_lp_abyp_enter(blocking);
			SDE_ATRACE_END("iris_abyp_enter");
		}
	} else if ((mode & 0x7F) == IRIS_PT_MODE) {
		SDE_ATRACE_BEGIN("iris_abyp_exit");
		rc = iris_lp_abyp_exit(blocking);
		SDE_ATRACE_END("iris_abyp_exit");
	} else
		IRIS_LOGE("%s: switch mode: %d not supported!", __func__, mode);

	return rc;
}

int iris_lightup_exit_abyp(bool one_wired, bool reset)
{
	int i = 0;
	int iris_abyp_ready_gpio = 0;

	/* check abyp gpio status */
	iris_abyp_ready_gpio = iris_check_abyp_ready();

	if (iris_abyp_ready_gpio != 1) {
		IRIS_LOGW("%s, Iris isn't in ABYP.", __func__);
	}

	if (reset) {
		iris_send_one_wired_cmd(IRIS_POWER_DOWN_MIPI);
		udelay(1000);
		iris_send_one_wired_cmd(IRIS_POWER_UP_MIPI);
		udelay(1000);
	}

	/* try to exit analog bypass */
	if (one_wired)
		iris_send_one_wired_cmd(IRIS_EXIT_ANALOG_BYPASS);
	else
		_iris_send_grcp_abyp(false); /* switch by GRCP command */

	/* check abyp gpio status */
	for (i = 0; i < 50; i++) {
		udelay(3 * 1000);
		iris_abyp_ready_gpio = iris_check_abyp_ready();
		IRIS_LOGI("%s(%d), ABYP status: %d.", __func__, __LINE__, iris_abyp_ready_gpio);
		if (iris_abyp_ready_gpio == 0) {
			iris_abyp_mode = IRIS_PT_MODE;
			break;
		}
	}
	if (iris_abyp_ready_gpio == 1) {
		IRIS_LOGE("Exit abyp failed!");
		return 1;
	}
	IRIS_LOGI("Exit abyp done");
	return 0;
}

void iris_dbp_switch(bool enter)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;

	if (pcfg->lp_ctrl.dbp_mode == enter) {
		IRIS_LOGW("%s same mode:%d!", __func__, enter);
		return;
	}

	if (!pcfg->dpp_only_enable) {
		IRIS_LOGW("%s dpp_only is disable!", __func__);
		return;
	}
	IRIS_LOGI("%s, enter: %d.", __func__, enter);

	if (enter)
		iris_dpp_digitalBypass_metaEn(false);

	if (!pcfg->lp_ctrl.dynamic_power && !enter) {
		//manual power on PQ/HDR domains in DBP if need,
		//should set 0 then set 1
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_OFF, ID_SYS_MPG_OFF, 0x1);
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_CTRL, ID_SYS_MPG_CTRL, 0x1);
		if (lp_hdr_power_type > 0) {
			// set pwil pp_en
			regval.ip = IRIS_IP_PWIL;
			regval.opt_id = 0xfc;
			regval.mask = 0x00000004;
			regval.value = 0x00000004;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);
			iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe2, 0xe2, 0x1);
		}
	}

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xe6;
	regval.mask = 0x7;
	if (enter)
		regval.value = 0x4;
	else if (lp_hdr_power_type > 0)
		regval.value = 0x6;
	else
		regval.value = 0x5;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_DMA;
	regval.mask = 0x2;
	regval.value = enter ? 0x2 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_REG;
	regval.mask = 0x2;
	regval.value = enter ? 0x2 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	if (!pcfg->lp_ctrl.dynamic_power && enter) {
		//manual power off PQ/HDR domains in DBP
		if (lp_hdr_power_type > 0) {
			// set pwil pp_en
			regval.ip = IRIS_IP_PWIL;
			regval.opt_id = 0xfc;
			regval.mask = 0x00000004;
			regval.value = 0x00000000;
			iris_update_bitmask_regval_nonread(&regval, false);
			iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);
			iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe4, 0xe4, 0x1);
		}
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_OFF, ID_SYS_MPG_OFF, 0x1);
	}

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ABP_CTRL;
	regval.mask = 0x00C00000;
	if (enter) {
		if (pcfg->lp_ctrl.abyp_lp != 2)
			regval.value = 0x00C00000;
	} else {
		if (pcfg->lp_ctrl.abyp_lp == 1)
			regval.value = 0x00800000;
		else if (pcfg->lp_ctrl.abyp_lp == 2)
			regval.value = 0x00400000;
		else
			regval.value = 0x00000000;
	}
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x00);

	iris_update_pq_opt(true);

	if (!enter)
		iris_dpp_digitalBypass_metaEn(true);

	pcfg->lp_ctrl.dbp_mode = enter;
}

void _iris_dbp_init(bool enable, bool chain)
{
	struct iris_update_regval regval;
	struct iris_cfg *pcfg = iris_get_cfg();
	IRIS_LOGI("%s enable: %d, chain: %d", __func__, enable, chain);

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0x10;	  //DPP_2
	regval.mask = 0x00000100; //meta_en
	regval.value = enable ? 0x0 : 0x100;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0x10, 0x10, 0x1);

	regval.ip = IRIS_IP_DPP;
	regval.opt_id = 0xa0; //gc_ctrl
	regval.mask = 0x800;
	regval.value = 0x800;
	if (pcfg->panel->panel_mode == DSI_OP_VIDEO_MODE)
		regval.value = enable ? 0x0 : 0x800;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(IRIS_IP_DPP, 0xa0, 0xa0, 0x1);
	iris_init_update_ipopt_t(IRIS_IP_DMA, 0xe3, 0xe3, 0x1);

	regval.ip = IRIS_IP_DMA;
	regval.opt_id = 0xe6;
	regval.mask = 0x7;
	if (enable)
		regval.value = 0x4;
	else if (lp_hdr_power_type > 0)
		regval.value = 0x6;
	else
		regval.value = 0x5;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x1);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_DMA;
	regval.mask = 0x2;
	regval.value = enable ? 0x2 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_REG;
	regval.mask = 0x2;
	regval.value = enable ? 0x2 : 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	if (!pcfg->lp_ctrl.dynamic_power && enable) {
		iris_init_update_ipopt_t(IRIS_IP_SYS, ID_SYS_MPG_OFF, ID_SYS_MPG_OFF, 0x01);
	}

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = ID_SYS_ABP_CTRL;
	regval.mask = 0x00C00000;
	if (enable) {
		if (pcfg->lp_ctrl.abyp_lp != 2)
			regval.value = 0x00C00000;
	} else {
		if (pcfg->lp_ctrl.abyp_lp == 1)
			regval.value = 0x00800000;
		else if (pcfg->lp_ctrl.abyp_lp == 2)
			regval.value = 0x00400000;
		else
			regval.value = 0x00000000;
	}
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, 0x01);

	regval.ip = IRIS_IP_SYS;
	regval.opt_id = 0xf7;
	regval.mask = 0x4; //PB_BPS_MODE_DPP_BPS_EN
	regval.value = enable ? 0x0 : 0x4;
	iris_update_bitmask_regval_nonread(&regval, false);
	iris_init_update_ipopt_t(regval.ip, regval.opt_id, regval.opt_id, chain);
	if (!chain)
		iris_update_pq_opt(true);

	pcfg->lp_ctrl.dbp_mode = enable;
}

int iris_lightup_opt_get(void)
{
	return debug_on_opt;
}

void iris_lp_setting_off(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_update_regval regval;

	regval.ip = IRIS_IP_RX;
	regval.opt_id = ID_MIPI_BYPASS_CTRL_DMA;
	regval.mask = 0x2;
	regval.value = 0x0;
	iris_update_bitmask_regval_nonread(&regval, false);
	pcfg->lp_ctrl.dbp_mode = false;

	pcfg->abyp_ctrl.pending_mode = MAX_MODE;
}

int iris_prepare_for_kickoff(void *phys_enc)
{
	struct sde_encoder_phys *phys_encoder = phys_enc;
	struct sde_connector *c_conn = NULL;
	struct dsi_display *display = NULL;
	//int mode;

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

	return 0;
}

int iris_vc_id_get(int type)
{
	struct iris_cfg *pcfg;
	int vc_id = 0;

	pcfg = iris_get_cfg();
	if (type == 0)
		vc_id = pcfg->vc_ctrl.to_iris_vc_id;
	else if (type == 1)
		vc_id = pcfg->vc_ctrl.to_panel_hs_vc_id;
	else if (type == 2)
		vc_id = pcfg->vc_ctrl.to_panel_lp_vc_id;
	return vc_id;
}

static int _iris_esd_read(void)
{
	int rc = 1;
	unsigned int run_status = 0x00;
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	if (pcfg->read_path == PATH_I2C) { //use i2c to read
		uint32_t val;

		rc = iris_ioctl_i2c_read(IRIS_RUN_STATUS, &val);
		run_status = val & 0x3;

		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("i2c read iris esd value: 0x%0x. run_status:0x%x. rc:%d",
					  val, run_status, rc);
		}

		if (rc) {
			IRIS_LOGI("%s i2c read iris esd err: %d", __func__, rc);
			rc = -1;
			goto exit;
		}
	} else {
		char get_diag_result[1] = {0x0f};
		char rbuf[16] = {0};
		struct dsi_cmd_desc cmds = {
			{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0,
			 sizeof(get_diag_result), get_diag_result, 2, rbuf},
			1,
			0};
		struct dsi_panel_cmd_set cmdset = {
			.state = DSI_CMD_SET_STATE_HS,
			.count = 1,
			.cmds = &cmds,
		};

		rc = iris_dsi_send_cmds(pcfg->panel, cmdset.cmds, cmdset.count,
								cmdset.state, pcfg->vc_ctrl.to_iris_vc_id);

		run_status = rbuf[1] & 0x3;
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("dsi read iris esd value: 0x%02x 0x%02x. run_status:0x%x. rc:%d.",
					  rbuf[0], rbuf[1], run_status, rc);
		}
		if (rc) {
			IRIS_LOGI("%s dsi read iris esd err: %d", __func__, rc);
			rc = -1;
			goto exit;
		}
	}

	if (run_status != 0) {
		pcfg->lp_ctrl.esd_cnt_iris++;
		IRIS_LOGI("iris esd err 0x%x detected. ctrl: %d; cnt: %d", run_status,
				  pcfg->lp_ctrl.esd_ctrl, pcfg->lp_ctrl.esd_cnt_iris);
		iris_dump_regs(_esd_regs, ARRAY_SIZE(_esd_regs));
		rc = -2;
	} else {
		rc = 1;
	}

exit:
	IRIS_LOGD("%s rc:%d", __func__, rc);

	return rc;
}

static bool _iris_dsi_display_validate_reg_read(struct dsi_panel *panel)
{
	int i, j = 0;
	int len = 0, *lenp;
	int group = 0, count = 0;
	struct drm_panel_esd_config *config;

	if (!panel)
		return false;

	config = &(panel->esd_config);

	lenp = config->status_valid_params ?: config->status_cmds_rlen;
	count = config->status_cmd.count;

	for (i = 0; i < count; i++)
		len += lenp[i];

	for (i = 0; i < len; i++)
		j += len;

	for (j = 0; j < config->groups; ++j) {
		for (i = 0; i < len; ++i) {
			if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
				IRIS_LOGI("panel esd:[%d] 0x%x", i, config->return_buf[i]);
			}
			if (config->return_buf[i] != config->status_value[group + i]) {
				IRIS_LOGI("panel esd err: [%d] 0x%x != 0x%x!", i,
						  config->return_buf[i], config->status_value[group + i]);
			}

			if (config->return_buf[i] !=
				config->status_value[group + i])
				break;
		}

		if (i == len)
			return true;
		group += len;
	}

	return false;
}

static int _iris_display_read(struct dsi_display_ctrl *ctrl,
							  struct dsi_panel *panel, int mode)
{
	int i, rc = 0, count = 0, start = 0, *lenp;
	struct drm_panel_esd_config *config;
	struct dsi_cmd_desc *cmds;
	u32 flags = 0;
	struct dsi_panel_cmd_set cmdset;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (!panel || !ctrl || !ctrl->ctrl)
		return -EINVAL;

	/*
	 * When DSI controller is not in initialized state, we do not want to
	 * report a false ESD failure and hence we defer until next read
	 * happen.
	 */
	if (!dsi_ctrl_validate_host_state(ctrl->ctrl))
		return 1;

	config = &(panel->esd_config);
	lenp = config->status_valid_params ?: config->status_cmds_rlen;
	count = config->status_cmd.count;
	cmds = config->status_cmd.cmds;
	flags |= (DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ);

	memset(&cmdset, 0x00, sizeof(cmdset));
	cmdset.state = config->status_cmd.state;
	cmdset.count = 1;

	for (i = 0; i < count; ++i) {
		memset(config->status_buf, 0x0, SZ_4K);
		if (cmds[i].last_command) {
			cmds[i].msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
			flags |= DSI_CTRL_CMD_LAST_COMMAND;
		}
		cmds[i].msg.rx_buf = config->status_buf;
		cmds[i].msg.rx_len = config->status_cmds_rlen[i];

		if (mode == IRIS_ABYP_MODE) {
			cmds[i].msg.channel = 0;
			rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &cmds[i].msg, &flags);
			if (rc <= 0) {
				IRIS_LOGI("rx cmd transfer failed rc=%d\n", rc);
				goto error;
			}
		} else {
			if (pcfg->vc_ctrl.vc_enable) {
				//use aux channel
				if (config->status_cmd.state == DSI_CMD_SET_STATE_HS)
					cmds[i].msg.channel = pcfg->vc_ctrl.to_panel_hs_vc_id;
				else
					cmds[i].msg.channel = pcfg->vc_ctrl.to_panel_lp_vc_id;
				rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &cmds[i].msg, &flags);
				if (rc <= 0) {
					IRIS_LOGI("rx cmd transfer failed rc=%d", rc);
					goto error;
				}
			} else {
				cmdset.cmds = &cmds[i];

				rc = iris_pt_send_panel_cmd(panel, &cmdset);

				if (rc < 0) {
					IRIS_LOGI("iris_pt_send_panel_cmd transfer failed rc=%d", rc);
					goto error;
				}
				rc = 1;
			}
		}
		memcpy(config->return_buf + start,
			   config->status_buf, lenp[i]);
		start += lenp[i];
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD()) {
			IRIS_LOGI("%s rc=%d, len[%d]:%d buf[0]:0x%x. mode: %d", __func__, rc, i,
					  lenp[i], config->status_buf[0], mode);
		}
	}

error:

	return rc;
}

int iris_esd_ctrl_get(void)
{
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();

	return pcfg->lp_ctrl.esd_ctrl;
}

int iris_status_get(struct dsi_display_ctrl *ctrl, struct dsi_panel *panel)
{
	int rc = 0;
	int mode;
	struct iris_cfg *pcfg;
	pcfg = iris_get_cfg();
	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
		IRIS_LOGI("esd %s start", __func__);

	//mutex_lock(&pcfg->panel->panel_lock);
	// check abyp mode
	mode = iris_get_abyp_mode_nonblocking();

	if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
		IRIS_LOGI("esd %s, mode: %d", __func__, mode);

	if (pcfg->abyp_ctrl.abyp_failed) {
		IRIS_LOGE("%s abyp switch failed. Need recovery.", __func__);
		rc = -2;
		goto exit;
	}

	if ((mode != IRIS_PT_MODE) && (mode != IRIS_ABYP_MODE)) {
		rc = 1;
		goto exit;
	}

	if ((mode == IRIS_PT_MODE) && (iris_esd_ctrl_get() & 0x1)) {

		if (pcfg->read_path == PATH_I2C) {
			rc = iris_dsirecover_check(IRIS_ESDCHECK_OP);
			if (rc != 0) {
				IRIS_LOGE("%s iris_dsirecover_check err", __func__);
				rc = -1;
				goto exit;
			}
		}

		if (pcfg->dport_is_disable == true) {
			IRIS_LOGD("%s, dport disabled, do not read iris ", __func__);
			rc = 1;
		} else {
			pcfg->frame_kickoff_count[2] = pcfg->frame_kickoff_count[0];
			if (abs(pcfg->frame_kickoff_count[2] - pcfg->frame_kickoff_count[1]) > 1) {
				// iris esd read in pt mode
				rc = _iris_esd_read();
				if (rc <= 0)
					goto exit;
			} else {
				IRIS_LOGD("%s, the dport enable and esd in same frame", __func__);
				rc = 1;
			}
		}
	}
	if (iris_esd_ctrl_get() & 0x2) {
		// panel esd read in abyp & pt mode
		rc = _iris_display_read(ctrl, panel, mode);
		if (rc <= 0) {
			rc = -1;
			goto exit;
		}
		rc = _iris_dsi_display_validate_reg_read(panel);
		if (rc <= 0) {
			rc = -3;
			pcfg->lp_ctrl.esd_cnt_panel++;
			IRIS_LOGI("iris panel esd err detected. ctrl: %d; cnt: %d",
					  pcfg->lp_ctrl.esd_ctrl, pcfg->lp_ctrl.esd_cnt_panel);
			if (mode == IRIS_PT_MODE)
				iris_dump_regs(_esd_regs, ARRAY_SIZE(_esd_regs));
			goto exit;
		}
	} else {
		rc = 1;
		goto exit;
	}

exit:
	//mutex_unlock(&pcfg->panel->panel_lock);
	if (rc <= 0) {
		IRIS_LOGI("%s esd err rc: %d, tol:%d", __func__, rc, _esd_tolerant_cnt++);
		if (rc == -1) {
			if (_esd_tolerant_cnt < 3)
				rc = 1;
			else
				_esd_tolerant_cnt = 0;
		}
		if ((iris_esd_ctrl_get() & 0x4) == 0)
			rc = 1; /* Force not return error */
	} else {
		_esd_tolerant_cnt = 0;
		if (iris_esd_ctrl_get() & 0x10) {
			rc = -1;
			pcfg->lp_ctrl.esd_ctrl &= ~0x10;
			IRIS_LOGI("esd %s force trigger", __func__);
		}
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
			IRIS_LOGI("esd %s done rc: %d", __func__, rc);
	}
	return rc;
}

/*== Low Power debug related ==*/

static ssize_t _iris_abyp_dbg_write(struct file *file,
				const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg;
	static int cnt;

	pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	if (!mutex_trylock(&pcfg->panel->panel_lock))
		return -EFAULT;

	if (val == 0) {
		iris_lp_abyp_exit(true);
		IRIS_LOGI("analog bypass->pt, %d", cnt);
	} else if (val == 1) {
		iris_lp_abyp_enter(true);
		IRIS_LOGI("pt->analog bypass, %d", cnt);
	} else if (val >= 11 && val <= 19) {
		IRIS_LOGI("%s one wired %d", __func__, (int)(val - 11));
		iris_send_one_wired_cmd((int)(val - 11));
	} else if (val == 20) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 5);
		IRIS_LOGI("miniPMU analog bypass->pt");
	} else if (val == 21) {
		iris_send_ipopt_cmds(IRIS_IP_SYS, 4);
		IRIS_LOGI("miniPMU pt->analog bypass");
	} else if (val == 30) {
		iris_dbp_switch(false);
	} else if (val == 31) {
		iris_dbp_switch(true);
	} else if (val == 32) {
		iris_lightup_exit_abyp(true, false);
	} else if (val == 33) {
		iris_lightup_exit_abyp(true, true);
	} else if (val == 100) {
		iris_check_abyp_ready();
	} else if (val == 200) {
		iris_reset();
	} else if (val == 201) {
		iris_preload();
	}
	mutex_unlock(&pcfg->panel->panel_lock);

	return count;
}

static ssize_t _iris_abyp_dbg_read(struct file *file, char __user *buff,
								  size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];
	int buf_len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	buf_len = sizeof(bp);

	if (*ppos)
		return 0;

	tot = scnprintf(bp, buf_len,
					"abyp status gpio: %d\n", iris_check_abyp_ready());
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp mode: %d\n", iris_abyp_mode);
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp lp: %d\n", pcfg->lp_ctrl.abyp_lp);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t _iris_lp_dbg_write(struct file *file,
								 const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	if (!mutex_trylock(&pcfg->panel->panel_lock))
		return -EFAULT;

	if (val == 0) {
		iris_dynamic_power_set(false, 0);
		iris_ulps_set(false, 0);
		IRIS_LOGI("disable dpg & ulps lp");
	} else if (val == 1) {
		iris_dynamic_power_set(true, 0);
		iris_ulps_set(true, 0);
		IRIS_LOGI("enable dpg & ulps lp");
	} else if (val == 2) {
		iris_dynamic_power_set(false, 0);
		IRIS_LOGI("disable dpg");
	} else if (val == 3) {
		iris_dynamic_power_set(true, 0);
		IRIS_LOGI("enable dpg");
	} else if (val == 10) {
		pcfg->lp_ctrl.abyp_lp = 0;
		IRIS_LOGI("disable abyp lp");
	} else if (val == 11) {
		pcfg->lp_ctrl.abyp_lp = 1;
		IRIS_LOGI("set abyp lp: %d", pcfg->lp_ctrl.abyp_lp);
	} else if (val == 12) {
		pcfg->lp_ctrl.abyp_lp = 2;
		IRIS_LOGI("set abyp lp: %d", pcfg->lp_ctrl.abyp_lp);
	} else if (val == 20) {
		iris_ulps_set(false, 0);
		IRIS_LOGI("disable iris ulps lp.");
	} else if (val == 21) {
		iris_ulps_set(true, 0);
		IRIS_LOGI("enable iris ulps lp.");
	} else if (val == 255) {
		IRIS_LOGI("lp debug usages:");
		IRIS_LOGI("0  -- disable dpg & ulps lp");
		IRIS_LOGI("1  -- enable dpg & ulps lp");
		IRIS_LOGI("2  -- disable dpg");
		IRIS_LOGI("3  -- enable dpg");
		IRIS_LOGI("10  -- disable abyp lp");
		IRIS_LOGI("11  -- set abyp lp 1");
		IRIS_LOGI("12  -- set abyp lp 2");
		IRIS_LOGI("20  -- disable ulps lp");
		IRIS_LOGI("21  -- enable ulps lp");
		IRIS_LOGI("255 -- show debug usages.");
	}
	mutex_unlock(&pcfg->panel->panel_lock);
	return count;
}

static ssize_t _iris_lp_dbg_read(struct file *file, char __user *buff,
								 size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];
	int buf_len;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	buf_len = sizeof(bp);

	if (*ppos)
		return 0;

	IRIS_LOGI("dynamic power: %d", iris_dynamic_power_get());
	IRIS_LOGI("abyp lp: %d", pcfg->lp_ctrl.abyp_lp);
	IRIS_LOGI("ulps enable: %d", iris_ulps_enable_get());

	tot = scnprintf(bp, buf_len,
					"dpg: %d\n", iris_dynamic_power_get());
	tot += scnprintf(bp + tot, buf_len - tot,
					 "ulps enable: %d\n", iris_ulps_enable_get());
	tot += scnprintf(bp + tot, buf_len - tot,
					 "abyp lp: %d\n", pcfg->lp_ctrl.abyp_lp);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

int iris_dbg_init_lp(struct dsi_display *display)
{
	struct iris_cfg *pcfg;
	static const struct file_operations iris_abyp_dbg_fops = {
		.open = simple_open,
		.write = _iris_abyp_dbg_write,
		.read = _iris_abyp_dbg_read,
	};

	static const struct file_operations iris_lp_dbg_fops = {
		.open = simple_open,
		.write = _iris_lp_dbg_write,
		.read = _iris_lp_dbg_read,
	};

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("debugfs_create_dir for iris_debug failed, error %ld",
					  PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("lp_opt", 0644, pcfg->dbg_root,
					   (u32 *)&debug_lp_opt);

	debugfs_create_u32("abyp_opt", 0644, pcfg->dbg_root,
					   (u32 *)&debug_on_opt);

	debugfs_create_u32("esd_ctrl", 0644, pcfg->dbg_root,
					   &(pcfg->lp_ctrl.esd_ctrl));

	debugfs_create_u32("esd_cnt_iris", 0644, pcfg->dbg_root,
					   (u32 *)&(pcfg->lp_ctrl.esd_cnt_iris));

	debugfs_create_u32("esd_cnt_panel", 0644, pcfg->dbg_root,
					   (u32 *)&(pcfg->lp_ctrl.esd_cnt_panel));

	debugfs_create_u32("dpp_only", 0644, pcfg->dbg_root,
					   (u32 *)&(pcfg->dpp_only_enable));

	debugfs_create_u8("read_path", 0644, pcfg->dbg_root,
					  (u8 *)&(pcfg->read_path));

	debugfs_create_u8("vc_enable", 0644, pcfg->dbg_root,
					  (u8 *)&(pcfg->vc_ctrl.vc_enable));

	if (debugfs_create_file("abyp", 0644, pcfg->dbg_root, display,
							&iris_abyp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				  __FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("lp", 0644, pcfg->dbg_root, display,
							&iris_lp_dbg_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				  __FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

void iris_dsi_recover(void)
{
	SDE_ATRACE_BEGIN("iris_sys_powerdown_cmd");
	iris_send_one_wired_cmd(IRIS_POWER_DOWN_SYS);
	udelay(1000);
	iris_send_one_wired_cmd(IRIS_POWER_UP_SYS);
	udelay(1000);
	SDE_ATRACE_BEGIN("iris_sys_powerup_cmd");
}

void iris_dump_regs(u32 regs[], u32 len)
{
	u32 value, i;
	struct iris_cfg *pcfg;

	pcfg = iris_get_cfg();

	for (i = 0; i < len; i++) {
		if (pcfg->read_path == PATH_I2C) //use i2c to read
			iris_ioctl_i2c_read(regs[i], &value);
		else {
			value = iris_ocp_read(regs[i], DSI_CMD_SET_STATE_HS);
		}
		IRIS_LOGE("[%02d] %08x : %08x", i, regs[i], value);
	}
}
