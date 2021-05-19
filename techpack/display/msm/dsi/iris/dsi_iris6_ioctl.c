#include "dsi_iris6_api.h"
#include "dsi_iris6_lightup.h"
#include "dsi_iris6_lightup_ocp.h"
#include "dsi_iris6_lp.h"
#include "dsi_iris6_lut.h"
#include "dsi_iris6_pq.h"
#include "dsi_iris6_ioctl.h"
#include "dsi_iris6_i3c.h"
#include "dsi_iris6_log.h"
#include "sde_trace.h"

// 0: mipi, 1: i2c
static int adb_type = 0;

static int mdss_mipi_dsi_command(void __user *values)
{
	struct msmfb_mipi_dsi_cmd cmd;
	struct dsi_cmd_desc desc = { { 0 } };
	struct dsi_cmd_desc *pdesc_multi = NULL;
	struct dsi_cmd_desc *pdesc;
	struct dsi_panel_cmd_set cmdset = {
		.count = 1,
		.cmds = &desc
	};
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ocp_dsi_tool_input iris_ocp_input = {0, 0, 0, 0, 0};
	char *pcmd_indx;
	int ret, indx, cmd_len, cmd_cnt;

	ret = copy_from_user(&cmd, values, sizeof(cmd));
	if (ret)
		return -EFAULT;

	if (cmd.rx_length) {
		desc.msg.rx_buf = kzalloc(cmd.rx_length + 0x100, GFP_KERNEL);
		if (!desc.msg.rx_buf)
			return -ENOMEM;
		desc.msg.rx_len = cmd.rx_length;
	}
	if (cmd.length < SZ_4K && cmd.payload) {
		desc.msg.tx_buf = kmalloc(cmd.length, GFP_KERNEL);
		if (!desc.msg.tx_buf) {
			ret = -ENOMEM;
			goto err;
		}
		desc.msg.tx_len = cmd.length;
		ret = copy_from_user((char *)desc.msg.tx_buf, cmd.payload, cmd.length);
		if (ret) {
			ret = -EFAULT;
			goto err;
		}
	}

	IRIS_LOGI("%s(%d), vc: %u, type: %02x, flag: %u, cmd len: %u, rx len: %u",
			__func__, __LINE__,
			cmd.vc, cmd.dtype, cmd.flags, cmd.length, cmd.rx_length);
	IRIS_LOGI("%s(%d), ocp type: %x, ocp addr: %x, ocp size: %x",
			__func__, __LINE__,
			cmd.iris_ocp_type, cmd.iris_ocp_addr, cmd.iris_ocp_size);

	desc.msg.type = cmd.dtype;
	desc.msg.channel = cmd.vc;
	desc.last_command = (cmd.flags & MSMFB_MIPI_DSI_COMMAND_LAST) > 0;
	desc.msg.flags |= ((cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK) > 0 ? MIPI_DSI_MSG_REQ_ACK : 0);
	desc.post_wait_ms = 0;
	desc.msg.ctrl = 0;
	if (cmd.dtype == 0x0f) {
		cmd_cnt = *((u8 *)desc.msg.tx_buf);
		pdesc_multi = kmalloc(sizeof(struct dsi_cmd_desc) * cmd_cnt, GFP_KERNEL);
		pcmd_indx = (char *)desc.msg.tx_buf + cmd_cnt + 1;
		for (indx = 0; indx < cmd_cnt; indx++) {
			pdesc = pdesc_multi + indx;
			cmd_len = *((char *)desc.msg.tx_buf + 1 + indx);
			pdesc->msg.type = *pcmd_indx;
			pdesc->msg.channel = 0;
			pdesc->last_command = false;
			pdesc->msg.flags |= 0;
			pdesc->msg.tx_len = cmd_len - 1;
			pdesc->post_wait_ms = 0;
			pdesc->msg.tx_buf = pcmd_indx + 1;

			pcmd_indx += cmd_len;
			if (indx == (cmd_cnt - 1))
				pdesc->last_command = true;
			IRIS_LOGE("type: %x, len: %zu, last: %d\n",
					pdesc->msg.type, pdesc->msg.tx_len, pdesc->last_command);
		}
		cmdset.cmds = pdesc_multi;
		cmdset.count = cmd_cnt;
	}

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK)
		desc.msg.flags = desc.msg.flags | DSI_CTRL_CMD_READ;

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_HS)
		cmdset.state = DSI_CMD_SET_STATE_HS;

	mutex_lock(&pcfg->panel->panel_lock);

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_TO_PANEL) {
		if (iris_get_abyp_mode_blocking() == IRIS_PT_MODE)
			iris_pt_send_panel_cmd(pcfg->panel, &cmdset);
		else
			iris_abyp_send_panel_cmd(pcfg->panel, &cmdset);
	} else if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_T) {
		u32 pktCnt = (cmd.iris_ocp_type >> 8) & 0xFF;

		//only test LUT send command
		if ((cmd.iris_ocp_type & 0xF) == PXLW_DIRECTBUS_WRITE) {
			u8 lut_type = (cmd.iris_ocp_type >> 8) & 0xFF;
			u8 lut_index = (cmd.iris_ocp_type >> 16) & 0xFF;
			u8 lut_parse = (cmd.iris_ocp_type >> 24) & 0xFF;
			u32 lut_pkt_index = cmd.iris_ocp_addr;
			if (lut_parse) // only parse firmware when value is not zero;
				iris_parse_lut_cmds();
			iris_send_lut(lut_type, lut_index, lut_pkt_index);
		} else { // test ocp write
			if (pktCnt > DSI_CMD_CNT)
				pktCnt = DSI_CMD_CNT;

			if (cmd.iris_ocp_size < OCP_MIN_LEN)
				cmd.iris_ocp_size = OCP_MIN_LEN;

			iris_ocp_input.iris_ocp_type = cmd.iris_ocp_type & 0xF;
			iris_ocp_input.iris_ocp_cnt = pktCnt;
			iris_ocp_input.iris_ocp_addr = cmd.iris_ocp_addr;
			iris_ocp_input.iris_ocp_value = cmd.iris_ocp_value;
			iris_ocp_input.iris_ocp_size = cmd.iris_ocp_size;

			if (pktCnt)
				iris_write_test_muti_pkt(pcfg->panel, &iris_ocp_input);
			else
				iris_write_test(pcfg->panel, cmd.iris_ocp_addr, cmd.iris_ocp_type & 0xF, cmd.iris_ocp_size);
				//iris_ocp_bitmask_write(ctrl,cmd.iris_ocp_addr,cmd.iris_ocp_size,cmd.iris_ocp_value);
		}
	} else {
		iris_abyp_send_panel_cmd(pcfg->panel, &cmdset);
	}

	mutex_unlock(&pcfg->panel->panel_lock);

	if (cmd.flags & MSMFB_MIPI_DSI_COMMAND_ACK) {
		ret = copy_to_user(cmd.rx_buf, desc.msg.rx_buf, desc.msg.rx_len);
		if (ret) {
			ret = -EFAULT;
			goto err;
		}
	}
	ret = copy_to_user(values, &cmd, sizeof(cmd));
	if (ret)
		ret = -EFAULT;
err:
	kfree(desc.msg.tx_buf);
	kfree(desc.msg.rx_buf);
	kfree(pdesc_multi);
	return ret;
}


int iris_operate_tool(struct msm_iris_operate_value *argp)
{
	int ret = -1;
	uint32_t parent_type = 0;
	struct iris_cfg *pcfg = NULL;

	// FIXME: copy_from_user() is failed.
	// ret = copy_from_user(&configure, argp, sizeof(configure));
	// if (ret) {
	//	IRIS_LOGE("1st %s type = %d, value = %d\n",
	//		__func__, configure.type, configure.count);
	//	return -EPERM;
	// }
	IRIS_LOGI("%s(), type: %d(%#x), value: %d", __func__,
			argp->type, argp->type, argp->count);

	pcfg = iris_get_cfg();
	if (pcfg == NULL || pcfg->valid < PARAM_PARSED) {
		IRIS_LOGE("%s(), target display does not exist!", __func__);
		return -EPERM;
	}

	parent_type = argp->type & 0xff;
	switch (parent_type) {
	case IRIS_OPRT_TOOL_DSI:
		ret = mdss_mipi_dsi_command(argp->values);
		break;
	default:
		IRIS_LOGE("%s(), invalid opertat type: %#x",
				__func__, argp->type);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static bool _iris_special_config(u32 type)
{
	bool ret = true;
	struct iris_cfg *pcfg = iris_get_cfg();

	/*PT mode and abypass mode can use iris_configure*/
	switch (type) {
	case IRIS_DBG_KERNEL_LOG_LEVEL:
	case IRIS_WAIT_VSYNC:
	case IRIS_CHIP_VERSION:
	case IRIS_FW_UPDATE:
		return ret;
	}

	/*PT mode can use  iris_config*/
	if (iris_get_abyp_mode_blocking() == IRIS_PT_MODE) {
		if (pcfg->valid < FULL_LIGHTUP)
			ret = false;
	} else {
		/*current is not PT mode, we can only set analog mode*/
		if (type != IRIS_ANALOG_BYPASS_MODE)
			ret = false;
	}

	return ret;
}

static bool _iris_panel_off_config(u32 type)
{
    switch (type) {
    case IRIS_DBG_KERNEL_LOG_LEVEL:
    case IRIS_CHIP_VERSION:
        return true;
    }

    return false;
}

static int _iris_configure(u32 type, u32 value)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	IRIS_LOGI("%s(), type: 0x%04x(%d), value: %d", __func__, type, type, value);

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EPERM;

	if (!_iris_special_config(type)) {
		IRIS_LOGW("%s type: %d status:%d!", __func__, type, pcfg->valid);
		return -EPERM;
	}

	// Always use primary display.
	switch (type) {
	case IRIS_CM_COLOR_TEMP_MODE:
		pqlt_cur_setting->pq_setting.cmcolortempmode = value & 0x3;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode > 2)
			goto error;

		iris_cm_colortemp_mode_set(pqlt_cur_setting->pq_setting.cmcolortempmode, true);
		break;
	case IRIS_CM_COLOR_GAMUT_PRE:
		iris_cm_color_gamut_pre_set(value & 0x03);
		break;
	case IRIS_CM_COLOR_GAMUT:
		pqlt_cur_setting->pq_setting.cmcolorgamut = value;
		if (pqlt_cur_setting->pq_setting.cmcolorgamut > 6)
			goto error;

		iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut, true);
		break;
	case IRIS_S_CURVE:
		pqlt_cur_setting->scurvelevel = value;
		if (pqlt_cur_setting->scurvelevel > 3)
			goto error;
		iris_scurve_enable_set(pqlt_cur_setting->scurvelevel);
		break;
	case IRIS_DPP_PATH_MUX:
		iris_dpp_path_set(value & 0x0ff);
		break;
	case IRIS_DPORT_DISABLE:
		pqlt_cur_setting->dportdisable = value;
		iris_dport_disable(value & 0x1, 0x1);
		break;
	case IRIS_DPP_3DLUT_GAIN:
		pqlt_cur_setting->lut3d_gain = value;
		iris_dpp_3dlut_gain(pqlt_cur_setting->lut3d_gain, true);
		break;
	case IRIS_PP_DATA_PATH:
		iris_pp_datapath_set(value & 0x01);
		break;
	case IRIS_AL_ENABLE:
		pqlt_cur_setting->pq_setting.alenable = value & 0x01;
		iris_al_enable(pqlt_cur_setting->pq_setting.alenable);
		break;
	case IRIS_DBC_LEVEL:
		pqlt_cur_setting->pq_setting.dbc = value & 0x3;
		iris_dbc_level_set(pqlt_cur_setting->pq_setting.dbc);
		break;
	case IRIS_BLC_PWM_ENABLE:
		iris_pwm_enable_set(value & 0x1);
		break;
	case IRIS_DEMO_MODE:
		pqlt_cur_setting->pq_setting.demomode = value & 0x3;
		break;
	case IRIS_DYNAMIC_POWER_CTRL:
		iris_dynamic_power_set(value & 0x01, 0);
		break;
	case IRIS_DMA_LOAD:
		break;
	case IRIS_DPP_ONLY:
		if (!pcfg->dpp_only_enable) {
			IRIS_LOGW("%s dpp_only is disable!", __func__);
			goto error;
		}
		iris_dbp_switch(value & 0x01);
		break;
	case IRIS_SDR2HDR:
		iris_set_sdr2hdr_mode((value & 0xf00) >> 8);
		value = value & 0xff;
		if (value/10 == 4) {/*magic code to enable YUV input.*/
			value -= 40;
		} else if (value/10 == 6) {
			iris_set_HDR10_YCoCg(true);
			value -= 60;
		} else {
			iris_set_HDR10_YCoCg(false);
		}

		if (pqlt_cur_setting->pq_setting.sdr2hdr != value
			&& iris_sdr2hdr_valid(value)) {
			pqlt_cur_setting->pq_setting.sdr2hdr = value;
			iris_sdr2hdr_level_set(pqlt_cur_setting->pq_setting.sdr2hdr, true);
		}
		break;
	case IRIS_COLOR_TEMP_VALUE:
		pqlt_cur_setting->colortempvalue = value;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL)
			iris_cm_color_temp_set();
		break;
	case IRIS_CCT_VALUE:
		pqlt_cur_setting->cctvalue = value;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_AUTO)
			iris_cm_color_temp_set();
		break;
	case IRIS_LUX_VALUE:
		/* move to iris_configure_ex*/
		pqlt_cur_setting->luxvalue = value;
		iris_lux_set(pqlt_cur_setting->luxvalue);
		break;
	case IRIS_HDR_MAXCLL:
		pqlt_cur_setting->maxcll = value;
		iris_sdr2hdr_maxcll_set(pqlt_cur_setting->maxcll);
		break;
	case IRIS_ANALOG_BYPASS_MODE:
		if ((value & 0x7F) == iris_get_abyp_mode_blocking()) {
			IRIS_LOGD("%s(), same bypass mode", __func__);
			break;
		}
		if ((value & 0x7F) == IRIS_ABYP_MODE) {
			//iris_panel_nits_set(0, true, value);
			//iris_quality_setting_off();
		}
		iris_abyp_switch_proc(pcfg->display, value, true);
		break;
	case IRIS_HDR_PANEL_NITES_SET:
		if (pqlt_cur_setting->al_bl_ratio != value) {
			pqlt_cur_setting->al_bl_ratio = value;
			iris_panel_nits_set(value, false, pqlt_cur_setting->pq_setting.sdr2hdr);
		}
		break;
		break;
	case IRIS_DBC_LED_GAIN:
		iris_dbc_led0d_gain_set(value & 0x3f);
		break;
	case IRIS_SCALER_PP_FILTER_LEVEL:
		iris_scaler_filter_update(SCALER_PP, value & 0x3f);
		break;
	case IRIS_HDR_PREPARE:
		//if ((value == 0) || ((value == 1) && !iris_get_debug_cap()) || (value == 2))
			//iris_hdr_csc_prepare();
		//else if (value == 3)
		iris_set_skip_dma(true);
		break;
	case IRIS_CLEAR_TRIGGER:
		iris_clear_init_trigger();
	break;
	case IRIS_HDR_COMPLETE:
		//if ((value == 3) || (value == 5))
			iris_set_skip_dma(false);
		//if ((value == 0) || ((value == 1) && !iris_get_debug_cap()))
		//	iris_hdr_csc_complete(value);
		//else if (value >= 2)
			iris_hdr_csc_complete(value);

		/*if (value != 2 && value != 4) {
			if (pqlt_cur_setting->pq_setting.sdr2hdr == SDR2HDR_Bypass)
				iris_panel_nits_set(0, true, value);
			else
				iris_panel_nits_set(PANEL_BL_MAX_RATIO, false, value);
		}*/
		break;
	case IRIS_DEBUG_CAP:
		iris_set_debug_cap(value & 0x01);
		break;
	case IRIS_DPORT_DIS_DEBUG:
		iris_set_dportdis_debug(value & 0x01);
		break;
	case IRIS_FW_UPDATE:
		// Need do multi-thread protection.
		if (value == 0) {
			/* before parsing firmware, free ip & opt buffer which alloc for LUT,
			 * if loading firmware failed before, need realloc seq space after
			 * updating firmware
			 */
			u8 firmware_status = iris_get_firmware_status();

			iris_free_ipopt_buf(IRIS_LUT_PIP_IDX);
			if (iris_parse_lut_cmds()) {
				IRIS_LOGE("%s(), fail to parse lut cmds", __func__);
				break;
			}
			if (firmware_status == FIRMWARE_LOAD_FAIL) {
				iris_free_seq_space();
				iris_alloc_seq_space();
			}
			if (iris_get_firmware_status() == FIRMWARE_LOAD_SUCCESS) {
				u8 lut_index = 0x00; //17-bin 3d-lut, 6 tables
				u8 i = 0;
				bool pt_switching = false;
				int mode_tmp = MAX_MODE;

				mode_tmp = iris_get_abyp_mode_blocking();
				if (mode_tmp == IRIS_ABYP_MODE) {
					/*switch to pt mode*/
					iris_abyp_switch_proc(pcfg->display, IRIS_PT_MODE, true);
					pt_switching = true;
				}
				mode_tmp = iris_get_abyp_mode_blocking();
				if (mode_tmp != IRIS_PT_MODE) {
					IRIS_LOGE("%s(), Current Need PT Mode  pt_switching:%d  mode_tmp:%d", __func__, pt_switching, mode_tmp);
				} else {
					IRIS_LOGE("%s(), Current is PT Mode  pt_switching:%d  mode_tmp:%d", __func__, pt_switching, mode_tmp);
					for (i = 0; i < 0x6; i++) {
						if (i != 0x5)
							iris_dpp_3dlut_send((lut_index+i), 1);
						else
							iris_dpp_3dlut_send((lut_index+i), 0);
					}
					iris_cm_color_gamut_set(pqlt_cur_setting->pq_setting.cmcolorgamut, true);
					//iris_scaler_gamma_enable(false, 1);
					iris_update_fw_status(FIRMWARE_IN_USING);
				}

				if (pt_switching == true) {
					/*switch to abyp mode*/
					iris_abyp_switch_proc(pcfg->display, IRIS_ABYP_MODE, true);
					mode_tmp = iris_get_abyp_mode_blocking();
					if (mode_tmp != IRIS_ABYP_MODE) {
						IRIS_LOGE("%s(), Current Should Change Back To ABYP Mode", __func__);
					}
				}
			}
		}
		break;
	case IRIS_DBG_KERNEL_LOG_LEVEL:
		iris_set_loglevel(value);
		break;
	case IRIS_WAIT_VSYNC:
		iris_wait_vsync(value);
		break;
	case IRIS_SDR2HDR_AI_ENALE:
		//iris_hdr_ai_enable(value);
		pqlt_cur_setting->ai_auto_en = value;
		switch (value) {
		case AI_AMBIENT_BACKLIGHT_DISABLE:
			iris_hdr_ai_input_al(500);
			iris_hdr_ai_input_bl(250);
			break;
		case AI_AMBIENT_ENABLE:
			iris_hdr_ai_input_al(pqlt_cur_setting->ai_ambient);
			iris_hdr_ai_input_bl(250);
			break;
		case AI_BACKLIGHT_ENABLE:
			iris_hdr_ai_input_al(500);
			iris_hdr_ai_input_bl(pqlt_cur_setting->ai_backlight);
			break;
		case AI_AMBIENT_BACKLIGHT_ENABLE:
			iris_hdr_ai_input_al(pqlt_cur_setting->ai_ambient);
			iris_hdr_ai_input_bl(pqlt_cur_setting->ai_backlight);
			break;
		default:
			break;
		}
		break;
	case IRIS_SDR2HDR_AI_INPUT_AMBIENTLIGHT:
		pqlt_cur_setting->ai_ambient = value;
		if (pqlt_cur_setting->ai_auto_en == AI_AMBIENT_ENABLE
			|| pqlt_cur_setting->ai_auto_en == AI_AMBIENT_BACKLIGHT_ENABLE)
			iris_hdr_ai_input_al(value);
		break;
	case IRIS_SDR2HDR_AI_INPUT_PANEL_NITS:
		iris_hdr_ai_input_panel_nits(value);
		break;
	case IRIS_SDR2HDR_AI_INPUT_BACKLIGHT:
		pqlt_cur_setting->ai_backlight = value;
		if (pqlt_cur_setting->ai_auto_en == AI_BACKLIGHT_ENABLE
			|| pqlt_cur_setting->ai_auto_en == AI_AMBIENT_BACKLIGHT_ENABLE)
			iris_hdr_ai_input_bl(value);
		break;
	case IRIS_SDR2HDR_AI_CTRL_TM_Y:
		iris_hdr_ai_ctrl_tm_y(value);
		break;
	case IRIS_SDR2HDR_AI_CTRL_TM_K:
		iris_hdr_ai_ctrl_tm_k(value);
		break;
	case IRIS_SDR2HDR_AI_CTRL_CAM_GAIN:
		iris_hdr_ai_ctrl_cam_gain(value);
		break;
	case IRIS_SDR2HDR_AI_CTRL_S_CURVE:
		iris_hdr_ai_ctrl_s_curve(value);
		break;
	case IRIS_SDR2HDR_AI_CTRL_GRAPHIC_WEIGHT:
		iris_hdr_ai_ctrl_graphic_weight(value);
		break;
	case IRIS_SDR2HDR_AI_CTRL_FADE:
		iris_hdr_ai_ctrl_fade(value);
		break;
	case IRIS_SDR2HDR_LCE:
		pqlt_cur_setting->sdr2hdr_lce = value;
		iris_sdr2hdr_lce(value);
		break;
	case IRIS_SDR2HDR_SCURVE:
		pqlt_cur_setting->sdr2hdr_scurve = value;
		iris_sdr2hdr_scurve(value);
		break;
	case IRIS_SDR2HDR_TF_COEF:
		if (value != pqlt_cur_setting->sdr2hdr_scurve) {
			pqlt_cur_setting->sdr2hdr_scurve = value;
			iris_sdr2hdr_tf_coef_set(value);
		}
		break;
	case IRIS_SDR2HDR_FTC:
		pqlt_cur_setting->sdr2hdr_ftc = value;
		iris_sdr2hdr_ftc(value);
		break;
	case IRIS_SDR2HDR_AI_SWITCH_PRE:
		iris_sdr2hdr_kb_switch_pre();
		break;
	case IRIS_SDR2HDR_AI_SWITCH:
		pqlt_cur_setting->ai_kb_switch = value;
		iris_sdr2hdr_kb_switch(value);
		break;
	default:
		goto error;
	}

	return 0;

error:
	return -EINVAL;

}

#define SYNC_CNT (2)
static uint32_t kickCnt;

int update_dc_brightness(uint32_t *dimmingvalues, uint32_t dimmingEnable,  uint32_t brightness, uint32_t syncount)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	uint32_t bl_lvl;
	u32 timeus0;
	ktime_t ktime0, ktime1;
	char atrace_buf[32];
	int event = 0;
	int dim_on_off = 0;

	if (!pcfg || !pcfg->panel) {
		IRIS_LOGE("pcfg parameter %p", pcfg);
		return -EINVAL;
	}

	bl_lvl = brightness;
	ktime0 = ktime_get();
	/* turn on off dc */
	if ((!iris_setting->quality_cur.dimmingEnable && dimmingEnable)
		|| (iris_setting->quality_cur.dimmingEnable && !dimmingEnable)) {
		dim_on_off = 1;
		IRIS_LOGW("dim_on_off %d", dim_on_off);
		if (dimmingEnable)
			event = 1;
	}
	if (dim_on_off) {
		snprintf(atrace_buf, sizeof(atrace_buf),
			"update_dc_brightness_%d", event);
		SDE_ATRACE_BEGIN(atrace_buf);
	}



	iris_setting->quality_cur.dimmingEnable = dimmingEnable;

	if (((syncount == 0)  && pcfg->dimming_first)
		|| ((pcfg->dimming_first == 0) && (syncount == SYNC_CNT)))
		iris_dcDimming_backlight_set(dimmingvalues);


	if (((syncount == SYNC_CNT) && pcfg->dimming_first) || ((pcfg->dimming_first == 0) && (syncount == 0))) {
		if (bl_lvl != iris_setting->quality_cur.system_brightness) {
			//if (dim_on_off)
			//	iris_wait_vsync(1);
			SDE_ATRACE_BEGIN("update_brightness");
			iris_update_backlight(bl_lvl);
			SDE_ATRACE_END("update_brightness");
		}
	}

	if (dim_on_off) {
		SDE_ATRACE_END(atrace_buf);
		ktime1 = ktime_get();
		timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
		IRIS_LOGI("dc %d time %d\n", event, timeus0);
	}

	return 0;
}

int iris_update_dc_brightness(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcfg || !pcfg->panel) {
		IRIS_LOGE("pcfg parameter %p", pcfg);
		return -EINVAL;
	}
	mutex_lock(&pcfg->panel->panel_lock);
	if (pcfg->dc_bl_pending == 1) {
		IRIS_LOGD("dc_bl_pending = 1 count: %d ", kickCnt);
		update_dc_brightness(&pcfg->pend_dimming[0], pcfg->pend_dimmingEnable,  pcfg->pend_brightness, kickCnt);
		kickCnt++;
		if (kickCnt == (SYNC_CNT+1)) {
			pcfg->dc_bl_pending = 0;
			kickCnt = 0;
		}
	}
	mutex_unlock(&pcfg->panel->panel_lock);
	return 0;
}

int iris_dc_on_off_pending(void)
{
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();

	if (pcfg->dc_bl_pending == 1) {
		IRIS_LOGW("dc_bl_pending skip setting backlight\n");
		return 1;
	}

	return 0;
}

int iris_configure(u32 type, u32 value)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc;
	ktime_t ktime;

	IRIS_LOGI("%s type=0x%04x, value=%d", __func__, type, value);

	if (!_iris_panel_off_config(type) &&
		!dsi_panel_initialized(pcfg->panel)) {
		IRIS_LOGE("%s(), panel is not initialized! type=0x%04x", __func__, type);
		return -2;
	}

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EPERM;

	switch (type) {
	case IRIS_DEMO_MODE:
	case IRIS_HDR_MAXCLL:
	case IRIS_DEBUG_CAP:
	case IRIS_DBG_KERNEL_LOG_LEVEL:
	case IRIS_WAIT_VSYNC:
	case IRIS_DPORT_DIS_DEBUG:
		/* don't lock panel_lock */
		return _iris_configure(type, value);
	}

	if (IRIS_IF_LOGI())
		ktime = ktime_get();

	mutex_lock(&pcfg->panel->panel_lock);
	rc = _iris_configure(type, value);
	mutex_unlock(&pcfg->panel->panel_lock);

	if (IRIS_IF_LOGI())
		IRIS_LOGI("%s(), spend %u us for type 0x%04x(%u) value %#x(%u)",
				__func__,
				(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime),
				type, type, value, value);
	return rc;
}

int iris_configure_t(u32 type, void __user *argp)
{
	int ret = -1;
	uint32_t value = 0;

	ret = copy_from_user(&value, argp, sizeof(uint32_t));
	if (ret) {
		IRIS_LOGE("can not copy from user");
		return -EPERM;
	}

	ret = iris_configure(type, value);
	return ret;
}

int iris_configure_ex(u32 type, u32 count, u32 *values)
{
	int ret = -1;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	struct msmfb_iris_tm_points_info iris_tm_points;
	struct msmfb_iris_tm_points_info *iris_tm_points_lut = NULL;

	IRIS_LOGI("%s type=0x%04x, count=%d, value=%d", __func__, type, count, values[0]);

	if (!dsi_panel_initialized(pcfg->panel)) {
		IRIS_LOGE("%s(), panel is not initialized!", __func__);
		return -2;
	}

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EPERM;

	if (!_iris_special_config(type)) {
		IRIS_LOGE("%s type=%d status=%d", __func__, type, pcfg->valid);
		return -EPERM;
	}

	mutex_lock(&pcfg->panel->panel_lock);

	// Always use primary display.
	switch (type) {
	case IRIS_HDR_MAXCLL:
		break;
	case IRIS_CCF1_UPDATE:
		/* Nothing to do for Iirs5*/
		break;
	case IRIS_CCF2_UPDATE:
		/* Nothing to do for Iirs5*/
		break;
	case IRIS_HUE_SAT_ADJ:
		IRIS_LOGD("dpp cm csc value: csc0 = 0x%x, csc1 = 0x%x, csc2 = 0x%x", values[0], values[1], values[2]);
		IRIS_LOGD("dpp cm csc value:  csc3 = 0x%x, csc4 = 0x%x", values[3], values[4]);
		iris_dpp_cmcsc_level_set(&values[0]);
		break;
	case IRIS_READING_MODE:
		iris_reading_mode_set(&values[0]);
		break;
	case IRIS_CONTRAST_DIMMING:
		{
			IRIS_LOGI("new dimming: 0x%x, old: 0x%x, bl: %d", values[1], pcfg->pend_dimming[1], values[3]);

			pcfg->pend_dimming[0] = values[0];
			pcfg->pend_dimming[1] = values[1];
			pcfg->pend_dimming[2] = values[2];
			pcfg->pend_dimming[3] = values[3];
			pcfg->pend_dimming[4] = values[4];
			pcfg->pend_dimmingEnable = values[4];
			pcfg->pend_brightness = values[3];
			pcfg->dc_bl_pending = 0;
			kickCnt = 0;

			if (pcfg->panel->bl_config.bl_inverted_dbv) {
				uint32_t bl_lvl = values[3];

				bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));
				pcfg->pend_brightness = bl_lvl;
			}
		}
		iris_dcDimming_backlight_set(&values[0]);
		break;
	case IRIS_DC_DIMMING:
		if (count == 4) {
			iris_dcDimming_enable(values[0]);
			iris_dcDimming_set(values[1], values[2], values[3]);
		} else if (count == 3) {
			iris_dcDimming_set(values[0], values[1], values[2]);
		} else {
			iris_dcDimming_enable(values[0]);
		}
		break;
	case IRIS_COLOR_TEMP_VALUE:
		pqlt_cur_setting->colortempvalue = values[0];

		if (count > 3) {
			pqlt_cur_setting->min_colortempvalue = values[2];
			pqlt_cur_setting->max_colortempvalue = values[3];
		} else {
			pqlt_cur_setting->min_colortempvalue = 0;
			pqlt_cur_setting->max_colortempvalue = 0;
		}
		if (pqlt_cur_setting->pq_setting.cmcolortempmode == IRIS_COLOR_TEMP_MANUL)
			iris_cm_color_temp_set();

		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_SET:
		if (adb_type == 0) {
			iris_ocp_write_val(values[0], values[1]);
		} else if (adb_type == 1) {
			if (pcfg->iris_i2c_write) {
				if (pcfg->iris_i2c_write(values[0], values[1]) < 0)
					IRIS_LOGE("i2c set reg fails, reg=0x%x, val=0x%x", values[0], values[1]);
			} else {
				IRIS_LOGE("Game Station is not connected");
			}
		}
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_SET2:
		if (adb_type == 0) {
			IRIS_LOGD("%s,%d: path select dsi", __func__, __LINE__);
			iris_ocp_write_vals(values[0], values[1], count-2, values+2, DSI_CMD_SET_STATE_HS);
		} else {
			IRIS_LOGD("%s,%d: path select i2c, header = %x", __func__, __LINE__, values[0]);
			if ((values[0] & 0x0f) == 0x0c) {
				IRIS_LOGD("%s,%d: direct bus write", __func__, __LINE__);
				iris_i2c_write(&values[1], count-2, I3C_DIRECT_WR_OP, (values[0] >> 24) & 0x0F);
			} else if ((values[0] & 0x0f) == 0x00) {
				IRIS_LOGD("%s,%d: ocp burst write", __func__, __LINE__);
				iris_i2c_write(&values[1], count-2, I3C_OCP_BURST_WR_OP, (values[0] >> 24) & 0x0F);
			} else if ((values[0] & 0x0f) == 0x05) {
				IRIS_LOGD("%s,%d: bit enable write", __func__, __LINE__);
				iris_i2c_bit_en_op(values[1], values[2], values[3]);
			}
		}
		break;
	case IRIS_CM_COLOR_TEMP_MODE:
		// phone
		pqlt_cur_setting->pq_setting.cmcolortempmode = values[0] & 0x3;
		if (pqlt_cur_setting->pq_setting.cmcolortempmode > 2)
			goto error;

		iris_cm_colortemp_mode_set(pqlt_cur_setting->pq_setting.cmcolortempmode, true);
		break;
	case IRIS_CSC_MATRIX:
		if (count > 9) {
			iris_dpp_cmcsc_level_set(&values[2]);
		} else
			return -EPERM;
		break;
	case IRIS_DBG_SEND_PACKAGE:
		ret = iris_send_ipopt_cmds(values[0], values[1]);
		IRIS_LOGD("iris config sends package: ip: %#x, opt: %#x, send: %d.",
			values[0], values[1], ret);
		break;
	case IRIS_DPP_DEMO_WINDOW:
		if (count > 4)
			goto error;

		iris_dpp_demo_window_set(values[0], values[1], values[2], values[3]);
		break;
	case IRIS_FINGER_DISPLAY:
		if (count > 5)
			goto error;

		iris_dpp_fingerDisplay_set(values[0], values[1], values[2], values[3], values[4]);
		break;
	case IRIS_DPP_FADE_INOUT:
		 //pr_err("fade_inout, count %d, values[%d, %d]", count, values[0], values[1]);
		if (count == 2)
			iris_dpp_fadeinout_step(values[0], values[1]);
		else if (count == 1)
			iris_dpp_fadeinout_enable(values[0]);
		else
			goto error;
		break;
	case IRIS_GAMMA_MODE:
		if (count > 2)
			goto error;
		if (count == 1)
			iris_dpp_gammamode_set(values[0], 0);
		else
			iris_dpp_gammamode_set(values[0], values[1]);
		break;
	case IRIS_HDR10PLUS:
		iris_tm_points_lut = iris_get_tm_points_info();
		iris_tm_points = *(struct msmfb_iris_tm_points_info *)(values);

		if (iris_tm_points.lut_lutx_payload != NULL) {
			ret = copy_from_user(iris_tm_points_lut->lut_lutx_payload, iris_tm_points.lut_lutx_payload, sizeof(uint32_t)*15);
			if (ret) {
				IRIS_LOGE("can not copy lut x from user sdr2hdr");
				goto error;
			}
		}
		if (iris_tm_points.lut_luty_payload != NULL) {
			ret = copy_from_user(iris_tm_points_lut->lut_luty_payload, iris_tm_points.lut_luty_payload, sizeof(uint32_t)*15);
			if (ret) {
				IRIS_LOGE("can not copy lut y from user sdr2hdr");
				goto error;
			}
		}
		iris_update_tm_lut();
		break;
	default:
		goto error;
	}

	mutex_unlock(&pcfg->panel->panel_lock);
	return 0;

error:
	mutex_unlock(&pcfg->panel->panel_lock);
	return -EINVAL;
}

static int iris_configure_ex_t(uint32_t type,
								uint32_t count, void __user *values)
{
	int ret = -1;
	uint32_t *val = NULL;

	val = vmalloc(count * sizeof(uint32_t));
	if (!val) {
		IRIS_LOGE("can not malloc space");
		return -ENOSPC;
	}
	ret = copy_from_user(val, values, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("can not copy from user");
		vfree(val);
		return -EPERM;
	}
	ret = iris_configure_ex(type, count, val);
	vfree(val);
	return ret;
}

int iris_configure_get(u32 type, u32 count, u32 *values)
{
	int rc = -1;
	int top_pmu_status = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;
	u32 reg_addr, reg_val;

	if (!_iris_panel_off_config(type) &&
		!dsi_panel_initialized(pcfg->panel)) {
		IRIS_LOGE("%s(), panel is not initialized! type=0x%04x", __func__, type);
		return -2;
	}

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EINVAL;

	switch (type) {
	case IRIS_CM_COLOR_TEMP_MODE:
		*values = pqlt_cur_setting->pq_setting.cmcolortempmode;
		break;
	case IRIS_CM_COLOR_GAMUT:
		*values = pqlt_cur_setting->pq_setting.cmcolorgamut;
		break;
	case IRIS_GRAPHIC_DET_ENABLE:
		*values = pqlt_cur_setting->pq_setting.graphicdet;
		break;
	case IRIS_AL_ENABLE:
		*values = pqlt_cur_setting->pq_setting.alenable;
		break;
	case IRIS_DBC_LEVEL:
		*values = pqlt_cur_setting->pq_setting.dbc;
		break;
	case IRIS_DEMO_MODE:
		*values = pqlt_cur_setting->pq_setting.demomode;
		break;
	case IRIS_DPORT_DISABLE:
		*values = pqlt_cur_setting->dportdisable;
		break;
	case IRIS_DPP_3DLUT_GAIN:
		*values = pqlt_cur_setting->lut3d_gain;
		break;
	case IRIS_SDR2HDR:
		*values = pqlt_cur_setting->pq_setting.sdr2hdr;
		break;
	case IRIS_LUX_VALUE:
		*values = pqlt_cur_setting->luxvalue;
		break;
	case IRIS_DYNAMIC_POWER_CTRL:
		*values = iris_dynamic_power_get();
		break;
	case IRIS_HDR_MAXCLL:
		*values = pqlt_cur_setting->maxcll;
		break;
	case IRIS_ANALOG_BYPASS_MODE:
		*values = iris_get_abyp_mode_nonblocking();
		break;
	case IRIS_DPP_ONLY:
		if (!pcfg->dpp_only_enable) {
			IRIS_LOGW("%s dpp_only is disable!", __func__);
			return -EFAULT;
		}
		*values = pcfg->lp_ctrl.dbp_mode;
		break;
	case IRIS_CM_COLOR_GAMUT_PRE:
		*values = pqlt_cur_setting->source_switch;
		break;
	case IRIS_CCT_VALUE:
		*values = pqlt_cur_setting->cctvalue;
		break;
	case IRIS_COLOR_TEMP_VALUE:
		*values = pqlt_cur_setting->colortempvalue;
		break;
	case IRIS_CHIP_VERSION:
		*values = 0;
		if (pcfg->iris_chip_enable)
			*values |= (1 << IRIS6_VER);
		if (pcfg->iris_soft_enable)
			*values |= (1 << IRISSOFT_VER);
		if (pcfg->iris_chip_enable && pcfg->iris_soft_enable)
			*values |= (1 << IRIS6DUAL_VER);
		if (pcfg->iris_default_mode_pt)
			*values |= (1 << IRIS_DEFAULT_MODE_PT);
		if (*values == 0)
			return -EFAULT;
		IRIS_LOGE("IRIS_CHIP_VERSION:0x%08x", *values);
		break;
	case IRIS_PANEL_TYPE:
		*values = pcfg->panel_type;
		break;
	case IRIS_PANEL_NITS:
		*values = pcfg->panel_nits;
		break;
	case IRIS_MCF_DATA:
		mutex_lock(&pcfg->panel->panel_lock);
		// read panel MCF data via mipi
		mutex_unlock(&pcfg->panel->panel_lock);
		break;
	case IRIS_DBG_TARGET_REGADDR_VALUE_GET:
		if ((iris_get_abyp_mode_blocking() == IRIS_ABYP_MODE) && (adb_type != 1))
			return -ENOTCONN;

		if (adb_type == 0) {
			mutex_lock(&pcfg->panel->panel_lock);
			*values = iris_ocp_read(*values, DSI_CMD_SET_STATE_HS);
			mutex_unlock(&pcfg->panel->panel_lock);
		} else if (adb_type == 1) {
			reg_addr = *values;
			if (pcfg->iris_i2c_read) {
				if (pcfg->iris_i2c_read(reg_addr, &reg_val) < 0)
					IRIS_LOGE("i2c read reg fails, reg=0x%x", reg_addr);
				else
					*values = reg_val;
			} else {
				IRIS_LOGE("Game Station is not connected");
			}
		}
		break;
	case IRIS_DBG_KERNEL_LOG_LEVEL:
		*values = iris_get_loglevel();
		break;
	case IRIS_WORK_MODE:
		*values = ((int)pcfg->tx_mode<<8) | ((int)pcfg->rx_mode);
		rc = iris_ioctl_i2c_read(IRIS_TOPPMU_STATUS, &top_pmu_status);
		if (!rc)
			if (!(top_pmu_status & 0x10000))
				*values |= 0x10000;
		IRIS_LOGI("rc = %d, top_pmu_status = 0x%08x, work_mode = 0x%08x", rc, top_pmu_status, *values);
		break;
	case IRIS_SDR2HDR_LCE:
		*values = pqlt_cur_setting->sdr2hdr_lce;
		break;
	case IRIS_SDR2HDR_SCURVE:
		*values = pqlt_cur_setting->sdr2hdr_scurve;
		break;
	case IRIS_SDR2HDR_TF_COEF:
		*values = pqlt_cur_setting->sdr2hdr_tf_coef;
		break;
	case IRIS_SDR2HDR_FTC:
		*values = pqlt_cur_setting->sdr2hdr_ftc;
		break;
	case IRIS_SDR2HDR_AI_ENALE:
		*values = pqlt_cur_setting->ai_auto_en;
		break;
	case IRIS_SDR2HDR_AI_INPUT_AMBIENTLIGHT:
		*values = pqlt_cur_setting->ai_ambient;
		break;
	case IRIS_SDR2HDR_AI_SWITCH:
		*values = pqlt_cur_setting->ai_kb_switch;
		break;
	case IRIS_SDR2HDR_AI_INPUT_PANEL_NITS:
	case IRIS_SDR2HDR_AI_INPUT_BACKLIGHT:
	case IRIS_SDR2HDR_AI_CTRL_TM_Y:
	case IRIS_SDR2HDR_AI_CTRL_TM_K:
	case IRIS_SDR2HDR_AI_CTRL_CAM_GAIN:
	case IRIS_SDR2HDR_AI_CTRL_S_CURVE:
	case IRIS_SDR2HDR_AI_CTRL_GRAPHIC_WEIGHT:
	case IRIS_SDR2HDR_AI_CTRL_FADE:
		*values = 0;
		break;
	default:
		return -EFAULT;
	}

	IRIS_LOGI("%s type=0x%04x, value=%d", __func__, type, *values);
	return 0;
}

int iris_configure_get_t(uint32_t type,
						uint32_t count, void __user *values)
{
	int ret = -1;
	uint32_t *val = NULL;

	val = vmalloc(count * sizeof(uint32_t));
	if (val == NULL) {
		IRIS_LOGE("could not malloc space for func = %s", __func__);
		return -ENOSPC;
	}
	ret = copy_from_user(val, values, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("can not copy from user");
		vfree(val);
		return -EPERM;
	}
	ret = iris_configure_get(type, count, val);
	if (ret) {
		IRIS_LOGE("get error type:%d", type);
		vfree(val);
		return ret;
	}
	ret = copy_to_user(values, val, sizeof(uint32_t) * count);
	if (ret) {
		IRIS_LOGE("copy to user error");
		vfree(val);
		return -EPERM;
	}
	vfree(val);
	return ret;
}

int iris_operate_conf(struct msm_iris_operate_value *argp)
{
	int ret = -1;
	uint32_t parent_type = 0;
	uint32_t child_type = 0;
	struct iris_cfg *pcfg = NULL;

	IRIS_LOGD("%s(), type: 0x%04x", __func__, argp->type);

	parent_type = argp->type & 0xff;
	child_type = (argp->type >> 8) & 0xff;
	/*close panel*/
	pcfg = iris_get_cfg();
	if (pcfg == NULL || pcfg->valid < PARAM_PARSED) {
		IRIS_LOGE("%s(), target display does not exist!", __func__);
		return -EPERM;
	}

	if (IRIS_CHIP_VERSION != child_type && pcfg->cont_splash_status == 0) {
		IRIS_LOGE("%s(), splash not complete:%d", __func__, pcfg->cont_splash_status);
		return -EPERM;
	}

	switch (parent_type) {
	case IRIS_OPRT_CONFIGURE:
		ret = iris_configure_t(child_type, argp->values);
		break;
	case IRIS_OPRT_CONFIGURE_NEW:
		ret = iris_configure_ex_t(child_type, argp->count, argp->values);
		break;
	case IRIS_OPRT_CONFIGURE_NEW_GET:
		ret = iris_configure_get_t(child_type, argp->count, argp->values);
		break;
	default:
		IRIS_LOGE("could not find right operate type = %d", argp->type);
		break;
	}

	return ret;
}

static ssize_t iris_adb_type_read(struct file *file, char __user *buff,
				size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	tot = scnprintf(bp, sizeof(bp), "%d\n", adb_type);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static ssize_t iris_adb_type_write(struct file *file,
	const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	adb_type = val;

	return count;
}

static const struct file_operations iris_adb_type_write_fops = {
	.open = simple_open,
	.write = iris_adb_type_write,
	.read = iris_adb_type_read,
};

int iris_dbg_init_adb_type(struct dsi_display *display)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (debugfs_create_file("adb_type", 0644, pcfg->dbg_root, display,
				&iris_adb_type_write_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail\n",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

/* Iris log level definition, for 'dsi_iris6_log.h' */
static int iris_log_level = 2;

void iris_set_loglevel(int level)
{
	iris_log_level = level;
}

inline int iris_get_loglevel(void)
{
	return iris_log_level;
}
