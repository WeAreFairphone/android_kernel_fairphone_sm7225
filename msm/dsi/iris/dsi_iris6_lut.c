#include <linux/firmware.h>
#include <linux/debugfs.h>
#include "dsi_iris6_api.h"
#include "dsi_iris6_lut.h"
#include "dsi_iris6_lightup.h"
#include "dsi_iris6_log.h"


#define IRIS_FIRMWARE_NAME	"iris6.fw"
#define IRIS_CCF1_FIRMWARE_NAME "iris6_ccf1.fw"
#define IRIS_CCF2_FIRMWARE_NAME "iris6_ccf2.fw"
#define IRIS_CCF3_FIRMWARE_NAME "iris6_ccf3.fw"
#define IRIS_CCF1_CALIBRATED_FIRMWARE_NAME "iris6_ccf1b.fw"
#define IRIS_CCF2_CALIBRATED_FIRMWARE_NAME "iris6_ccf2b.fw"
#define IRIS_CCF3_CALIBRATED_FIRMWARE_NAME "iris6_ccf3b.fw"

#define DIRECT_BUS_HEADER_SIZE 8
#define GAMMA_LUT_NUMBER 6//12   //00, 01, 02, 10, 20, 30
#define CM_LUT_NUMBER 38
#define SCALER1D_LUT_NUMBER 9
#define SDR2HDR_LUT_BLOCK_SIZE (128*4)
#define SDR2HDR_LUT2_BLOCK_NUMBER (6)
#define SDR2HDR_LUTUVY_BLOCK_NUMBER (12)
#define SDR2HDR_LUT2_ADDRESS 0x3000
#define SDR2HDR_LUTUVY_ADDRESS 0x6000
#define SDR2HDR_LUT_BLOCK_ADDRESS_INC 0x400
#define SDR2HDR_LUT2_BLOCK_CNT (6)  //for ambient light lut
#define SDR2HDR_LUTUVY_BLOCK_CNT (12)  // for maxcll lut

struct lut_node {
	u32 lut_cmd_cnts_max;
	u32 hdr_lut2_pkt_cnt;
	u32 hdr_lutuvy_pkt_cnt;
};

static u8 payload_size;
static uint32_t lut_lut2[LUT_LEN] = {};
static uint32_t lut2_fw[3 * LUT_LEN] = {};

static struct iris_ambient_info iris_ambient;

static u8 *iris_ambient_buf;
static struct dsi_cmd_desc *dynamic_lut_send_cmd;

static u8 *iris_maxcll_buf;
static struct dsi_cmd_desc *dynamic_lutuvy_send_cmd;

static struct lut_node iris_lut_param;
static u8 fw_load_status = FIRMWARE_LOAD_FAIL;
static u16 fw_calibrate_status;
static u8 gamma_apl_status;

u8 iris_get_firmware_status(void)
{
	return fw_load_status;
}

void iris_update_fw_status(u8 value)
{
	fw_load_status = value;
}

u8 iris_get_firmware_aplstatus_value(void)
{
	return gamma_apl_status;
}

struct iris_ambient_info *iris_get_ambient_lut(void)
{
	return &iris_ambient;
}

static void _iris_init_ambient_lut(void)
{
	iris_ambient.lux = 0;
	iris_ambient.bl_ratio = 0;
	iris_ambient.payload = &lut_lut2;

	if (iris_ambient_buf != NULL) {
		vfree(iris_ambient_buf);
		iris_ambient_buf = NULL;
	}

	dynamic_lut_send_cmd = NULL;
}

static void _iris_init_lut_buf(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	payload_size = pcfg->split_pkt_size;
	memset(&iris_lut_param, 0x00, sizeof(iris_lut_param));

	/* for HDR ambient light */
	_iris_init_ambient_lut();

	iris_init_tm_points_lut();
}

static int32_t _iris_request_firmware(const struct firmware **fw,
		const uint8_t *name)
{
	int32_t rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct device *dev = &pcfg->display->pdev->dev;

	if (name == NULL) {
		IRIS_LOGE("%s(), firmware is null", __func__);
		return -EINVAL;
	}
 
  if (pcfg->iris_isolate_status == 1) {
    return -EINVAL;
  }

	rc = request_firmware(fw, name, dev);
	if (rc) {
		IRIS_LOGE("%s(), failed to request firmware: %s, rc: %d",
				__func__, name, rc);
		return rc;
	}

	IRIS_LOGI("%s(), request firmware: %s, size: %zu bytes",
			__func__, name, (*fw)->size);

	return rc;
}

static void _iris_release_firmware(const struct firmware **fw)
{
	if (*fw) {
		release_firmware(*fw);
		*fw = NULL;
	}
}

static int _iris_change_lut_type_addr(
		struct iris_ip_opt *dest, struct iris_ip_opt *src)
{
	int rc = -EINVAL;
	struct dsi_cmd_desc *desc = NULL;

	if (!src || !dest) {
		IRIS_LOGE("%s(), invalid src: %p or dest: %p", __func__, src, dest);
		return rc;
	}

	desc = src->cmd;
	if (!desc) {
		IRIS_LOGE("%s(), invalid desc.", __func__);
		return rc;
	}

	IRIS_LOGD("%s(), desc len: %zu", __func__, desc->msg.tx_len);
	iris_change_type_addr(dest, src);

	return 0;
}

static int _iris_change_dbc_type_addr(void)
{
	int i = 0;
	int rc = -EINVAL;
	u8 ip = IRIS_IP_DBC;
	u8 opt_id = 0xFE;
	u8 lut_opt_id = CABC_DLV_OFF;
	struct iris_ip_opt *lut_popt = NULL;
	struct iris_ip_opt *popt = NULL;

	/*DBC change*/
	IRIS_LOGD("%s(%d)", __func__, __LINE__);
	popt = iris_find_ip_opt(ip, opt_id);
	if (!popt)
		return rc;

	for (i = 0; i < DBC_HIGH; i++) {
		lut_popt = iris_find_ip_opt(DBC_LUT, lut_opt_id + i);
		if (!lut_popt)
			return rc;

		rc = _iris_change_lut_type_addr(lut_popt, popt);
	}
	return rc;
}

static int _iris_change_gamma_type_addr(void)
{
	int i = 0;
	int rc = -EINVAL;
	u8 ip = IRIS_IP_DPP;
	u8 opt_id = 0xFE;
	//u8 lut_opt_id = 0;
	struct iris_ip_opt *lut_popt = NULL;
	struct iris_ip_opt *popt = NULL;
	uint8_t get_optid = 0;
	uint8_t lutip = 0;
	int32_t type = 0;
	struct iris_ip_index *pip_index = NULL;
	bool bApl = 0;
	uint8_t level = 0x0;
	IRIS_LOGD("%s(%d)", __func__, __LINE__);
	popt = iris_find_ip_opt(ip, opt_id);
	if (!popt) {
		IRIS_LOGE("%s(%d), can't find valid option for ip: %#x, opt: %#x",
			__func__, __LINE__, ip, opt_id);
		return rc;
	}

	gamma_apl_status = 0;
	type = IRIS_LUT_PIP_IDX;
	lutip = GAMMA_LUT - LUT_IP_START;

	pip_index = iris_get_ip_idx(type) + lutip;

	for (i = 0; i < pip_index->opt_cnt; i++) {
		lut_popt = pip_index->opt + i;
		rc = _iris_change_lut_type_addr(lut_popt, popt);
		get_optid = lut_popt->opt_id;
		level = get_optid & 0xf;
		bApl = get_optid & 0x80?1 : 0;
		if (bApl)
			gamma_apl_status |= 0x1 << level;

	}

/*	for (i = 0; i <= GAMMA_LUT_NUMBER; i++) {
		lut_opt_id = 0;
		lut_popt = iris_find_ip_opt(GAMMA_LUT, lut_opt_id + i);
		if (lut_popt) {
			rc = _iris_change_lut_type_addr(lut_popt, popt);
			IRIS_LOGI("%s(%d), find valid lut option, input ip: %#x, opt: %#x.",
				__func__, __LINE__, GAMMA_LUT, lut_opt_id + i);
		} else  {
			IRIS_LOGE("%s(%d), can't find valid lut option input ip: %#x, opt: %#x.",
				 __func__, __LINE__, GAMMA_LUT, lut_opt_id + i);
		}

		lut_opt_id = 0x10;
		lut_popt = iris_find_ip_opt(GAMMA_LUT, lut_opt_id + i);

		if (lut_popt) {
			rc = _iris_change_lut_type_addr(lut_popt, popt);
			IRIS_LOGI("%s(%d), find valid lut option, input ip: %#x, opt: %#x.",
				__func__, __LINE__, GAMMA_LUT, lut_opt_id + i);
		} else {
			IRIS_LOGE("%s(%d), can't find valid lut option input ip: %#x, opt: %#x.",
				__func__, __LINE__, GAMMA_LUT, lut_opt_id + i);
		}

		lut_opt_id = 0x20;
		lut_popt = iris_find_ip_opt(GAMMA_LUT, lut_opt_id + i);
		if (lut_popt) {
			rc = _iris_change_lut_type_addr(lut_popt, popt);
			IRIS_LOGI("%s(%d), find valid lut option, input ip: %#x, opt: %#x.",
				 __func__, __LINE__, GAMMA_LUT, lut_opt_id + i);
		} else {
			IRIS_LOGE("%s(%d), can't find valid lut option input ip: %#x, opt: %#x.",
				 __func__, __LINE__, GAMMA_LUT, lut_opt_id + i);
		}

		lut_opt_id = 0x30;
		lut_popt = iris_find_ip_opt(GAMMA_LUT, lut_opt_id + i);

		if (lut_popt) {
			rc = _iris_change_lut_type_addr(lut_popt, popt);
			IRIS_LOGI("%s(%d), find valid lut option,  input ip: %#x, opt: %#x.",
				 __func__, __LINE__, GAMMA_LUT, lut_opt_id + i);

		} else {
			IRIS_LOGE("%s(%d), can't find valid lut option input ip: %#x, opt: %#x.",
				 __func__, __LINE__, GAMMA_LUT, lut_opt_id + i);

		}
	}
*/
	return rc;
}

static int _iris_change_dither_type_addr(void)
{
	int rc = -EINVAL;
	u8 ip = IRIS_IP_DPP;
	u8 opt_id = 0xF8;
	u8 lut_opt_id = 0;
	struct iris_ip_opt *lut_popt = NULL;
	struct iris_ip_opt *popt = NULL;

	IRIS_LOGD("%s(%d)", __func__, __LINE__);
	popt = iris_find_ip_opt(ip, opt_id);
	if (!popt) {
		IRIS_LOGE("%s(), can't find valid option for ip: %#x, opt: %#x.",
			__func__, ip, opt_id);
		return rc;
	}

	lut_popt = iris_find_ip_opt(DPP_DITHER_LUT, lut_opt_id);
	if (!lut_popt) {
		IRIS_LOGE("%s(), can't find valid lut option for ip: %#x, opt: %#x.",
			__func__, DPP_DITHER_LUT, lut_opt_id);
		return rc;
	}

	rc = _iris_change_lut_type_addr(lut_popt, popt);

	return rc;
}

static int _iris_update_for_dma(void)
{
	int rc = 0;

	/*register level*/
	rc = _iris_change_dbc_type_addr();
	rc = _iris_change_gamma_type_addr();
	rc = _iris_change_dither_type_addr();

	return rc;
}

static void _iris_parse_misc_info(void)
{
	uint8_t ip, opt;
	uint8_t i, j, v;
	char str[41];
	uint32_t *p = NULL;
	uint8_t *pc = NULL;
	uint32_t len = 0;
	uint8_t Change_Id[21];
	uint32_t pcs_ver;
	uint32_t date;
	uint8_t calibration_status;
	uint16_t panel_nits;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(),%d", __func__, __LINE__);
	/* iris6.fw: ip = MISC_INFO_LUT, opt = 0
	 * iris6_ccf1.fw: ip = MISC_INFO_LUT, opt = 1
	 * iris6_ccf2.fw: ip = MISC_INFO_LUT, opt = 2
	 */
	ip = MISC_INFO_LUT;
	opt = 0x1;

	if (iris_find_ip_opt(ip, opt) == NULL) {
		IRIS_LOGE("%s(%d), can not find misc info ip-opt", __func__, __LINE__);
		return;
	}

	p = iris_get_ipopt_payload_data(ip, opt, 2);
	if (!p) {
		IRIS_LOGE("%s(%d), can not get misc info payload", __func__, __LINE__);
		return;
	}
	pc = (uint8_t *)p;

	len = iris_get_ipopt_payload_len(ip, opt, 2);
	if (len != 40) {
		IRIS_LOGE("%s(%d), invalid payload len %d", __func__, __LINE__, len);
		return;
	}

	for (i = 0; i < 8; i++)
		IRIS_LOGD("p[%d] = 0x%08x", i, p[i]);

	memcpy(Change_Id, pc, 21);
	pcs_ver = pc[21]<<24 | pc[22]<<16 | pc[23]<<8 | pc[24];
	date = pc[25]<<24 | pc[26]<<16 | pc[27]<<8 | pc[28];
	calibration_status = pc[29];
	panel_nits = pc[30]<<8 | pc[31];

	pcfg->panel_nits = panel_nits;

	str[0] = (char)Change_Id[0];
	for (i = 1; i < 21; i++) {
		for (j = 0; j < 2; j++) {
			if (j == 0)
				v = Change_Id[i]/16;
			else
				v = Change_Id[i]%16;
			if ((v >= 0) && (v <= 9))
				str[(i-1)*2+j+1] = v + 48;
			else
				str[(i-1)*2+j+1] = v + 87;
		}
	}

	IRIS_LOGI("Change_Id: %s", str);
	IRIS_LOGI("pcs_ver = %08x", pcs_ver);
	IRIS_LOGI("date = %08x", date);
	IRIS_LOGI("calibration_status = %d", calibration_status);
	IRIS_LOGI("panel_nits = %04x", panel_nits);
}

int iris_parse_lut_cmds(void)
{
	int ret = 0;
	struct iris_data data[3] = { {NULL, 0}, {NULL, 0}, {NULL, 0} };
	const struct firmware *fw = NULL;
	const struct firmware *ccf1_fw = NULL;
	const struct firmware *ccf2_fw = NULL;
	const struct firmware *ccf3_fw = NULL;
	struct iris_ip_index *pip_index = NULL;
	u8 firmware_state = 0;
	char ccf1_name[256] = {};
	char ccf2_name[256] = {};
	char ccf3_name[256] = {};
	uint saved_timing;
	struct iris_cfg *pcfg = iris_get_cfg();

	saved_timing = pcfg->cur_timing;
	pcfg->cur_timing = 0;

	fw_calibrate_status = 0;
	pip_index = iris_get_ip_idx(IRIS_LUT_PIP_IDX);
	_iris_init_lut_buf();

	// Load "iris6.fw".
	ret = _iris_request_firmware(&fw, IRIS_FIRMWARE_NAME);
	if (!ret) {
		firmware_state |= (1<<0);
		data[0].buf = fw->data;
		data[0].size = fw->size;
		IRIS_LOGI("%s(%d), request name: %s, size: %u.",
			__func__, __LINE__, IRIS_FIRMWARE_NAME, data[0].size);
	} else {
		IRIS_LOGE("%s(), failed to request %s, do not load other FWs", __func__, IRIS_FIRMWARE_NAME);
		goto load_done;
	}

	// Load "iris6_ccf1b.fw".
	strlcpy(ccf1_name, IRIS_CCF1_CALIBRATED_FIRMWARE_NAME, 256);
	ret = _iris_request_firmware(&ccf1_fw, ccf1_name);
	if (!ret) {
		firmware_state |= (1<<1);
		data[1].buf = ccf1_fw->data;
		data[1].size = ccf1_fw->size;
		IRIS_LOGI("%s(%d), request name: %s, size: %u.",
		 __func__, __LINE__, ccf1_name, data[1].size);
	}  else {
		IRIS_LOGE("%s(), failed to request %s, go to load golden FWs", __func__, ccf1_name);
		goto load_golden;
	}
	// Load "iris6_ccf2b.fw".
	strlcpy(ccf2_name, IRIS_CCF2_CALIBRATED_FIRMWARE_NAME, 256);
	ret = _iris_request_firmware(&ccf2_fw, ccf2_name);
	if (!ret) {
		firmware_state |= (1<<2);
		data[2].buf = ccf2_fw->data;
		data[2].size = ccf2_fw->size;
	} else {
		IRIS_LOGE("%s(), failed to request %s, go to load golden FWs", __func__, ccf2_name);
		goto load_golden;
	}
	// Load "iris6_ccf3b.fw".
	strlcpy(ccf3_name, IRIS_CCF3_CALIBRATED_FIRMWARE_NAME, 256);
	ret = _iris_request_firmware(&ccf3_fw, ccf3_name);
	if (!ret) {
		const uint32_t ccf3_tail_size = 0;
		uint32_t ccf3_data_size = 0;

		if ((ccf3_fw->size - ccf3_tail_size) >= ((CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP)) {
			firmware_state |= (1<<3);
			ccf3_data_size = ccf3_fw->size;
			iris_crst_coef_check(ccf3_fw->data, ccf3_fw->size);
			IRIS_LOGI("%s(%d), request name: %s, size: %u.", __func__, __LINE__, ccf3_name, ccf3_data_size);
		} else {
			IRIS_LOGE("%s(), invalid format for firmware %s, go to load golden FWs", __func__, ccf3_name);
		}
	} else {
		IRIS_LOGE("%s(), failed to request %s, go to load golden FWs", __func__, ccf3_name);
	}

	if (firmware_state == 0xf)
		goto load_done;

load_golden:
	firmware_state = firmware_state & 0x1;
	//load golden ccf1.fw firmware
	strlcpy(ccf1_name, IRIS_CCF1_FIRMWARE_NAME, 256);
	ret = _iris_request_firmware(&ccf1_fw, ccf1_name);

	if (!ret) {
		firmware_state |= (1<<1);
		data[1].buf = ccf1_fw->data;
		data[1].size = ccf1_fw->size;
		IRIS_LOGI("%s(%d), request name: %s, size: %u.",
			__func__, __LINE__, ccf1_name, data[1].size);
	} else {
		IRIS_LOGE("%s(), failed to request %s", __func__, ccf1_name);
	}

	//load goldern ccf2.fw firmware
	strlcpy(ccf2_name, IRIS_CCF2_FIRMWARE_NAME, 256);
	ret = _iris_request_firmware(&ccf2_fw, ccf2_name);
	if (!ret) {
		firmware_state |= (1<<2);
		data[2].buf = ccf2_fw->data;
		data[2].size = ccf2_fw->size;
	} else {
		IRIS_LOGE("%s(), failed to request %s", __func__, ccf2_name);
	}

	// Load "iris6_ccf3.fw".
	strlcpy(ccf3_name, IRIS_CCF3_FIRMWARE_NAME, 256);
	ret = _iris_request_firmware(&ccf3_fw, ccf3_name);
	if (!ret) {
		//Temporary modify for lut load, will replace a determined value after usecase fixed
		const uint32_t ccf3_tail_size = 0;
		uint32_t ccf3_data_size = 0;

		if ((ccf3_fw->size - ccf3_tail_size) >= ((CRSTK_COEF_SIZE + CCT_VALUE_SIZE) * CRSTK_COEF_GROUP)) {
			firmware_state |= (1<<3);
			ccf3_data_size = ccf3_fw->size;
			iris_crst_coef_check(ccf3_fw->data, ccf3_fw->size);
			IRIS_LOGI("%s(%d), request name: %s, size: %u.",
				__func__, __LINE__, ccf3_name, ccf3_data_size);
		} else {
			IRIS_LOGE("%s(), invalid format for firmware %s",
				__func__, ccf3_name);
		}
	} else {
		IRIS_LOGE("%s(), failed to request %s",
			__func__, ccf3_name);
	}

load_done:
	fw_load_status = (firmware_state == 0x0f ? FIRMWARE_LOAD_SUCCESS : FIRMWARE_LOAD_FAIL);
	IRIS_LOGI("%s(), load firmware %s, state %#x", __func__,
			fw_load_status == FIRMWARE_LOAD_SUCCESS?"success":"fail",
			firmware_state);
	if (firmware_state == 0x0f) {
		ret = iris_attach_cmd_to_ipidx(data, ARRAY_SIZE(data),
					 pip_index);
		if (ret) {
			IRIS_LOGE("%s(), failed to parse ccf and iris fw", __func__);
		} else {
			_iris_update_for_dma();
			_iris_parse_misc_info();
		}
	} else
		ret = -1;

	_iris_release_firmware(&fw);
	_iris_release_firmware(&ccf1_fw);
	_iris_release_firmware(&ccf2_fw);
	_iris_release_firmware(&ccf3_fw);

	pcfg->cur_timing = saved_timing;
	return ret;
}

/*add lut cmds to bufs for sending*/
static void _iris_prepare_lut_cmds(struct iris_ip_opt *popt)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc *pdesc = pcfg->iris_cmds.iris_cmds_buf;
	int pos = pcfg->iris_cmds.cmds_index;

	IRIS_LOGD("%s(), %p %p cmd count: %d",
			__func__, &pdesc[pos], popt, popt->cmd_cnt);
	memcpy(&pdesc[pos], popt->cmd, sizeof(*pdesc) * popt->cmd_cnt);
	pos += popt->cmd_cnt;
	pcfg->iris_cmds.cmds_index = pos;
}

static void _iris_fomat_lut_cmds(u8 lut_type, u8 opt_id)
{
	struct iris_ip_opt *popt = NULL;

	popt = iris_find_ip_opt(lut_type, opt_id);
	if (!popt) {
		IRIS_LOGW("%s(%d), invalid opt id: %#x.",
			__func__, __LINE__, opt_id);
		return;
	}

	_iris_prepare_lut_cmds(popt);
}

int iris_send_lut(u8 lut_type, u8 table_index, u32 ab_table)
{
	int i = 0;
	struct iris_cfg *pcfg;
	struct dsi_cmd_desc *pdesc = NULL;
	u8 lut_opt_id = 0xfe;
	int len = 0;

	pcfg = iris_get_cfg();
	pdesc = pcfg->iris_cmds.iris_cmds_buf;

	switch (lut_type) {
	case DBC_LUT:
		if (!((table_index <= DBC_HIGH && table_index >= DBC_INIT)
					|| (table_index <= CABC_DLV_HIGH
						&& table_index >= CABC_DLV_OFF)))
			break;

		/*ab_table will be used at AB_table index here.*/
		if (ab_table > 0)
			ab_table = 1;

		len = 1;
		if (table_index < CABC_DLV_OFF) {
			/*find lut table ip and opt id*/
			lut_opt_id = (table_index & 0x3f) | (ab_table << 7);
			/*even and odd*/
			if (table_index != DBC_INIT)
				len = 2;
		} else
			lut_opt_id = table_index;

		for (i = 0; i < len; i++) {
			/*dbc level as to a table for example:odd 0x1 and even 0x41*/
			lut_opt_id = (i << 6 | lut_opt_id);
			_iris_fomat_lut_cmds(lut_type, lut_opt_id);
		}
		IRIS_LOGI("%s(), call DBC_LUT, index: %#x.", __func__, table_index);
	break;

	case CM_LUT:
		if (table_index >= CM_LUT_NUMBER) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
				__func__, __LINE__, table_index, lut_type);
			break;
		}

		lut_opt_id = table_index & 0xff;
		_iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call CM_LUT, index: %#x.", __func__, table_index);
	break;

	case SDR2HDR_LUT:
		if (table_index > (SDR2HDR_LEVEL5 | (SDR2HDR_INV_UV1<<4))) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
				__func__, __LINE__, table_index, lut_type);
			break;
		}

		lut_opt_id = table_index & 0xff;
		_iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call SDR2HDR_LUT, index: %#x.",
			__func__, table_index);
	break;

	case SCALER1D_LUT:
	case SCALER1D_PP_LUT:
		if ((table_index & 0x0f) >= SCALER1D_LUT_NUMBER) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
				__func__, __LINE__, table_index, lut_type);
			break;
		}

		lut_opt_id = table_index & 0xff;
		_iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call SCALER1D, ip: %#x, index: %d.", __func__,
			 lut_type, table_index);
	break;

	case AMBINET_HDR_GAIN:
		if (!iris_maxcll_buf || !dynamic_lutuvy_send_cmd)
			break;

		memcpy(&pdesc[pcfg->iris_cmds.cmds_index], &dynamic_lutuvy_send_cmd[0],
				sizeof(struct dsi_cmd_desc)*iris_lut_param.hdr_lutuvy_pkt_cnt);

		pcfg->iris_cmds.cmds_index += iris_lut_param.hdr_lutuvy_pkt_cnt;
		IRIS_LOGI("%s(), ambinet hdr gain.", __func__);
	break;

	case AMBINET_SDR2HDR_LUT:
		if (!iris_ambient_buf || !dynamic_lut_send_cmd)
			break;

		memcpy(&pdesc[pcfg->iris_cmds.cmds_index],
			&dynamic_lut_send_cmd[0],
			sizeof(struct dsi_cmd_desc)
				* iris_lut_param.hdr_lut2_pkt_cnt);

		pcfg->iris_cmds.cmds_index +=
				iris_lut_param.hdr_lut2_pkt_cnt;
	break;

	case GAMMA_LUT:
		if (table_index > 0xaf) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
			 __func__, __LINE__, table_index, lut_type);
			break;
		}

		lut_opt_id = table_index & 0xff;
		_iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call GAMMA_LUT, index: %d.",
			 __func__, table_index);
	break;
	case ADJ_LUT:
	case COL_ENH_LUT:
		lut_opt_id = table_index & 0xff;
		_iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call COL_ENH_LUT, index: %d., lut_type: 0x%x",
			__func__,  table_index, lut_type);
	break;

	case DPP_DITHER_LUT:
		if (table_index != 0) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
				__func__, __LINE__, table_index, lut_type);
			break;
		}

		lut_opt_id = table_index & 0xff;
		_iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call DPP_DITHER_LUT, index: %d.",
			__func__, table_index);
	break;

	case DTG_PHASE_LUT:
		if (table_index != 0) {
			IRIS_LOGW("%s(%d), invalid table index %d of type: %#x.",
				__func__, __LINE__, table_index, lut_type);
			break;
		}

		lut_opt_id = table_index & 0xff;
		_iris_fomat_lut_cmds(lut_type, lut_opt_id);
		IRIS_LOGI("%s(), call DTG_PHASE_LUT, index: %d.",
			__func__, table_index);
	break;

	default:
		IRIS_LOGW("%s(), type of %d have no cmd.", __func__, lut_type);
	}

	IRIS_LOGD("%s(), lut type: %#x, lut index: %#x, ab index: %d, cmd count: %d, max count: %d",
			__func__,
			lut_type, table_index, ab_table,
			pcfg->iris_cmds.cmds_index, iris_lut_param.lut_cmd_cnts_max);

	return 0;
}

void iris_update_ambient_lut(enum LUT_TYPE lut_type, u32 lut_pos)
{
	u32 len = 0;
	u32 hdr_payload_size = payload_size;
	u32 hdr_pkt_size = hdr_payload_size + DIRECT_BUS_HEADER_SIZE;
	u32 hdr_block_pkt_cnt =
		(SDR2HDR_LUT_BLOCK_SIZE/2 + hdr_payload_size - 1) / hdr_payload_size;
	u32 iris_lut_buf_index, lut_block_index, lut_block_cnt;
	u32 lut_pkt_cmd_index;
	u32 temp_index, index;
	u32 dbus_addr_start;
	u32 lut_fw_index;
	u32 cmd_payload_len;
	struct ocp_header ocp_hdr;

	if (lut_type != AMBINET_SDR2HDR_LUT) {
		IRIS_LOGE("%s(), invalid lut type %d", __func__, lut_type);
		return;
	}

	memset(&ocp_hdr, 0, sizeof(ocp_hdr));
	ocp_hdr.header = 0x0004000C;
	ocp_hdr.address = SDR2HDR_LUT2_ADDRESS;

	if (lut_pos == 0xFFE00000)
		hdr_block_pkt_cnt = (SDR2HDR_LUT_BLOCK_SIZE + hdr_payload_size - 1)
			/ hdr_payload_size;

	dbus_addr_start = SDR2HDR_LUT2_ADDRESS;
	if (lut_pos != 0xFFE00000)
		dbus_addr_start += lut_pos * SDR2HDR_LUT_BLOCK_SIZE / 2;

	lut_block_cnt = SDR2HDR_LUT2_BLOCK_CNT;

	/* copy lut2 to the firmware format.
	 * lut2 is EVEN+ODD,
	 * lut2_fw is  EVEN ODD EVEN ODD EVEN ODD
	 */
	for (index = 0; index < LUT_LEN; index++) {
		if (lut_pos == 0xFFE00000) {
			lut_fw_index = index/2;
			if (index%2 != 0)  // ODD
				lut_fw_index = LUT_LEN/2;

			lut2_fw[lut_fw_index] = lut_lut2[index];
			lut2_fw[lut_fw_index + LUT_LEN] = lut_lut2[index];
			lut2_fw[lut_fw_index + 2 * LUT_LEN] = lut_lut2[index];
		} else {
			if (index % 2 == 0) {
				lut_fw_index = index/4;
				if (index%4 != 0) // ODD
					lut_fw_index += LUT_LEN/4;

				lut2_fw[lut_fw_index] = lut_lut2[index];
				lut2_fw[lut_fw_index + LUT_LEN/2] = lut_lut2[index];
				lut2_fw[lut_fw_index + LUT_LEN] = lut_lut2[index];
			}
		}
	}

	if (dynamic_lut_send_cmd == NULL) {
		len = sizeof(struct dsi_cmd_desc)
			* hdr_pkt_size * hdr_block_pkt_cnt * SDR2HDR_LUT2_BLOCK_NUMBER;
		dynamic_lut_send_cmd = vzalloc(len);
		iris_lut_param.lut_cmd_cnts_max +=
			hdr_block_pkt_cnt * SDR2HDR_LUT2_BLOCK_NUMBER;
		iris_lut_param.hdr_lut2_pkt_cnt =
			hdr_block_pkt_cnt * SDR2HDR_LUT2_BLOCK_NUMBER;
	}

	if (iris_ambient_buf)
		memset(iris_ambient_buf, 0,
				hdr_pkt_size * iris_lut_param.hdr_lut2_pkt_cnt);

	if (!iris_ambient_buf) {
		len = hdr_pkt_size * iris_lut_param.hdr_lut2_pkt_cnt;
		iris_ambient_buf = vzalloc(len);
	}
	if (!iris_ambient_buf)
		return;

	lut_fw_index = 0;
	/*parse LUT2*/
	for (lut_block_index = 0; lut_block_index < lut_block_cnt;
			lut_block_index++) {
		ocp_hdr.address = dbus_addr_start
			+ lut_block_index * SDR2HDR_LUT_BLOCK_ADDRESS_INC;

		for (lut_pkt_cmd_index = 0; lut_pkt_cmd_index < hdr_block_pkt_cnt;
				lut_pkt_cmd_index++) {
			iris_lut_buf_index =
				lut_block_index * hdr_pkt_size * hdr_block_pkt_cnt
				+ lut_pkt_cmd_index * hdr_pkt_size;
			if (lut_pkt_cmd_index == hdr_block_pkt_cnt-1) {
				cmd_payload_len = SDR2HDR_LUT_BLOCK_SIZE / 2
					- (hdr_block_pkt_cnt-1) * hdr_payload_size;
				if (lut_pos == 0xFFE00000)
					cmd_payload_len += SDR2HDR_LUT_BLOCK_SIZE/2;
			} else {
				cmd_payload_len = hdr_payload_size;
			}

			temp_index = lut_pkt_cmd_index
				+ hdr_block_pkt_cnt * lut_block_index;
			dynamic_lut_send_cmd[temp_index].msg.type = 0x29;
			dynamic_lut_send_cmd[temp_index].msg.tx_len =
				cmd_payload_len + DIRECT_BUS_HEADER_SIZE;
			dynamic_lut_send_cmd[temp_index].post_wait_ms = 0;
			dynamic_lut_send_cmd[temp_index].msg.tx_buf =
				iris_ambient_buf + iris_lut_buf_index;

			memcpy(&iris_ambient_buf[iris_lut_buf_index],
					&ocp_hdr, DIRECT_BUS_HEADER_SIZE);
			iris_lut_buf_index += DIRECT_BUS_HEADER_SIZE;

			memcpy(&iris_ambient_buf[iris_lut_buf_index],
					&lut2_fw[lut_fw_index], cmd_payload_len);

			lut_fw_index += cmd_payload_len / 4;
			ocp_hdr.address += cmd_payload_len;
		}
	}
}

int iris_dbg_init_fw_calibrate_status(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("%s(), for iris_debug failed, error %ld",
					__func__, PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u16("fw_calibrate_status", 0644, pcfg->dbg_root,
	       &fw_calibrate_status);

	return 0;
}
