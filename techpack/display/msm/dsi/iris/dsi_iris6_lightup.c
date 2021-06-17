#include <drm/drm_mipi_dsi.h>
#include <video/mipi_display.h>
#include <dsi_drm.h>
#include <sde_encoder_phys.h>
#include "sde_trace.h"
#include "dsi_iris6_api.h"
#include "dsi_iris6_lightup.h"
#include "dsi_iris6_lightup_ocp.h"
#include "dsi_iris6_lp.h"
#include "dsi_iris6_pq.h"
#include "dsi_iris6_ioctl.h"
#include "dsi_iris6_lut.h"
#include "dsi_iris6_gpio.h"
#include "dsi_iris6_i3c.h"
#include "dsi_iris6_loopback.h"
#include "dsi_iris6_log.h"
#include <linux/kthread.h>

#define IRIS_CHIP_VER_0   0
#define IRIS_CHIP_VER_1   1
#define IRIS_OCP_HEADER_ADDR_LEN  8

#define calc_space_left(x, y) (x - y%x)
#define NON_EMBEDDED_BUF_SIZE (256*1024)  //256k
#define MIPI_RX_DSI_STATUS 0xF1100454
#define RX_DSI_STATUS_VALUE  0x11550005

enum PANEL_TYPE {
	PANEL_LCD_SRGB = 0,
	PANEL_LCD_P3,
	PANEL_OLED,
};

enum {
	DSI_CMD_ONE_LAST_FOR_MULT_IPOPT = 0,
};

enum iris_send_mode {
	DSI_NON_EMBEDDED_MODE = 0,/*NON_EMBEDDED using MA default*/
	DSI_EMBEDDED_NO_MA_MODE,
	DSI_EMBEDDED_MA_MODE,
};

/* use to parse dtsi cmd list and lut */
struct iris_cmd_header {
	uint32_t dsi_type;  /* dsi command type 0x23 0x29*/
	uint32_t last_pkt; /*last in chain*/
	uint32_t wait_us; /*wait time*/
	uint32_t ip_type; /*ip type*/
	uint32_t opt_and_link; /*ip option and lp or hs*/
	uint32_t payload_len; /*payload len*/
};

struct iris_cmd_comp {
	int32_t link_state;
	int32_t cmd_cnt;
	struct dsi_cmd_desc *cmd;
	enum iris_op_type op_type;
	enum iris_send_mode send_mode;
};

static struct iris_cfg g_cfg;
static uint8_t g_cont_splash_type = IRIS_CONT_SPLASH_NONE;

int iris_dbg_init_fstatus(struct dsi_display *display);
static int _iris_dbg_init_cont_splash(struct dsi_display *display);
static void _iris_send_cont_splash_pkt(uint32_t type);
static int _iris_set_pkt_last(struct dsi_cmd_desc *cmd, int32_t cmd_cnt);


void iris_init(struct dsi_display *display, struct dsi_panel *panel)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(%d)", __func__, __LINE__);
	pcfg->display = display;
	pcfg->panel = panel;
	pcfg->iris_i2c_read = iris_ioctl_i2c_read;
	pcfg->iris_i2c_write = iris_ioctl_i2c_write;
	pcfg->iris_i2c_burst_write = iris_ioctl_i2c_burst_write;
	pcfg->dport_is_disable = false;
	iris_lp_init();

	iris_dbg_init_lp(display);
	iris_dbg_init_pq(display);
	_iris_dbg_init_cont_splash(display);
	iris_dbg_init_adb_type(display);
	iris_dbg_init_fw_calibrate_status();
	iris_dbg_init_fstatus(display);
	iris_dbg_init_gpio();

	iris_driver_register();
	iris_i2c_bus_init();
}

struct iris_cfg *iris_get_cfg(void)
{
	return &g_cfg;
}

uint8_t iris_get_cont_splash_type(void)
{
	return g_cont_splash_type;
}

struct iris_ctrl_seq *iris_get_ctrl_seq_addr(
		struct iris_ctrl_seq *base, uint8_t chip_id)
{
	struct iris_ctrl_seq *pseq = NULL;

	switch (chip_id) {
	case IRIS_CHIP_VER_0:
		pseq = base;
		break;
	case IRIS_CHIP_VER_1:
		pseq = base + 1;
		break;
	default:
		IRIS_LOGE("%s(), invalid chip id: %d", __func__, chip_id);
	}
	return pseq;
}

static bool _iris_is_valid_ip(uint32_t ip)
{
	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return true;

	if (ip >= IRIS_IP_START && ip < IRIS_IP_END)
		return true;

	IRIS_LOGE("%s(), invalid ip: %#x", __func__, ip);

	return false;
}

static struct iris_ctrl_seq *_iris_get_ctrl_seq_common(int32_t type)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ctrl_seq *pseq = NULL;

	if (type == IRIS_CONT_SPLASH_NONE)
		pseq = iris_get_ctrl_seq_addr(pcfg->timing[0].ctrl_seq, pcfg->chip_id);
	else if (type == IRIS_CONT_SPLASH_LK)
		pseq = iris_get_ctrl_seq_addr(pcfg->timing[0].ctrl_seq_cs, pcfg->chip_id);

	return pseq;
}

static struct iris_ctrl_seq *_iris_get_ctrl_seq(void)
{
	return _iris_get_ctrl_seq_common(IRIS_CONT_SPLASH_NONE);
}

static bool _iris_is_lut(uint8_t ip)
{
	if (ip >= LUT_IP_START && ip < LUT_IP_END)
		return true;

	return false;
}

static uint32_t _iris_get_ocp_type(const uint8_t *payload)
{
	uint32_t *pval = (uint32_t *)payload;

	return cpu_to_le32(pval[0]);
}

static uint32_t _iris_get_ocp_base_addr(const uint8_t *payload)
{
	uint32_t *pval = (uint32_t *)payload;

	return cpu_to_le32(pval[1]);
}

static void _iris_set_ocp_type(const uint8_t *payload, uint32_t val)
{
	uint32_t *pval = (uint32_t *)payload;

	IRIS_LOGV("%s(), change addr from %#x to %#x.", __func__, pval[0], val);

	pval[0] = val;
}

static void _iris_set_ocp_base_addr(const uint8_t *payload, uint32_t val)
{
	uint32_t *pval  = (uint32_t *)payload;

	IRIS_LOGV("%s(), change addr from %#x to %#x.", __func__, pval[1], val);

	pval[1] = val;
}

static void _iris_set_ocp_first_val(const uint8_t *payload, uint32_t val)
{
	uint32_t *pval  = (uint32_t *)payload;

	pval[2] = val;
}

static bool _iris_is_direct_bus(const uint8_t *payload)
{
	uint8_t val = _iris_get_ocp_type(payload) & 0x0f;

	//the last 4bit will show the ocp type
	if (val == 0x00 || val == 0x0c)
		return true;

	return false;
}

static int _iris_split_mult_pkt(const uint8_t *payload, int payload_size)
{
	uint32_t pkt_size = 0;
	int pkt_cnt = 1;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!_iris_is_direct_bus(payload))
		return pkt_cnt;

	pkt_size = pcfg->split_pkt_size;
	if (payload_size > pkt_size + IRIS_OCP_HEADER_ADDR_LEN)
		pkt_cnt = (payload_size - IRIS_OCP_HEADER_ADDR_LEN + pkt_size - 1)
					/ pkt_size;

	return pkt_cnt;
}

static void _iris_set_cont_splash_type(uint8_t type)
{
	g_cont_splash_type = type;
}

static struct iris_ip_index *_iris_get_specific_ip_idx(int32_t type, int32_t tm_idx)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (unlikely(type < IRIS_DTSI_PIP_IDX || type >= IRIS_PIP_IDX_CNT)) {
		IRIS_LOGE("%s(), invalid type: %d", __func__, type);
		return NULL;
	}

	if (unlikely(tm_idx < 0 || tm_idx > IRIS_MAX_TIMING_MODES)) {
		IRIS_LOGE("%s(), invalid timing index: %d", __func__, tm_idx);
		return NULL;
	}

	if (type == IRIS_LUT_PIP_IDX)
		tm_idx = 0;

	return pcfg->timing[tm_idx].ip_index_arr[type];
}

struct iris_ip_index *iris_get_ip_idx(int32_t type)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return _iris_get_specific_ip_idx(type, pcfg->cur_timing);
}

static int32_t  _iris_get_ip_idx_type(const struct iris_ip_index *pip_index)
{
	int32_t type = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pip_index == pcfg->timing[pcfg->cur_timing].ip_index_arr[IRIS_DTSI_PIP_IDX])
		type = IRIS_DTSI_PIP_IDX;
	else if (pip_index == pcfg->timing[pcfg->cur_timing].ip_index_arr[IRIS_LUT_PIP_IDX])
		type = IRIS_LUT_PIP_IDX;

	return type;
}

static void _iris_init_ip_index(struct iris_ip_index *pip_index)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t cnt = 0;
	int32_t ip_cnt = IRIS_IP_CNT;

	if (_iris_get_ip_idx_type(pip_index) == IRIS_LUT_PIP_IDX)
		ip_cnt = LUT_IP_END - LUT_IP_START;

	for (i = 0; i < ip_cnt; i++) {
		cnt = pip_index[i].opt_cnt;
		for (j = 0; j < cnt; j++) {
			pip_index[i].opt[j].cmd = NULL;
			pip_index[i].opt[j].link_state = 0xff;
		}
	}
}

static int32_t _iris_alloc_pip_buf(struct iris_ip_index *pip_index)
{
	int i = 0;
	int j = 0;
	int opt_cnt = 0;
	int size;
	int ip_cnt = IRIS_IP_CNT;

	if (_iris_get_ip_idx_type(pip_index) == IRIS_LUT_PIP_IDX)
		ip_cnt = LUT_IP_END - LUT_IP_START;

	for (i = 0; i < ip_cnt; i++) {
		opt_cnt = pip_index[i].opt_cnt;
		if (opt_cnt != 0) {
			size = opt_cnt * sizeof(struct iris_ip_opt);
			pip_index[i].opt = kmalloc(size, GFP_KERNEL);
			if (!pip_index[i].opt) {
				/*free already malloc space*/
				for (j = 0; j < i; j++) {
					kfree(pip_index[j].opt);
					pip_index[j].opt = NULL;
				}
				return -ENOMEM;
			}
		}
	}

	return 0;
}

static int32_t _iris_alloc_desc_buf(struct dsi_cmd_desc **cmds, int cmd_cnt)
{
	int cmd_size = 0;

	/*create dsi cmds*/
	if (cmd_cnt == 0)
		return -EINVAL;

	cmd_size = cmd_cnt * sizeof(struct dsi_cmd_desc);
	*cmds = vzalloc(cmd_size);
	if (!(*cmds)) {
		IRIS_LOGE("%s(), can not malloc space for dsi", __func__);
		return -ENOMEM;
	}
	IRIS_LOGI("%s(), alloc %p, count %d", __func__, *cmds, cmd_cnt);

	return 0;
}

static int32_t _iris_alloc_cmd_buf(struct dsi_cmd_desc **cmds,
		struct iris_ip_index *pip_index, int cmd_cnt)
{
	int32_t rc = 0;

	rc = _iris_alloc_desc_buf(cmds, cmd_cnt);
	if (rc)
		return rc;

	rc = _iris_alloc_pip_buf(pip_index);
	if (rc) {
		vfree(*cmds);
		*cmds = NULL;
	}

	return rc;
}

static int32_t _iris_write_ip_opt(struct dsi_cmd_desc *cmd,
		const struct iris_cmd_header *hdr, int32_t pkt_cnt,
		struct iris_ip_index *pip_index)
{
	uint8_t i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	uint8_t opt_cnt = 0;

	if (!hdr || !cmd || !pip_index) {
		IRIS_LOGE("%s(), invalid input parameter.", __func__);
		return -EINVAL;
	}

	ip = hdr->ip_type & 0xff;
	opt_id = hdr->opt_and_link & 0xff;

	if (ip >= LUT_IP_START)
		ip -= LUT_IP_START;

	opt_cnt = pip_index[ip].opt_cnt;

	for (i = 0; i < opt_cnt; i++) {
		if (pip_index[ip].opt[i].cmd == NULL) {
			pip_index[ip].opt[i].cmd = cmd;
			pip_index[ip].opt[i].cmd_cnt = pkt_cnt;
			pip_index[ip].opt[i].opt_id = opt_id;
			break;
		} else if (pip_index[ip].opt[i].opt_id == opt_id) {
			/*find the right opt_id*/
			pip_index[ip].opt[i].cmd_cnt += pkt_cnt;
			break;
		}
	}

	if (i == opt_cnt) {
		IRIS_LOGE("%s(), can't find popt for ip: 0x%02x opt: 0x%02x.",
			__func__, ip, opt_id);
		return -EINVAL;
	}

	/*to set link state*/
	if (pip_index[ip].opt[i].link_state == 0xff
		&& pip_index[ip].opt[i].opt_id == opt_id) {
		uint8_t link_state = 0;

		link_state = (hdr->opt_and_link >> 8) & 0xff;
		pip_index[ip].opt[i].link_state =
			link_state ? DSI_CMD_SET_STATE_LP : DSI_CMD_SET_STATE_HS;
	}

	return 0;
}

static int32_t _iris_trans_section_hdr_to_desc(
		const struct iris_cmd_header *hdr, struct dsi_cmd_desc *cmd)
{
	memset(cmd, 0x00, sizeof(struct dsi_cmd_desc));

	cmd->msg.type = (hdr->dsi_type & 0xff);
	cmd->post_wait_ms = (hdr->wait_us & 0xff);
	cmd->last_command = ((hdr->last_pkt & 0xff) != 0);
	cmd->msg.tx_len = hdr->payload_len;

	IRIS_LOGV("%s(), cmd list, dtype: %0x wait: %0x last: %s len: %zu",
		__func__, cmd->msg.type, cmd->post_wait_ms,
		cmd->last_command?"true":"false", cmd->msg.tx_len);

	return cmd->msg.tx_len;
}

static void _iris_change_last_and_size(struct iris_cmd_header *dest,
		const struct iris_cmd_header *src, int index, const int pkt_cnt)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int pkt_size = pcfg->split_pkt_size;

	memcpy(dest, src, sizeof(*src));
	if (index == pkt_cnt-1) {
		dest->payload_len = src->payload_len * sizeof(uint32_t)
							- (pkt_cnt - 1) * pkt_size;
		return;
	}

	dest->last_pkt = 0;
	dest->payload_len = (pkt_size + IRIS_OCP_HEADER_ADDR_LEN);
}

static int _iris_write_cmd_hdr(struct dsi_cmd_desc *cmd,
		const struct iris_cmd_header *phdr, int pkt_cnt)
{
	int i = 0;
	struct iris_cmd_header dest_hdr;

	for (i = 0; i < pkt_cnt; i++) {
		_iris_change_last_and_size(&dest_hdr, phdr, i, pkt_cnt);
		_iris_trans_section_hdr_to_desc(&dest_hdr, cmd+i);
	}

	return 0;
}

static void _iris_create_cmd_payload(const uint8_t *payload,
		uint8_t *msg_buf, int32_t buf_size)
{
	int32_t i = 0;
	uint32_t *pval = NULL;
	uint32_t cnt = 0;

	pval = (uint32_t *)payload;
	cnt = (buf_size) >> 2;
	for (i = 0; i < cnt; i++)
		*(uint32_t *)(msg_buf + (i << 2)) = cpu_to_le32(pval[i]);
}

static int _iris_write_cmd_payload(struct dsi_cmd_desc *pdesc,
		const char *payload, int pkt_cnt)
{
	int i = 0;
	uint32_t msg_len = 0;
	uint8_t *msg_buf = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t pkt_size = pcfg->split_pkt_size;
	uint32_t ocp_type = _iris_get_ocp_type(payload);
	uint32_t base_addr = _iris_get_ocp_base_addr(payload);

	if (pkt_cnt == 1) {
		msg_len = pdesc->msg.tx_len;
		msg_buf = kmalloc(msg_len, GFP_KERNEL);
		if (!msg_buf) {
			IRIS_LOGE("%s(%d), failed to allocate for tx buf",
				__func__, __LINE__);
			return -ENOMEM;
		}

		_iris_create_cmd_payload(payload, msg_buf, msg_len);
		pdesc->msg.tx_buf = msg_buf;
	} else {
		/*remove header and base address*/
		payload += IRIS_OCP_HEADER_ADDR_LEN;
		for (i = 0; i < pkt_cnt; i++) {
			msg_len = pdesc[i].msg.tx_len;
			msg_buf = kmalloc(msg_len, GFP_KERNEL);
			if (!msg_buf) {
				IRIS_LOGE("%s(%d), failed to allocate for tx buf",
					__func__, __LINE__);
				return -ENOMEM;
			}

			_iris_set_ocp_base_addr(msg_buf, base_addr + i * pkt_size);
			_iris_set_ocp_type(msg_buf, ocp_type);
			_iris_create_cmd_payload(payload,
					msg_buf + IRIS_OCP_HEADER_ADDR_LEN,
					msg_len - IRIS_OCP_HEADER_ADDR_LEN);

			/* add payload */
			payload += (msg_len - IRIS_OCP_HEADER_ADDR_LEN);
			pdesc[i].msg.tx_buf = msg_buf;
		}
	}

	if (IRIS_IF_LOGVV()) {
		int print_len = 0;

		for (i = 0; i < pkt_cnt; i++) {
			print_len = (pdesc[i].msg.tx_len > 16) ? 16 : pdesc[i].msg.tx_len;
			print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4,
					pdesc[i].msg.tx_buf, print_len, false);
		}
	}

	return 0;
}

void iris_change_type_addr(struct iris_ip_opt *dest, struct iris_ip_opt *src)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint32_t pkt_size = pcfg->split_pkt_size;
	const void *buf = src->cmd->msg.tx_buf;
	int pkt_cnt = dest->cmd_cnt;
	uint32_t ocp_type = _iris_get_ocp_type(buf);
	uint32_t base_addr = _iris_get_ocp_base_addr(buf);

	for (i = 0; i < pkt_cnt; i++) {
		buf = dest->cmd[i].msg.tx_buf;
		_iris_set_ocp_base_addr(buf, base_addr + i * pkt_size);
		_iris_set_ocp_type(buf, ocp_type);
		if (IRIS_LOGD_IF(i == 0)) {
			IRIS_LOGD("%s(), ocp type: 0x%08x, base addr: 0x%08x.",
				 __func__, ocp_type, base_addr);
		}
	}
}

static struct iris_ip_opt *_iris_find_specific_ip_opt(uint8_t ip, uint8_t opt_id, int32_t tm_idx)
{
	int32_t i = 0;
	int32_t type = IRIS_DTSI_PIP_IDX;
	struct iris_ip_opt *popt = NULL;
	struct iris_ip_index *pip_index = NULL;

	IRIS_LOGV("%s(), ip: %#x, opt: %#x, timing index: %d", __func__, ip, opt_id, tm_idx);
	if (!_iris_is_valid_ip(ip)) {
		IRIS_LOGE("%s(), ip %d is out of range", __func__, ip);
		return NULL;
	}

	if (unlikely(type < IRIS_DTSI_PIP_IDX || type >= IRIS_PIP_IDX_CNT)) {
		IRIS_LOGE("%s(), invalid type: %d", __func__, type);
		return NULL;
	}

	if (unlikely(tm_idx < 0 || tm_idx > IRIS_MAX_TIMING_MODES)) {
		IRIS_LOGE("%s(), invalid timing index: %d", __func__, tm_idx);
		return NULL;
	}

	if (ip >= LUT_IP_START) {
		type = IRIS_LUT_PIP_IDX;
		ip -= LUT_IP_START;
	}

	pip_index = _iris_get_specific_ip_idx(type, tm_idx) + ip;

	for (i = 0; i < pip_index->opt_cnt; i++) {
		popt = pip_index->opt + i;
		if (popt->opt_id == opt_id)
			return popt;
	}

	return NULL;
}

struct iris_ip_opt *iris_find_ip_opt(uint8_t ip, uint8_t opt_id)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return _iris_find_specific_ip_opt(ip, opt_id, pcfg->cur_timing);
}

struct iris_ip_opt *iris_find_ip_opt_mask(uint8_t ip, uint8_t opt_id, uint8_t opt_mask)
{
	int32_t i = 0;
	int32_t type = IRIS_DTSI_PIP_IDX;
	struct iris_ip_opt *popt = NULL;
	struct iris_ip_index *pip_index = NULL;

	IRIS_LOGV("%s(), ip: %#x, opt: %#x", __func__, ip, opt_id);

	if (!_iris_is_valid_ip(ip)) {
		IRIS_LOGE("%s(), ip %#x is out of range", __func__, ip);
		return NULL;
	}

	if (ip >= LUT_IP_START) {
		type = IRIS_LUT_PIP_IDX;
		ip -= LUT_IP_START;
	}

	pip_index = iris_get_ip_idx(type) + ip;

	for (i = 0; i < pip_index->opt_cnt; i++) {
		popt = pip_index->opt + i;
		IRIS_LOGI("opt_cnt: %d, opt_id: 0x%x", pip_index->opt_cnt, popt->opt_id);
		if ((popt->opt_id & opt_mask) == (opt_id & opt_mask))
			return popt;

	}

	return NULL;
}

static void _iris_print_ipopt(const struct iris_ip_index *pip_index)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t ip_cnt = IRIS_IP_CNT;

	if (_iris_get_ip_idx_type(pip_index) == IRIS_LUT_PIP_IDX)
		ip_cnt = LUT_IP_END - LUT_IP_START;

	for (i = 0; i < ip_cnt; i++) {
		for (j = 0; j < pip_index[i].opt_cnt; j++) {
			struct iris_ip_opt *popt = &(pip_index[i].opt[j]);

			IRIS_LOGI("%s(%d), ip: %02x opt: %02x, cmd: %p, cmd count: %d,"
					" link state: %#x", __func__, __LINE__, i,
					popt->opt_id, popt->cmd, popt->cmd_cnt, popt->link_state);
		}
	}
}

static int32_t _iris_add_cmd_to_ipidx(const struct iris_data *data,
		struct dsi_cmd_desc *cmds, int cmd_pos,
		struct iris_ip_index *pip_index)
{
	int32_t span = 0;
	int32_t pkt_cnt = 0;
	int32_t total_size = 0;
	int32_t payload_size = 0;
	struct dsi_cmd_desc *pdesc = NULL;
	const uint8_t *payload = NULL;
	const struct iris_cmd_header *hdr = NULL;
	const uint8_t *buf_ptr = (uint8_t *)data->buf;
	int32_t data_size = data->size;
	int32_t cmd_index = cmd_pos;

	while (total_size < data_size) {
		hdr = (const struct iris_cmd_header *)buf_ptr;
		pdesc = &cmds[cmd_index];
		payload = buf_ptr + sizeof(struct iris_cmd_header);
		payload_size = hdr->payload_len * sizeof(uint32_t);
		total_size += sizeof(struct iris_cmd_header) + payload_size;

		pkt_cnt = _iris_split_mult_pkt(payload, payload_size);
		if (IRIS_LOGV_IF(pkt_cnt > 1)) {
			IRIS_LOGV("%s(), pkt_cnt is: %d.", __func__, pkt_cnt);
		}

		/*need to first write desc header and then write payload*/
		_iris_write_cmd_hdr(pdesc, hdr, pkt_cnt);
		_iris_write_cmd_payload(pdesc, payload, pkt_cnt);

		/*write cmd link information*/
		_iris_write_ip_opt(pdesc, hdr, pkt_cnt, pip_index);
		buf_ptr += sizeof(struct iris_cmd_header) + payload_size;
		cmd_index += pkt_cnt;
	}
	span = cmd_index - cmd_pos;

	if (IRIS_IF_LOGVV())
		_iris_print_ipopt(pip_index);

	return span;
}

static int32_t _iris_create_ipidx(const struct iris_data *data,
		int32_t data_cnt, struct iris_ip_index *pip_index, int cmd_cnt)
{
	int32_t i = 0;
	int32_t rc = 0;
	int32_t cmd_pos = 0;
	struct dsi_cmd_desc *cmds = NULL;

	/* create dsi cmd list */
	rc = _iris_alloc_cmd_buf(&cmds, pip_index, cmd_cnt);
	if (rc) {
		IRIS_LOGE("%s(), failed to create cmd memory!", __func__);
		return -ENOMEM;
	}

	_iris_init_ip_index(pip_index);
	for (i = 0; i < data_cnt; i++) {
		if (data[i].size == 0) {
			IRIS_LOGW("%s(), data[%d] length is %d.",
				__func__, i, data[i].size);
			continue;
		}
		cmd_pos += _iris_add_cmd_to_ipidx(&data[i], cmds, cmd_pos, pip_index);
	}

	if (cmd_cnt != cmd_pos) {
		IRIS_LOGE("%s(), can't find desc for cmd cnt: %d, cmd_pos: %d.",
			__func__, cmd_cnt, cmd_pos);
		return -EINVAL;
	}

	return 0;
}

static int32_t _iris_accum_section_desc_cnt(const struct iris_cmd_header *hdr,
		const uint8_t *payload, int *pcmd_cnt)
{
	int pkt_cnt = 1;
	int32_t payload_size = 0;

	if (!hdr || !pcmd_cnt || !payload) {
		IRIS_LOGE("%s(%d), invalid input parameter!", __func__, __LINE__);
		return -EINVAL;
	}

	payload_size = hdr->payload_len * sizeof(uint32_t);
	pkt_cnt = _iris_split_mult_pkt(payload, payload_size);

	/* it will split to pkt_cnt dsi cmds
	 * add (pkt_cnt-1) ocp_header(4 bytes) and ocp_type(4 bytes)
	 */
	*pcmd_cnt += pkt_cnt;

	IRIS_LOGV("%s(), dsi cmd cnt: %d", __func__, *pcmd_cnt);

	return 0;
}

static int32_t _iris_accum_section_opt_cnt(const struct iris_cmd_header *hdr,
		struct iris_ip_index *pip_index)
{
	uint8_t last = 0;
	uint8_t ip = 0;

	if (!hdr || !pip_index) {
		IRIS_LOGE("%s(%d), invalid input parameter.", __func__, __LINE__);
		return -EINVAL;
	}

	last = hdr->last_pkt & 0xff;
	ip = hdr->ip_type & 0xff;

	if ((ip >= LUT_IP_END) || ((ip >= IRIS_IP_CNT) && (ip < LUT_IP_START))) {
		IRIS_LOGE("%s(%d), invalid ip parameter.", __func__, __LINE__);
		return -EINVAL;
	}

	if (last == 1) {
		if (ip >= LUT_IP_START) {
			if ((ip >= LUT_IP_START) && (ip < LUT_IP_END)) {
				ip -= LUT_IP_START;
				pip_index[ip].opt_cnt++;
			}
		} else {
			if (ip < IRIS_IP_CNT)
				pip_index[ip].opt_cnt++;
		}
	}

	return 0;
}

static int32_t _iris_poll_each_section(const struct iris_cmd_header *hdr,
		const char *payload, struct iris_ip_index *pip_index, int *pcmd_cnt)
{
	int32_t rc = 0;

	rc = _iris_accum_section_desc_cnt(hdr, payload, pcmd_cnt);
	if (rc) {
		IRIS_LOGE("%s(), fail to accumulate desc count, return: %d",
			__func__, rc);
		return -EINVAL;
	}

	rc = _iris_accum_section_opt_cnt(hdr, pip_index);
	if (rc) {
		IRIS_LOGE("%s(), fail to accumulate option count, return: %d",
			__func__, rc);
		return -EINVAL;
	}

	return 0;
}

static int32_t _iris_verify_dtsi(const struct iris_cmd_header *hdr,
		int32_t type)
{
	uint32_t *pval = NULL;
	uint8_t tmp = 0;
	int32_t rc = 0;

	if (type == IRIS_DTSI_PIP_IDX) {
		if (hdr->ip_type >= IRIS_IP_CNT) {
			IRIS_LOGE("%s(), ip %#x is out of range",
				__func__, hdr->ip_type);
			rc = -EINVAL;
		} else if (((hdr->opt_and_link >> 8) & 0xff)  > 1) {
			IRIS_LOGE("%s(), opt %#x is without valid link state",
				__func__, hdr->opt_and_link);
			rc = -EINVAL;
		}
	} else {
		if (hdr->ip_type >= LUT_IP_END || hdr->ip_type < LUT_IP_START) {
			IRIS_LOGE("%s(), ip %#x is out of range",
				__func__, hdr->ip_type);
			rc = -EINVAL;
		}
	}

	switch (hdr->dsi_type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		break;
	case MIPI_DSI_GENERIC_LONG_WRITE:
		/*judge payload0 for iris header*/
		pval = (uint32_t *)hdr + (sizeof(*hdr) >> 2);
		tmp = *pval & 0x0f;
		if (tmp == 0x00 || tmp == 0x05 || tmp == 0x0c || tmp == 0x08) {
			break;
		} else if (tmp == 0x04) {
			if ((hdr->payload_len - 1) % 2 != 0) {
				IRIS_LOGE("%s(), invalid len: %d",
						__func__, hdr->payload_len);
				rc = -EINVAL;
			}
		} else {
			IRIS_LOGE("%s(), invalid hdr: %#x", __func__, *pval);
			rc = -EINVAL;
		}
		break;
	default:
		IRIS_LOGE("%s(), invalid type: %#x", __func__, hdr->dsi_type);
		rc = -EINVAL;
	}

	if (rc) {
		IRIS_LOGI("%s(), hdr info: %#x %#x %#x %#x %#x %#x", __func__,
			hdr->dsi_type, hdr->last_pkt, hdr->wait_us, hdr->ip_type,
			hdr->opt_and_link, hdr->payload_len);
	}

	return rc;
}

static int32_t _iris_parse_platform_type(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;

	rc = of_property_read_u32(np, "pxlw,platform", &(pcfg->platform_type));
	if (rc) {
		IRIS_LOGE("%s(), failed to pxlw platform, return: %d",
			__func__, rc);
		return rc;
	}

	IRIS_LOGI("%s(), pxlw platform type: %#x", __func__, pcfg->platform_type);

	return rc;
}

static int32_t _iris_parse_panel_type(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	u32 value = 0;
	u8 values[2] = {};
	int32_t rc = 0;
	const char *data = of_get_property(np, "pxlw,panel-type", NULL);

	if (data) {
		if (!strcmp(data, "PANEL_LCD_SRGB"))
			pcfg->panel_type = PANEL_LCD_SRGB;
		else if (!strcmp(data, "PANEL_LCD_P3"))
			pcfg->panel_type = PANEL_LCD_P3;
		else if (!strcmp(data, "PANEL_OLED"))
			pcfg->panel_type = PANEL_OLED;
		else/*default value is 0*/
			pcfg->panel_type = PANEL_LCD_SRGB;
	} else { /*default value is 0*/
		pcfg->panel_type = PANEL_LCD_SRGB;
		IRIS_LOGW("%s(), failed to parse panel type!", __func__);
	}

	rc = of_property_read_u32(np, "pxlw,panel-dimming-brightness", &value);
	if (rc == 0) {
		pcfg->panel_dimming_brightness = value;
	} else {
		/* for V30 panel, 255 may cause panel during exit HDR, and lost TE.*/
		pcfg->panel_dimming_brightness = 250;
		IRIS_LOGW("%s(), failed to parse panel dimming brightness, return: %d",
			__func__, rc);
	}

	rc = of_property_read_u8_array(np, "pxlw,panel-hbm", values, 2);
	if (rc == 0) {
		pcfg->panel_hbm[0] = values[0];
		pcfg->panel_hbm[1] = values[1];
	} else {
		pcfg->panel_hbm[0] = 0xff;
		pcfg->panel_hbm[1] = 0xff;
	}

	return 0;
}

static int32_t _iris_parse_chip_ver(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;

	pcfg->iris_chip_enable = of_property_read_bool(np,
			"pxlw,iris-chip-enable");
	IRIS_LOGI("%s: pxlw,iris-chip-enable %s", __func__,
		(pcfg->iris_chip_enable ? "enable" : "disable"));

	pcfg->iris_soft_enable = of_property_read_bool(np,
			"pxlw,iris-soft-enable");
	IRIS_LOGI("%s: pxlw,iris-soft-enable %s", __func__,
		(pcfg->iris_soft_enable ? "enable" : "disable"));

	pcfg->iris_default_mode_pt = of_property_read_bool(np,
			"pxlw,iris-default-mode-pt");
	IRIS_LOGI("%s: pxlw,iris-default-modt-pt %s", __func__,
		(pcfg->iris_default_mode_pt ? "enable" : "disable"));

	rc = of_property_read_u32(np, "pxlw,chip-ver", &(pcfg->chip_ver));
	if (rc) {
		IRIS_LOGE("%s(), failed to parse chip verion, return: %d",
			__func__, rc);
		return rc;
	}

	IRIS_LOGI("%s(), chip version: %#x", __func__, pcfg->chip_ver);

	return rc;
}

static int32_t _iris_parse_lut_mode(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	const char *data = of_get_property(np, "pxlw,lut-mode", NULL);

	if (data) {
		if (!strcmp(data, "single"))
			pcfg->lut_mode = SINGLE_MODE;
		else if (!strcmp(data, "interpolation"))
			pcfg->lut_mode = INTERPOLATION_MODE;
		else/*default value is 0*/
			pcfg->lut_mode = INTERPOLATION_MODE;
	} else { /*default value is 0*/
		pcfg->lut_mode = INTERPOLATION_MODE;
		IRIS_LOGE("%s(), failed to parse lut mode", __func__);
	}

	IRIS_LOGI("%s(), lut mode: %d", __func__, pcfg->lut_mode);

	return 0;
}

static int32_t _iris_parse_lp_ctrl(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 vals[3] = {0, 0, 0};

	pcfg->lp_ctrl.esd_ctrl = 1;
	rc = of_property_read_u32(np, "pxlw,esd-ctrl", &(pcfg->lp_ctrl.esd_ctrl));
	if (rc) {
		IRIS_LOGE("%s(), failed to pxlw esd-ctrl, return: %d",
				  __func__, rc);
	}
	IRIS_LOGI("%s(), pxlw esd-ctrl : %#x", __func__, pcfg->lp_ctrl.esd_ctrl);

	pcfg->lp_ctrl.qsync_mode = 0;
	rc = of_property_read_u32(np, "pxlw,qsync-mode", &(pcfg->lp_ctrl.qsync_mode));
	if (rc) {
		IRIS_LOGE("%s(), failed to pxlw qsync-mode, return: %d",
				  __func__, rc);
	}
	IRIS_LOGI("%s(), pxlw qsync-mode : %#x", __func__, pcfg->lp_ctrl.qsync_mode);

	rc = of_property_read_u8_array(np, "pxlw,low-power", vals, 3);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse low power, return: %d", __func__, rc);
		return 0;
	}

	pcfg->lp_ctrl.dynamic_power = (bool)vals[0];
	pcfg->lp_ctrl.ulps_lp = (bool)vals[1];
	pcfg->lp_ctrl.abyp_lp = (u8)vals[2];

	IRIS_LOGI("%s(), parse pxlw,low-power [%d %d %d]",
		__func__, vals[0], vals[1], vals[2]);

	return rc;
}

static int32_t _iris_parse_path_sel(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 vals[3] = {0, 0, 0};

	rc = of_property_read_u8_array(np, "pxlw,path-sel", vals, 3);
	if (rc)
		IRIS_LOGE("%s(), failed to parse path sel, return: %d", __func__, rc);


	pcfg->light_up_path = vals[0] ? PATH_I2C : PATH_DSI;
	pcfg->pq_update_path = vals[1] ? PATH_I2C : PATH_DSI;
	pcfg->single_ipopt_path = vals[2] ? PATH_I2C : PATH_DSI;

	IRIS_LOGI("%s(), parse pxlw, path sel [%d %d %d]",
		__func__, vals[0], vals[1], vals[2]);

	return rc;
}

static int32_t _iris_parse_virtual_channel_id(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u8 vals[3] = {0, 0, 0};

	pcfg->vc_ctrl.vc_enable = of_property_read_bool(np,
			"pxlw,virtual-channel-enable");
	IRIS_LOGI("%s: pxlw,virtual-channel-enable %s", __func__,
		(pcfg->vc_ctrl.vc_enable ? "enable" : "disable"));

	if (pcfg->vc_ctrl.vc_enable) {
		rc = of_property_read_u8_array(np, "pxlw,virtual-channel-id", vals, 3);
		if (rc)
			IRIS_LOGE("%s(), failed to parse virtual channel id, return: %d", __func__, rc);
	}

	pcfg->vc_ctrl.to_iris_vc_id = vals[0];
	pcfg->vc_ctrl.to_panel_hs_vc_id  = vals[1];
	pcfg->vc_ctrl.to_panel_lp_vc_id  = vals[2];

	IRIS_LOGI("%s(), parse pxlw,virtual-channel-id [%d %d %d]",
		__func__, vals[0], vals[1], vals[2]);

	return rc;
}

static int32_t _iris_parse_dpp_only(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;

	pcfg->dpp_only_enable = 0;

	rc = of_property_read_u32(np, "pxlw,dpp-only-enable", &(pcfg->dpp_only_enable));
	if (rc) {
		IRIS_LOGE("%s(), failed to pxlw dpp-only-enable, return: %d",
				  __func__, rc);
		return rc;
	}

	IRIS_LOGI("%s(), pxlw dpp-only-enable : %#x", __func__, pcfg->dpp_only_enable);

	return rc;
}

static int32_t _iris_parse_split_pkt_info(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	uint32_t arr_32[2] = {0, 0};
	uint8_t arr_8[2] = {0, 0};

	rc = of_property_read_u32(np, "pxlw,pkt-payload-size",
			&(pcfg->split_pkt_size));
	if (rc) {
		IRIS_LOGE("%s(), failed to parse pkt payload size, return: %d",
			__func__, rc);
		return rc;
	}
	IRIS_LOGI("%s(), pkt payload size: %d", __func__, pcfg->split_pkt_size);

	rc = of_property_read_u32(np, "pxlw,last-per-pkt",
			&(pcfg->add_last_flag));
	if (rc) {
		IRIS_LOGE("%s(), failed to parse last per pkt, return: %d",
			__func__, rc);
		return rc;
	}
	IRIS_LOGI("%s(), parse add last for split pkt: %d",
		__func__, pcfg->add_last_flag);


	rc = of_property_read_u8_array(np, "pxlw,dsi-embedded-enable", arr_8, 2);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse embedded enable, return: %d", __func__, rc);
		return rc;
	}
	pcfg->dsi_embedded_enable[0] = arr_8[0];
	pcfg->dsi_embedded_enable[1] = arr_8[1];
	IRIS_LOGI("%s()%d: lightup using %s mode, ", __func__, __LINE__,
		(pcfg->dsi_embedded_enable[0] ? "embedded" : "non-embedded"));
	IRIS_LOGI("%s()%d: pq update using %s mode, ", __func__, __LINE__,
		(pcfg->dsi_embedded_enable[1] ? "embedded" : "non-embedded"));

	rc = of_property_read_u32_array(np, "pxlw,non-embedded-mode-transfer-length",
					arr_32, 2);
	if (rc) {
		IRIS_LOGE("%s(), failed to non embedded mode transfer length, return: %d",
			__func__, rc);
		return rc;
	}
	pcfg->non_embedded_xfer_len[0] = arr_32[0];
	pcfg->non_embedded_xfer_len[1] = arr_32[1];
	IRIS_LOGI("%s(), parse non embedded mode transfer length: lightup %d, pq update %d",
		__func__, pcfg->non_embedded_xfer_len[0], pcfg->non_embedded_xfer_len[1]);

	return 0;
}

static int32_t _iris_parse_color_temp_range(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;

	rc = of_property_read_u32(np, "pxlw,min-color-temp", &pcfg->min_color_temp);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse min color temp, return: %d",
			__func__, rc);
		return rc;
	}
	IRIS_LOGI("%s(), parse min color temp: %d", __func__, pcfg->min_color_temp);

	rc = of_property_read_u32(np, "pxlw,max-color-temp", &pcfg->max_color_temp);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse max color temp, return: %d",
			__func__, rc);
		return rc;
	}
	IRIS_LOGI("%s(), parse max color temp: %d", __func__, pcfg->max_color_temp);

	rc = of_property_read_u32(np, "pxlw,P3-color-temp", &(pcfg->P3_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,P3_color_temp\n");
		return rc;
	}
	IRIS_LOGI("pxlw,P3_color_temp: %d\n", pcfg->P3_color_temp);

	rc = of_property_read_u32(np, "pxlw,sRGB-color-temp", &(pcfg->sRGB_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,sRGB-color-temp\n");
		return rc;
	}
	IRIS_LOGI("pxlw,sRGB-color-temp: %d\n", pcfg->sRGB_color_temp);

	rc = of_property_read_u32(np, "pxlw,vivid-color-temp", &(pcfg->vivid_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,vivid-color-temp\n");
		return rc;
	}
	IRIS_LOGI("pxlw,vivid-color-temp: %d\n", pcfg->vivid_color_temp);

	rc = of_property_read_u32(np, "pxlw,hdr-color-temp", &(pcfg->hdr_color_temp));
	if (rc) {
		IRIS_LOGE("can not get property: pxlw,hdr-color-temp\n");
		return rc;
	}
	IRIS_LOGI("pxlw,hdr-color-temp: %d\n", pcfg->hdr_color_temp);

	return rc;
}

static int32_t _iris_poll_cmd_lists(const struct iris_data *data, int data_cnt,
		struct iris_ip_index *pip_index, int *pcmd_cnt)
{
	int32_t rc = 0;
	int32_t i = 0;
	int32_t payload_size = 0;
	int32_t data_len = 0;
	const uint8_t *buf_ptr = NULL;
	const struct iris_cmd_header *hdr = NULL;
	int32_t type = _iris_get_ip_idx_type(pip_index);

	if (data == NULL || !pcmd_cnt || pip_index == NULL) {
		IRIS_LOGE("%s(), invalid input!", __func__);
		return -EINVAL;
	}

	for (i = 0; i < data_cnt; i++) {
		if (data[i].size == 0) {
			IRIS_LOGW("%s(), data[%d] length is 0", __func__, i);
			continue;
		}

		buf_ptr = data[i].buf;
		data_len = data[i].size;

		while (data_len >= sizeof(struct iris_cmd_header)) {
			hdr = (const struct iris_cmd_header *)buf_ptr;
			data_len -= sizeof(struct iris_cmd_header);
			payload_size = hdr->payload_len * sizeof(uint32_t);
			if (payload_size > data_len || payload_size == 0) {
				IRIS_LOGE("%s(), length error, ip: 0x%02x opt: 0x%02x,"
						" payload len: %d", __func__,
						hdr->ip_type, hdr->opt_and_link, hdr->payload_len);
				return -EINVAL;
			}

			rc = _iris_verify_dtsi(hdr, type);
			if (rc) {
				IRIS_LOGE("%s(%d), verify dtsi return: %d",
						__func__, __LINE__, rc);
				return rc;
			}

			IRIS_LOGV("%s(), hdr info, type: 0x%02x, last: 0x%02x,"
					" wait: 0x%02x, ip: 0x%02x, opt: 0x%02x, data len: %d.",
					__func__,
					hdr->dsi_type, hdr->last_pkt, hdr->wait_us,
					hdr->ip_type, hdr->opt_and_link, hdr->payload_len);

			//payload
			buf_ptr += sizeof(struct iris_cmd_header);

			rc = _iris_poll_each_section(hdr, buf_ptr, pip_index, pcmd_cnt);
			if (rc) {
				IRIS_LOGE("%s(), failed to poll section: %d, return: %d",
					__func__, hdr->ip_type, rc);
				return rc;
			}

			buf_ptr += payload_size;
			data_len -= payload_size;
		}
		if (data_len != 0) {
			IRIS_LOGE("%s(), trailing garbage, len %d",
				  __func__, data_len);
			return -EINVAL;
		}
	}

	return rc;
}

static int32_t _iris_alloc_dtsi_cmd_buf(const struct device_node *np,
		const uint8_t *key, uint8_t **buf)
{
	int32_t cmd_size = 0;
	int32_t cmd_len = 0;
	const void *ret = of_get_property(np, key, &cmd_len);

	if (!ret) {
		IRIS_LOGE("%s(), can't find '%s'", __func__, key);
		return -EINVAL;
	}

	if (cmd_len % 4 != 0) {
		IRIS_LOGE("%s(), lenght %d is not multpile of 4", __func__, cmd_len);
		return -EINVAL;
	}

	cmd_size = sizeof(char) * cmd_len;
	*buf = vzalloc(cmd_size);
	if (!*buf) {
		IRIS_LOGE("%s(), failed to malloc memory", __func__);
		return  -ENOMEM;
	}

	return cmd_size;
}

static int32_t _iris_write_dtsi_cmd_to_buf(const struct device_node *np,
		const uint8_t *key, uint8_t **buf, int size)
{
	int32_t rc = of_property_read_u32_array(np, key,
			(uint32_t *)(*buf), size >> 2);

	if (rc != 0) {
		IRIS_LOGE("%s(%d), read array is not right", __func__, __LINE__);
		return -EINVAL;
	}

	return rc;
}

static void _iris_free_dtsi_cmd_buf(uint8_t **buf)
{
	if (*buf) {
		vfree(*buf);
		*buf = NULL;
	}
}

static void _iris_save_cmd_count(const struct iris_ip_index *pip_index,
		const int cmd_cnt)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int32_t idx_type = _iris_get_ip_idx_type(pip_index);

	if (idx_type == IRIS_DTSI_PIP_IDX) {
		if (cmd_cnt > pcfg->timing[pcfg->cur_timing].dtsi_cmds_cnt)
			pcfg->timing[pcfg->cur_timing].dtsi_cmds_cnt = cmd_cnt;
		return;
	}

	if (idx_type == IRIS_LUT_PIP_IDX) {
		pcfg->timing[pcfg->cur_timing].lut_cmds_cnt = cmd_cnt;
		return;
	}

	IRIS_LOGI("%s(), doesn't save count for type %#x pip index %p",
			__func__, idx_type, pip_index);
}

int32_t iris_attach_cmd_to_ipidx(const struct iris_data *data,
		int32_t data_cnt, struct iris_ip_index *pip_index)
{
	int32_t rc = 0;
	int32_t cmd_cnt = 0;

	rc = _iris_poll_cmd_lists(data, data_cnt, pip_index, &cmd_cnt);
	if (rc) {
		IRIS_LOGE("%s(), fail to parse dtsi/lut cmd list, data count %d",
			__func__, data_cnt);
		return rc;
	}

	_iris_save_cmd_count(pip_index, cmd_cnt);

	rc = _iris_create_ipidx(data, data_cnt, pip_index, cmd_cnt);

	return rc;
}

static int32_t _iris_parse_dtsi_cmd(const struct device_node *lightup_node,
		uint32_t cmd_index)
{
	int32_t rc = 0;
	int32_t cmd_size = 0;
	int32_t data_cnt = 0;
	uint8_t *dtsi_buf = NULL;
	struct iris_ip_index *pip_index = NULL;
	struct iris_data data[1];
	const uint8_t *key = "pxlw,iris-cmd-list";

	memset(data, 0x00, sizeof(data));

	// need to keep dtsi buf and release after used
	cmd_size = _iris_alloc_dtsi_cmd_buf(lightup_node, key, &dtsi_buf);
	if (cmd_size <= 0) {
		IRIS_LOGE("%s(), can not malloc space for dtsi cmd", __func__);
		return -ENOMEM;
	}

	rc = _iris_write_dtsi_cmd_to_buf(lightup_node, key, &dtsi_buf, cmd_size);
	if (rc) {
		IRIS_LOGE("%s(), cant not write dtsi cmd to buf", __func__);
		goto FREE_DTSI_BUF;
	}

	data[0].buf = dtsi_buf;
	data[0].size = cmd_size;

	pip_index = iris_get_ip_idx(cmd_index);
	data_cnt = ARRAY_SIZE(data);

	rc = iris_attach_cmd_to_ipidx(data, data_cnt, pip_index);

FREE_DTSI_BUF:
	_iris_free_dtsi_cmd_buf(&dtsi_buf);

	return rc;
}

static void _iris_add_cmd_seq(struct iris_ctrl_opt *ctrl_opt,
		int item_cnt, const uint8_t *pdata)
{
	int32_t i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	uint8_t chain = 0;
	const int32_t span = 3;

	for (i = 0; i < item_cnt; i++) {
		ip = pdata[span * i];
		opt_id = pdata[span * i + 1];
		chain = pdata[span * i + 2];

		ctrl_opt[i].ip = ip & 0xff;
		ctrl_opt[i].opt_id = opt_id & 0xff;
		ctrl_opt[i].chain = chain & 0xff;

		if (IRIS_IF_LOGV())
			IRIS_LOGE("%s(), ip: %d opt: %d  chain: %d",
				__func__, ip, opt_id, chain);
	}
}

static int32_t _iris_alloc_cmd_seq(struct iris_ctrl_seq *pctrl_seq,
		int32_t seq_cnt)
{
	pctrl_seq->ctrl_opt =
		kmalloc_array(seq_cnt, sizeof(struct iris_ctrl_seq), GFP_KERNEL);

	if (pctrl_seq->ctrl_opt == NULL) {
		IRIS_LOGE("%s(), failed to alloc space for pctrl opt", __func__);
		return -ENOMEM;
	}
	pctrl_seq->cnt = seq_cnt;

	return 0;
}

static int32_t _iris_parse_cmd_seq_data(const struct device_node *np,
		const uint8_t *key, const uint8_t **pval)
{
	int32_t item_cnt = 0;
	int32_t seq_cnt = 0;
	int32_t span = 3;
	const uint8_t *pdata = of_get_property(np, key, &item_cnt);

	if (!pdata) {
		IRIS_LOGE("%s(), %s is invalid", __func__, key);
		return -EINVAL;
	}

	seq_cnt = item_cnt / span;
	if (item_cnt == 0 || item_cnt != span * seq_cnt) {
		IRIS_LOGE("%s(), invalid item count: %d, for parsing: %s",
				__func__, item_cnt, key);
		return -EINVAL;
	}
	*pval = pdata;

	return seq_cnt;
}


static int32_t _iris_parse_cmd_seq_common(const struct device_node *np,
		const uint8_t *pre_key, const uint8_t *key,
		struct iris_ctrl_seq *pctrl_seq)
{
	int32_t rc = 0;
	int32_t pre_seq_cnt = 0;
	int32_t seq_cnt = 0;
	const uint8_t *pdata = NULL;
	const uint8_t *pre_pdata = NULL;

	pre_seq_cnt = _iris_parse_cmd_seq_data(np, pre_key, &pre_pdata);
	if (pre_seq_cnt <= 0)
		return -EINVAL;

	seq_cnt = _iris_parse_cmd_seq_data(np, key, &pdata);
	if (seq_cnt <= 0)
		return -EINVAL;

	rc = _iris_alloc_cmd_seq(pctrl_seq, pre_seq_cnt + seq_cnt);
	if (rc != 0)
		return rc;

	_iris_add_cmd_seq(pctrl_seq->ctrl_opt, pre_seq_cnt, pre_pdata);
	_iris_add_cmd_seq(&pctrl_seq->ctrl_opt[pre_seq_cnt], seq_cnt, pdata);

	return rc;
}

static int32_t _iris_parse_pre_cmd_seq(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	uint8_t *pre_key = "pxlw,iris-lightup-sequence-pre";
	int32_t pre_seq_cnt = 0;
	const uint8_t *pre_pdata = NULL;
	struct iris_ctrl_seq *pctrl_seq = &pcfg->timing[pcfg->cur_timing].ctrl_seq_pre;

	pre_seq_cnt = _iris_parse_cmd_seq_data(np, pre_key, &pre_pdata);
	if (pre_seq_cnt <= 0)
		return -EINVAL;

	rc = _iris_alloc_cmd_seq(pctrl_seq, pre_seq_cnt);
	if (rc != 0)
		return rc;

	_iris_add_cmd_seq(pctrl_seq->ctrl_opt, pre_seq_cnt, pre_pdata);

	return rc;

}

static int32_t _iris_parse_cmd_seq(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	uint8_t *pre0_key = "pxlw,iris-lightup-sequence-pre0";
	uint8_t *pre1_key = "pxlw,iris-lightup-sequence-pre1";
	uint8_t *key = "pxlw,iris-lightup-sequence";

	rc = _iris_parse_cmd_seq_common(np, pre0_key, key,
					pcfg->timing[pcfg->cur_timing].ctrl_seq);
	if (rc != 0)
		return rc;

	return _iris_parse_cmd_seq_common(np, pre1_key, key,
					  pcfg->timing[pcfg->cur_timing].ctrl_seq + 1);
}

static int32_t _iris_parse_mode_switch_seq(
		const struct device_node *np, struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	int32_t seq_cnt = 0;
	const uint8_t *key = "pxlw,iris-mode-switch-sequence";
	const uint8_t *pdata = NULL;

	seq_cnt = _iris_parse_cmd_seq_data(np, key, &pdata);
	if (seq_cnt <= 0) {
		IRIS_LOGI("%s, [optional] without mode switch sequence, seq count: %d",
				__func__, seq_cnt);

		return 0;
	}

	rc = _iris_alloc_cmd_seq(&pcfg->timing[pcfg->cur_timing].mode_switch_seq, seq_cnt);
	if (rc != 0) {
		IRIS_LOGE("%s, alloc buffer for mode switch sequence failed, return %d",
				__func__, rc);

		return rc;
	}

	_iris_add_cmd_seq(pcfg->timing[pcfg->cur_timing].mode_switch_seq.ctrl_opt, seq_cnt, pdata);

	return rc;
}

static int32_t _iris_count_ip(const uint8_t *data, int32_t len, int32_t *pval)
{
	int i = 0;
	int j = 0;
	int tmp = 0;

	if (data == NULL || len == 0 || pval == NULL) {
		IRIS_LOGE("%s(), invalid data:%p, len: %d, or val: %p",
			__func__, data, len, pval);
		return -EINVAL;
	}

	tmp = data[0];
	len = len >> 1;

	for (i = 0; i < len; i++) {
		if (tmp == data[2 * i]) {
			pval[j]++;
		} else {
			tmp = data[2 * i];
			j++;
			pval[j]++;
		}
	}

	/*j begin from 0*/
	return j + 1;
}

static int32_t _iris_alloc_pq_init_space(struct iris_cfg *pcfg,
		const uint8_t *pdata, int32_t item_cnt)
{
	int32_t rc = 0;
	int32_t i = 0;
	int32_t size = 0;
	int32_t ip_cnt = 0;
	int32_t *ptr = NULL;
	struct iris_pq_init_val *pinit_val = &(pcfg->timing[pcfg->cur_timing].pq_init_val);

	if (pdata == NULL || item_cnt == 0) {
		IRIS_LOGE("%s(), invalid data: %p or item count: %0x",
				__func__, pdata, item_cnt);
		return -EINVAL;
	}

	size = sizeof(*ptr) * (item_cnt >> 1);
	ptr = kzalloc(size, GFP_KERNEL);
	if (ptr == NULL) {
		IRIS_LOGE("%s(), failed to alloc for data", __func__);
		return -EINVAL;
	}

	ip_cnt = _iris_count_ip(pdata, item_cnt, ptr);
	if (ip_cnt <= 0) {
		IRIS_LOGE("%s(), failed to count ip", __func__);
		rc = -EINVAL;
		goto EXIT_FREE;
	}

	pinit_val->ip_cnt = ip_cnt;
	size = sizeof(struct iris_pq_ipopt_val) * ip_cnt;
	pinit_val->val = kmalloc(size, GFP_KERNEL);
	if (pinit_val->val == NULL) {
		IRIS_LOGE("%s(), failed to alloc for pq ipopt val", __func__);
		rc = -EINVAL;
		goto EXIT_FREE;
	}

	for (i = 0; i < ip_cnt; i++) {
		pinit_val->val[i].opt_cnt = ptr[i];
		size = sizeof(uint8_t) * ptr[i];
		pinit_val->val[i].popt = kmalloc(size, GFP_KERNEL);
	}

EXIT_FREE:
	kfree(ptr);
	return rc;

}

static int32_t _iris_parse_default_pq_param(const struct device_node *np,
					    struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	int32_t item_cnt = 0;
	const uint8_t *key = "pxlw,iris-pq-default-val";
	struct iris_pq_init_val *pinit_val = &pcfg->timing[pcfg->cur_timing].pq_init_val;
	const uint8_t *pdata = of_get_property(np, key, &item_cnt);

	if (!pdata) {
		IRIS_LOGE("%s(), failed to parse pq default value", __func__);
		return -EINVAL;
	}

	rc = _iris_alloc_pq_init_space(pcfg, pdata, item_cnt);
	if (rc) {
		IRIS_LOGE("%s(), failed to alloc pq default param buffer, return: %d",
			__func__, rc);
		return rc;
	}

	for (i = 0; i < pinit_val->ip_cnt; i++) {
		struct iris_pq_ipopt_val *pval = &(pinit_val->val[i]);

		pval->ip = pdata[k++];
		for (j = 0; j < pval->opt_cnt; j++) {
			pval->popt[j] = pdata[k];
				k += 2;
		}
		/*need to skip one*/
		k -= 1;
	}

	if (IRIS_IF_LOGV()) {
		IRIS_LOGV("%s(), ip cnt: %d", __func__, pinit_val->ip_cnt);
		for (i = 0; i < pinit_val->ip_cnt; i++) {
			char ptr[256];
			int32_t len = 0;
			int32_t sum = 256;
			struct iris_pq_ipopt_val *pval = &(pinit_val->val[i]);

			snprintf(ptr, sum, "ip is %0x opt is ", pval->ip);
			for (j = 0; j < pval->opt_cnt; j++) {
				len = strlen(ptr);
				sum -= len;
				snprintf(ptr + len, sum, "%0x ", pval->popt[j]);
			}
			IRIS_LOGV("%s(), %s", __func__, ptr);
		}
	}

	return rc;
}

static int32_t _iris_parse_tx_mode(const struct device_node *np,
		struct iris_cfg *pcfg, struct dsi_panel *panel)
{

	pcfg->rx_mode = panel->panel_mode;
	pcfg->tx_mode = panel->panel_mode;

	return 0;
}

static int32_t _iris_parse_extra_info(const struct device_node *np,
		struct iris_cfg *pcfg)
{
	int32_t rc = 0;
	u32 iris_scaling = 0;

	rc = of_property_read_u32(np, "pxlw,output-iris-scaling", &iris_scaling);
	if (!rc)
		IRIS_LOGW("%s(), failed to get 'output-iris-scaling', return: %d",
				__func__, iris_scaling);
	pcfg->output_iris_scaling = iris_scaling != 0;

	return 0;
}

static void __iris_cont_splash_work_handler(struct work_struct *work)
{
	iris_lp_abyp_exit(false);
}

static void __iris_cont_splash_video_path_check(struct iris_cfg *pcfg) {
	pcfg->cont_splash_status = 1;
	if (pcfg->path_backup_need_restore == 1) {
		pcfg->light_up_path = pcfg->light_up_path_backup;
		pcfg->pq_update_path = pcfg->pq_update_path_backup;
		pcfg->single_ipopt_path = pcfg->single_ipopt_path_backup;
		pcfg->dsirecover_check_method = pcfg->dsirecover_check_method_backup;
		pcfg->lp_ctrl.esd_ctrl = pcfg->lp_ctrl.esd_ctrl_backup;
		pcfg->path_backup_need_restore = 0;
	}
}

int iris_parse_param(struct device_node *np, struct dsi_panel *panel)
{
	int32_t rc = 0;
	const struct device_node *lightup_node = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct device_node *timings_np, *child_np;

	IRIS_LOGI("%s(%d), enter.", __func__, __LINE__);

	pcfg->valid = PARAM_EMPTY;
	if (!np) {
		IRIS_LOGE("%s(), invalid device node", __func__);
		return -EINVAL;
	}

	spin_lock_init(&pcfg->iris_1w_lock);
	init_completion(&pcfg->frame_ready_completion);

	timings_np = of_get_child_by_name(np, "qcom,mdss-dsi-display-timings");
	if (!timings_np) {
		IRIS_LOGE("%s(), no timing modes defined", __func__);
		return -EINVAL;
	}

	/* get first timing */
	child_np = of_get_next_child(timings_np, NULL);
	if (!child_np) {
		IRIS_LOGE("%s(), invalid number of timing modes", __func__);
		return -EINVAL;
	}

	lightup_node = of_parse_phandle(child_np, "pxlw,iris-lightup-config", 0);
	if (!lightup_node) {
		IRIS_LOGE("%s(), failed to parse lightup config node", __func__);
		return -EINVAL;
	}

	rc = _iris_parse_split_pkt_info(lightup_node, pcfg);
	if (rc) {
		/*use 64 split packet and do not add last for every packet.*/
		pcfg->split_pkt_size = 64;
		pcfg->add_last_flag = 0;
		pcfg->dsi_embedded_enable[0] = 0;
		pcfg->dsi_embedded_enable[1] = 0;
		pcfg->non_embedded_xfer_len[0] = 0;
		pcfg->non_embedded_xfer_len[1] = 0x400;
	}

	pcfg->dsirecover_check_method = CHECK_WRITE_AND_READ; //CHECK_WRITE_AND_READ;
	pcfg->dsirecover_check_path = PATH_I2C; //PATH_I2C;
	pcfg->pq_update_is_dsi_hs = 0; //1;

	rc = _iris_parse_color_temp_range(lightup_node, pcfg);
	if (rc) {
		/*use 2500K~7500K if do not define in dtsi*/
		pcfg->min_color_temp = 2500;
		pcfg->max_color_temp = 7500;
		pcfg->P3_color_temp = 7350;
		pcfg->sRGB_color_temp = 7350;
		pcfg->vivid_color_temp = 7350;
		pcfg->hdr_color_temp = 7350;

	}

	pcfg->cur_timing = 0;
	pcfg->timing_cnt = 1;
	rc = _iris_parse_dtsi_cmd(lightup_node, IRIS_DTSI_PIP_IDX);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse dtsi cmd, return: %d", __func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_pre_cmd_seq(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse pre cmd seq, return: %d", __func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_cmd_seq(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse cmd seq, return: %d", __func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_mode_switch_seq(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGI("%s, [optional] have not mode switch sequence", __func__);
	}

	rc = _iris_parse_default_pq_param(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse pq default param, return: %d",
			__func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_platform_type(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse platform type, return: %d",
			__func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_panel_type(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse panel type, return: %d", __func__, rc);
		return -EINVAL;
	}

	_iris_parse_lut_mode(lightup_node, pcfg);

	rc = _iris_parse_chip_ver(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse chip version, return: %d",
			__func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_lp_ctrl(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse low power control info, return: %d",
			__func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_path_sel(lightup_node, pcfg);
	if (rc) {
		pcfg->light_up_path = PATH_DSI;
		pcfg->pq_update_path = PATH_DSI;
		pcfg->single_ipopt_path = PATH_DSI;
	}

	pcfg->cont_splash_status = 0;
	pcfg->path_backup_need_restore = 0;
	pcfg->light_up_path_backup = pcfg->light_up_path;
	pcfg->pq_update_path_backup = pcfg->pq_update_path;
	pcfg->single_ipopt_path_backup = pcfg->single_ipopt_path;
	pcfg->dsirecover_check_method_backup = pcfg->dsirecover_check_method;
	pcfg->lp_ctrl.esd_ctrl_backup = pcfg->lp_ctrl.esd_ctrl;

	rc = _iris_parse_virtual_channel_id(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse virtual channel id info, return: %d",
			__func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_tx_mode(lightup_node, pcfg, panel);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse tx mode, return: %d",
				__func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_extra_info(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse tx mode, return: %d",
				__func__, rc);
		return -EINVAL;
	}

	rc = _iris_parse_dpp_only(lightup_node, pcfg);
	if (rc) {
		IRIS_LOGE("%s(), failed to dpp only, return: %d",
				  __func__, rc);
		return -EINVAL;
	}

	/* get additional timings */
	while ((child_np = of_get_next_child(timings_np, child_np)) != NULL) {
		lightup_node = of_parse_phandle(child_np, "pxlw,iris-lightup-config", 0);
		if (!lightup_node) {
			IRIS_LOGE("%s(), can't find lightup config node", __func__);
			return -EINVAL;
		}
		pcfg->cur_timing++;
		pcfg->timing_cnt++;
		if (pcfg->timing_cnt > IRIS_MAX_TIMING_MODES) {
			IRIS_LOGE("%s(), too many timing modes", __func__);
			return -EINVAL;
		}
		rc = _iris_parse_dtsi_cmd(lightup_node, IRIS_DTSI_PIP_IDX);
		if (rc) {
			IRIS_LOGE("%s(), failed to parse dtsi cmd, return: %d", __func__, rc);
			return -EINVAL;
		}

		rc = _iris_parse_pre_cmd_seq(lightup_node, pcfg);
		if (rc) {
			IRIS_LOGE("%s(), failed to parse pre cmd seq, return: %d", __func__, rc);
			return -EINVAL;
		}

		rc = _iris_parse_cmd_seq(lightup_node, pcfg);
		if (rc) {
			IRIS_LOGE("%s(), failed to parse cmd seq, return: %d", __func__, rc);
			return -EINVAL;
		}

		rc = _iris_parse_mode_switch_seq(lightup_node, pcfg);
		if (rc) {
			IRIS_LOGI("%s, [optional] have not mode switch sequence", __func__);
		}

		rc = _iris_parse_default_pq_param(lightup_node, pcfg);
		if (rc) {
			IRIS_LOGE("%s(), failed to parse pq default param, return: %d",
				__func__, rc);
			return -EINVAL;
		}
	}

	pcfg->cur_timing = 0;

	if (pcfg->output_iris_scaling)
		iris_init_cmds();

	INIT_WORK(&pcfg->cont_splash_work, __iris_cont_splash_work_handler);

  if (strnstr(saved_command_line, "androidboot.mode=", strlen(saved_command_line)) != NULL) {
          IRIS_LOGI("%s(%d), saved_command_line: %s", __func__, __LINE__, saved_command_line);
          pcfg->iris_isolate_status = 1;
	pcfg->valid = PARAM_NONE;
  } else {
          pcfg->iris_isolate_status = 0;
	pcfg->valid = PARAM_PARSED;
  }
	IRIS_LOGI("%s(%d), exit.", __func__, __LINE__);

	return 0;
}

struct iris_pq_ipopt_val *iris_get_cur_ipopt_val(uint8_t ip)
{
	int i = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_pq_init_val *pinit_val = &(pcfg->timing[0].pq_init_val);

	for (i = 0; i < pinit_val->ip_cnt; i++) {
		struct iris_pq_ipopt_val *pq_ipopt_val = pinit_val->val + i;

		if (ip == pq_ipopt_val->ip)
			return pq_ipopt_val;
	}

	return NULL;
}

static void _iris_reset_out_cmds(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	memset(pcfg->iris_cmds.iris_cmds_buf, 0x00,
			pcfg->iris_cmds_cnt * sizeof(struct dsi_cmd_desc));
	pcfg->iris_cmds.cmds_index = 0;
}

int32_t _iris_init_cmd_comp(int32_t ip, int32_t opt_index,
		struct iris_cmd_comp *pcmd_comp)
{
	struct iris_ip_opt *popt = NULL;

	if (!_iris_is_valid_ip(ip)) {
		IRIS_LOGE("%s(), invalid ip: %#x", __func__, ip);
		return -EINVAL;
	}

	popt = iris_find_ip_opt(ip, opt_index);
	if (!popt) {
		IRIS_LOGE("%s(), can not find popt, ip: %#x, popt: %#x",
			__func__, ip, opt_index);
		return -EINVAL;
	}

	pcmd_comp->cmd = popt->cmd;
	pcmd_comp->cmd_cnt = popt->cmd_cnt;
	pcmd_comp->link_state = popt->link_state;

	IRIS_LOGV("%s(), cmd count: %d, link state: %#x",
		__func__, pcmd_comp->cmd_cnt, pcmd_comp->link_state);

	return 0;
}

void iris_print_desc_cmds(struct dsi_cmd_desc *cmds, int cmd_cnt, int state)
{
	int i = 0;
	int j = 0;
	int len = 0;
	int msg_len = 0;
	uint8_t *arr = NULL;
	uint8_t *ptr = NULL;
	uint8_t *ptr_tx = NULL;
	struct dsi_cmd_desc *pcmd = NULL;

	IRIS_LOGI("%s(), cmd count: %d, state: %s", __func__, cmd_cnt,
			 (state == DSI_CMD_SET_STATE_HS) ? "high speed" : "low power");

	for (i = 0; i < cmd_cnt; i++) {
		pcmd = cmds + i;
		msg_len = pcmd->msg.tx_len;
		len = 3 * msg_len + 23; //3* 7(dchdr) + 1(\n) + 1 (0)
		arr = vmalloc(len * sizeof(uint8_t));
		if (!arr) {
			IRIS_LOGE("%s(), failed to alloc for tx buffer", __func__);
			return;
		}
		memset(arr, 0x00, sizeof(uint8_t) * len);

		ptr = arr;
		ptr_tx = (uint8_t *) pcmd->msg.tx_buf;
		len = snprintf(ptr, sizeof(arr), "\" %02X", pcmd->msg.type);
		ptr += len;
		for (j = 0; j < msg_len; j++) {
			len = snprintf(ptr, sizeof(arr)-(ptr-arr), " %02X", ptr_tx[j]);
			ptr += len;
		}
		snprintf(ptr, sizeof(arr)-(ptr-arr), "\\n\"");
		IRIS_LOGE("%s", arr);

		if (pcmd->post_wait_ms > 0)
			IRIS_LOGE("\" FF %02X\\n\"", pcmd->post_wait_ms);

		vfree(arr);
		arr = NULL;
	}
}

static void _iris_print_total_cmds(struct dsi_cmd_desc *cmds, int cmd_cnt)
{
	int i = 0;
	int j = 0;
	int value_count = 0;
	int print_count = 0;
	struct dsi_cmd_desc *pcmd = NULL;
	uint32_t *pval = NULL;

	if (IRIS_IF_NOT_LOGD())
		return;

	IRIS_LOGD("%s(), package count in cmd list: %d", __func__, cmd_cnt);
	for (i = 0; i < cmd_cnt; i++) {
		pcmd = cmds + i;
		value_count = pcmd->msg.tx_len/sizeof(uint32_t);
		print_count = value_count;

		pval = (uint32_t *)pcmd->msg.tx_buf;
		IRIS_LOGV("%s(), payload value count: %d, print count: %d, ocp type: 0x%08x, addr: 0x%08x",
			__func__, value_count, print_count, pval[0], pval[1]);

		for (j = 0; j < print_count; j++)
			IRIS_LOGV("0x%08x ", pval[j]);
	}
}

static void _iris_print_spec_cmds(struct dsi_cmd_desc *cmds, int cmd_cnt)
{
	int i = 0;
	int j = 0;
	int value_count = 0;
	int print_count = 0;
	struct dsi_cmd_desc *pcmd = NULL;
	uint32_t *pval = NULL;

	if (IRIS_IF_NOT_LOGD())
		return;

	IRIS_LOGD("%s(), package count in cmd list: %d", __func__, cmd_cnt);
	for (i = 0; i < cmd_cnt; i++) {
		pcmd = cmds + i;
		value_count = pcmd->msg.tx_len/sizeof(uint32_t);
		print_count = value_count;
		if (value_count > 16)
			print_count = 16;

		pval = (uint32_t *)pcmd->msg.tx_buf;
		if (i == 0 || i == cmd_cnt-1) {
			IRIS_LOGD("%s(), package: %d, type: 0x%02x, last: %s,"
					" channel: 0x%02x, flags: 0x%04x, wait: 0x%02x,"
					" send size: %zu", __func__,
					i, pcmd->msg.type, pcmd->last_command?"true":"false",
					pcmd->msg.channel, pcmd->msg.flags, pcmd->post_wait_ms,
					pcmd->msg.tx_len);

			if (IRIS_IF_NOT_LOGV())
				continue;

			IRIS_LOGV("%s(), payload value count: %d, print count: %d,"
					" ocp type: 0x%08x, addr: 0x%08x", __func__,
					value_count, print_count, pval[0], pval[1]);
			for (j = 2; j < print_count; j++)
				IRIS_LOGV("0x%08x", pval[j]);

			if (i == cmd_cnt-1
					&& value_count > 4
					&& print_count != value_count) {
				IRIS_LOGV("%s(), payload tail: 0x%08x, 0x%08x, 0x%08x, 0x%08x.",
					__func__,
					pval[value_count-4], pval[value_count-3],
					pval[value_count-2], pval[value_count-1]);
			}
		}
	}
}

void _iris_print_dtsi_cmds_for_lk(struct dsi_cmd_desc *cmds, int32_t cnt,
		int32_t wait, int32_t link_state)
{
	if (iris_get_cont_splash_type() != IRIS_CONT_SPLASH_LK)
		return;

	//restore the last cmd wait time
	if (wait != 0)
		cmds[cnt-1].post_wait_ms = 1;

	iris_print_desc_cmds(cmds, cnt, link_state);
}

static int32_t _iris_i2c_send_ocp_cmds(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp)
{
	int ret = 0;
	int i = 0;
	struct iris_i2c_msg *msg = NULL;
	uint32_t msg_num = 0;

	msg_num = pcmd_comp->cmd_cnt;
	msg = kmalloc_array(msg_num, sizeof(struct iris_i2c_msg), GFP_KERNEL);
	if (msg == NULL) {
		IRIS_LOGE("%s(), failed to allocate memory", __func__);
		return -EINVAL;
	}

	for (i = 0; i < msg_num; i++) {
		msg[i].buf = (uint8_t *)pcmd_comp->cmd[i].msg.tx_buf;
		msg[i].len = pcmd_comp->cmd[i].msg.tx_len;
	}

	ret = iris_i2c_multi_write(msg, msg_num);

	kfree(msg);

	return ret;
}

static int32_t _iris_dsi_single_addr_send(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp)
{
	int32_t ret;
	uint32_t wait = 0;
	struct dsi_cmd_desc *cmd = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcmd_comp) {
		IRIS_LOGE("%s(), cmd list is null.", __func__);
		return -EINVAL;
	}

	/*fill last_command*/
	_iris_set_pkt_last(pcmd_comp->cmd, pcmd_comp->cmd_cnt);

	/*use us than ms*/
	cmd = pcmd_comp->cmd + pcmd_comp->cmd_cnt - 1;
	wait = cmd->post_wait_ms;
	if (wait)
		cmd->post_wait_ms = 0;

	ret = iris_dsi_send_cmds(panel, pcmd_comp->cmd,
			pcmd_comp->cmd_cnt, pcmd_comp->link_state, pcfg->vc_ctrl.to_iris_vc_id);
	if (wait)
		udelay(wait);

	_iris_print_spec_cmds(pcmd_comp->cmd, pcmd_comp->cmd_cnt);
	_iris_print_dtsi_cmds_for_lk(pcmd_comp->cmd, pcmd_comp->cmd_cnt,
			wait, pcmd_comp->link_state);

	return ret;
}

static void __iris_mult_addr_pad(uint8_t **p, uint32_t *poff, uint32_t left_len)
{
	switch (left_len) {
	case 4:
		_iris_set_ocp_type(*p, 0x00000405);
		*p += 4;
		*poff += 4;
		break;
	case 8:
		_iris_set_ocp_type(*p, 0x00000805);
		_iris_set_ocp_base_addr(*p, 0x00000000);
		*p += 8;
		*poff += 8;
		break;
	case 12:
		_iris_set_ocp_type(*p, 0x00000c05);
		_iris_set_ocp_base_addr(*p, 0x00000000);
		_iris_set_ocp_first_val(*p, 0x00000000);
		*p += 12;
		*poff += 12;
		break;
	case 0:
		break;
	default:
		IRIS_LOGE("%s()%d, left len not aligh to 4.", __func__, __LINE__);
		break;
	}

}

static void __iris_mult_addr_sw_split(uint8_t **p, uint32_t *poff,
	uint32_t left_len, struct dsi_cmd_desc *pdesc)
{
	uint8_t *ptmp = NULL;
	uint32_t tmp_len, wc, ocp_header, base_addr;

	wc = pdesc->msg.tx_len;
	ocp_header = _iris_get_ocp_type(pdesc->msg.tx_buf);
	base_addr = _iris_get_ocp_base_addr(pdesc->msg.tx_buf);

	if (left_len%8 == 0)
		tmp_len = left_len - 4;
	else
		tmp_len = left_len;

	memcpy(*p, pdesc->msg.tx_buf, tmp_len);
	ocp_header &= 0xFF0000FF;
	ocp_header |= (tmp_len << 8) | 0x1;
	_iris_set_ocp_type(*p, ocp_header);
	*p += tmp_len;
	*poff += tmp_len;

	if (left_len%8 == 0)
		__iris_mult_addr_pad(p, poff, 4);

	ptmp = (uint8_t *)pdesc->msg.tx_buf;
	ptmp += tmp_len;
	ocp_header &= 0xFF0000FF;
	ocp_header |= (wc-tmp_len+4) << 8;

	_iris_set_ocp_type(*p, ocp_header);
	*p += 4;
	*poff += 4;
	memcpy(*p, ptmp, wc-tmp_len);
	*p += wc-tmp_len;
	*poff += wc-tmp_len;

}

static void __iris_mult_addr_bw_split(uint8_t **p, uint32_t *poff,
	uint32_t left_len, struct dsi_cmd_desc *pdesc)
{
	uint8_t *ptmp = NULL;
	uint32_t tmp_len, wc, ocp_header, base_addr;

	wc = pdesc->msg.tx_len;
	ocp_header = _iris_get_ocp_type(pdesc->msg.tx_buf);
	base_addr = _iris_get_ocp_base_addr(pdesc->msg.tx_buf);

	tmp_len = left_len;
	memcpy(*p, pdesc->msg.tx_buf, tmp_len);
	ocp_header &= 0xFF0000FF;
	ocp_header |= (tmp_len << 8) | 0x1;
	_iris_set_ocp_type(*p, ocp_header);
	*p += tmp_len;
	*poff += tmp_len;

	ptmp = (uint8_t *)pdesc->msg.tx_buf;
	ptmp += tmp_len;
	ocp_header &= 0xFF0000FF;
	ocp_header |= (wc-tmp_len+8) << 8;

	_iris_set_ocp_type(*p, ocp_header);
	_iris_set_ocp_base_addr(*p, base_addr + (tmp_len-8));
	*p += 8;
	*poff += 8;
	memcpy(*p, ptmp, wc-tmp_len);
	*p += wc-tmp_len;
	*poff += wc-tmp_len;

}

static int32_t _iris_dsi_mult_addr_send(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp)
{
	int32_t ret = 0;
	int32_t link_state, i;
	uint32_t wait, offset, wc, ocp_header, base_addr;
	uint32_t total_payload_len, left_len, trans_len, trans_num = 0;
	uint8_t *ptr = NULL;
	uint32_t split_len = 256;
	const uint32_t pad_len = 12;
	struct dsi_cmd_desc *pdesc = NULL;
	struct mipi_dsi_msg *pmsg = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pcmd_comp) {
		IRIS_LOGE("%s(), cmd list is null.", __func__);
		return -EINVAL;
	}

	if (pcmd_comp->send_mode == DSI_NON_EMBEDDED_MODE) {
		if (pcmd_comp->op_type == IRIS_LIGHTUP_OP)
			trans_len = pcfg->non_embedded_xfer_len[0];
		else
			trans_len = pcfg->non_embedded_xfer_len[1];
		split_len = 256;
	} else {
		trans_len = pcfg->split_pkt_size + 8;
		split_len = trans_len;
	}
	IRIS_LOGD("%s(%d), trans_len is %d, split_len is %d.",
		__func__, __LINE__, trans_len, split_len);

	total_payload_len = 0;
	for (i = 0; i < pcmd_comp->cmd_cnt; i++) {
		pdesc = pcmd_comp->cmd + i;
		total_payload_len += pdesc->msg.tx_len;
	}
	IRIS_LOGD("%s(%d), total_payload_len is %d.",
		__func__, __LINE__, total_payload_len);

	link_state = pcmd_comp->link_state;
	wait = pcmd_comp->cmd[pcmd_comp->cmd_cnt - 1].post_wait_ms;

	memset(pcfg->non_embedded_buf, 0x00, NON_EMBEDDED_BUF_SIZE);
	offset = 0;
	ptr = pcfg->non_embedded_buf;
	for (i = 0; i < pcmd_comp->cmd_cnt; i++) {
		pdesc = pcmd_comp->cmd + i;
		wc = pdesc->msg.tx_len;
		ocp_header = _iris_get_ocp_type(pdesc->msg.tx_buf);
		base_addr = _iris_get_ocp_base_addr(pdesc->msg.tx_buf);

		left_len = calc_space_left(split_len, offset);
		if (left_len <= pad_len)
			__iris_mult_addr_pad(&ptr, &offset, left_len);

		left_len = calc_space_left(split_len, offset);
		if (left_len >= wc) {
			memcpy(ptr, pdesc->msg.tx_buf, wc);
			ocp_header &= 0xFF0000FF;
			ocp_header |= (wc << 8) | 0x1;
			_iris_set_ocp_type(ptr, ocp_header);
			ptr += wc;
			offset += wc;
		} else {
			switch (ocp_header & 0xF) {
			case 0x4:
				__iris_mult_addr_sw_split(&ptr, &offset, left_len, pdesc);
				break;
			case 0x0:
			case 0xc:
				__iris_mult_addr_bw_split(&ptr, &offset, left_len, pdesc);
				break;
			default:
				IRIS_LOGE("%s(),%d, invalid cmd type.", __func__, __LINE__);
			}
		}
	}

	IRIS_LOGD("%s(),%d, offset is %d.", __func__, __LINE__, offset);

	if (trans_len != 0) {
		trans_num = (offset + trans_len - 1)/trans_len;
		WARN_ON(trans_len > 0x1000000);
	} else {
		trans_num = 1;
		WARN_ON(offset > 0x1000000);
	}

	IRIS_LOGD("%s(),%d, trans_len is %d, trans_num is %d.",
		__func__, __LINE__, trans_len, trans_num);

	WARN_ON(!trans_num);

	pdesc = kmalloc_array(trans_num, sizeof(struct dsi_cmd_desc), GFP_KERNEL);
	if (!pdesc) {
		IRIS_LOGE("%s(), failed to alloc desc cmd", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < trans_num; i++) {
		pdesc[i].post_wait_ms = 0;
		pdesc[i].last_command = 1;
		pmsg = &pdesc[i].msg;
		memcpy(pmsg, &pcmd_comp->cmd[0].msg, sizeof(struct mipi_dsi_msg));
		if ((trans_len == 0) && (trans_num == 1)) {
			pmsg->tx_len = offset;
			pmsg->tx_buf = pcfg->non_embedded_buf;
		} else {
			if ((i == trans_num-1) && (offset%trans_len))
				pmsg->tx_len = offset%trans_len;
			else
				pmsg->tx_len = trans_len;
			pmsg->tx_buf = pcfg->non_embedded_buf + i*trans_len;
		}
	}

	/*fill last_command*/
	if (pcmd_comp->send_mode == DSI_EMBEDDED_MA_MODE)
		_iris_set_pkt_last(pdesc, trans_num);

	ret = iris_dsi_send_cmds(panel, pdesc,
			trans_num, link_state, pcfg->vc_ctrl.to_iris_vc_id);
	if (wait)
		udelay(wait);

	if (ret)
		IRIS_LOGE("%s(), [%d]transfer failed, ret = %d.", __func__, i, ret);

	_iris_print_total_cmds(pdesc, trans_num);
	_iris_print_total_cmds(pcmd_comp->cmd, pcmd_comp->cmd_cnt);

	kfree(pdesc);
	return ret;
}

static int32_t _iris_dsi_send_ocp_cmds(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp)
{
	int32_t ret = 0;
	uint8_t embedded_mode_en = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcmd_comp->op_type == IRIS_LIGHTUP_OP)
		embedded_mode_en = pcfg->dsi_embedded_enable[0];
	else
		embedded_mode_en = pcfg->dsi_embedded_enable[1];

	IRIS_LOGD("%s, op_type: %s, embedded_en: %d", __func__,
		pcmd_comp->op_type ? "pq update" : "light up",
		embedded_mode_en);

	pcmd_comp->send_mode = embedded_mode_en;

	switch (embedded_mode_en) {
	case DSI_EMBEDDED_NO_MA_MODE:
		ret = _iris_dsi_single_addr_send(panel, pcmd_comp);
		break;
	case DSI_EMBEDDED_MA_MODE:
	case DSI_NON_EMBEDDED_MODE:
		ret = _iris_dsi_mult_addr_send(panel, pcmd_comp);
		break;
	default:
		IRIS_LOGW("%s(%d), invalid send mode [%d], using single addr mode",
			__func__, __LINE__, embedded_mode_en);
		ret = _iris_dsi_single_addr_send(panel, pcmd_comp);
		break;
	}

	return ret;

}

static int32_t _iris_send_cmds(struct dsi_panel *panel,
		struct iris_cmd_comp *pcmd_comp, uint8_t path)
{
	int32_t ret = 0;

	IRIS_LOGD("%s(%d), path: %d", __func__, __LINE__, path);

	if (!pcmd_comp) {
		IRIS_LOGE("%s(), cmd list is null", __func__);
		return -EINVAL;
	}

	if (pcmd_comp->link_state == DSI_CMD_SET_STATE_LP)
		path = PATH_I2C;

	if (path == PATH_DSI) {
		ret = _iris_dsi_send_ocp_cmds(panel, pcmd_comp);
		return ret;
	}

	if (path == PATH_I2C) {
		ret = _iris_i2c_send_ocp_cmds(panel, pcmd_comp);
		return ret;
	}

	return -EINVAL;
}

int32_t iris_send_ipopt_cmds(int32_t ip, int32_t opt_id)
{
	int32_t rc = 0;
	int32_t i = 0;
	struct iris_cmd_comp cmd_comp;
	struct dsi_cmd_desc *cmd;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGD("%s(), ip: %#x, opt: %#x.", __func__, ip, opt_id);

	rc = _iris_init_cmd_comp(ip, opt_id, &cmd_comp);
	if (rc) {
		IRIS_LOGE("%s(), can not find cmd in seq, for ip: 0x%02x opt: 0x%02x.",
				__func__, ip, opt_id);
		return rc;
	}

	if (IRIS_IF_LOGVV()) {
		for (i = 0; i < cmd_comp.cmd_cnt; i++) {
			cmd = cmd_comp.cmd + i;
			IRIS_LOGD("%s(), ip: %0x opt: %0x last: %d len: %zu",
				__func__, ip, opt_id, cmd->last_command, cmd->msg.tx_len);
		}
	}
	cmd_comp.op_type = IRIS_PQUPDATE_OP;
	rc = _iris_send_cmds(pcfg->panel, &cmd_comp, pcfg->single_ipopt_path);
	if (pcfg->single_ipopt_path == PATH_DSI)
		iris_dsirecover_check(IRIS_PQUPDATE_OP);

	return rc;
}

/* the API will only be called when suspend/resume and boot up */
static void _iris_send_spec_lut(uint8_t lut_table, uint8_t lut_idx)
{
	if (lut_table == AMBINET_HDR_GAIN || lut_table == AMBINET_SDR2HDR_LUT)
		return;

	if (lut_table == DBC_LUT && lut_idx < CABC_DLV_OFF)
		iris_send_lut(lut_table, lut_idx, 1);

	iris_send_lut(lut_table, lut_idx, 0);
}

static void _iris_send_new_lut(uint8_t lut_table, uint8_t lut_idx)
{
	uint8_t dbc_lut_index = 0;

	if (lut_table == DBC_LUT)
		dbc_lut_index = iris_get_dbc_lut_index();

	iris_send_lut(lut_table, lut_idx, dbc_lut_index);
}

static void _iris_update_cmds(struct iris_cmd_comp *pcmd_comp,
		int32_t link_state, enum iris_op_type op)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	_iris_reset_out_cmds();

	memset(pcmd_comp, 0x00, sizeof(*pcmd_comp));
	pcmd_comp->cmd = pcfg->iris_cmds.iris_cmds_buf;
	pcmd_comp->link_state = link_state;
	pcmd_comp->cmd_cnt = pcfg->iris_cmds.cmds_index;
	pcmd_comp->op_type = op;
}

static void _iris_remove_desc_last(struct dsi_cmd_desc *pcmd, int cmd_cnt)
{
	int i = 0;

	for (i = 0; i < cmd_cnt; i++)
		pcmd[i].last_command = false;
}

static void _iris_add_desc_last(struct dsi_cmd_desc *pcmd, int cmd_cnt)
{
	int i = 0;

	for (i = 0; i < cmd_cnt; i++)
		pcmd[i].last_command = true;
}

static void _iris_add_mult_pkt_last(struct dsi_cmd_desc *cmd,
		int cmd_cnt)
{
	int i = 0;
	int num = 0;
	int tail = 0;

	struct iris_cfg *pcfg = iris_get_cfg();
	int span = pcfg->add_last_flag;

	num =  cmd_cnt / span;
	tail = cmd_cnt % span;

	for (i = 0; i < num; i++) {
		_iris_remove_desc_last(cmd + i*span, span - 1);
		_iris_add_desc_last(cmd + i*span + span - 1, 1);
	}

	if (tail) {
		_iris_remove_desc_last(cmd + i*span, tail - 1);
		_iris_add_desc_last(cmd + i*span + tail - 1, 1);
	}

}

static int _iris_set_pkt_last(struct dsi_cmd_desc *cmd, int32_t cmd_cnt)
{
	int32_t ret = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	int32_t add_last_flag = pcfg->add_last_flag;

	if (add_last_flag == DSI_CMD_ONE_LAST_FOR_MULT_IPOPT) {
		_iris_remove_desc_last(cmd, cmd_cnt);
		_iris_add_desc_last(cmd + cmd_cnt - 1, 1);
	} else {
		_iris_add_mult_pkt_last(cmd, cmd_cnt);
	}

	return ret;
}

static int _iris_send_lut_pkt(struct iris_ctrl_opt *popt,
		struct iris_cmd_comp *pcomp, bool is_update, uint8_t path)
{
	int32_t cur = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint8_t opt_id = popt->opt_id;
	uint8_t ip = popt->ip;
	int32_t chain = popt->chain;
	int32_t prev = pcomp->cmd_cnt;

	IRIS_LOGD("%s(), ip: %#x opt: %#x, chain: %d, update: %s",
		 __func__, ip, opt_id, chain, is_update?"true":"false");

	pcfg->iris_cmds.cmds_index = prev;
	if (is_update)
		_iris_send_new_lut(ip, opt_id);
	else
		_iris_send_spec_lut(ip, opt_id);

	cur = pcfg->iris_cmds.cmds_index;

	if (cur == prev) {
		IRIS_LOGD("%s(), invalid lut table for ip: 0x%02x opt: 0x%02x",
				__func__, popt->ip, opt_id);
		return 0;
	}

	pcomp->cmd_cnt = cur;
	if (!chain) {
		_iris_send_cmds(pcfg->panel, pcomp, path);
		_iris_update_cmds(pcomp, pcomp->link_state, pcomp->op_type);
	}

	return 0;
}

static int _iris_send_dtsi_pkt(struct iris_ctrl_opt *pip_opt,
		struct iris_cmd_comp *pcomp, uint8_t path)
{
	int32_t prev = 0;
	int32_t cur = 0;
	int32_t rc = 0;
	struct iris_cmd_comp comp_priv;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint8_t ip = pip_opt->ip;
	uint8_t opt_id = pip_opt->opt_id;
	int32_t chain = pip_opt->chain;

	IRIS_LOGD("%s(), ip: %#x opt: %#x, chain: %d.",
		 __func__, ip, opt_id, chain);

	/*get single/multiple selection(s) according to option of ip*/
	rc = _iris_init_cmd_comp(ip, opt_id, &comp_priv);
	if (rc) {
		IRIS_LOGE("%s(), invalid ip: %#x opt: %#x.", __func__, ip, opt_id);
		return -EINVAL;
	}

	/*save ipopt link state*/
	if ((pcomp->cmd_cnt == 0) && (pcomp->op_type == IRIS_LIGHTUP_OP))
		pcomp->link_state = comp_priv.link_state;

	prev = pcomp->cmd_cnt;
	/*move single/multiples selection to one command*/

	memcpy(pcomp->cmd + pcomp->cmd_cnt, comp_priv.cmd,
		comp_priv.cmd_cnt * sizeof(*comp_priv.cmd));
	pcomp->cmd_cnt += comp_priv.cmd_cnt;

	cur = pcomp->cmd_cnt;
	// if need to send or the last packet of sequence,
	// it should send out to the MIPI
	if (!chain) {
		_iris_send_cmds(pcfg->panel, pcomp, path);
		_iris_update_cmds(pcomp, pcomp->link_state, pcomp->op_type);
	}

	return 0;
}

static void _iris_send_assembled_pkt(struct iris_ctrl_opt *arr, int seq_cnt)
{
	int i = 0;
	uint8_t ip = 0;
	uint8_t opt_id = 0;
	int32_t rc = -1;
	struct iris_cmd_comp cmd_comp;
	struct iris_cfg *pcfg = iris_get_cfg();

	_iris_update_cmds(&cmd_comp, DSI_CMD_SET_STATE_HS, IRIS_LIGHTUP_OP);

	for (i = 0; i < seq_cnt; i++) {
		ip = arr[i].ip;
		opt_id = arr[i].opt_id;
		IRIS_LOGV("%s(), ip: %#x opt: %#x", __func__, ip, opt_id);

		/*lut table*/
		if (_iris_is_lut(ip))
			rc = _iris_send_lut_pkt(arr + i, &cmd_comp, false, pcfg->light_up_path);
		else
			rc = _iris_send_dtsi_pkt(arr + i, &cmd_comp, pcfg->light_up_path);

		if (rc)
			IRIS_LOGE("%s(), FATAL ERROR, return: %d", __func__, rc);
	}
}

void iris_send_mode_switch_pkt(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ctrl_seq *pseq = &pcfg->timing[pcfg->cur_timing].mode_switch_seq;
	ktime_t ktime = 0;

	if (IRIS_IF_LOGI())
		ktime = ktime_get();

	SDE_ATRACE_BEGIN("iris_send_mode_switch_pkt");
	_iris_send_assembled_pkt(pseq->ctrl_opt, pseq->cnt);
	SDE_ATRACE_END("iris_send_mode_switch_pkt");

	pcfg->prev_timing = pcfg->cur_timing;
	if (IRIS_IF_LOGI())
		IRIS_LOGI("%s() takes %d us.", __func__,
				(u32)ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime));
}

static void _iris_send_pre_lightup_pkt(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ctrl_seq *pseq = &pcfg->timing[pcfg->cur_timing].ctrl_seq_pre;

	_iris_send_assembled_pkt(pseq->ctrl_opt, pseq->cnt);
}

static void _iris_send_lightup_pkt(void)
{
	struct iris_ctrl_seq *pseq = _iris_get_ctrl_seq();

	_iris_send_assembled_pkt(pseq->ctrl_opt, pseq->cnt);
}

void iris_init_update_ipopt(struct iris_update_ipopt *popt,
		uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain)
{
	popt->ip = ip;
	popt->opt_old = opt_old;
	popt->opt_new = opt_new;
	popt->chain = chain;
}

int iris_init_update_ipopt_t(uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain)
{
	uint32_t index = 0;
	uint32_t max_cnt = 0;
	struct iris_update_ipopt *popt = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	index = pcfg->timing[pcfg->cur_timing].pq_update_cmd.array_index;
	max_cnt = pcfg->timing[pcfg->cur_timing].ip_opt_cnt;

	if (index >= max_cnt) {
		IRIS_LOGE("%s(), there no empty space for install ip:%#x opt:%#x",
			__func__, ip, opt_new);
		return -EINVAL;
	}

	popt = pcfg->timing[pcfg->cur_timing].pq_update_cmd.update_ipopt_array + index;
	pcfg->timing[pcfg->cur_timing].pq_update_cmd.array_index++;

	iris_init_update_ipopt(popt, ip, opt_old, opt_new, chain);

	return (index + 1);
}

static int _iris_read_chip_id(void)
{
	uint32_t sys_pll_ro_status = 0xf0000010;
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->chip_id = 0;
	// if chip version is set by sw, skip hw read chip id.
	if (pcfg->chip_ver == IRIS3_CHIP_VERSION)
		pcfg->chip_id = iris_ocp_read(sys_pll_ro_status, DSI_CMD_SET_STATE_HS)
						& 0xFF;

	IRIS_LOGI("%s(), chip ver: %#x, chip id: %#x",
		__func__, pcfg->chip_ver, pcfg->chip_id);

	return pcfg->chip_id;
}

void iris_free_ipopt_buf(uint32_t ip_type)
{
	int ip_index = 0;
	int opt_index = 0;
	uint32_t desc_index = 0;
	int ip_cnt = IRIS_IP_CNT;
	struct dsi_cmd_desc *pdesc_addr = NULL;
	struct iris_ip_index *pip_index = iris_get_ip_idx(ip_type);

	if (ip_type == IRIS_LUT_PIP_IDX)
		ip_cnt = LUT_IP_END - LUT_IP_START;

	for (ip_index = 0; ip_index < ip_cnt; ip_index++) {
		if (pip_index[ip_index].opt_cnt == 0 || pip_index[ip_index].opt == NULL)
			continue;

		for (opt_index = 0; opt_index < pip_index[ip_index].opt_cnt;
				opt_index++) {
			if (pip_index[ip_index].opt[opt_index].cmd_cnt == 0
				|| pip_index[ip_index].opt[opt_index].cmd == NULL)
				continue;

			/* get desc cmd start address */
			if (pdesc_addr == NULL
					|| pip_index[ip_index].opt[opt_index].cmd < pdesc_addr) {
				pdesc_addr = pip_index[ip_index].opt[opt_index].cmd;
			}

			for (desc_index = 0;
					desc_index < pip_index[ip_index].opt[opt_index].cmd_cnt;
					desc_index++) {
				if (pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf == NULL
					|| pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_len == 0)
					continue;

				/* free cmd payload, which alloc in "_iris_write_cmd_payload()" */
				kfree(pip_index[ip_index]
					      .opt[opt_index]
					      .cmd[desc_index]
					      .msg.tx_buf);
				pip_index[ip_index].opt[opt_index].cmd[desc_index].msg.tx_buf = NULL;
			}

			/* set each desc cmd to NULL first */
			pip_index[ip_index].opt[opt_index].cmd = NULL;
		}

		/* free opt buffer for each ip, which alloc in "_iris_alloc_pip_buf()" */
		kfree(pip_index[ip_index].opt);
		pip_index[ip_index].opt = NULL;
		pip_index[ip_index].opt_cnt = 0;
	}

	/* free desc cmd buffer, which alloc in "_iris_alloc_desc_buf()", desc
	 * cmd buffer is continus memory, so only free once on start address
	 */
	if (pdesc_addr != NULL) {
		IRIS_LOGI("%s(), free desc cmd buffer %p, type %#x",
			__func__, pdesc_addr, ip_type);
		vfree(pdesc_addr);
		pdesc_addr = NULL;
	}
}

void iris_free_seq_space(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	/* free cmd to sent buffer, which alloc in "iris_alloc_seq_space()" */
	if (pcfg->iris_cmds.iris_cmds_buf != NULL) {
		IRIS_LOGI("%s()%d, free %p", __func__, __LINE__, pcfg->iris_cmds.iris_cmds_buf);
		vfree(pcfg->iris_cmds.iris_cmds_buf);
		pcfg->iris_cmds.iris_cmds_buf = NULL;
	}

	if (pcfg->non_embedded_buf != NULL) {
		IRIS_LOGI("%s()%d, free %p", __func__, __LINE__, pcfg->non_embedded_buf);
		vfree(pcfg->non_embedded_buf);
		pcfg->non_embedded_buf = NULL;
	}
}

void iris_alloc_seq_space(void)
{
	uint8_t *buf = NULL;
	struct dsi_cmd_desc *pdesc = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	int sum = 0;
	uint i;

	for (i = 0; i < pcfg->timing_cnt; i++) {
		if (pcfg->timing[i].dtsi_cmds_cnt > sum)
			sum = pcfg->timing[i].dtsi_cmds_cnt;
	}
	IRIS_LOGI("%s(), dtsi cmd count: %u, lut cmd count: %u",
		__func__, sum, pcfg->timing[0].lut_cmds_cnt);
	sum += pcfg->timing[0].lut_cmds_cnt;

	pdesc = vmalloc(sum * sizeof(struct dsi_cmd_desc));
	if (!pdesc) {
		IRIS_LOGE("%s(), failed to alloc desc cmd", __func__);
		return;
	}

	pcfg->iris_cmds_cnt = sum;
	pcfg->iris_cmds.iris_cmds_buf = pdesc;

	IRIS_LOGI("%s()%d, alloc %p", __func__, __LINE__, pcfg->iris_cmds.iris_cmds_buf);

	_iris_reset_out_cmds();

	buf = vmalloc(NON_EMBEDDED_BUF_SIZE);
	if (!buf) {
		IRIS_LOGE("%s(), failed to alloc non embedded buf", __func__);
		return;
	}
	pcfg->non_embedded_buf = buf;

	IRIS_LOGI("%s()%d, alloc %p", __func__, __LINE__, pcfg->non_embedded_buf);

	// Need to init PQ parameters here for video panel.
	iris_pq_parameter_init();
}

void iris_alloc_update_ipopt_space(void)
{
	struct iris_update_ipopt *p = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	int sum = 0;
	int i = 0, j = 0;
	struct iris_ip_index *pip_index = NULL;

	for (i = IRIS_DTSI_PIP_IDX; i < IRIS_PIP_IDX_CNT; i++) {
		pip_index = iris_get_ip_idx(i);
		for (j = 0; j < IRIS_IP_CNT; j++)
			sum += pip_index[i].opt_cnt;
	}

	pcfg->timing[pcfg->cur_timing].ip_opt_cnt = sum;
	IRIS_LOGI("%s(), ip opt count: %u", __func__, pcfg->timing[pcfg->cur_timing].ip_opt_cnt);

	p = kcalloc(sum, sizeof(struct iris_update_ipopt), GFP_KERNEL);
	if (!p) {
		IRIS_LOGE("%s(), failed to allo update ipopt space", __func__);
		return;
	}

	pcfg->timing[pcfg->cur_timing].pq_update_cmd.update_ipopt_array = p;

	IRIS_LOGI("%s(), alloc %p", __func__, pcfg->timing[pcfg->cur_timing].pq_update_cmd.update_ipopt_array);

	iris_init_ipopt_t();

}

static void _iris_i3c_filter_cfg(void)
{

	if (iris_platform_get() == 0) {
		iris_ocp_bit_en_write(SYS_I3C_CTRL_ADDR, 0x1, 0x0,
				DSI_CMD_SET_STATE_LP);
	}

}

static void _iris_pre_lightup(struct dsi_panel *panel)
{
	_iris_i3c_filter_cfg();
	_iris_send_pre_lightup_pkt();
	_iris_read_chip_id();
	iris_pq_parameter_init();
}

void iris_read_power_mode(struct dsi_panel *panel)
{
	char get_power_mode[1] = {0x0a};
	char read_cmd_rbuf[16] = {0};
	struct dsi_cmd_desc cmds = {
			{0, MIPI_DSI_DCS_READ, MIPI_DSI_MSG_REQ_ACK, 0, 0,
			 sizeof(get_power_mode), get_power_mode, 1, read_cmd_rbuf},
			1, 0};
	struct dsi_panel_cmd_set cmdset = {
		.state = DSI_CMD_SET_STATE_HS,
		.count = 1,
		.cmds = &cmds,
	};
	struct iris_cfg *pcfg = iris_get_cfg();

	read_cmd_rbuf[0] = 0;
	if (iris_get_abyp_mode_blocking() == IRIS_ABYP_MODE) {
		iris_abyp_send_panel_cmd(panel, &cmdset);
	} else {
		iris_pt_send_panel_cmd(panel, &cmdset);
	}
	pcfg->power_mode = read_cmd_rbuf[0];

	IRIS_LOGI("%s(), power mode: 0x%02x", __func__, pcfg->power_mode);
}

/*static void _iris_update_gamma(void)
{
	if (iris_get_firmware_status() != FIRMWARE_LOAD_SUCCESS)
		return;

	iris_scaler_gamma_enable(true, 1);
	iris_update_fw_status(FIRMWARE_IN_USING);
}*/

static void _iris_send_iris_cmds(struct dsi_panel *panel, int type)
{
	char *ls_arr = NULL;
	struct dsi_panel_cmd_set panel_cmds;

	if (type == 1)
		iris_get_cmds(&panel_cmds, &ls_arr);
	else
		iris_get_lightoff_cmds(&panel_cmds, &ls_arr);

	iris_pt_send_panel_cmd(panel, &panel_cmds);
}

int iris_lightup(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds)
{
	ktime_t ktime0;
	ktime_t ktime1;
	uint32_t timeus0 = 0;
	uint32_t timeus1 = 0;
	uint8_t type = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;

	IRIS_LOGI("%s(%d), mode: %s, timing@%u +++", __func__, __LINE__,
		iris_get_abyp_mode_blocking() == IRIS_PT_MODE ? "PT" : "ABYP",
		pcfg->cur_timing);

 if (pcfg->iris_isolate_status == 1) {
    return rc;
 }
	ktime0 = ktime_get();
	iris_set_pinctrl_state(true);
	_iris_pre_lightup(panel);
	type = iris_get_cont_splash_type();
	pcfg->frame_kickoff_count[0] = 0;

	/*use to debug cont splash*/
	if (type == IRIS_CONT_SPLASH_LK) {
		IRIS_LOGI("%s(%d), enter cont splash", __func__, __LINE__);
		_iris_send_cont_splash_pkt(IRIS_CONT_SPLASH_LK);
	} else {
		_iris_send_lightup_pkt();
		pcfg->dport_is_disable = false;
		//_iris_update_gamma();
	}

	if (panel->bl_config.type == DSI_BACKLIGHT_PWM)
		iris_pwm_freq_set(panel->bl_config.pwm_period_usecs);

	ktime1 = ktime_get();
	if (on_cmds)
		iris_pt_send_panel_cmd(panel, on_cmds);

	if (pcfg->output_iris_scaling)
		_iris_send_iris_cmds(panel, 1);

	/* do not merge */
	//if (pcfg->pq_update_path == PATH_DSI)
	//	iris_dsirecover_check(IRIS_PQUPDATE_OP);
	/* do not merge end */ 

	if (type == IRIS_CONT_SPLASH_LK)
		IRIS_LOGI("%s(), exit cont splash", __func__);
	else
		/*continuous splahs should not use dma setting low power*/
		iris_lp_enable_post();

	pcfg->prev_timing = pcfg->cur_timing;

	timeus0 = (u32) ktime_to_us(ktime1) - (u32)ktime_to_us(ktime0);
	timeus1 = (u32) ktime_to_us(ktime_get()) - (u32)ktime_to_us(ktime1);
	IRIS_LOGI("%s() spend time0 %d us, time1 %d us.",
		__func__, timeus0, timeus1);

	if (pcfg->light_up_path == PATH_DSI) {
		rc = iris_dsirecover_check(IRIS_LIGHTUP_OP);
		if (!rc) { /*PT lightup panel*/
			pcfg->valid = FULL_LIGHTUP;
		}
	} else {
		pcfg->valid = FULL_LIGHTUP;
	}

        iris_dma_ch1_trigger(true, 0);
#ifdef IRIS_MIPI_TEST
	iris_read_power_mode(panel);
#endif
	IRIS_LOGI("%s(%d), +++", __func__, __LINE__);

	return rc;
}

int iris_enable(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	int lightup_opt = iris_lightup_opt_get();
	u32 regs[] = {0xf1100808, 0xf1100818, 0xf1100a00, 0xf1100034, 0xf1100204};

	__iris_cont_splash_video_path_check(pcfg);
 
 	//set pcfg->pq_update_is_dsi_hs to 1 after panel on/off
  	pcfg->pq_update_is_dsi_hs = 1;

	IRIS_LOGI("%s(), lightup opt: 0x%x", __func__, lightup_opt);

	iris_lp_enable_pre();

	rc = iris_abyp_send_panel_cmd(panel, on_cmds);
	if (rc) {
		IRIS_LOGE("abyp light up failed!!!");
		return rc;
	}

	/* light up with PT */
	if (iris_get_abyp_mode_nonblocking() == IRIS_PT_MODE) {
		rc = 0;
		while (1) {
			if (rc != 0) {
				iris_dump_regs(regs, ARRAY_SIZE(regs));
				iris_dsi_recover();
				IRIS_LOGE("iris_lightup dsi_recover");
			}

			rc = iris_lightup_exit_abyp(true, true);
			if (rc != 0) {
				IRIS_LOGE("iris_lightup_exit_abyp failed.");
				continue;
			}

			rc = iris_lightup(panel, NULL);
			if (rc != 0) {
				IRIS_LOGE("iris_lightup failed.");
				continue;
			}

			if ((lightup_opt & 0x10) == 0x10 || iris_platform_get() == 0) {
				/* FPGA platform or pcfg->iris_default_mode_pt */
				break;
			}

			rc = iris_abyp_switch_proc(pcfg->display, IRIS_ABYP_MODE, true);
			if (rc == 0) {
				break;
			}

			IRIS_LOGE("enter abyp failed.");
		}
	}

	return rc;
}

int iris_disable(struct dsi_panel *panel, struct dsi_panel_cmd_set *off_cmds)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	pcfg->pq_update_is_dsi_hs = 1;
	return iris_lightoff(panel, off_cmds);
}

enum {
	SWITCH_ABYP_TO_ABYP = 0,
	SWITCH_ABYP_TO_PT,
	SWITCH_PT_TO_ABYP,
	SWITCH_PT_TO_PT,
	SWITCH_NONE,
};

static const char *_iris_switch_case_name(const uint32_t switch_case)
{
	const char *name = NULL;

	switch (switch_case) {
	case SWITCH_ABYP_TO_ABYP:
		name = "ABYP==>ABYP";
		break;
	case SWITCH_ABYP_TO_PT:
		name = "ABYP==>PT";
		break;
	case SWITCH_PT_TO_ABYP:
		name = "PT==>ABYP";
		break;
	case SWITCH_PT_TO_PT:
		name = "PT==>PT";
		break;
	default:
		name = "unknown";
	}

	return name;
}

static uint32_t _iris_switch_case(const u32 refresh_rate,
		const u32 frame_width, const u32 frame_height)
{
	bool cur_pt_mode = (iris_get_abyp_mode_blocking() == IRIS_PT_MODE);

	IRIS_LOGD("%s(), refersh rate %u, width %u, height %u, iris mode '%s'",
			__func__,
			refresh_rate, frame_width, frame_height,
			cur_pt_mode?"PT":"ABYP");

	if (cur_pt_mode)
		return SWITCH_PT_TO_PT;
	else
		return SWITCH_ABYP_TO_ABYP;
}


int iris_switch(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *switch_cmds,
		struct dsi_mode_info *mode_info)
{
	int i;
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	int lightup_opt = iris_lightup_opt_get();
	struct dsi_display_mode *cur_mode =
		container_of(mode_info, struct dsi_display_mode, timing);
	struct dsi_display *display = pcfg->display;
	u32 switch_case = _iris_switch_case(
			mode_info->refresh_rate,
			mode_info->h_active,
			mode_info->v_active);
	ktime_t ktime = ktime_get();

	for (i = 0; i < panel->num_display_modes; i++) {
		struct dsi_display_mode *m = &display->modes[i];

		if (mode_info->v_active == m->timing.v_active &&
			mode_info->h_active == m->timing.h_active &&
			mode_info->refresh_rate == m->timing.refresh_rate &&
			cur_mode->panel_mode == m->panel_mode &&
			cur_mode->pixel_clk_khz == m->pixel_clk_khz) {
			pcfg->cur_timing = i;
			break;
		}
	}

	IRIS_LOGI("%s(), switch to %ux%u@%uHz, timing idx %u, switch case %s",
			__func__,
			mode_info->h_active,
			mode_info->v_active,
			mode_info->refresh_rate,
			pcfg->cur_timing,
			_iris_switch_case_name(switch_case));

	if (lightup_opt & 0x8) {
		rc = iris_abyp_send_panel_cmd(panel, switch_cmds);
		IRIS_LOGI("%s(), force switch from ABYP to ABYP, total cost '%d us'",
				__func__,
				(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

		return rc;
	}

	if (switch_case == SWITCH_ABYP_TO_ABYP) {
		rc = iris_abyp_send_panel_cmd(panel, switch_cmds);
	}

	if (switch_case == SWITCH_PT_TO_PT) {
		rc = iris_pt_send_panel_cmd(panel, switch_cmds);
		iris_send_mode_switch_pkt();
	}

	IRIS_LOGI("%s(), return %d, total cost '%d us'",
			__func__, rc,
			(u32)(ktime_to_us(ktime_get()) - ktime_to_us(ktime)));

	return 0;
}

static int _iris_cont_splash_video_lightup_thread_main(void *data)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s(%d) >>>>>START>>>---", __func__, __LINE__);
	iris_lp_enable_pre();
	iris_set_pinctrl_state(true);
	_iris_pre_lightup(pcfg->panel);
	_iris_send_lightup_pkt();
	iris_lp_enable_post();

	/* FPGA platform or pcfg->iris_default_mode_pt */
	iris_lightup_exit_abyp(true, true);
	if ((iris_lightup_opt_get() & 0x10) == 0) {
		/*Default mode ABYP*/
		iris_abyp_switch_proc(pcfg->display, IRIS_ABYP_MODE, true);
	}
	__iris_cont_splash_video_path_check(pcfg);

#if 0
  	//we use PATH_I2C as the first path for pq update until panel on/off
  	pcfg->light_up_path = PATH_I2C;
  	pcfg->pq_update_path = PATH_I2C;
  	pcfg->single_ipopt_path = PATH_I2C;
  	pcfg->path_backup_need_restore = 1;
#endif

  	//set pcfg->pq_update_is_dsi_hs to 0 before panel on/off
  	pcfg->pq_update_is_dsi_hs = 0;
 
	IRIS_LOGI("%s(%d) <<<<<<END<<<---", __func__, __LINE__);
	return 0;
}

static void _iris_send_cont_splash_pkt(uint32_t type)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rt = 0;

	if (type == IRIS_CONT_SPLASH_LK) {
		/* Do nothing */
		pcfg->cont_splash_status = 1;
	} else if (type == IRIS_CONT_SPLASH_KERNEL) {
		iris_lp_enable_pre();
		/* kernel simulates UEFI light up, called by UEFI */
		rt = iris_lightup_exit_abyp(true, true);
		if (!rt) {
			iris_set_pinctrl_state(true);
			_iris_pre_lightup(pcfg->panel);
			_iris_send_lightup_pkt();
			iris_lp_enable_post();
		}
		pcfg->cont_splash_status = 1;
	} else if (type == IRIS_CONT_SPLASH_BYPASS) {
		iris_lp_enable_pre();
		/* UEFI continuous splash, UEFI works in ABP mode */
		rt = iris_lightup_exit_abyp(true, true);
		if (!rt) {
			iris_set_pinctrl_state(true);
			_iris_pre_lightup(pcfg->panel);
			_iris_send_lightup_pkt();
			iris_lp_enable_post();
			iris_abyp_switch_proc(pcfg->display, IRIS_ABYP_MODE, true);
		}
		pcfg->cont_splash_status = 1;
	} else if (type == IRIS_CONT_SPLASH_VIDEO_BYPASS) {
		pcfg->light_up_path = PATH_I2C;
		pcfg->pq_update_path = PATH_I2C;
		pcfg->single_ipopt_path = PATH_I2C;
		pcfg->dsirecover_check_method = CHECK_NONE;
		pcfg->lp_ctrl.esd_ctrl = 0;
		pcfg->path_backup_need_restore = 1;
		kthread_run(_iris_cont_splash_video_lightup_thread_main, NULL, "iris_splash_video");
	}
}

int iris_preload(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int rt = 0;

	_iris_pre_lightup(pcfg->panel);
	_iris_send_lightup_pkt();
	iris_lp_enable_post();

	return rt;
}

void iris_send_cont_splash(uint32_t type)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (iris_get_abyp_mode_blocking() == IRIS_ABYP_MODE)
		return;

  if (pcfg->iris_isolate_status == 1) {
    pcfg->dsirecover_check_method = CHECK_NONE;
    pcfg->lp_ctrl.esd_ctrl = 0;
    return;
  }

	mutex_lock(&pcfg->panel->panel_lock);
	_iris_send_cont_splash_pkt(type);
	pcfg->valid = FULL_LIGHTUP;
	mutex_unlock(&pcfg->panel->panel_lock);
}


int iris_lightoff(struct dsi_panel *panel, struct dsi_panel_cmd_set *off_cmds)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	/*do not need to send dsi cmd when panel pre-off*/
	if (pcfg->valid == FULL_LIGHTUP)
		pcfg->valid = PARAM_PARSED;

	__iris_cont_splash_video_path_check(pcfg);
	IRIS_LOGI("%s(%d), mode: %s ---", __func__, __LINE__,
		iris_get_abyp_mode_blocking() == IRIS_PT_MODE ? "PT" : "ABYP");

	if (iris_get_abyp_mode_blocking() == IRIS_PT_MODE) {
		if (pcfg->output_iris_scaling) {
			off_cmds = NULL;
			iris_quality_setting_off();
			_iris_send_iris_cmds(panel, 0);
		} else {
			iris_pt_send_panel_cmd(panel, off_cmds);
			iris_quality_setting_off();
		}
	} else {
		iris_abyp_send_panel_cmd(panel, off_cmds);
		iris_quality_setting_off();
	}
	iris_lp_setting_off();
	iris_set_pinctrl_state(false);

	/* clear abyp mode when panel off */
	if (pcfg->valid == PARAM_PARSED)
		iris_force_abyp_mode(IRIS_PT_MODE);

	IRIS_LOGI("%s(%d) ---", __func__, __LINE__);

	return 0;
}

static void _iris_send_update_opt(struct iris_update_ipopt *popt,
		struct iris_cmd_comp *pasm_comp, uint8_t path)
{
	int32_t ip = 0;
	int32_t rc = 0;
	struct iris_ctrl_opt ctrl_opt;

	ip = popt->ip;
	ctrl_opt.ip = popt->ip;
	ctrl_opt.opt_id = popt->opt_new;
	ctrl_opt.chain = popt->chain;

	/*speical deal with lut table*/
	if (_iris_is_lut(ip))
		rc = _iris_send_lut_pkt(&ctrl_opt, pasm_comp, true, path);
	else
		rc = _iris_send_dtsi_pkt(&ctrl_opt, pasm_comp, path);

	if (rc)
		panic("%s\n", __func__);
}



static void _iris_send_pq_cmds(struct iris_update_ipopt *popt,
		int ipopt_cnt)
{
	int32_t i = 0;
	uint8_t path = 0;
	int32_t link_state = 0;
	struct iris_cmd_comp cmd_comp;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!popt || !ipopt_cnt) {
		IRIS_LOGE("%s(), invalid popt: %p or ipopt count: %d",
				__func__, popt, ipopt_cnt);
		return;
	}

	path = pcfg->pq_update_path;
	link_state = pcfg->pq_update_is_dsi_hs ?
					DSI_CMD_SET_STATE_HS : DSI_CMD_SET_STATE_LP;
	_iris_update_cmds(&cmd_comp, link_state, IRIS_PQUPDATE_OP);

	for (i = 0; i < ipopt_cnt; i++) {
		_iris_send_update_opt(&popt[i], &cmd_comp, path);
	}

	if (path == PATH_DSI) {
		iris_dsirecover_check(IRIS_PQUPDATE_OP);
	}
}

static int _iris_update_pq_seq(struct iris_update_ipopt *popt, int ipopt_cnt)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t ip = 0;
	int32_t opt_id = 0;
	struct iris_ctrl_seq *pseq = _iris_get_ctrl_seq();

	for (i = 0; i < ipopt_cnt; i++) {
		/*need to update sequence*/
		if (popt[i].opt_new != popt[i].opt_old) {
			for (j = 0; j < pseq->cnt; j++) {
				ip = pseq->ctrl_opt[j].ip;
				opt_id = pseq->ctrl_opt[j].opt_id;

				if ((ip == popt[i].ip) && (ip == GAMMA_LUT))
					break;

				if (ip == popt[i].ip && opt_id == popt[i].opt_old)
					break;
			}

			if (j == pseq->cnt) {
				IRIS_LOGE("%s(), failed to find seq for ip: %d opt: 0x%x",
						__func__, popt[i].ip, popt[i].opt_old);
				return -EINVAL;
			}

			pseq->ctrl_opt[j].opt_id = popt[i].opt_new;
		}
	}

	return 0;
}

void iris_update_pq_opt(bool bcommit)
{
	int32_t rc = 0;
	int ipopt_cnt = 0;
	struct iris_update_ipopt *popt = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();

	popt = pcfg->timing[pcfg->cur_timing].pq_update_cmd.update_ipopt_array;
	ipopt_cnt = pcfg->timing[pcfg->cur_timing].pq_update_cmd.array_index;

	if (!popt || !ipopt_cnt) {
		IRIS_LOGE("%s(), invalid popt: %p or ipopt count: %d",
				__func__, popt, ipopt_cnt);
		return;
	}

	popt[ipopt_cnt-1].chain = 0;

	rc = _iris_update_pq_seq(popt, ipopt_cnt);
	if ((!rc) && bcommit)
		_iris_send_pq_cmds(popt, ipopt_cnt);

	iris_init_ipopt_t();
}

static struct dsi_cmd_desc *_iris_get_specific_desc_from_ipopt(uint8_t ip,
		uint8_t opt_id, int32_t pos, int32_t tm_idx)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct iris_ip_opt *popt = _iris_find_specific_ip_opt(ip, opt_id, tm_idx);

	if (popt == NULL) {
		IRIS_LOGE("%s(), failed to find desc for ip: 0x%02x opt: 0x%02x",
			__func__, ip, opt_id);
		return NULL;
	}

	if (pos < 2) {
		IRIS_LOGE("%s(), invalid pos: %d", __func__, pos);
		return NULL;
	}

	return popt->cmd
			+ (pos * 4 - IRIS_OCP_HEADER_ADDR_LEN) / pcfg->split_pkt_size;
}

static struct dsi_cmd_desc *_iris_get_desc_from_ipopt(uint8_t ip,
		uint8_t opt_id, int32_t pos)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return _iris_get_specific_desc_from_ipopt(ip, opt_id, pos, pcfg->cur_timing);
}

static uint32_t *_iris_get_specific_ipopt_payload_data(uint8_t ip,
		uint8_t opt_id, int32_t pos, int32_t tm_idx)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc *pdesc = _iris_get_specific_desc_from_ipopt(ip, opt_id, pos, tm_idx);

	if (!pdesc) {
		IRIS_LOGE("%s(), can't find desc for ip: 0x%02x, opt: 0x%02x, pos: %d",
			__func__, ip, opt_id, pos);
		return NULL;
	} else if (pos > pdesc->msg.tx_len) {
		IRIS_LOGE("%s(), pos %d is out of paload length %zu",
			__func__, pos, pdesc->msg.tx_len);
		return NULL;
	}

	return (uint32_t *)((uint8_t *)pdesc->msg.tx_buf
						+ (pos * 4) % pcfg->split_pkt_size);
}

uint32_t *iris_get_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return _iris_get_specific_ipopt_payload_data(ip, opt_id, pos, pcfg->cur_timing);
}

uint32_t iris_get_ipopt_payload_len(uint8_t ip, uint8_t opt_id, int32_t pos)
{
	struct dsi_cmd_desc *pdesc = _iris_get_desc_from_ipopt(ip, opt_id, pos);

	if (!pdesc) {
		IRIS_LOGE("%s(), can't find desc for ip: 0x%02x, opt: 0x%02x, pos: %d",
			__func__, ip, opt_id, pos);
		return 0;
	} else if (pos > pdesc->msg.tx_len) {
		IRIS_LOGE("%s(), pos %d is out of paload length %zu",
			__func__, pos, pdesc->msg.tx_len);
		return 0;
	}

	return (uint32_t)(pdesc->msg.tx_len);
}

static bool _iris_timing_switch_supported(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->timing_cnt > IRIS_MIN_TIMING_MODES;
}

static void _iris_sync_ipopt_payload_data(uint8_t ip, uint8_t opt_id,
		int32_t pos, uint32_t value)
{
	uint32_t *pvalue = NULL;
	struct dsi_cmd_desc *pdesc = NULL;
	int32_t tm_idx = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!_iris_timing_switch_supported())
		return;

	for (tm_idx = 0; tm_idx < pcfg->timing_cnt; tm_idx++) {
		if (tm_idx == pcfg->cur_timing)
			continue;

		pdesc = _iris_get_specific_desc_from_ipopt(ip, opt_id, pos, tm_idx);
		if (!pdesc) {
			IRIS_LOGE("%s(), can't change value: %u for ip: 0x%02x, opt: 0x%02x, pos: %d, timing index: %d",
					__func__, value, ip, opt_id, pos, tm_idx);
			continue;
		}

		pvalue = (uint32_t *)((uint8_t *)pdesc->msg.tx_buf
				+ (pos * 4) % pcfg->split_pkt_size);
		pvalue[0] = value;
	}
}

void iris_set_ipopt_payload_data(uint8_t ip, uint8_t opt_id,
		int32_t pos, uint32_t value)
{
	uint32_t *pvalue = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc *pdesc = _iris_get_desc_from_ipopt(ip, opt_id, pos);

	if (!pdesc) {
		IRIS_LOGE("%s(), can't change value: %u for ip: 0x%02x, opt: 0x%02x, pos: %d",
				__func__, value, ip, opt_id, pos);
		return;
	}

	pvalue = (uint32_t *)((uint8_t *)pdesc->msg.tx_buf
			+ (pos * 4) % pcfg->split_pkt_size);
	pvalue[0] = value;

	_iris_sync_ipopt_payload_data(ip, opt_id, pos, value);
}

static void _iris_sync_bitmask(struct iris_update_regval *pregval)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t orig_val = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	struct iris_ip_opt *popt = NULL;
	int32_t tm_idx = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (!pregval) {
		IRIS_LOGE("%s(), invalid input", __func__);
		return;
	}

	if (!_iris_timing_switch_supported())
		return;

	ip = pregval->ip;
	opt_id = pregval->opt_id;

	for (tm_idx = 0; tm_idx < pcfg->timing_cnt; tm_idx++) {
		if (tm_idx == pcfg->cur_timing)
			continue;

		popt = _iris_find_specific_ip_opt(ip, opt_id, tm_idx);
		if (popt == NULL) {
			IRIS_LOGW("%s(), can't find ip: 0x%02x opt: 0x%02x, from timing index: %d",
					__func__, ip, opt_id, tm_idx);
			continue;
		} else if (popt->cmd_cnt != 1) {
			IRIS_LOGW("%s(), invalid bitmask for ip: 0x%02x, opt: 0x%02x, tming index: %d, popt len: %d",
					__func__, ip, opt_id, tm_idx, popt->cmd_cnt);
			continue;
		}

		data = (uint32_t *)popt->cmd[0].msg.tx_buf;

		orig_val = cpu_to_le32(data[2]);
		val = orig_val & (~pregval->mask);
		val |= (pregval->value & pregval->mask);
		data[2] = val;
	}
}

void iris_update_bitmask_regval_nonread(struct iris_update_regval *pregval,
		bool is_commit)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t orig_val = 0;
	uint32_t *data = NULL;
	uint32_t val = 0;
	struct iris_ip_opt *popt = NULL;

	if (!pregval) {
		IRIS_LOGE("%s(), pregval is null", __func__);
		return;
	}

	ip = pregval->ip;
	opt_id = pregval->opt_id;

	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL) {
		IRIS_LOGE("%s(), can't find popt for ip: 0x%02x opt: 0x%02x",
				__func__, ip, opt_id);
		return;
	}

	if (popt->cmd_cnt != 1) {
		IRIS_LOGE("%s(), invalid bitmask popt->cmd_cnt: %d",
				__func__, popt->cmd_cnt);
		return;
	}

	data = (uint32_t *)popt->cmd[0].msg.tx_buf;
	orig_val = cpu_to_le32(data[2]);
	val = orig_val & (~pregval->mask);
	val |= (pregval->value  & pregval->mask);
	data[2] = val;

	_iris_sync_bitmask(pregval);

	pregval->value = val;

	if (is_commit)
		iris_send_ipopt_cmds(ip, opt_id);
}

void iris_update_bitmask_regval(struct iris_update_regval *pregval,
		bool is_commit)
{
	int32_t ip = 0;
	int32_t opt_id = 0;
	uint32_t *data = NULL;
	struct iris_ip_opt *popt = NULL;

	if (!pregval) {
		IRIS_LOGE("%s(), pregval is null", __func__);
		return;
	}

	ip = pregval->ip;
	opt_id = pregval->opt_id;
	popt = iris_find_ip_opt(ip, opt_id);
	if (popt == NULL) {
		IRIS_LOGE("%s(), can't find popt for ip: 0x%02x opt: 0x%02x",
				__func__, ip, opt_id);
		return;
	}

	if (popt->cmd_cnt != 2) {
		IRIS_LOGE("%s(), invalid bitmask opt cmd count: %d",
				__func__, popt->cmd_cnt);
		return;
	}

	data = (uint32_t *)popt->cmd[1].msg.tx_buf;
	data[2] = cpu_to_le32(pregval->mask);
	data[3] = cpu_to_le32(pregval->value);

	if (is_commit)
		iris_send_ipopt_cmds(ip, opt_id);
}

static void _iris_error_check_dir_write(void)
{
	uint32_t header, address;
	uint32_t val[2] = {0};

	header = PXLW_DIRECTBUS_WRITE;
	address = 0xF1090500;
	val[0] = 0xF1090500;

	iris_ocp_write_vals(header, address, 1, val, DSI_CMD_SET_STATE_HS);
	IRIS_LOGD("%s", __func__);
}

static int _iris_error_check_reg_read(uint32_t addr, uint32_t checkvalue)
{
	int ret = 0;
	uint32_t val;
	struct iris_cfg *pcfg = iris_get_cfg();
	
	if (pcfg->dsirecover_check_path == PATH_I2C) {
		ret = iris_ioctl_i2c_read(addr, &val);
		if (ret) {
			IRIS_LOGE("%s(%d), i2c read registers 0x%x val: 0x%x, fail.", __func__, __LINE__, addr, val);
			return 1;
		}
	} else {
		val = _iris_dsi_ocp_read(addr, DSI_CMD_SET_STATE_HS);
	}

	if ((val & 0xffffffff) !=  checkvalue)
		ret = 1;
	else
		ret = 0;

	IRIS_LOGI("iris check: 0x%02x, : check 0x%x, val: 0x%x %s", addr, checkvalue, val, (ret == 0)?"okay":"failure");


	return ret;
}

int iris_dsirecover_check(enum iris_op_type op_type)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGV("dsirecover_check_method = %d, op_type=%d", pcfg->dsirecover_check_method, op_type);

	if ((op_type != IRIS_LIGHTUP_OP) && (op_type != IRIS_PQUPDATE_OP) && (op_type != IRIS_ESDCHECK_OP)) {
		IRIS_LOGE("invalid op_type [%d]", op_type);
		return -EINVAL;
	}

	if ((op_type == IRIS_PQUPDATE_OP) || (op_type == IRIS_ESDCHECK_OP))
		return rc;

	switch (pcfg->dsirecover_check_method) {
	case CHECK_NONE:
		break;
	case CHECK_ONLY_READ:
		rc = _iris_error_check_reg_read(MIPI_RX_DSI_STATUS, RX_DSI_STATUS_VALUE);
		break;
	case CHECK_WRITE_AND_READ:
		_iris_error_check_dir_write();
		rc = _iris_error_check_reg_read(MIPI_RX_DSI_STATUS, RX_DSI_STATUS_VALUE);
		break;
	default:
		break;
	}

	return rc;
}

static ssize_t _iris_cont_splash_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	_iris_set_cont_splash_type(val);

	if ((val == IRIS_CONT_SPLASH_KERNEL) || (val == IRIS_CONT_SPLASH_BYPASS)) {
		struct iris_cfg *pcfg = iris_get_cfg();

		mutex_lock(&pcfg->panel->panel_lock);
		_iris_send_cont_splash_pkt(val);
		mutex_unlock(&pcfg->panel->panel_lock);
	}

	IRIS_LOGI("%s(), value is %zu", __func__, val);

	return count;
}

static ssize_t _iris_cont_splash_read(struct file *file,
		char __user *buff, size_t count, loff_t *ppos)
{
	uint8_t type;
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	type = iris_get_cont_splash_type();
	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", type);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}


static const struct file_operations iris_cont_splash_fops = {
	.open = simple_open,
	.write = _iris_cont_splash_write,
	.read = _iris_cont_splash_read,
};

static ssize_t _iris_dbg_chip_id_read(struct file *file,
		char __user *buff, size_t count, loff_t *ppos)
{
	int tot = 0;
	char bp[512];
	struct iris_cfg *pcfg = iris_get_cfg();

	if (*ppos)
		return 0;

	tot = scnprintf(bp, sizeof(bp), "%u\n", pcfg->chip_id);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_chip_id_fops = {
	.open = simple_open,
	.read = _iris_dbg_chip_id_read,
};

static ssize_t _iris_dbg_power_mode_read(struct file *file,
		char __user *buff, size_t count, loff_t *ppos)
{
	int tot = 0;
	struct iris_cfg *pcfg = NULL;
	char bp[512];

	if (*ppos)
		return 0;

	pcfg = iris_get_cfg();

	tot = scnprintf(bp, sizeof(bp), "0x%02x\n", pcfg->power_mode);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;
	*ppos += tot;

	return tot;
}

static const struct file_operations iris_power_mode_fops = {
	.open = simple_open,
	.read = _iris_dbg_power_mode_read,
};

static ssize_t _iris_dbg_i2c_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{

	unsigned long val;
	int ret = 0;
	uint32_t addr, dat = 0;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	IRIS_LOGI("%s(%d)", __func__, __LINE__);

	addr = 0xF1608000;
	dat = val + 0x1;
	ret = iris_ioctl_i2c_write(addr, dat);
	if (ret)
		IRIS_LOGE("%s(%d), ret: %d", __func__, __LINE__, ret);

	return count;

}

static ssize_t _iris_dbg_i2c_read(struct file *file,
		char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int tot = 0;
	char bp[512];
	uint32_t addr, val = 0;

	if (*ppos)
		return 0;

	addr = 0xF1608000;

	ret = iris_ioctl_i2c_read(addr, &val);
	if (ret)
		IRIS_LOGE("%s(%d) ret: %d", __func__, __LINE__, ret);

	IRIS_LOGI("%s(%d) val: 0x%x", __func__, __LINE__, val);

	tot = scnprintf(bp, sizeof(bp), "0x%x\n", val);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_i2c_srw_fops = {
	.open = simple_open,
	.write = _iris_dbg_i2c_write,
	.read = _iris_dbg_i2c_read,
};

static ssize_t _iris_list_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	int32_t ip;
	int32_t opt_id;
	int32_t pos;
	uint32_t value;
	char buf[64];
	uint32_t *payload = NULL;

	if (count > sizeof(buf))
		return -EINVAL;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end if string */

	if (sscanf(buf, "%x %x %x %x", &ip, &opt_id, &pos, &value) != 4)
		return -EINVAL;

	IRIS_LOGI("%x %x %x %x", ip, opt_id, pos, value);

	payload = iris_get_ipopt_payload_data(ip&0xFF, opt_id&0xFF, 2);
	iris_set_ipopt_payload_data(ip&0xFF, opt_id&0xFF, pos, value);

	return count;
}

static const struct file_operations iris_list_debug_fops = {
	.open = simple_open,
	.write = _iris_list_write,
};

static ssize_t _iris_dbg_loopback_read(struct file *file,
		char __user *buff, size_t count, loff_t *ppos)
{
	int ret = -1;
	int tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	ret = iris_loopback_validate();
	IRIS_LOGI("%s(%d) ret: %d", __func__, __LINE__, ret);

	tot = scnprintf(bp, sizeof(bp), "0x%x\n", ret);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_loopback_debug_fops = {
	.open = simple_open,
	.read = _iris_dbg_loopback_read,
};

/* adb shell "echo 05 01 > d/iris/iris_send_ipopt_cmd"
 * parameters: ip(Hex) opt(Hex)
 * send ip opt cmds to iris
 */
static ssize_t _iris_send_ipopt_cmds_(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	uint8_t ip;
	uint8_t opt_id;
	int32_t size = 0;
	char buf[64];

	if (count > sizeof(buf))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end if string */

	size = sscanf(buf, "%x %x", &ip, &opt_id);
	if (size == 2) {
		iris_send_ipopt_cmds(ip, opt_id);
	} else {
		return -EINVAL;
	}

	IRIS_LOGI("%s(), ip: 0x%02X opt: 0x%02X",
			__func__, ip, opt_id);

	return count;
}

static const struct file_operations iris_send_ipopt_cmds_fops = {
	.open = simple_open,
	.write = _iris_send_ipopt_cmds_,
};
/* adb shell "echo 05 00 45 00 > d/iris/iris_cmd_payload"
 * or
 * adb shell "echo 05 00 45 > d/iris/iris_cmd_payload"
 * parameters: ip(Hex) opt(Hex) pos(Dec) timing index(Dec)
 *             param 4th default is current timing
 * check dump value by kernel log
 */
static ssize_t _iris_dump_cmd_payload(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	uint8_t ip;
	uint8_t opt_id;
	int32_t pos;
	int32_t tm_idx = -1;
	int32_t size = 0;
	char buf[64];
	uint32_t *payload = NULL;

	if (count > sizeof(buf))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	/* end if string */

	size = sscanf(buf, "%x %x %d %d", &ip, &opt_id, &pos, &tm_idx);
	if (size == 3) {
		struct iris_cfg *pcfg = iris_get_cfg();
		tm_idx = pcfg->cur_timing;
		IRIS_LOGW("%s(), current timing index: %d", __func__, tm_idx);
	} else if (size == 4) {
		IRIS_LOGW("%s(), specific timing index: %d", __func__, tm_idx);
	} else {
		return -EINVAL;
	}

	payload = _iris_get_specific_ipopt_payload_data(ip, opt_id, 2, tm_idx);
	if (payload == NULL)
		return -EFAULT;

	IRIS_LOGW("%s(), for ip: 0x%02X opt: 0x%02X, timing index: %d, payload[%d] is: 0x%08X",
			__func__, ip, opt_id, tm_idx, pos, payload[pos]);

	return count;
}

static const struct file_operations iris_cmd_payload_fops = {
	.open = simple_open,
	.write = _iris_dump_cmd_payload,
};

static int _iris_dbg_init_cont_splash(struct dsi_display *display)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("%s(), failed to create dir 'iris', error: %ld",
				   __func__, PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("last_per_pkt", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->add_last_flag);

	debugfs_create_u32("light_up_path", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->light_up_path);

	debugfs_create_u32("pq_update_path", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->pq_update_path);

	debugfs_create_u32("single_ipopt_path", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->single_ipopt_path);

	debugfs_create_u8("embedded_en_for_lightup", 0644, pcfg->dbg_root,
		&pcfg->dsi_embedded_enable[0]);

	debugfs_create_u8("embedded_en_for_pqupdate", 0644, pcfg->dbg_root,
		&pcfg->dsi_embedded_enable[1]);

	debugfs_create_u32("non_embedded_len_for_lightup", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->non_embedded_xfer_len[0]);

	debugfs_create_u32("non_embedded_len_for_pqupdate", 0644, pcfg->dbg_root,
		(u32 *)&pcfg->non_embedded_xfer_len[1]);

	debugfs_create_u8("dsirecover_check_method", 0644, pcfg->dbg_root,
		(u8 *)&pcfg->dsirecover_check_method);

	debugfs_create_u8("dsirecover_check_path", 0644, pcfg->dbg_root,
		(u8 *)&pcfg->dsirecover_check_path);

	debugfs_create_u8("pq_update_is_dsi_hs", 0644, pcfg->dbg_root,
		(u8 *)&pcfg->pq_update_is_dsi_hs);

	if (debugfs_create_file("iris_cont_splash", 0644, pcfg->dbg_root, display,
				&iris_cont_splash_fops) == NULL) {
		IRIS_LOGE("%s(%d): failed to create file 'iris_cont_splash'",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("chip_id", 0644, pcfg->dbg_root, display,
				&iris_chip_id_fops) == NULL) {
		IRIS_LOGE("%s(%d): failed to create file 'chip_id'",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("power_mode", 0644, pcfg->dbg_root, display,
				&iris_power_mode_fops) == NULL) {
		IRIS_LOGE("%s(%d): failed to create file 'power_mode'",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_i2c_srw",	0644, pcfg->dbg_root, display,
				&iris_i2c_srw_fops) == NULL) {
		IRIS_LOGE("%s(%d): failed to create file 'iris_i2c_srw'",
			__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_list_debug", 0644, pcfg->dbg_root, display,
				&iris_list_debug_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	if (debugfs_create_file("iris_loopback_debug", 0644, pcfg->dbg_root, display,
				&iris_loopback_debug_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file: index fail",
				__FILE__, __LINE__);
		return -EFAULT;
	}
	if (debugfs_create_file("iris_send_ipopt_cmds", 0644, pcfg->dbg_root, display,
				&iris_send_ipopt_cmds_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file for 'iris_send_ipopt' failed",
				__FILE__, __LINE__);
		return -EFAULT;
	}
	if (debugfs_create_file("iris_cmd_payload", 0644, pcfg->dbg_root, display,
				&iris_cmd_payload_fops) == NULL) {
		IRIS_LOGE("%s(%d): debugfs_create_file for 'iris_cmd_payload' failed",
				__FILE__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

void iris_prepare(void)
{
	static bool iris_boot;
	struct iris_cfg *pcfg = iris_get_cfg();
	uint i;

	if (iris_get_cfg()->valid < PARAM_PARSED)
		return;

	if (iris_boot)
		return;

	if (iris_parse_lut_cmds()) {
		pcfg->valid = PARAM_NONE;
		iris_force_abyp_mode(IRIS_ABYP_MODE);
	}
	iris_alloc_seq_space();
	for (i = 0; i < pcfg->timing_cnt; i++) {
		pcfg->cur_timing = i;
		iris_alloc_update_ipopt_space();
	}
	pcfg->cur_timing = 0;
	iris_boot = true;
}

int iris_wait_vsync(uint32_t count)
{
	struct drm_encoder *drm_enc;
	struct iris_cfg *pcfg = iris_get_cfg();
	int i;

	if (pcfg->display->bridge == NULL)
		return -ENOLINK;

	drm_enc = pcfg->display->bridge->base.encoder;
	if (!drm_enc || !drm_enc->crtc)
		return -ENOLINK;

	if (sde_encoder_is_disabled(drm_enc))
		return -EIO;

	//mutex_unlock(&pcfg->panel->panel_lock);
	for (i = 0; i < count; i++)
		sde_encoder_wait_for_event(drm_enc, MSM_ENC_VBLANK);
	//mutex_lock(&pcfg->panel->panel_lock);

	return 0;
}

void iris_deinit(void)
{
	int i;
	struct iris_cfg *pcfg = iris_get_cfg();

	for (pcfg->cur_timing = 0; pcfg->cur_timing < pcfg->timing_cnt; pcfg->cur_timing++) {
		for (i = 0; i < IRIS_PIP_IDX_CNT; i++)
			iris_free_ipopt_buf(i);

		if (pcfg->timing[pcfg->cur_timing].pq_update_cmd.update_ipopt_array) {
			kfree(pcfg->timing[pcfg->cur_timing]
				      .pq_update_cmd.update_ipopt_array);
			pcfg->timing[pcfg->cur_timing].pq_update_cmd.update_ipopt_array = NULL;
			pcfg->timing[pcfg->cur_timing].pq_update_cmd.array_index = 0;
		}
	}

	iris_free_seq_space();
	iris_driver_unregister();
	iris_i2c_bus_exit();
}

static int _iris_dev_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s()", __func__);
	if (!pdev) {
		IRIS_LOGE("%s(), pdev not found", __func__);
		return -ENODEV;
	}

	pcfg->pdev = pdev;
	dev_set_drvdata(&pdev->dev, pcfg);

	rc = iris_enable_pinctrl(pdev);
	if (rc) {
		IRIS_LOGE("%s(), failed to enable pinctrl, return: %d",
				__func__, rc);
		return rc;
	}

	rc = iris_parse_gpio(pdev);
	if (rc) {
		IRIS_LOGE("%s(), failed to parse gpio, return: %d",
				__func__, rc);
		return rc;
	}

	if (pcfg->iris_clk == NULL) {
		pcfg->iris_clk = clk_get(&pdev->dev, "iris_clk");
		if (IS_ERR_OR_NULL(pcfg->iris_clk)) {
			IRIS_LOGE("%s(), failed to get iris_clk", __func__);
			pcfg->iris_clk = NULL;
		} else {
			rc = clk_prepare_enable(pcfg->iris_clk);
			IRIS_LOGI("%s(), iris get iris_clk ok", __func__);
		}
	}

	return 0;
}

static int _iris_dev_remove(struct platform_device *pdev)
{
	struct iris_cfg *pcfg = dev_get_drvdata(&pdev->dev);

	IRIS_LOGI("%s()", __func__);

	iris_release_gpio(pcfg);

	return 0;
}

static const struct of_device_id iris_dt_match[] = {
	{.compatible = "pxlw,iris"},
	{}
};

static struct platform_driver iris_driver = {
	.probe = _iris_dev_probe,
	.remove = _iris_dev_remove,
	.driver = {
		.name = "pxlw-iris",
		.of_match_table = iris_dt_match,
	},
};

int iris_driver_register(void)
{
	return platform_driver_register(&iris_driver);
}

void iris_driver_unregister(void)
{
	return platform_driver_unregister(&iris_driver);
}
