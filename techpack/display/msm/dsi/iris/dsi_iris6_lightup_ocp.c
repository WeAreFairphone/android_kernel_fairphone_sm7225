#include <video/mipi_display.h>
#include "dsi_iris6_api.h"
#include "dsi_iris6_lightup.h"
#include "dsi_iris6_lightup_ocp.h"
#include "dsi_iris6_lp.h"
#include "dsi_iris6_i3c.h"
#include "dsi_iris6_log.h"


#define IRIS_TX_HV_PAYLOAD_LEN   120
#define IRIS_TX_PAYLOAD_LEN 124
#define IRIS_PT_RD_CMD_NUM 3
#define IRIS_RD_PACKET_DATA 0xF141C018
#define IRIS_TX_INTSTAT_RAW 0xF141FFE4
#define IRIS_TX_READ_RESPONSE_RECEIVED 0x80000000
#define IRIS_TX_READ_ERR_MASK 0x6FEFFEFF
#define IRIS_TX_INTCLR 0xF141FFF0
#define IRIS_RD_PACKET_DATA_I3  0xF0C1C018

static char iris_read_cmd_rbuf[16];
static struct iris_ocp_cmd ocp_cmd;
static struct iris_ocp_cmd ocp_test_cmd[DSI_CMD_CNT];
static struct dsi_cmd_desc iris_test_cmd[DSI_CMD_CNT];

static void _iris_add_cmd_addr_val(struct iris_ocp_cmd *pcmd, u32 addr, u32 val)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(addr);
	*(u32 *)(pcmd->cmd + pcmd->cmd_len + 4) = cpu_to_le32(val);
	pcmd->cmd_len += 8;
}

static void _iris_add_cmd_payload(struct iris_ocp_cmd *pcmd, u32 payload)
{
	*(u32 *)(pcmd->cmd + pcmd->cmd_len) = cpu_to_le32(payload);
	pcmd->cmd_len += 4;
}

void iris_ocp_write_val(u32 address, u32 value)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0} };

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	_iris_add_cmd_payload(&ocp_cmd, 0xFFFFFFF0 | OCP_SINGLE_WRITE_BYTEMASK);
	_iris_add_cmd_addr_val(&ocp_cmd, address, value);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, value);

	iris_dsi_send_cmds(pcfg->panel, iris_ocp_cmd, 1, DSI_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);
}

void iris_ocp_write_vals(u32 header, u32 address, u32 size, u32 *pvalues, enum dsi_cmd_set_state state)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0} };
	u32 max_size = CMD_PKT_SIZE / 4 - 2;
	u32 i;

	while (size > 0) {
		memset(&ocp_cmd, 0, sizeof(ocp_cmd));

		_iris_add_cmd_payload(&ocp_cmd, header);
		_iris_add_cmd_payload(&ocp_cmd, address);
		if (size < max_size) {
			for (i = 0; i < size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			size = 0;
		} else {
			for (i = 0; i < max_size; i++)
				_iris_add_cmd_payload(&ocp_cmd, pvalues[i]);

			address += max_size * 4;
			pvalues += max_size;
			size -= max_size;
		}
		iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;
		IRIS_LOGD("%s(), header: %#x, addr: %#x, len: %zu", __func__,
				header, address, iris_ocp_cmd[0].msg.tx_len);

		iris_dsi_send_cmds(pcfg->panel, iris_ocp_cmd, 1, state, pcfg->vc_ctrl.to_iris_vc_id);
	}
}

void iris_ocp_bit_en_write(u32 address, u32 bit_en, u32 bit_val, enum dsi_cmd_set_state state)
{

	u32 vals[2] = {0};

	vals[0] = bit_en;
	vals[1] = bit_val;
	iris_ocp_write_vals(OCP_SINGLE_WRITE_BITEN, address, 2, vals, state);
}

static void _iris_ocp_write_addr(u32 address, u32 mode)
{
	struct iris_ocp_cmd ocp_cmd;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct dsi_cmd_desc iris_ocp_cmd[] = {
		{{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			 CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0} };

	/* Send OCP command.*/
	memset(&ocp_cmd, 0, sizeof(ocp_cmd));

	_iris_add_cmd_payload(&ocp_cmd, OCP_SINGLE_READ);
	_iris_add_cmd_payload(&ocp_cmd, address);
	iris_ocp_cmd[0].msg.tx_len = ocp_cmd.cmd_len;

	iris_dsi_send_cmds(pcfg->panel, iris_ocp_cmd, 1, mode, pcfg->vc_ctrl.to_iris_vc_id);
}

static u32 _iris_ocp_read_value(u32 mode)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	char pi_read[1] = {0x00};
	struct dsi_cmd_desc pi_read_cmd[] = {
		{{0, MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM, MIPI_DSI_MSG_REQ_ACK,
			 0, 0, sizeof(pi_read), pi_read, 0, NULL},
		1, 0} };
	u32 response_value;

	/* Read response.*/
	memset(iris_read_cmd_rbuf, 0, sizeof(iris_read_cmd_rbuf));
	pi_read_cmd[0].msg.rx_len = 4;
	pi_read_cmd[0].msg.rx_buf = iris_read_cmd_rbuf;
	iris_dsi_send_cmds(pcfg->panel, pi_read_cmd, 1, mode, pcfg->vc_ctrl.to_iris_vc_id);

	IRIS_LOGD("%s(), read register: 0x%02x 0x%02x 0x%02x 0x%02x", __func__,
			iris_read_cmd_rbuf[0], iris_read_cmd_rbuf[1],
			iris_read_cmd_rbuf[2], iris_read_cmd_rbuf[3]);

	response_value = iris_read_cmd_rbuf[0] | (iris_read_cmd_rbuf[1] << 8)
				| (iris_read_cmd_rbuf[2] << 16) | (iris_read_cmd_rbuf[3] << 24);

	return response_value;
}

static u32 _iris_i2c_single_read(u32 address)
{
	int ret = 0;
	u32 val = 0;

	ret = iris_i2c_read(address, &val, 1);
	if (ret) {
		IRIS_LOGE("%s(%d), i2c ocp single read fail, return: %d",
			__func__, __LINE__, ret);
		return 0;
	}

	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, val);

	return val;
}

u32 _iris_dsi_ocp_read(u32 address, u32 mode)
{
	u32 value = 0;

	_iris_ocp_write_addr(address, mode);

	value = _iris_ocp_read_value(mode);
	IRIS_LOGD("%s(), addr: %#x, value: %#x", __func__, address, value);

	return value;
}

u32 iris_ocp_read(u32 address, u32 mode)
{
	uint8_t path = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	path = pcfg->light_up_path;

	if (path == PATH_I2C) {
		IRIS_LOGD("%s(%d), path select i2c", __func__, __LINE__);
		return _iris_i2c_single_read(address);
	}

	if (path == PATH_DSI) {
		IRIS_LOGD("%s(%d), path select dsi", __func__, __LINE__);
		return _iris_dsi_ocp_read(address, mode);
	}

	IRIS_LOGE("%s(%d), invalid path: %d, neither i2c nor dsi",
		__func__, __LINE__, path);

	return 0;
}

void _iris_dump_packet(u8 *data, int size)
{
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_NONE, 16, 4, data, size, false);
}

void iris_write_test(struct dsi_panel *panel,
		u32 iris_addr, int ocp_type, u32 pkt_size)
{
	union iris_ocp_cmd_header ocp_header;
	struct dsi_cmd_desc iris_cmd = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0,
			CMD_PKT_SIZE, ocp_cmd.cmd, 0, NULL},
		1, 0};
	u32 test_value = 0xFFFF0000;
	struct iris_cfg *pcfg = iris_get_cfg();

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	memset(&ocp_cmd, 0, sizeof(ocp_cmd));
	memcpy(ocp_cmd.cmd, &ocp_header.header32, OCP_HEADER);
	ocp_cmd.cmd_len = OCP_HEADER;

	switch (ocp_type) {
	case OCP_SINGLE_WRITE_BYTEMASK:
	case OCP_SINGLE_WRITE_BITMASK:
		for (; ocp_cmd.cmd_len <= (pkt_size - 8); ) {
			_iris_add_cmd_addr_val(&ocp_cmd, iris_addr, test_value);
			test_value++;
			}
		break;

	case OCP_BURST_WRITE:
		test_value = 0xFFFF0000;
		_iris_add_cmd_addr_val(&ocp_cmd, iris_addr, test_value);
		if (pkt_size <= ocp_cmd.cmd_len)
			break;
		test_value++;
		for (; ocp_cmd.cmd_len <= pkt_size - 4;) {
			_iris_add_cmd_payload(&ocp_cmd, test_value);
			test_value++;
		}
		break;
	default:
		break;
	}

	IRIS_LOGI("%s(), len: %d iris addr: %#x test value: %#x", __func__,
		ocp_cmd.cmd_len, iris_addr, test_value);

	iris_cmd.msg.tx_len = ocp_cmd.cmd_len;
	iris_dsi_send_cmds(panel, &iris_cmd, 1, DSI_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);

	if (IRIS_IF_LOGD())
		_iris_dump_packet(ocp_cmd.cmd, ocp_cmd.cmd_len);
}

void iris_write_test_muti_pkt(struct dsi_panel *panel,
		struct iris_ocp_dsi_tool_input *ocp_input)
{
	union iris_ocp_cmd_header ocp_header;
	u32 test_value = 0xFF000000;
	int cnt = 0;
	u32 iris_addr, ocp_type, pkt_size, total_cnt;
	struct iris_cfg *pcfg = iris_get_cfg();

	ocp_type = ocp_input->iris_ocp_type;
	test_value = ocp_input->iris_ocp_value;
	iris_addr = ocp_input->iris_ocp_addr;
	total_cnt = ocp_input->iris_ocp_cnt;
	pkt_size = ocp_input->iris_ocp_size;

	memset(iris_test_cmd, 0, sizeof(iris_test_cmd));
	memset(ocp_test_cmd, 0, sizeof(ocp_test_cmd));

	memset(&ocp_header, 0, sizeof(ocp_header));
	ocp_header.header32 = 0xFFFFFFF0 | ocp_type;

	switch (ocp_type) {
	case OCP_SINGLE_WRITE_BYTEMASK:
	case OCP_SINGLE_WRITE_BITMASK:

		for (cnt = 0; cnt < total_cnt; cnt++) {

			memcpy(ocp_test_cmd[cnt].cmd,
					&ocp_header.header32, OCP_HEADER);
			ocp_test_cmd[cnt].cmd_len = OCP_HEADER;

			test_value = 0xFF000000;
			test_value = 0xFF000000 | (cnt << 16);
			while (ocp_test_cmd[cnt].cmd_len <= (pkt_size - 8)) {
				_iris_add_cmd_addr_val(&ocp_test_cmd[cnt],
					(iris_addr + cnt*4), test_value);
				test_value++;
			}

			iris_test_cmd[cnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
			iris_test_cmd[cnt].msg.tx_len = ocp_test_cmd[cnt].cmd_len;
			iris_test_cmd[cnt].msg.tx_buf = ocp_test_cmd[cnt].cmd;
		}
		iris_test_cmd[total_cnt - 1].last_command = true;
		break;

	case OCP_BURST_WRITE:
		for (cnt = 0; cnt < total_cnt; cnt++) {
			memcpy(ocp_test_cmd[cnt].cmd, &ocp_header.header32, OCP_HEADER);
			ocp_test_cmd[cnt].cmd_len = OCP_HEADER;
			test_value = 0xFF000000;
			test_value = 0xFF000000 | (cnt << 16);

			_iris_add_cmd_addr_val(&ocp_test_cmd[cnt],
					(iris_addr + cnt*4), test_value);
			test_value++;

			while (ocp_test_cmd[cnt].cmd_len <= pkt_size - 4) {
				_iris_add_cmd_payload(&ocp_test_cmd[cnt], test_value);
				test_value++;
			}

			iris_test_cmd[cnt].msg.type = MIPI_DSI_GENERIC_LONG_WRITE;
			iris_test_cmd[cnt].msg.tx_len = ocp_test_cmd[cnt].cmd_len;
			iris_test_cmd[cnt].msg.tx_buf = ocp_test_cmd[cnt].cmd;

		}
		iris_test_cmd[total_cnt - 1].last_command = true;
		break;
	default:
		break;

	}

	IRIS_LOGI("%s(), total cnt: %#x iris addr: %#x test value: %#x",
		__func__, total_cnt, iris_addr, test_value);

	iris_dsi_send_cmds(panel, iris_test_cmd, total_cnt, DSI_CMD_SET_STATE_HS, pcfg->vc_ctrl.to_iris_vc_id);

	if (IRIS_IF_NOT_LOGV())
		return;

	for (cnt = 0; cnt < total_cnt; cnt++)
		_iris_dump_packet(ocp_test_cmd[cnt].cmd, ocp_test_cmd[cnt].cmd_len);

}

int iris_dsi_send_cmds(struct dsi_panel *panel, struct dsi_cmd_desc *cmds,
		u32 count, enum dsi_cmd_set_state state, u8 vc_id)
{
	int rc = 0;
	int i = 0;
	ssize_t len;
	struct iris_cfg *pcfg = NULL;
	struct dsi_display *display = NULL;
	const struct mipi_dsi_host_ops *ops = NULL;

	if (!panel || !panel->cur_mode)
		return -EINVAL;

	if (count == 0) {
		IRIS_LOGD("%s(), panel: %s no commands to be sent by state %d",
			 __func__, panel->name, state);
		goto error;
	}

	pcfg = iris_get_cfg();
	display = pcfg->display;
	ops = panel->host->ops;

	for (i = 0; i < count; i++) {
		cmds->msg.channel = vc_id;

		cmds->msg.flags = 0;

		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		WARN_ON(!mutex_is_locked(&panel->panel_lock));
		len = ops->transfer(panel->host, &cmds->msg);

		if (IRIS_IF_LOGVV())
			_iris_dump_packet((u8 *)cmds->msg.tx_buf, cmds->msg.tx_len);

		if (len < 0) {
			rc = len;
			IRIS_LOGE("%s(), failed to set cmds, type: %d, return: %d",
				__func__, cmds->msg.type, rc);
			goto error;
		}

		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms*1000, cmds->post_wait_ms*1000 + 10);

		cmds++;
	}

error:
	return rc;
}

static u32 _iris_pt_get_split_pkt_cnt(int dlen)
{
	u32 sum = 1;

	if (dlen > IRIS_TX_HV_PAYLOAD_LEN)
		sum = (dlen - IRIS_TX_HV_PAYLOAD_LEN
			+ IRIS_TX_PAYLOAD_LEN - 1) / IRIS_TX_PAYLOAD_LEN + 1;

	return sum;
}

/*
 * @Description: use to do statitics for cmds which should not less than 252
 *  if the payload is out of 252, it will change to more than one cmds
 * the first payload need to be
 *	4 (ocp_header) + 8 (tx_addr_header + tx_val_header)
 *	+ 2* payload_len (TX_payloadaddr + payload_len)<= 252
 * the sequence payloader need to be
 *	4 (ocp_header) + 2* payload_len (TX_payloadaddr + payload_len)<= 252
 * so the first payload should be no more than 120
 * the second and sequence need to be no more than 124
 * @Param: cmdset  cmds request
 * @return: the cmds number need to split
 */
static u32 _iris_pt_calc_cmd_cnt(struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u32 sum = 0;
	u32 dlen = 0;

	for (i = 0; i < cmdset->count; i++) {
		dlen = cmdset->cmds[i].msg.tx_len;

		sum += _iris_pt_get_split_pkt_cnt(dlen);
	}
	return sum;
}

static int _iris_pt_alloc_cmds(struct dsi_panel_cmd_set *cmdset,
		struct dsi_cmd_desc **ptx_cmds, struct iris_ocp_cmd **pocp_cmds)
{
	u32 cmds_cnt = _iris_pt_calc_cmd_cnt(cmdset);

	*ptx_cmds = vmalloc(cmds_cnt * sizeof(**ptx_cmds));
	if (!(*ptx_cmds)) {
		IRIS_LOGE("%s(), failed to alloc desc cmd by count: %u, len: %lu",
			__func__, cmds_cnt, cmds_cnt * sizeof(**ptx_cmds));
		return -ENOMEM;
	}

	*pocp_cmds = vmalloc(cmds_cnt * sizeof(**pocp_cmds));
	if (!(*pocp_cmds)) {
		IRIS_LOGE("%s(), failed to alloc pocp cmds by count: %u, len: %lu",
			__func__, cmds_cnt, cmds_cnt * sizeof(**pocp_cmds));
		vfree(*ptx_cmds);
		*ptx_cmds = NULL;
		return -ENOMEM;
	}

	return cmds_cnt;
}

static void _iris_pt_init_tx_cmd_hdr(struct dsi_panel_cmd_set *cmdset,
		struct dsi_cmd_desc *dsi_cmd, union iris_mipi_tx_cmd_header *header)
{
	u8 dtype = dsi_cmd->msg.type;

	memset(header, 0x00, sizeof(*header));
	header->stHdr.dtype = dtype;
	header->stHdr.linkState = (cmdset->state == DSI_CMD_SET_STATE_LP) ? 1 : 0;
}

static void _iris_pt_set_cmd_hdr(union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd, bool is_write)
{
	u32 dlen = 0;
	u8 *ptr = NULL;

	if (dsi_cmd == NULL)
		return;

	dlen = dsi_cmd->msg.tx_len;

	if (is_write)
		pheader->stHdr.writeFlag = 0x01;
	else
		pheader->stHdr.writeFlag = 0x00;

	if (pheader->stHdr.longCmdFlag == 0) {
		ptr = (u8 *)dsi_cmd->msg.tx_buf;
		if (dlen == 1) {
			pheader->stHdr.len[0] = ptr[0];
		} else if (dlen == 2) {
			pheader->stHdr.len[0] = ptr[0];
			pheader->stHdr.len[1] = ptr[1];
		}
	} else {
		pheader->stHdr.len[0] = dlen & 0xff;
		pheader->stHdr.len[1] = (dlen >> 8) & 0xff;
	}
}

static void _iris_pt_set_wrcmd_hdr(union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	_iris_pt_set_cmd_hdr(pheader, dsi_cmd, true);
}

static void _iris_pt_set_rdcmd_hdr(union iris_mipi_tx_cmd_header *pheader,
		struct dsi_cmd_desc *dsi_cmd)
{
	_iris_pt_set_cmd_hdr(pheader, dsi_cmd, false);
}

static void _iris_pt_init_ocp_cmd(struct iris_ocp_cmd *pocp_cmd)
{
	union iris_ocp_cmd_header ocp_header;

	if (!pocp_cmd) {
		IRIS_LOGE("%s(), invalid pocp cmd!", __func__);
		return;
	}

	memset(pocp_cmd, 0x00, sizeof(*pocp_cmd));
	ocp_header.header32 = 0xfffffff0 | OCP_SINGLE_WRITE_BYTEMASK;
	memcpy(pocp_cmd->cmd, &ocp_header.header32, OCP_HEADER);
	pocp_cmd->cmd_len = OCP_HEADER;
}

static void _iris_add_tx_cmds(struct dsi_cmd_desc *ptx_cmd,
		struct iris_ocp_cmd *pocp_cmd, u8 wait)
{
	struct dsi_cmd_desc desc_init_val = {
		{0, MIPI_DSI_GENERIC_LONG_WRITE, 0, 0, 0, CMD_PKT_SIZE, NULL, 0, NULL},
		1, 0};

	memcpy(ptx_cmd, &desc_init_val, sizeof(struct dsi_cmd_desc));
	ptx_cmd->msg.tx_buf = pocp_cmd->cmd;
	ptx_cmd->msg.tx_len = pocp_cmd->cmd_len;
	ptx_cmd->post_wait_ms = wait;
}

static u32 _iris_pt_short_write(struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader, struct dsi_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS6_CHIP_VERSION ?
		IRIS_MIPI_TX_HEADER_ADDR : IRIS_MIPI_TX_HEADER_ADDR_I3;

	pheader->stHdr.longCmdFlag = 0x00;

	_iris_pt_set_wrcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: 0x%4x", __func__, __LINE__, pheader->hdr32);

	_iris_add_cmd_addr_val(pocp_cmd, address, pheader->hdr32);

	return sum;
}

static u32 _iris_pt_short_read(struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader, struct dsi_cmd_desc *dsi_cmd)
{
	u32 sum = 1;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS6_CHIP_VERSION ?
		IRIS_MIPI_TX_HEADER_ADDR : IRIS_MIPI_TX_HEADER_ADDR_I3;

	pheader->stHdr.longCmdFlag = 0x00;
	_iris_pt_set_rdcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: 0x%4x", __func__, __LINE__, pheader->hdr32);

	_iris_add_cmd_addr_val(pocp_cmd, address, pheader->hdr32);

	return sum;
}

static u32 _iris_pt_get_split_pkt_len(u16 dlen, int sum, int k)
{
	u16 split_len = 0;

	if (k == 0)
		split_len = dlen <  IRIS_TX_HV_PAYLOAD_LEN
					? dlen : IRIS_TX_HV_PAYLOAD_LEN;
	else if (k == sum - 1)
		split_len = dlen - IRIS_TX_HV_PAYLOAD_LEN
				- (k - 1) * IRIS_TX_PAYLOAD_LEN;
	else
		split_len = IRIS_TX_PAYLOAD_LEN;

	return split_len;
}

static void _iris_pt_add_split_pkt_payload(struct iris_ocp_cmd *pocp_cmd,
		u8 *ptr, u16 split_len)
{
	u32 i = 0;
	union iris_mipi_tx_cmd_payload payload;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS6_CHIP_VERSION ?
		IRIS_MIPI_TX_PAYLOAD_ADDR : IRIS_MIPI_TX_PAYLOAD_ADDR_I3;

	memset(&payload, 0x00, sizeof(payload));
	for (i = 0; i < split_len; i += 4, ptr += 4) {
		if (i + 4 > split_len) {
			payload.pld32 = 0;
			memcpy(payload.p, ptr, split_len - i);
		} else
			payload.pld32 = *(u32 *)ptr;

		IRIS_LOGD("%s(), payload: %#x", __func__, payload.pld32);

		_iris_add_cmd_addr_val(pocp_cmd, address, payload.pld32);
	}
}

static u32 _iris_pt_long_write(struct iris_ocp_cmd *pocp_cmd,
		union iris_mipi_tx_cmd_header *pheader, struct dsi_cmd_desc *dsi_cmd)
{
	u8 *ptr = NULL;
	u32 i = 0;
	u32 sum = 0;
	u16 dlen = 0;
	u32 split_len = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS6_CHIP_VERSION ?
		IRIS_MIPI_TX_HEADER_ADDR : IRIS_MIPI_TX_HEADER_ADDR_I3;

	dlen = dsi_cmd->msg.tx_len;

	pheader->stHdr.longCmdFlag = 0x1;
	_iris_pt_set_wrcmd_hdr(pheader, dsi_cmd);

	IRIS_LOGD("%s(%d), header: %#x", __func__, __LINE__, pheader->hdr32);

	_iris_add_cmd_addr_val(pocp_cmd, address, pheader->hdr32);

	ptr = (u8 *)dsi_cmd->msg.tx_buf;
	sum = _iris_pt_get_split_pkt_cnt(dlen);

	while (i < sum) {
		ptr += split_len;
		split_len = _iris_pt_get_split_pkt_len(dlen, sum, i);
		_iris_pt_add_split_pkt_payload(pocp_cmd + i, ptr, split_len);
		i++;
		if (i < sum)
			_iris_pt_init_ocp_cmd(pocp_cmd + i);
	}

	return sum;
}

static u32 _iris_pt_add_cmd(struct dsi_cmd_desc *ptx_cmd,
		struct iris_ocp_cmd *pocp_cmd, struct dsi_cmd_desc *dsi_cmd,
		struct dsi_panel_cmd_set *cmdset)
{
	u32 i = 0;
	u16 dtype = 0;
	u32 sum = 0;
	u8 wait = 0;
	union iris_mipi_tx_cmd_header header;

	_iris_pt_init_tx_cmd_hdr(cmdset, dsi_cmd, &header);

	dtype = dsi_cmd->msg.type;
	switch (dtype) {
	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_DCS_READ:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
		sum = _iris_pt_short_read(pocp_cmd, &header, dsi_cmd);
		break;
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_DCS_COMPRESSION_MODE:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		sum = _iris_pt_short_write(pocp_cmd, &header, dsi_cmd);
		break;
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_PPS_LONG_WRITE:
		sum = _iris_pt_long_write(pocp_cmd, &header, dsi_cmd);
		break;
	default:
		IRIS_LOGE("%s(), invalid type: %#x", __func__, dsi_cmd->msg.type);
		break;
	}

	for (i = 0; i < sum; i++) {
		wait = (i == sum - 1) ? dsi_cmd->post_wait_ms : 0;
		_iris_add_tx_cmds(ptx_cmd + i, pocp_cmd + i, wait);
	}

	return sum;
}

static int _iris_pt_send_cmds(struct dsi_panel *panel,
							  struct dsi_cmd_desc *ptx_cmds, u32 cmds_cnt)
{
	struct dsi_panel_cmd_set panel_cmds;
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;
	memset(&panel_cmds, 0x00, sizeof(panel_cmds));

	panel_cmds.cmds = ptx_cmds;
	panel_cmds.count = cmds_cnt;
	panel_cmds.state = DSI_CMD_SET_STATE_HS;
	rc = iris_dsi_send_cmds(panel, panel_cmds.cmds,
							panel_cmds.count, panel_cmds.state, pcfg->vc_ctrl.to_panel_hs_vc_id);

	if (iris_get_cont_splash_type() == IRIS_CONT_SPLASH_LK)
		iris_print_desc_cmds(panel_cmds.cmds,
				panel_cmds.count, panel_cmds.state);
	return rc;
}

static int _iris_pt_write_panel_cmd(struct dsi_panel *panel,
									struct dsi_panel_cmd_set *cmdset)
{
	int i = 0;
	int j = 0;
	u32 offset = 0;
	int cmds_cnt = 0;
	int rc = 0;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct dsi_cmd_desc *ptx_cmds = NULL;
	struct dsi_cmd_desc *dsi_cmds = NULL;

	if (!panel || !cmdset) {
		IRIS_LOGE("%s(), invalid panel: %p or cmdset: %p",
			__func__, panel, cmdset);
		return -1;
	}

	if (cmdset->count == 0) {
		IRIS_LOGD("%s(%d)", __func__, __LINE__);
		return -1;
	}

	cmds_cnt = _iris_pt_alloc_cmds(cmdset, &ptx_cmds, &pocp_cmds);
	if (cmds_cnt < 0) {
		IRIS_LOGE("%s(), fail to alloc cmd buffer", __func__);
		return -1;
	}

	for (i = 0; i < cmdset->count; i++) {
		/*initial val*/
		dsi_cmds = cmdset->cmds + i;
		_iris_pt_init_ocp_cmd(pocp_cmds + j);
		offset = _iris_pt_add_cmd(ptx_cmds + j, pocp_cmds + j,
								dsi_cmds, cmdset);
		j += offset;
	}

	if (j == cmds_cnt)
		rc = _iris_pt_send_cmds(panel, ptx_cmds, cmds_cnt);
	else
		IRIS_LOGE("%s(), invalid cmds cnt: %d, j: %d", __func__, cmds_cnt, j);

	vfree(pocp_cmds);
	vfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;
	return rc;
}

static int _iris_i2c_send_dsi_cmds(struct dsi_cmd_desc *ptx_cmds, u32 cmds_cnt)
{
	int ret = 0;
	int i = 0;
	struct iris_i2c_msg *msg = NULL;
	uint32_t msg_num = 0;

	msg_num = cmds_cnt;
	msg = vmalloc(sizeof(struct iris_i2c_msg) * msg_num);
	if (msg == NULL) {
		IRIS_LOGE("%s(), failed to allocate memory", __func__);
		return -EINVAL;
	}

	for (i = 0; i < msg_num; i++) {
		msg[i].buf = (uint8_t *)ptx_cmds[i].msg.tx_buf;
		msg[i].len = ptx_cmds[i].msg.tx_len;
	}

	ret = iris_i2c_multi_write(msg, msg_num);

	vfree(msg);

	return ret;
}

static int _iris_i2c_write_panel_cmd(struct dsi_panel *panel, struct dsi_panel_cmd_set *cmdset)
{
	int i = 0;
	int j = 0;
	u32 offset = 0;
	int cmds_cnt = 0;
	int rc = 0;
	struct iris_ocp_cmd *pocp_cmds = NULL;
	struct dsi_cmd_desc *ptx_cmds = NULL;
	struct dsi_cmd_desc *dsi_cmds = NULL;

	if (!panel || !cmdset) {
		IRIS_LOGE("%s(), invalid panel: %p or cmdset: %p",
				  __func__, panel, cmdset);
		return -1;
	}

	if (cmdset->count == 0) {
		IRIS_LOGD("%s(%d)", __func__, __LINE__);
		return -1;
	}

	cmds_cnt = _iris_pt_alloc_cmds(cmdset, &ptx_cmds, &pocp_cmds);
	if (cmds_cnt < 0) {
		IRIS_LOGE("%s(), fail to alloc cmd buffer", __func__);
		return -1;
	}

	for (i = 0; i < cmdset->count; i++) {
		/*initial val*/
		dsi_cmds = cmdset->cmds + i;
		_iris_pt_init_ocp_cmd(pocp_cmds + j);
		offset = _iris_pt_add_cmd(ptx_cmds + j, pocp_cmds + j,
								  dsi_cmds, cmdset);
		j += offset;
	}

	if (j == cmds_cnt)
		rc = _iris_i2c_send_dsi_cmds(ptx_cmds, cmds_cnt);
	else
		IRIS_LOGE("%s(), invalid cmds cnt: %d, j: %d", __func__, cmds_cnt, j);

	vfree(pocp_cmds);
	vfree(ptx_cmds);
	pocp_cmds = NULL;
	ptx_cmds = NULL;
	return rc;
}

static void _iris_pt_switch_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset, struct dsi_cmd_desc *dsi_cmd)
{
	if (!panel || !cmdset || !dsi_cmd) {
		IRIS_LOGE("%s(), invalid panel: %p, or cmdset: %p, or dsi cmd: %p",
			__func__, panel, cmdset, dsi_cmd);
		return;
	}

	cmdset->cmds = dsi_cmd;
	cmdset->count = 1;

	// FIXME: support DMA_TPG
	// if ((dsi_cmd->dchdr.dlen < DMA_TPG_FIFO_LEN)
	//	&& (ctrl->shared_data->hw_rev >= MDSS_DSI_HW_REV_103))
	//	cmdreq->flags |= CMD_REQ_DMA_TPG;
}

static int _iris_pt_write_max_pkt_size(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset)
{
	u32 rlen = 0;
	int rc = 0;
	struct dsi_panel_cmd_set local_cmdset;
	static char max_pktsize[2] = {0x00, 0x00}; /* LSB tx first, 10 bytes */
	static struct dsi_cmd_desc pkt_size_cmd = {
		{0, MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, MIPI_DSI_MSG_REQ_ACK, 0, 0,
			sizeof(max_pktsize), max_pktsize, 0, NULL},
		1, 0};
	struct iris_cfg *pcfg = iris_get_cfg();

	rlen = cmdset->cmds[0].msg.rx_len;
	/*ic confirm the max read length is 256-byte in iris6*/
	if (rlen > 256) {
		IRIS_LOGE("%s(), len(%d) > 256", __func__, rlen);
		return -EINVAL;
	}

	max_pktsize[0] = (rlen & 0xFF);
	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	_iris_pt_switch_cmd(panel, &local_cmdset, &pkt_size_cmd);
	if (pcfg->read_path == PATH_I2C)
		rc = _iris_i2c_write_panel_cmd(panel, &local_cmdset);
	else
		rc = _iris_pt_write_panel_cmd(panel, &local_cmdset);

	return rc;
}

static int _iris_pt_send_panel_rdcmd(struct dsi_panel *panel,
									 struct dsi_panel_cmd_set *cmdset)
{
	struct dsi_panel_cmd_set local_cmdset;
	struct dsi_cmd_desc *dsi_cmd = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	int rc = 0;

	dsi_cmd = cmdset->cmds;

	memset(&local_cmdset, 0x00, sizeof(local_cmdset));

	_iris_pt_switch_cmd(panel, &local_cmdset, dsi_cmd);

	/*passthrough write to panel*/
	if (pcfg->read_path == PATH_I2C)
		rc = _iris_i2c_write_panel_cmd(panel, &local_cmdset);
	else
		rc = _iris_pt_write_panel_cmd(panel, &local_cmdset);
	return rc;
}

static int _iris_pt_remove_respond_hdr(char *ptr, int *offset)
{
	int rc = 0;
	char cmd;

	if (!ptr)
		return -EINVAL;

	cmd = ptr[0];
	switch (cmd) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		IRIS_LOGD("%s(), rx ACK_ERR_REPORT", __func__);
		rc = -EINVAL;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		*offset = 1;
		rc = 1;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		*offset = 1;
		rc = 2;
		break;
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		*offset = 4;
		rc = ptr[1];
		break;
	default:
		rc = 0;
	}

	return rc;
}

static int _iris_pt_read(struct dsi_panel_cmd_set *cmdset, uint8_t path)
{
	u32 i = 0;
	u32 rlen = 0;
	u32 intstat = 0;
	int retry_cnt = 10;
	u32 offset = 0;
	int rc = 0;
	union iris_mipi_tx_cmd_payload val;
	u8 *rbuf = NULL;
	struct iris_cfg *pcfg = iris_get_cfg();
	u32 address = pcfg->chip_ver == IRIS6_CHIP_VERSION ?
		IRIS_RD_PACKET_DATA : IRIS_RD_PACKET_DATA_I3;

	rbuf = (u8 *)cmdset->cmds[0].msg.rx_buf;
	rlen = cmdset->cmds[0].msg.rx_len;

	if (!rbuf || rlen <= 0) {
		IRIS_LOGE("%s(), buf: %p len: %d", __func__, rbuf, rlen);
		return rc;
	}

	/*read iris for data*/
	for (i = 0; i < retry_cnt; i++) {
		udelay(1000 * (i + 1));
		if (path == PATH_I2C) {
			rc = iris_ioctl_i2c_read(IRIS_TX_INTSTAT_RAW, &intstat);
			if (rc < 0) {
				IRIS_LOGE("%s(), i2c failed to read 0x%x, rc: %d",
						  __func__, IRIS_TX_INTSTAT_RAW, rc);
				return rc;
			}
		} else {
			intstat = iris_ocp_read(IRIS_TX_INTSTAT_RAW, cmdset->state);
		}

		if (intstat & IRIS_TX_READ_RESPONSE_RECEIVED)
			break;
		if ((iris_esd_ctrl_get() & 0x8) || IRIS_IF_LOGD())
			IRIS_LOGI("%s retry: %d", __func__, (i + 1));
	}

	if (intstat & IRIS_TX_READ_ERR_MASK) {
		rc = -1;
		IRIS_LOGE("%s(), Tx read error 0x%x, rc: %d",
				  __func__, intstat, rc);
		return rc;
	}
	if (path == PATH_I2C) {
		rc = iris_ioctl_i2c_read(address, &val.pld32);
		if (rc < 0) {
			IRIS_LOGE("%s(), i2c failed to read 0x%x, rc: %d",
						__func__, address, rc);
			return rc;
		}
		iris_ioctl_i2c_write(IRIS_TX_INTCLR, 0xFFFFFFFF);
	} else {
		val.pld32 = iris_ocp_read(address, cmdset->state);
		iris_ocp_write_val(IRIS_TX_INTCLR, 0xFFFFFFFF);
	}

	rlen = _iris_pt_remove_respond_hdr(val.p, &offset);
	if (rlen <= 0) {
		rc = -1;
		IRIS_LOGE("%s(), failed to remove respond header, val: 0x%x, rlen: %d, rc: %d",
				  __func__, val.pld32, rlen, rc);
		return rc;
	}

	if (rlen > 2) {
		int j = 0;
		int len = 0;
		int num = (rlen + 3) / 4;

		for (i = 0; i < num; i++) {
			len = (i == num - 1) ? rlen - 4 * i : 4;
			if (path == PATH_I2C)
				rc = iris_ioctl_i2c_read(address, &val.pld32);
			else
				val.pld32 = iris_ocp_read(address, DSI_CMD_SET_STATE_HS);
			for (j = 0; j < len; j++)
				rbuf[i * 4 + j] = val.p[j];
		}
	} else {
		for (i = 0; i < rlen; i++)
			rbuf[i] = val.p[offset + i];
	}

	return rc;
}

int iris_pt_read_panel_cmd(struct dsi_panel *panel,
						   struct dsi_panel_cmd_set *cmdset)
{
	u8 vc_id = 0;
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	IRIS_LOGD("%s()", __func__);

	if (!panel || !cmdset || cmdset->count != 1) {
		IRIS_LOGE("%s(), invalid input param", __func__);
		return -EINVAL;
	}

	if (!pcfg->vc_ctrl.vc_enable) {
		/*step1  write max packet size*/
		rc = _iris_pt_write_max_pkt_size(panel, cmdset);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			return rc;
		}

		/*step2 write read cmd to panel*/
		rc = _iris_pt_send_panel_rdcmd(panel, cmdset);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			return rc;
		}

		/*step3 read panel data*/
		rc = _iris_pt_read(cmdset, pcfg->read_path);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			return rc;
		}
	} else {
		vc_id = (cmdset->state == DSI_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
		rc = iris_dsi_send_cmds(panel, cmdset->cmds,
								cmdset->count, DSI_CMD_SET_STATE_HS, vc_id);
		if (rc < 0) {
			IRIS_LOGI("%s %d rc:%d", __func__, __LINE__, rc);
			return rc;
		}
	}

	return rc;
}

int iris_pt_send_panel_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset)
{
	u8 vc_id = 0;
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	if (!cmdset || !panel) {
		IRIS_LOGE("%s(), invalid panel: %p or cmdset: %p",
			__func__, panel, cmdset);
		return -EINVAL;
	}

	if (cmdset->count == 1 && cmdset->cmds[0].msg.type == MIPI_DSI_DCS_READ) {
		rc = iris_pt_read_panel_cmd(panel, cmdset);
		return rc;
	}

	if (!pcfg->vc_ctrl.vc_enable) {
		_iris_pt_write_panel_cmd(panel, cmdset);
	} else {
		vc_id = (cmdset->state == DSI_CMD_SET_STATE_LP) ?
			pcfg->vc_ctrl.to_panel_lp_vc_id :
			pcfg->vc_ctrl.to_panel_hs_vc_id;
		iris_dsi_send_cmds(panel, cmdset->cmds,
			cmdset->count, DSI_CMD_SET_STATE_HS, vc_id);
	}

	rc = iris_dsirecover_check(IRIS_PQUPDATE_OP);
	return rc;
}

int iris_abyp_send_panel_cmd(struct dsi_panel *panel,
		struct dsi_panel_cmd_set *cmdset)
{
	u8 vc_id = 0;

	if (!cmdset || !panel) {
		IRIS_LOGE("%s(), invalid panel: %p or cmdset: %p",
			__func__, panel, cmdset);
		return -EINVAL;
	}

	if (cmdset->count)
		vc_id = cmdset->cmds[0].msg.channel;
	iris_dsi_send_cmds(panel, cmdset->cmds,
			cmdset->count, cmdset->state, vc_id);
	return 0;
}


int iris_platform_get(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	return pcfg->platform_type;
}
