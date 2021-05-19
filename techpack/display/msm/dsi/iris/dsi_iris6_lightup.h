#ifndef _DSI_IRIS_LIGHTUP_H_
#define _DSI_IRIS_LIGHTUP_H_

#include <linux/completion.h>
#include "dsi_iris6.h"
#include "dsi_iris6_def.h"


//#define IRIS_ABYP_LIGHTUP
//#define IRIS_MIPI_TEST
#define IRIS_CHIP_CNT   2

/* iris ip option, it will create according to opt_id.
 * link_state will be create according to the last cmds
 */
struct iris_ip_opt {
	uint8_t opt_id; /*option identifier*/
	uint8_t link_state; /*high speed or low power*/
	uint32_t cmd_cnt; /*option length*/
	struct dsi_cmd_desc *cmd; /*the first cmd of desc*/
};

/*ip search index*/
struct iris_ip_index {
	int32_t opt_cnt; /*ip option number*/
	struct iris_ip_opt *opt; /*option array*/
};

struct iris_pq_ipopt_val {
	int32_t opt_cnt;
	uint8_t ip;
	uint8_t *popt;
};

struct iris_pq_init_val {
	int32_t ip_cnt;
	struct iris_pq_ipopt_val *val;
};

/*used to control iris_ctrl opt sequence*/
struct iris_ctrl_opt {
	uint8_t ip;
	uint8_t opt_id;
	uint8_t chain;
};

struct iris_ctrl_seq {
	int32_t cnt;
	struct iris_ctrl_opt *ctrl_opt;
};

//will pack all the commands here
struct iris_out_cmds {
	/* will be used before cmds sent out */
	struct dsi_cmd_desc *iris_cmds_buf;
	u32 cmds_index;
};

struct iris_pq_update_cmd {
	struct iris_update_ipopt *update_ipopt_array;
	u32 array_index;
};

struct iris_i2c_cfg {
	uint8_t *buf;
	uint32_t buf_index;
};

typedef int (*iris_i2c_read_cb)(u32 reg_addr, u32 *reg_val);
typedef int (*iris_i2c_write_cb)(u32 reg_addr, u32 reg_val);
typedef int (*iris_i2c_burst_write_cb)(u32 start_addr, u32 *lut_buffer, u16 reg_num);

enum IRIS_STATUS_VALID {
	PARAM_NONE = 0,
	PARAM_EMPTY,
	PARAM_PARSED,
	FULL_LIGHTUP,
};


enum iris_op_type {
	IRIS_LIGHTUP_OP = 0,
	IRIS_PQUPDATE_OP,
	IRIS_ESDCHECK_OP,
};

enum iris_dsirecover_check {
	CHECK_NONE = 0,
	CHECK_ONLY_READ = 1,
	CHECK_WRITE_AND_READ = 2,
};

/* iris commands specific to panel timing mode */
struct iris_timing_cfg {
	uint32_t lut_cmds_cnt;
	uint32_t dtsi_cmds_cnt;
	uint32_t ip_opt_cnt;
	/*one for lut and none_lut ip index arr*/
	struct iris_ip_index ip_index_arr[IRIS_PIP_IDX_CNT][IRIS_IP_CNT];
	struct iris_ctrl_seq ctrl_seq_pre;
	struct iris_ctrl_seq ctrl_seq[IRIS_CHIP_CNT];
	struct iris_ctrl_seq ctrl_seq_cs[IRIS_CHIP_CNT];
	struct iris_ctrl_seq mode_switch_seq;
	struct iris_pq_init_val pq_init_val;
	struct iris_pq_update_cmd pq_update_cmd;
};

#define IRIS_MIN_TIMING_MODES 1
#define IRIS_MAX_TIMING_MODES 4

/*iris lightup configure commands*/
struct iris_cfg {
	struct platform_device *pdev;
	struct {
		struct pinctrl *pinctrl;
		struct pinctrl_state *active;
		struct pinctrl_state *suspend;
	} pinctrl;
	int iris_reset_gpio;
	int iris_wakeup_gpio;
	int iris_abyp_ready_gpio;
	int iris_vdd_on_gpio;
	struct clk *iris_clk;

	struct dsi_display *display;
	struct dsi_panel *panel;

	/* hardware version and initialization status */
	uint8_t chip_id;
	uint32_t chip_ver;
	bool iris_chip_enable;
	bool iris_soft_enable;
	bool iris_default_mode_pt;
	uint8_t valid; /* 0: none, 1: empty, 2: parse ok */
	uint32_t platform_type; /* 0: FPGA, 1~: ASIC */

	/* static configuration */
	uint8_t panel_type;
	uint8_t lut_mode;
	uint32_t add_last_flag;
	uint32_t split_pkt_size;
	bool output_iris_scaling; // output with iris, scaling 480x854 to 1080x1920
	uint32_t min_color_temp;
	uint32_t max_color_temp;
	uint32_t P3_color_temp;
	uint32_t sRGB_color_temp;
	uint32_t vivid_color_temp;
	uint32_t hdr_color_temp;
	u8 rx_mode; /* 0: DSI_VIDEO_MODE, 1: DSI_CMD_MODE */
	u8 tx_mode;
	uint8_t read_path; /* 0: DSI, 1: I2C */

	/* current state */
	struct iris_lp_ctrl lp_ctrl;
	struct iris_abyp_ctrl abyp_ctrl;
	struct work_struct abyp_check_work;
	struct iris_vc_ctrl vc_ctrl;
	int dpp_only_enable; /* 0: disable, 1: enable */
	uint16_t panel_nits;
	uint32_t panel_dimming_brightness;
	uint8_t panel_hbm[2];
	uint8_t power_mode;
	uint32_t cur_h_active;
	uint32_t cur_v_active;
	uint32_t cur_timing;
	uint32_t prev_timing;

	/* configuration commands, parsed from dt, dynamically modified;
	 * panel->panel_lock must be locked before access and for DSI command send
	 */
	int timing_cnt;
	struct iris_timing_cfg timing[IRIS_MAX_TIMING_MODES];
	uint32_t iris_cmds_cnt;
	struct iris_out_cmds iris_cmds;

	/* one wire gpio lock */
	spinlock_t iris_1w_lock;
	struct dentry *dbg_root;
	struct work_struct cont_splash_work;
	struct completion frame_ready_completion;

	/* hook for i2c extension */
	iris_i2c_read_cb iris_i2c_read;
	iris_i2c_write_cb iris_i2c_write;
	iris_i2c_burst_write_cb iris_i2c_burst_write;
	struct iris_i2c_cfg iris_i2c_cfg;

	/* lightup and pq path sel */
	enum iris_path_sel light_up_path;
	enum iris_path_sel pq_update_path;
	enum iris_path_sel single_ipopt_path;
	uint8_t path_backup_need_restore;
	enum iris_path_sel light_up_path_backup;
	enum iris_path_sel pq_update_path_backup;
	enum iris_path_sel single_ipopt_path_backup;
	enum iris_dsirecover_check dsirecover_check_method_backup;

	/*dsi send mode select*/
	uint8_t dsi_embedded_enable[2];
	uint8_t *non_embedded_buf;
	/*lightup pqupdate*/
	uint32_t non_embedded_xfer_len[2];
	int dc_bl_pending;
	uint8_t dimming_first;
	uint32_t pend_dimming[5];
	uint32_t pend_dimmingEnable;
	uint32_t pend_brightness;
	/*mipi WA debug_enable*/
	enum iris_dsirecover_check dsirecover_check_method;
	uint8_t dsirecover_check_path; /* 0: DSI, 1: I2C */
	uint8_t pq_update_is_dsi_hs; /* true: hs, false: lp */
	bool dport_is_disable;
	uint32_t frame_kickoff_count[3]; /*0: kick count, 1: dport kick off count, 2: esd kick off count*/
	uint8_t cont_splash_status;
};

struct iris_data {
	const uint8_t *buf;
	uint32_t size;
};


struct iris_cfg *iris_get_cfg(void);

int iris_lightup(struct dsi_panel *panel, struct dsi_panel_cmd_set *on_cmds);
int iris_lightoff(struct dsi_panel *panel, struct dsi_panel_cmd_set *off_cmds);
int32_t iris_send_ipopt_cmds(int32_t ip, int32_t opt_id);
void iris_update_pq_opt(bool bcommit);
void iris_update_bitmask_regval(
				struct iris_update_regval *pregval, bool is_commit);
void iris_update_bitmask_regval_nonread(
				struct iris_update_regval *pregval, bool is_commit);

void iris_alloc_seq_space(void);

void iris_init_update_ipopt(struct iris_update_ipopt *popt,
		uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain);
struct iris_pq_ipopt_val *iris_get_cur_ipopt_val(uint8_t ip);

int iris_init_update_ipopt_t(uint8_t ip, uint8_t opt_old, uint8_t opt_new, uint8_t chain);
struct iris_ip_opt *iris_find_ip_opt(uint8_t ip, uint8_t opt_id);

/*
 * @description  get assigned position data of ip opt
 * @param ip       ip sign
 * @param opt_id   option id of ip
 * @param pos      the position of option payload
 * @return   fail NULL/success payload data of position
 */
uint32_t *iris_get_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos);
uint32_t iris_get_ipopt_payload_len(uint8_t ip, uint8_t opt_id, int32_t pos);
void iris_set_ipopt_payload_data(uint8_t ip, uint8_t opt_id, int32_t pos, uint32_t value);

/*
 * @Description: get current continue splash stage
 *				first light up panel only
 *				second pq effect
 */
uint8_t iris_get_cont_splash_type(void);

/*
 * @Description: print continuous splash commands for bootloader
 * @param: cmds: cmds array  cmd_cnt: cmds count
 */
void iris_print_desc_cmds(struct dsi_cmd_desc *cmds, int cmd_cnt, int state);

/*
 * @Description: set Iris sys, send preload data from AP to Iris, set power, etc
 * it will be called in PT mode only held the panel_lock when use DSI
 * @param: void
 * @return: 0 is sucess, others error
 */
int iris_preload(void);

void iris_read_power_mode(struct dsi_panel *panel);

int iris_init_cmds(void);
void iris_get_cmds(struct dsi_panel_cmd_set *cmds, char **ls_arr);
void iris_get_lightoff_cmds(struct dsi_panel_cmd_set *cmds, char **ls_arr);

int32_t iris_attach_cmd_to_ipidx(const struct iris_data *data,
		int32_t data_cnt, struct iris_ip_index *pip_index);

struct iris_ip_index *iris_get_ip_idx(int32_t type);

void iris_change_type_addr(struct iris_ip_opt *dest, struct iris_ip_opt *src);

struct iris_ip_opt *iris_find_ip_opt(uint8_t ip, uint8_t opt_id);
struct iris_ip_opt *iris_find_ip_opt_mask(uint8_t ip, uint8_t opt_id, uint8_t opt_mask);
void iris_free_ipopt_buf(uint32_t ip_type);
void iris_free_seq_space(void);
int iris_wait_vsync(uint32_t count);
void _iris_rx_bypass_en(void);
int iris_dsirecover_check(enum iris_op_type op_type);
void iris_send_mode_switch_pkt(void);

int iris_driver_register(void);
void iris_driver_unregister(void);
#endif // _DSI_IRIS_LIGHTUP_H_
