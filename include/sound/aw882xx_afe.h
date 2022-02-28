
#ifndef __AW882xx_AFE_H__
#define __AW882xx_AFE_H__

struct aw_afe_func {
	int (*afe_get_topology)(int port_id);
	int (*aw_send_afe_cal_apr)(uint32_t param_id,void *buf, int cmd_size, bool write);
	int (*aw_send_afe_rx_module_enable)(void *buf, int size);
	int (*aw_send_afe_tx_module_enable)(void *buf, int size);
	int (*aw_adm_param_enable)(int port_id, int module_id,  int param_id, int enable);
};

void aw_reg_fae_func(struct aw_afe_func * fn);
#endif
