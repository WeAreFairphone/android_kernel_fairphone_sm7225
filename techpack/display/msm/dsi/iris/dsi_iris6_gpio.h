#ifndef __DSI_IRIS_GPIO__
#define __DSI_IRIS_GPIO__


typedef enum {
	IRIS_POWER_UP_SYS,
	IRIS_ENTER_ANALOG_BYPASS,
	IRIS_EXIT_ANALOG_BYPASS,
	IRIS_POWER_DOWN_SYS,
	IRIS_RESET_SYS,
	IRIS_FORCE_ENTER_ANALOG_BYPASS,
	IRIS_FORCE_EXIT_ANALOG_BYPASS,
	IRIS_POWER_UP_MIPI,
	IRIS_POWER_DOWN_MIPI,
	IRIS_ONE_WIRE_CMD_CNT
} IRIS_ONE_WIRE_TYPE;

int iris_init_one_wired(void);
void iris_send_one_wired_cmd(IRIS_ONE_WIRE_TYPE type);
int iris_check_abyp_ready(void);
int iris_parse_gpio(void *pdev);
int iris_release_gpio(void *cfg);
int iris_enable_pinctrl(void *dev);
int iris_set_pinctrl_state(bool enable);

int iris_dbg_init_gpio(void);

#endif // __DSI_IRIS_GPIO__
