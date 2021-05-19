#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include "dsi_panel.h"
#include "dsi_iris6_log.h"
#include "dsi_iris6_def.h"
#include "dsi_iris6_gpio.h"
#include "dsi_iris6_lightup.h"


#define IRIS_GPIO_HIGH 1
#define IRIS_GPIO_LOW  0
#define IRIS_POR_CLOCK 180   /* 0.1MHz */

static int gpio_pulse_delay = 16 * 16 * 4 * 10 / IRIS_POR_CLOCK;
static int gpio_cmd_delay = 10;

int iris_enable_pinctrl(void *dev)
{
	int rc = 0;
	struct platform_device *pdev = dev;
	struct iris_cfg *pcfg = iris_get_cfg();

	pcfg->pinctrl.pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(pcfg->pinctrl.pinctrl)) {
		rc = PTR_ERR(pcfg->pinctrl.pinctrl);
		IRIS_LOGE("%s(), failed to get pinctrl, return: %d",
				__func__, rc);
		return -EINVAL;
	}

	pcfg->pinctrl.active = pinctrl_lookup_state(pcfg->pinctrl.pinctrl,
			"iris_active");
	if (IS_ERR_OR_NULL(pcfg->pinctrl.active)) {
		rc = PTR_ERR(pcfg->pinctrl.active);
		IRIS_LOGE("%s(), failed to get pinctrl active state, return: %d",
				__func__, rc);
		return -EINVAL;
	}

	pcfg->pinctrl.suspend = pinctrl_lookup_state(pcfg->pinctrl.pinctrl,
			"iris_suspend");
	if (IS_ERR_OR_NULL(pcfg->pinctrl.suspend)) {
		rc = PTR_ERR(pcfg->pinctrl.suspend);
		IRIS_LOGE("%s(), failed to get pinctrl suspend state, retrun: %d",
				__func__, rc);
		return -EINVAL;
	}

	return 0;
}

int iris_set_pinctrl_state(bool enable)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();
	struct pinctrl_state *state;

	IRIS_LOGI("%s(), set state: %s", __func__,
			enable?"active":"suspend");
	if (enable)
		state = pcfg->pinctrl.active;
	else
		state = pcfg->pinctrl.suspend;

	rc = pinctrl_select_state(pcfg->pinctrl.pinctrl, state);
	if (rc)
		IRIS_LOGE("%s(), failed to set pin state %d, return: %d",
				__func__, enable, rc);

	return rc;
}

int iris_parse_gpio(void *dev)
{
	struct platform_device *pdev = dev;
	struct device_node *of_node = pdev->dev.of_node;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s()", __func__);

	pcfg->iris_wakeup_gpio = of_get_named_gpio(of_node,
			"qcom,iris-wakeup-gpio", 0);
	IRIS_LOGI("%s(), abyp gpio %d", __func__,
			pcfg->iris_wakeup_gpio);
	if (!gpio_is_valid(pcfg->iris_wakeup_gpio))
		IRIS_LOGW("%s(), abyp gpio not specified", __func__);

	pcfg->iris_abyp_ready_gpio = of_get_named_gpio(of_node,
			"qcom,iris-abyp-ready-gpio", 0);

	IRIS_LOGI("%s(), abyp status gpio %d", __func__,
			pcfg->iris_abyp_ready_gpio);
	if (!gpio_is_valid(pcfg->iris_abyp_ready_gpio))
		IRIS_LOGW("%s(), abyp status gpio not specified", __func__);

	pcfg->iris_reset_gpio = of_get_named_gpio(of_node,
			"qcom,iris-reset-gpio", 0);

	IRIS_LOGI("%s(), iris reset gpio %d", __func__,
			pcfg->iris_reset_gpio);

	if (!gpio_is_valid(pcfg->iris_reset_gpio))
		IRIS_LOGW("%s(), iris reset gpio not specified", __func__);

	pcfg->iris_vdd_on_gpio = of_get_named_gpio(of_node,
			"qcom,iris-vdd-gpio", 0);

	IRIS_LOGI("%s(), iris vdd on gpio %d", __func__,
			pcfg->iris_vdd_on_gpio);

	if (!gpio_is_valid(pcfg->iris_vdd_on_gpio))
		IRIS_LOGW("%s(), iris vdd on gpio not specified", __func__);

	return 0;
}

int iris_request_gpio(void)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	IRIS_LOGI("%s()", __func__);

	if (gpio_is_valid(pcfg->iris_wakeup_gpio)) {
		rc = gpio_request(pcfg->iris_wakeup_gpio, "analog_bypass");
		if (rc != 0) {
			IRIS_LOGW("%s(), request for iris abyp gpio failed, return %d",
					__func__, rc);
		}
	}

	if (gpio_is_valid(pcfg->iris_abyp_ready_gpio)) {
		rc = gpio_request(pcfg->iris_abyp_ready_gpio,
				"analog_bypass_status");
		if (rc != 0) {
			IRIS_LOGE("%s(), request iris abyp status gpio failed, return %d",
					__func__, rc);
		}
	}

	if (gpio_is_valid(pcfg->iris_reset_gpio)) {
		rc = gpio_request(pcfg->iris_reset_gpio, "iris_reset");
		if (rc != 0) {
			IRIS_LOGE("%s(), request for iris reset gpio failed, return %d",
					__func__, rc);
		}
	}

	if (gpio_is_valid(pcfg->iris_vdd_on_gpio)) {
		rc = gpio_request(pcfg->iris_vdd_on_gpio, "iris_vdd_on");
		if (rc != 0) {
			IRIS_LOGE("%s(), request for iris vdd on gpio failed, return %d",
					__func__, rc);
		}
	}

	return 0;
}

int iris_release_gpio(void *cfg)
{
	struct iris_cfg *pcfg = cfg;

	IRIS_LOGI("%s()", __func__);

	if (gpio_is_valid(pcfg->iris_wakeup_gpio))
		gpio_free(pcfg->iris_wakeup_gpio);
	if (gpio_is_valid(pcfg->iris_abyp_ready_gpio))
		gpio_free(pcfg->iris_abyp_ready_gpio);
	if (gpio_is_valid(pcfg->iris_reset_gpio))
		gpio_free(pcfg->iris_reset_gpio);
	if (gpio_is_valid(pcfg->iris_vdd_on_gpio))
		gpio_free(pcfg->iris_vdd_on_gpio);

	return 0;
}

int iris_reset(void)
{
	int rc = 0;
	struct iris_cfg *pcfg = iris_get_cfg();

	if (gpio_is_valid(pcfg->iris_reset_gpio)) {
		rc = gpio_direction_output(pcfg->iris_reset_gpio, IRIS_GPIO_LOW);
		if (rc != 0) {
			IRIS_LOGE("%s(), unable to set iris reset gpio, return %d",
					__func__, rc);
			return rc;
		}
	}

	gpio_set_value(pcfg->iris_reset_gpio, IRIS_GPIO_LOW);
	usleep_range(2000, 2001);
	gpio_set_value(pcfg->iris_reset_gpio, IRIS_GPIO_HIGH);
	usleep_range(3*1000, 3*1001);

	return 0;
}

int iris_clk_enable(bool enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->iris_clk != NULL) {
		IRIS_LOGI("%s() %s", __func__, enable?"enable":"disable");
		if (enable)
			clk_prepare_enable(pcfg->iris_clk);
		else
			clk_disable_unprepare(pcfg->iris_clk);
	} else 
		IRIS_LOGE("%s(), iris_clk is NULL", __func__);

	return 0;
}

int iris_vdd_enable(int enable)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (gpio_is_valid(pcfg->iris_vdd_on_gpio)) {
		if (enable)
			gpio_set_value(pcfg->iris_vdd_on_gpio, 1);
		else
			gpio_set_value(pcfg->iris_vdd_on_gpio, 0);
	}

	return 0;
}

int iris_reset_off(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (gpio_is_valid(pcfg->iris_reset_gpio))
		gpio_set_value(pcfg->iris_reset_gpio, IRIS_GPIO_LOW);

	iris_clk_enable(false);
	iris_vdd_enable(false);

	return 0;
}


int iris_init_one_wired(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int one_wired_status_gpio = 0;
	int one_wired_gpio = pcfg->iris_wakeup_gpio;

	IRIS_LOGI("%s()", __func__);

	if (!gpio_is_valid(one_wired_gpio)) {
		pcfg->abyp_ctrl.abyp_disable = true;

		IRIS_LOGE("%s(), one wired GPIO not configured", __func__);
		return 0;
	}

	gpio_direction_output(one_wired_gpio, 0);

	one_wired_status_gpio = pcfg->iris_abyp_ready_gpio;
	if (!gpio_is_valid(one_wired_status_gpio)) {
		IRIS_LOGE("%s(), ABYP status GPIO not configured.", __func__);
		return 0;
	}

	gpio_direction_input(one_wired_status_gpio);

	return 0;
}

void iris_send_one_wired_cmd(IRIS_ONE_WIRE_TYPE type)
{
	int cnt = 0;
	u32 start_end_delay = 0;
	u32 pulse_delay = 0;
	unsigned long flags;
	struct iris_cfg *pcfg = iris_get_cfg();
	int one_wired_gpio = pcfg->iris_wakeup_gpio;
	const int pulse_count[IRIS_ONE_WIRE_CMD_CNT] = {
		1, 2, 3, 4, 5, 6, 7, 8, 9,
	};

	if (!gpio_is_valid(one_wired_gpio)) {
		IRIS_LOGE("%s(), one wired GPIO not configured", __func__);
		return;
	}

	start_end_delay = 16 * 16 * 16 * 10 / IRIS_POR_CLOCK;  /* us */
	pulse_delay = gpio_pulse_delay;  /* us */

	IRIS_LOGI("%s(), pulse_count: %d, pulse delay: %d, gpio cmd delay: %d",
			  __func__, pulse_count[type], pulse_delay, gpio_cmd_delay);

	spin_lock_irqsave(&pcfg->iris_1w_lock, flags);
	for (cnt = 0; cnt < pulse_count[type]; cnt++) {
		gpio_set_value(one_wired_gpio, IRIS_GPIO_HIGH);
		udelay(pulse_delay);
		gpio_set_value(one_wired_gpio, IRIS_GPIO_LOW);
		udelay(pulse_delay);
	}

	udelay(gpio_cmd_delay);
	spin_unlock_irqrestore(&pcfg->iris_1w_lock, flags);

	udelay(start_end_delay);
}

int iris_check_abyp_ready(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();
	int iris_abyp_ready_gpio = 0;

	if (!gpio_is_valid(pcfg->iris_abyp_ready_gpio)) {
		IRIS_LOGE("%s(), ABYP status GPIO is not configured.", __func__);
		return -EINVAL;
	}

	iris_abyp_ready_gpio = gpio_get_value(pcfg->iris_abyp_ready_gpio);

	IRIS_LOGD("%s(), ABYP status %d.", __func__, iris_abyp_ready_gpio);

	return iris_abyp_ready_gpio;
}

int iris_dbg_init_gpio(void)
{
	struct iris_cfg *pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir("iris", NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("%s(), create debug dir for iris failed, error %ld",
					__func__, PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	debugfs_create_u32("pulse_delay", 0644, pcfg->dbg_root,
			(u32 *)&gpio_pulse_delay);
	debugfs_create_u32("cmd_delay", 0644, pcfg->dbg_root,
			(u32 *)&gpio_cmd_delay);

	return 0;
}
