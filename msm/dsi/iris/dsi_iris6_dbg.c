#include "dsi_iris6_api.h"
#include "dsi_iris6_lightup.h"
#include "dsi_iris6_lightup_ocp.h"
#include "dsi_iris6_pq.h"
#include "dsi_iris6_ioctl.h"
#include "dsi_iris6_lut.h"
#include "dsi_iris6_log.h"

#define IRIS_DBG_TOP_DIR "iris"
#define IRIS_DBG_FUNCSTATUS_FILE "iris_func_status"

int iris_dbg_fstatus_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;
	return 0;
}

/**
 * read module's status
 */
static ssize_t iris_dbg_fstatus_read(struct file *file, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	char *kbuf = NULL;
	int size = count < PAGE_SIZE ? PAGE_SIZE : (int)count;
	int len = 0;
	struct iris_setting_info *iris_setting = iris_get_setting();
	struct quality_setting *pqlt_cur_setting = &iris_setting->quality_cur;

	if (*ppos)
		return 0;

	kbuf = vzalloc(size);
	if (kbuf == NULL) {
		IRIS_LOGE("%s(), fatal erorr: No mem!", __func__);
		return -ENOMEM;
	}

	len += snprintf(kbuf + len, size - len,
		"***system_brightness***\n"
		"%-20s:\t%d\n",
		"system_brightness",
		pqlt_cur_setting->system_brightness);

	len += snprintf(kbuf + len, size - len,
		"***dspp_dirty***\n"
		"%-20s:\t%d\n",
		"dspp_dirty", pqlt_cur_setting->dspp_dirty);

	len += snprintf(kbuf + len, size - len,
		"***dbc_setting***\n"
		"%-20s:\t%d\n",
		"dbc", pqlt_cur_setting->pq_setting.dbc);

	len += snprintf(kbuf + len, size - len,
		"***cm_setting***\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n",
		"cmcolortempmode",
		pqlt_cur_setting->pq_setting.cmcolortempmode,
		"colortempvalue", pqlt_cur_setting->colortempvalue,
		"min_colortempvalue",
		pqlt_cur_setting->min_colortempvalue,
		"max_colortempvalue",
		pqlt_cur_setting->max_colortempvalue,
		"cmcolorgamut",
		pqlt_cur_setting->pq_setting.cmcolorgamut,
		"demomode", pqlt_cur_setting->pq_setting.demomode,
		"source_switch", pqlt_cur_setting->source_switch);

	len += snprintf(kbuf + len, size - len,
		"***lux_value***\n"
		"%-20s:\t%d\n",
		"luxvalue", pqlt_cur_setting->luxvalue);

	len += snprintf(kbuf + len, size - len,
		"***cct_value***\n"
		"%-20s:\t%d\n",
		"cctvalue", pqlt_cur_setting->cctvalue);

	len += snprintf(kbuf + len, size - len,
		"***reading_mode***\n"
		"%-20s:\t%d\n",
		"readingmode", pqlt_cur_setting->pq_setting.readingmode);

	len += snprintf(kbuf + len, size - len,
		"***ambient_lut***\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n",
		"al_en", pqlt_cur_setting->pq_setting.alenable,
		"al_luxvalue", pqlt_cur_setting->luxvalue,
		"al_bl_ratio", pqlt_cur_setting->al_bl_ratio);

	len += snprintf(kbuf + len, size - len,
		"***sdr2hdr***\n"
		"%-20s:\t%d\n"
		"%-20s:\t%d\n",
		"sdr2hdr", pqlt_cur_setting->pq_setting.sdr2hdr,
		"maxcll", pqlt_cur_setting->maxcll);

	len += snprintf(kbuf + len, size - len,
		"***analog_abypass***\n"
		"%-20s:\t%d\n",
		"abyp_mode", iris_get_abyp_mode_nonblocking());

	size = len;
	if (len >= count)
		size = count - 1;

	if (copy_to_user(ubuf, kbuf, size)) {
		vfree(kbuf);
		return -EFAULT;
	}

	vfree(kbuf);
	*ppos += size;

	return size;
}


static const struct file_operations iris_dbg_fstatus_fops = {
	.open = iris_dbg_fstatus_open,
	.read = iris_dbg_fstatus_read,
};

int iris_dbg_init_fstatus(struct dsi_display *display)
{
	struct iris_cfg *pcfg = NULL;

	pcfg = iris_get_cfg();

	if (pcfg->dbg_root == NULL) {
		pcfg->dbg_root = debugfs_create_dir(IRIS_DBG_TOP_DIR, NULL);
		if (IS_ERR_OR_NULL(pcfg->dbg_root)) {
			IRIS_LOGE("create dir for iris failed, error %ld",
				  PTR_ERR(pcfg->dbg_root));
			return -ENODEV;
		}
	}

	if (debugfs_create_file(IRIS_DBG_FUNCSTATUS_FILE, 0644,
				pcfg->dbg_root, display,
				&iris_dbg_fstatus_fops) == NULL)
		IRIS_LOGE("create file func_status failed\n");

	return 0;
}

