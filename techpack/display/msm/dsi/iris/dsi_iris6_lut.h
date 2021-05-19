#ifndef _DSI_IRIS_LUT_H_
#define _DSI_IRIS_LUT_H_


#define LUT_LEN 256
#define CM_LUT_GROUP 3 // table 0,3,6 should store at the same address in iris
#define CRSTK_COEF_SIZE        (18*3*2)  //crstk0_coef + crstk1_coef (uint16) * 3 for each group
#define CRSTK_COEF_GROUP        7
#define CCT_VALUE_SIZE          (3*2) //warm, standard & cool
enum LUT_TYPE {
	LUT_IP_START = 128, /*0x80*/
	DBC_LUT = LUT_IP_START,
	CM_LUT,
	ADJ_LUT,
	GAMMA_LUT,
	SCALER1D_PP_LUT,
	COL_ENH_LUT,
	SDR2HDR_LUT,
	AMBINET_HDR_GAIN, /*HDR case*/
	AMBINET_SDR2HDR_LUT, /*SDR2HDR case;*/
	DPP_DITHER_LUT,
	SCALER1D_LUT,
	DTG_PHASE_LUT,
	MISC_INFO_LUT,
	LUT_IP_END
};

enum DBC_LEVEL {
	DBC_INIT = 0,
	DBC_OFF,
	DBC_LOW,
	DBC_MIDDLE,
	DBC_HIGH,
	CABC_DLV_OFF = 0xF1,
	CABC_DLV_LOW,
	CABC_DLV_MIDDLE,
	CABC_DLV_HIGH,
};

enum SDR2HDR_LEVEL {
	SDR2HDR_LEVEL0 = 0,
	SDR2HDR_LEVEL1,
	SDR2HDR_LEVEL2,
	SDR2HDR_LEVEL3,
	SDR2HDR_LEVEL4,
	SDR2HDR_LEVEL5,
	SDR2HDR_LEVEL_CNT
};

enum SDR2HDR_TABLE_TYPE {
	SDR2HDR_LUT0 = 0,
	SDR2HDR_LUT1,
	SDR2HDR_LUT2,
	SDR2HDR_LUT3,
	SDR2HDR_UVY0,
	SDR2HDR_UVY1,
	SDR2HDR_UVY2,
	SDR2HDR_INV_UV0,
	SDR2HDR_INV_UV1,
};

enum LUT_MODE {
	INTERPOLATION_MODE = 0,
	SINGLE_MODE,
};

enum SCALER_IP_TYPE {
	SCALER_INPUT = 0,
	SCALER_PP,
};

enum FIRMWARE_STATUS {
	FIRMWARE_LOAD_FAIL,
	FIRMWARE_LOAD_SUCCESS,
	FIRMWARE_IN_USING,
};

int iris_parse_lut_cmds(void);
int iris_send_lut(u8 lut_type, u8 table_index, u32 ab_table);
void iris_update_ambient_lut(enum LUT_TYPE lut_type, u32 lut_pos);
u8 iris_get_firmware_status(void);
void iris_update_fw_status(u8 value);
int iris_dbg_init_fw_calibrate_status(void);
void iris_crst_coef_check(const u8 *fw_data, size_t fw_size);
u8 iris_get_firmware_aplstatus_value(void);
#endif // _DSI_IRIS_LUT_H_
