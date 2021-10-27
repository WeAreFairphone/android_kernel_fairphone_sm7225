#ifndef _DSI_IRIS_IOCTL_H_
#define _DSI_IRIS_IOCTL_H_

int iris_configure(u32 type, u32 value);
int iris_configure_ex(u32 type, u32 count, u32 *values);
int iris_configure_get(u32 type, u32 count, u32 *values);
int iris_dbg_init_adb_type(struct dsi_display *display);

#endif // _DSI_IRIS_IOCTL_H_
