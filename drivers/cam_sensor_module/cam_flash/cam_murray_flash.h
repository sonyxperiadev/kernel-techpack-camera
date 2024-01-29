// SPDX-License-Identifier: GPL-2.0-only
/**
 * cam_murray_flash.h - Murray camera flash driver
 * Copyright (C) 2024 Pavel Dubrova <pashadubrova@gmail.com>
 */

#ifndef _CAM_MURRAY_FLASH_H_
#define _CAM_MURRAY_FLASH_H_

int cam_murray_flash_set_mode(struct cam_flash_ctrl *fctrl, int mode);

int cam_murray_flash_control_create_device(struct device *dev);
int cam_murray_flash_control_remove_device(struct device *dev);

int cam_murray_flash_init_module(void);
void cam_murray_flash_exit_module(void);

#endif /* _CAM_MURRAY_FLASH_H_ */
