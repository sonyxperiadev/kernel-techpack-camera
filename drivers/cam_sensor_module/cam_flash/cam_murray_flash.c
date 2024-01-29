// SPDX-License-Identifier: GPL-2.0-only
/**
 * cam_murray_flash.c - Murray camera flash driver
 * Copyright (C) 2024 Pavel Dubrova <pashadubrova@gmail.com>
 */

#define DEBUG

#include <linux/pwm.h>
#include <linux/power/sm5038_charger.h>

#include "cam_flash_core.h"
#include "cam_res_mgr_api.h"
#include "cam_murray_flash.h"

enum {
	DEVICE_MURRAY,
	DEVICE_ZAMBEZI
};

struct cam_murray_flash_dev {
	struct pwm_device *pwm;
	int8_t mode;
	int8_t device;
};

struct cam_murray_flash_desc {
	bool gpio_enabled;

	bool pwm_enabled;
	uint64_t pwm_period;
	uint64_t pwm_duty_cycle;
};

static struct cam_murray_flash_dev *static_cam_murray_flash_dev;

static int cam_murray_flash_set_pwm_state(struct cam_murray_flash_dev *flash,
		struct cam_murray_flash_desc desc)
{
	struct pwm_state pstate;
	int rc = -EINVAL;

	if (flash == NULL) {
		CAM_ERR(CAM_FLASH, "Flash device is invalid");
		return rc;
	}

	pwm_get_state(flash->pwm, &pstate);

	pstate.period = desc.pwm_period;
	pstate.duty_cycle = desc.pwm_duty_cycle;
	pstate.enabled = desc.pwm_enabled;

	rc = pwm_apply_state(flash->pwm, &pstate);
	if (rc < 0)
		CAM_ERR(CAM_FLASH, "Failed to configure PWM: %d\n", rc);

	return rc;
}

static struct cam_murray_flash_desc murray_flash_config(int mode)
{
	struct cam_murray_flash_desc desc;

	switch (mode) {
	default:
	case CAMERA_SENSOR_FLASH_OP_OFF:
		sm5038_fled_mode_ctrl(SM5038_FLED_MODE_OFF, 0);
		sm5038_fled_mode_ctrl(SM5038_FLED_MODE_CLOSE_FLASH, 0);

		desc.gpio_enabled = false;
		desc.pwm_enabled = false;
		desc.pwm_period = 500000;
		desc.pwm_duty_cycle = 500000;
		break;
	case CAMERA_SENSOR_FLASH_OP_FIRELOW:
		sm5038_fled_mode_ctrl(SM5038_FLED_MODE_PREPARE_FLASH, 0);
		sm5038_fled_mode_ctrl(SM5038_FLED_MODE_TORCH_FLASH, 100);

		desc.gpio_enabled = false;
		desc.pwm_enabled = true;
		desc.pwm_period = 500000;
		desc.pwm_duty_cycle = 500000;
		break;
	case CAMERA_SENSOR_FLASH_OP_FIREHIGH:
		sm5038_fled_mode_ctrl(SM5038_FLED_MODE_PREPARE_FLASH, 0);
		sm5038_fled_mode_ctrl(SM5038_FLED_MODE_MAIN_FLASH, 800);

		desc.gpio_enabled = true;
		desc.pwm_enabled = false;
		desc.pwm_period = 500000;
		desc.pwm_duty_cycle = 500000;
		break;
	}

	return desc;
}

static struct cam_murray_flash_desc zambezi_flash_config(int mode)
{
	struct cam_murray_flash_desc desc;

	switch (mode) {
	default:
	case CAMERA_SENSOR_FLASH_OP_OFF:
		desc.gpio_enabled = false;
		desc.pwm_enabled = false;
		desc.pwm_period = 10000;
		desc.pwm_duty_cycle = 10000;
		break;
	case CAMERA_SENSOR_FLASH_OP_FIRELOW:
		desc.gpio_enabled = false;
		desc.pwm_enabled = true;
		desc.pwm_period = 100;
		desc.pwm_duty_cycle = 100;
		break;
	case CAMERA_SENSOR_FLASH_OP_FIREHIGH:
		desc.gpio_enabled = true;
		desc.pwm_enabled = true;
		desc.pwm_period = 10000;
		desc.pwm_duty_cycle = 10000;
		break;
	}

	return desc;
}

int cam_murray_flash_set_mode(struct cam_flash_ctrl *fctrl, int mode)
{
	struct cam_murray_flash_dev *flash = static_cam_murray_flash_dev;
	struct cam_murray_flash_desc desc;
	struct gpio *gpio_tbl = fctrl->soc_info.gpio_data->cam_gpio_req_tbl;
	uint8_t size = fctrl->soc_info.gpio_data->cam_gpio_req_tbl_size;
	int i, rc = -EINVAL;

	if (flash == NULL) {
		CAM_ERR(CAM_FLASH, "Flash device is invalid");
		goto end;
	}

	desc = (flash->device == DEVICE_MURRAY) ?
			murray_flash_config(mode) : zambezi_flash_config(mode);

	rc = cam_murray_flash_set_pwm_state(flash, desc);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "Failed to set PWM state for flash mode: %d", rc);
		goto end;
	}

	rc = cam_sensor_util_request_gpio_table(&fctrl->soc_info, 1);
	if (rc < 0) {
		CAM_ERR(CAM_FLASH, "Request gpio table fails");
		goto end;
	}

	for (i = 0; i < size; i++) {
		CAM_DBG(CAM_FLASH, "Set gpio %s: %s", gpio_tbl[i].label,
				desc.gpio_enabled ? "enable" : "disable");
		cam_res_mgr_gpio_set_value(gpio_tbl[i].gpio, desc.gpio_enabled);
	}

	cam_sensor_util_request_gpio_table(&fctrl->soc_info, 0);

end:
	return rc;
}
EXPORT_SYMBOL_GPL(cam_murray_flash_set_mode);

static ssize_t enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cam_murray_flash_dev *flash = static_cam_murray_flash_dev;
	int rc = -EINVAL;

	if (flash == NULL) {
		CAM_ERR(CAM_FLASH, "Flash device is invalid");
		return rc;
	}

	if (flash->mode == CAMERA_SENSOR_FLASH_OP_OFF)
		rc = sprintf(buf, "%s\n", "off");
	else if (flash->mode == CAMERA_SENSOR_FLASH_OP_FIRELOW)
		rc = sprintf(buf, "%s\n", "torch");
	else if (flash->mode == CAMERA_SENSOR_FLASH_OP_FIREHIGH)
		rc = sprintf(buf, "%s\n", "flash");

	return rc;
}

static ssize_t enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cam_murray_flash_dev *flash = static_cam_murray_flash_dev;
	struct cam_flash_ctrl *fctrl = dev_get_drvdata(dev);
	ssize_t rc = -EINVAL;
	char mode[6];

	if (flash == NULL) {
		CAM_ERR(CAM_FLASH, "Flash device is invalid");
		return rc;
	}

	rc = sscanf(buf, "%s", mode);
	if (rc != 1)
		return rc;

	if (!strncmp(mode, "off", 3))
		flash->mode = CAMERA_SENSOR_FLASH_OP_OFF;
	else if (!strncmp(mode, "torch", 5))
		flash->mode = CAMERA_SENSOR_FLASH_OP_FIRELOW;
	else if (!strncmp(mode, "flash", 5))
		flash->mode = CAMERA_SENSOR_FLASH_OP_FIREHIGH;
	else
		return rc;

	/* First of all, reset the previous state */
	cam_murray_flash_set_mode(fctrl, CAMERA_SENSOR_FLASH_OP_OFF);
	usleep_range(5000, 6000);

	cam_murray_flash_set_mode(fctrl, flash->mode);

	CAM_INFO(CAM_FLASH, "Mode %s triggered", mode);

	return size;
}

static DEVICE_ATTR_RW(enable);

static struct device_attribute *cam_murray_flash_attr[] = {
	&dev_attr_enable,
};

int cam_murray_flash_control_create_device(struct device* dev)
{
	int i, rc = 0;

	if (static_cam_murray_flash_dev == NULL)
		return 0;

	for (i = 0; i < ARRAY_SIZE(cam_murray_flash_attr); i++) {
		rc = device_create_file(dev, cam_murray_flash_attr[i]);
		if (rc) {
			CAM_ERR(CAM_FLASH, "failed: sysfs file %s",
					cam_murray_flash_attr[i]->attr.name);
			cam_murray_flash_control_remove_device(dev);
			return rc;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cam_murray_flash_control_create_device);

int cam_murray_flash_control_remove_device(struct device* dev)
{
	int i;

	if (static_cam_murray_flash_dev == NULL)
		return 0;

	for (i = 0; i < ARRAY_SIZE(cam_murray_flash_attr); i++) {
		device_remove_file(dev, cam_murray_flash_attr[i]);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cam_murray_flash_control_remove_device);

static const struct of_device_id cam_murray_flash_dt_match[] = {
	{
		.compatible = "somc,murray_flash",
		.data = (void *)(uintptr_t) DEVICE_MURRAY,
	},
	{
		.compatible = "somc,zambezi_flash",
		.data = (void *)(uintptr_t) DEVICE_ZAMBEZI,
	},
	{}
};

static int cam_murray_flash_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct cam_murray_flash_dev *flash;
	int rc = 0;

	flash = devm_kzalloc(dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	match = of_match_node(cam_murray_flash_dt_match, dev->of_node);
	if (!match) {
		CAM_ERR(CAM_FLASH, "could not find compatible string match");
		return -ENODEV;
	}

	flash->device = (uintptr_t)match->data;
	flash->mode = CAMERA_SENSOR_FLASH_OP_OFF;
	flash->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(flash->pwm))
		return PTR_ERR(flash->pwm);

	static_cam_murray_flash_dev = flash;
	platform_set_drvdata(pdev, flash);

	return rc;
}

static struct platform_driver cam_murray_flash_platform_driver = {
	.probe = cam_murray_flash_probe,
	.driver = {
		.name = "CAM-MURRAY-FLASH-DRIVER",
		.owner = THIS_MODULE,
		.of_match_table = cam_murray_flash_dt_match,
	},
};

int cam_murray_flash_init_module(void)
{
	int rc = -EINVAL;

	rc = platform_driver_register(&cam_murray_flash_platform_driver);
	if (rc < 0)
		CAM_ERR(CAM_FLASH, "platform probe failed rc: %d", rc);

	return rc;
}
EXPORT_SYMBOL_GPL(cam_murray_flash_init_module);

void cam_murray_flash_exit_module(void)
{
	platform_driver_unregister(&cam_murray_flash_platform_driver);
}
EXPORT_SYMBOL_GPL(cam_murray_flash_exit_module);

MODULE_AUTHOR("Pavel Dubrova <pashadubrova@gmail.com>");
MODULE_DESCRIPTION("CAM MURRAY FLASH");
MODULE_LICENSE("GPL");
