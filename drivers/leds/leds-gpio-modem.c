/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is disributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <soc/qcom/sysmon.h>
#include <linux/pinctrl/consumer.h>

struct BLINK_LED_data{
	int blink_flag;
	int blink_on_time;  //ms
	int blink_off_time;  //ms
	struct led_classdev led;
};

struct STATUS_LED_data {
	struct mutex data_lock;
	struct BLINK_LED_data blink_led[3];  /* red, green, trickle*/
	int red_led;
	int green_led;
	int trickle_red_led;
	int gpio_base;
	int led_suspend_flag;
};

static struct of_device_id gpio_led_of_match[] = {
	{.compatible = "gpio-modem-leds",},
	{},
};

struct STATUS_LED_data *STATUS_LED = NULL;

static int send_info = 0;
enum {
	RED_ON,
	RED_OFF,
	RED_BLINK_ON,
	RED_BLINK_OFF,
	GREEN_ON,
	GREEN_OFF,
	GREEN_BLINK_ON,
	GREEN_BLINK_OFF,
};
static char* led_cmd[] = {
	[RED_ON]			= "leds:red:on",
	[RED_OFF]			= "leds:red:off",
	[RED_BLINK_ON]		= "leds:red:blink_on",
	[RED_BLINK_OFF]		= "leds:red:blink_off",
	[GREEN_ON]			= "leds:green:on",
	[GREEN_OFF]			= "leds:green:off",
	[GREEN_BLINK_ON]	= "leds:green:blink_on",
	[GREEN_BLINK_OFF]	= "leds:green:blink_off",
};

static void gpio_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	if (!send_info) {
		char cmd[50];
		int rc;
		
		snprintf(cmd, sizeof(cmd),"leds:info:%d:%d:%d:%d",
				STATUS_LED->red_led - STATUS_LED->gpio_base,STATUS_LED->green_led - STATUS_LED->gpio_base,
				STATUS_LED->blink_led[0].blink_on_time, 
				STATUS_LED->blink_led[0].blink_off_time);
		rc = sysmon_send_led_cmd(SYSMON_SS_MODEM, cmd);
		if (!rc)
			send_info = 1;
		else {
			dev_info(led_cdev->dev, "led send info error in %s, %s\n", __func__, led_cdev->name);
			return;
		}
	}

	if (!strcmp(led_cdev->name, "red")) 
	{
		if(value == LED_OFF)
		{
			sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[RED_OFF]);
		} else {
			sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[RED_ON]);
		}
	} 
	else if(!strcmp(led_cdev->name, "green"))//green
	{
		if(value == LED_OFF)
		{
			sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[GREEN_OFF]);
		} else {
			sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[GREEN_ON]);
		}		
	}
	else //trickle
	{
		if(value == LED_OFF)
                {
			gpio_direction_output(STATUS_LED->trickle_red_led, 0);
                } else {
			gpio_direction_output(STATUS_LED->trickle_red_led, 1);
                }

	}
}


static ssize_t led_blink_solid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "green"))
		idx = 1;
	else
		idx = 2;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

	/* no lock needed for this */
	sprintf(buf, "%u\n", STATUS_LED->blink_led[idx].blink_flag);
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t led_blink_solid_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct STATUS_LED_data *STATUS_LED;
	int idx = 1;
	char *after;
	unsigned long state;
	ssize_t ret = -EINVAL;
	size_t count;

	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (!strcmp(led_cdev->name, "red"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "green"))
		idx = 1;
	else
		idx = 2;

	STATUS_LED = container_of(led_cdev, struct STATUS_LED_data, blink_led[idx].led);

	if (!send_info) {
		char cmd[50];
		int rc;

		snprintf(cmd, sizeof(cmd),"leds:info:%d:%d:%d:%d",
				STATUS_LED->red_led - STATUS_LED->gpio_base,STATUS_LED->green_led - STATUS_LED->gpio_base,
				STATUS_LED->blink_led[idx].blink_on_time, 
				STATUS_LED->blink_led[idx].blink_off_time);
		rc = sysmon_send_led_cmd(SYSMON_SS_MODEM, cmd);
		if (!rc)
			send_info = 1;
		else {
			dev_info(dev, "led send info error in %s\n", __func__);
			return ret;
		}
	}
	state = simple_strtoul(buf, &after, 10);

	count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		mutex_lock(&STATUS_LED->data_lock);
		if(0 == state)
		{
			if (idx == 0)
				sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[RED_BLINK_OFF]);
			else if(idx == 1)
				sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[GREEN_BLINK_OFF]);
			else
				printk("trickle led not blink 1\n");
			STATUS_LED->blink_led[idx].blink_flag= 0;
		}
		else
		{
			if (idx == 0)
				sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[RED_BLINK_ON]);
			else if(idx == 1)
				sysmon_send_led_cmd(SYSMON_SS_MODEM, led_cmd[GREEN_BLINK_ON]);
			else
				printk("trickle led not blink 2\n");
			STATUS_LED->blink_led[idx].blink_flag= 1;

		}
		mutex_unlock(&STATUS_LED->data_lock);
	}

	return ret;
}

static DEVICE_ATTR(blink, 0644, led_blink_solid_show, led_blink_solid_store);

static int led_parse_dt(struct platform_device *pdev)
{
	u32 temp_val;
	int rc;
	char *key;

	key = "qcom,red-led";
	rc = of_get_named_gpio(pdev->dev.of_node, key, 0);
	if (rc < 0)
		goto parse_error;
	STATUS_LED->red_led = rc;

	key = "qcom,green-led";
	rc = of_get_named_gpio(pdev->dev.of_node, key, 0);
	if (rc < 0)
		goto parse_error;
	STATUS_LED->green_led = rc;

	key = "qcom,trickle_red-led";
	rc = of_get_named_gpio(pdev->dev.of_node, key, 0);
	if (rc < 0)
		goto parse_error;
	STATUS_LED->trickle_red_led = rc;

	key = "qcom,blink-on-time";
	rc = of_property_read_u32(pdev->dev.of_node, key, &temp_val);
	if (rc && (rc != -EINVAL)) 
		goto parse_error;
	else if(rc) {
		dev_warn(&pdev->dev,"led have no property %s, use default\n", key);
		STATUS_LED->blink_led[0].blink_on_time = 500;
		STATUS_LED->blink_led[1].blink_on_time = 500;			
		STATUS_LED->blink_led[2].blink_on_time = 500;			
	} else {
		STATUS_LED->blink_led[0].blink_on_time = temp_val;
		STATUS_LED->blink_led[1].blink_on_time = temp_val;
		STATUS_LED->blink_led[2].blink_on_time = temp_val;
	}

	key = "qcom,blink-off-time";
	rc = of_property_read_u32(pdev->dev.of_node, key, &temp_val);
	if (rc && (rc != -EINVAL)) 
		goto parse_error;
	else if(rc) {
		dev_warn(&pdev->dev,"led have no property %s, use default\n", key);
		STATUS_LED->blink_led[0].blink_off_time = 500;
		STATUS_LED->blink_led[1].blink_off_time = 500;			
		STATUS_LED->blink_led[2].blink_off_time = 500;			
	} else {
		STATUS_LED->blink_led[0].blink_off_time = temp_val;
		STATUS_LED->blink_led[1].blink_off_time = temp_val;
		STATUS_LED->blink_led[2].blink_off_time = temp_val;
	}

	return 0;
parse_error:
	dev_err(&pdev->dev,"parse property %s failed, rc = %d\n", key, rc);
	return rc;
}

static int led_gpio_config(struct platform_device *pdev)
{
	int rc;

	rc = gpio_request(STATUS_LED->red_led, "red_led");
	if (rc) {
		dev_err(&pdev->dev, "Failed to request gpio %d,rc = %d\n", STATUS_LED->red_led, rc);
		return rc;
	}

	rc = gpio_request(STATUS_LED->green_led, "green_led");
	if (rc) {
		dev_err(&pdev->dev,"Failed to request gpio %d,rc = %d\n", STATUS_LED->green_led, rc);
		gpio_free(STATUS_LED->red_led);
		return rc;
	}

	rc = gpio_request(STATUS_LED->trickle_red_led, "trickle_red_led");
	if (rc) {
		dev_err(&pdev->dev,"Failed to request gpio %d,rc = %d\n", STATUS_LED->green_led, rc);
		gpio_free(STATUS_LED->red_led);
		return rc;
	}
	gpio_direction_output(STATUS_LED->trickle_red_led, 1);
	return 0;
}

static void led_gpio_unconfig(void)
{
	gpio_free(STATUS_LED->trickle_red_led);
	gpio_free(STATUS_LED->green_led);
	gpio_free(STATUS_LED->red_led);
}

static int gpio_led_probe(struct platform_device *pdev)
{
	int rc = 0;
	int i, j;
	struct pinctrl *pinctrl;

	if (pdev->dev.of_node == NULL) {
		dev_info(&pdev->dev, "can not find device tree node\n");
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_warn(&pdev->dev, "pins are not configured from the driver\n");
		return -EINVAL;
	}

	STATUS_LED = kzalloc(sizeof(struct STATUS_LED_data), GFP_KERNEL);
	if (STATUS_LED == NULL) {
		dev_err(&pdev->dev,"no memory for device\n");
		return -ENOMEM;
	}

	rc = led_parse_dt(pdev);
	if (rc) {
		goto err_alloc_failed;
	}

	rc = led_gpio_config(pdev);
	if (rc) {
		dev_warn(&pdev->dev,"config led gpio failed\n");
	}
	STATUS_LED->gpio_base = gpio_to_chip(STATUS_LED->red_led)->base;

	STATUS_LED->blink_led[0].led.name = "red";
	STATUS_LED->blink_led[0].led.brightness_set = gpio_led_set;
	STATUS_LED->blink_led[0].led.brightness = LED_OFF;
	STATUS_LED->blink_led[0].blink_flag = 0;

	STATUS_LED->blink_led[1].led.name = "green";
	STATUS_LED->blink_led[1].led.brightness_set = gpio_led_set;
	STATUS_LED->blink_led[1].led.brightness = LED_OFF;
	STATUS_LED->blink_led[1].blink_flag = 0;

	STATUS_LED->blink_led[2].led.name = "trickle_red";
	STATUS_LED->blink_led[2].led.brightness_set = gpio_led_set;
	STATUS_LED->blink_led[2].led.brightness = LED_OFF;
	STATUS_LED->blink_led[2].blink_flag = 0;

	mutex_init(&STATUS_LED->data_lock);

	for (i = 0; i < 3; i++) {	/* red, green, trickle*/
		rc= led_classdev_register(&pdev->dev, &STATUS_LED->blink_led[i].led);
		if (rc) {
			dev_err(&pdev->dev,"STATUS_LED: led_classdev_register failed\n");
			goto err_led_classdev_register_failed;
		}
	}

	for (i = 0; i < 3; i++) {
		rc = device_create_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
		if (rc) {
			dev_err(&pdev->dev,"STATUS_LED: create dev_attr_blink failed\n");
			goto err_out_attr_blink;
		}
	}

	dev_set_drvdata(&pdev->dev, STATUS_LED);

	printk(KERN_NOTICE "PM_DEBUG_MXP:Exit gpio_led_probe.\r\n");		
	return 0;

err_out_attr_blink:
	for (j = 0; j < i; j++)
		device_remove_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
	i = 3;
err_led_classdev_register_failed:
	for (j = 0; j < i; j++)
		led_classdev_unregister(&STATUS_LED->blink_led[i].led);
	led_gpio_unconfig();
err_alloc_failed:
	kfree(STATUS_LED);

	return rc;

}

static int gpio_led_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < 3; i++){
		device_remove_file(STATUS_LED->blink_led[i].led.dev, &dev_attr_blink);
		led_classdev_unregister(&STATUS_LED->blink_led[i].led);
	}
	led_gpio_unconfig();
	kfree(STATUS_LED);
	return 0;
}

static int gpio_led_suspend(struct device *dev)
{
	STATUS_LED->led_suspend_flag = 1;
	return 0;
}

static int gpio_led_resume(struct device *dev)
{
	STATUS_LED->led_suspend_flag = 0;
	return 0;
}

static SIMPLE_DEV_PM_OPS(led_pm_ops, gpio_led_suspend, gpio_led_resume);

static struct platform_driver gpio_led_driver = {
	.probe		= gpio_led_probe,
	.remove		= gpio_led_remove,
	.driver		= {
		.name	= "zte,leds",
		.owner	= THIS_MODULE,
		.of_match_table = gpio_led_of_match,
		.pm = &led_pm_ops,
	},
};

static int __init gpio_led_init(void)
{
	return platform_driver_register(&gpio_led_driver);
}

static void __exit gpio_led_exit(void)
{
	platform_driver_unregister(&gpio_led_driver);
}

late_initcall(gpio_led_init);
module_exit(gpio_led_exit);
