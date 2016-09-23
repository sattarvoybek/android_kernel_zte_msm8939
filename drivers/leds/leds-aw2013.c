/*
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
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/printk.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sensor_power.h>

#define RSTR_REG 	0x00
#define GCR_REG 	0x01
#define ISR_REG 	0x02
#define LCTR_REG 	0x30
#define LCFG0_REG 	0x31
#define LCFG1_REG 	0x32
#define LCFG2_REG 	0x33
#define PWM0_REG 	0x34
#define PWM1_REG 	0x35
#define PWM2_REG 	0x36
#define LED0T0_REG 	0x37
#define LED0T1_REG 	0x38
#define LED0T2_REG 	0x39
#define LED1T0_REG 	0x3a
#define LED1T1_REG 	0x3b
#define LED1T2_REG 	0x3c
#define LED2T0_REG 	0x3d
#define LED2T1_REG 	0x3e
#define LED2T2_REG 	0x3f
#define IADR_REG 	0x77

#define LEDO_ENA_MSK	0x01
#define LED1_ENA_MSK	0x02
#define LED2_ENA_MSK	0x04

#define CURRENT_0	0X00
#define CURRENT_5MA	0X01
#define CURRENT_10MA	0X02
#define CURRENT_15MA	0X03
#define PWM_DIRECT	0x00
#define ONE_PROGRAM	0x10

#define PWM_MAX 	255

typedef enum {
	T0,
	T1,
	T2,
	T3,
	T4,
} time_flag;

struct breathing_data{
	struct led_classdev led;
	int breathing_flag;
	int t1;  //ms
	int t2;  //ms
	int t3;  //ms
	int t4;  //ms
	int delay_to_start;  //ms
	int repeat;  //0~15,0 means forever
};

struct aw2013_led_data {
	struct mutex data_lock;
	struct breathing_data breathing_dev[3];
	int irq_gpio;
	struct i2c_client * client;
	struct pinctrl *p;
	struct pinctrl_state *int_state;
	struct pinctrl_state *en_state;
};

static struct aw2013_led_data *aw2013_led = NULL;
static int chipid = 0xff;

static int breathing_time[] = {130, 260, 520, 1040, 2080, 4160, 8320, 16640};

static void led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	int err;
	int val;
	int idx;

	if (!strcmp(led_cdev->name, "right")) 
		idx = 0;
	else if (!strcmp(led_cdev->name, "middle")) 
		idx = 1;
	else if (!strcmp(led_cdev->name, "left")) 
		idx = 2;

	mutex_lock(&aw2013_led->data_lock);
	if(value == LED_OFF) {
		val = i2c_smbus_read_byte_data(aw2013_led->client, LCTR_REG);
		if(val < 0) {
			dev_err(&aw2013_led->client->dev, "read LCTR failed in set led off");
			mutex_unlock(&aw2013_led->data_lock);
			return;
		} else {
			val = val & (~(LEDO_ENA_MSK << idx));
			err = i2c_smbus_write_byte_data(aw2013_led->client, LCTR_REG, val);
		}

		if ((val & (LEDO_ENA_MSK | LED1_ENA_MSK | LED2_ENA_MSK)) == 0)
			err = i2c_smbus_write_byte_data(aw2013_led->client, GCR_REG, 0x0);

	} else {
		err = i2c_smbus_write_byte_data(aw2013_led->client, GCR_REG, 0x1);
		if (!err)
			err = i2c_smbus_write_byte_data(aw2013_led->client, LCFG0_REG + idx, PWM_DIRECT | CURRENT_10MA);
		if (!err)
			err = i2c_smbus_write_byte_data(aw2013_led->client, PWM0_REG + idx, value);
		if (!err){
			val = i2c_smbus_read_byte_data(aw2013_led->client, LCTR_REG);
			if(val < 0) {
				dev_err(&aw2013_led->client->dev, "read LCTR failed in set led on");
				mutex_unlock(&aw2013_led->data_lock);
				return;
			} else {
				err = i2c_smbus_write_byte_data(aw2013_led->client, LCTR_REG, val | (LEDO_ENA_MSK << idx));
			}
		}
	}
	mutex_unlock(&aw2013_led->data_lock);

	if(err)
		dev_err(&aw2013_led->client->dev, "write reg failed in %s", __func__);
	return;
}

static int aw2013_start_breathing(int led)
{
	int err;
	int val;

	if ((led != 0) && (led != 1) && (led != 2)) {
		dev_err(&aw2013_led->client->dev, "invalid led number in start breathing");
		return -EINVAL;
	}
	err = i2c_smbus_write_byte_data(aw2013_led->client, GCR_REG, 0x1);
	if (!err)
		err = i2c_smbus_write_byte_data(aw2013_led->client, LCFG0_REG + led, ONE_PROGRAM | CURRENT_5MA);
	if (!err)
		err = i2c_smbus_write_byte_data(aw2013_led->client, PWM0_REG + led, 30);
	if (!err)
		err = i2c_smbus_write_byte_data(aw2013_led->client, LED0T0_REG + led*3, aw2013_led->breathing_dev[led].t2 | (aw2013_led->breathing_dev[led].t1 << 4));
	if (!err)
		err = i2c_smbus_write_byte_data(aw2013_led->client, LED0T1_REG + led*3, aw2013_led->breathing_dev[led].t4 | (aw2013_led->breathing_dev[led].t3 << 4));
	if (!err)
		err = i2c_smbus_write_byte_data(aw2013_led->client, LED0T2_REG + led*3, 0x0);
	if (!err){
		val = i2c_smbus_read_byte_data(aw2013_led->client, LCTR_REG);
		if(val < 0) {
			dev_err(&aw2013_led->client->dev, "read LCTR failed in start breathing");
			return val;
		} else {
			err = i2c_smbus_write_byte_data(aw2013_led->client, LCTR_REG, val | (LEDO_ENA_MSK << led));
		}
	}
	if(err)
		dev_err(&aw2013_led->client->dev, "write reg failed in %s", __func__);
	return err;

}

static int aw2013_stop_breathing(int led)
{
	int err;
	int val;

	if ((led !=0) && (led!=1) && (led !=2)) {
		dev_err(&aw2013_led->client->dev, "invalid led number in stop breathing");
		return -EINVAL;
	}

	val = i2c_smbus_read_byte_data(aw2013_led->client, LCTR_REG);
	if(val < 0) {
		dev_err(&aw2013_led->client->dev, "read LCTR failed in stop breathing");
		return val;
	} else {
		val = val & (~(LEDO_ENA_MSK << led));
		err = i2c_smbus_write_byte_data(aw2013_led->client, LCTR_REG, val);
	} 	

	if ((val & (LEDO_ENA_MSK | LED1_ENA_MSK | LED2_ENA_MSK)) == 0)
		err = i2c_smbus_write_byte_data(aw2013_led->client, GCR_REG, 0x0);
	if (err)
		dev_err(&aw2013_led->client->dev, "write reg failed in %s", __func__);
	return err;
		
}

static int get_time_reg(int time_ms, time_flag flag)
{
	int i;
	int ret;

	if (time_ms < 0)
		return -1;
	for (i = 0; i < ARRAY_SIZE(breathing_time); i++)
		if(time_ms <= breathing_time[i])
			break;
	if (i == ARRAY_SIZE(breathing_time))
		i--;
	
	switch (flag) {
	case T0:
		if (time_ms == 0)
			ret = 0;
		else
			ret = i++;
		break;
	case T2:
		ret = i > 5 ? 5 : i;
		break;
	case T1:
	case T3:
	case T4:
		ret = i;
		break;
	default:
		printk("aw2013: error param in %s\n", __func__);
		ret = -2;
	}

	return ret;
}

static int get_time_ms(int time_reg)
{
	int i;
	int ret = 1;

	for (i = 0; i < time_reg; i++)
		ret *=2;
	return (ret * 130);
}

static ssize_t aw2013_breathing_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int idx = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = 0;

	if (!strcmp(led_cdev->name, "right"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "middle"))
		idx = 1;
	else
		idx = 2;

	/* no lock needed for this */
	sprintf(buf, "flag:%u time:%d:%d:%d:%d\n", aw2013_led->breathing_dev[idx].breathing_flag, 
							get_time_ms(aw2013_led->breathing_dev[idx].t1), 
							get_time_ms(aw2013_led->breathing_dev[idx].t2), 
							get_time_ms(aw2013_led->breathing_dev[idx].t3), 
							get_time_ms(aw2013_led->breathing_dev[idx].t4));
	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t aw2013_breathing_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int idx = 0;
	ssize_t ret = -EINVAL;
	int t1, t2, t3, t4;

	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (!strcmp(led_cdev->name, "right"))
		idx = 0;
	else if (!strcmp(led_cdev->name, "middle"))
		idx = 1;
	else
		idx = 2;

	ret = sscanf(buf, "%d:%d:%d:%d", &t1, &t2, &t3, &t4);

	mutex_lock(&aw2013_led->data_lock);
	if ((t1 >0) && (t2 >0) && (t3 >0) && (t4 >0)){
		aw2013_led->breathing_dev[idx].t1 = get_time_reg(t1, T1);
		aw2013_led->breathing_dev[idx].t2 = get_time_reg(t2, T2);
		aw2013_led->breathing_dev[idx].t3 = get_time_reg(t3, T3);
		aw2013_led->breathing_dev[idx].t4 = get_time_reg(t4, T4);
		printk("aw2013 set param %d %d %d %d %d\n", idx, t1, t2, t3, t4);
		aw2013_start_breathing(idx);
		aw2013_led->breathing_dev[idx].breathing_flag = 1;
	} else if ((t1 == 0) || (t2 == 0) || (t3 == 0) || (t4 == 0)){
		aw2013_stop_breathing(idx);
		aw2013_led->breathing_dev[idx].breathing_flag = 0;
	
	} else {
		printk("aw2013, invalid parameter %d %d %d %d %d, nothing to do\n", idx, t1, t2, t3, t4);
	}
	mutex_unlock(&aw2013_led->data_lock);

	return  size;
}

static DEVICE_ATTR(breathing, 0640, aw2013_breathing_show, aw2013_breathing_store);

static int led_parse_dt(struct device *dev)
{
	int rc;
	char *key;

	key = "gpios";
	rc = of_get_gpio(dev->of_node, 0);
	aw2013_led->irq_gpio = rc;

	return 0;
}

static int led_gpio_config(struct device *dev)
{
	int rc;

	if (gpio_is_valid(aw2013_led->irq_gpio)) {
		rc = gpio_request(aw2013_led->irq_gpio, "aw2013_int");
		if (rc) {
			dev_err(dev, "Failed to request gpio %d,rc = %d", aw2013_led->irq_gpio, rc);
			return rc;
		}
		gpio_direction_input(aw2013_led->irq_gpio);
	} else
		dev_warn(dev, "invalid gpio%d, no need to config", aw2013_led->irq_gpio);
	return 0;
}

static void led_gpio_unconfig(void)
{
	if (gpio_is_valid(aw2013_led->irq_gpio))
		gpio_free(aw2013_led->irq_gpio);
}

static int aw2013_pinctrl_select(struct device *dev)
{
        int ret;

        aw2013_led->p = devm_pinctrl_get(dev);
        if (IS_ERR_OR_NULL(aw2013_led->p)) {
		dev_err(dev, "can't get pinctrl");
                return PTR_ERR(aw2013_led->p);
	}
/*
        aw2013_led->int_state = pinctrl_lookup_state(aw2013_led->p, "breathing_int");
        if (IS_ERR_OR_NULL(aw2013_led->int_state)) {
		dev_err(dev, "fail look pinctl int state");
                devm_pinctrl_put(aw2013_led->p);
                return PTR_ERR(aw2013_led->int_state);
        }

        ret = pinctrl_select_state(aw2013_led->p, aw2013_led->int_state);
        if (ret < 0) {
		dev_err(dev, "fail select pinctl int state");
                devm_pinctrl_put(aw2013_led->p);
                return ret;
        }
*/
        aw2013_led->en_state = pinctrl_lookup_state(aw2013_led->p, "breathing_en");
        if (IS_ERR_OR_NULL(aw2013_led->en_state)) {
		dev_err(dev, "fail look pinctl en state");
                devm_pinctrl_put(aw2013_led->p);
                return PTR_ERR(aw2013_led->en_state);
        }

        ret = pinctrl_select_state(aw2013_led->p, aw2013_led->en_state);
        if (ret < 0) {
		dev_err(dev, "fail select pinctl en state");
                devm_pinctrl_put(aw2013_led->p);
                return ret;
        }
	return 0;
}

static int aw2013_led_probe(struct i2c_client *clientp, const struct i2c_device_id *id)
{
	int rc = 0;
	int i, j;

        printk("aw2013 probe begin\n");
        if (!i2c_check_functionality(clientp->adapter,
                                I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
                dev_err(&clientp->dev, "client is not i2c capable\n");
                return -ENXIO;
        }   

	rc = sensor_power_onoff(true);
	if(rc < 0) {
		dev_err(&clientp->dev, "power on failed");
		return rc;
	}
	chipid = i2c_smbus_read_byte_data(clientp, RSTR_REG);
	if(chipid < 0) {
		dev_err(&clientp->dev, "find device error");
		chipid = 0xff;
		rc = -ENODEV;
		goto err_read_id;
	} else  if (chipid == 0x33)
		dev_info(&clientp->dev, "device connected");

	if (clientp->dev.of_node == NULL) {
		dev_err(&clientp->dev, "can not find device tree node\n");
		rc = -ENODEV;
		goto err_read_id;
	} else {
		aw2013_led = kzalloc(sizeof(struct aw2013_led_data), GFP_KERNEL);
		if (aw2013_led == NULL) {
			dev_err(&clientp->dev,"no memory for device\n");
			rc = -ENOMEM;
			goto err_read_id;
		}

		rc = aw2013_pinctrl_select(&clientp->dev);
		if (rc)
			goto err_pinctrl;
		
		rc = led_parse_dt(&clientp->dev);
		if (rc) {
			goto err_pinctrl;
		}

		rc = led_gpio_config(&clientp->dev);
		if (rc) {
			dev_warn(&clientp->dev,"config led gpio failed\n");
		}
	}

	for (i = 0; i < 3; i++) {
		aw2013_led->breathing_dev[i].led.brightness_set = led_set;
		aw2013_led->breathing_dev[i].led.brightness = LED_OFF;
		aw2013_led->breathing_dev[i].t1 = 0;
		aw2013_led->breathing_dev[i].t2 = 0;
		aw2013_led->breathing_dev[i].t3 = 0;
		aw2013_led->breathing_dev[i].t4 = 0;
		aw2013_led->breathing_dev[i].delay_to_start = 0;
		aw2013_led->breathing_dev[i].repeat = 0;
	}
	aw2013_led->breathing_dev[0].led.name = "right";
	aw2013_led->breathing_dev[1].led.name = "middle";
	aw2013_led->breathing_dev[2].led.name = "left";

	mutex_init(&aw2013_led->data_lock);
	aw2013_led->client = clientp;

	for (i = 0; i < 3; i++) {
		rc= led_classdev_register(&clientp->dev, &aw2013_led->breathing_dev[i].led);
		if (rc) {
			dev_err(&clientp->dev,": led_classdev_register failed");
			goto err_led_classdev_register_failed;
		}
	}

	for (i = 0; i < 3; i++) {
		rc = device_create_file(aw2013_led->breathing_dev[i].led.dev, &dev_attr_breathing);
		if (rc) {
			dev_err(&clientp->dev,"create dev_attr_breathing failed");
			goto err_out_attr_breathing;
		}
	}

	i2c_set_clientdata(clientp, aw2013_led);

	printk(KERN_NOTICE "Exit_aw2013 led_probe.\r\n");		
	return 0;

err_out_attr_breathing:
	for (j = 0; j < i; j++)
		device_remove_file(aw2013_led->breathing_dev[i].led.dev, &dev_attr_breathing);
	i = 3;
err_led_classdev_register_failed:
	for (j = 0; j < i; j++)
		led_classdev_unregister(&aw2013_led->breathing_dev[i].led);
	led_gpio_unconfig();
err_pinctrl:
	if (aw2013_led->p)
		devm_pinctrl_put(aw2013_led->p);
	kfree(aw2013_led);
err_read_id:
	sensor_power_onoff(false);

	return rc;

}

static int aw2013_led_remove(struct i2c_client *clientp)
{
	int i;

	for (i = 0; i < 3; i++){
		device_remove_file(aw2013_led->breathing_dev[i].led.dev, &dev_attr_breathing);
		led_classdev_unregister(&aw2013_led->breathing_dev[i].led);
	}
	led_gpio_unconfig();
	devm_pinctrl_put(aw2013_led->p);
	kfree(aw2013_led);
	sensor_power_onoff(false);
	return 0;
}

static const struct i2c_device_id aw2013_led_id[] = {
	{"aw2013", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, aw2013_led_id);

static struct of_device_id aw2013_led_of_match[] = {
	{.compatible = "awinic,aw2013",},
	{},
};

static struct i2c_driver aw2013_led_driver = {
	.probe		= aw2013_led_probe,
	.remove		= aw2013_led_remove,
	.id_table       = aw2013_led_id,
	.driver		= {
		.name	= "aw2013,breathing leds",
		.owner	= THIS_MODULE,
		.of_match_table = aw2013_led_of_match,
	},
};

module_i2c_driver(aw2013_led_driver);
