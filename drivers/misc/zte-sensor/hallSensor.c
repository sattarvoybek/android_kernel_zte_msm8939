#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#include <linux/module.h>
extern  void msleep(int ms);
static int hall_state = 1;
module_param(hall_state, int, 0644);


//MODULE_PARM_DESC(hall_state, "hall sensor state");
/**/

struct hall_chip {
	struct input_dev *idev;
	struct work_struct work;
	bool enabled;
	int gpio;
	int irq;
}*hall_ic_data;



extern int g_is_touchscreen_window;

static irqreturn_t hall_irq_handler(int irq, void *dev_id)
{
	    return IRQ_WAKE_THREAD;
}

static irqreturn_t hall_irq_handler_thread(int irq, void *handle)
{
    schedule_work(&hall_ic_data->work);
    return IRQ_HANDLED;
}

static void hall_work_func(struct work_struct * work) 
{
	int value1,value2,value;
	int report_en=0;

	report_en=0;

	//msleep(60);
	
	value1=gpio_get_value(hall_ic_data->gpio);	

	msleep(60);

	value2=gpio_get_value(hall_ic_data->gpio);

	if (value1==value2){
           value=value1 ; 
	    report_en=1;   
	 }
	 
		
	

	printk("DEBUG: hall get value %d\n", value);

	
	if((hall_ic_data->enabled==1)&&(1==report_en)){
		if(value==1){
			g_is_touchscreen_window = 0;
			input_report_key(hall_ic_data->idev, KEY_HALL_SENSOR_OPEN, 1);
			input_sync(hall_ic_data->idev);
			input_report_key(hall_ic_data->idev, KEY_HALL_SENSOR_OPEN, 0);
                     hall_state=1;
			
		} else {
			g_is_touchscreen_window = 1;
			input_report_key(hall_ic_data->idev, KEY_HALL_SENSOR_CLOSE, 1);
			input_sync(hall_ic_data->idev);
			input_report_key(hall_ic_data->idev, KEY_HALL_SENSOR_CLOSE, 0);
			 hall_state=0;
		}
		input_sync(hall_ic_data->idev);
	}
	
	
}


static ssize_t hall_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int val;

	val=gpio_get_value(hall_ic_data->gpio);
	return sprintf(buf, "%d %d\n", hall_ic_data->enabled, val);
}

static ssize_t  hall_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{

	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	printk("hall driver get enable %d\n", (int)value);
	if (value)	
	{
		if(hall_ic_data->enabled==0)
		{
			enable_irq_wake(hall_ic_data->irq);
			hall_ic_data->enabled=1;
		}
	} else {
		if(hall_ic_data->enabled==1)
		{
			disable_irq_wake(hall_ic_data->irq);
			hall_ic_data->enabled=0;
		}
    }	
	return size;
}

static struct device_attribute hall_attrs[] = {
	__ATTR(enable, 0664, hall_enable_show, hall_enable_store),
};

static int add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err( dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static int remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size ; i++)
		device_remove_file(dev, a + i);
	return 0;
}

static int hall_init(struct platform_device * pdev )
{

	int ret;
	int value_state;
	struct pinctrl *pinctrl;
	
	pr_info ("hall  : init start");
		
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
			dev_err(&pdev->dev, "pins are not configured from the driver");
		    return -ENODEV;
	} 
		
	hall_ic_data = kzalloc(sizeof(struct hall_chip), GFP_KERNEL);
	if (!hall_ic_data) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "kzalloc failed");
		return ret;
	}
	
	ret = of_get_gpio(pdev->dev.of_node, 0); 
	if (gpio_is_valid(ret)) {
		printk(KERN_INFO "hall irq get gpio %d\n", ret);
		gpio_request(ret, "hall_irq");
		gpio_direction_input(ret);
		hall_ic_data->gpio = ret;
		hall_ic_data->irq = gpio_to_irq(ret);
	} else {
		dev_err(&pdev->dev, "error irq gpio %d", ret);
		goto failed_get_gpio;
	}
	INIT_WORK(&(hall_ic_data->work),hall_work_func);
	
	hall_ic_data->idev = input_allocate_device();
	if (!hall_ic_data->idev) {
		dev_err(&pdev->dev, "allocate input device failed\n");
		ret = -ENODEV;
		goto failed_allocate_input;
	}
	hall_ic_data->idev->name = "hall_ic";
	__set_bit(EV_KEY, hall_ic_data->idev->evbit);
	input_set_capability(hall_ic_data->idev, EV_KEY, KEY_HALL_SENSOR_OPEN);
	input_set_capability(hall_ic_data->idev, EV_KEY, KEY_HALL_SENSOR_CLOSE);
	ret = input_register_device(hall_ic_data->idev);
	if (ret) {
		dev_err(&pdev->dev, "register input device failed\n");
		goto failed_register_input;
	}

	ret = add_sysfs_interfaces(&hall_ic_data->idev->dev,
			hall_attrs, ARRAY_SIZE(hall_attrs));
	if (ret){
		dev_err(&pdev->dev, "add sysfs failed\n");
		goto failed_add_sysfs;
	}

	ret = request_threaded_irq(hall_ic_data->irq, hall_irq_handler, hall_irq_handler_thread,
			IRQ_TYPE_EDGE_BOTH,
			"hall_ic_irq", NULL);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed\n");
			goto failed_request_irq;
	}

	enable_irq_wake(hall_ic_data->irq);
	hall_ic_data->enabled=1;

	value_state=gpio_get_value(hall_ic_data->gpio);	

	if(value_state==1){
        hall_state=1;
	}
	else{
          hall_state=0;

	}
	
       printk("hall Init hall_state=%d \n",hall_state);
	printk("%s:hall Init success!\n",__func__);
	return 0;

failed_request_irq:
	remove_sysfs_interfaces(&hall_ic_data->idev->dev, hall_attrs, ARRAY_SIZE(hall_attrs));
failed_add_sysfs:
	input_unregister_device(hall_ic_data->idev);
failed_register_input:
	input_free_device(hall_ic_data->idev);
failed_allocate_input:
	gpio_free(hall_ic_data->gpio);
failed_get_gpio:
	kfree(hall_ic_data);
	printk("hall ic probe failed \n");
	return ret;
}

		
static int hall_exit(struct platform_device * pdev)
{
	remove_sysfs_interfaces(&hall_ic_data->idev->dev, hall_attrs, ARRAY_SIZE(hall_attrs));
	input_unregister_device(hall_ic_data->idev);
	input_free_device(hall_ic_data->idev);
	gpio_free(hall_ic_data->gpio);
	kfree(hall_ic_data);
	return 0;
}

static struct of_device_id hall_match_table[] = {
	{ .compatible = "hall,hall-ic", },
	{ },
};
struct platform_driver hall_ic_driver =
{
	.probe = hall_init,
	.remove = hall_exit,
	.driver = {
		.name	= "hall_ic",
		.of_match_table = hall_match_table, 
	},

};
module_platform_driver(hall_ic_driver);

