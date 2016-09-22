#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h> /* BUS_I2C */
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include "CwMcuSensor.h"
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/sensor_power.h>
#include <linux/pinctrl/consumer.h>
#include <linux/wakelock.h>

/* GPIO for MCU control */
//nexus5
/*
#define GPIO_CW_MCU_WAKE_UP		67
#define GPIO_CW_MCU_BOOT		139
#define GPIO_CW_MCU_RESET		140
#define GPIO_CW_MCU_INTERRUPT	73

#define GPIO_CW_MCU_WAKE_UP		138
#define GPIO_CW_MCU_BOOT		139
#define GPIO_CW_MCU_RESET		140
#define GPIO_CW_MCU_INTERRUPT	39
*/

#define DPS_MAX			        (1 << (16 - 1))

/* Input poll interval in milliseconds */
#define CWMCU_POLL_INTERVAL	10
#define CWMCU_POLL_MAX		200
#define CWMCU_POLL_MIN		10

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define CWMCU_MAX_OUTPUT_ID		(CW_SNAP+1)
#define CWMCU_MAX_OUTPUT_BYTE		(CWMCU_MAX_OUTPUT_ID * 7)
#define CWMCU_MAX_DRIVER_OUTPUT_BYTE		256

#define VERBOSE(format,...) do {            \
                                if (flagVerbose) printk(KERN_DEBUG format, ##__VA_ARGS__); \
                            } while (0)

/* Flag to enable/disable verbose message, for debug convinience. */
static volatile int flagVerbose;

/* turn on gpio interrupt if defined */
#define CWMCU_INTERRUPT

struct CWMCU_data {
	struct i2c_client *client;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct input_polled_dev *input_polled;
	struct input_dev *input;
	struct timeval previous;
	struct timeval now;
	int mcu_mode;
	int mcu_status;
	int mcu_reset;

	struct hrtimer timer;

	struct workqueue_struct *driver_wq;
	struct work_struct work;

	/* enable & batch list */
	uint32_t enabled_list;
	uint32_t batched_list;
	uint32_t flush_list;
	uint32_t step_counter;
	bool use_interrupt;
	int batch_enabled;
	uint64_t batch_timeout;
	int64_t sensor_timeout[CW_SENSORS_ID_END];
	int64_t current_timeout;
	int timeout_count;
	uint32_t interrupt_status;

	/* report time */
	int64_t	sensors_time[CW_SENSORS_ID_END];/* pre_timestamp(us) */
	int64_t time_diff[CW_SENSORS_ID_END];
	int64_t	report_period[CW_SENSORS_ID_END];/* ms */
	uint32_t update_list;

	/* power status */
	int power_on_list;

	/* debug */
	int debug_count;

	/* calibrator */
	uint8_t cal_cmd;
	uint8_t cal_type;
	uint8_t cal_id;

	/* gpio */
	int irq_gpio;
	int wakeup_gpio;
	int reset_gpio;
	int boot_gpio;

	int compass_gpio;
	int acc_gpio;
	int als_gpio;

	int cmd;
	uint32_t addr;
	int len;
	int value;
	int mcu_slave_addr;
	int fwupdate_status;
	int cw_i2c_rw;	/* r = 0 , w = 1 */
	int cw_i2c_len;
	uint8_t cw_i2c_data[300];
	char firmware_path[50];

	struct wake_lock data_report_lock;
	int product_flag;
};

struct CWMCU_data *sensor;

static int version[2] = {0, 1};
static int16_t last_data[5][3];

static int CWMCU_i2c_write(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int dummy;
	int i;
	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(sensor->client, reg_addr++, data[i]);
		if (dummy < 0) {
		pr_err("i2c write error =%d\n", dummy);
			return dummy;
		}
	}
	return 0;
}

/* Returns the number of read bytes on success */
static int CWMCU_i2c_read(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len)
{
	VERBOSE("--CWMCU-- %s\n", __func__);
	return i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len, data);
}

/* write format    1.slave address  2.data[0]  3.data[1] 4.data[2] */
static int CWMCU_write_i2c_block(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len)
{
	int dummy;
	dummy = i2c_smbus_write_i2c_block_data(sensor->client, reg_addr, len, data);
	if (dummy < 0) {
		pr_err("i2c write error =%d\n", dummy);
		return dummy;
	}
	return 0;
}

static int CWMCU_write_serial(u8 *data, int len)
{
	int dummy;
	dummy = i2c_master_send(sensor->client, data, len);
	if (dummy < 0) {
		pr_err("i2c write error =%d\n", dummy);
		return dummy;
	}
	return 0;
}

static int CWMCU_read_serial(u8 *data, int len)
{
	int dummy;
	dummy = i2c_master_recv(sensor->client, data, len);
	if (dummy < 0) {
		pr_err("i2c read error =%d\n", dummy);
		return dummy;
	}
	return 0;
}

static void cwmcu_powermode_switch(SWITCH_POWER_ID id, int onoff)
{
	if (onoff) {
		if (sensor->power_on_list == 0) {
			gpio_set_value(sensor->wakeup_gpio, onoff);
		}
		sensor->power_on_list |= ((uint32_t)(1) << id);
		VERBOSE("--CWMCU-- power mode sleep~!!!\n");
		usleep_range(1000, 2000);
	} else {
		sensor->power_on_list &= ~(1 << id);
		if (sensor->power_on_list == 0) {
			gpio_set_value(sensor->wakeup_gpio, onoff);
		}
	}
	/* printk(KERN_DEBUG "--CWMCU--%s id = %d, onoff = %d\n", __func__, id, onoff); */
}

static int cwmcu_get_version(void)
{
    int ret;
    uint8_t count;
    uint8_t data[6] = {0};

    printk("--CWMCU-- %s\n", __func__);

	cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 1);
    for (count = 0; count < 3; count++) {
        if ((ret = CWMCU_i2c_read(sensor, CW_FWVERSION, data, 2)) >= 0) {
            printk("--CWMCU-- CHECK_FIRMWAVE_VERSION : %d.%d\n", data[0],data[1]);
            cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 0);
			version[0] = data[0];
			version[1] = data[1];
            return 0;
        }
        mdelay(20);
    }
    cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 0);
	version[0] = 0;
	version[1] = 0;
    printk("--CWMCU-- Cannot get fw version, [ret:%d]\n", ret);
    return ret;
}

static void cwmcu_send_time_sync_event(void)
{
	printk( "--CWMCU-- send time sync event!\n");
	input_report_abs(sensor->input, CW_ABS_TIMESYNC, 0);
	input_sync(sensor->input);

	input_report_abs(sensor->input, CW_ABS_TIMESYNC, 1);
	input_sync(sensor->input);
}


static void cwmcu_restore_status(void)
{
    int id,part;
    u8 list;
    uint8_t delay_ms = 0;
	uint8_t data[5] = {0};
	uint8_t sensorId = 0;

    printk("--CWMCU-- %s\n", __func__);

    //enable sensor
    for (id = 0; id < CW_SENSORS_ID_END; id++) {
        if (sensor->enabled_list & (1<<id)) {
            part = id / 8;
            list = (u8)(sensor->enabled_list>>(part*8));
            CWMCU_i2c_write(sensor, CW_ENABLE_REG+part, &list, 1);
            id = ((part+1)*8)-1;
        }
    }

    //set sensor report rate
    for (id = 0; id < CW_SENSORS_ID_END; id++) {
        if (sensor->enabled_list & (1<<id)) {
            delay_ms = (uint8_t)sensor->report_period[id];
			data[0] = (uint8_t)id;
			data[1] = delay_ms;
			CWMCU_write_i2c_block(sensor, CW_SENSOR_DELAY_SET, data, 2);
        }
    }

    /* flush status */
    for(id=0;id<CW_SENSORS_ID_END;id++) {
        if(sensor->flush_list & (1<<id)) {
            sensorId = (uint8_t)id;
            if (CWMCU_i2c_write(sensor, CW_BATCHFLUSH, &sensorId, 1) < 0) {
                printk("--CWMCU-- flush => i2c error~!!\n");
            }
        }
    }

    /* batch status */
    for(id=0;id<CW_SENSORS_ID_END;id++)
    {
        if(sensor->batched_list & (1<<id))
        {
            data[0] = (uint8_t)id;
            memcpy(&data[1],&sensor->sensor_timeout[id],sizeof(uint8_t)*4);
            CWMCU_write_i2c_block(sensor, CW_BATCHTIMEOUT, data, 5);
        }
    }

	/* pedometer */
    memcpy(data,&sensor->step_counter,sizeof(uint8_t)*4);
    if(CWMCU_write_i2c_block(sensor, CW_SET_STEP_COUNTER, data, 4) < 0) {
        printk("--CWMCU-- restore pedometer error!!\n");
    }
}

static void cwmcu_check_mcu_enable(void)
{
	uint8_t data[6];
	uint32_t mcu_enable_list = 0;
	uint32_t android_enable_list = 0;

	printk("--CWMCU-- check mcu enable status.\n");

	if (CWMCU_i2c_read(sensor, CW_ENABLE_STATUS, data, 4) < 0) {
        printk("--CWMCU-- get mcu enable list error.\n");
	} else {
		mcu_enable_list = (uint32_t)data[3]<< 24 | (uint32_t)data[2]<< 16 | (uint32_t)data[1]<< 8 | data[0];
		if(mcu_enable_list != android_enable_list) {
			cwmcu_restore_status();
		}
	}
}

static int cwmcu_error_log_read(struct CWMCU_data *sensor)
{
    uint8_t data[32] = {0};
    char log_message[32];
    int data_count = 0;
    int i = 0;

    if (sensor->mcu_mode == CW_BOOT) {
        return 0;
    }

    if (CWMCU_i2c_read(sensor, CW_ERROR_COUNT_GET, data, 1) >= 0) {
        data_count = data[0];
        printk("--CWMCU-- read error_log: count = %d\n", data_count);
        for (i = 0; i < data_count; i++) {
            /* read 32bytes */
            if (CWMCU_i2c_read(sensor, CW_ERROR_INFO_GET, data, 32) >= 0) {
                    memcpy(log_message,data,32);
                    printk("--CWMCU-- error_log => %s\n", log_message);
            } else {
                printk("--CWMCU-- read error_log failed.\n");
            }
        }
    }

    return 1;
}

static int cwmcu_debug_log_read(struct CWMCU_data *sensor)
{
    uint8_t data[32] = {0};
    char log_message[32];
    int data_count = 0;
    int i = 0;

    if (sensor->mcu_mode == CW_BOOT) {
        return 0;
    }

    if (CWMCU_i2c_read(sensor, CW_DEBUG_COUNT_GET, data, 1) >= 0) {
        data_count = data[0];
        printk("--CWMCU-- read debug_log: count = %d\n", data_count);
        for (i = 0; i < data_count; i++) {
            /* read 32bytes */
            memset(data,0,32);
            memset(log_message,0,32);
            if (CWMCU_i2c_read(sensor, CW_DEBUG_INFO_GET, data, 32) >= 0) {
                memcpy(log_message,data,32);
                printk("--CWMCU-- debug_log => %s\n", log_message);

            } else {
                printk("--CWMCU-- read debug_log failed.\n");
            }
        }
    }

    return 1;
}

/*
*	Sensor get data and report event
*	format data[0] = id
*	format data[7]~data[10] = mcu time stamp [L/H]
*	format data[1]~data[6] = data X,Y,Z [L/H]
*
*	Sensor flush event
*	format data[0] = meta data id
*	format data[5] , data[6] = flush sensor id
*/
static int cwmcu_batch_read(struct CWMCU_data *sensor)
{
	int i = 0;
	uint8_t data[20] = {0};
	uint32_t data_event[6] = {0};
	uint32_t timestamp = 0;
    uint16_t batch_count = 0;

	if (sensor->mcu_mode == CW_BOOT) {
        return 0;
    }

	/* read the count of batch queue */
	if (CWMCU_i2c_read(sensor, CW_BATCHCOUNT, data, 2) >= 0) {
		batch_count = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
        VERBOSE("--CWMCU-- read batch count = %d\n", batch_count);
	} else {
        printk("--CWMCU-- read batch count fail.\n");
	}

    //4,294,967,295
	for (i = 0; i < batch_count; i++) {
        if (CWMCU_i2c_read(sensor, CW_BATCHEVENT, data, 11) >= 0) {
			/* check if there are no data from queue */
			if (data[0] != CWMCU_NODATA) {
				if (data[0] == CW_META_DATA) {
					data_event[0] = ((u32)data[0] << 16) | ((u32)data[5] << 8) | (u32)data[6];
					sensor->flush_list &= ~(1<<data[6]);
                    VERBOSE("--CWMCU-- META_DATA => %02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
                        data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10]);
					input_report_abs(sensor->input, CW_ABS_Z, data_event[0]);
					input_sync(sensor->input);
					input_report_abs(sensor->input, CW_ABS_Z, 0xFF0000);
                    input_sync(sensor->input);
				} else {
					data_event[0] = ((u32)data[0] << 16) | ((u32)data[2] << 8) | (u32)data[1];
					data_event[1] = ((u32)data[0] << 16) | ((u32)data[4] << 8) | (u32)data[3];
					data_event[2] = ((u32)data[0] << 16) | ((u32)data[6] << 8) | (u32)data[5];
					timestamp = (uint32_t)(data[10]<<24) | (uint32_t)(data[9]<<16) | (uint32_t)(data[8]<<8) | (uint32_t)data[7];
					data_event[3] = timestamp;
					//data_event[3] = ((u32)data[0] << 16) | ((u32)data[8] << 8) | (u32)data[7];//timediff

					/* check flush event */
					input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
					input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
					input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
                    input_report_abs(sensor->input, CW_ABS_TIMESTAMP, data_event[3]);
					input_sync(sensor->input);
                    VERBOSE("--CWMCU-- Batch data_event[0-4] => %u, %u, %u, H:%u, L:%u\n",
                            data_event[0], data_event[1], data_event[2], data_event[3],data_event[4]);

                    VERBOSE("--CWMCU-- Batch raw data => %2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x\n",
                        data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10]);
					/* reset input event */
					input_report_abs(sensor->input, CW_ABS_X, 0xFF0000);
                    input_report_abs(sensor->input, CW_ABS_Y, 0xFF0000);
                    input_report_abs(sensor->input, CW_ABS_Z, 0xFF0000);
                    input_sync(sensor->input);
				}
			}
		}
        else {
            printk("--CWMCU-- read batch event failed~!!\n");
        }
	}
    return 1;
}

static void cwmcu_gesture_read(struct CWMCU_data *sensor)
{
	uint8_t data[2] = {0};
	uint32_t data_event;
	int data_count = 0;
	int i = 0;

	if (CWMCU_i2c_read(sensor, CW_READ_GESTURE_EVENT_COUNT, data, 1) >= 0) {
		data_count = data[0];
		printk(KERN_INFO "--CWMCU-- read gesture count = %d\n", data_count);
		for (i = 0; i < data_count; i++) {
			/* read 2byte */
			if (CWMCU_i2c_read(sensor, CW_READ_GESTURE_EVENT_DATA, data, 2) >= 0) {
				VERBOSE(KERN_DEBUG "--CWMCU-- read gesture data = %d\n", data_count);
				if (data[0] != CWMCU_NODATA) {
					data_event = ((u32)data[0] << 16) | (((u32)data[1]));
					printk(KERN_INFO "--CWMCU--Normal mgesture %d data -> x = %d\n"
							, data[0]
							, data[1]
							);
					input_report_abs(sensor->input, CW_ABS_X, data_event);
					input_sync(sensor->input);
					input_report_abs(sensor->input, CW_ABS_X, 0xFF0000);
                    input_sync(sensor->input);
				}
			} else {
				printk(KERN_DEBUG "--CWMCU-- read gesture failed~!!\n");
			}
		}
	}
}

#if 0
static void cwmcu_check_sensor_update(void)
{
	int id = 0;
	int64_t tvusec, temp = 0;
	unsigned int tvsec;

	do_gettimeofday(&sensor->now);
	tvsec = sensor->now.tv_sec;
	tvusec = sensor->now.tv_usec;

	temp = (int64_t)(tvsec * 1000000LL) + tvusec;

	/* printk( "--CWMCU-- time(u) = %llu, tv_sec = %u, v_usec = %llu\n",temp, tvsec, tvusec); */

	for (id = 0; id < CW_SENSORS_ID_END; id++) {

		if( sensor->enabled_list & (1<<id) && (sensor->sensor_timeout[id] == 0)) {
			/* printk( "--CWMCU-- id = %d\n", id); */
			sensor->time_diff[id] = temp - sensor->sensors_time[id];
			if (sensor->time_diff[id] >= (sensor->report_period[id] * 1000)) {
				sensor->update_list |= 1<<id;
				sensor->sensors_time[id] = temp;
			} else {
				sensor->update_list &= ~(1<<id);
			}
		} else {
			sensor->update_list &= ~(1<<id);
		}
	}
	/*
	printk( "--CWMCU-- sensor->update_list = %d\n", sensor->update_list);
	*/
}
#endif
static int CWMCU_read(struct CWMCU_data *sensor)
{
	int id_check = 0;
	uint8_t data[20] = {0};
	uint32_t data_event[7] = {0};

	uint8_t MCU_REG = 0;
	static int16_t als_last = 0;

	if ((sensor->mcu_mode == CW_BOOT) || (sensor->mcu_status == CW_INVAILD)) {
		/* it will not get data if status is bootloader mode */
		return 0;
	}

	//cwmcu_check_sensor_update();
	VERBOSE(KERN_DEBUG "--CWMCU--%s\n", __func__);

	if (sensor->update_list) {
		for (id_check = 0; id_check < CW_SENSORS_ID_END; id_check++) {
			if ((sensor->update_list & (1<<id_check)) && (sensor->sensor_timeout[id_check] == 0)) {
					switch (id_check) {
					case CW_ACCELERATION:
					case CW_MAGNETIC:
					case CW_GYRO:
					case CW_LIGHT:
					case CW_PROXIMITY:
					case CW_PRESSURE:
					case CW_ORIENTATION:
					case CW_ROTATIONVECTOR:
					case CW_LINEARACCELERATION:
					case CW_GRAVITY:
					case CW_GAME_ROTATION_VECTOR:
					case CW_GEOMAGNETIC_ROTATION_VECTOR:

						MCU_REG = CWMCU_I2C_SENSORS_REG_START+id_check;
						/* read 6byte */
						if (CWMCU_i2c_read(sensor, MCU_REG, data, 6) >= 0) {
								data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
								data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
								data_event[2] = ((u32)id_check << 16) | (((u32)data[5] << 8) | (u32)data[4]);

								if (id_check == CW_PROXIMITY){
									printk(KERN_INFO "--CWMCU--Normal prox data -> x = %d, y = %d, z = %d\n"
											, (int16_t)((u32)data[1] << 8) | (u32)data[0]
											, (int16_t)((u32)data[3] << 8) | (u32)data[2]
											, (int16_t)((u32)data[5] << 8) | (u32)data[4]
										   );
								} else if ((id_check == CW_LIGHT) && (als_last != ((int16_t)((u32)data[1] << 8) | (u32)data[0]))){
									als_last = (int16_t)((u32)data[1] << 8) | (u32)data[0];
									//printk(KERN_INFO "--CWMCU--Normal als data -> x = %u\n"
									//		, (uint16_t)((u32)data[1] << 8) | (u32)data[0]);
								} else {
									VERBOSE(KERN_DEBUG "--CWMCU--Normal %d data -> x = %d, y = %d, z = %d\n"
											, id_check
											, (int16_t)((u32)data[1] << 8) | (u32)data[0]
											, (int16_t)((u32)data[3] << 8) | (u32)data[2]
											, (int16_t)((u32)data[5] << 8) | (u32)data[4]
										   );
								}

								if ((id_check == CW_ACCELERATION) || (id_check == CW_MAGNETIC) || (id_check == CW_GYRO) || (id_check == CW_PROXIMITY)){
									last_data[id_check][0] = (int16_t)((u32)data[1] << 8) | (u32)data[0];
									last_data[id_check][1] = (int16_t)((u32)data[3] << 8) | (u32)data[2];
									last_data[id_check][2] = (int16_t)((u32)data[5] << 8) | (u32)data[4];
									if ((id_check == CW_GYRO) && (last_data[id_check][0] == 0) && (last_data[id_check][1] == 0) && (last_data[id_check][2] == 0))
										last_data[id_check][0] = last_data[id_check][1] = last_data[id_check][2] = 1;
								}
								if (id_check == CW_MAGNETIC || id_check == CW_ORIENTATION) {
									if (CWMCU_i2c_read(sensor, CW_ACCURACY, data, 1) >= 0) {
										data_event[6] = ((u32)id_check << 16) | (u32)data[0];
									}
									VERBOSE(KERN_DEBUG "--CWMCU--MAG ACCURACY = %d\n", data[0]);
									input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
									input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
									input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
									input_report_abs(sensor->input, CW_ABS_ACCURACY, data_event[6]);
									input_sync(sensor->input);
								} else {
									input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
									input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
									input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
									input_sync(sensor->input);
								}
								/* reset x,y,z */
								input_report_abs(sensor->input, CW_ABS_X, 0xFF0000);
								input_report_abs(sensor->input, CW_ABS_Y, 0xFF0000);
								input_report_abs(sensor->input, CW_ABS_Z, 0xFF0000);
								input_report_abs(sensor->input, CW_ABS_ACCURACY, 0xFF0000);
								input_sync(sensor->input);
						} else {
							printk(KERN_DEBUG "--CWMCU-- CWMCU_i2c_read error 0x%x~!!!\n", CWMCU_I2C_SENSORS_REG_START+id_check);
						}
						break;
					case CW_MAGNETIC_UNCALIBRATED:
					case CW_GYROSCOPE_UNCALIBRATED:
							/* read 12byte */
							if (CWMCU_i2c_read(sensor, CWMCU_I2C_SENSORS_REG_START+id_check, data, 12) >= 0) {
									data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
									data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
									data_event[2] = ((u32)id_check << 16) | (((u32)data[5] << 8) | (u32)data[4]);
									data_event[3] = ((u32)id_check << 16) | (((u32)data[7] << 8) | (u32)data[6]);
									data_event[4] = ((u32)id_check << 16) | (((u32)data[9] << 8) | (u32)data[8]);
									data_event[5] = ((u32)id_check << 16) | (((u32)data[11] << 8) | (u32)data[10]);

									VERBOSE(KERN_DEBUG "--CWMCU--Normal %d data -> x = %d, y = %d, z = %d, x_bios = %d, y_bios = %d, z_bios = %d,\n"
												, id_check
												, (int16_t)((u32)data[1] << 8) | (u32)data[0]
												, (int16_t)((u32)data[3] << 8) | (u32)data[2]
												, (int16_t)((u32)data[5] << 8) | (u32)data[4]
												, (int16_t)((u32)data[7] << 8) | (u32)data[6]
												, (int16_t)((u32)data[9] << 8) | (u32)data[8]
												, (int16_t)((u32)data[11] << 8) | (u32)data[10]
												);
									if (CWMCU_i2c_read(sensor, CW_ACCURACY, data, 1) >= 0) {
											data_event[6] = ((u32)id_check << 16) | (u32)data[0];
										}
									input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
									input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
									input_report_abs(sensor->input, CW_ABS_Z, data_event[2]);
									input_report_abs(sensor->input, CW_ABS_X1, data_event[3]);
									input_report_abs(sensor->input, CW_ABS_Y1, data_event[4]);
									input_report_abs(sensor->input, CW_ABS_Z1, data_event[5]);
									input_report_abs(sensor->input, CW_ABS_ACCURACY, data_event[6]);
									input_sync(sensor->input);

									input_report_abs(sensor->input, CW_ABS_X, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Y, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Z, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_X1, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Y1, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_Z1, 0xFF0000);
									input_report_abs(sensor->input, CW_ABS_ACCURACY, 0xFF0000);
									input_sync(sensor->input);
							}
							break;
					case CW_STEP_COUNTER:
							/* read 6byte */
							if (CWMCU_i2c_read(sensor, CWMCU_I2C_SENSORS_REG_START+id_check, data, 4) >= 0) {
									data_event[0] = ((u32)id_check << 16) | (((u32)data[1] << 8) | (u32)data[0]);
									data_event[1] = ((u32)id_check << 16) | (((u32)data[3] << 8) | (u32)data[2]);
									sensor->step_counter = (uint32_t)data_event[1]<<16 | (uint32_t)data_event[0];

									input_report_abs(sensor->input, CW_ABS_X, data_event[0]);
									input_report_abs(sensor->input, CW_ABS_Y, data_event[1]);
									input_sync(sensor->input);

									VERBOSE(KERN_DEBUG "--CWMCU--Normal %d data -> x = %d, y = %d, z = 0\n"
												, id_check
												, (int16_t)((u32)data[1] << 8) | (u32)data[0]
												, (int16_t)((u32)data[3] << 8) | (u32)data[2]);
								} else {
									printk(KERN_DEBUG "--CWMCU-- CWMCU_i2c_read error 0x%x~!!!\n", CWMCU_I2C_SENSORS_REG_START+id_check);
								}
							break;
					default:
							break;
					}
			}
		}
	}
	return 0;
}

/*==========sysfs node=====================*/

static ssize_t active_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int enabled = 0;
	int sensors_id = 0;
	//int error_msg = 0;
	uint8_t data = 0;
	uint8_t i = 0;
	uint8_t delay_ms = 0;
	uint8_t buffer[2] = {0};
	uint8_t error = 0;

	sscanf(buf, "%d %d\n", &sensors_id, &enabled);

	sensor->enabled_list &= ~(1<<sensors_id);
	sensor->enabled_list |= ((uint32_t)enabled)<<sensors_id;

	/* clean timeout value if sensor turn off */
	if (enabled == 0) {
		sensor->sensor_timeout[sensors_id] = 0;
		sensor->sensors_time[sensors_id] = 0;
	} else {
		do_gettimeofday(&sensor->now);
		sensor->sensors_time[sensors_id] = (sensor->now.tv_sec * 1000000LL) + sensor->now.tv_usec;
	}

	if ((sensor->mcu_mode == CW_BOOT) || (sensor->mcu_status == CW_INVAILD)) {
		return count;
	}

	i = sensors_id / 8;
	data = (u8)(sensor->enabled_list>>(i*8));
	cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 1);

	while (error < 5) {
		//printk("--CWMCU-- %s => i2c check~!!\n", __func__);
		if (CWMCU_i2c_write(sensor, CW_ENABLE_REG+i, &data, 1) < 0) {
			printk("--CWMCU-- %s => i2c error~!!\n", __func__);
			error++;
			continue;
		} else {
			//printk("--CWMCU-- %s => i2c ok~!!\n", __func__);
			error = 5;
		}
	}

	error = 0;

	//error_msg += CWMCU_i2c_write(sensor, CW_ENABLE_REG+i, &data, 1);

	printk( "--CWMCU-- data =%d, i = %d, sensors_id=%d enable=%d  enable_list=0x%x\n", data, i, sensors_id, enabled, sensor->enabled_list);

	delay_ms = (uint8_t)sensor->report_period[sensors_id];

	buffer[0] = (uint8_t)sensors_id;
    buffer[1] = delay_ms;

	while (error < 5) {
		//printk("--CWMCU-- %s => i2c check~!!\n", __func__);
		if (CWMCU_write_i2c_block(sensor, CW_SENSOR_DELAY_SET, buffer, 2) < 0) {
			printk("--CWMCU-- %s => i2c error~!!\n", __func__);
			error++;
			continue;
		} else {
			//printk("--CWMCU-- %s => i2c ok~!!\n", __func__);
			error = 5;
		}
	}

    //CWMCU_write_i2c_block(sensor, CW_SENSOR_DELAY_SET, buffer, 2);

	cwmcu_powermode_switch(SWITCH_POWER_ENABLE, 0);

	return count;
}

static ssize_t active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "0x%08x\n",sensor->enabled_list);
}

static ssize_t interval_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int id;
    uint8_t sensorId;
    uint8_t data[2];
    uint8_t sensor_report_rate[CW_SENSORS_ID_END] = {0};

    for(id=0;id<CW_SENSORS_ID_END;id++)
    {
        sensorId = (uint8_t)id;
        if(CWMCU_i2c_write(sensor, CW_SENSOR_DELAY_ID_SET, &sensorId, 1) >= 0) {
            if (CWMCU_i2c_read(sensor, CW_SENSOR_DELAY_GET, data, 1) >= 0) {
                sensor_report_rate[id] = data[0];
            } else {
                printk("--CWMCU-- interval_show()=> Get sensor report rate error. sensorId=%d\n",sensorId);
            }
        } else {
            printk("--CWMCU-- interval_show()=> Set sensor ID error. sensorId=%d\n",sensorId);
        }
    }

    for(id=0;id<CW_SENSORS_ID_END;id++) {
        printk("--CWMCU-- sensor report rate(ms) %d => %d\n",id, sensor_report_rate[id]);
    }

	return snprintf(buf, 16, "%d\n", CWMCU_POLL_INTERVAL);
}

static ssize_t interval_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	int sensors_id = 0;
	uint8_t delay_ms = 0;
	uint8_t data[2];
	
	sscanf(buf, "%d %d\n", &sensors_id , &val);
	if (val < CWMCU_POLL_MIN)
		val = CWMCU_POLL_MIN;

	sensor->report_period[sensors_id] = val;

	if ((sensor->mcu_mode == CW_BOOT) || (sensor->mcu_status == CW_INVAILD)) {
		return count;
	}

	delay_ms = (uint8_t)val;
	data[0] = (uint8_t)sensors_id;
	data[1] = delay_ms;

	cwmcu_powermode_switch(SWITCH_POWER_DELAY, 1);
	CWMCU_write_i2c_block(sensor, CW_SENSOR_DELAY_SET, data, 2);
	cwmcu_powermode_switch(SWITCH_POWER_DELAY, 0);
	printk( "--CWMCU-- sensors_id=%d delay_ms=%d\n", sensors_id, delay_ms);
	return count;
}

static ssize_t batch_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int sensors_id = 0;
	int delay_ms = 0;
	int64_t timeout = 0;
	int batch_mode = -1;
	uint8_t data[5] = {0};
	int id = 0;

//	printk( "--CWMCU-- %s in~!!\n", __func__);
	sscanf(buf, "%d %d %d %lld\n", &sensors_id, &batch_mode, &delay_ms, &timeout);
	printk("--CWMCU-- sensors_id = %d, batch_mode = %d, delay_ms = %d, timeout = %lld\n", sensors_id, batch_mode, delay_ms, timeout);
	sensor->sensor_timeout[sensors_id] = timeout;
	sensor->report_period[sensors_id] = delay_ms;

	if ((sensor->mcu_mode == CW_BOOT) || (sensor->mcu_status == CW_INVAILD)) {
		return count;
	}

	data[0] = (uint8_t)sensors_id;
	data[1] = (uint8_t)(timeout);
	data[2] = (uint8_t)(timeout >> 8);
	data[3] = (uint8_t)(timeout >> 16);
	data[4] = (uint8_t)(timeout >> 24);

	cwmcu_powermode_switch(SWITCH_POWER_BATCH, 1);
	CWMCU_write_i2c_block(sensor, CW_BATCHTIMEOUT, data, 5);
	cwmcu_powermode_switch(SWITCH_POWER_BATCH, 0);

	if (timeout > 0) {
		sensor->batched_list |= ((uint32_t)1<<sensors_id);
		sensor->batch_timeout = timeout;
	} else {
		sensor->batched_list &= ~((uint32_t)1<<sensors_id);
	}

	for(id = 0; id < CW_SENSORS_ID_END; id++) {
		if (sensor->batched_list & (1<<id)) {
			if (sensor->sensor_timeout[id] < sensor->batch_timeout)
				sensor->batch_timeout = sensor->sensor_timeout[id];
		}
	}

    //Set sensor report rate
    if(sensor->enabled_list & (1<<sensors_id)) {
        data[0] = (uint8_t)sensors_id;
        data[1] = delay_ms;
        cwmcu_powermode_switch(SWITCH_POWER_BATCH, 1);
        CWMCU_write_i2c_block(sensor, CW_SENSOR_DELAY_SET, data, 2);
        cwmcu_powermode_switch(SWITCH_POWER_BATCH, 0);
    }

//	printk( "--CWMCU-- BatchSet=> id = %d, timeout = %lld, delay_ms = %d, batch_timeout = %lld\n",
//		sensors_id, timeout, delay_ms, sensor->batch_timeout);

	return count;
}

static ssize_t batch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 255, "sensor->batched_list = %d, sensor->current_timeout = %lld\n"
					,sensor->batched_list, sensor->batch_timeout);
}

static ssize_t flush_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int sensors_id = 0;
	uint8_t data = 0;
	uint8_t error = 0;

	//printk(KERN_DEBUG "--CWMCU-- %s in\n", __func__);

	sscanf(buf, "%d\n", &sensors_id);

	sensor->flush_list &= ~(1<<sensors_id);
	sensor->flush_list |= (uint32_t)1<<sensors_id;

	if ((sensor->mcu_mode == CW_BOOT) || sensor->mcu_status == CW_INVAILD) {
		return count;
	}

	data = (uint8_t)sensors_id;
	printk(KERN_DEBUG "--CWMCU-- flush sensors_id = %d~!!\n", sensors_id);

    cwmcu_powermode_switch(SWITCH_POWER_BATCH, 1);
	while (error < 5) {
		//printk("--CWMCU-- flush => i2c check~!!\n");
		if (CWMCU_i2c_write(sensor, CW_BATCHFLUSH, &data, 1) < 0) {
			printk("--CWMCU-- flush => i2c error~!!\n");
			error++;
			continue;
		} else {
		//	printk("--CWMCU-- flush => i2c ok~!!\n");
			error = 5;
		}
	}
    cwmcu_powermode_switch(SWITCH_POWER_BATCH, 0);

	return count;
}

static ssize_t flush_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "--CWMCU-- %s in\n", __func__);
	return snprintf(buf, 255, "flush_list = 0x%08x\n",sensor->flush_list);
}

static ssize_t set_firmware_update_i2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int intsize = sizeof(int);
	memcpy(&sensor->cw_i2c_rw, buf, intsize);
	memcpy(&sensor->cw_i2c_len, &buf[4], intsize);
	memcpy(sensor->cw_i2c_data, &buf[8], sensor->cw_i2c_len);

	return count;
}

static ssize_t get_firmware_update_i2(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;
	if (sensor->cw_i2c_rw) {
		if (CWMCU_write_serial(sensor->cw_i2c_data, sensor->cw_i2c_len) < 0) {
			status = -1;
		}
		memcpy(buf, &status, sizeof(int));
		return 4;
	} else {
		if (CWMCU_read_serial(sensor->cw_i2c_data, sensor->cw_i2c_len) < 0) {
			status = -1;
			memcpy(buf, &status, sizeof(int));
			return 4;
		}
		memcpy(buf, &status, sizeof(int));
		memcpy(&buf[4], sensor->cw_i2c_data, sensor->cw_i2c_len);
		return 4+sensor->cw_i2c_len;
	}
	return  0;
}

static ssize_t firmware_update_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("--CWMCU-- %s in\n", __func__);
	return snprintf(buf, 8, "%d\n", sensor->fwupdate_status);
}

static ssize_t firmware_update_cmd_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    bool ret = false;
    sscanf(buf, "%d\n", &sensor->cmd);
    printk(KERN_DEBUG "CWMCU cmd=%d \n", sensor->cmd);

    switch (sensor->cmd) {

    case CHANGE_TO_BOOTLOADER_MODE:
		printk("CWMCU CHANGE_TO_BOOTLOADER_MODE\n");
		sensor->mcu_mode = CW_BOOT;
		disable_irq_wake(sensor->client->irq);
		/* need to config irq to output for AP */
		gpio_direction_output(sensor->irq_gpio, 0);

		/* reset mcu */
		gpio_set_value(sensor->boot_gpio, 1);
        gpio_direction_output(sensor->reset_gpio, 1);
		usleep_range(19000, 20000);
		gpio_set_value(sensor->reset_gpio, 0);
		usleep_range(19000, 20000);
		gpio_set_value(sensor->reset_gpio, 1);

		sensor->client->addr = 0x72 >> 1;
		usleep_range(19000, 20000);
		gpio_direction_input(sensor->irq_gpio);
        break;

    case CHANGE_TO_NORMAL_MODE:
        printk("CWMCU CHANGE_TO_NORMAL_MODE\n");

        /* boot low  reset high */
		gpio_set_value(sensor->boot_gpio, 0);
		gpio_set_value(sensor->reset_gpio, 1);
		mdelay(10);
		gpio_set_value(sensor->reset_gpio, 0);
		mdelay(10);
		gpio_set_value(sensor->reset_gpio, 1);
		mdelay(10);
		gpio_direction_input(sensor->reset_gpio);

		mdelay(80);
		sensor->client->addr = 0x3a;
		sensor->mcu_mode = CW_NORMAL;

        if (ret == true) {
            sensor->fwupdate_status = 2;
        } else {
            sensor->fwupdate_status = -1;
        }
        if (cwmcu_get_version())
            sensor->mcu_status = CW_INVAILD;
        else {
            sensor->mcu_status = CW_VAILD;
            enable_irq_wake(sensor->client->irq);
        }
        break;
     case MCU_RESET:
		printk("CWMCU MCU_RESET\n");
     	gpio_set_value(sensor->reset_gpio, 1);
		msleep(500);
		gpio_set_value(sensor->reset_gpio, 0);
		msleep(500);
		gpio_set_value(sensor->reset_gpio, 1);
		msleep(1000);
		break;
    default:
        break;
    }
    return count;
}

static ssize_t set_mcu_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u8 data[40] = {0};
    s16 data_buff[3] = {0};
    u8 regValue;
    u32 McuReg = 0x80;
    int i;

    sscanf(buf, "%d %x %d\n", &sensor->cmd, &sensor->addr, &sensor->value);
    McuReg = sensor->addr;
    regValue = sensor->value;
    printk(KERN_DEBUG "CWMCU cmd=%d addr=0x%x value=%d\n", sensor->cmd, McuReg, sensor->value);

	cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 1);

    switch (sensor->cmd) {

    case MCU_REGISTER_WRITE_CTRL:
        printk("CWMCU MCU_REGISTER_WRITE_CTRL\n");
        if (CWMCU_i2c_write(sensor, McuReg, &regValue, 1) < 0) {
            printk("--CWMCU-- Write MCU Reg error.\n");
        } else {
            printk("--CWMCU-- Write MCU Reg=0x%x: value=%d\n",McuReg,regValue);
        }
        break;

    case MCU_REGISTER_READ_CTRL:
        printk("CWMCU MCU_REGISTER_READ_CTRL\n");
        if (CWMCU_i2c_read(sensor, McuReg, data, regValue) < 0) {
            printk("--CWMCU-- Read MCU Reg error.\n");
        } else {
            printk("--CWMCU-- Read MCU Reg=0x%x: read_size=%d\n",McuReg,regValue);
            for(i=0; i<regValue; i++) {
                printk("Read: data[%d] = %2x\n",i,data[i]);
            }
        }
        break;

    case CHECK_ACC_DATA:
        printk(KERN_DEBUG "CWMCU CHECK_ACC_DATA\n");
        if (CWMCU_i2c_read(sensor, CW_READ_ACCELERATION, data, 6) >= 0) {
            data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
            data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
            data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

            printk("--CWMCU-- ACC_DATA: x = %d, y = %d, z = %d\n",
                data_buff[0], data_buff[1], data_buff[2]);
        }
        break;

    case CHECK_MAG_DATA:
        printk(KERN_DEBUG "CWMCU CHECK_MAG_DATA\n");
        if (CWMCU_i2c_read(sensor, CW_READ_MAGNETIC, data, 6) >= 0) {
            data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
            data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
            data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

            printk("--CWMCU-- MAG_DATA: x = %d, y = %d, z = %d\n",
                data_buff[0], data_buff[1], data_buff[2]);
        }
        break;

    case CHECK_GYRO_DATA:
        printk(KERN_DEBUG "CWMCU CHECK_GYRO_DATA\n");
        if (CWMCU_i2c_read(sensor, CW_READ_GYRO, data, 6) >= 0) {
            data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
            data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
            data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

            printk("--CWMCU-- GYRO_DATA: d[0-6] = %2x %2x %2x %2x %2x %2x\n",
                data[0],data[1],data[2],data[3],data[4],data[5]);
        }
        break;

    default:
            break;
    }
    cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 0);
    return count;
}

/* get calibrator data */
static ssize_t get_calibrator_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    int i = 0;
    uint8_t data[32] = {0};

    data[0] = sensor->cal_cmd;
    data[1] = sensor->cal_type;
    data[2] = sensor->cal_id;

	if ((sensor->mcu_mode == CW_BOOT) || (sensor->mcu_status == CW_INVAILD)) {
		printk(KERN_ERR "--CWMCU--%s sensor->mcu_mode = %d cmd: %d %d %d\n", __func__, sensor->mcu_mode, data[0], data[1], data[2]);
		return sprintf(buf, "128 %derror\n", sensor->mcu_mode);
	}
	cwmcu_powermode_switch(SWITCH_POWER_CALIB, 1);

    switch (sensor->cal_cmd) {

    case CWMCU_CALIBRATOR_STATUS:
        printk("--CWMCU-- CWMCU_CALIBRATOR_STATUS\n");
        if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_STATUS, &data[3], 1) >= 0) {
            printk("--CWMCU-- calibrator status = %d\n", data[3]);
			cwmcu_powermode_switch(SWITCH_POWER_CALIB, 0);
            return sprintf(buf, "%d\n",data[3]);
        } else {
            printk("--CWMCU-- calibrator => i2c error, calibrator status = %d\n", data[3]);
			cwmcu_powermode_switch(SWITCH_POWER_CALIB, 0);
            return sprintf(buf, "calibrator i2c error: %d\n",data[3]);
        }
        break;

    case CWMCU_CALIBRATOR_ACCELERATION:
        printk("--CWMCU-- CWMCU_CALIBRATOR_ACCELERATION read data\n");
        if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_GET_BIAS_ACC, &data[3], 6) <= 0) {
            printk("--CWMCU-- i2c calibrator read fail!!! [ACC]\n");
        }
        break;

    case CWMCU_CALIBRATOR_MAGNETIC:
/*
        printk("--CWMCU-- CWMCU_CALIBRATOR_MAGNETIC read data\n");
        if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_GET_BIAS_MAG, &data[9], 30) <= 0) {
            printk("--CWMCU-- i2c calibrator read fail!!! [MAG]\n");
        }
*/
        break;

    case CWMCU_CALIBRATOR_GYRO:
        printk("--CWMCU-- CWMCU_CALIBRATOR_GYRO read data\n");
        if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_GET_BIAS_GYRO, &data[3], 6) <= 0) {
            printk("--CWMCU-- i2c calibrator read fail!!! [GYRO]\n");
        }
        break;

    case CWMCU_CALIBRATOR_LIGHT:
        printk("--CWMCU-- CWMCU_CALIBRATOR_LIGHT read data\n");
        if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_GET_BIAS_LIGHT, &data[3], 6) <= 0) {
            printk("--CWMCU-- i2c calibrator read fail!!! [LIGHT]\n");
        }
        break;

    case CWMCU_CALIBRATOR_PROXIMITY:
        printk("--CWMCU-- CWMCU_CALIBRATOR_PROXIMITY read data\n");
        if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_GET_BIAS_PROXIMITY, &data[3], 6) <= 0) {
            printk("--CWMCU-- i2c calibrator read fail!!! [PROXIMITY]\n");
        }
        break;

    case CWMCU_CALIBRATOR_PRESSURE:
        printk("--CWMCU-- CWMCU_CALIBRATOR_PRESSURE read data\n");
        if (CWMCU_i2c_read(sensor, CW_CALIBRATOR_GET_BIAS_PRESSURE, &data[3], 6) <= 0) {
            printk("--CWMCU-- i2c calibrator read fail!!! [PRESSURE]\n");
        }
        break;
	default:
		printk("CWMCU calibrator read parameter invalid\n");
		return sprintf(buf, "0 invalid parameter");
    }

    for (i = 0; i < 32; i++) {
        printk(KERN_DEBUG "--CWMCU-- castor read data[%d] = %u\n", i, data[i]);
    }

	cwmcu_powermode_switch(SWITCH_POWER_CALIB, 0);

    return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
            data[0], data[1], data[2],
            data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12],
            data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22],
            data[23], data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31]);

}

static ssize_t set_calibrator_data(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    int i = 0;
    uint8_t data[32] = {0};
    int temp[32] = {0};

    char source[512];
    char *pch;
    int buf_count=0;

    char *myBuf= source;

    sensor->cal_cmd = -1;
	if ((sensor->mcu_mode == CW_BOOT) || (sensor->mcu_status == CW_INVAILD)) {
		printk(KERN_ERR "--CWMCU--%s sensor->mcu_mode = %d\n", __func__, sensor->mcu_mode);
		return count;
	}

	cwmcu_powermode_switch(SWITCH_POWER_CALIB, 1);

    strcpy(source,buf);
    //printk("--CWMCU-- source = %s | count:%d\n", source, count);

    while ((pch = strsep(&myBuf, ", ")) != NULL) {
        buf_count++;
    }
    //printk("--CWMCU-- buf = %s | bufcount:%d\n", buf, buf_count);

    /* Command mode */
    if (buf_count == 3) {
        sscanf(buf, "%d %d %d",&temp[0], &temp[1], &temp[2]);
        sensor->cal_cmd = (uint8_t)temp[0];
        sensor->cal_type = (uint8_t)temp[1];
        sensor->cal_id = (uint8_t)temp[2];
        printk("--CWMCU-- Calibrator=> cmd:%d type:%d id:%d\n", sensor->cal_cmd, sensor->cal_type, sensor->cal_id);

        if (sensor->cal_cmd == CWMCU_CALIBRATOR_ENABLE) {
            /* type=> 1=calibrator 2=selftest */
            printk("--CWMCU-- Calibrator=> set calibrator info\n");
            CWMCU_i2c_write(sensor, CW_CALIBRATOR_TYPE, &sensor->cal_type, 1);
            CWMCU_i2c_write(sensor, CW_CALIBRATOR_SENSOR_ID, &sensor->cal_id, 1);
        } else {
            printk(KERN_DEBUG "--CWMCU-- set command\n");
            return count;
        }
    } else if (buf_count == 4) {  /* write calibrator bias to mcu */
        sscanf(buf, "%d %d %d %d\n",&temp[0], &temp[1], &temp[2],&temp[3]);

        for (i = 0; i < 4; i++) {
            data[i] = (uint8_t)temp[i];
        }

        sensor->cal_cmd = data[0];
        sensor->cal_type = data[1];
        sensor->cal_id = data[2];

        printk("--CWMCU-- Calibrator=> set command=%d , type=%d, sensorid=%d\n", sensor->cal_cmd, sensor->cal_type, sensor->cal_id);

        switch (sensor->cal_cmd) {

        case CWMCU_CALIBRATOR_WRITE_BIAS:
            printk("--CWMCU-- CWMCU_CALIBRATOR_ACCELERATION write data\n");
            CWMCU_i2c_write(sensor, CW_CALIBRATOR_SET_BIAS, &sensor->cal_id, 1);
            break;
        }
    } else {
        printk("--CWMCU-- input parameter incorrect !!! | %d\n",(int)count);
        return count;
    }
	cwmcu_powermode_switch(SWITCH_POWER_CALIB, 0);

    return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t data[2] = {0};

    printk("--CWMCU-- %s\n", __func__);

    if (cwmcu_get_version())
        sensor->mcu_status = CW_INVAILD;
    else {
        sensor->mcu_status = CW_VAILD;
		cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 1);
        if (CWMCU_i2c_read(sensor, CW_FWVERSION, data, 2) >= 0) {
            printk("CHECK_FIRMWAVE_VERSION : %d.%d\n", data[0],data[1]);
        }
        cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 0);
    }

    return snprintf(buf, 8, "%d.%d\n", data[0], data[1]);
}

static ssize_t timestamp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t data[4] = {0,};
	uint32_t timestamp = 0;

    printk("--CWMCU-- %s\n", __func__);

	if (sensor->mcu_mode == CW_NORMAL) {
		cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 1);
        if (CWMCU_i2c_read(sensor, CW_MCU_TIMESTAMP, data, 4) >= 0) {
			timestamp = (uint32_t)(data[3]<<24) | (uint32_t)(data[2]<<16) | (uint32_t)(data[1]<<8) | (uint32_t)data[0];
			printk("--CWMCU-- mcu timestamp:%u\n", timestamp);
		}
		cwmcu_powermode_switch(SWITCH_POWER_NORMAL, 0);
    }

    return snprintf(buf, 32, "%u\n", timestamp);
}

static ssize_t version_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	char data[50];

	printk("--CWMCU-- %s\n", __func__);
	sscanf(buf, "%s\n", data);

	snprintf(sensor->firmware_path, sizeof(sensor->firmware_path), "%s\n", data);

	printk("--CWMCU-- sensor->firmware_path :%s, data : %s\n", sensor->firmware_path, data);

	return count;
}

static ssize_t verbose_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    printk("--CWMCU-- %s\n", __func__);

    return snprintf(buf, 50, "%d\n", flagVerbose);
}

static ssize_t verbose_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%d\n", &flagVerbose);
    printk("--CWMCU-- %s, parsed %d\n", __func__, flagVerbose);

    return count;
}

static ssize_t power_control_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	uint8_t data;

	printk("--CWMCU-- %s\n", __func__);
	sscanf(buf, "%s\n", &data);

	cwmcu_powermode_switch(SWITCH_POWER_PCBA, 1);
	CWMCU_i2c_write(sensor, CW_POWER_CTRL, &data, 1);
	cwmcu_powermode_switch(SWITCH_POWER_PCBA, 0);

	return count;
}

static ssize_t psensor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t data = 0;

    printk("--CWMCU-- %s\n", __func__);

    if (CWMCU_i2c_read(sensor, CW_PSENSOR_RAW_DATA_GET, &data, 1) >= 0) {
        printk("Get P-sensor RAWDATA: %d\n", data);
    }

    return snprintf(buf, 8, "%d\n", data);
}

static ssize_t product_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	switch(sensor->product_flag) {
		case GET_VERSION:
			if (sensor->mcu_mode == CW_BOOT) {
				return snprintf(buf, 5, "0.2\n");
			} else if (sensor->mcu_status == CW_INVAILD){
				return snprintf(buf, 5, "0.3\n");
			} else {
				if (version[1] < 10)
					return snprintf(buf, 10, "%d.0%d\n", version[0], version[1]);
				else
					return snprintf(buf, 10, "%d.%d\n", version[0], version[1]);
			}
			break;
		case GET_ACCEL_STATUS:
			return sprintf(buf, "%d,%d,%d\n", last_data[CW_ACCELERATION][0], last_data[CW_ACCELERATION][1], last_data[CW_ACCELERATION][2]);
			break;
		case GET_COMPASS_STATUS:
			return sprintf(buf, "%d,%d,%d\n", last_data[CW_MAGNETIC][0], last_data[CW_MAGNETIC][1], last_data[CW_MAGNETIC][2]);
			break;
		case GET_GYRO_STATUS:
			return sprintf(buf, "%d,%d,%d\n", last_data[CW_GYRO][0], last_data[CW_GYRO][1], last_data[CW_GYRO][2]);
			break;
		case GET_LGT_STATUS:
			return sprintf(buf, "%d,%d,%d\n", last_data[CW_PROXIMITY][0], last_data[CW_PROXIMITY][1], last_data[CW_PROXIMITY][2]);
			break;
		default:
			break;
	}
	return sprintf(buf, "unknown\n");
}

static ssize_t product_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int product_flag;

	sscanf(buf, "%d", &product_flag);
	if (product_flag !=0)
		sensor->product_flag = product_flag;
	printk("CWMCU set flag %d\n", product_flag);

	return count;
}

static DEVICE_ATTR(enable, 0666, active_show, active_set);
static DEVICE_ATTR(delay_ms, 0666, interval_show, interval_set);

static DEVICE_ATTR(batch, 0666, batch_show, batch_set);
static DEVICE_ATTR(flush, 0666, flush_show, flush_set);
static DEVICE_ATTR(firmware_update_i2c, 0666, get_firmware_update_i2, set_firmware_update_i2);
static DEVICE_ATTR(firmware_update_cmd, 0666, firmware_update_cmd_show, firmware_update_cmd_set);
static DEVICE_ATTR(mcu_cmd, 0666, NULL, set_mcu_cmd);
static DEVICE_ATTR(calibrator_cmd, 0666, get_calibrator_data, set_calibrator_data);
static DEVICE_ATTR(version, 0666, version_show, version_set);
static DEVICE_ATTR(verbose, 0666, verbose_show, verbose_set);
static DEVICE_ATTR(timestamp, 0666, timestamp_show, NULL);

static DEVICE_ATTR(power_control, 0666, NULL, power_control_set);
static DEVICE_ATTR(psensor, 0644, psensor_show, NULL);
static DEVICE_ATTR(product, 0666, product_show, product_set);

static struct attribute *sysfs_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay_ms.attr,
	&dev_attr_batch.attr,
	&dev_attr_flush.attr,
	&dev_attr_firmware_update_i2c.attr,
	&dev_attr_firmware_update_cmd.attr,
	&dev_attr_mcu_cmd.attr,
	&dev_attr_calibrator_cmd.attr,
	&dev_attr_version.attr,
	&dev_attr_verbose.attr,
	&dev_attr_timestamp.attr,

	&dev_attr_power_control.attr,
	&dev_attr_psensor.attr,
	&dev_attr_product.attr,
	NULL
};

static struct attribute_group sysfs_attribute_group = {
	.attrs = sysfs_attributes
};

/*=======input device==========*/

static void CWMCU_init_input_device(struct CWMCU_data *sensor, struct input_dev *idev)
{
	idev->name = CWMCU_I2C_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &sensor->client->dev;
	idev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_ABS);
	set_bit(EV_KEY, idev->evbit);

	set_bit(EV_ABS, idev->evbit);
	input_set_abs_params(idev, CW_ABS_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_X1, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_Y1, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_Z1, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_TIMEDIFF, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_ACCURACY, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_TIMESTAMP, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, CW_ABS_TIMESYNC, -DPS_MAX, DPS_MAX, 0, 0);
}

/*=======polling device=========*/
static void CWMCU_poll(struct input_polled_dev *dev)
{
	#ifndef CWMCU_INTERRUPT
	CWMCU_read(dev->private);
	#endif
}

static int CWMCU_open(struct CWMCU_data *sensor)
{
	int error;
	error = pm_runtime_get_sync(&sensor->client->dev);
	if (error && error != -ENOSYS)
		return error;
	return 0;
}

static void CWMCU_close(struct CWMCU_data *sensor)
{
	pm_runtime_put_sync(&sensor->client->dev);
}

static void CWMCU_poll_open(struct input_polled_dev *ipoll_dev)
{
	struct CWMCU_data *sensor = ipoll_dev->private;
	CWMCU_open(sensor);
}

static void CWMCU_poll_close(struct input_polled_dev *ipoll_dev)
{
	struct CWMCU_data *sensor = ipoll_dev->private;
	CWMCU_close(sensor);
}

static int CWMCU_register_polled_device(struct CWMCU_data *sensor)
{
	int error = -1;
	struct input_polled_dev *ipoll_dev;

	/* poll device */
	ipoll_dev = input_allocate_polled_device();
	if (!ipoll_dev)
		return -ENOMEM;

	ipoll_dev->private = sensor;
	ipoll_dev->open = CWMCU_poll_open;
	ipoll_dev->close = CWMCU_poll_close;
	ipoll_dev->poll = CWMCU_poll;
	ipoll_dev->poll_interval = CWMCU_POLL_INTERVAL;
	ipoll_dev->poll_interval_min = CWMCU_POLL_MIN;
	ipoll_dev->poll_interval_max = CWMCU_POLL_MAX;

	CWMCU_init_input_device(sensor, ipoll_dev->input);

	error = input_register_polled_device(ipoll_dev);
	if (error) {
		input_free_polled_device(ipoll_dev);
		return error;
	}

	sensor->input_polled = ipoll_dev;
	sensor->input = ipoll_dev->input;

	return 0;
}

static int CWMCU_suspend(struct device *dev)
{
	printk(KERN_DEBUG "--CWMCU--%s\n", __func__);
	return 0;
}

static int CWMCU_resume(struct device *dev)
{
	printk(KERN_DEBUG "--CWMCU--%s\n", __func__);
	return 0;
}

#ifdef CWMCU_INTERRUPT
static irqreturn_t CWMCU_irq_handler(int irq, void *dev_id)
{
    return IRQ_WAKE_THREAD;
}
static irqreturn_t CWMCU_interrupt_thread(int irq, void *data)
{
	VERBOSE(KERN_DEBUG "--CWMCU--%s in\n", __func__);
	if ((sensor->mcu_mode == CW_BOOT) || (sensor->mcu_status == CW_INVAILD)) {
		printk(KERN_DEBUG "--CWMCU--%s sensor->mcu_mode = %d\n", __func__, sensor->mcu_mode);
		return IRQ_HANDLED;
	}
	schedule_work(&sensor->work);

	return IRQ_HANDLED;
}

/*  function for interrupt work  */
static void cwmcu_work_report(struct work_struct *work)
{
	uint8_t temp[6] = {0};
	uint8_t mStatus;

	VERBOSE(KERN_DEBUG "--CWMCU--%s in\n", __func__);

	wake_lock_timeout(&sensor->data_report_lock, 10*HZ);

	if ((sensor->mcu_mode == CW_BOOT) || (sensor->mcu_status == CW_INVAILD)) {
		printk(KERN_DEBUG "--CWMCU--%s sensor->mcu_mode = %d\n", __func__, sensor->mcu_mode);
		return;
	}

	cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT, 1);

	/* check mcu interrupt status */
	if (CWMCU_i2c_read(sensor, CW_INTERRUPT_STATUS, temp, 6) >= 0) {
		sensor->interrupt_status = (u32)temp[1] << 8 | (u32)temp[0];
		sensor->update_list = (u32)temp[5] << 24 | (u32)temp[4] << 16 | (u32)temp[3] << 8 | (u32)temp[2];
		VERBOSE( "--CWMCU-- sensor->interrupt_status~ = %d\n", sensor->interrupt_status);
	} else {
		printk( "--CWMCU-- check interrupt_status failed~!!\n");
		sensor->interrupt_status = 0;
	}

    /* MCU reset */
    if (sensor->interrupt_status & (1<<INTERRUPT_INIT)) {
		if (CWMCU_i2c_read(sensor, CW_MCU_STATUS_GET, &temp[0], 1) >= 0) {
			mStatus = temp[0];
			if(mStatus==0) {
				printk( "--CWMCU-- Check MCU Reset => %d\n",sensor->mcu_reset);
				if(sensor->mcu_reset > 0) {
					cwmcu_restore_status();
					printk( "--CWMCU-- Check MCU restore status.\n");
				}
				sensor->mcu_reset++;
			}
			else if(mStatus==1) {
				cwmcu_check_mcu_enable();
			}
		}
		else {
			printk( "--CWMCU-- Check MCU status i2c fail.\n");
		}
    }

	/* MCU time sync */
    if (sensor->interrupt_status & (1<<INTERRUPT_TIME_SYNC)) {
		cwmcu_send_time_sync_event();
		printk( "--CWMCU-- MCU Time sync event send.\n");
    }

    /* read MCU error log data */
    if (sensor->interrupt_status & (1<<INTERRUPT_ERROR_LOG)) {
        cwmcu_error_log_read(sensor);
    }

    /* read MCU debug log data */
    if (sensor->interrupt_status & (1<<INTERRUPT_DEBUG_LOG)) {
        cwmcu_debug_log_read(sensor);
    }

    /* read sensor data of batch mode*/
    if (sensor->interrupt_status & (1<<INTERRUPT_BATCHTIMEOUT)
        || sensor->interrupt_status & (1<<INTERRUPT_BATCHFULL)
        || sensor->interrupt_status & (1<<INTERRUPT_BATCH_FLUSH)
        || sensor->interrupt_status & (1<<INTERRUPT_BATCH_TIMESTAMP_SYNC)) {
        cwmcu_batch_read(sensor);
    }

	/* read gesture event */
	if (sensor->interrupt_status & (1<<INTERRUPT_GESTURE)) {
		VERBOSE( "--CWMCU-- INTERRUPT_GESTURE~!!\n");
		cwmcu_gesture_read(sensor);
	}

	/* read sensor data of normal mode*/
	if (sensor->interrupt_status & (1<<INTERRUPT_DATAREADY)) {
		VERBOSE( "--CWMCU-- INTERRUPT_DATAREADY~!!\n");
		CWMCU_read(sensor);
	}
	cwmcu_powermode_switch(SWITCH_POWER_INTERRUPT, 0);
}
#endif

static int cwstm_parse_dt(struct device *dev,
			 struct CWMCU_data *sensor)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = of_get_named_gpio(np, "cwstm,irq-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"cwstm,irq_gpio\"\n");
		goto err;
	}
	sensor->irq_gpio = ret;

	ret = of_get_named_gpio(np, "cwstm,boot-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"boot\"\n");
		goto err;
	}
	sensor->boot_gpio = ret;

	ret = of_get_named_gpio(np, "cwstm,reset-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"reset\"\n");
		goto err;
	}
	sensor->reset_gpio = ret;

	ret = of_get_named_gpio(np, "cwstm,wakeup-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"wakeup\"\n");
		goto err;
	}
	sensor->wakeup_gpio = ret;

	ret = of_get_named_gpio(np, "cwstm,compass-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"compass\"\n");
	}
	sensor->compass_gpio = ret;

	ret = of_get_named_gpio(np, "cwstm,acc-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"acc\"\n");
	}
	sensor->acc_gpio = ret;

	ret = of_get_named_gpio(np, "cwstm,als-gpio", 0);
	if (ret < 0) {
		pr_err("failed to get \"als\"\n");
	}
	sensor->als_gpio = ret;
err:
	return ret;
}
static int cwstm_config_gpio(struct CWMCU_data *sensor)
{
	int ret = 0;
	
	if(gpio_is_valid(sensor->wakeup_gpio)){
		ret = gpio_request(sensor->wakeup_gpio, "cwstm,wakeup-gpio");
		gpio_direction_output(sensor->wakeup_gpio, 0);
	}
	if(gpio_is_valid(sensor->irq_gpio)){
		ret = gpio_request(sensor->irq_gpio, "cwstm,irq-gpio");
		gpio_direction_input(sensor->irq_gpio);
	}
	if(gpio_is_valid(sensor->reset_gpio))
		ret = gpio_request(sensor->reset_gpio, "cwstm,reset-gpio");
	if(gpio_is_valid(sensor->boot_gpio))
		ret = gpio_request(sensor->boot_gpio, "cwstm,boot-gpio");

	if(gpio_is_valid(sensor->compass_gpio)){
		ret = gpio_request(sensor->compass_gpio, "cwstm,compass-gpio");
		gpio_direction_input(sensor->compass_gpio);
	}
	if(gpio_is_valid(sensor->acc_gpio)){
		ret = gpio_request(sensor->acc_gpio, "cwstm,acc-gpio");
		gpio_direction_input(sensor->acc_gpio);
	}
	if(gpio_is_valid(sensor->als_gpio)){
		ret = gpio_request(sensor->als_gpio, "cwstm,als-gpio");
		gpio_direction_input(sensor->als_gpio);
	}
	return ret;
}
static void cwstm_unconfig_gpio(struct CWMCU_data *sensor)
{
	gpio_free(sensor->wakeup_gpio);
	gpio_free(sensor->irq_gpio);
	gpio_free(sensor->reset_gpio);
	gpio_free(sensor->boot_gpio);
	gpio_free(sensor->compass_gpio);
	gpio_free(sensor->acc_gpio);
	gpio_free(sensor->als_gpio);
}
static void cwstm_reset(struct CWMCU_data *sensor)
{
	gpio_direction_output(sensor->reset_gpio, 1);
	gpio_direction_output(sensor->boot_gpio, 1);
	msleep(10);
	gpio_set_value(sensor->boot_gpio, 0);
	gpio_set_value(sensor->reset_gpio, 1);
	msleep(10);
	gpio_set_value(sensor->reset_gpio, 0);
	msleep(10);
	gpio_set_value(sensor->reset_gpio, 1);
	msleep(10);
	gpio_direction_input(sensor->reset_gpio);
}
#if 0
static int cwstm_power_on(bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(sensor->vdd);
	if (rc) {
		dev_err(&sensor->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(sensor->vcc_i2c);
	if (rc) {
		dev_err(&sensor->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(sensor->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(sensor->vdd);
	if (rc) {
		dev_err(&sensor->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(sensor->vcc_i2c);
	if (rc) {
		dev_err(&sensor->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		regulator_enable(sensor->vdd);
	}

	return rc;
}

static int cwstm_power_init(bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	sensor->vdd = regulator_get(&sensor->client->dev, "cwstm,vdd_ana");
	if (IS_ERR(sensor->vdd)) {
		rc = PTR_ERR(sensor->vdd);
		dev_err(&sensor->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(sensor->vdd) > 0) {
		rc = regulator_set_voltage(sensor->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&sensor->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	sensor->vcc_i2c = regulator_get(&sensor->client->dev, "cwstm,vcc_i2c");
	if (IS_ERR(sensor->vcc_i2c)) {
		rc = PTR_ERR(sensor->vcc_i2c);
		dev_err(&sensor->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(sensor->vcc_i2c) > 0) {
		rc = regulator_set_voltage(sensor->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&sensor->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(sensor->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(sensor->vdd) > 0)
		regulator_set_voltage(sensor->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(sensor->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(sensor->vdd) > 0)
		regulator_set_voltage(sensor->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(sensor->vdd);

	if (regulator_count_voltages(sensor->vcc_i2c) > 0)
		regulator_set_voltage(sensor->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(sensor->vcc_i2c);
	return 0;
}
#endif
static int  CWMCU_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int error;
	int i = 0;
	struct pinctrl *pinctrl;

	printk(KERN_DEBUG "--CWMCU-- %s\n", __func__);

	dev_dbg(&client->dev, "%s:\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "--CWMCU-- i2c_check_functionality error\n");
		return -EIO;
	}
    pinctrl = devm_pinctrl_get_select_default(&client->dev);
    if (IS_ERR(pinctrl)) {
         dev_warn(&client->dev, "pins are not configured from the driver\n");
         return -EINVAL;
    } 
	sensor = kzalloc(sizeof(struct CWMCU_data), GFP_KERNEL);
	if (!sensor) {
		dev_err(&client->dev, "--CWMCU-- kzalloc error\n");
		return -ENOMEM;
	}

	sensor->client = client;
	i2c_set_clientdata(client, sensor);

	error = cwstm_parse_dt(&client->dev, sensor);
	if (error < 0) {
		pr_err("failed to parse device tree: %d\n", error);
		goto err_parse_dt;
	}

	error = cwstm_config_gpio(sensor);
	if (error < 0) {
		dev_err(&client->dev, "failed to request gpio %d\n", error);
		goto err_parse_dt;
	}
	error = sensor_power_onoff(true);
	if (error) {
    	dev_err(&client->dev, "power on failed");
    	goto err_power_on;
	} 
/*
	error = cwstm_power_init(true);
	if (error) {
		dev_err(&client->dev, "power init failed");
	}

	error = cwstm_power_on(true);
	if (error) {
		dev_err(&client->dev, "power on failed");
	}
*/
	/* mcu reset */
	cwstm_reset(sensor);

	error = CWMCU_register_polled_device(sensor);
	if (error) {
		dev_err(&client->dev, "--CWMCU-- CWMCU_register_polled_device error");
		goto err_register_polled_device;
	}

	error = sysfs_create_group(&sensor->input->dev.kobj,
					&sysfs_attribute_group);
	if (error)
		goto exit_free_input;

	for (i = 0; i < CW_SENSORS_ID_END; i++) {
		sensor->sensors_time[i] = 0;
		sensor->report_period[i] = 20000;
		sensor->time_diff[i] = 0;
	}
	
	sensor->mcu_mode = CW_NORMAL;
	sensor->current_timeout = 0;
	sensor->timeout_count = 0;
	wake_lock_init(&sensor->data_report_lock, WAKE_LOCK_SUSPEND, "cwmcu-wake-lock");

#ifdef CWMCU_INTERRUPT
	sensor->client->irq = gpio_to_irq(sensor->irq_gpio);

	printk(KERN_DEBUG "--CWMCU--sensor->client->irq  =%d~!!\n", sensor->client->irq);

	if (sensor->client->irq > 0) {

		error = request_threaded_irq(sensor->client->irq, CWMCU_irq_handler,
						   CWMCU_interrupt_thread,
						   IRQF_TRIGGER_RISING,
						   "cwmcu", sensor);
		if (error < 0) {
				pr_err("request irq %d failed\n", sensor->client->irq);
				goto exit_request_irq;
			}
		INIT_WORK(&sensor->work, cwmcu_work_report);
		enable_irq_wake(sensor->client->irq);
	}
#endif

	pm_runtime_enable(&client->dev);
	/*
	if (cwmcu_get_version())
       sensor->mcu_status = CW_INVAILD;
    else
       sensor->mcu_status = CW_VAILD;
*/
	printk(KERN_DEBUG "--CWMCU-- CWMCU_i2c_probe success!\n");

	return 0;

exit_request_irq:
	 wake_lock_destroy(&sensor->data_report_lock);
exit_free_input:
	input_unregister_polled_device(sensor->input_polled);
	input_free_polled_device(sensor->input_polled);
err_register_polled_device:
	sensor_power_onoff(false);
err_power_on:
	cwstm_unconfig_gpio(sensor);
err_parse_dt:
	kfree(sensor);
	return error;
}

static int CWMCU_i2c_remove(struct i2c_client *client)
{
	struct CWMCU_data *sensor = i2c_get_clientdata(client);
	
	free_irq(sensor->client->irq, sensor);
	wake_lock_destroy(&sensor->data_report_lock);
	input_unregister_polled_device(sensor->input_polled);
	input_free_polled_device(sensor->input_polled);	
	cwstm_unconfig_gpio(sensor);
	kfree(sensor);
	sensor_power_onoff(false);
	return 0;
}

static struct of_device_id cwstm_match_table[] = {
	{ .compatible = "cwstm,cwstm32",},
	{ },
};

static const struct dev_pm_ops CWMCU_pm_ops = {
	.suspend = CWMCU_suspend,
	.resume = CWMCU_resume
};

static const struct i2c_device_id CWMCU_id[] = {
	{ CWMCU_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, CWMCU_id);

static struct i2c_driver CWMCU_driver = {
	.driver = {
		.name = CWMCU_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &CWMCU_pm_ops,
		.of_match_table = cwstm_match_table,
	},
	.probe    = CWMCU_i2c_probe,
	.remove   = CWMCU_i2c_remove,
	.id_table = CWMCU_id,
};

static int __init CWMCU_i2c_init(void){
	printk(KERN_DEBUG "CWMCU_i2c_init\n");
	return i2c_add_driver(&CWMCU_driver);
}

static void __exit CWMCU_i2c_exit(void){
	i2c_del_driver(&CWMCU_driver);
}

module_init(CWMCU_i2c_init);
module_exit(CWMCU_i2c_exit);

MODULE_DESCRIPTION("CWMCU I2C Bus Driver");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
