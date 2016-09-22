/* CWMCU.h - header file for CyWee digital 3-axis gyroscope
 *
 * Copyright (C) 2014 Cywee Motion Group Ltd.
 * Author: cywee-motion <cywee-motion@cywee.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __CWMCUSENSOR_H__
#define __CWMCUSENSOR_H__
#include <linux/ioctl.h>

#define CWMCU_I2C_NAME "CwMcuSensor"

enum ABS_status {
	CW_ABS_X = 0x01,
	CW_ABS_Y,
	CW_ABS_Z,
	CW_ABS_X1,
	CW_ABS_Y1,
	CW_ABS_Z1,
	CW_ABS_TIMEDIFF,
	CW_ABS_ACCURACY,
	//CW_ABS_TIMESTAMP_H,
	//CW_ABS_TIMESTAMP_L
	CW_ABS_TIMESTAMP,
	CW_ABS_TIMESYNC

};

enum MCU_mode {
	CW_NORMAL = 0x00,
	CW_SLEEP,
	CW_NO_SLEEP,
	CW_BOOT
};

enum MCU_status {
    CW_VAILD = 0x00,
    CW_INVAILD
};

/* power manager status */
typedef enum {
	SWITCH_POWER_ENABLE     = 0,
	SWITCH_POWER_DELAY,
	SWITCH_POWER_BATCH,
	SWITCH_POWER_NORMAL,
	SWITCH_POWER_CALIB,
	SWITCH_POWER_INTERRUPT,
	SWITCH_POWER_PCBA
} SWITCH_POWER_ID;

/* interrupt status */
typedef enum {
	INTERRUPT_NON                   = 0,
	INTERRUPT_INIT                  = 1,
	INTERRUPT_BATCHTIMEOUT          = 2,
	INTERRUPT_BATCHFULL             = 3,
    INTERRUPT_BATCH_FLUSH           = 4,
    INTERRUPT_BATCH_TIMESTAMP_SYNC  = 5,
	INTERRUPT_DATAREADY             = 6,
    INTERRUPT_GESTURE               = 7,
    INTERRUPT_ERROR_LOG             = 8,
    INTERRUPT_DEBUG_LOG             = 9,
    INTERRUPT_TIME_SYNC             = 10
} INTERRUPT_STATUS_LIST;

/* calibrator command */
typedef enum {
	CWMCU_CALIBRATOR_ACCELERATION   = 0,
	CWMCU_CALIBRATOR_MAGNETIC       = 1,
	CWMCU_CALIBRATOR_GYRO           = 2,
	CWMCU_CALIBRATOR_LIGHT          = 3,
	CWMCU_CALIBRATOR_PROXIMITY      = 4,
	CWMCU_CALIBRATOR_PRESSURE       = 5,
	CWMCU_CALIBRATOR_STATUS         = 6,
	CWMCU_CALIBRATOR_ENABLE         = 7,
	CWMCU_CALIBRATOR_WRITE_BIAS     = 8
} CALIBRATOR_CMD;

/* firmware command */
typedef enum {
    CHANGE_TO_BOOTLOADER_MODE = 1, /*  firmware upgrade command  */
    CHANGE_TO_NORMAL_MODE     = 2, /*  firmware upgrade command  */
    MCU_RESET		          = 3, /*  firmware upgrade command  */
} FIRMWARE_CMD;

/* mcu command */
typedef enum {
    MCU_REGISTER_WRITE_CTRL     = 1,
    MCU_REGISTER_READ_CTRL      = 2,
    CHECK_ACC_DATA              = 7,
    CHECK_MAG_DATA              = 8,
    CHECK_GYRO_DATA             = 9,
} MCU_CMD;

/* sensor id */
typedef enum {
	CW_ACCELERATION					= 0,
	CW_MAGNETIC						= 1,
	CW_GYRO							= 2,
	CW_LIGHT						= 3,
	CW_PROXIMITY					= 4,
	CW_PRESSURE						= 5,
	CW_ORIENTATION					= 6,
	CW_ROTATIONVECTOR				= 7,
	CW_LINEARACCELERATION			= 8,
	CW_GRAVITY						= 9,
	CW_STEP_COUNTER					= 10,
	CW_STEP_DETECTOR			    = 11,
	CW_MAGNETIC_UNCALIBRATED		= 12,
	CW_GYROSCOPE_UNCALIBRATED		= 13,
	CW_GAME_ROTATION_VECTOR			= 14,
	CW_GEOMAGNETIC_ROTATION_VECTOR	= 15,
	CW_SIGNIFICANT_MOTION			= 16,
	CW_SINGLE_SNAP					= 17,
	CW_DOUBLE_SNAP					= 18,
	CW_TAP							= 19,
	CW_CONTEXT_AWARE                = 20,
    CW_SCREEN_ON                    = 24,
	CW_FLIP                         = 26,
    CW_SCREEN_COVER                 = 27,
	CW_PHONE_CALL                   = 28,
	CW_SCREEN_MIRROR                = 29,
	CW_POCKET_MODE                  = 30,
	CW_SENSORS_ID_END,
	CW_META_DATA					= 99,
	CW_MAGNETIC_UNCALIBRATED_BIAS	= 100,
	CW_GYROSCOPE_UNCALIBRATED_BIAS	= 101
} CW_SENSORS_ID;


typedef enum {
    GET_VERSION			= 1, /*  get stored version */
    GET_ACCEL_STATUS    = 2, /*  get to know if accel hardware is ok  */
    GET_COMPASS_STATUS	= 3, /*  get to know if compass hardware is ok  */
    GET_GYRO_STATUS     = 4, /*  get to know if gyro hardware is ok  */
    GET_LGT_STATUS		= 5, /*  get to know if lgt hardware is ok  */
} PRODUCT_CMD;

#define CW_FWVERSION							0x3A

/*  0x01 ~ 0x0F  */
#define CW_ENABLE_REG                           0x01
#define CW_ENABLE_STATUS                        0x05
#define CW_SENSOR_DELAY_SET                     0x06    /* 2byte: byte1=>id, byte2=>ms*/
#define CW_SENSOR_DELAY_ID_SET                  0x07
#define CW_SENSOR_DELAY_GET                     0x08
#define CW_INTERRUPT_STATUS                     0x0F    /* 4Byte */

/*  0x10 ~ 0x1F  */
#define CW_BATCH_ENABLE_REG                     0x10
#define CW_BATCHENABLE_STATUS                   0x14
#define CW_BATCHTIMEOUT                         0x15    /* 5Byte ms */
#define CW_BATCHFLUSH                           0x16
#define CW_BATCHCOUNT                           0x17
#define CW_BATCHEVENT                           0x18
#define CW_BATCHTIME_SYNC                       0x19


#define	CWMCU_I2C_SENSORS_REG_START				(0x60)
#define CW_READ_ACCELERATION					(CWMCU_I2C_SENSORS_REG_START + CW_ACCELERATION)
#define CW_READ_MAGNETIC						(CWMCU_I2C_SENSORS_REG_START + CW_MAGNETIC)
#define CW_READ_GYRO							(CWMCU_I2C_SENSORS_REG_START + CW_GYRO)
#define CW_READ_LIGHT   						(CWMCU_I2C_SENSORS_REG_START + CW_LIGHT)
#define CW_READ_PROXIMITY    					(CWMCU_I2C_SENSORS_REG_START + CW_PROXIMITY)
#define CW_READ_PRESSURE	    				(CWMCU_I2C_SENSORS_REG_START + CW_PRESSURE)
#define CW_READ_ORIENTATION    					(CWMCU_I2C_SENSORS_REG_START + CW_ORIENTATION)
#define CW_READ_ROTATIONVECTOR    				(CWMCU_I2C_SENSORS_REG_START + CW_ROTATIONVECTOR)
#define CW_READ_LINEARACCELERATION				(CWMCU_I2C_SENSORS_REG_START + CW_LINEARACCELERATION)
#define CW_READ_GRAVITY   						(CWMCU_I2C_SENSORS_REG_START + CW_GRAVITY)
#define CW_READ_STEP_COUNTER					(CWMCU_I2C_SENSORS_REG_START + CW_STEP_COUNTER)
#define CW_READ_MAGNETIC_UNCALIBRATED			(CWMCU_I2C_SENSORS_REG_START + CW_MAGNETIC_UNCALIBRATED)
#define CW_READ_GYROSCOPE_UNCALIBRATED			(CWMCU_I2C_SENSORS_REG_START + CW_GYROSCOPE_UNCALIBRATED)
#define CW_READ_GAME_ROTATION_VECTOR			(CWMCU_I2C_SENSORS_REG_START + CW_GAME_ROTATION_VECTOR)
#define CW_READ_GEOMAGNETIC_ROTATION_VECTOR		(CWMCU_I2C_SENSORS_REG_START + CW_GEOMAGNETIC_ROTATION_VECTOR)

#define CW_READ_PROXIMITY_GESTURE				0x70
#define CW_READ_LIGHT_RGB						0x71
#define CW_READ_GESTURE_EVENT_COUNT				0x7A
#define CW_READ_GESTURE_EVENT_DATA				0x7B	/* read 2byte id: 1byte, data: 1byte */
#define CW_ACCURACY								0X7C


#define CW_CALIBRATOR_TYPE                      0x40
#define CW_CALIBRATOR_SENSOR_ID                 0x41
#define CW_CALIBRATOR_STATUS                    0x42
#define CW_CALIBRATOR_GET_BIAS_ACC              0x43
#define CW_CALIBRATOR_GET_BIAS_MAG              0x44
#define CW_CALIBRATOR_GET_BIAS_GYRO             0x45
#define CW_CALIBRATOR_GET_BIAS_LIGHT            0x46
#define CW_CALIBRATOR_GET_BIAS_PROXIMITY        0x47
#define CW_CALIBRATOR_GET_BIAS_PRESSURE         0x48
#define CW_CALIBRATOR_SET_BIAS                  0x49
/*
Power mode control :
  Register : 0x90
  Parameter:
    PMU_SLEEP      = 0
    PMU_DEEP_SLEEP = 1
    PMU_POWERDOWN  = 2
*/

#define CW_MCU_TIMESTAMP						0X83

#define CW_POWER_CTRL							0x90
#define CW_LED_CTRL								0x92

#define CW_MCU_STATUS_SET						0x94
#define CW_MCU_STATUS_GET						0x95

/* mcu log */
#define CW_DEBUG_INFO_GET       				0x96
#define CW_DEBUG_COUNT_GET      				0x97
#define CW_ERROR_INFO_GET       				0x98
#define CW_ERROR_COUNT_GET      				0x99
#define CW_PSENSOR_RAW_DATA_GET     			0x9A    /* read, 1 byte */

#define CW_SET_STEP_COUNTER  					0x9E    /* write, 4 bytes */

//#define CW_GYRO_TEST							0xB6

/* check data of queue if queue is empty */
#define CWMCU_NODATA							0xff

#define DPS_MAX			(1 << (16 - 1))
#ifdef __KERNEL__

struct CWMCU_platform_data {
	unsigned char Acceleration_hwid;
	unsigned char Acceleration_deviceaddr;
	unsigned char Acceleration_axes;
	unsigned char Magnetic_hwid;
	unsigned char Magnetic_deviceaddr;
	unsigned char Magnetic_axes;
	unsigned char Gyro_hwid;
	unsigned char Gyro_deviceaddr;
	unsigned char Gyro_axes;
};
#endif /* __KERNEL */

#endif /* __CWMCUSENSOR_H__ */
