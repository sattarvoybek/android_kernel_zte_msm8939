/* Lite-On LTR-559ALS Android / Linux Driver
 *
 * Copyright (C) 2013 Lite-On Technology Corp (Singapore)
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/version.h>
//#include <linux/earlysuspend.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h> //lijiangshuo add 20141231 for proc system
#include <linux/input-polldev.h> // lijiangshuo add 20150109
#include <linux/sensor_power.h>// lijiangshuo add 20150109

/* LTR-559 Registers */
#define LTR559_ALS_CONTR			0x80
#define LTR559_PS_CONTR			0x81
#define LTR559_PS_LED				0x82
#define LTR559_PS_N_PULSES			0x83
#define LTR559_PS_MEAS_RATE		0x84
#define LTR559_ALS_MEAS_RATE		0x85
#define LTR559_PART_ID				0x86
#define LTR559_MANUFACTURER_ID	0x87
#define LTR559_ALS_DATA_CH1_0		0x88
#define LTR559_ALS_DATA_CH1_1		0x89
#define LTR559_ALS_DATA_CH0_0		0x8A
#define LTR559_ALS_DATA_CH0_1		0x8B
#define LTR559_ALS_PS_STATUS		0x8C
#define LTR559_PS_DATA_0			0x8D
#define LTR559_PS_DATA_1			0x8E
#define LTR559_INTERRUPT			0x8F
#define LTR559_PS_THRES_UP_0		0x90
#define LTR559_PS_THRES_UP_1		0x91
#define LTR559_PS_THRES_LOW_0		0x92
#define LTR559_PS_THRES_LOW_1		0x93
#define LTR559_PS_OFFSET_1			0x94
#define LTR559_PS_OFFSET_0			0x95
#define LTR559_ALS_THRES_UP_0		0x97
#define LTR559_ALS_THRES_UP_1		0x98
#define LTR559_ALS_THRES_LOW_0	0x99
#define LTR559_ALS_THRES_LOW_1	0x9A
#define LTR559_INTERRUPT_PRST		0x9E
/* LTR-559 Registers */

#define SET_BIT 1
#define CLR_BIT 0

#define ALS 0
#define PS 1
#define ALSPS 2
//#define PS_W_SATURATION_BIT	3

/* address 0x80 */
#define ALS_MODE_ACTIVE	(1 << 0)
#define ALS_MODE_STDBY		(0 << 0)
#define ALS_SW_RESET		(1 << 1)
#define ALS_SW_NRESET		(0 << 1)
#define ALS_GAIN_1x			(0 << 2)
#define ALS_GAIN_2x			(1 << 2)
#define ALS_GAIN_4x			(2 << 2)
#define ALS_GAIN_8x			(3 << 2)
#define ALS_GAIN_48x		(6 << 2)
#define ALS_GAIN_96x		(7 << 2)
#define ALS_MODE_RDBCK		0
#define ALS_SWRT_RDBCK		1
#define ALS_GAIN_RDBCK			2
#define ALS_CONTR_RDBCK		3

/* address 0x81 */
#define PS_MODE_ACTIVE		(3 << 0)
#define PS_MODE_STDBY		(0 << 0)
#define PS_GAIN_16x			(0 << 2)
#define PS_GAIN_32x			(2 << 2)
#define PS_GAIN_64x			(3 << 2)
#define PS_SATUR_INDIC_EN	(1 << 5)
#define PS_SATU_INDIC_DIS	(0 << 5)
#define PS_MODE_RDBCK			0
#define PS_GAIN_RDBCK			1
#define PS_SATUR_RDBCK			2
#define PS_CONTR_RDBCK		3

/* address 0x82 */
#define LED_CURR_5MA			(0 << 0)
#define LED_CURR_10MA			(1 << 0)
#define LED_CURR_20MA			(2 << 0)
#define LED_CURR_50MA			(3 << 0)
#define LED_CURR_100MA			(4 << 0)
#define LED_CURR_DUTY_25PC	(0 << 3)
#define LED_CURR_DUTY_50PC	(1 << 3)
#define LED_CURR_DUTY_75PC	(2 << 3)
#define LED_CURR_DUTY_100PC	(3 << 3)
#define LED_PUL_FREQ_30KHZ		(0 << 5)
#define LED_PUL_FREQ_40KHZ		(1 << 5)
#define LED_PUL_FREQ_50KHZ		(2 << 5)
#define LED_PUL_FREQ_60KHZ		(3 << 5)
#define LED_PUL_FREQ_70KHZ		(4 << 5)
#define LED_PUL_FREQ_80KHZ		(5 << 5)
#define LED_PUL_FREQ_90KHZ		(6 << 5)
#define LED_PUL_FREQ_100KHZ	(7 << 5)
#define LED_CURR_RDBCK				0
#define LED_CURR_DUTY_RDBCK		1
#define LED_PUL_FREQ_RDBCK		2
#define PS_LED_RDBCK				3

/* address 0x84 */
#define PS_MEAS_RPT_RATE_50MS		(0 << 0)
#define PS_MEAS_RPT_RATE_70MS		(1 << 0)
#define PS_MEAS_RPT_RATE_100MS	(2 << 0)
#define PS_MEAS_RPT_RATE_200MS	(3 << 0)
#define PS_MEAS_RPT_RATE_500MS	(4 << 0)
#define PS_MEAS_RPT_RATE_1000MS	(5 << 0)
#define PS_MEAS_RPT_RATE_2000MS	(6 << 0)
#define PS_MEAS_RPT_RATE_10MS		(8 << 0)

/* address 0x85 */
#define ALS_MEAS_RPT_RATE_50MS	(0 << 0)
#define ALS_MEAS_RPT_RATE_100MS	(1 << 0)
#define ALS_MEAS_RPT_RATE_200MS	(2 << 0)
#define ALS_MEAS_RPT_RATE_500MS	(3 << 0)
#define ALS_MEAS_RPT_RATE_1000MS	(4 << 0)
#define ALS_MEAS_RPT_RATE_2000MS	(5 << 0)
#define ALS_INTEG_TM_100MS			(0 << 3)
#define ALS_INTEG_TM_50MS			(1 << 3)
#define ALS_INTEG_TM_200MS			(2 << 3)
#define ALS_INTEG_TM_400MS			(3 << 3)
#define ALS_INTEG_TM_150MS			(4 << 3)
#define ALS_INTEG_TM_250MS			(5 << 3)
#define ALS_INTEG_TM_300MS			(6 << 3)
#define ALS_INTEG_TM_350MS			(7 << 3)
#define ALS_MEAS_RPT_RATE_RDBCK		0
#define ALS_INTEG_TM_RDBCK			1
#define ALS_MEAS_RATE_RDBCK			2

/* address 0x86 */
#define PART_NUM_ID_RDBCK		0
#define REVISION_ID_RDBCK		1
#define PART_ID_REG_RDBCK		2

/* address 0x8C */
#define PS_DATA_STATUS_RDBCK		0
#define PS_INTERR_STATUS_RDBCK	1
#define ALS_DATA_STATUS_RDBCK		2
#define ALS_INTERR_STATUS_RDBCK	3
#define ALS_GAIN_STATUS_RDBCK		4
#define ALS_VALID_STATUS_RDBCK	5
#define ALS_PS_STATUS_RDBCK		6

/* address 0x8F */
#define INT_MODE_00				(0 << 0)
#define INT_MODE_PS_TRIG			(1 << 0)
#define INT_MODE_ALS_TRIG			(2 << 0)
#define INT_MODE_ALSPS_TRIG		(3 << 0)
#define INT_POLAR_ACT_LO			(0 << 2)
#define INT_POLAR_ACT_HI			(1 << 2)
#define INT_MODE_RDBCK				0
#define INT_POLAR_RDBCK				1
#define INT_INTERRUPT_RDBCK			2

/* address 0x9E */
#define ALS_PERSIST_SHIFT	0
#define PS_PERSIST_SHIFT	4
#define ALS_PRST_RDBCK		0
#define PS_PRST_RDBCK		1
#define ALSPS_PRST_RDBCK	2

#define PON_DELAY			600

#define ALS_MIN_MEASURE_VAL		0
#define ALS_MAX_MEASURE_VAL		65535
#define ALS_VALID_MEASURE_MASK	ALS_MAX_MEASURE_VAL
#define PS_MIN_MEASURE_VAL		0
#define PS_MAX_MEASURE_VAL		2047
#define PS_VALID_MEASURE_MASK   	PS_MAX_MEASURE_VAL
#define LO_LIMIT						0
#define HI_LIMIT						1
#define LO_N_HI_LIMIT				2
#define PS_OFFSET_MIN_VAL			0
#define PS_OFFSET_MAX_VAL			1023
#define 	FAR_VAL					1
#define 	NEAR_VAL					0

#define DRIVER_VERSION "1.13"
#define PARTID 0x92
#define MANUID 0x05

#define I2C_RETRY 5

#define DEVICE_NAME "LTR559ALSPS"

struct ltr559_data {
	/* Device */
	struct i2c_client *i2c_client;
	//struct input_dev *als_input_dev;
	struct input_polled_dev *als_input_dev; //lijiangshuo modified 20150109
	struct input_dev *ps_input_dev;
	struct work_struct work; // lijiangshuo add 20141226
	struct wake_lock ps_wake_lock;
	struct mutex bus_lock;

	/* Device mode
	 * 0 = ALS
	 * 1 = PS
	 */
	uint8_t mode;

	/* ALS */
	uint8_t als_enable_flag;
	uint8_t als_suspend_enable_flag;
	uint8_t als_irq_flag;
	uint8_t als_opened;
	uint16_t als_lowthresh;
	uint16_t als_highthresh;
	uint16_t default_als_lowthresh;
	uint16_t default_als_highthresh;
	uint16_t *adc_levels;
	/* Flag to suspend ALS on suspend or not */
	uint8_t disable_als_on_suspend;
	int als_poll_interval; // lijiangshuo add 20150109
	unsigned int als_min_interval; // lijiangshuo add 20150109
	
	/* PS */
	uint8_t ps_enable_flag;
	uint8_t ps_suspend_enable_flag;
	uint8_t ps_irq_flag;
	uint8_t ps_opened;
	uint16_t ps_lowthresh;
	uint16_t ps_highthresh;
	uint16_t default_ps_lowthresh;
	uint16_t default_ps_highthresh;
	/* Flag to suspend PS on suspend or not */
	uint8_t disable_ps_on_suspend;

	/* LED */
	int led_pulse_freq;
	int led_duty_cyc;
	int led_peak_curr;
	int led_pulse_count;

	/* Interrupt */
	int gpio_int; // lijiangshuo modified 20141226
	int is_suspend;
};
struct ltr559_data *sensor_info;

#define	PS_MAX_INIT_KEPT_DATA_COUNTER		8
#define	PS_MAX_MOV_AVG_KEPT_DATA_CTR		7

uint16_t	winfac1 = 100;
uint16_t	winfac2 = 80;
uint16_t	winfac3 = 44;
uint8_t	eqn_prev = 0;
uint8_t	ratio_old = 0;
uint16_t	ps_init_kept_data[PS_MAX_INIT_KEPT_DATA_COUNTER];
uint16_t	ps_ct_avg;
uint8_t	ps_grabData_stage = 0;
uint32_t	ftn_init;
uint32_t	ftn_final;
uint32_t	ntf_final;
uint16_t	lux_val_prev = 0;
uint8_t	ps_kept_data_counter = 0;
uint16_t	ps_movavg_data[PS_MAX_MOV_AVG_KEPT_DATA_CTR];
uint8_t	ps_movavg_data_counter = 0;
uint16_t	ps_movct_avg;
//uint16_t ps_thresh_hi, ps_thresh_lo;

/* lijiangshuo add 20141231 for proc system start */
static int chip_id = 0xff;
static struct proc_dir_entry *ltr559_info_proc_file;
/* lijiangshuo add 20141231 for proc system end */

/* I2C Read */
// take note --------------------------------------- 
// for i2c read, need to send the register address follwed by buffer over to register.
// There should not be a stop in between register address and buffer.  
// There should not be release of lock in between register address and buffer. 
// take note ---------------------------------------
static int8_t I2C_Read(uint8_t *rxData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 2) > 0)
			break;
		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Read Fail !!!!\n",__func__);
		return -EIO;
	}

	return 0;
}


/* I2C Write */
static int8_t I2C_Write(uint8_t *txData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = sensor_info->i2c_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	for (index = 0; index < I2C_RETRY; index++) {
		if (i2c_transfer(sensor_info->i2c_client->adapter, data, 1) > 0)
			break;
		mdelay(10);
	}

	if (index >= I2C_RETRY) {
		pr_alert("%s I2C Write Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}


/* Set register bit */
static int8_t _ltr559_set_bit(struct i2c_client *client, uint8_t set, uint8_t cmd, uint8_t data)
{
	uint8_t buffer[2];
	uint8_t value;
	int8_t ret = 0;

	buffer[0] = cmd;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value = buffer[0];

	if (set)
		value |= data;
	else
		value &= ~data;

	buffer[0] = cmd;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return -EIO;
	}

	return ret;
}


static uint16_t lux_formula(uint16_t ch0_adc, uint16_t ch1_adc, uint8_t eqtn)
{
	uint32_t luxval = 0;
	uint32_t luxval_i = 0;
	uint32_t luxval_f = 0;
	uint16_t ch0_coeff_i = 0;
	uint16_t ch1_coeff_i = 0;
	uint16_t ch0_coeff_f = 0;
	uint16_t ch1_coeff_f = 0;
	int8_t ret; 
	uint8_t gain = 1, als_int_fac;
	uint8_t buffer[2];
	uint16_t win_fac = 0;
	int8_t fac = 1;

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	gain = (buffer[0] & 0x70);
	gain >>= 4;

	if (gain == 0) {			//gain 1
		gain = 1;
	} else if (gain == 1) {		//gain 2
		gain = 2;
	} else if (gain == 2) {		//gain 4
		gain = 4;
	} else if (gain == 3) {		//gain 8
		gain = 8;
	} else if (gain == 6) {		//gain 48
		gain = 48;
	} else if (gain == 7) {		//gain 96
		gain = 96;
	}

	buffer[0] = LTR559_ALS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	als_int_fac = buffer[0] & 0x38;
	als_int_fac >>= 3;

	if (als_int_fac == 0) {
		als_int_fac = 10;
	} else if (als_int_fac == 1) {
		als_int_fac = 5;
	} else if (als_int_fac == 2) {
		als_int_fac = 20;
	} else if (als_int_fac == 3) {
		als_int_fac = 40;
	} else if (als_int_fac == 4) {
		als_int_fac = 15;
	} else if (als_int_fac == 5) {
		als_int_fac = 25;
	} else if (als_int_fac == 6) {
		als_int_fac = 30;
	} else if (als_int_fac == 7) {
		als_int_fac = 35;
	}

	if (eqtn == 1) {
		ch0_coeff_i = 1;
		ch1_coeff_i = 1;
		ch0_coeff_f = 7743;
		ch1_coeff_f = 1059;
		fac = 1;
		win_fac = winfac1;
		luxval_i = ((ch0_adc * ch0_coeff_i) + (ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) + (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		//luxval = ((17743 * ch0_calc) + (11059 * ch1_adc));
		//luxval = ((1.7743 * ch0_calc) + (1.1059 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 2) {
		ch0_coeff_i = 4;
		ch1_coeff_i = 1;
		ch0_coeff_f = 2785;
		ch1_coeff_f = 696;
		win_fac = winfac2;
		if ((ch1_coeff_f * ch1_adc) < (ch0_adc * ch0_coeff_f)) {
			fac = 1;
			luxval_f = (((ch0_adc * ch0_coeff_f) - (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		} else {
			fac = -1;
			luxval_f = (((ch1_adc * ch1_coeff_f) - (ch0_adc * ch0_coeff_f)) / 100) * win_fac;
		}
		luxval_i = ((ch0_adc * ch0_coeff_i) - (ch1_adc * ch1_coeff_i)) * win_fac;
		//luxval = ((42785 * ch0_calc) - (10696 * ch1_adc));
		//luxval = ((4.2785 * ch0_calc) - (1.9548 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 3) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 5926;
		ch1_coeff_f = 1300;
		fac = 1;
		win_fac = winfac3;
		luxval_i = ((ch0_adc * ch0_coeff_i) + (ch1_adc * ch1_coeff_i)) * win_fac;
		luxval_f = (((ch0_adc * ch0_coeff_f) + (ch1_adc * ch1_coeff_f)) / 100) * win_fac;
		//luxval = ((5926 * ch0_calc) + (1185 * ch1_adc));
		//luxval = ((0.5926 * ch0_calc) + (0.1185 * ch1_adc)) / (gain * (als_int_fac / 10));
	} else if (eqtn == 4) {
		ch0_coeff_i = 0;
		ch1_coeff_i = 0;
		ch0_coeff_f = 0;
		ch1_coeff_f = 0;
		fac = 1;
		luxval_i = 0;
		luxval_f = 0;
		//luxval = 0;
	}

	luxval = (luxval_i  + ((fac) * luxval_f) / 100) / (gain * als_int_fac);

	return luxval;
}

static uint16_t ratioHysterisis (uint16_t ch0_adc, uint16_t ch1_adc)
{
#define	RATIO_HYSVAL	10
	int ratio;
	uint8_t buffer[2], eqn_now;
	int8_t ret;
	uint16_t ch0_calc;
	uint32_t luxval = 0;
	int abs_ratio_now_old;

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	ch0_calc = ch0_adc;
	if ((buffer[0] & 0x20) == 0x20) {
		ch0_calc = ch0_adc - ch1_adc;
	}

	if ((ch1_adc + ch0_calc) == 0) {
		ratio = 100;
	} else {
		ratio = (ch1_adc*100) / (ch1_adc + ch0_calc);
	}

	if (ratio < 45) {
		eqn_now = 1;
	} else if ((ratio >= 45) && (ratio < 68)) {
		eqn_now = 2;
	} else if ((ratio >= 68) && (ratio < 99)) {
		eqn_now = 3;
	} else if (ratio >= 99) {
		eqn_now = 4;
	}

	if (eqn_prev == 0) {
		luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
		ratio_old = ratio;
		eqn_prev = eqn_now;
	} else {
		if (eqn_now == eqn_prev) {
			luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
			ratio_old = ratio;
			eqn_prev = eqn_now;
		} else {
			abs_ratio_now_old = ratio - ratio_old;
			if (abs_ratio_now_old < 0) {
				abs_ratio_now_old *= (-1);
			}
			if (abs_ratio_now_old > RATIO_HYSVAL) {
				luxval = lux_formula(ch0_calc, ch1_adc, eqn_now);
				ratio_old = ratio;
				eqn_prev = eqn_now;
			} else {
				luxval = lux_formula(ch0_calc, ch1_adc, eqn_prev);
			}
		}
	}
	return luxval;
}

static uint16_t read_als_adc_value(struct ltr559_data *ltr559)
{
	int8_t ret = -99;
	uint16_t value = -99;
	int ch0_val;
	int ch1_val;
	uint8_t gain, value_temp, gain_chg_req = 0;
	uint8_t buffer[4], temp;

#define AGC_UP_THRESHOLD		40000
#define AGC_DOWN_THRESHOLD  	5000
#define AGC_HYS					15
#define MAX_VAL					50000

	/* ALS */
	buffer[0] = LTR559_ALS_DATA_CH1_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 4);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	/* ALS Ch0 */
 	ch0_val = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
		dev_dbg(&ltr559->i2c_client->dev, 
			"%s | als_ch0 value = 0x%04X\n", __func__, 
			ch0_val);

	if (ch0_val > ALS_MAX_MEASURE_VAL) {
		dev_err(&ltr559->i2c_client->dev,
		        "%s: ALS Value Error: 0x%X\n", __func__,
		        ch0_val);
	}
	ch0_val &= ALS_VALID_MEASURE_MASK;

	/* ALS Ch1 */
 	ch1_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&ltr559->i2c_client->dev, 
			"%s | als_ch1 value = 0x%04X\n", __func__, 
			ch1_val);

	if (ch1_val > ALS_MAX_MEASURE_VAL) {
		dev_err(&ltr559->i2c_client->dev,
		        "%s: ALS Value Error: 0x%X\n", __func__,
		        ch1_val);
	}
	ch1_val &= ALS_VALID_MEASURE_MASK;

	buffer[0] = LTR559_ALS_PS_STATUS;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}

	value_temp = buffer[0];
	temp = buffer[0];
	gain = (value_temp & 0x70);	
	gain >>= 4;

	if (gain == 0) {			//gain 1
		gain = 1;
	} else if (gain == 1) {		//gain 2
		gain = 2;
	} else if (gain == 2) {		//gain 4
		gain = 4;
	} else if (gain == 3) {		//gain 8
		gain = 8;
	} else if (gain == 6) {		//gain 48
		gain = 48;
	} else if (gain == 7) {		//gain 96
		gain = 96;
	}

	buffer[0] = LTR559_ALS_CONTR;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}
	value_temp = buffer[0];
	value_temp &= 0xE3;

	if ((ch0_val == 0) && (ch1_val > 50 )) {
		value = lux_val_prev;
	} else {
		if (gain == 1) {
			if ((ch0_val + ch1_val) < ((AGC_DOWN_THRESHOLD * 10) / AGC_HYS)) {
				value = ratioHysterisis(ch0_val, ch1_val);
				value_temp |= ALS_GAIN_8x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val, ch1_val);
			}
		} else if (gain == 8) {
			if ((ch0_val + ch1_val) > AGC_UP_THRESHOLD) {
				value = ratioHysterisis(ch0_val, ch1_val);
				value_temp |= ALS_GAIN_1x;
				gain_chg_req = 1;
			} else {
				value = ratioHysterisis(ch0_val, ch1_val);
			}
		} else {
			value = ratioHysterisis(ch0_val, ch1_val);
		}
		if (gain_chg_req) {
			buffer[0] = LTR559_ALS_CONTR;
			buffer[1] = value_temp;
			ret = I2C_Write(buffer, 2);
			if (ret < 0) {
				dev_err(&ltr559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
				return ret;
			}
		}
	}

	if ((value > MAX_VAL) || (((ch0_val + ch1_val) > MAX_VAL) && (temp & 0x80))) {
		value = MAX_VAL;
	}
	lux_val_prev = value;	
	input_report_abs(ltr559->als_input_dev->input, ABS_MISC, value);
	input_sync(ltr559->als_input_dev->input);
//	printk("ljs %s als_value=%d\n", __func__, value);

	return value;
}

static uint16_t read_ps_adc_value(struct ltr559_data *ltr559)
{
	int8_t ret = -99;
	uint16_t value = -99;
	uint16_t ps_val;
	uint8_t buffer[4];

	buffer[0] = LTR559_PS_DATA_0;

	/* read data bytes from data regs */
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	ps_val = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
		dev_dbg(&ltr559->i2c_client->dev, 
			"%s | ps value = 0x%04X\n", __func__, 
			ps_val);

	if (ps_val > PS_MAX_MEASURE_VAL) {
		dev_err(&ltr559->i2c_client->dev,
		        "%s: PS Value Error: 0x%X\n", __func__,
		        ps_val);
	}
	ps_val &= PS_VALID_MEASURE_MASK;				
					
	value = ps_val;

	return value;
}

static int8_t als_mode_setup (uint8_t alsMode_set_reset, struct ltr559_data *ltr559)
{
	int8_t ret = 0;

	ret = _ltr559_set_bit(ltr559->i2c_client, alsMode_set_reset, LTR559_ALS_CONTR, ALS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s ALS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t als_contr_setup(uint8_t als_contr_val, struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_ALS_CONTR;

	/* Default settings used for now. */
	buffer[1] = als_contr_val;
	buffer[1] &= 0x1F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | ALS_CONTR (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}

static int8_t ps_mode_setup (uint8_t psMode_set_reset, struct ltr559_data *ltr559)
{
	int8_t ret = 0;

	ret = _ltr559_set_bit(ltr559->i2c_client, psMode_set_reset, LTR559_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s PS mode setup fail...\n", __func__);
		return ret;
	}

	return ret;
}

static int8_t ps_contr_setup(uint8_t ps_contr_val, struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_CONTR;

	/* Default settings used for now. */
	buffer[1] = ps_contr_val;
	buffer[1] &= 0x2F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | PS_CONTR (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}

/* LED Setup */
static int8_t ps_led_setup(uint8_t ps_led_val, struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_LED;

	/* Default settings used for now. */
	buffer[1] = ps_led_val;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | PS_LED (0x%02X) setup fail...", __func__, buffer[0]);
	}
	return ret;
}

static int8_t ps_ledPulseCount_setup(uint8_t pspulsecount_val, struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_PS_N_PULSES;

	/* Default settings used for now. */
	if (pspulsecount_val > 15) {
		pspulsecount_val = 15;
	}
	buffer[1] = pspulsecount_val;
	buffer[1] &= 0x0F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | PS_LED_COUNT (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}

static int8_t ps_meas_rate_setup(uint16_t meas_rate_val, struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[2], value;

	buffer[0] = LTR559_PS_MEAS_RATE;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}

	value = buffer[0];
	value &= 0xF0;

	if (meas_rate_val == 50) {
		value |= PS_MEAS_RPT_RATE_50MS;
	} else if (meas_rate_val == 70) {
		value |= PS_MEAS_RPT_RATE_70MS;
	} else if (meas_rate_val == 100) {
		value |= PS_MEAS_RPT_RATE_100MS;
	} else if (meas_rate_val == 200) {
		value |= PS_MEAS_RPT_RATE_200MS;
	} else if (meas_rate_val == 500) {
		value |= PS_MEAS_RPT_RATE_500MS;
	} else if (meas_rate_val == 1000) {
		value |= PS_MEAS_RPT_RATE_1000MS;
	} else if (meas_rate_val == 2000) {
		value |= PS_MEAS_RPT_RATE_2000MS;
	} else if (meas_rate_val == 10) {
		value |= PS_MEAS_RPT_RATE_10MS;		
	}

	buffer[0] = LTR559_PS_MEAS_RATE;
	buffer[1] = value;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s PS measurement rate setup fail...\n", __func__);

		return ret;
	}

	return ret;
}

static int8_t als_meas_rate_reg_setup(uint8_t als_meas_rate_reg_val, struct ltr559_data *ltr559)
{
	int8_t ret = 0;
	uint8_t buffer[3];

	buffer[0] = LTR559_ALS_MEAS_RATE;

	buffer[1] = als_meas_rate_reg_val;
	buffer[1] &= 0x3F;
	ret = I2C_Write(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | ALS_MEAS_RATE (0x%02X) setup fail...", __func__, buffer[0]);
	}

	return ret;
}


/* Set ALS range */
static int8_t set_als_range(uint16_t lt, uint16_t ht, uint8_t lo_hi)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR559_ALS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0xFF;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0xFF;
		num_data = 5;
	}	

	ret = I2C_Write(buffer, num_data);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	dev_dbg(&sensor_info->i2c_client->dev, "%s Set als range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);

	return ret;
}


/* Set PS range */
static int8_t set_ps_range(uint16_t lt, uint16_t ht, uint8_t lo_hi, struct ltr559_data *ltr559)
{
	int8_t ret;
	uint8_t buffer[5], num_data = 0;

	if (lo_hi == LO_LIMIT) {
		buffer[0] = LTR559_PS_THRES_LOW_0;
		buffer[1] = lt & 0xFF;
		buffer[2] = (lt >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == HI_LIMIT) {
		buffer[0] = LTR559_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		num_data = 3;
	} else if (lo_hi == LO_N_HI_LIMIT) {
		buffer[0] = LTR559_PS_THRES_UP_0;
		buffer[1] = ht & 0xFF;
		buffer[2] = (ht >> 8) & 0x07;
		buffer[3] = lt & 0xFF;
		buffer[4] = (lt >> 8) & 0x07;
		num_data = 5;
	}	

	ret = I2C_Write(buffer, num_data);
	if (ret <0) {
		pr_alert("%s | 0x%02X", __func__, buffer[0]);

		return ret;
	}
	dev_dbg(&ltr559->i2c_client->dev, "%s Set ps range:0x%04x"
	                                       " - 0x%04x\n", __func__, lt, ht);

	return ret;
}

#if 0
static uint16_t discardMinMax_findCTMov_Avg (uint16_t *ps_val)
{
#define MAX_NUM_PS_DATA1		PS_MAX_MOV_AVG_KEPT_DATA_CTR
#define STARTING_PS_INDEX1		0
#define ENDING_PS_INDEX1		5
#define NUM_AVG_DATA1			5

	uint8_t i_ctr, i_ctr2, maxIndex, minIndex;
	uint16_t maxVal, minVal, _ps_val[MAX_NUM_PS_DATA1];
	uint16_t temp = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		_ps_val[i_ctr] = ps_val[i_ctr];
	}

	maxVal = ps_val[STARTING_PS_INDEX1];
	maxIndex = STARTING_PS_INDEX1;
	minVal = ps_val[STARTING_PS_INDEX1];
	minIndex = STARTING_PS_INDEX1;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] > maxVal) {
			maxVal = ps_val[i_ctr];
			maxIndex = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
		if (ps_val[i_ctr] < minVal) {
			minVal = ps_val[i_ctr];
			minIndex = i_ctr;
		}
	}

	i_ctr2 = 0;

	if (minIndex != maxIndex) {
		for (i_ctr = STARTING_PS_INDEX1; i_ctr < MAX_NUM_PS_DATA1; i_ctr++) {
			if ((i_ctr != minIndex) && (i_ctr != maxIndex)) {
				ps_val[i_ctr2] = _ps_val[i_ctr];
				i_ctr2++;
			}
		}
	}
	ps_val[MAX_NUM_PS_DATA1 - 1] = 0;
	ps_val[MAX_NUM_PS_DATA1 - 2] = 0;

	for (i_ctr = STARTING_PS_INDEX1; i_ctr < ENDING_PS_INDEX1; i_ctr++) {
		temp += ps_val[i_ctr];
	}
	
	temp = (temp / NUM_AVG_DATA1);

	return temp;
}
#endif

static uint16_t findCT_Avg (uint16_t *ps_val)
{
#define MAX_NUM_PS_DATA2		PS_MAX_INIT_KEPT_DATA_COUNTER
#define STARTING_PS_INDEX2		3
#define NUM_AVG_DATA2			3

	uint8_t i_ctr, min_Index, max_Index;
	uint16_t max_val, min_val;
	uint16_t temp = 0;
	//struct ltr559_data *ltr559 = sensor_info;

	max_val = ps_val[STARTING_PS_INDEX2];
	max_Index = STARTING_PS_INDEX2;
	min_val = ps_val[STARTING_PS_INDEX2];
	min_Index = STARTING_PS_INDEX2;

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] > max_val) {
			max_val = ps_val[i_ctr];
			max_Index = i_ctr;
		}
	}

	for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
		if (ps_val[i_ctr] < min_val) {
			min_val = ps_val[i_ctr];
			min_Index = i_ctr;
		}
	}

	if (min_val == max_val) {
		// all values are the same
		temp = ps_val[STARTING_PS_INDEX2];
	} else {
		for (i_ctr = STARTING_PS_INDEX2; i_ctr < MAX_NUM_PS_DATA2; i_ctr++) {
			if ((i_ctr != min_Index) && (i_ctr != max_Index)) {
				temp += ps_val[i_ctr];
			}
		}
		temp = (temp / NUM_AVG_DATA2);
	}

	//temp = (temp / NUM_AVG_DATA2);

	return temp;
}

#if 0
// take note ------------------------------------------
// This function should be called in the function which is called when the CALL button is pressed.
// take note ------------------------------------------
static void setThrDuringCall (void)
{
	int8_t ret;
	struct ltr559_data *ltr559 = sensor_info;

	// set ps measurement rate to 10ms
	ret = ps_meas_rate_setup(10, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
	}

	ps_grabData_stage = 0;
	ps_kept_data_counter = 0;
	ps_movavg_data_counter = 0;

	ret = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s : PS thresholds setting Fail...\n", __func__);
	}

	ret = ps_contr_setup(0x03, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
	}
}
#endif

//(Linux RTOS)>
/* Report PS input event */
static void report_ps_input_event(struct ltr559_data *ltr559)
{
	int8_t ret;
	uint16_t adc_value;
	int thresh_hi, thresh_lo;

	adc_value = read_ps_adc_value (ltr559);
	printk("ljs %s ps_value=%d\n", __func__, adc_value);

#if 0
	if (ps_grabData_stage == 0) {
		if (ps_kept_data_counter < PS_MAX_INIT_KEPT_DATA_COUNTER) {
			if (adc_value != 0) {
				ps_init_kept_data[ps_kept_data_counter] = adc_value;
				ps_kept_data_counter++;
			}
		} 

		if (ps_kept_data_counter >= PS_MAX_INIT_KEPT_DATA_COUNTER) {
			ps_ct_avg = findCT_Avg(ps_init_kept_data);
			ftn_init = ps_ct_avg * 15;
			ps_grabData_stage = 1;
		}
	}

	if (ps_grabData_stage == 1) {
		if ((ftn_init - (ps_ct_avg * 10)) < 1000) {
			ftn_final = (ps_ct_avg * 10) + 1000;
		} else {
			if ((ftn_init - (ps_ct_avg * 10)) > 1500) {
				ftn_final = (ps_ct_avg * 10) + 1500;
			} else {
				ftn_final = ftn_init;
			}
		}
		ntf_final = (ftn_final - (ps_ct_avg * 10));
		ntf_final *= 4;
		ntf_final /= 100;
		ntf_final += ps_ct_avg;
		ftn_final /= 10;
		if (ntf_final >= PS_MAX_MEASURE_VAL) {
			ntf_final = PS_MAX_MEASURE_VAL;
		}
		if (ftn_final >= PS_MAX_MEASURE_VAL) {
			ftn_final = PS_MAX_MEASURE_VAL;
		}

		ret = ps_meas_rate_setup(50, ltr559);
		if (ret < 0) {
			dev_err(&ltr559->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
		}
		
		ps_grabData_stage = 2;
	}

	if (ps_grabData_stage == 2) {
		/* report NEAR or FAR to the user layer */
		if ((adc_value > ftn_final) || (adc_value < ntf_final)) {
			// FTN
			if (adc_value > ftn_final) {
				input_report_abs(ltr559->ps_input_dev, ABS_DISTANCE, NEAR_VAL);
				input_sync(ltr559->ps_input_dev);
			}
			// FTN

			// NTF
			if (adc_value < ntf_final) {
				input_report_abs(ltr559->ps_input_dev, ABS_DISTANCE, FAR_VAL);
				input_sync(ltr559->ps_input_dev);
			}
			// NTF
		}
		/* report NEAR or FAR to the user layer */

		if (ps_movavg_data_counter < PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
			if (adc_value != 0) {
				ps_movavg_data[ps_movavg_data_counter] = adc_value;
				ps_movavg_data_counter++;
			}
		} 

		if (ps_movavg_data_counter >= PS_MAX_MOV_AVG_KEPT_DATA_CTR) {
			ps_movct_avg = discardMinMax_findCTMov_Avg(ps_movavg_data);

			if (ps_movct_avg < ps_ct_avg) {
				ps_ct_avg = ps_movct_avg;
				ftn_init = ps_ct_avg * 17;
				ps_grabData_stage = 1;
			}
			ps_movavg_data_counter = 5;
		}
	}
#else
	if (adc_value > ltr559->ps_highthresh) {
		thresh_lo = ltr559->ps_lowthresh;
		thresh_hi = PS_MAX_MEASURE_VAL;
		input_report_abs(ltr559->ps_input_dev, ABS_DISTANCE, 0);
		input_sync(ltr559->ps_input_dev);

	} else if (adc_value < ltr559->ps_lowthresh) {
		thresh_lo = PS_MIN_MEASURE_VAL;
		thresh_hi = ltr559->ps_highthresh;
		input_report_abs(ltr559->ps_input_dev, ABS_DISTANCE, 1);
		input_sync(ltr559->ps_input_dev);
	} else {
		return;
	}

	/* Adjust measurement range using a crude filter to prevent interrupt
	 *  jitter. */
	if (thresh_lo < PS_MIN_MEASURE_VAL)
		thresh_lo = PS_MIN_MEASURE_VAL;
	if (thresh_hi > PS_MAX_MEASURE_VAL)
		thresh_hi = PS_MAX_MEASURE_VAL;
	ret = set_ps_range((uint16_t)thresh_lo, (uint16_t)thresh_hi, LO_N_HI_LIMIT, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
	}
#endif
}

/* Work when interrupt */
static void ltr559_schedwork(struct work_struct *work)
{
	int8_t ret;
	uint8_t status;
	uint8_t	interrupt_stat, newdata;
	struct ltr559_data *ltr559 = sensor_info;
	uint8_t buffer[2];

	wake_lock_timeout(&(ltr559->ps_wake_lock), HZ/2);

	buffer[0] = LTR559_ALS_PS_STATUS;	
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s | 0x%02X", __func__, buffer[0]);
		return;
	}
	status = buffer[0];
	interrupt_stat = status & 0x0A;
	newdata = status & 0x05;

	// PS interrupt and PS with new data
	if ((interrupt_stat & 0x02) && (newdata & 0x01)) {
		ltr559->ps_irq_flag = 1;
		report_ps_input_event(ltr559);
		ltr559->ps_irq_flag = 0;
	}

//	enable_irq(ltr559->gpio_int);
}

/* IRQ Handler */
static irqreturn_t ltr559_irq_handler(int irq, void *data)
{
	struct ltr559_data *ltr559 = data;

    	schedule_work(&ltr559->work);
    	return IRQ_HANDLED;
}

/* lijiangshuo add 20150109 start */
static ssize_t prox_cal_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint16_t adc_value;
	int rc;
	ps_kept_data_counter = 0;
	
	if (sensor_info->ps_enable_flag || sensor_info->als_enable_flag) {
		dev_err(&sensor_info->i2c_client->dev, "prox cal not execute due to %s", sensor_info->ps_enable_flag ? "prox enabled" : "als enabled");
		return -1;
	}

	printk("ljs %s\n", __func__);
	
	/* Set thresholds where interrupt will *not* be generated */
	rc = ps_led_setup(0x7F, sensor_info);
	if (rc < 0) {
		dev_err(&sensor_info->i2c_client->dev, "%s: PS LED Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_ledPulseCount_setup(0x08, sensor_info);
	if (rc < 0) {
		dev_err(&sensor_info->i2c_client->dev, "%s: PS LED pulse count setup Fail...\n", __func__);
	}

	rc = ps_meas_rate_setup(10, sensor_info);
	if (rc < 0) {
		dev_err(&sensor_info->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
		return rc;
	}

	rc = ps_contr_setup(0x03, sensor_info);
	if (rc < 0) {
		dev_err(&sensor_info->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
		return rc;
	}

	for(; ps_kept_data_counter<PS_MAX_INIT_KEPT_DATA_COUNTER; ps_kept_data_counter++)
	{
		mdelay(150);
		adc_value = read_ps_adc_value(sensor_info);
		if(adc_value) {
			ps_init_kept_data[ps_kept_data_counter] = adc_value;
			printk("read_ps_data[%d]: %d\n", ps_kept_data_counter, adc_value);
		}
	}

	ps_ct_avg = findCT_Avg(ps_init_kept_data);
	ftn_init = ps_ct_avg * 15;

	if((ftn_init - ps_ct_avg*10) < 1000)
		ftn_final = ps_ct_avg*10 + 1000;
	else
		if((ftn_init - ps_ct_avg*10) > 1500)
			ftn_final = ps_ct_avg*10 + 1500;
		else
			ftn_final = ftn_init;

	ntf_final = ftn_final - ps_ct_avg*10;
	ntf_final *= 4;
	ntf_final /= 100;
	ntf_final += ps_ct_avg;
	ftn_final /= 10;

	if (ntf_final >= PS_MAX_MEASURE_VAL)
		ntf_final = PS_MAX_MEASURE_VAL;
	if (ftn_final >= PS_MAX_MEASURE_VAL)
		ftn_final = PS_MAX_MEASURE_VAL;

	// lijiangshuo add for power on calibration fail 20150211
	if(ftn_final > 1500)
	{
		printk("ljs %s something wrong\n", __func__);
		return count;
	}
		
	sensor_info->ps_highthresh = ftn_final + 350; // lijiangshuo add for prox sensor dirty problem 20150211
	sensor_info->ps_lowthresh = ntf_final + 400; // lijiangshuo add for prox sensor dirty problem 20150211
	
	rc = ps_mode_setup(CLR_BIT, sensor_info);
	if (rc < 0) {
		dev_err(&sensor_info->i2c_client->dev, "%s: PS Disable Fail...\n", __func__);
		return rc;
	}

	printk("set ps_highthresh=%d ps_lowthresh=%d\n", sensor_info->ps_highthresh, sensor_info->ps_lowthresh);
	return count;
}
static DEVICE_ATTR(prox_cal,S_IRUGO | S_IWUSR, NULL, prox_cal_store);

static ssize_t proxdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 proxdata[3];
	int i = 0;

	for (i = 0; i < 3; i++) {
		mdelay(150);
		proxdata[i] = read_ps_adc_value (sensor_info);
	}

	return sprintf(buf, "%d, %d, %d\n", proxdata[0], proxdata[1], proxdata[2]);
}
static DEVICE_ATTR(prxdata, S_IRUGO, proxdata_show, NULL);

static ssize_t prxthreshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "lo=%d, hi=%d\n", sensor_info->ps_lowthresh, sensor_info->ps_highthresh);
}
static DEVICE_ATTR(prxthreshold, S_IRUGO, prxthreshold_show, NULL);

static ssize_t ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sensor_info->ps_enable_flag);
}

/* PS Enable */
static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
/* lijiangshuo add 20150109 end  */
{
	int8_t rc = 0;
	uint8_t buffer[1]; // for dummy read
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
	    return -EINVAL;
	dev_info(&sensor_info->i2c_client->dev, "~~~~~PS set poll enable %ld~~~~", val);

	if(val)
	{
//		setThrDuringCall(); // lijiangshuo deleted 20150112

		if (sensor_info->ps_enable_flag) 
		{
			dev_info(&sensor_info->i2c_client->dev, "%s: already enabled\n", __func__);
			return -1;
		}

		/* Set thresholds where interrupt will *not* be generated */
		//rc = set_ps_range(PS_MIN_MEASURE_VAL, PS_MIN_MEASURE_VAL, LO_N_HI_LIMIT);
		//rc = set_ps_range(PS_MIN_MEASURE_VAL, 400, LO_N_HI_LIMIT);
		rc = set_ps_range(sensor_info->ps_lowthresh, sensor_info->ps_highthresh, LO_N_HI_LIMIT, sensor_info);//ljs test
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s : PS Thresholds Write Fail...\n", __func__);
			return rc;
		}

		rc = ps_led_setup(0x7F, sensor_info);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s: PS LED Setup Fail...\n", __func__);
			return rc;
		}

		rc = ps_ledPulseCount_setup(0x08, sensor_info);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s: PS LED pulse count setup Fail...\n", __func__);
		}

		rc = ps_meas_rate_setup(10, sensor_info);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s: PS MeasRate Setup Fail...\n", __func__);
			return rc;
		}

		rc = ps_contr_setup(0x03, sensor_info);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s: PS Enable Fail...\n", __func__);
			return rc;
		}

		// dummy read
		buffer[0] = LTR559_PS_CONTR;
		I2C_Read(buffer, 1);
		// dumy read

		sensor_info->ps_enable_flag = 1;	
	}
	else
	{
		if (sensor_info->ps_enable_flag == 0) 
		{
		dev_info(&sensor_info->i2c_client->dev, "%s: already disabled\n", __func__);
		return -1;
		}

		//rc = _ltr559_set_bit(ltr559->i2c_client, CLR_BIT, LTR559_PS_CONTR, PS_MODE);
		rc = ps_mode_setup(CLR_BIT, sensor_info);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s: PS Disable Fail...\n", __func__);
			return rc;
		}

		sensor_info->ps_enable_flag = 0;
	}
	return count;
}
static DEVICE_ATTR(prox_enable, S_IRUGO | S_IWUSR, ps_enable_show, ps_enable_store);

/* lijiangshuo add 20150109 start */
static ssize_t als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sensor_info->als_enable_flag);
}

static ssize_t als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
/* lijiangshuo add 20150109 end */
{
	int8_t rc = 0;
	uint8_t buffer[1]; // for dummy read
	unsigned long val;
//	printk("ljs %s\n", __func__);

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	dev_info(&sensor_info->i2c_client->dev, "~~~~~ALS set poll enable %ld~~~~", val);

	if(val)
	{
		/* if device not enabled, enable it */
		if (sensor_info->als_enable_flag) {
			dev_err(&sensor_info->i2c_client->dev, "%s: ALS already enabled...\n", __func__);
			return -1;
		}

		rc = als_meas_rate_reg_setup(0x03, sensor_info);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s: ALS_Meas_Rate register Setup Fail...\n", __func__);
			return rc;
		}

		/* Set minimummax thresholds where interrupt will *not* be generated */
		rc = set_als_range(ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, LO_N_HI_LIMIT);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s : ALS Thresholds Write Fail...\n", __func__);
			return rc;
		}

		rc = als_contr_setup(0x0D, sensor_info);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s: ALS Enable Fail...\n", __func__);
			return rc;
		}

		// dummy read
		buffer[0] = LTR559_ALS_CONTR;
		I2C_Read(buffer, 1);
		// dumy read

		sensor_info->als_enable_flag = 1;
	}
	else
	{
		if (sensor_info->als_enable_flag == 0) {
			dev_err(&sensor_info->i2c_client->dev, "%s : ALS already disabled...\n", __func__);
			return -1;
		}

		//rc = _ltr559_set_bit(ltr559->i2c_client, CLR_BIT, LTR559_ALS_CONTR, ALS_MODE);
		rc = als_mode_setup(CLR_BIT, sensor_info);
		if (rc < 0) {
			dev_err(&sensor_info->i2c_client->dev,"%s: ALS Disable Fail...\n", __func__);
			return rc;
		}
		sensor_info->als_enable_flag = 0;
	}
	return count;
}
static DEVICE_ATTR(als_enable, S_IRUGO | S_IWUSR, als_enable_show, als_enable_store);

/* lijiangshuo add for system interface 20150107 start */
static ssize_t als_pollrate_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sensor_info->als_poll_interval);
}
static ssize_t als_pollrate_ms_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;

	interval_ms = max((unsigned int)interval_ms, sensor_info->als_min_interval);
	dev_info(&sensor_info->i2c_client->dev, "als set poll interval %ld", interval_ms);
	sensor_info->als_poll_interval = interval_ms;
	sensor_info->als_input_dev->poll_interval = interval_ms;

	return count;
}
static DEVICE_ATTR(als_pollrate_ms, S_IRUGO | S_IWUSR, als_pollrate_ms_show, als_pollrate_ms_store);

static struct attribute *sensor_attr[] = {
	&dev_attr_prox_enable.attr,
	&dev_attr_prxdata.attr,
	&dev_attr_prxthreshold.attr,
	&dev_attr_prox_cal.attr,
	&dev_attr_als_enable.attr,
	&dev_attr_als_pollrate_ms.attr,
       NULL,
};

static struct attribute_group sensor_dev_attr_grp = {
        .attrs = sensor_attr,
};
/* lijiangshuo add for system interface 20150107 end */

// lijiangshuo add 20150109
static void als_poll(struct input_polled_dev *dev)
{
	if(!sensor_info->als_enable_flag)
		return;
	read_als_adc_value(sensor_info);
}

static int als_setup(struct ltr559_data *ltr559)
{
	int ret;
//	printk("ljs %s\n", __func__);

	ltr559->als_input_dev = input_allocate_polled_device();
	if (!ltr559->als_input_dev) {
		dev_err(&ltr559->i2c_client->dev, "%s: ALS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}

	ltr559->als_input_dev->input->name = "lightsensor-level"; //"ltr559_als";
	ltr559->als_input_dev->poll_interval = ltr559->als_poll_interval; // lijiangshuo add 20150109
	ltr559->als_input_dev->poll = als_poll;// lijiangshuo add 20150109
	ltr559->als_input_dev->input->dev.parent = &ltr559->i2c_client->dev;

	set_bit(EV_ABS, ltr559->als_input_dev->input->evbit);
	input_set_abs_params(ltr559->als_input_dev->input, ABS_MISC, ALS_MIN_MEASURE_VAL, ALS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_polled_device(ltr559->als_input_dev);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: ALS Register Input Device Fail...\n", __func__);
		goto err_als_register_input_device;
	}

	return ret;

err_als_register_input_device:
	input_free_polled_device(ltr559->als_input_dev);

	return ret;
}

/* lijiangshuo add 20150109 start */
static int ps_input_open(struct input_dev *dev)
{
	dev_info(&sensor_info->i2c_client->dev, "enable wake");
	enable_irq_wake(sensor_info->i2c_client->irq);
	return 0;
}
static void ps_input_close(struct input_dev *dev)
{
	dev_info(&sensor_info->i2c_client->dev, "disable wake");
	disable_irq_wake(sensor_info->i2c_client->irq);
}
/* lijiangshuo add 20150109 end */

static int ps_setup(struct ltr559_data *ltr559)
{
	int ret;
//	printk("ljs %s\n", __func__);

	ltr559->ps_input_dev = input_allocate_device();
	if (!ltr559->ps_input_dev) {
		dev_err(&ltr559->i2c_client->dev, "%s: PS Input Allocate Device Fail...\n", __func__);
		return -ENOMEM;
	}
	ltr559->ps_input_dev->name = "proximity";//"ltr559_ps";
/* lijiangshuo add 20150109 start */
	ltr559->ps_input_dev->open = ps_input_open;
	ltr559->ps_input_dev->close = ps_input_close;
	ltr559->ps_input_dev->dev.parent = &ltr559->i2c_client->dev;
/* lijiangshuo add 20150109 end */
	set_bit(EV_ABS, ltr559->ps_input_dev->evbit);
	input_set_abs_params(ltr559->ps_input_dev, ABS_DISTANCE, PS_MIN_MEASURE_VAL, PS_MAX_MEASURE_VAL, 0, 0);

	ret = input_register_device(ltr559->ps_input_dev);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: PS Register Input Device Fail...\n", __func__);
		goto err_ps_register_input_device;
	}

	return ret;

err_ps_register_input_device:
	input_free_device(ltr559->ps_input_dev);

	return ret;
}

//LJS

static uint8_t _check_part_id(struct ltr559_data *ltr559)
{
	uint8_t ret;
	uint8_t buffer[2];

	buffer[0] = LTR559_PART_ID;
	ret = I2C_Read(buffer, 1);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: Read failure :0x%02X",
		        __func__, buffer[0]);
		return -1;
	}

	if (buffer[0] != PARTID) {
		dev_err(&ltr559->i2c_client->dev, "%s: Part failure miscompare"
		        " act:0x%02x exp:0x%02x\n", __func__, buffer[0], PARTID);
		return -2;
	}

	chip_id = buffer[0];
	printk("ljs %s chip_id=0x%x\n", __func__, chip_id);

	return 0;
}


static int ltr559_setup(struct ltr559_data *ltr559)
{
	int ret = 0;
//	printk("ljs %s\n", __func__);

	/* Reset the devices */
	ret = _ltr559_set_bit(ltr559->i2c_client, SET_BIT, LTR559_ALS_CONTR, ALS_SW_RESET);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: ALS reset fail...\n", __func__);
		goto err_out1;
	}

	ret = _ltr559_set_bit(ltr559->i2c_client, CLR_BIT, LTR559_PS_CONTR, PS_MODE_ACTIVE);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: PS reset fail...\n", __func__);
		goto err_out1;
	}

	msleep(PON_DELAY);
	dev_dbg(&ltr559->i2c_client->dev, "%s: Reset ltr559 device\n", __func__);

	/* Do another part read to ensure we have exited reset */
	if (_check_part_id(ltr559) < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: Part ID Read Fail after reset...\n", __func__);
		goto err_out1;
	}

// lijiangshuo add 20141226
	ret =  request_irq( ltr559->gpio_int, ltr559_irq_handler, IRQ_TYPE_EDGE_FALLING, DEVICE_NAME, ltr559); // IRQ_TYPE_EDGE_FALLING
	if (ret) {
		dev_err(&ltr559->i2c_client->dev, "%s %s fail to request irq, ret = %d\n", __FILE__, __func__, ret);
		goto err_out1;
	}

	/* Set count of measurements outside data range before interrupt is generated */
	ret = _ltr559_set_bit(ltr559->i2c_client, SET_BIT, LTR559_INTERRUPT_PRST, 0x01);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: ALS Set Persist Fail...\n", __func__);
		goto err_out2;
	}

	ret = _ltr559_set_bit(ltr559->i2c_client, SET_BIT, LTR559_INTERRUPT_PRST, 0x10);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,"%s: PS Set Persist Fail...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr559->i2c_client->dev, "%s: Set ltr559 persists\n", __func__);

	/* Enable interrupts on the device and clear only when status is read */
	//ret = _ltr559_set_bit(ltr559->i2c_client, SET_BIT, LTR559_INTERRUPT, INT_MODE_ALSPS_TRIG);
	ret = _ltr559_set_bit(ltr559->i2c_client, SET_BIT, LTR559_INTERRUPT, INT_MODE_PS_TRIG);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: Enabled interrupts failed...\n", __func__);
		goto err_out2;
	}
	dev_dbg(&ltr559->i2c_client->dev, "%s Enabled interrupt to device\n", __func__);

	return ret;

err_out2:
	free_irq(ltr559->gpio_int, ltr559);

err_out1:
	dev_err(&ltr559->i2c_client->dev, "%s Unable to setup device\n", __func__);

	return ret;
}

/* lijiangshuo add 20141231 for proc system start */
static int ltr559_proc_show(struct seq_file *m, void *v)
{
        return seq_printf(m, "0x%x\n", chip_id);
}
static int ltr559_proc_open(struct inode *inode, struct file *file)
{
        return single_open(file, ltr559_proc_show, NULL);
}

static const struct file_operations ltr559_proc_fops = {
        .open           = ltr559_proc_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static void create_ltr559_info_proc_file(void)
{
  ltr559_info_proc_file = proc_create("driver/alsprx", 0644, NULL, &ltr559_proc_fops);
  if (!ltr559_info_proc_file) {
		printk(KERN_INFO "ltr559 proc file create failed!\n");
   }
}

static void remove_ltr559_info_proc_file(void)
{
	if(ltr559_info_proc_file){
		remove_proc_entry("driver/alsprx", NULL);
		ltr559_info_proc_file = NULL;
	}
}

/* lijiangshuo add 20141231 for proc system end */

static int ltr559_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct ltr559_data *ltr559;

//	printk("ljs %s\n", __func__);
	
	ltr559 = kzalloc(sizeof(struct ltr559_data), GFP_KERNEL);
	if (!ltr559)
	{
		dev_err(&ltr559->i2c_client->dev, "%s: Mem Alloc Fail...\n", __func__);
		return -ENOMEM;
	}

	/* Global pointer for this device */
	sensor_info = ltr559;

	/* Set initial defaults */
	ltr559->als_enable_flag = 0;
	ltr559->ps_enable_flag = 0;

/* lijiangshuo add start 20141226 */
	if (client->dev.of_node)
	{
		u32 temp_val;
		struct pinctrl *pinctrl;

		pinctrl = devm_pinctrl_get_select_default(&client->dev);
		if (IS_ERR(pinctrl))
		{
			dev_warn(&client->dev, "pins are not configured from the driver\n");
			goto err_out;
		} 
	
		temp_val = of_get_gpio(client->dev.of_node, 0); 
		if (gpio_is_valid(temp_val))
		{
			printk("ljs %s ltr559 irq get gpio %d\n", __func__, temp_val);
			gpio_request(temp_val, "ltr559_irq");
			gpio_direction_input(temp_val);
			client->irq = gpio_to_irq(temp_val);
		}
		else
		{
			dev_err(&client->dev,"invalid gpio");
			goto err_out;
		}

		ret = of_property_read_u32(client->dev.of_node, "default-ps-lowthresh", &temp_val);
		if (ret) 
		{
			dev_err(&client->dev, "Unable to read default-ps-lowthresh");
			goto property_read_fail;
		}
		else
		{
			ltr559->default_ps_lowthresh = temp_val;
			ltr559->ps_lowthresh = temp_val;
		}

		ret = of_property_read_u32(client->dev.of_node, "default-ps-highthresh", &temp_val);
		if (ret) 
		{
			dev_err(&client->dev, "Unable to read default-ps-highthresh");
			goto property_read_fail;
		}
		else
		{
			ltr559->default_ps_highthresh = temp_val;
			ltr559->ps_highthresh = temp_val;
		}

//lijiangshuo add 20150109
		ret = of_property_read_u32(client->dev.of_node, "als-min-interval", &temp_val);
		if (ret) {
		    dev_err(&client->dev, "Unable to read min-interval");
		    goto property_read_fail;
		} else 
			ltr559->als_min_interval = temp_val;
//		printk("ljs %s min-interval=%d\n", __func__, ltr559->als_min_interval);

		ret = of_property_read_u32(client->dev.of_node, "als-poll-interval", &temp_val);
		if (ret) {
		    dev_err(&client->dev, "Unable to read poll-interval");
		    goto property_read_fail;
		} else 
			ltr559->als_poll_interval = temp_val;
//		printk("ljs %s poll-interval=%d\n", __func__, ltr559->als_poll_interval);

	}
/* lijiangshuo add end 20141226*/

	ltr559->i2c_client = client;
	ltr559->gpio_int = client->irq;

	i2c_set_clientdata(client, ltr559);

	sensor_power_onoff(1); // lijiangshuo add 20150109

	if (_check_part_id(ltr559) < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: Part ID Read Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the ALS */
	ret = als_setup(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,"%s: ALS Setup Fail...\n", __func__);
		goto err_out;
	}

	/* Setup the input subsystem for the PS */
	ret = ps_setup(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: PS Setup Fail...\n", __func__);
		goto err_out;
	}

	/* Wake lock option for promity sensor */
	wake_lock_init(&(ltr559->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
	INIT_WORK(&(ltr559->work),ltr559_schedwork);
//	mutex_init(&ltr559->bus_lock);

	/* Setup and configure both the ALS and PS on the ltr559 device */
	ret = ltr559_setup(ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev, "%s: Setup Fail...\n", __func__);
		goto property_read_fail;
	}

/* lijiangshuo add for system interface 20150107 */
	ret = sysfs_create_group(&client->dev.kobj, &sensor_dev_attr_grp);
	if (ret) {
	    dev_err(&client->dev, "create sys file failed\n");
	    goto property_read_fail;
	}    


	dev_dbg(&ltr559->i2c_client->dev, "%s: probe complete\n", __func__);
	printk("ljs %s done\n", __func__);

	return ret;

property_read_fail:
	gpio_free(ltr559->gpio_int);
err_out:
	kfree(ltr559);

	return ret;
}


static const struct i2c_device_id ltr559_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};

/* lijiangshuo add for device tree driver 20150105 start */
static struct of_device_id ltr559_match_table[] = {
         { .compatible = "ltr559", },
         { },
};
/* lijiangshuo add for device tree driver 20150105 end */

static struct i2c_driver ltr559_driver = {
	.probe = ltr559_probe,
	.id_table = ltr559_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
		.of_match_table = ltr559_match_table, //lijiangshuo add for device tree driver 20150105

	},
};


static int __init ltr559_init(void)
{
	int ret = 0;

	if((ret = i2c_add_driver(&ltr559_driver)) < 0)
		printk(KERN_ERR "ltr559 i2c_add_driver() failed!\n");
	
	if(0xff != chip_id)
		create_ltr559_info_proc_file();

	return (ret);
}

static void __exit ltr559_exit(void)
{
	i2c_del_driver(&ltr559_driver);

	if(0xff != chip_id)
		remove_ltr559_info_proc_file();

	free_irq(sensor_info->i2c_client->irq, NULL);
	sysfs_remove_group(&sensor_info->i2c_client->dev.kobj, &sensor_dev_attr_grp);
	gpio_free(sensor_info->gpio_int);
	wake_lock_destroy(&sensor_info->ps_wake_lock);
}

module_init(ltr559_init)
module_exit(ltr559_exit)

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-559ALSPS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
