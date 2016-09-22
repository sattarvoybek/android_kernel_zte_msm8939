/*******************************************************************************
*                                                                              *
*   File Name:    taos.c                                                      *
*   Description:   Linux device driver for Taos ambient light and         *
*   proximity sensors.                                     *
*   Author:         John Koshi                                             *
*   History:   09/16/2009 - Initial creation                          *
*           10/09/2009 - Triton version         *
*           12/21/2009 - Probe/remove mode                *
*           02/07/2010 - Add proximity          *
*                                                                              *
********************************************************************************
*    Proprietary to Taos Inc., 1001 Klein Road #300, Plano, TX 75074        *
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <linux/taos_common.h>
#include <linux/delay.h>
//iVIZM
#include <linux/irq.h> 
#include <linux/interrupt.h> 
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/proc_fs.h> 
#include <linux/of_gpio.h>
#include <linux/sensor_power.h>

// device name/id/address/counts
#define TAOS_DEVICE_NAME                "taos"
#define TAOS_DEVICE_ID                  "taos"
#define TAOS_ID_NAME_SIZE               10
#define TAOS_TRITON_CHIPIDVAL           0x00
#define TAOS_DEVICE_ADDR1               0x29
#define TAOS_DEVICE_ADDR2               0x39
#define TAOS_DEVICE_ADDR3               0x49
#define TAOS_MAX_NUM_DEVICES            3

// TRITON register offsets
#define TAOS_TRITON_CNTRL               0x00
#define TAOS_TRITON_ALS_TIME            0X01
#define TAOS_TRITON_PRX_TIME            0x02
#define TAOS_TRITON_WAIT_TIME           0x03
#define TAOS_TRITON_ALS_MINTHRESHLO     0X04
#define TAOS_TRITON_ALS_MINTHRESHHI     0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO     0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI     0X07
#define TAOS_TRITON_PRX_MINTHRESHLO     0X08
#define TAOS_TRITON_PRX_MINTHRESHHI     0X09
#define TAOS_TRITON_PRX_MAXTHRESHLO     0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI     0X0B
#define TAOS_TRITON_INTERRUPT           0x0C
#define TAOS_TRITON_PRX_CFG             0x0D
#define TAOS_TRITON_PRX_COUNT           0x0E
#define TAOS_TRITON_GAIN                0x0F
#define TAOS_TRITON_REVID               0x11
#define TAOS_TRITON_CHIPID              0x12
#define TAOS_TRITON_STATUS              0x13
#define TAOS_TRITON_ALS_CHAN0LO         0x14
#define TAOS_TRITON_ALS_CHAN0HI         0x15
#define TAOS_TRITON_ALS_CHAN1LO         0x16
#define TAOS_TRITON_ALS_CHAN1HI         0x17
#define TAOS_TRITON_PRX_LO              0x18
#define TAOS_TRITON_PRX_HI              0x19
#define TAOS_TRITON_PRX_OFFSET     	    0x1E

// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG             0X80
#define TAOS_TRITON_CMD_AUTO            0x10 //iVIZM
#define TAOS_TRITON_CMD_BYTE_RW         0x00
#define TAOS_TRITON_CMD_WORD_BLK_RW     0x20
#define TAOS_TRITON_CMD_SPL_FN          0x60
#define TAOS_TRITON_CMD_PROX_INTCLR     0X05
#define TAOS_TRITON_CMD_ALS_INTCLR      0X06
#define TAOS_TRITON_CMD_PROXALS_INTCLR  0X07
#define TAOS_TRITON_CMD_TST_REG         0X08
#define TAOS_TRITON_CMD_USER_REG        0X09

// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL  0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL   0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL  0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL  0X04
#define TAOS_TRITON_CNTL_ADC_ENBL       0x02
#define TAOS_TRITON_CNTL_PWRON          0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID     0x01
#define TAOS_TRITON_STATUS_PRXVALID     0x02
#define TAOS_TRITON_STATUS_ADCINTR      0x10
#define TAOS_TRITON_STATUS_PRXINTR      0x20
#define TAOS_TRITON_STATUS_PRXSAT      	0x40

// lux constants
#define TAOS_MAX_LUX                    10000
#define TAOS_SCALE_MILLILUX             3
#define TAOS_FILTER_DEPTH               3

typedef enum {
	PROX_CAL_NOT_START,
	PROX_CAL_RUNNING,
	PROX_CAL_SUCCEED,
	PROX_CAL_FAIL_TOO_SMALL,
	PROX_CAL_FAIL_TOO_BIG,
	PROX_CAL_FAIL_REG,
	PROX_CAL_FAIL_PROX_OR_ALS_ENABLED,
}prox_cal_type;
struct taos_data {
    struct i2c_client *client;
    struct input_dev *prox_dev;
    struct input_polled_dev *als_dev;
    struct work_struct work;
    struct wake_lock taos_wake_lock;
    struct mutex taos_mutex;
    int irq_gpio;

    int prox_enable;
    int als_enable;
    unsigned int als_min_interval;
    int als_poll_interval;

    prox_cal_type prox_cal_status;
} *taos_datap;

// device configuration
struct taos_cfg *taos_cfgp;
static u16 als_time_param = 200;        //0.2*3 = 0.6
static u16 scale_factor_param = 1;            //0x0c
static u16 prox_threshold_hi_param = 0x230;
static u16 prox_threshold_lo_param = 0x210;
static u16 als_threshold_hi_param = 3000;
static u16 als_threshold_lo_param = 10;
static u8 prox_als_time_param = 0xDB;//100ms, must be a multiple of 50ms
static u8 prox_time_param = 0xFF;
static u8 wait_time_param = 0xEE;//50ms
static u8 pers_param = 0x23;
static u8 prox_config_param = 0x00;
static u8 prox_gain_param = 0x61;
#ifdef CONFIG_PROJECT_p816A12
static u8 prox_pulse_cnt_param = 16;
static u8 prox_offset_param = 0x90;
#else
static u8 prox_pulse_cnt_param = 16;
static u8 prox_offset_param = 0x90;
#endif

// prox info
#define CAL_NUMBER 10
struct taos_prox_info prox_cur_info;
static u16 sat_als = 0;

// device reg init values
u8 taos_triton_reg_init[16] = {0x00,0xFF,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00,0X00};

// lux time scale
struct time_scale_factor  {
    u16 numerator;
    u16 denominator;
    u16 saturation;
};
struct time_scale_factor TritonTime = {1, 0, 0};

// gain table
u8 taos_triton_gain_table[] = {1, 8, 16, 120};

// lux data
struct lux_data {
    u16 ratio;  //红外线比上全部域的光线
    u16 clear; //红外线比上全部域的光线
    u16 ir;        //光线中的红外线

};
struct lux_data TritonFN_lux_data[] = {
    { 9830,  8320,  15360 },
    { 12452, 10554, 22797 },
    { 14746, 6234,  11430 },
    { 17695, 3968,  6400  },
    { 0,     0,     0     }
};
static int lux_history[TAOS_FILTER_DEPTH] = {-ENODATA, -ENODATA, -ENODATA};//iVIZM
static int chip_id = 0xff;

// read/calculate lux value
static int taos_get_lux(void) {
    u16 raw_clear = 0, raw_ir = 0, raw_lux = 0;
    u32 lux = 0;
    u32 ratio = 0;
    u8 dev_gain = 0;
    u16 Tint = 0;
    struct lux_data *p;
    int ret = 0;
    u16 chdata[2];
    int tmp = 0, i = 0;
    int als_time = 0;

    if(taos_datap->prox_enable) {
	    als_time = (256 - taos_cfgp->prox_als_time) / 18;
	    als_time = als_time * 50;
    } else
	    als_time = taos_cfgp->als_time;
    for (i = 0; i < 2; i++) {
        if ((chdata[i] = (i2c_smbus_read_word_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW | (TAOS_TRITON_ALS_CHAN0LO + i*2))))) < 0) {
            dev_err(&taos_datap->client->dev, "i2c_smbus_read to chan0/1/lo/hi reg failed\n");
            return (ret);
        }
    }
//    printk("ch0=%d\n",chdata[0]+chdata[1]*256);
//    printk("ch1=%d\n",chdata[2]+chdata[3]*256);

    tmp = (als_time + 25)/50; //if atime =100  tmp = (atime+25)/50=2.5  tine = 2.7*(256-atime)=  412.5
    TritonTime.numerator = 1;
    TritonTime.denominator = tmp;

    tmp = 300 * als_time;  //tmp = 300*atime  400
    if(tmp > 65535)
        tmp = 65535;
    TritonTime.saturation = tmp;
    raw_clear = chdata[0];
    raw_ir    = chdata[1];

    raw_clear *= (taos_cfgp->scale_factor * 11);   
    raw_ir *= (taos_cfgp->scale_factor * 3);

    if(raw_ir > raw_clear) {
        raw_lux = raw_ir;
        raw_ir = raw_clear;
        raw_clear = raw_lux;
    }
    dev_gain = taos_triton_gain_table[taos_cfgp->als_gain & 0x3];
    if(raw_clear >= TritonTime.saturation) {
		dev_info(&taos_datap->client->dev, "raw_clear saturate, %d, %d\n",raw_clear, TritonTime.saturation);
        return(TAOS_MAX_LUX);
    }
    if(raw_ir >= TritonTime.saturation) {
		dev_info(&taos_datap->client->dev, "raw_ir saturate, %d, %d\n",raw_ir, TritonTime.saturation);
        return(TAOS_MAX_LUX);
    }
    if(raw_clear <= 10) {
		dev_info(&taos_datap->client->dev, "raw_clear too small %d\n", raw_clear);
        return(0);
    }
    if(dev_gain == 0 || dev_gain > 127) {
        dev_err(&taos_datap->client->dev, "dev_gain = 0 or > 127 in %s\n", __func__);
        return -1;
    }
    if(TritonTime.denominator == 0) {
        dev_err(&taos_datap->client->dev, "TAOS: lux_timep->denominator = 0 in %s\n", __func__);
        return -1;
    }
    ratio = (raw_ir<<15)/raw_clear;
    for (p = TritonFN_lux_data; p->ratio && p->ratio < ratio; p++);
    if(!p->ratio) {//iVIZM
    	 dev_info(&taos_datap->client->dev, "ratio = 0,ch0=%d, ch1=%d, lux=10000\n",chdata[0], chdata[1]);
    	 return 10000;
    }
    Tint = als_time;
    raw_clear = ((raw_clear*400 + (dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
    raw_ir = ((raw_ir*400 +(dev_gain>>1))/dev_gain + (Tint>>1))/Tint;
    lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)));
    lux = (lux + 32000)/64000;
    if(lux > TAOS_MAX_LUX) {
	 	dev_info(&taos_datap->client->dev, "big lux,ch0=%d, ch1=%d, lux=10000\n",chdata[0], chdata[1]);
        lux = TAOS_MAX_LUX;
    }
    //return(lux)*taos_cfgp->filter_count;
    return(lux);
}

static int taos_als_get_data(void)
{
    int ret = 0;
    u8 reg_val;
    int lux_val = 0;

    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0) {
        dev_err(&taos_datap->client->dev, "i2c_smbus_write_byte failed %s %d\n",__func__, __LINE__);
        return (ret);
    }
    reg_val = i2c_smbus_read_byte(taos_datap->client);
    if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID)
    {
    	dev_err(&taos_datap->client->dev,"als adc haven't complete\n");
		return -ENODATA;
    }
    if ((lux_val = taos_get_lux()) < 0)
	    dev_err(&taos_datap->client->dev, "call to taos_get_lux() returned error lux %d\n", lux_val);
    input_report_abs(taos_datap->als_dev->input,ABS_MISC,lux_val);
    input_sync(taos_datap->als_dev->input);
    return ret;
}

static void als_poll(struct input_polled_dev *dev)
{
	if(!taos_datap->als_enable)
		return;
	taos_als_get_data();
}

static int taos_prox_poll(struct taos_prox_info *prxp) {
    int i = 0, ret = 0; 
    u16 chdata[3];
    for (i = 0; i < 3; i++) {
	    chdata[i] = (i2c_smbus_read_word_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW | (TAOS_TRITON_ALS_CHAN0LO + i*2))));
    }
    prxp->prox_clear = chdata[0];
    if (prxp->prox_clear > ((sat_als*80)/100)) {
	    dev_err(&taos_datap->client->dev, "als data satuated, maybe strong sun light\n");
	    return -ENODATA;
    }
    prxp->prox_data = chdata[2];
    return (ret);
}

static int taos_prox_threshold_set(int first)
{
	int i,ret = 0;
	u16 chdata[3];
	u16 proxdata = 0;
	u16 cleardata = 0;
	int data = 0;
	u8 pro_buf[4];
	
	for (i = 0; i < 3; i++) {
		chdata[i] = (i2c_smbus_read_word_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW| (TAOS_TRITON_ALS_CHAN0LO + i*2))));
	}
	cleardata = chdata[0];
	proxdata = chdata[2];

	if (first || proxdata < taos_cfgp->prox_threshold_lo ) {
		pro_buf[0] = 0x0;
		pro_buf[1] = 0x0;
		pro_buf[2] = taos_cfgp->prox_threshold_hi & 0x0ff;
		pro_buf[3] = taos_cfgp->prox_threshold_hi >> 8;
		data = 1;
		input_report_abs(taos_datap->prox_dev,ABS_DISTANCE,data);
	} else if (proxdata > taos_cfgp->prox_threshold_hi ){
		if (cleardata > ((sat_als*80)/100)){
			dev_err(&taos_datap->client->dev, "prox data satuated in %s", __func__);
			return -ENODATA;
		}
		pro_buf[0] = taos_cfgp->prox_threshold_lo & 0x0ff;
		pro_buf[1] = taos_cfgp->prox_threshold_lo >> 8;
		pro_buf[2] = 0xff;
		pro_buf[3] = 0xff;
		data = 0;
		input_report_abs(taos_datap->prox_dev,ABS_DISTANCE,data);
    }
    input_sync(taos_datap->prox_dev);

    printk("TAOS: prox_threshold_lo = 0x%x, prox_threashold_hi = 0x%x, proxdata = 0x%x，data = %d\n", taos_cfgp->prox_threshold_lo, taos_cfgp->prox_threshold_hi, proxdata,data);
	
    for( i=0; i<4; i++ ) { 
        if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_MINTHRESHLO) + i, pro_buf[i]))) < 0) {
             dev_err(&taos_datap->client->dev, "failed to set taos prox threshold\n");
             return (ret);
        }
    }
	
    return ret;
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id)
{
    return IRQ_WAKE_THREAD;
}

static irqreturn_t taos_irq_handler_thread(int irq, void *dev_id) //iVIZM
{
    schedule_work(&taos_datap->work);
    return IRQ_HANDLED;
}

static int taos_get_data(void)
{
    int ret = 0;
    int status;
    int loop =5;
	
    while(loop){
	    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS)))) < 0) {
		    mdelay(20);
		    loop--;
		    dev_err(&taos_datap->client->dev, "failed in taos_get_data(), try again number is %d",loop);
	    }else{
		    loop = 0;
	    }
    }
    status = i2c_smbus_read_byte(taos_datap->client);
    if((status & TAOS_TRITON_STATUS_PRXINTR) == TAOS_TRITON_STATUS_PRXINTR) {
	    if ((status & TAOS_TRITON_STATUS_PRXSAT) == TAOS_TRITON_STATUS_PRXSAT)
		    dev_info(&taos_datap->client->dev, "tmd2772 prox measurement is saturated\n");
	    ret = taos_prox_threshold_set(0);
    }
    return ret;
}

static int taos_interrupts_clear(void)//iVIZM
{
    int ret = 0;
    int loop =5;
    while(loop){
	    if ((ret = (i2c_smbus_write_byte(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|TAOS_TRITON_CMD_PROXALS_INTCLR)))) < 0) {
		    mdelay(20);
		    loop--;
		    dev_err(&taos_datap->client->dev, "failed in taos_interrupts_clear(), try again number is %d\n",loop);
	    }else{
		    loop = 0;
	    }
    }
    return ret;
}

static void taos_work_func(struct work_struct * work)
{
    wake_lock_timeout(&taos_datap->taos_wake_lock, HZ/2);
    taos_get_data();
    taos_interrupts_clear();  
}

static int taos_write_byte(u8 reg)
{
	int  ret = 0;
	if ((ret = i2c_smbus_write_byte(taos_datap->client, reg)) < 0)
		dev_err(&taos_datap->client->dev, "failed in %s, reg=0x%x\n", __func__, reg);
	return ret;
}
static int taos_write_byte_data(u8 reg, u8 data)
{
	int  ret = 0;
	if ((ret = i2c_smbus_write_byte_data(taos_datap->client, reg, data)) < 0)
		dev_err(&taos_datap->client->dev, "failed in %s, reg=0x%x data = 0x%x\n", __func__, reg, data);
	return ret;
}
static int taos_sensors_als_on(void) {
    int  ret = 0, i = 0;
    u8 itime = 0, reg_val = 0;

    for (i = 0; i < TAOS_FILTER_DEPTH; i++)
    	lux_history[i] = -ENODATA;
    ret = taos_write_byte(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|TAOS_TRITON_CMD_ALS_INTCLR);
    itime = (((taos_cfgp->als_time/50) * 18) - 1);
    itime = (~itime);
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME, itime);
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN, taos_cfgp->prox_gain);
    reg_val = (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON);
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL, reg_val);
    if (ret){
	    dev_err(&taos_datap->client->dev, "failed in als_on\n");
    } else
	    dev_info(&taos_datap->client->dev, "als on use als_time");
    return ret;
}	

static int prox_calibration(void) 
{
    int prox_sum = 0, prox_mean = 0, prox_max = 0;
    int ret = 0, i = 0;
    u8 reg_val = 0;
    struct taos_prox_info prox_cal_info[CAL_NUMBER];

    taos_datap->prox_cal_status = PROX_CAL_RUNNING;
    dev_info(&taos_datap->client->dev, "calibrating prox\n");
    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME, 0xF7);
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME, taos_cfgp->prox_time);
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME, taos_cfgp->wait_time);
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_CFG, taos_cfgp->prox_config);
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT, taos_cfgp->prox_pulse_cnt);
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN, taos_cfgp->prox_gain);
    if((chip_id == 0x39) && !ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_OFFSET, taos_cfgp->prox_offset);
    reg_val = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_ADC_ENBL;
    if (!ret)
	    ret = taos_write_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL, reg_val);
    if (ret){
	    dev_err(&taos_datap->client->dev, "prox calibrate failed\n");
	    taos_datap->prox_cal_status = PROX_CAL_FAIL_REG;
	    return ret;
    }

    prox_sum = 0;
    prox_max = 0;
    for (i = 0; i < CAL_NUMBER; i++) {
	    mdelay(30);
	    if ((ret = taos_prox_poll(&prox_cal_info[i])) < 0) {
		    dev_err(&taos_datap->client->dev,"call to prox_poll failed in ioctl prox_calibrate\n");
		    taos_datap->prox_cal_status = PROX_CAL_FAIL_REG;
		    return ret;
	    }
	    prox_sum += prox_cal_info[i].prox_data;
	    if (prox_cal_info[i].prox_data > prox_max)
		    prox_max = prox_cal_info[i].prox_data;
	    printk("taos: cal data i=%d, data=0x%x\n", i, prox_cal_info[i].prox_data);
    }

    prox_mean = prox_sum/CAL_NUMBER;
    if ((prox_max - prox_mean) < 150) {
	    int prox_diff = (prox_max - prox_mean) > 12 ? (prox_max - prox_mean) : 12;	
	    taos_cfgp->prox_threshold_lo = (((prox_diff * 180) + 50)/100) + prox_mean;
	    taos_cfgp->prox_threshold_hi = taos_cfgp->prox_threshold_lo + 0x20;
	    dev_info(&taos_datap->client->dev, "first prox_threshold_lo = 0x%x, prox_threashold_hi = 0x%x, max=0x%x, mean=0x%x\n", 
			    taos_cfgp->prox_threshold_lo, taos_cfgp->prox_threshold_hi, prox_max, prox_mean);
	    taos_datap->prox_cal_status = PROX_CAL_SUCCEED;
    } else {
	    taos_cfgp->prox_threshold_hi = 0x3ff;
	    dev_info(&taos_datap->client->dev, "data difference too big\n");
    }
    if ((taos_cfgp->prox_threshold_hi >= 850) ||(taos_cfgp->prox_threshold_hi == taos_cfgp->prox_threshold_lo)){
	    dev_info(&taos_datap->client->dev, "threshold too big, use default threshold\n");
	    taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
	    taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
	    taos_datap->prox_cal_status = PROX_CAL_FAIL_TOO_BIG;
    }  else if  (taos_cfgp->prox_threshold_hi<350){
	    dev_info(&taos_datap->client->dev, "threshold too small, use default threshold\n");
	    taos_cfgp->prox_threshold_hi = 400;
	    taos_cfgp->prox_threshold_lo = 360;			
	    taos_datap->prox_cal_status = PROX_CAL_FAIL_TOO_SMALL;
    }

    for (i = 0; i < sizeof(taos_triton_reg_init); i++){
	    if(i !=11){
		    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i)), taos_triton_reg_init[i]))) < 0) {
			    dev_err(&taos_datap->client->dev, "failed to init all reg\n");
			    taos_datap->prox_cal_status = PROX_CAL_FAIL_REG;
			    return ret;
		    }
	    }
    }

    return (ret);
}

static ssize_t proxdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    u16 proxdata[3];
	int i = 0;

	for (i = 0; i < 3; i++) {
		mdelay(150);
		proxdata[i] = (i2c_smbus_read_word_data(taos_datap->client, (TAOS_TRITON_CMD_REG | TAOS_TRITON_CMD_WORD_BLK_RW| TAOS_TRITON_PRX_LO)));
	}

	return sprintf(buf, "0x%x, 0x%x, 0x%x\n", proxdata[0], proxdata[1], proxdata[2]);
}
static DEVICE_ATTR(prxdata, S_IRUGO, proxdata_show, NULL);

static ssize_t prxpulse_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", taos_cfgp->prox_pulse_cnt);
}
static DEVICE_ATTR(prxpulse, S_IRUGO, prxpulse_show, NULL);

static ssize_t prxthreshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "lo=0x%x, hi=0x%x\n", taos_cfgp->prox_threshold_lo, taos_cfgp->prox_threshold_hi);
}
static DEVICE_ATTR(prxthreshold, S_IRUGO, prxthreshold_show, NULL);

static ssize_t proxoffset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%x\n", taos_cfgp->prox_offset);
}
static ssize_t proxoffset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	dev_info(&taos_datap->client->dev, "set proxoffset to %ld", val);
	if((val >=0) && (val <= 0xff))
		taos_cfgp->prox_offset = val;
	return count; 
}
static DEVICE_ATTR(proxoffset, S_IRUGO | S_IWUSR, proxoffset_show, proxoffset_store);

static ssize_t prox_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", taos_datap->prox_enable);
}
static ssize_t prox_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret = 0;
	u8 reg_val = 0;

        if (strict_strtoul(buf, 10, &val))
                return -EINVAL;
        dev_info(&taos_datap->client->dev, "~~~~~PROX set poll enable %ld~~~~", val);

	mutex_lock(&taos_datap->taos_mutex);
	if(val){
		ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_ALS_TIME, taos_cfgp->prox_als_time);
		if(!ret)
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_TIME, taos_cfgp->prox_time);
		if(!ret)
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_WAIT_TIME, taos_cfgp->wait_time);
		if(!ret)
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_INTERRUPT, taos_cfgp->pers);
		if(!ret)
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_CFG, taos_cfgp->prox_config);
		if(!ret)
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_PRX_COUNT, taos_cfgp->prox_pulse_cnt);
		if(!ret)
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_GAIN, taos_cfgp->prox_gain);
		if ((chip_id == 0x39) && !ret)
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_PRX_OFFSET, taos_cfgp->prox_offset);
		reg_val = TAOS_TRITON_CNTL_PROX_DET_ENBL | TAOS_TRITON_CNTL_PWRON | TAOS_TRITON_CNTL_PROX_INT_ENBL | 
			TAOS_TRITON_CNTL_ADC_ENBL;
		if(!ret)
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL, reg_val);
		if(ret){
			dev_err(&taos_datap->client->dev, "prox_on failed\n");
			mutex_unlock(&taos_datap->taos_mutex);
			return ret; 
		}    
		taos_datap->prox_enable = 1; 
		taos_prox_threshold_set(1);
	} else {
		ret = taos_write_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL, 0x00);
		if (ret){
			dev_err(&taos_datap->client->dev, "prox off failed\n");
			mutex_unlock(&taos_datap->taos_mutex);
			return ret;
		}
		cancel_work_sync(&taos_datap->work);
		taos_datap->prox_enable = 0;

		if(taos_datap->als_enable)
			taos_sensors_als_on();
	}
	mutex_unlock(&taos_datap->taos_mutex);

	return count; 
}
static DEVICE_ATTR(prox_enable, S_IRUGO | S_IWUSR, prox_enable_show, prox_enable_store);

static ssize_t prox_cal_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", taos_datap->prox_cal_status);
}
static ssize_t prox_cal_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        dev_info(&taos_datap->client->dev, "~~~~~prox calibrate start~~~~");
	mutex_lock(&taos_datap->taos_mutex);
	if (taos_datap->prox_enable || taos_datap->als_enable) {
		taos_datap->prox_cal_status = PROX_CAL_FAIL_PROX_OR_ALS_ENABLED;
		dev_err(&taos_datap->client->dev, "prox cal not execute due to %s", taos_datap->prox_enable ? "prox enabled" : "als enabled");
		mutex_unlock(&taos_datap->taos_mutex);
		return -1;
	}
	prox_calibration();
	mutex_unlock(&taos_datap->taos_mutex);
	return count;
}
static DEVICE_ATTR(prox_cal, S_IRUGO | S_IWUSR, prox_cal_show, prox_cal_store);

static ssize_t als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", taos_datap->als_enable);
}
static ssize_t als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret = 0;

        if (strict_strtoul(buf, 10, &val))
                return -EINVAL;
        dev_info(&taos_datap->client->dev, "~~~~~ALS set poll enable %ld~~~~", val);

	mutex_lock(&taos_datap->taos_mutex);
	if(val) {
		if(taos_datap->prox_enable)
			dev_info(&taos_datap->client->dev, "als on use prox_als_time");
		else
			taos_sensors_als_on();
		taos_datap->als_enable = 1;
	} else {
		if (taos_datap->prox_enable) {
			dev_info(&taos_datap->client->dev, "als off use prox_als_time");
		} else {
			ret = taos_write_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL, 0x00);
			if (ret){
				dev_err(&taos_datap->client->dev, "als off failed\n");
				mutex_unlock(&taos_datap->taos_mutex);
				return ret;
			}
		}
		taos_datap->als_enable = 0;
	}
	mutex_unlock(&taos_datap->taos_mutex);
	
	return count;
}
static DEVICE_ATTR(als_enable, S_IRUGO | S_IWUSR, als_enable_show, als_enable_store);

static ssize_t als_pollrate_ms_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", taos_datap->als_poll_interval);
}
static ssize_t als_pollrate_ms_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long interval_ms;

        if (strict_strtoul(buf, 10, &interval_ms))
                return -EINVAL;

	interval_ms = max((unsigned int)interval_ms, taos_datap->als_min_interval);
        dev_info(&taos_datap->client->dev, "als set poll interval %ld", interval_ms);
	taos_datap->als_poll_interval = interval_ms;
	taos_datap->als_dev->poll_interval = interval_ms;

	return count;
}
static DEVICE_ATTR(als_pollrate_ms, S_IRUGO | S_IWUSR, als_pollrate_ms_show, als_pollrate_ms_store);

static struct attribute *sensor_attr[] = {
        &dev_attr_prxdata.attr,
	&dev_attr_prxpulse.attr,
	&dev_attr_prxthreshold.attr,
	&dev_attr_proxoffset.attr,
	&dev_attr_prox_enable.attr,
	&dev_attr_prox_cal.attr,
	&dev_attr_als_enable.attr,
	&dev_attr_als_pollrate_ms.attr,
        NULL
};

static struct attribute_group sensor_dev_attr_grp = {
        .attrs = sensor_attr,
};

static int prox_input_open(struct input_dev *dev)
{
	dev_info(&taos_datap->client->dev, "enable wake");
	enable_irq_wake(taos_datap->client->irq);
	return 0;
}
static void prox_input_close(struct input_dev *dev)
{
	dev_info(&taos_datap->client->dev, "disable wake");
	disable_irq_wake(taos_datap->client->irq);
}
static int taos_prox_setup_input(struct taos_data *data)
{
	struct input_dev *input_dev;
	int ret = 0;

	input_dev = input_allocate_device();
	if(!input_dev){
		dev_err(&data->client->dev, "can't allocate prox input device\n");	
		return -ENOMEM;
	}
	data->prox_dev = input_dev;
	input_dev->name = "proximity";
	input_dev->open = prox_input_open;
	input_dev->close = prox_input_close;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(taos_datap->prox_dev, ABS_DISTANCE, 0, 100, 0, 0);
	if ((ret = input_register_device(input_dev))) {
		dev_err(&data->client->dev, "can't register prox input device\n");
		input_free_device(input_dev);
		return ret;
	}

	return 0;
}

static int taos_als_setup_input(struct taos_data *data)
{
        int err;
        struct input_polled_dev *poll_dev;

        poll_dev = input_allocate_polled_device();
        if (!poll_dev) {
                dev_err(&data->client->dev,
                        "Failed to allocate als polled device\n");
                return -ENOMEM;
        }

        data->als_dev = poll_dev;

        poll_dev->private = data;
        poll_dev->poll = als_poll;
        poll_dev->poll_interval = data->als_poll_interval;

        set_bit(EV_ABS, poll_dev->input->evbit);
        input_set_abs_params(poll_dev->input, ABS_MISC, 0, 10000, 0, 0);
        poll_dev->input->name = "lightsensor-level";
        poll_dev->input->id.bustype = BUS_I2C;
        poll_dev->input->dev.parent = &data->client->dev;

        err = input_register_polled_device(poll_dev);
        if (err) {
                dev_err(&data->client->dev,
                        "Unable to register polled device, err=%d\n", err);
                input_free_polled_device(poll_dev);
                return err;
        }

        return 0;
}

static int taos_probe(struct i2c_client *clientp, const struct i2c_device_id *idp) 
{
    int ret = 0;

    dev_info( &clientp->dev, "%s begin\n", __func__);
    if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&clientp->dev, "i2c smbus byte data functions unsupported\n");
	ret = -EOPNOTSUPP;
	return ret;
    }
    if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
        dev_err(&clientp->dev, "i2c smbus word data functions unsupported\n");
    }
    if (!i2c_check_functionality(clientp->adapter, I2C_FUNC_SMBUS_BLOCK_DATA)) {
        dev_err(&clientp->dev, "i2c smbus block data functions unsupported\n");
    }

    sensor_power_onoff(true);
    if ((ret = (i2c_smbus_write_byte(clientp, (TAOS_TRITON_CMD_REG | (TAOS_TRITON_CNTRL + TAOS_TRITON_CHIPID))))) < 0) {
	    dev_err(&clientp->dev, "i2c_smbus_write_byte() to chipid reg failed in %s\n", __func__);
	    goto power_fail;
    }
    chip_id = i2c_smbus_read_byte(clientp);	
    dev_info( &clientp->dev, "taos chip id is:0x%x\n",chip_id);
    if((chip_id != 0x20) && (chip_id != 0x29) && (chip_id != 0x39)) {
	    dev_err(&clientp->dev, " chip_id = %d, not suported\n",chip_id);
	    goto power_fail;
    }

    taos_datap = kzalloc(sizeof(struct taos_data), GFP_KERNEL);
    if (!taos_datap) {
	    dev_err(&clientp->dev, "kzalloc for struct taos_data failed\n");
	    ret = -ENOMEM;
	    goto power_fail;
    }
	
    if (clientp->dev.of_node) {
	    u32 temp_val;
		struct pinctrl *pinctrl;

		pinctrl = devm_pinctrl_get_select_default(&clientp->dev);
		if (IS_ERR(pinctrl)) {
			dev_warn(&clientp->dev, "pins are not configured from the driver\n");
		    goto get_gpio_fail;
		} 
	    temp_val = of_get_gpio(clientp->dev.of_node, 0); 
	    if (gpio_is_valid(temp_val)) {
		    printk(KERN_INFO "taos irq get gpio %d\n", temp_val);
		    gpio_request(temp_val, "taos_irq");
		    gpio_direction_input(temp_val);
		    clientp->irq = gpio_to_irq(temp_val);
		    taos_datap->irq_gpio = temp_val;
	    } else {
		    dev_err(&clientp->dev,"invalid gpio");
		    goto get_gpio_fail;
	    }

	    ret = of_property_read_u32(clientp->dev.of_node, "taos,min-interval", &temp_val);
	    if (ret) {
		    dev_err(&clientp->dev, "Unable to read min-interval");
		    goto property_read_fail;
	    } else {
		    taos_datap->als_min_interval = temp_val;
	    }
	    ret = of_property_read_u32(clientp->dev.of_node, "taos,poll-interval", &temp_val);
	    if (ret) {
		    dev_err(&clientp->dev, "Unable to read poll-interval");
		    goto property_read_fail;
	    } else {
		    taos_datap->als_poll_interval = temp_val;
	    }

    }
    wake_lock_init(&taos_datap->taos_wake_lock, WAKE_LOCK_SUSPEND, "taos-wake-lock");
    taos_datap->client = clientp;
    i2c_set_clientdata(clientp, taos_datap);
    INIT_WORK(&(taos_datap->work),taos_work_func);
    mutex_init(&taos_datap->taos_mutex);

    ret = taos_prox_setup_input(taos_datap);
    if (ret) {
	    dev_err(&clientp->dev, "failed setup prox input device\n");
	    goto prox_setup_fail;
    }    

    ret =  request_threaded_irq( clientp->irq, taos_irq_handler, taos_irq_handler_thread,
		    IRQ_TYPE_EDGE_FALLING, "tms2771_irq", taos_datap);
    if (ret) {
	    dev_err(&clientp->dev, "%s %s fail to request irq, ret = %d\n", __FILE__, __func__, ret);
	    goto req_irq_fail;
    }

    ret = taos_als_setup_input(taos_datap);
    if (ret) {
	    dev_err(&clientp->dev, "failed setup als input device\n");
	    goto als_setup_fail;
    }    
    ret = sysfs_create_group(&clientp->dev.kobj, &sensor_dev_attr_grp);
    if (ret) {
	    dev_err(&clientp->dev, "create sys file failed\n");
	    goto sysfs_create_fail;
    }    

   strlcpy(clientp->name, TAOS_DEVICE_ID, I2C_NAME_SIZE);
	
   if (!(taos_cfgp = kzalloc(sizeof(struct taos_cfg), GFP_KERNEL))) {
        dev_err(&clientp->dev, "kmalloc for struct taos_cfg failed\n");
		ret = -ENOMEM;
		goto malloc_fail;
    }
    taos_cfgp->als_time = als_time_param;
    taos_cfgp->scale_factor = scale_factor_param;
    taos_cfgp->als_threshold_hi = als_threshold_hi_param;//iVIZM
    taos_cfgp->als_threshold_lo = als_threshold_lo_param;//iVIZM
    taos_cfgp->prox_threshold_hi = prox_threshold_hi_param;
    taos_cfgp->prox_threshold_lo = prox_threshold_lo_param;
    taos_cfgp->prox_als_time = prox_als_time_param;
    taos_cfgp->prox_time = prox_time_param;
    taos_cfgp->wait_time = wait_time_param;
    taos_cfgp->pers = pers_param;
    taos_cfgp->prox_config = prox_config_param;
	taos_cfgp->prox_pulse_cnt = prox_pulse_cnt_param;
    taos_cfgp->prox_gain = prox_gain_param;
    taos_cfgp->als_gain = taos_cfgp->prox_gain & 0x3;
    taos_cfgp->prox_offset = prox_offset_param;
    sat_als = (256 - taos_cfgp->prox_als_time) << 10;

    /*dmobile ::power down for init ,Rambo liu*/
    printk("Rambo::light sensor will pwr down \n");
    if ((ret = (i2c_smbus_write_byte_data(taos_datap->client, (TAOS_TRITON_CMD_REG|TAOS_TRITON_CNTRL), 0x00))) < 0) {
        printk(KERN_ERR "TAOS:Rambo, i2c_smbus_write_byte_data failed in power down\n");
		goto smbus_fail;
    }

   return (ret);

smbus_fail:
   kfree(taos_cfgp);
malloc_fail:
   sysfs_remove_group(&clientp->dev.kobj, &sensor_dev_attr_grp);
sysfs_create_fail:
   input_unregister_polled_device(taos_datap->als_dev);
   input_free_polled_device(taos_datap->als_dev);
als_setup_fail:
   free_irq(clientp->irq, NULL);
req_irq_fail:
   input_unregister_device(taos_datap->prox_dev);
prox_setup_fail:
   mutex_destroy(&taos_datap->taos_mutex);
   wake_lock_destroy(&taos_datap->taos_wake_lock);
property_read_fail:
   gpio_free(taos_datap->irq_gpio);
get_gpio_fail:
   kfree(taos_datap);
power_fail:
   sensor_power_onoff(false);
   dev_err(&clientp->dev, "taos probe failed");
   return ret;
}


static int taos_remove(struct i2c_client *client) {
	free_irq(taos_datap->client->irq, NULL);
	kfree(taos_cfgp);
	sysfs_remove_group(&taos_datap->client->dev.kobj, &sensor_dev_attr_grp);
	input_unregister_device(taos_datap->prox_dev);
	input_unregister_polled_device(taos_datap->als_dev);
	input_free_polled_device(taos_datap->als_dev);
	mutex_destroy(&taos_datap->taos_mutex);
	wake_lock_destroy(&taos_datap->taos_wake_lock);
	gpio_free(taos_datap->irq_gpio);
	kfree(taos_datap);
	sensor_power_onoff(false);
	return 0;
}

static int taos_resume(struct device *dev) {
	if(!taos_datap->prox_enable){
		printk("taos: resume to power on\n");
		sensor_power_onoff(true);
	}
	return 0;
}

static int taos_suspend(struct device *dev) {
	if(!taos_datap->prox_enable){
		sensor_power_onoff(false);
		printk("taos: suspend to power off\n");
	}
	return 0;
}

static struct proc_dir_entry *taos_info_proc_file;
static int taos_proc_show(struct seq_file *m, void *v)
{
        return seq_printf(m, "0x%x\n", chip_id);
}
static int taos_proc_open(struct inode *inode, struct file *file)
{
        return single_open(file, taos_proc_show, NULL);
}

static const struct file_operations taos_proc_fops = {
        .open           = taos_proc_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static void create_taos_info_proc_file(void)
{
  taos_info_proc_file = proc_create("driver/alsprx", 0644, NULL, &taos_proc_fops);
  if (!taos_info_proc_file) {
		printk(KERN_INFO "taos proc file create failed!\n");
   }
}

static void remove_taos_info_proc_file(void)
{
	if(taos_info_proc_file){
		remove_proc_entry("driver/alsprx", NULL);
		taos_info_proc_file = NULL;
	}
}

static struct i2c_device_id taos_idtable[] = {
    {TAOS_DEVICE_ID, 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, taos_idtable);

static struct of_device_id taos_match_table[] = {
        { .compatible = "taos,tmd277x", },
        { },
};

static SIMPLE_DEV_PM_OPS(taos_pm_ops, taos_suspend, taos_resume);

static struct i2c_driver taos_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = TAOS_DEVICE_NAME,
		.of_match_table = taos_match_table,
		.pm = &taos_pm_ops,
    },
    .id_table = taos_idtable,
    .probe = taos_probe,
    .remove = taos_remove,   
};
//zhangjian modify for lattice hub
#ifdef CONFIG_IR_LATTICE
int taos_init(void) {
    int ret = 0;

    if ((ret = (i2c_add_driver(&taos_driver))) < 0) {
        printk(KERN_ERR "TAOS: i2c_add_driver() failed in taos_init()\n");
    }
    if(0xff != chip_id)
		create_taos_info_proc_file();
		
    return (ret);
}

static void taos_exit(void) {
    i2c_del_driver(&taos_driver);
    if(0xff != chip_id){
    	remove_taos_info_proc_file();
    }
}

EXPORT_SYMBOL(taos_init);
#else
static int __init taos_init(void) {
    int ret = 0;

    if ((ret = (i2c_add_driver(&taos_driver))) < 0) {
        printk(KERN_ERR "TAOS: i2c_add_driver() failed in taos_init()\n");
    }
    if(0xff != chip_id)
		create_taos_info_proc_file();
		
    return (ret);
}

static void taos_exit(void) {
    i2c_del_driver(&taos_driver);
    if(0xff != chip_id){
    	remove_taos_info_proc_file();
    }
}
late_initcall(taos_init);
#endif
//end modify
MODULE_AUTHOR("John Koshi - Surya Software");
MODULE_DESCRIPTION("TAOS ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");

//late_initcall(taos_init);
module_exit(taos_exit);

