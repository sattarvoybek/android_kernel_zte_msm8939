/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


/* check which notifying callback method is used */
#include <linux/spi/spidev_lattice_reg.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define SUSPEND_LEVEL 1
#endif

#define LATTICE_SPIDEV_LOG(fmt, args...) printk("[lattice]:" fmt"\n", ##args) 
#define LATTICE_SPIDEV_ERR(fmt, args...) printk(KERN_ERR"[lattice]" fmt"\n", ##args )

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */



static DECLARE_BITMAP(minors, N_SPI_MINORS);

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)


//struct delayed_work input_work_tx;
//struct delayed_work input_work_rx;
struct delayed_work cdone_work_firmwaredownload;
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

#define MEMBLOCK                69632
static  char fw_path[]= "/system/etc/firmware/irda/irda_top_bitmap.bin";
uint8_t pirdata[69632]={0};
char acc_txbuf[1028]={0};
u8 rxbuf_test[4096]={0};
u8 txbuf_test[139]={0x02, 0x08, 0x00, 0x5A, 0x01, 0xAD, 0x00, 0x16, 0x00, 0x16,
				    0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16,
				    0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16,
				    0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16,
				    0x00, 0x41, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x41,
				    0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16,
				    0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16,
				    0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 
				    0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x16,
				    0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16,
				    0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16,
				    0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16, 0x00, 0x16,
				    0x00, 0x41, 0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x41,
				    0x00, 0x16, 0x00, 0x41, 0x00, 0x16, 0x00, 0x80, 0x01 };
static char ir_status[16] = "recv on";
static uint gsensor_init_flag = -1;

extern int lis3dh_acc_init(void);
extern int akm_compass_init(void);
extern int taos_init(void);

//extern ssize_t spiphy_dev_write(u8 bits_per_word,u32 speed_hz,const char *buf,size_t count);
unsigned char spi_func_rx(unsigned short addr);
static int download_flag = 0;
/*
 * This can be used for testing the controller, given the busnum and the
 * cs required to use. If those parameters are used, spidev is
 * dynamically added as device on the busnum, and messages can be sent
 * via this interface.
 */
static int busnum = 3;
module_param(busnum, int, S_IRUGO);
MODULE_PARM_DESC(busnum, "bus num of the controller");

static int chipselect = 0;
module_param(chipselect, int, S_IRUGO);
MODULE_PARM_DESC(chipselect, "chip select of the desired device");

static int maxspeed = 1000000;
module_param(maxspeed, int, S_IRUGO);
MODULE_PARM_DESC(maxspeed, "max_speed of the desired device");

static int spimode = SPI_MODE_0;
module_param(spimode, int, S_IRUGO);
MODULE_PARM_DESC(spimode, "mode of the desired device");

static struct spi_device *spi;
static struct spi_device *spi_irq;

static int spi_data_tx(struct spi_device *spi,char *txbuf, int len);
static int spi_data_rx(struct spi_device *spi,char *txbuf, char *rxbuf, int tx_len, int rx_len);
//static void ice_spi_report_values(struct spi_device *spi);
static void ir_enable_init(void);
//static void ice_spi_report_values_sma(struct spi_device *spi);
static int debug_en=0;
static int sma_detect_en=0; 				/* Smart Alert detect */
static int sleep_status=0;
//static int sma_status=0;
void ir_spi_cs(int level)
{

  gpio_set_value(GPIO_SPI_CS, level);
  mdelay(10);
  
  return;
}

static int check_ice40_state(void)
{
  int level = -1;
  level = gpio_get_value(GPIO_CDONE);
  return level;
}

static int ir_download_fw(void * ptr)
{
  //uint8_t buf[100]={0};
  uint8_t *ptrdata_tmp = NULL;
  //int i;
  int ret = 0;
  printk("ir_download_fw in  ptr=%p!\n",ptr);
  download_flag = 1;
  ptrdata_tmp=kzalloc(MEMBLOCK+8+100,GFP_KERNEL);
  if(!ptrdata_tmp)
  {
	printk("kzalloc error\n");
	return -1;
  }
  memcpy((ptrdata_tmp+8),ptr,MEMBLOCK);
  /*
  //send 8 clocks
  for(i=0; i<8; i++)
  {
    ret=spi_data_tx(spi, buf, 8);
    if(ret == -1)
    {
        printk(KERN_ERR"spiphy_dev_write failed\n");
        return -1;
    }
  }
  */
  //send data from bin file
  //for(i=0; i<MEMBLOCK; i++)
  //{
    ret=spi_data_tx(spi, ptrdata_tmp, MEMBLOCK+100+8);
    if(ret == -1)
    {
        printk(KERN_ERR"write firmware failed\n");
		kfree(ptrdata_tmp);
        return -1;
    }
  //}
  printk("ir_download_fw in  ptr2=%p!\n",ptr);
  /*
  //send 100 clocks
  for(i=0; i<100; i++)
  {
	ret=spi_data_tx(spi, buf, 100);
     if(ret == -1)
    {
        printk(KERN_ERR"spiphy_dev_write 2 failed\n");
        return -1;
    }
  }
  */
	kfree(ptrdata_tmp);
	return 0;
}

static void ir_reset(void)
{
  gpio_set_value(GPIO_VCC_IO2, 1);
  mdelay(10);
  
  gpio_set_value(GPIO_VCC_IO2, 0);
  mdelay(10);
  
  gpio_set_value(GPIO_VCC_IO2, 1);
  mdelay(10);
  return;
}



static int ir_get_fw_block(char *buf, int len, void *image)
{
  struct file *fp = (struct file *)image;
  int rdlen;

  if (!image)
    return 0;

  rdlen = kernel_read(fp, fp->f_pos, buf, len);
  if (rdlen > 0)
    fp->f_pos += rdlen;

  return rdlen;
}

static void ir_close_fw(void *fw)
{
  if (fw)
    filp_close((struct file *)fw, NULL);

  return;
}
static void * ir_open_fw(char *filename)
{
  struct file *fp;

  fp = filp_open(filename, O_RDONLY, 0);
  if (IS_ERR(fp))
  {
    printk("filp_open fail \n");
    fp = NULL;
  }

  return fp;
}

/*
static void ir_enable(void)
{
  gpio_set_value(GPIO_VCC_1_2, 1);
  mdelay(10);
  return;
}
*/
/*---------------------------------------------------------------------------------------*/
static int ir_config_ice40(int type, char *pfw_path)
{
 
  void *fw = NULL;
  //uint8_t *memblock = NULL, *memptr;
  int len ;
  printk(" ir_config_ice40 in!\n");
  printk(" check cdone gpio=%d!\n",check_ice40_state());
  //ir_enable();
  // read fw to ram
  printk(" zhangjian_pfw_path is %s!\n",pfw_path);
  fw = ir_open_fw(pfw_path);
  if(fw == NULL)
  {
    printk("ir_open_fw fail !\n");
    goto err;
  }
  while ((len = ir_get_fw_block((char*)pirdata, MEMBLOCK, fw))) {
      printk("len=%d \n", len);
  }

  //
  // Reset the ice40 device
  //
  ir_spi_cs(0);
  //Set SPI_SS_B low
  ir_reset();
  ir_download_fw(pirdata);
  //ir_spi_cs(1);
  download_flag = 0;
  // Verify successful configuration
    if (check_ice40_state())
    {
        strncpy(ir_status, "success", 7);
		printk("fm download ok\n ");
		if(gsensor_init_flag != 1)
		{
			mdelay(100);			
			lis3dh_acc_init();
      akm_compass_init();
      taos_init();
			gsensor_init_flag = 1;
		}
	
        return 0;
    }
    else
    {
        strncpy(ir_status, "fail", 4);
		printk("fm download failed\n ");
        return -1;
    }

 err:
  if (fw)
    ir_close_fw(fw);
  return -1;
 }

/*---------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------*/

/*
If compiler shows unused function warning for spi_suspend & spi_resume, then please make sure if the
any one of the notifing methods are enabled i.e CONFIG_HAS_EARLYSUSPEND/CONFIG_FB 

*/

static int spi_suspend(struct spi_device *spi)
{
	char txbuf[6]={0};
	int i=0;
	int tx_len;
	char read_data=0x00;
	//char Pedometer_txbuf[4]={0x02, 0x10, 0x03, 0x10};

#ifdef READ_CLKEN_REG
	int rx_len=1;
	//char rxbuf[6]={0};	
	txbuf[i++]=SPI_READ; 
	txbuf[i++]=H_BYTE(CLK_EN_REG_ADR); 
	txbuf[i++]=L_BYTE(CLK_EN_REG_ADR); 
	tx_len = i;
	spi_data_rx(spi,txbuf,txbuf,tx_len,rx_len);
	read_data = txbuf[4];      // Remove dummy byte
	i=0;
#endif
    //LATTICE_SPIDEV_LOG("%s gpio-cdone=%d\n",__func__,check_ice40_state());
	
	sleep_status=1;
	txbuf[i++]=SPI_WRITE; 
	txbuf[i++]=H_BYTE(CLK_EN_REG_ADR); 
	txbuf[i++]=L_BYTE(CLK_EN_REG_ADR); 
	//txbuf[i++]= read_data | 0x04;
	txbuf[i++]= read_data | 0x02;
	tx_len = i;
	spi_data_tx(spi,txbuf,tx_len);
	//pedometer enable
	//spi_data_tx(spi,Pedometer_txbuf,4);
	return 0;
}

static int spi_resume(struct spi_device *spi)
{
	char txbuf[6]={0};	
	int i=0;
	int tx_len;
	char read_data=0x00;
	//char Pedometer_txbuf[4]={0x02, 0x10, 0x03, 0x00};

#ifdef READ_CLKEN_REG
	int rx_len=1;
	//char rxbuf[6]={0};	
	txbuf[i++]=SPI_READ; 
	txbuf[i++]=H_BYTE(CLK_EN_REG_ADR); 
	txbuf[i++]=L_BYTE(CLK_EN_REG_ADR);
	tx_len = i;
	spi_data_rx(spi,txbuf,txbuf,tx_len, rx_len);
	read_data = txbuf[4];       // Remove dummy byte
	i=0;
#endif
    //LATTICE_SPIDEV_LOG("%s gpio-cdone=%d\n",__func__,check_ice40_state());
	txbuf[i++]=SPI_WRITE; 
	txbuf[i++]=H_BYTE(CLK_EN_REG_ADR); 
	txbuf[i++]=L_BYTE(CLK_EN_REG_ADR); 
	//txbuf[i++]=read_data & (~0x04);
	txbuf[i++]=read_data |0x02;
	//txbuf[i++]=(read_data |0x02)&0xEF ;
	tx_len = i;
	spi_data_tx(spi,txbuf,tx_len);
	//pedometer disable
	//spi_data_tx(spi,Pedometer_txbuf,4);
	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct spidev_data *spidev =
		container_of(self, struct spidev_data, fb_notif);

    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	if (evdata && evdata->data && spidev && spidev->spi) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK)
			{
				//if(check_ice40_state()==0)   //if lattice has been power down ,should download firmware again
					//ir_config_ice40(1,fw_path);
				spi_resume(spidev->spi);
				}
			else if (*blank == FB_BLANK_POWERDOWN)
				spi_suspend(spidev->spi);
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void spi_early_suspend(struct early_suspend *h)
{
	struct spidev_data *spidev = container_of(h, struct spidev_data,
						early_suspend);
    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	spi_suspend(spidev->spi);
}

static void spi_late_resume(struct early_suspend *h)
{
	struct spidev_data *spidev = container_of(h, struct spidev_data,
						early_suspend);
    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	spi_resume(spidev->spi);
}

#endif
/*---------------------------IRQ Function----------------------------------------------*/

/* Read data whenever interrupt occur */
static int spi_data_rx(struct spi_device *spi,char *txbuf, char *rxbuf, int tx_len,int rx_len)
{
#if 1
	int retval;
	struct spi_message msg;
	struct spidev_data *spidev = spi_get_drvdata(spi);
	int len= tx_len + rx_len +1; // Adding +1 for dummy byte
	struct spi_transfer xfer = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len = len,      
		.cs_change = 0,
		.speed_hz=CLK_FREQ,
	};
    //LATTICE_SPIDEV_LOG("SPI data reading start \n");
	
	mutex_lock(&spidev->buf_lock);
	spi_message_init(&msg);

	spi_message_add_tail(&xfer, &msg);

	retval = spi_sync(spi, &msg);
	if(retval < 0){
    		LATTICE_SPIDEV_ERR("spi data read failed\n");
		return retval;
	}
	mutex_unlock(&spidev->buf_lock);
#endif
	return retval;
}

/* Transmit data whenever interrupt occur */
static int spi_data_tx(struct spi_device *spi, char *txbuf, int len)
{
#if 1
	
	int retval;
	struct spi_message msg;
	struct spidev_data *spidev = spi_get_drvdata(spi);
	struct spi_transfer k_tmp={
				.tx_buf= txbuf,
				.rx_buf=NULL,
				.len=len,  
				.cs_change=0,
				.speed_hz=CLK_FREQ,
				};
	if (download_flag)
	{
		k_tmp.speed_hz = k_tmp.speed_hz*10;
	}

    //LATTICE_SPIDEV_LOG("SPI data tranmission start \n");
	mutex_lock(&spidev->buf_lock);
	spi_message_init(&msg);
	spi_message_add_tail(&k_tmp, &msg);
	retval = spi_sync(spi, &msg);
	if(retval < 0){
    		LATTICE_SPIDEV_ERR("spi data tranfer failed\n");
		return retval ;
	}
	mutex_unlock(&spidev->buf_lock);
#endif	
	return retval;
}	
/*
static void spi_work_func_tx(struct work_struct *work)
{
	struct spi_device *spi=spi_irq;
	char txbuf[2]={0xF0,0xA8};
	int len = sizeof(txbuf);	
    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	spi_data_tx(spi,txbuf,len);
	return;
}
*/
/*
static void spi_work_func_rx(struct work_struct *work)
{
	struct spi_device *spi=spi_irq;
	int i=0,tx_len,rx_len=1;
	char txbuf[6]={0};
	char read_data=0x00;
#if SMA_FEATURE_EN	
	char func_en=0x00;
#endif	
     	LATTICE_SPIDEV_LOG("%s\n", __func__);
	txbuf[i++]=SPI_READ;
	txbuf[i++]=H_BYTE(INTR_STAT_REG_ADR);
	txbuf[i++]=L_BYTE(INTR_STAT_REG_ADR);
	tx_len=i;
	spi_data_rx(spi,txbuf,txbuf,tx_len,rx_len);
	read_data= txbuf[++i];
	if(debug_en == 0){
#if SHK_FEATURE_EN
		if((read_data & SHAKE_INTR_DETECT) == SHAKE_INTR_DETECT ){
	   	ice_spi_report_values(spi);
    		LATTICE_SPIDEV_ERR("Shake interrupt found \n");
	}else{		
    		LATTICE_SPIDEV_ERR("Shake interrupt not found \n");
		}
#endif	
#if SMA_FEATURE_EN	
		if((read_data & SMA_INTR_DETECT) == SMA_INTR_DETECT ){
		   	ice_spi_report_values_sma(spi);
			i=0;
			txbuf[i++]=SPI_READ;
			txbuf[i++]=H_BYTE(FUNC_EN_REG_ADR);
			txbuf[i++]=L_BYTE(FUNC_EN_REG_ADR);
			tx_len=i;
			spi_data_rx(spi,txbuf,txbuf,tx_len,rx_len);
			func_en = txbuf[++i];		// Remove dummy byte
			
			i=0;
			txbuf[i++]=SPI_WRITE; 
			txbuf[i++]=H_BYTE(FUNC_EN_REG_ADR); 
			txbuf[i++]=L_BYTE(FUNC_EN_REG_ADR); 
	
			//Disable Smart Alert detect
			func_en = func_en & (~SMA_DETECT_EN);
			txbuf[i++]= func_en;    // Smart Alert detect 
			tx_len = i;		
			spi_data_tx(spi,txbuf,tx_len);
			sma_detect_en =0;
    			LATTICE_SPIDEV_ERR("SMA interrupt found \n");
		}else{		
    			LATTICE_SPIDEV_ERR("SMA interrupt not found \n");
		}
#endif	
	}
	else{
#if SHK_FEATURE_EN
		   	ice_spi_report_values(spi);
    			LATTICE_SPIDEV_ERR("debug_en, interrupt found \n");
#endif
#if SMA_FEATURE_EN	
		   	ice_spi_report_values_sma(spi);
#endif
	}
	return;
}
*/

static void spi_work_func_cdone_firmware(struct work_struct *work)
{
	
	if(check_ice40_state()==0)   
		ir_config_ice40(1,fw_path);
	if(check_ice40_state()==1)
		enable_irq(gpio_to_irq(GPIO_CDONE));
	return;
}
/* Interrupt handle for requested gpio */	
/*
static irqreturn_t spi_interrupt_handler(int irq,void *dev_id) {
	struct spi_device *spi= (struct spi_device *)dev_id;
	
	disable_irq_nosync(irq);
	spi_irq=spi;	
    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	
	schedule_delayed_work(&input_work_rx, 0);

	enable_irq(irq);
	return IRQ_HANDLED;
}

static int enable_spi_interrupt(struct spi_device *spi) {
    int err = 0;
    LATTICE_SPIDEV_LOG("%s\n",__func__);
    err= request_irq(gpio_to_irq(GPIO_INTR),spi_interrupt_handler,
                IRQF_TRIGGER_FALLING, "spidev_irq", spi);
    	
     //enable_irq(gpio_to_irq(GPIO_INTR)); 
		disable_irq(gpio_to_irq(GPIO_INTR));	 
    return err;
}

static int disable_spi_interrupt(struct spi_device *spi) {

     	LATTICE_SPIDEV_LOG("%s\n",__func__);
	disable_irq(gpio_to_irq(GPIO_INTR));  
	free_irq(gpio_to_irq(GPIO_INTR), spi);
	gpio_free(GPIO_INTR);
    	
	return 0;
}
*/
/*---------------------------IRQ Function End-----------------------------------------------------**/
/*---------------------------CDONE IRQ Function start-----------------------------------------------------**/

static irqreturn_t spi_interrupt_handler(int irq,void *dev_id) {
	struct spi_device *spi= (struct spi_device *)dev_id;
	
	disable_irq_nosync(irq);
	spi_irq=spi;	
    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	
	schedule_delayed_work(&cdone_work_firmwaredownload, 1*HZ);

	//enable_irq(irq);
	return IRQ_HANDLED;
}

static int enable_spi_interrupt(struct spi_device *spi) {
    int err = 0;
    LATTICE_SPIDEV_LOG("%s\n",__func__);
    err= request_irq(gpio_to_irq(GPIO_CDONE),spi_interrupt_handler,
                IRQF_TRIGGER_FALLING, "spidev_irq", spi);
    	
     enable_irq(gpio_to_irq(GPIO_CDONE)); 
		//disable_irq(gpio_to_irq(GPIO_CDONE));	 
    return err;
}

static int disable_spi_interrupt(struct spi_device *spi) {

     	LATTICE_SPIDEV_LOG("%s\n",__func__);
	disable_irq(gpio_to_irq(GPIO_CDONE));  
	free_irq(gpio_to_irq(GPIO_CDONE), spi);
	gpio_free(GPIO_CDONE);
    	
	return 0;
}
/*---------------------------CDONE IRQ Function End-----------------------------------------------------**/

/*-------------------------Create Event in /dev/input--------------------------------------------*/
/*
static void ice_spi_report_values(struct spi_device *spi)
{
	struct spidev_data *spidev = spi_get_drvdata(spi);
     	LATTICE_SPIDEV_LOG("%s\n",__func__);
	input_report_key(spidev->input_dev_detect, KEY_POWER, 0);
	input_sync(spidev->input_dev_detect);
	
	input_report_key(spidev->input_dev_detect, KEY_POWER, 1);
	input_sync(spidev->input_dev_detect);
	
}
*/
/*
#if SMA_FEATURE_EN
static void ice_spi_report_values_sma(struct spi_device *spi)
{
	struct spidev_data *spidev = spi_get_drvdata(spi);
	sma_status = !(sma_status);
#if 0
	input_report_key(spidev->input_dev_detect_sma, KEY_POWER, 0);
	input_sync(spidev->input_dev_detect_sma);
	
	input_report_key(spidev->input_dev_detect_sma, KEY_POWER, 1);
	input_sync(spidev->input_dev_detect_sma);
#endif 
	
	input_report_abs(spidev->input_dev_detect_sma, ABS_MISC, sma_status);
	input_sync(spidev->input_dev_detect_sma);
}
#endif 
*/
int ice_spi_input_open(struct input_dev *input)
{
     	LATTICE_SPIDEV_LOG("%s\n",__func__);
	//enable_irq(gpio_to_irq(GPIO_INTR));  
 	return 0;
}

void ice_spi_input_close(struct input_dev *input)
{
     	LATTICE_SPIDEV_LOG("%s\n",__func__);
	//disable_irq(gpio_to_irq(GPIO_INTR));  
 	return;
}
/*Function for create entry in input device*/
static int spidev_input_init(struct spidev_data *spidev)
{
	int err;
     	LATTICE_SPIDEV_LOG("spidev input init start = %s\n",__func__);
	spidev->input_dev_detect = input_allocate_device();
	if (!spidev->input_dev_detect) {
		err = -ENOMEM;
    		LATTICE_SPIDEV_ERR("input allocation failed\n");
		return err;
	}
	
	spidev->input_dev_detect->open = ice_spi_input_open;
	spidev->input_dev_detect->close = ice_spi_input_close;
	spidev->input_dev_detect->name = INPUT_DEVICE_NAME;
	spidev->input_dev_detect->id.bustype = BUS_SPI;
	spidev->input_dev_detect->dev.parent = &spidev->spi->dev;
	
	input_set_drvdata(spidev->input_dev_detect, spidev);
	
	set_bit(EV_KEY, spidev->input_dev_detect->evbit);
	set_bit(KEY_POWER, spidev->input_dev_detect->keybit);

	err = input_register_device(spidev->input_dev_detect);
	if (err) {
    		LATTICE_SPIDEV_ERR("input register device failed :%s\n",spidev->input_dev_detect->name);
		return err;
	}

	return 0;
}

static void spidev_input_cleanup(struct spidev_data *spidev)
{
	input_unregister_device(spidev->input_dev_detect);
	input_free_device(spidev->input_dev_detect);
}
#if SMA_FEATURE_EN
int ice_spi_input_open_sma(struct input_dev *input)
{
     	LATTICE_SPIDEV_LOG("%s\n",__func__);
 	return 0;
}

void ice_spi_input_close_sma(struct input_dev *input)
{
     	LATTICE_SPIDEV_LOG("%s\n",__func__);
 	return;
}
static int spidev_input_init_sma(struct spidev_data *spidev)
{
	int err;
     	LATTICE_SPIDEV_LOG("spidev input init start = %s\n",__func__);
	spidev->input_dev_detect_sma = input_allocate_device();
	if (!spidev->input_dev_detect_sma) {
		err = -ENOMEM;
    		LATTICE_SPIDEV_ERR("input allocation failed\n");
		return err;
	}
	
	spidev->input_dev_detect_sma->open = ice_spi_input_open_sma;
	spidev->input_dev_detect_sma->close = ice_spi_input_close_sma;
	spidev->input_dev_detect_sma->name = INPUT_DEVICE_NAME_SMA;
	spidev->input_dev_detect_sma->id.bustype = BUS_SPI;
	spidev->input_dev_detect_sma->dev.parent = &spidev->spi->dev;
	
#if 0
	input_set_drvdata(spidev->input_dev_detect_sma, spidev);
	set_bit(EV_KEY, spidev->input_dev_detect->evbit);
	set_bit(KEY_POWER, spidev->input_dev_detect->keybit);
#endif
	
	set_bit(EV_ABS, spidev->input_dev_detect_sma->evbit);

	input_set_abs_params(spidev->input_dev_detect_sma, ABS_MISC, 1, 0, 0, 0);

	err = input_register_device(spidev->input_dev_detect_sma);
	if (err) {
    		LATTICE_SPIDEV_ERR("input register device failed :%s\n",spidev->input_dev_detect->name);
		return err;
	}

	return 0;
}

static void spidev_input_cleanup_sma(struct spidev_data *spidev)
{
	input_unregister_device(spidev->input_dev_detect_sma);
	input_free_device(spidev->input_dev_detect_sma);
}
#endif
/*----------------------------------------------------------------------------------*/


/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spidev_sync(struct spidev_data *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidev->spi, message);
	spin_unlock_irq(&spidev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
spidev_sync_write(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spidev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

static inline ssize_t
spidev_sync_read(struct spidev_data *spidev, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= spidev->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spidev, &m);
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;

	
	LATTICE_SPIDEV_LOG("%s\n",__func__);
	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	status = spidev_sync_read(spidev, count);
	if (status > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spidev->buffer, status);
		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	}
	mutex_unlock(&spidev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct spidev_data	*spidev;
	ssize_t			status = 0;
	unsigned long		missing;
	LATTICE_SPIDEV_LOG("%s\n",__func__);
	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;

	mutex_lock(&spidev->buf_lock);
	missing = copy_from_user(spidev->buffer, buf, count);
	if (missing == 0) {
		status = spidev_sync_write(spidev, count);
	} else
		status = -EFAULT;
	mutex_unlock(&spidev->buf_lock);

	return status;
}

static int spidev_message(struct spidev_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*buf, *bufrx;
	int			status = -EFAULT;

	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = spidev->buffer;
	bufrx = spidev->bufferrx;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = bufrx;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}
		buf += k_tmp->len;
		bufrx += k_tmp->len;

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spidev->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = spidev_sync(spidev, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = spidev->bufferrx;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
		}
		buf += u_tmp->len;
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct spidev_data	*spidev;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;
printk(KERN_ERR"####   %s cmd=0x%x, arg=0x%x\n", __func__, cmd, (int)arg);

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;

	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}
printk(KERN_ERR"####   %s n_ioc=%d\n", __func__, n_ioc);
		/* translate to spi_message, execute */
		retval = spidev_message(spidev, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!spidev->buffer) {
			spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (!spidev->bufferrx) {
			spidev->bufferrx = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->bufferrx) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				kfree(spidev->buffer);
				spidev->buffer = NULL;
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			spidev->users++;
			filp->private_data = spidev;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = 0;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int		dofree;

		kfree(spidev->buffer);
		spidev->buffer = NULL;
		kfree(spidev->bufferrx);
		spidev->bufferrx = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
};

static ssize_t spi_store_debug_en(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
	unsigned long val;

    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if(val){
    		LATTICE_SPIDEV_LOG("debug enable\n");
		debug_en=1;
	}else{
    		LATTICE_SPIDEV_LOG("debug disable\n");
		debug_en=0;
	}
	return size;
}

static ssize_t spi_show_debug_en(struct device *dev, struct device_attribute *attr,
                char *buf)
{
	int val;
    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	val = debug_en;
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

#if SMA_FEATURE_EN
static ssize_t spi_store_sma_detect_en(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t size)
{
	unsigned long val;
	int i=0;
	char txbuf[6]={0};
	int tx_len=0;
	int rx_len=2;
	char reset_reg=0x00;
	char func_en_reg=0x00;
	char rxbuf[6]={0};	
    	
	struct spi_device *spi= to_spi_device(dev);
	
	LATTICE_SPIDEV_LOG("%s\n",__func__);
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

		txbuf[i++]=SPI_READ; 
		txbuf[i++]=H_BYTE(RST_REG_ADR); 
		txbuf[i++]=L_BYTE(RST_REG_ADR); 
		tx_len = i;
		if(debug_en == 0){
			spi_data_rx(spi,txbuf,rxbuf,tx_len,rx_len);
		reset_reg  = rxbuf[4];
		func_en_reg  = rxbuf[5];
		}

	if(val){
    		LATTICE_SPIDEV_LOG("smart alert enable\n");
		sma_detect_en=1;
		if(sleep_status == 1){
			i=0;
			msleep(500);
			txbuf[i++]=SPI_WRITE; 
			txbuf[i++]=H_BYTE(FUNC_EN_REG_ADR); 
			txbuf[i++]=L_BYTE(FUNC_EN_REG_ADR); 
	
			//Enable Smart Alert detect
			func_en_reg = func_en_reg | SMA_DETECT_EN;
			txbuf[i++]= func_en_reg;    // Smart Alert detect 
			tx_len = i;
		
			spi_data_tx(spi,txbuf,tx_len);


			//Set Reset register	
			i=0;
			msleep(500);
			txbuf[i++]=SPI_WRITE; 
			txbuf[i++]=H_BYTE(RST_REG_ADR); 
			txbuf[i++]=L_BYTE(RST_REG_ADR); 
			//Reset smart alert detect
			reset_reg = reset_reg | SMA_DETECT_RST;
			txbuf[i++]= reset_reg;    // reset sma detect 
			tx_len = i;
			spi_data_tx(spi,txbuf,tx_len);

			// Clear Reset register
			i=0;
			msleep(500);
			txbuf[i++]=SPI_WRITE; 
			txbuf[i++]=H_BYTE(RST_REG_ADR); 
			txbuf[i++]=L_BYTE(RST_REG_ADR); 
			reset_reg = reset_reg & (~SMA_DETECT_EN);
			txbuf[i++]= reset_reg;     
			tx_len = i;
			spi_data_tx(spi,txbuf,tx_len);
		}
			
	}else{
    		LATTICE_SPIDEV_LOG("smart alert disable\n");
		sma_detect_en=0;
		i=0;
		txbuf[i++]=SPI_WRITE; 
		txbuf[i++]=H_BYTE(FUNC_EN_REG_ADR); 
		txbuf[i++]=L_BYTE(FUNC_EN_REG_ADR); 
	
		//Disable Smart Alert detect
		func_en_reg = func_en_reg & (~SMA_DETECT_EN);
		txbuf[i++]= func_en_reg;    // Smart Alert detect 
		tx_len = i;		
		spi_data_tx(spi,txbuf,tx_len);
	}
	return size;
}

static ssize_t spi_show_sma_detect_en(struct device *dev, struct device_attribute *attr,
                char *buf)
{
	int val;
    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	val = sma_detect_en;
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static struct device_attribute attributes[] = {
#if SMA_FEATURE_EN
	__ATTR(sma_detect_en, 0660 , spi_show_sma_detect_en, spi_store_sma_detect_en),
#endif
	__ATTR(debug_sma_en, 0660 , spi_show_debug_en, spi_store_debug_en),
};
static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;
    	LATTICE_SPIDEV_LOG("%s\n",__func__);
	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		err = device_create_file(dev, attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}
#endif
/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

/*-------------------------------------------------------------------------*/

static int  spidev_probe(struct spi_device *spi)
{
	struct spidev_data	*spidev;
	int			status;
	unsigned long		minor;

	/* Allocate driver data */
	printk(KERN_INFO "spidev register is probedi\n");
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				    spidev, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, spidev);
	else
		kfree(spidev);

	status = enable_spi_interrupt(spidev->spi);     /*register interrupt handler */
	if(status < 0){
    		LATTICE_SPIDEV_ERR("Interrupt Enable failed \n");
	}
	//don`t use tx interrupt
	//INIT_DELAYED_WORK(&input_work_tx, spi_work_func_tx); /*schedule function for rx & tx */
	//INIT_DELAYED_WORK(&input_work_rx, spi_work_func_rx);
	INIT_DELAYED_WORK(&cdone_work_firmwaredownload, spi_work_func_cdone_firmware);
	//enable_irq(gpio_to_irq(GPIO_CDONE)); 
	
	/* create entry in input device */
	status = spidev_input_init(spidev);   
	if (status < 0) {
    		LATTICE_SPIDEV_ERR("spidev input init failed \n");
		return status;
	}
#if SMA_FEATURE_EN	
	/* create entry in input device */
	status = spidev_input_init_sma(spidev);   
	if (status < 0) {
    		LATTICE_SPIDEV_ERR("spidev input init failed \n");
		return status;
	}
#endif
#if defined(CONFIG_FB)
	spidev->fb_notif.notifier_call = fb_notifier_callback;
	status = fb_register_client(&spidev->fb_notif);
	if (status){
    		LATTICE_SPIDEV_ERR("fb register client failed \n");
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	spidev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						SUSPEND_LEVEL;
	spidev->early_suspend.suspend = spi_early_suspend;
	spidev->early_suspend.resume = spi_late_resume;
	register_early_suspend(&spidev->early_suspend);
#endif
#if PEDOMETER_FEATURE_EN
	status = ice_step_register(spidev);
	if(status < 0){
    		LATTICE_SPIDEV_ERR("spidev input init failed \n");
		return status;
	}

#endif
#if SHK_SMA_FEATURE_EN	
	status = create_sysfs_interfaces(&spi->dev);
	if (status < 0) {
    		LATTICE_SPIDEV_ERR("Failed to create sysfs entries\n");
		return status;
	}
#endif
     	LATTICE_SPIDEV_LOG("spidev driver probed = %s\n",__func__);
	return status;
}

static int  spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);

	disable_spi_interrupt(spi);  		/* free the irq & gpio. */
#if defined(CONFIG_FB)
	if (fb_unregister_client(&spidev->fb_notif))
		dev_err(&spi->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&spidev->early_suspend);
#endif

	spidev_input_cleanup(spidev);          /* remove event entry from /dev/input */
#if SMA_FEATURE_EN
	spidev_input_cleanup_sma(spidev);          /* remove event entry from /dev/input */
#endif

#if PEDOMETER_FEATURE_EN
	ice_step_unregister(spidev);
#endif
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);
	return 0;
}

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"spidev",
		.owner =	THIS_MODULE,
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------
		add by tony for lattice gpio init
-------------------------------------------------------------------------*/
static int lattice_gpio_init(void)
{
	int err = -1;

	//GPIO120
        err = gpio_request(GPIO_VCC_1_2, "VCC_1_2");
		if (err)
		{
			LATTICE_SPIDEV_ERR("%s: vcc_1_2 gpio request error\n",__func__);
			return err;
		}
		err = gpio_direction_output(GPIO_VCC_1_2, 0);
		if (err) 
		{
			LATTICE_SPIDEV_ERR("%s: vcc_1_2 gpio direction error\n",__func__);
			return err;
		}
		//GPIO00
err = gpio_request(GPIO_VCC_1_8, "VCC_1_8");
	if (err)
	{
		LATTICE_SPIDEV_ERR("%s: vcc_1_8 gpio request error\n",__func__);
		return err;
	}
	err = gpio_direction_output(GPIO_VCC_1_8, 0);
    if (err) 
	{
		LATTICE_SPIDEV_ERR("%s: vcc_1_8 gpio direction error\n",__func__);
		return err;
	}
		
	//GPIO121
	err = gpio_request(GPIO_VCC_3_3, "VCC_3_3");
	if (err)
	{
		LATTICE_SPIDEV_ERR("%s: vcc_3_3 gpio request error\n",__func__);
		return err;
	}
	err = gpio_direction_output(GPIO_VCC_3_3, 0);
    if (err) 
	{
		LATTICE_SPIDEV_ERR("%s: vcc_3_3 gpio direction error\n",__func__);
		return err;
	}
	
	//GPIO02
	err = gpio_request(GPIO_VCC_IO2, "VCC_IO2");
	if (err)
	{
		LATTICE_SPIDEV_ERR("%s: vcc_io2 gpio request error\n",__func__);
		return err;
	}
	err = gpio_direction_output(GPIO_VCC_IO2, 0);
    if (err) 
	{
		LATTICE_SPIDEV_ERR("%s: vcc_io2 gpio direction error\n",__func__);
		return err;
	}
	
	//GPIO03
	err = gpio_request(GPIO_CDONE, "CDONE");
	if (err)
	{
		LATTICE_SPIDEV_ERR("%s: cdone gpio request error\n",__func__);
		return err;
	}
	err = gpio_direction_input(GPIO_CDONE);
    if (err) 
	{
		LATTICE_SPIDEV_ERR("%s: cdone gpio direction error\n",__func__);
		return err;
	}
	
	err = gpio_request(GPIO_SPI_CS, "SPI_CS");
  if (err) {
    printk("gpio_request SPI_CS error\n");
    return err;
  }

  err = gpio_direction_output(GPIO_SPI_CS, 0);
  if (err) {
    printk("SPI_CS Unable to set direction\n");
    return err;
  }
  ir_spi_cs(0);
	
	return err;
}
/*-------------------------------------------------------------------------*/
static int tx_test(void)
{
	int retval =-1;
	u8	save=0; 
	u32 speed_save=0;
	
	u8 tx_len_buf[5]={0x02, 0x00, 0x02, 0x88, 0x00};
	u8 carrier_divider_buf[5]={0x02, 0x00, 0x06, 0x38, 0x01};
	u8 tx_enable_buf[4]={0x02, 0x00, 0x00, 0x01};
	u8 tx_disable_buf[4]={0x02, 0x00, 0x00, 0x00};
	u8 clk_en_buf[4]={0x02, 0x10, 0x01, 0x01};
	//tx disable
	spi_data_tx(spi,tx_disable_buf,4);
	//set wr mode
	save = spi->mode; 
	spi->mode = (u8)(spi->mode & ~SPI_MODE_MASK);
	retval = spi_setup(spi);
	if (retval < 0)
		spi->mode = save;
	
	//set lsb mode
	save = spi->mode; 
	spi->mode &= ~SPI_LSB_FIRST;
	retval = spi_setup(spi);
	if (retval < 0)
		spi->mode = save;
	//set bit per word
	save = spi->bits_per_word;
	spi->bits_per_word = 8;
	retval = spi_setup(spi);
	if (retval < 0)
		spi->bits_per_word = save;
	//set speed
	speed_save = spi->max_speed_hz;
	spi->max_speed_hz = 1000000;
	retval = spi_setup(spi);
	if (retval < 0)
		spi->max_speed_hz = save;
	//set tx len
	spi_data_tx(spi,tx_len_buf,5);
	//set carrier divider
	spi_data_tx(spi,carrier_divider_buf,5);
	//set clk enable
	spi_data_tx(spi,clk_en_buf,4);
	//write data to buf
	spi_data_tx(spi,txbuf_test,139);
	//tx enable
	spi_data_tx(spi,tx_enable_buf,4);
	mdelay(10);
	//tx disable
	spi_data_tx(spi,tx_disable_buf,4);
	return 0;
}
/*-------------------------------------------------------------------------*/
static int rx_test(void)
{
	int retval =-1;
	u8	save=0; 
	u32 speed_save=0;
	u8 clk_en_buf[4]={0x02, 0x10, 0x01, 0x01};
	u8 rx_enable_buf[4]={0x02, 0x00, 0x00, 0x02};
	u8 rx_disable_buf[4]={0x02, 0x00, 0x00, 0x00};
	u8 status=-1;
	u32 read_len=0;
	int i=0;
	int average_deviation =0;
	unsigned long before=0;
	printk("in rx_test\n");
	//set rd mode
	save = spi->mode; 
	spi->mode = (u8)(spi->mode & ~SPI_MODE_MASK);
	retval = spi_setup(spi);
	if (retval < 0)
		spi->mode = save;
	
	//set lsb mode
	save = spi->mode; 
	spi->mode &= ~SPI_LSB_FIRST;
	retval = spi_setup(spi);
	if (retval < 0)
		spi->mode = save;
	//set bit per word
	save = spi->bits_per_word;
	spi->bits_per_word = 8;
	retval = spi_setup(spi);
	if (retval < 0)
		spi->bits_per_word = save;
	//set speed
	speed_save = spi->max_speed_hz;
	spi->max_speed_hz = 1000000;
	retval = spi_setup(spi);
	if (retval < 0)
		spi->max_speed_hz = save;

	//set clk enable
	spi_data_tx(spi,clk_en_buf,4);
	//set slef learning enable
	spi_data_tx(spi,rx_enable_buf,4);
	before = jiffies;
	while(status!=1)
	{
		status=spi_func_rx(0x0001);
		if(time_after(jiffies,before+HZ*10))
			return TEST_FAIL;
	}
	read_len=((spi_func_rx(0x0005)<<8)&0xff00)|(spi_func_rx(0x0004)&0xff);
	if(read_len != 136)
	{
		//set slef learning disable
		spi_data_tx(spi,rx_disable_buf,4);
		return TEST_FAIL;
	}
	rxbuf_test[0]=0x03;
	rxbuf_test[1]=0x08;
	rxbuf_test[2]=0x00;
	spi_data_rx(spi,rxbuf_test,rxbuf_test,3,read_len);
	//set slef learning disable
	spi_data_tx(spi,rx_disable_buf,4);
	printk("rx data:len=%d\n",read_len);
	for(i=0;i<read_len;i++)
	{
		printk(" %d",rxbuf_test[i+4]);
	}
	for(i=0;i<read_len-2;i++)
	{
		if(rxbuf_test[i+4] > txbuf_test[i+3])
			average_deviation+=rxbuf_test[i+4]-txbuf_test[i+3];
		else
			average_deviation+=txbuf_test[i+3]-rxbuf_test[i+4];
	}
	printk("\n");
	average_deviation=average_deviation/(read_len-2);
	printk("rx data:average_deviation=%d\n",average_deviation);
	return average_deviation;
}
/*-------------------------------------------------------------------------*/

static int ir_recv_proc(struct seq_file *seq, void *offset)
{
  int ret = 0;
  printk(KERN_INFO"ir_recv_proc in\n");
  if (NULL == seq)	
  {
    return 0;
  }
  //ir_config_ice40(1,fw_path);
  
  ret = seq_printf(seq, "%s\n", ir_status);	

  return ret;
}

static int ir_open_proc(struct inode *inode, struct file *file)
{
	printk(KERN_INFO"ir_open_proc in\n");
	return single_open(file, ir_recv_proc, NULL);		
}

void spi_func_tx(unsigned short addr, unsigned char data)
{
	char txbuf[4]={0x02,(addr>>8)&0xff,addr&0xff,data};
	int len = sizeof(txbuf);
	printk("spi_func_tx addr=0x%04x, data=0x%02x\n",addr,data);
	spi_data_tx(spi,txbuf,len);
	return;
}
unsigned char spi_func_rx(unsigned short addr)
{
	char txbuf[5]={0x03,(addr>>8)&0xff,addr&0xff};
	int txlen = 3;
	int rxlen = 1;
	spi_data_rx(spi,txbuf,txbuf,txlen,rxlen);
	printk("spi_func_rx read register addr=0x%04x register=0x%02x\n",addr,txbuf[4]);
	return txbuf[4];
}

//this will run in init.qcom.rc when phone power on
static ssize_t ir_contrl_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{

  unsigned char buf[10]={0};//zhangjian modify 9->10
  unsigned short addr=0;
  unsigned char data=0;
  char  c;
  int tmp1,tmp2;
  char keyword[5]={0};
  unsigned char readdata;
  keyword[4]='\0';
  printk("ir_contrl_proc in\n");
  if (copy_from_user(buf, buff, 9))
  {
	    printk("%s, copy_from_user error\n", __func__);
    	return -EFAULT;
   }
  if (buf[0]=='F' && buf[1]=='M')
  { //download firmware in init.qcom.rc 
  	//ir_enable_init();//power on
	ir_config_ice40(1,fw_path);
	return len;
  }
  else if(buf[0]=='w')
  {
	  sscanf(buf, "%c %x %x",&c,&tmp1,&tmp2);
	  printk("[tony] cmd=%c address=%04x data=%02x \n",c ,tmp1,tmp2);
	  addr=tmp1 & 0xffff;
	  data=tmp2 & 0xff;
	  spi_func_tx(addr,data);
	  return len;
  }
  else if(buf[0]=='r')
  {
	  if(buf[2]=='a' && buf[3]=='c')
	  {
		  
		  int txlen = 3;
		  int rxlen = 1024;   
		  int i=0;
		  addr=0x2400;
		  acc_txbuf[0]=0x03;
		  acc_txbuf[1]=(addr>>8)&0xff;
		  acc_txbuf[2]=addr&0xff;
		  spi_data_rx(spi,acc_txbuf,acc_txbuf,txlen,rxlen);
		  printk("tony acc data:");
		  for(i=0;i<1024;i++)
		   printk("%02x ",acc_txbuf[i+4]);
		 printk("\n");
			return len;
	  }
	  else
	  {
		sscanf(buf, "%c %x %x",&c,&tmp1,&tmp2);
		printk("[tony] cmd=%c address=%04x data=%02x \n",c ,tmp1,tmp2);
		addr=tmp1 & 0xffff;
		readdata=spi_func_rx(addr);
		printk("tony****** readdata=0x%02x",readdata);
		return len;

	  }	  	 
  }
  else if(buf[0]=='t')//for test
  {
	  sscanf(buf, "%c %c%c%c%c %x",&c,&keyword[0],&keyword[1],&keyword[2],&keyword[3],&tmp2);
	  printk("[tony] cmd=%c keyword is %s data=%02x \n",c ,keyword,tmp2);
	  if(strncmp("chip",keyword,5)==0)
	  {
        if (tmp2 != 0x2) //zhangjian add 
            tmp2 =0x2;
		spi_func_tx(0x100A,(tmp2 & 0xff));
		if(spi_func_rx(0x100A)==(tmp2 & 0xff))
		{
			strncpy(ir_status, "chip test ok", 13);
			return TEST_SUCCESS;
		}
		else
		{
			strncpy(ir_status, "chip test fail", 15);
			return TEST_FAIL;
		}
	  }
	  else if(strncmp("Txon",keyword,5)==0)
	  {
		printk("Tx on\n");
		tx_test();
		//strncpy(ir_status, "cmd error", 11);
		//return TEST_CMD_ERR;
		return len;
	  }
	    else if(strncmp("Rxon",keyword,5)==0)
	  {
		printk("Rx on\n");
		return rx_test();
	  }
    else if(strncmp("fwvs",keyword,5)==0)
    {
        printk("FW version read\n");
        return spi_func_rx(0x1005);
    }
	  else
	  {
		printk("tony:cmd error\n");
		strncpy(ir_status, "cmd error", 11);
		return TEST_CMD_ERR;
	  }
  }
  else
  {
	printk("tony: cmd error\n");
	return len;
  }

 }
 /*
 static ssize_t ir_read_proc(struct file *filp, char *buff, size_t len, loff_t *off)
{
	int buf_size = 0;
	ir_status[15] = '\0'; 
	buf_size = strlen(ir_status)+1;
	printk("in ir_read_proc len=%d\n",buf_size);
	printk("in ir_read_proc ir_status=%s\n",ir_status);
	if(copy_to_user(buff, ir_status,buf_size))
	{
		printk("%s, copy_to_user error\n", __func__);
    	return -EFAULT;
	}
	return buf_size;
}
*/ 
static const struct file_operations ir_proc_fops = {
	.owner		= THIS_MODULE,
	.open       = ir_open_proc,
	.read       = seq_read,
	//.read       = ir_read_proc,
	.write      = ir_contrl_proc,
};

static void create_ir_proc_file(void)
{
  struct proc_dir_entry *ir_proc_file = proc_create("driver/irda", 0777, NULL,&ir_proc_fops);
  if (ir_proc_file == NULL) 
  {
    printk(KERN_INFO "proc file create failed!\n");
  } 
  
}
//power on
static void ir_enable_init(void)
{
  
  gpio_set_value(GPIO_VCC_1_2, 1);
  mdelay(10);
  gpio_set_value(GPIO_VCC_1_8, 1);
  mdelay(10);
  gpio_set_value(GPIO_VCC_3_3, 1);
  mdelay(10);
  return;
}
/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;
	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
    LATTICE_SPIDEV_LOG("%s\n",__func__);


	//add by tony for gpio init	
	if(lattice_gpio_init())
	{
		LATTICE_SPIDEV_ERR("%s: gpio init err\n",__func__);
	}
	
	
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (status < 0){
		printk(KERN_INFO "register char dev failed \n");
		return status;}

	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
		printk(KERN_INFO "register class create failed \n");
		status = PTR_ERR(spidev_class);
		goto error_class;
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0){
		printk(KERN_INFO "Register failed\n");
		goto error_register;
		}

	if (busnum != -1 && chipselect != -1) {
		struct spi_board_info chip = {
					.modalias	= "spidev",
					.mode		= spimode,
					.bus_num	= busnum,
					.chip_select	= chipselect,
					.max_speed_hz	= maxspeed,
		};
		
		struct spi_master *master;
		printk(KERN_INFO "Tony busnum = %d\n",busnum);
		master = spi_busnum_to_master(busnum);
		if (!master) {
		printk(KERN_INFO "spi_busnum failed\n");
			status = -ENODEV;
			goto error_busnum;
		}

		/* We create a virtual device that will sit on the bus */
		spi = spi_new_device(master, &chip);
		if (!spi) {
		printk(KERN_INFO "spi_new_device failed\n");
			status = -EBUSY;
			goto error_mem;
		}
		dev_dbg(&spi->dev, "busnum=%d cs=%d bufsiz=%d maxspeed=%d",
			busnum, chipselect, bufsiz, maxspeed);

		dev_err(&spi->dev, "busnum=%d cs=%d bufsiz=%d maxspeed=%d",
			busnum, chipselect, bufsiz, maxspeed);
	}
	create_ir_proc_file();
	ir_enable_init();
	printk(KERN_INFO "spi driver init success \n");
	return 0;
error_mem:
error_busnum:
	spi_unregister_driver(&spidev_spi_driver);
error_register:
	class_destroy(spidev_class);
error_class:
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	if (spi) {
		spi_unregister_device(spi);
		spi = NULL;
	}
     	LATTICE_SPIDEV_LOG("%s\n",__func__);
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(SPIDEV_MAJOR, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface. Lattice iCE SB interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
