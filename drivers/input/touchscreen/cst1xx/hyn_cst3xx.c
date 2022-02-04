/* 
 * drivers/input/touchscreen/CST3XX.c
 *
 * hynitron TouchScreen driver. 
 *
 * Copyright (c) 2015  hynitron
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
 * VERSION      	DATE			AUTHOR
 *  1.0		    2015-10-12		    Tim
 *
 * note: only support mulititouch
 */
  
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
//#include <linux/rtpm_prio.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <asm/uaccess.h>
//#include "mt_boot_common.h"



#include "hyn_cst3xx_RS659_fw.h"


#define HYN_DRIVER_VERSION                  "Hynitron_MTK_CST3XX_Driver_V1.2_20190717"

/*************************************************
*************************************************
*************************************************
config macro
*************************************************
*************************************************
*************************************************/

#define ANDROID_TOOL_SURPORT
//#define HYN_SYSFS_NODE_EN


//#define TPD_HAVE_BUTTON        			 //report key position
//#define GTP_HAVE_TOUCH_KEY    			 //report keyevent value
//#define HYN_UPDATE_FIRMWARE_ENABLE          //update firmware online enable 
//#define HYN_UPDATE_FIRMWARE_FORCE          //update firmware online force 
//#define HYN_UPDATE_POWERON_ENABLE          // power on to enter bootloader

//#define CST2XX_SUPPORT
//#define ICS_SLOT_REPORT
#define SLEEP_CLEAR_POINT
//#define HIGH_SPEED_IIC_TRANSFER
#define TRANSACTION_LENGTH_LIMITED
#define HYN_RESET_HARDWARE  				//reset function with reset pin or software
#define GTP_DEBUG_FUNC_ON

#ifdef	GTP_DEBUG_FUNC_ON
#define HYN_DBG(fmt, arg...)  printk("[HYN]:[LINE=%d] "fmt,__LINE__, ##arg)
#define HYN_INF(fmt, arg...)  printk("[HYN]:[LINE=%d] "fmt,__LINE__, ##arg)
#define HYN_ERR(fmt, arg...)  printk("[HYN]:[LINE=%d] "fmt,__LINE__, ##arg)
#else
#define HYN_DBG(fmt, arg...)  
#define HYN_INF(fmt, arg...) 
#define HYN_ERR(fmt, arg...)  printk("[HYN]:[LINE=%d] "fmt,__LINE__, ##arg)
#endif

#define TPD_DEVICE_NAME			    "Hyn_device" //TPD_DEVICE
#define TPD_DRIVER_NAME			    "Hyn_driver"
#define TPD_MAX_FINGERS			    5
#define TPD_MAX_X				    720
#define TPD_MAX_Y				    1440

#define TPD_REVERT_XY				0
#define TPD_REVERT_X				0
#define TPD_REVERT_Y			    0



#define CST3XX_I2C_ADDR				0x1A
#define CST3XX_I2C_BOOT_ADDR  	    0x1A
#define I2C_BUS_NUMBER               1 //IIC bus num for mtk

//#define GTP_RST_PORT    0
//#define GTP_INT_PORT    1

#ifdef CONFIG_OF
int cst_rst_gpio;
int cstx_int_gpio;
#endif


#ifdef CONFIG_OF

#define GTP_RST_PORT cst_rst_gpio
#define GTP_INT_PORT cstx_int_gpio
#else
#define GTP_RST_PORT    102
#define GTP_INT_PORT    52
#endif


#define GTP_INT_IRQ     gpio_to_irq(GTP_INT_PORT)
/*#define GTP_INT_CFG     S3C_GPIO_SFN(0xF)*/

#define GTP_GPIO_AS_INPUT(pin)          do {\
	gpio_direction_input(pin);\
} while (0)
#define GTP_GPIO_AS_INT(pin)            do {\
	GTP_GPIO_AS_INPUT(pin);\
} while (0)
#define GTP_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin, level)      gpio_direction_output(pin, level)
#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}





/*************************************************
***********************************************
***********************************************
static functions and variables declaration

***********************************************
***********************************************
*************************************************/

static int hyn_boot_update_fw(struct i2c_client * client);
static int cst3xx_firmware_info(struct i2c_client * client);
static int cst3xx_update_firmware(struct i2c_client * client, const unsigned char *pdata);
#ifdef CST2XX_SUPPORT
static int cst2xx_update_firmware(struct i2c_client * client,unsigned char *pdata);
#endif



static struct input_dev *input_dev;




/*************************************************
Gesture Function Code
*************************************************/
#ifdef HYN_GESTURE
static int hyn_lcd_flag = 0;
static int tpd_halt=0;
//static struct input_dev *hyn_power_idev;
//static char hyn_gesture_c = 0;
//static int hyn_gesture_flag = 0;

#define  HYN_TPD_GES_COUNT  14


#define  TPD_GES_KEY_DOUBLECLICK  26
#define  TPD_GES_KEY_LEFT		  4
#define  TPD_GES_KEY_RIGHT		  4
#define  TPD_GES_KEY_UP			  4
#define  TPD_GES_KEY_DOWN		  4
#define  TPD_GES_KEY_O			  4
#define  TPD_GES_KEY_W			  4
#define  TPD_GES_KEY_M			  4
#define  TPD_GES_KEY_E			  4
#define  TPD_GES_KEY_C			  4
#define  TPD_GES_KEY_S			  4
#define  TPD_GES_KEY_V  		  4
#define  TPD_GES_KEY_Z			  4


static int tpd_keys_gesture[HYN_TPD_GES_COUNT] ={KEY_POWER, \
												TPD_GES_KEY_DOUBLECLICK,\
												TPD_GES_KEY_LEFT,\
												TPD_GES_KEY_RIGHT,\
												TPD_GES_KEY_UP,\
												TPD_GES_KEY_DOWN,\
												TPD_GES_KEY_O,\
												TPD_GES_KEY_W,  \
												TPD_GES_KEY_M,\
												TPD_GES_KEY_E,\
												TPD_GES_KEY_C,\
												TPD_GES_KEY_S,\
												TPD_GES_KEY_V,\
												TPD_GES_KEY_Z};

#endif 




/*************************************************
ANDROID DEBUG Function Code
*************************************************/
#define	CST3XX_INT_PORT         5
#ifdef ANDROID_TOOL_SURPORT
static  unsigned short g_unnormal_mode = 0;
static unsigned short g_cst3xx_tx = 10;
static unsigned short g_cst3xx_rx = 20;
#ifdef HYN_SYSFS_NODE_EN 
static struct mutex g_device_mutex;
static DEFINE_MUTEX(g_device_mutex);
static struct kobject *k_obj = NULL;
#endif

#endif



#pragma pack(1)
typedef struct
{
    u16 pid;                 //product id   //
    u16 vid;                 //version id   //
} st_tpd_info;
#pragma pack()

st_tpd_info tpd_info;

static unsigned char  report_flag = 0;
static unsigned int   g_cst3xx_ic_version  = 0;
static unsigned int   g_cst3xx_ic_checksum = 0;
static unsigned int   g_cst3xx_ic_checkcode =0;
static unsigned int   g_cst3xx_ic_project_id  = 0;
static unsigned int   g_cst3xx_ic_type  = 0;
static unsigned int   touch_irq = 0;


struct hynitron_fw_array {
	const char* name;
	const int project_id;
    unsigned char *fw;
} cst3xx_fw_grp[] = {
    { "hyn_cst3xx_RS659_fw",      0x2843,  cst3_fw },	// default
//	{ "hyn_cst3xx_K1336A_fw_v2",   0x0501,  cst3_fw_v2 } //ÐÂÄ£×é
    
};

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DECLARE_WAIT_QUEUE_HEAD(dbg_waiter);
static struct task_struct *thread = NULL;
static int tpd_flag = 0;


extern struct tpd_device  *tpd;
static struct i2c_client *g_i2c_client = NULL;

static int cst3xx_boot_update_fw(struct i2c_client   *client,  unsigned char *pdata);
static unsigned char *pcst3xx_update_firmware = (unsigned char *)cst3_fw ; //the updating firmware
 

#ifdef ICS_SLOT_REPORT
#include <linux/input/mt.h> // Protocol B, for input_mt_slot(), input_mt_init_slot()
#endif

/*************************************************
KEY Function Code
*************************************************/
#ifdef TPD_HAVE_BUTTON
#define TPD_KEY_COUNT	3
//#define TPD_KEYS		{KEY_MENU,KEY_BACK}
#define TPD_KEYS		{KEY_BACK, KEY_HOMEPAGE, KEY_MENU}
/* {button_center_x, button_center_y, button_width, button_height*/
#define TPD_KEYS_DIM	{{100, 1200, 60, 60},{250, 1200, 60, 60},{400, 1200, 60, 60}}
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif



#ifdef GTP_HAVE_TOUCH_KEY
#define TPD_KEY_COUNT	3
#define TPD_KEYS		{KEY_BACK, KEY_HOMEPAGE, KEY_MENU}
//#define TPD_KEYS		{KEY_MENU,KEY_BACK}
const u16 touch_key_array[] = TPD_KEYS;
//#define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif

#if defined(TPD_HAVE_BUTTON) || defined(GTP_HAVE_TOUCH_KEY)
static unsigned char  key_index = 0xFF;
#endif

/*************************************************
************************************************
************************************************
HYN IIC TRANSFER Function Code
************************************************
************************************************
*************************************************/
#ifdef HIGH_SPEED_IIC_TRANSFER
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	struct i2c_msg msg; 
	int ret = -1; 
	int retries = 0; 
	
	msg.flags |= I2C_M_RD; 
	msg.addr   = client->addr;
	msg.len    = len; 
	msg.buf    = buf;	

	while (retries < 2) { 
		ret = i2c_transfer(client->adapter, &msg, 1); 
		if(ret == 1)
			break; 
		retries++; 
	} 
	
	return ret; 
} 


/*******************************************************
Function:
    read data from register.
Input:
    buf: first two byte is register addr, then read data store into buf
    len: length of data that to read
Output:
    success: number of messages
    fail:	negative errno
*******************************************************/
static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	struct i2c_msg msgs[2]; 
	int ret = -1; 
	int retries = 0; 
	
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].addr  = client->addr;  
	msgs[0].len   = 2;
	msgs[0].buf   = buf; 

	msgs[1].flags |= I2C_M_RD;
	msgs[1].addr   = client->addr; 
	msgs[1].len    = len; 
	msgs[1].buf    = buf;

	while (retries < 2) { 
		ret = i2c_transfer(client->adapter, msgs, 2); 
		if(ret == 2)
			break; 
		retries++; 
	} 
	
	return ret; 
} 

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	struct i2c_msg msg; 
	int ret = -1; 
	int retries = 0;

	msg.flags = client->flags & I2C_M_TEN; 
	msg.addr  = client->addr; 
	msg.len   = len; 
	msg.buf   = buf;		  
	  
	while (retries < 2) { 
		ret = i2c_transfer(client->adapter, &msg, 1); 
		if(ret == 1)
			break; 
		retries++; 
	} 	
	
	return ret; 
}

#else
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
	int retries = 0; 

	while (retries < 2) { 
		ret = i2c_master_recv(client, buf, len); 
		if(ret<=0) 
		    retries++;
        else
            break; 
	} 
	
	return ret; 
} 

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
	int retries = 0; 

	while (retries < 2) { 
		ret = i2c_master_send(client, buf, len); 
		if(ret<=0) 
		    retries++;
        else
            break; 
	} 
	
	return ret; 
}

static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len) 
{ 
	int ret = -1; 
    
    ret = cst3xx_i2c_write(client, buf, 2);

    ret = cst3xx_i2c_read(client, buf, len);
	
    return ret; 
} 
#endif


static void cst3xx_reset_ic(unsigned int ms)
{   

	unsigned char buf[4];
	buf[0] = 0xD1;
	buf[1] = 0x0E;	 
	cst3xx_i2c_write(g_i2c_client, buf, 2);
	
#ifdef HYN_RESET_HARDWARE	
//	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	mdelay(5);
//	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);	
#endif
	mdelay(ms);
}


/*******************************************************
Function:
    test i2c communication
Input:
    client: i2c client
Output:

    success: big than 0
    fail:	negative
*******************************************************/
static int cst3xx_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int ret;
	unsigned char buf[4];

	buf[0] = 0xD1;
	buf[1] = 0x06;
	while (retry++ < 5) {
		ret = cst3xx_i2c_write(client, buf, 2);
		if (ret > 0)
			return ret;
		
		mdelay(2);		
	}

    if(retry==5) HYN_ERR(" cst3xx hyn I2C TEST error.ret:%d;\n", ret);
	
	return ret;
}

void hyn_key_report(int key_value)
{
#if 1
	input_report_key(input_dev, key_value, 1);
	input_sync(input_dev);	
	input_report_key(input_dev, key_value, 0);
	input_sync(input_dev);
#else
	input_report_key(input_dev, KEY_POWER, 1);
	input_sync(input_dev);
	input_report_key(input_dev, KEY_POWER, 0);
	input_sync(input_dev);
#endif
}




static void cst3xx_touch_down(struct input_dev *input_dev,s32 id,s32 x,s32 y,s32 w)
{
    s32 temp_w = (w>>1);
	
#ifdef ICS_SLOT_REPORT
    input_mt_slot(input_dev, id);
    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_PRESSURE, temp_w);
#else
    input_report_key(input_dev, BTN_TOUCH, 1);
    input_report_abs(input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(input_dev);
#endif

#if 0
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {   
		tpd_button(x, y, 1);  
	}
#endif
}

static void cst3xx_touch_up(struct input_dev *input_dev, int id)
{
#ifdef ICS_SLOT_REPORT
    input_mt_slot(input_dev, id);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_mt_sync(input_dev);
#endif

#if 0
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {   
	    tpd_button(0, 0, 0);  
	}
#endif
}

#ifdef ANDROID_TOOL_SURPORT   //debug tool support
#define CST3XX_PROC_DIR_NAME	"cst1xx_ts"
#define CST3XX_PROC_FILE_NAME	"cst1xx-update"
static struct proc_dir_entry *g_proc_dir, *g_update_file;
static int CMDIndex = 0;

static struct file *cst3xx_open_fw_file(char *path, mm_segment_t * old_fs_p)
{
	struct file * filp = NULL;
	int ret;

	*old_fs_p = get_fs();
	//set_fs(KERNEL_DS);
	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp)) 
	{   
        ret = PTR_ERR(filp);
		HYN_ERR("cst3xx_open_fw_file:filp_open error.\n");
        return NULL;
    }
    filp->f_op->llseek(filp, 0, 0);
	
    return filp;
}

static void cst3xx_close_fw_file(struct file * filp,mm_segment_t old_fs)
{
	 //set_fs(old_fs);
	if(filp)
	    filp_close(filp,NULL);
}

static int cst3xx_read_fw_file(unsigned char *filename, unsigned char *pdata, int *plen)
{
	struct file *fp;
	int ret = -1;
	loff_t pos;
	off_t fsize;
	struct inode *inode;
	unsigned long magic;
	mm_segment_t old_fs;
	
	HYN_INF("cst3xx_read_fw_file enter.\n");
	
	if((pdata == NULL) || (strlen(filename) == 0)) {
		HYN_ERR(" cst3xx file name is null.\n");
		return ret;
	}
	fp = cst3xx_open_fw_file(filename,&old_fs);
	if(fp == NULL) {		
        HYN_ERR(" cst3xx Open bin file faild.path:%s.\n", filename);
		goto clean;
	}

#if 0
	
	length = fp->f_op->llseek(fp, 0, SEEK_END); 
	fp->f_op->llseek(fp, 0, 0);	
	size = fp->f_op->read(fp, pdata, length, &fp->f_pos);
	if(size == length) 
	{
    	ret = 0;
    	*plen = length;
	} 	
	else
	{
		HYN_INF("read bin file length fail****size:%d*******length:%d .\n", size,length);

	}

#else 

	if (IS_ERR(fp)) {
		HYN_ERR("error occured while opening file %s.\n", filename);
		return -EIO;
	}
	inode = fp->f_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;		
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	ret=vfs_read(fp, pdata, fsize, &pos);
	if(ret==fsize){
		HYN_INF("vfs_read success.ret:%d.\n",ret);
	}else{
		HYN_ERR("vfs_read fail.ret:%d.\n",ret);
	}
	filp_close(fp, NULL);
	set_fs(old_fs);
	
	HYN_INF("vfs_read done.\n");

#endif

clean:
	cst3xx_close_fw_file(fp,old_fs);
	return ret;
}

static int cst3xx_apk_fw_dowmload(struct i2c_client *client,
		unsigned char *pdata, int length) 
{ 
	int ret=-1;

	HYN_INF("cst3xx_apk_fw_dowmload enter.\n");

	pcst3xx_update_firmware=pdata;

#ifdef CST2XX_SUPPORT
	if(hyn_ic_type==18868) {
	    ret = cst2xx_update_firmware(g_i2c_client, pdata);
	   return ret;
	}else if(hyn_ic_type==340) 
#endif	
	{
		ret = cst3xx_update_firmware(g_i2c_client, pdata);
		return ret;
	}

	return 0;
}
static int hyn_online_testflag = 0;
static ssize_t cst3xx_proc_read_foobar(struct file *page,char __user *user_buf, size_t count, loff_t *data)
{	
	unsigned char buf[1020];  //if need more,please define extern buf[].
	int len = 0;	
	int ret;

    HYN_INF("  cst3xx is entering cst3xx_proc_read_foobar. CMDIndex = %d\n", CMDIndex);
  
	if (CMDIndex == 0) {
		//sprintf(buf,"Hynitron touchscreen driver 1.0\n");
		//strcpy(page,buf);	
		//len = strlen(buf);
		//ret = copy_to_user(user_buf,buf,len);
        
        
		buf[0] = 0xD1;
		buf[1] = 0x01;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if (ret < 0) return -1;
		
		mdelay(10);

		buf[0] = 0xD1;
		buf[1] = 0xF4;//E8
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 28);//4   //4
		if (ret < 0) return -1;	

		g_cst3xx_tx=buf[0];
		g_cst3xx_rx=buf[2];

		sprintf(buf, "  cst3xx_proc_read_foobar:g_cst3xx_tx:%d,g_cst3xx_rx:%d.\n",g_cst3xx_tx,g_cst3xx_rx);
        len = strlen(buf);
		ret = copy_to_user(user_buf,buf,len);
	}
	else if (CMDIndex == 1) {

		buf[0] = 0xD1;
		buf[1] = 0x01;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if (ret < 0) return -1;
		
		mdelay(10);

		buf[0] = 0xD1;
		buf[1] = 0xF4;//E8
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 28);//4   //4
		if (ret < 0) return -1;	

		g_cst3xx_tx=buf[0];
		g_cst3xx_rx=buf[2];

		HYN_INF("  cst3xx_proc_read_foobar:g_cst3xx_tx:%d,g_cst3xx_rx:%d.\n",g_cst3xx_tx,g_cst3xx_rx);
		
		//buf[0] = g_cst3xx_rx;
		//buf[1] = g_cst3xx_tx;
		len = 28;
		ret = copy_to_user(user_buf,buf,len);//4

		buf[0] = 0xD1;
		buf[1] = 0x09;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		
	}
	else if(CMDIndex == 2 || CMDIndex == 3||CMDIndex == 4) {		
		unsigned short rx,tx;
		int data_len;
		
		rx = g_cst3xx_rx;
		tx = g_cst3xx_tx;
		data_len = rx*tx*2 + 4 + (tx+rx)*2 + rx + rx;
		
		tpd_flag = 0;
		if(CMDIndex == 2)  //read diff
		{
			buf[0] = 0xD1;
			buf[1] = 0x0D;
		}
		else  if(CMDIndex == 3)        //rawdata
		{  
			buf[0] = 0xD1;
			buf[1] = 0x0A;
		}
		else if(CMDIndex == 4)          //factory test
		{  
			buf[0] = 0xD1;
			buf[1] = 0x19;
			data_len = rx*tx*4 +(4 + tx + rx)*2;
		}
		hyn_online_testflag = 1;		
		ret = i2c_master_send(g_i2c_client, buf, 2);  
		if(ret < 0) {			
			HYN_ERR(" cst3xx Write command raw/diff mode failed.error:%d.\n", ret);
			goto END;
		}

		g_unnormal_mode = 1;
		mdelay(14);			
		wait_event(dbg_waiter, tpd_flag!=0);
		tpd_flag = 0;
		
		HYN_INF(" cst3xx Read wait_event interrupt");
		
		if(CMDIndex == 4)
		{
			buf[0] = 0x12;
			buf[1] = 0x15;
		}
		else
		{
			buf[0] = 0x80;
			buf[1] = 0x01;
		}
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if(ret < 0) {				
			HYN_ERR(" cst3xx Write command(0x8001) failed.error:%d.\n", ret);
			goto END;
		}		
		ret = cst3xx_i2c_read(g_i2c_client, &buf[2], data_len);
		if(ret < 0) {
			HYN_ERR(" cst3xx Read raw/diff data failed.error:%d.\n", ret);
			goto END;
		}	

		mdelay(1);
		
		buf[0] = 0xD1;
		buf[1] = 0x09;		
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2); 		
		if(ret < 0) {				
			HYN_ERR(" cst3xx Write command normal mode failed.error:%d.\n", ret);
			goto END;
		}	
		
		buf[0] = rx;
		buf[1] = tx;	
		ret = copy_to_user(user_buf,buf,data_len + 2);
    	len = data_len + 2;
	}	

END:	
	hyn_online_testflag = 0;
	g_unnormal_mode = 0;
	CMDIndex = 0;	
	return len;
}

static ssize_t cst3xx_proc_write_foobar(struct file *file, const char __user *buffer,size_t count, loff_t *data)
{
    unsigned char cmd[128];
    unsigned char *pdata = NULL;
	int len;
	int ret;
    int length = 24*1024;

	if (count > 128) 
		len = 128;
	else 
		len = count;

   HYN_INF("  cst3xx is entering cst3xx_proc_write_foobar .\n");
    
	if (copy_from_user(cmd, buffer, len))  {
		HYN_ERR(" cst3xx copy data from user space failed.\n");
		return -EFAULT;
	}
	
	 HYN_INF("  cmd:%d......%d.......len:%d\r\n", cmd[0], cmd[1], len);
	
	if (cmd[0] == 0) {
	    pdata = kzalloc(sizeof(char)*length, GFP_KERNEL);
	    if(pdata == NULL) {
	        HYN_ERR(" cst3xxzalloc GFP_KERNEL memory fail.\n");
	        return -ENOMEM;
	    }
		ret = cst3xx_read_fw_file(&cmd[1], pdata, &length);
	  	if(ret < 0) {
			if(pdata != NULL) {
				kfree(pdata);
				pdata = NULL;	
			}				
			return -EPERM;
	  	}
		
		ret = cst3xx_apk_fw_dowmload(g_i2c_client, pdata, length);
	  	if(ret < 0){
	        HYN_ERR(" cst3xxupdate firmware failed.\n");
			if(pdata != NULL) {
				kfree(pdata);
				pdata = NULL;	
			}	
	        return -EPERM;
		}
        mdelay(50);
		
		cst3xx_firmware_info(g_i2c_client);    
		
		if(pdata != NULL) {
			kfree(pdata);
			pdata = NULL;	
		}
	}
	else if (cmd[0] == 2) {					
		//cst3xx_touch_release();		
		CMDIndex = cmd[1];			
	}			
	else if (cmd[0] == 3) {				
		CMDIndex = 0;		
	}	
			
	return count;
}

static const struct file_operations proc_tool_debug_fops = {

	.owner		= THIS_MODULE,

	.read	    = cst3xx_proc_read_foobar,

	.write		= cst3xx_proc_write_foobar, 	

};



static int  cst3xx_proc_fs_init(void)

{

	int ret;	

	g_proc_dir = proc_mkdir(CST3XX_PROC_DIR_NAME, NULL);

	if (g_proc_dir == NULL) {

		ret = -ENOMEM;

		goto out;

	}

#if 0
    g_update_file = proc_create(CST3XX_PROC_FILE_NAME, 0777, g_proc_dir,&proc_tool_debug_fops);

   if (g_update_file == NULL)
   {
      ret = -ENOMEM;
      HYN_INF("proc_create CST3XX_PROC_FILE_NAME failed.\n");
      goto no_foo;

   }
#else
	g_update_file = proc_create_data(CST3XX_PROC_FILE_NAME, 0777 | S_IFREG, g_proc_dir, &proc_tool_debug_fops, (void *)g_i2c_client);
	if (NULL == g_update_file) {
		ret = -ENOMEM;
      	HYN_ERR("proc_create_data CST3XX_PROC_FILE_NAME failed.\n");
      	goto no_foo;
	}
#endif

  
	return 0;

no_foo:

	remove_proc_entry(CST3XX_PROC_FILE_NAME, g_proc_dir);

out:

	return ret;

}


#ifdef HYN_SYSFS_NODE_EN 

static ssize_t hyn_tpfwver_show(struct device *dev,	struct device_attribute *attr,char *buf)
{
	ssize_t num_read_chars = 0;
	u8 buf1[20];
	int ret=-1;
	unsigned int firmware_version,module_version,project_version,chip_type,checksum;
	
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	memset((u8 *)buf1, 0, 20);
	mutex_lock(&g_device_mutex);


	firmware_version=0;
	module_version=0;
	project_version=0;
	chip_type=0;
	checksum=0;


	buf1[0] = 0xD1;
	buf1[1] = 0x01;
	ret = cst3xx_i2c_write(g_i2c_client, buf1, 2);
	if (ret < 0) return -1;
	
	mdelay(10);

	buf1[0] = 0xD2;
	buf1[1] = 0x04;
	ret = cst3xx_i2c_read_register(g_i2c_client, buf1, 4);
	if (ret < 0) return -1;	


	chip_type = buf1[3];
	chip_type <<= 8;
	chip_type |= buf1[2];

	
	project_version |= buf1[1];
	project_version <<= 8;
	project_version |= buf1[0];

	buf1[0] = 0xD2;
	buf1[1] = 0x08;
	ret = cst3xx_i2c_read_register(g_i2c_client, buf1, 4);
	if (ret < 0) return -1;	


	firmware_version = buf1[3];
	firmware_version <<= 8;
	firmware_version |= buf1[2];
	firmware_version <<= 8;
	firmware_version |= buf1[1];
	firmware_version <<= 8;
	firmware_version |= buf1[0];

	buf1[0] = 0xD2;
	buf1[1] = 0x0C;
	ret = cst3xx_i2c_read_register(g_i2c_client, buf1, 4);
	if (ret < 0) return -1;	


	checksum = buf1[3];
	checksum <<= 8;
	checksum |= buf1[2];
	checksum <<= 8;
	checksum |= buf1[1];
	checksum <<= 8;
	checksum |= buf1[0];	

	buf1[0] = 0xD1;
	buf1[1] = 0x09;
	ret = cst3xx_i2c_write(g_i2c_client, buf1, 2);

	num_read_chars = snprintf(buf, 128, "firmware_version: 0x%02X,module_version:0x%02X,project_version:0x%02X,chip_type:0x%02X,checksum:0x%02X .\n",firmware_version,module_version, project_version,chip_type,checksum);
	

	mutex_unlock(&g_device_mutex);

	return num_read_chars;
}

static ssize_t hyn_tpfwver_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}


static ssize_t hyn_tprwreg_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}

static ssize_t hyn_tprwreg_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u16 regaddr = 0xff;
	u8 valbuf[10] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			HYN_ERR("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = kstrtoul(valbuf, 16, &wmreg);
	

	if (0 != retval) {
		HYN_ERR("%s() - ERROR: The given input was: \"%s\"\n",__func__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		/*read register*/
		regaddr = valbuf[0]<<8;
		regaddr |= valbuf[1];

		if(regaddr==0x3838){   //88-ascll
			disable_irq(touch_irq);
		}else if(regaddr==0x3939){//99-ascll
			enable_irq(touch_irq);
		}else if(regaddr==0x3737){
			cst3xx_reset_ic(10);
		}

		
		if (cst3xx_i2c_read_register(g_i2c_client, valbuf,num_read_chars) < 0)
			HYN_ERR("Could not read the register(0x%02x).\n",regaddr);
		else
			HYN_ERR("the register(0x%02x) is 0x%02x\n",regaddr,valbuf[0] );
	} else {
		regaddr = valbuf[0]<<8;
		regaddr |= valbuf[1];
		if (cst3xx_i2c_read_register(g_i2c_client, valbuf, num_read_chars) < 0)
			HYN_ERR("Could not write the register(0x%02x)\n",regaddr);
		else
			HYN_ERR("Write 0x%02x into register(0x%02x) successful\n",regaddr, valbuf[0]);
	}

error_return:
	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t hyn_fwupdate_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/*upgrade from *.i*/
static ssize_t hyn_fwupdate_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	HYN_INF("hyn_fwupdate_store enter.\n");
	mutex_lock(&g_device_mutex);
    hyn_boot_update_fw(g_i2c_client);
	mutex_unlock(&g_device_mutex);
	return count;
}

static ssize_t hyn_fwupgradeapp_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}


/*upgrade from app.bin*/
static ssize_t hyn_fwupgradeapp_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	char fwname[256];
	int ret;
	unsigned char *pdata = NULL;
	int length = 24*1024;
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);


	HYN_INF("hyn_fwupgradeapp_store enter.\n");

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "/mnt/%s", buf);	
	fwname[count-1+8] = '\0';
	
	HYN_INF("fwname:%s.\n",fwname);
	pdata = kzalloc(sizeof(char)*length, GFP_KERNEL);
    if(pdata == NULL) 
	{
        HYN_ERR("hyn_fwupgradeapp_store GFP_KERNEL memory fail.\n");
        return -ENOMEM;
    }

	mutex_lock(&g_device_mutex);
	ret = cst3xx_read_fw_file(fwname, pdata, &length);
  	if(ret < 0) 
  	{
		HYN_ERR("cst3xx_read_fw_file fail.\n");
		if(pdata != NULL) 
		{
			kfree(pdata);
			pdata = NULL;	
		}			
  	}else{

		ret = cst3xx_apk_fw_dowmload(g_i2c_client, pdata, length);
	  	if(ret < 0)
	  	{
	        HYN_ERR("cst2xx_apk_fw_dowmload failed.\n");
			if(pdata != NULL) 
			{
				kfree(pdata);
				pdata = NULL;	
			}	
		}
	}

	mutex_unlock(&g_device_mutex);
	
	HYN_INF("hyn_fwupgradeapp_store exit.\n");
	
	return count;
}



/*sysfs */
/*get the fw version
*example:cat hyntpfwver
*/
static DEVICE_ATTR(hyntpfwver, S_IRUGO | S_IWUSR, hyn_tpfwver_show,
			hyn_tpfwver_store);

/*upgrade from *.i
*example: echo 1 > hynfwupdate
*/
static DEVICE_ATTR(hynfwupdate, S_IRUGO | S_IWUSR, hyn_fwupdate_show,
			hyn_fwupdate_store);

/*read and write register
*read example: echo 88 > hyntprwreg ---read register 0x88
*write example:echo 8807 > hyntprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(hyntprwreg, S_IRUGO | S_IWUSR, hyn_tprwreg_show,
			hyn_tprwreg_store);


/*upgrade from app.bin
*example:echo "*_app.bin" > hynfwupgradeapp
*/
static DEVICE_ATTR(hynfwupgradeapp, S_IRUGO | S_IWUSR, hyn_fwupgradeapp_show,
			hyn_fwupgradeapp_store);

/*add your attr in here*/
static struct attribute *hyn_attributes[] = {
	&dev_attr_hyntpfwver.attr,
	&dev_attr_hynfwupdate.attr,
	&dev_attr_hyntprwreg.attr,
	&dev_attr_hynfwupgradeapp.attr,
	NULL
};

static struct attribute_group hyn_attribute_group = {
	.attrs = hyn_attributes
};
/*create sysfs for debug*/

int hyn_create_sysfs(struct i2c_client *client)
{
	int err;
	g_i2c_client=client;
  	if ((k_obj = kobject_create_and_add("hynitron_debug", NULL)) == NULL ) {
     	HYN_ERR("hynitron_debug sys node create error.\n"); 	
    }
	err = sysfs_create_group(k_obj, &hyn_attribute_group);
	if (0 != err) {
		HYN_ERR("%s() - ERROR: sysfs_create_group() failed.\n",__func__);
		sysfs_remove_group(k_obj, &hyn_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_device_mutex);
		HYN_ERR("cst3xx:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	return err;
}

void hyn_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(k_obj, &hyn_attribute_group);
	mutex_destroy(&g_device_mutex);		
}
#endif 


#endif




#ifdef CST2XX_SUPPORT

#define CST2XX_BASE_ADDR		(24 * 1024)

#if 0
static int cst2xx_checksum_judge( unsigned char *pdata)
{
	int ret;
	int i;
	unsigned char buf[4];
	unsigned char *pBuf;
	unsigned int fw_checksum;
    //check_sum
	for(i=0; i<5; i++)
	{

		buf[0] = 0xD0;
		buf[1] = 0x4C;
		buf[2] = 0x55;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 3);
		if (ret < 0)
		{
			mdelay(1);
			continue;
		}

		mdelay(80);

		buf[0] = 0xD0;
		buf[1] = 0x08;
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 4);
		if(ret < 0)
		{
			mdelay(1);
			continue;
		}

		g_cst3xx_ic_checksum = buf[3];
		g_cst3xx_ic_checksum <<= 8;
		g_cst3xx_ic_checksum |= buf[2];
		g_cst3xx_ic_checksum <<= 8;
		g_cst3xx_ic_checksum |= buf[1];
		g_cst3xx_ic_checksum <<= 8;
		g_cst3xx_ic_checksum |= buf[0];	

	}

	

	pBuf = &pdata[6*1024+24-8];
	
	fw_checksum = pBuf[3];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[2];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[1];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[0];	


   HYN_INF(" cst3xx[cst3xx]the updating firmware:fw_checksum:0x%x,the chip checksum:0x%x .\n",
		 fw_checksum,g_cst3xx_ic_checksum);
	if(g_cst3xx_ic_checksum!=fw_checksum)
	{
        return -1;	
	}

	return 0;

}

#endif


static int cst2xx_enter_download_mode(void)
{
	int ret;
	int i;
	unsigned char buf[4];

	cst3xx_reset_ic(5);

	for(i=0; i<40; i++)
	{		
		buf[0] = 0xD1;
		buf[1] = 0x11;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if (ret < 0)
		{
			mdelay(1);
			continue;
		}
	
		mdelay(1); //wait enter download mode
		
		buf[0] = 0xD0;
		buf[1] = 0x01;
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 1);
		if(ret < 0)
		{
			mdelay(1);
			continue;
		}

		if (buf[0] == 0x55)
			break;
	}

	if(buf[0] != 0x55)
	{
		HYN_ERR("  [HYN]reciev 0x55 failed.\r\n");
		return -1;
	}

	cancel_delayed_work(&cst3xx_reset_chip_work);

	buf[0] = 0xD1;
	buf[1] = 0x10;   //enter writer register mode
	ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
	if (ret < 0)
	{
		HYN_ERR("  [HYN]Send cmd 0xD110 failed. \r\n");
		return -1;
	}

	return 0;
}

static int cst2xx_download_program(unsigned char *data, int len)
{	
	int ret;
	int i, j;
	unsigned int wr_addr;
	unsigned char *pData;
	unsigned char *pSrc;
	unsigned char *pDst;
	unsigned char  i2c_buf[8];

	pData = kmalloc(sizeof(unsigned char)*(1024 + 4), GFP_KERNEL);
	if(NULL == pData)
	{
		HYN_ERR("  malloc data buffer failed.\n");
		return -1;
	}
	
	pSrc = data;
	HYN_INF("  write program data begain:0x%x.\n", len);
	
	for(i=0; i<(len/1024); i++)
	{
		wr_addr  = (i<<10) + CST2XX_BASE_ADDR;
		
		pData[0] = (wr_addr >> 24) & 0xFF;
		pData[1] = (wr_addr >> 16) & 0xFF;
		pData[2] = (wr_addr >> 8) & 0xFF;
		pData[3] =  wr_addr & 0xFF;

		pDst = pData + 4;
		
		for(j=0; j<256; j++)
		{
			*pDst       = *(pSrc + 3);
			*(pDst + 1) = *(pSrc + 2);
			*(pDst + 2) = *(pSrc + 1);
			*(pDst + 3) = *pSrc;
			
			pDst += 4;
			pSrc += 4;
		}

		#ifdef TRANSACTION_LENGTH_LIMITED
		for(j=0; j<256; j++)
		{
            i2c_buf[0] = (wr_addr >> 24) & 0xFF;
    		i2c_buf[1] = (wr_addr >> 16) & 0xFF;
    		i2c_buf[2] = (wr_addr >> 8) & 0xFF;
    		i2c_buf[3] =  wr_addr & 0xFF;

            i2c_buf[4] =  pData[j*4+4+0];
    		i2c_buf[5] =  pData[j*4+4+1];
    		i2c_buf[6] =  pData[j*4+4+2];
    		i2c_buf[7] =  pData[j*4+4+3];    
    		
    		ret = cst3xx_i2c_write(g_i2c_client, i2c_buf, 8);
    		if(ret < 0)
    		{
    			HYN_ERR("  program failed.\n");
    			goto ERR_OUT;
    		}

			wr_addr += 4;
		}
		#else
		ret = cst3xx_i2c_write(g_i2c_client, pData, 1024+4);
		if (ret < 0)
		{
			HYN_ERR("  program failed.\n");
			goto ERR_OUT;
		}
		
		#endif

		mdelay(200);
	}

	mdelay(10);


    //clear update key
	pData[3] = 0x20000FF8 & 0xFF;
	pData[2] = (0x20000FF8>>8)  & 0xFF;
	pData[1] = (0x20000FF8>>16) & 0xFF;	
	pData[0] = (0x20000FF8>>24) & 0xFF;
	pData[4] = 0x00;
	pData[5] = 0x00;
	pData[6] = 0x00;
	pData[7] = 0x00;	
	ret = cst3xx_i2c_write(g_i2c_client, pData, 8);
	if (ret < 0)
	{
		HYN_ERR("  clear update key failed.\n");
		goto ERR_OUT;
	}
	
	pData[3] = 0xD013D013 & 0xFF;
	pData[2] = (0xD013D013>>8)  & 0xFF;
	pData[1] = (0xD013D013>>16) & 0xFF;	
	pData[0] = (0xD013D013>>24) & 0xFF;
	ret = cst3xx_i2c_write(g_i2c_client, pData, 4);
	if (ret < 0)
	{
		HYN_ERR("  exit register read/write mode failed.\n");
		goto ERR_OUT;
	}

	pData[0] = 0xD0;
	pData[1] = 0x00;
	ret = cst3xx_i2c_read_register(g_i2c_client, pData, 1);
	if (ret < 0)
	{
		HYN_ERR("  exit register read/write mode failed.\n");
		goto ERR_OUT;
	}
	if(pData[0]!=0x0A)
	{
        goto ERR_OUT;
	}
	
	HYN_INF("  --------write program data end--------.\r\n");

	if (pData != NULL)
	{
		kfree(pData);
		pData = NULL;
	}	
	return 0;
	
ERR_OUT:
	if (pData != NULL)
	{
		kfree(pData);
		pData = NULL;
	}
	return -1;	
}

static int cst2xx_read_checksum(void)
{
	int ret;
	int i;
	unsigned int  checksum;
	unsigned int  bin_checksum;
	unsigned char buf[4];
	unsigned char *pData;

	for(i=0; i<10; i++)
	{
		buf[0] = 0xD0;
		buf[1] = 0x00;
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 1);
		if(ret < 0)
		{
			mdelay(2);
			continue;
		}

		if((buf[0]==0x01) || (buf[0]==0x02))
			break;

		mdelay(2);
	}

	if((buf[0]==0x01) || (buf[0]==0x02))
	{
		buf[0] = 0xD0;
		buf[1] = 0x08;
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 4);
		
		if(ret < 0)	return -1;
		
		//handle read data  --> checksum
		checksum = buf[0] + (buf[1]<<8) + (buf[2]<<16) + (buf[3]<<24);

        pData = pcst3xx_update_firmware + 6160; //6*1024+16
		bin_checksum = pData[0] + (pData[1]<<8) + (pData[2]<<16) + (pData[3]<<24);

		if(checksum!=bin_checksum)
		{
			HYN_ERR("  Check sum error.\n");
		}

        HYN_INF("  checksum ic:0x%x. bin:0x%x-----\n", checksum, bin_checksum);
		
		buf[0] = 0xD0;
		buf[1] = 0x01;
		buf[2] = 0xA5;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 3);
		
		if(ret < 0) return -1;
	}
	else
	{
		HYN_ERR("  No checksum.\n");
		return -1;
	}
	
	return 0;
}

static int cst2xx_update_firmware(struct i2c_client * client,
	unsigned char *pdata)
{
	int ret;
	int retry;
    int data_len=6*1024;
	
	retry = 0;
	
start_flow:

	HYN_INF("  enter the update firmware.\n");

	ret = cst2xx_enter_download_mode();
	if (ret < 0)
	{
		HYN_ERR("  enter download mode failed.\n");
		goto fail_retry;
	}
	
	ret = cst2xx_download_program(pdata, data_len);
	if (ret < 0)
	{
		HYN_ERR("  download program failed.\n");
		goto fail_retry;
	}

	mdelay(3);
		
	ret = cst2xx_read_checksum();
	if (ret < 0)
	{
		HYN_ERR("  checksum failed.\n");
		goto fail_retry;
	}

	cst3xx_reset_ic(100);

	HYN_INF("  Download firmware succesfully.\n");

	return 0;
	
fail_retry:
	if (retry < 10)
	{
		retry++;
		goto start_flow;
	}
	
	return -1;
}

#endif






/*******************************************************
Function:
    read checksum in bootloader mode
Input:
    client: i2c client
    strict: check checksum value
Output:
    success: 0
    fail:	-1
*******************************************************/

#define CST3XX_BIN_SIZE    (24*1024 + 24)

static int cst3xx_check_checksum(struct i2c_client * client)
{
	int ret;
	int i;
	unsigned int  checksum;
	unsigned int  bin_checksum;
	unsigned char buf[4];
	const unsigned char *pData;

	for(i=0; i<5; i++)
	{
		buf[0] = 0xA0;
		buf[1] = 0x00;
		ret = cst3xx_i2c_read_register(client, buf, 1);
		if(ret < 0)
		{
			mdelay(2);
			continue;
		}

		if(buf[0]!=0)
			break;
		else
		mdelay(2);
	}
    mdelay(2);


    if(buf[0]==0x01)
	{
		buf[0] = 0xA0;
		buf[1] = 0x08;
		ret = cst3xx_i2c_read_register(client, buf, 4);
		
		if(ret < 0)	return -1;
		
		// read chip checksum
		checksum = buf[0] + (buf[1]<<8) + (buf[2]<<16) + (buf[3]<<24);

        pData=(unsigned char  *)pcst3xx_update_firmware +24*1024+16;   //7*1024 +512
		bin_checksum = pData[0] + (pData[1]<<8) + (pData[2]<<16) + (pData[3]<<24);

        HYN_INF("  hyn the updated ic checksum is :0x%x. the updating firmware checksum is:0x%x------\n", checksum, bin_checksum);
    
        if(checksum!=bin_checksum)
		{
			HYN_ERR(" cst3xx hyn check sum error.\n");		
			return -1;
			
		}
		
	}
	else
	{
		HYN_ERR(" cst3xx hyn No checksum.\n");
		return -1;
	}	
	return 0;
}
static int cst3xx_into_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[4];
	
	buf[0] = 0xA0;
	buf[1] = 0x01;	
	buf[2] = 0xAA;	//set cmd to enter program mode		
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)  return -1;

	mdelay(2);
	
	buf[0] = 0xA0;
	buf[1] = 0x02;	//check whether into program mode
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;
	
	if (buf[0] != 0x55) return -1;
	
	return 0;
}

static int cst3xx_exit_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];
	
	buf[0] = 0xA0;
	buf[1] = 0x06;
	buf[2] = 0xEE;
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)
		return -1;
	
	mdelay(10);	//wait for restart

	
	return 0;
}

static int cst3xx_erase_program_area(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];
	
	buf[0] = 0xA0;
	buf[1] = 0x02;	
	buf[2] = 0x00;		//set cmd to erase main area		
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0) return -1;
	
	mdelay(5);
	
	buf[0] = 0xA0;
	buf[1] = 0x03;
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;
	
	if (buf[0] != 0x55) return -1;

	return 0;
}

static int cst3xx_write_program_data(struct i2c_client * client,
		const unsigned char *pdata)
{
	int i, ret;
	unsigned char *i2c_buf;
	unsigned short eep_addr;
	int total_kbyte;
#ifdef TRANSACTION_LENGTH_LIMITED
	unsigned char temp_buf[8];
	unsigned short iic_addr;
	int  j;

#endif
	

	i2c_buf = kmalloc(sizeof(unsigned char)*(1024 + 2), GFP_KERNEL);
	if (i2c_buf == NULL) 
		return -1;
	
	//make sure fwbin len is N*1K
	//total_kbyte = len / 1024;
	total_kbyte = 24;
	for (i=0; i<total_kbyte; i++) {
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x14;
		eep_addr = i << 10;		//i * 1024
		i2c_buf[2] = eep_addr;
		i2c_buf[3] = eep_addr>>8;
		ret = cst3xx_i2c_write(client, i2c_buf, 4);
		if (ret < 0)
			goto error_out;




	
#ifdef TRANSACTION_LENGTH_LIMITED
		memcpy(i2c_buf, pdata + eep_addr, 1024);
		for(j=0; j<256; j++) {
			iic_addr = (j<<2);
    	temp_buf[0] = (iic_addr+0xA018)>>8;
    	temp_buf[1] = (iic_addr+0xA018)&0xFF;
		temp_buf[2] = i2c_buf[iic_addr+0];
		temp_buf[3] = i2c_buf[iic_addr+1];
		temp_buf[4] = i2c_buf[iic_addr+2];
		temp_buf[5] = i2c_buf[iic_addr+3];
    	ret = cst3xx_i2c_write(client, temp_buf, 6);
    		if (ret < 0)
    			goto error_out;		
		}
#else
		
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x18;
		memcpy(i2c_buf + 2, pdata + eep_addr, 1024);
		ret = cst3xx_i2c_write(client, i2c_buf, 1026);
		if (ret < 0)
			goto error_out;
#endif
		
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x04;
		i2c_buf[2] = 0xEE;
		ret = cst3xx_i2c_write(client, i2c_buf, 3);
		if (ret < 0)
			goto error_out;
		
		mdelay(60);	
		
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x05;
		ret = cst3xx_i2c_read_register(client, i2c_buf, 1);
		if (ret < 0)
			goto error_out;
		
		if (i2c_buf[0] != 0x55)
			goto error_out;

	}
	
	i2c_buf[0] = 0xA0;
	i2c_buf[1] = 0x03;
	i2c_buf[2] = 0x00;
	ret = cst3xx_i2c_write(client, i2c_buf, 3);
	if (ret < 0)
		goto error_out;
	
	mdelay(8);	
	
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}

	return 0;
	
error_out:
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}
	return -1;
}

static int cst3xx_update_firmware(struct i2c_client * client, const unsigned char *pdata)
{
	int ret;
	int retry = 0;
	unsigned short iic_addr;
	
	HYN_INF(" cst3xx----------upgrade cst3xx begain------------\n");
	disable_irq(touch_irq);
	mdelay(20);
START_FLOW:	


#ifdef HYN_UPDATE_POWERON_ENABLE
 // 	ret = regulator_disable(vdd_ana);
	if (ret != 0)
		HYN_ERR(" cst3xx hyn Failed to enable reg-vgp2: %d\n", ret);	
	mdelay(10);
//	ret = regulator_enable(vdd_ana);
	if (ret != 0)
		HYN_ERR(" cst3xx hyn Failed to enable reg-vgp2: %d\n", ret);
	mdelay(5+retry);
#else
	client->addr = CST3XX_I2C_ADDR;
	cst3xx_reset_ic(5+retry);
#endif
	client->addr = CST3XX_I2C_BOOT_ADDR;
	HYN_INF("i2c_client_HYN->boot_addr=%d.\n",client->addr);	

	ret = cst3xx_into_program_mode(client);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx]into program mode failed.\n");
		goto err_out;
	}

	ret = cst3xx_erase_program_area(client);
	if (ret) {
		HYN_ERR(" cst3xx[cst3xx]erase main area failed.\n");
		goto err_out;
	}

	ret = cst3xx_write_program_data(client, pdata);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx]write program data into cstxxx failed.\n");
		goto err_out;
	}

    ret =cst3xx_check_checksum(client);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx] after write program cst3xx_check_checksum failed.\n");
		goto err_out;
	}

	ret = cst3xx_exit_program_mode(client);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx]exit program mode failed.\n");
		goto err_out;
	}
	client->addr = CST3XX_I2C_ADDR;

	cst3xx_reset_ic(30);
	
	HYN_INF(" cst3xx hyn----------cst3xx_update_firmware  end------------\n");

	enable_irq(touch_irq);
	
	return 0;
	
err_out:
	if (retry < 30) {
		retry++;
		mdelay(30);
		goto START_FLOW;
	} 
	else {	
		client->addr = CST3XX_I2C_ADDR;
		enable_irq(touch_irq);
		return -1;
	}
}

static int cst3xx_update_judge( unsigned char *pdata, int strict)
{
	unsigned short ic_type, project_id;
	unsigned int fw_checksum, fw_version;
	const unsigned int *p;
	int i;
	unsigned char *pBuf;
		
	fw_checksum = 0x55;
	p = (const unsigned int *)pdata;
	for (i=0; i<(CST3XX_BIN_SIZE-4); i+=4) {
		fw_checksum += (*p);
		p++;
	}
	
	if (fw_checksum != (*p)) {
		HYN_ERR(" cst3xx[cst3xx]calculated checksum error:0x%x not equal 0x%x.\n", fw_checksum, *p);
		return -1;	//bad fw, so do not update
	}
	
	pBuf = &pdata[CST3XX_BIN_SIZE-16];
	
	project_id = pBuf[1];
	project_id <<= 8;
	project_id |= pBuf[0];

	ic_type = pBuf[3];
	ic_type <<= 8;
	ic_type |= pBuf[2];

	fw_version = pBuf[7];
	fw_version <<= 8;
	fw_version |= pBuf[6];
	fw_version <<= 8;
	fw_version |= pBuf[5];
	fw_version <<= 8;
	fw_version |= pBuf[4];

	fw_checksum = pBuf[11];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[10];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[9];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[8];	
	
	HYN_INF(" cst3xx[cst3xx]the updating firmware:project_id:0x%04x,ic type:0x%04x,version:0x%x,checksum:0x%x\n",
			project_id, ic_type, fw_version, fw_checksum);
#ifndef HYN_UPDATE_FIRMWARE_ENABLE
	HYN_INF("[cst3xx] HYN_UPDATE_FIRMWARE_ENABLE is not open.\n");
    return -1;
#endif

	if (strict > 0) {
		
		if (g_cst3xx_ic_checksum != fw_checksum){
			
#ifdef HYN_UPDATE_FIRMWARE_FORCE
		    HYN_INF("[cst3xx] update firmware online force.\n");
		    return 0;
#endif 
			
			if (g_cst3xx_ic_version >fw_version){
				HYN_ERR("[cst3xx]fw version(%d), ic version(%d).\n",fw_version, g_cst3xx_ic_version);
				return -1;
			}
		}else{
			HYN_ERR("[cst3xx]fw checksum(0x%x), ic checksum(0x%x).\n",fw_checksum, g_cst3xx_ic_checksum);
			return -1;
		}
	}	
	
	return 0;
}


/*******************************************************
Function:
    get firmware version, ic type...
Input:
    client: i2c client
Output:
    success: 0
    fail:	-1
*******************************************************/
static int cst3xx_firmware_info(struct i2c_client * client)
{
	int ret;
	unsigned char buf[28];
//	unsigned short ic_type, project_id;

	
	buf[0] = 0xD1;
	buf[1] = 0x01;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
	
	mdelay(10);

	buf[0] = 0xD1;
	buf[1] = 0xFC;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;

	//0xCACA0000
	g_cst3xx_ic_checkcode = buf[3];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[2];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[1];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[0];
	
	printk("linc cst3xx [cst3xx] the chip g_cst3xx_ic_checkcode:0x%x.\r\n",g_cst3xx_ic_checkcode);
	if((g_cst3xx_ic_checkcode&0xffff0000)!=0xCACA0000){
		printk("linc cst3xx [cst3xx] cst3xx_firmware_info read error .\r\n");
		return -1;
	}

	mdelay(10);

	buf[0] = 0xD2;
	buf[1] = 0x04;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;
	g_cst3xx_ic_type = buf[3];
	g_cst3xx_ic_type <<= 8;
	g_cst3xx_ic_type |= buf[2];

	
	g_cst3xx_ic_project_id = buf[1];
	g_cst3xx_ic_project_id <<= 8;
	g_cst3xx_ic_project_id |= buf[0];

	printk("linc cst3xx [cst3xx] the chip ic g_cst3xx_ic_type :0x%x, g_cst3xx_ic_project_id:0x%x\r\n",
		g_cst3xx_ic_type, g_cst3xx_ic_project_id);

	mdelay(2);
	
	buf[0] = 0xD2;
	buf[1] = 0x08;
	ret = cst3xx_i2c_read_register(client, buf, 8);
	if (ret < 0) return -1;	

	g_cst3xx_ic_version = buf[3];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[2];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[1];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[0];

	g_cst3xx_ic_checksum = buf[7];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[6];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[5];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[4];	

	tpd_info.vid = g_cst3xx_ic_version;
    tpd_info.pid = 0x00;


	HYN_INF(" cst3xx [cst3xx] the chip ic version:0x%x, checksum:0x%x\r\n",
		g_cst3xx_ic_version, g_cst3xx_ic_checksum);

	if(g_cst3xx_ic_version==0xA5A5A5A5)
	{
		HYN_ERR(" cst3xx [cst3xx] the chip ic don't have firmware. \n");
		return -1;
	}

    buf[0] = 0xD1;
	buf[1] = 0x09;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
    mdelay(5);
	
	
	return 0;
}

static int cst3xx_boot_update_fw(struct i2c_client   *client,  unsigned char *pdata)
{
	int ret;
	int retry = 0;

	
	ret = cst3xx_update_judge(pdata, 1);
	if (ret < 0) {
		HYN_ERR(" cst3xx[cst3xx] no need to update firmware.\n");
		return 0;
	}

	
	ret = cst3xx_update_firmware(client, pdata);
	if (ret < 0){
		HYN_ERR(" cst3xx [cst3xx] update firmware failed.\n");
		return -1;
	}

    mdelay(50);

	ret = cst3xx_firmware_info(client);
	if (ret < 0) {
		HYN_ERR(" cst3xx [cst3xx] after update read version and checksum fail.\n");
		return -1;
	}

	

	return 0;
}

static int cst3xx_find_fw_idx( int project_id )
{
	

	int i = 0;
	if (0 != project_id) {
		for (i=0; i<ARRAY_SIZE(cst3xx_fw_grp); i++) {
			if (project_id==cst3xx_fw_grp[i].project_id){
				return i;
			}
		}
	}
	return -1;

}
static int hyn_boot_update_fw(struct i2c_client * client)
{
	unsigned char *ptr_fw;
	int ret;
	int retry=0;
	int proj_id=0;

	while (retry++ < 3) {
		ret = cst3xx_firmware_info(client);
		if (ret == 0) {
			break;
		}
	}

	proj_id=cst3xx_find_fw_idx(g_cst3xx_ic_project_id);
    printk("[cst3xx] cst3xx_firmware_info proj_id:%d.\r\n",proj_id);
	
	if(proj_id<0){
		printk("cst3xx: not find matched TP firmware,update default firmware !\n"); 
		return -1;		
	}else{	
		printk("cst3xx: find matched TP firmware(%s)!\n", cst3xx_fw_grp[proj_id].name);
		pcst3xx_update_firmware=cst3xx_fw_grp[proj_id].fw;

	}
	ptr_fw = pcst3xx_update_firmware;



	
	
#ifdef CST2XX_SUPPORT
	if(hyn_ic_type==18868) {
	    ret = cst2xx_update_firmware(client, ptr_fw);
	    return ret;
	}
#endif	
	
	    ret = cst3xx_boot_update_fw(client, ptr_fw);
	    return ret;
	
}


static void cst3xx_touch_report(struct input_dev *input_dev)
{
	unsigned char buf[30];
	unsigned char i2c_buf[8];
	unsigned char key_status, key_id = 0, finger_id, sw;
	unsigned int  input_x = 0; 
#if TPD_REVERT_XY 
	unsigned int  input_xy_temp = 0; 
#endif
	unsigned int  input_y = 0; 
	unsigned int  input_w = 0;
    unsigned char cnt_up, cnt_down;
	int   i, ret, idx; 
	int cnt, i2c_len;
#ifdef TRANSACTION_LENGTH_LIMITED
	int  len_1, len_2;
#endif

	printk(" cst3xx_touch_report \n");

    key_status = 0;

	buf[0] = 0xD0;
	buf[1] = 0x00;
	ret = cst3xx_i2c_read_register(g_i2c_client, buf, 7);
	if(ret < 0) {
		HYN_ERR("  iic read touch point data failed.\n");
		goto OUT_PROCESS;
	}

		
	if(buf[6] != 0xAB) {
		//HYN_INF("  data is not valid..\r\n");
		goto OUT_PROCESS;
	}

	if(buf[5] == 0x80) {
		key_status = buf[0];
		key_id = buf[1];		
		goto KEY_PROCESS;
	} 
	
	cnt = buf[5] & 0x7F;
	if(cnt > TPD_MAX_FINGERS) goto OUT_PROCESS;
	else if(cnt==0)     goto CLR_POINT;
	
	if(cnt == 0x01) {
		goto FINGER_PROCESS;
	} 
	else {
		#ifdef TRANSACTION_LENGTH_LIMITED
		if((buf[5]&0x80) == 0x80) //key
		{
			i2c_len = (cnt - 1)*5 + 3;
			len_1   = i2c_len;
			for(idx=0; idx<i2c_len; idx+=6) {
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;
				
				if(len_1>=6) {
					len_2  = 6;
					len_1 -= 6;
				}
				else {
					len_2 = len_1;
					len_1 = 0;
				}
				
    			ret = cst3xx_i2c_read_register(g_i2c_client, i2c_buf, len_2);
    			if(ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++) {
                   buf[5+idx+i] = i2c_buf[i];
				}
			}
			
			i2c_len   += 5;
			key_status = buf[i2c_len - 3];
			key_id     = buf[i2c_len - 2];
		} 
		else {			
			i2c_len = (cnt - 1)*5 + 1;
			len_1   = i2c_len;
			
			for(idx=0; idx<i2c_len; idx+=6) {
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;
				
				if(len_1>=6) {
					len_2  = 6;
					len_1 -= 6;
				}
				else {
					len_2 = len_1;
					len_1 = 0;
				}
				
    			ret = cst3xx_i2c_read_register(g_i2c_client, i2c_buf, len_2);
    			if (ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++) {
                   buf[5+idx+i] = i2c_buf[i];
				}
			}			
			i2c_len += 5;
		}
		#else
		if ((buf[5]&0x80) == 0x80) {
			buf[5] = 0xD0;
			buf[6] = 0x07;
			i2c_len = (cnt - 1)*5 + 3;
			ret = cst3xx_i2c_read_register(g_i2c_client, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
			key_status = buf[i2c_len - 3];
			key_id = buf[i2c_len - 2];
		} 
		else {			
			buf[5] = 0xD0;
			buf[6] = 0x07;			
			i2c_len = (cnt - 1)*5 + 1;
			ret = cst3xx_i2c_read_register(g_i2c_client, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
		}
		#endif

		if (buf[i2c_len - 1] != 0xAB) {
			goto OUT_PROCESS;
		}
	}	

    //both key and point
	if((cnt>0)&&(key_status&0x80))  {
        if(report_flag==0xA5) goto KEY_PROCESS; 
	}
	
FINGER_PROCESS:

	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, i2c_buf, 3);
	if(ret < 0) {
		HYN_ERR(" cst3xx hyn send read touch info ending failed.\r\n"); 
		cst3xx_reset_ic(20);
	}
	
	idx = 0;
    cnt_up = 0;
    cnt_down = 0;
	for (i = 0; i < cnt; i++) {
		
		input_x = (unsigned int)((buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0F));
		input_y = (unsigned int)((buf[idx + 2] << 4) | (buf[idx + 3] & 0x0F));	
		input_w = (unsigned int)(buf[idx + 4]);
		sw = (buf[idx] & 0x0F) >> 1;
		finger_id = (buf[idx] >> 4) & 0x0F;

	   // HYN_DEG(" cst3xx buf[0]:%d, buf[1]:%d, buf[2]:%d, buf[3]:%d.\n", buf[idx], buf[idx+1], buf[idx+2], buf[idx+3]);

#if TPD_REVERT_XY 
		input_xy_temp=input_x;
		input_x =input_y ;
		input_y = input_xy_temp;
#endif
	
#if TPD_REVERT_X       	
		input_x = TPD_MAX_X-input_x;
#endif 
#if TPD_REVERT_Y       	
		input_y = TPD_MAX_Y-input_y;
#endif
	   
       // HYN_INF(" cst3xxPoint x:%d, y:%d, id:%d, sw:%d. \n", input_x, input_y, finger_id, sw);

		if (sw == 0x03) {
			cst3xx_touch_down(input_dev, finger_id, input_x, input_y, input_w);
            cnt_down++;
        }
		else {
            cnt_up++;
            #ifdef ICS_SLOT_REPORT
			cst3xx_touch_up(input_dev, finger_id);
            #endif
        }
		idx += 5;
	}
    
    #ifndef ICS_SLOT_REPORT
    if((cnt_up>0) && (cnt_down==0))
        cst3xx_touch_up(input_dev, 0);
    #endif

	if(cnt_down==0)  report_flag = 0;
	else report_flag = 0xCA;

    input_sync(input_dev);
	goto END;

KEY_PROCESS:

	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, i2c_buf, 3);
	if (ret < 0) {
		HYN_ERR(" cst3xx hyn send read touch info ending failed.\r\n"); 
		cst3xx_reset_ic(20);
	}
	
    #ifdef GTP_HAVE_TOUCH_KEY
	if(key_status&0x80) {
		i = (key_id>>4)-1;
        if((key_status&0x7F)==0x03) {
			if((i==key_index)||(key_index==0xFF)) {
        		//cst3xx_touch_down(input_dev, 0, tpd_keys_dim_local[i][0], tpd_keys_dim_local[i][1], 50);
                input_report_key(input_dev, touch_key_array[i], 1);
    			report_flag = 0xA5;
				key_index   = i;
			}
			else {
                input_report_key(input_dev, touch_key_array[key_index], 0);
				key_index = 0xFF;
			}
		}
    	else {
			input_report_key(input_dev, touch_key_array[i], 0);
            cst3xx_touch_up(input_dev, 0);
			report_flag = 0;	
			key_index = 0xFF;
    	}
	}
	#endif	

    #ifdef TPD_HAVE_BUTTON
	if(key_status&0x80) {
		i = (key_id>>4)-1;
        if((key_status&0x7F)==0x03) {
			if((i==key_index)||(key_index==0xFF)) {
        		cst3xx_touch_down(input_dev, 0, tpd_keys_dim_local[i][0], tpd_keys_dim_local[i][1], 50);
    			report_flag = 0xA5;
				key_index   = i;
			}
			else {
				
				key_index = 0xFF;
			}
		}
    	else {
            cst3xx_touch_up(input_dev, 0);
			report_flag = 0;	
			key_index = 0xFF;
    	}
	}
    

	#endif	
	
	input_sync(input_dev);
    goto END;

CLR_POINT:
#ifdef SLEEP_CLEAR_POINT
	#ifdef ICS_SLOT_REPORT
		for(i=0; i<=10; i++) {	
			input_mt_slot(input_dev, i);
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
		}
	#else
	    input_report_key(input_dev, BTN_TOUCH, 0);
		input_mt_sync(input_dev);
	#endif
		input_sync(input_dev);	
#endif		
		
OUT_PROCESS:
	buf[0] = 0xD0;
	buf[1] = 0x00;
	buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, buf, 3);
	if (ret < 0) {
		HYN_ERR("  send read touch info ending failed.\n"); 
		cst3xx_reset_ic(20);
	}

	
	
END:	
	return;
}
static int cst3xx_touch_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 4 };

    //HYN_INF("  ===cst3xx_touch_handler2222\n");
	
	sched_setscheduler(current, SCHED_RR, &param);

	do {
        //enable_irq(touch_irq);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event(waiter, tpd_flag!=0);		
		if(hyn_online_testflag == 1)
		{
			continue;
		}
		tpd_flag = 0;
        //TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
//      eint_flag = 0;
		cst3xx_touch_report(input_dev);
		
	} while (!kthread_should_stop());

	return 0;
}

static irqreturn_t cst3xx_ts_irq_handler(int irq, void *dev_id)
{
  //  HYN_INF("  ===[hyn]tpd irq interrupt===\n");
	//eint_flag = 1;
	tpd_flag  = 1;
	if(hyn_online_testflag == 1)
	{
		HYN_INF("tpd irq interrupt to dbg_waiter\n");
		wake_up(&dbg_waiter);
	}
	else
	{
		HYN_INF("tpd irq interrupt to waiter\n");
		wake_up(&waiter);
	}
	return IRQ_HANDLED;
}

const struct of_device_id touch_of_match[] = {
	{ .compatible = "mediatek,touch", },
	{},
};






static int tpd_irq_registration(void)
{
	int  ret=-1;
	
	if(g_i2c_client->irq ==0){
		HYN_ERR(" cst3xx*** Unable to irq_of_parse_and_map() ***\n");
	}
	else
	{        
		ret = request_irq(g_i2c_client->irq, cst3xx_ts_irq_handler,IRQF_TRIGGER_RISING, "TOUCH_PANEL-eint_cst3xx", NULL);
		if (ret > 0){
			HYN_ERR(" cst3xx tpd request_irq IRQ LINE NOT AVAILABLE!.\n");
		}
	}

	return 0;
}




/*
 * Devices Tree support,
 */


static struct regulator *vdd_ana;
/**
 * gt1x_parse_dt - parse platform infomation form devices tree.
 */
static int gt1x_parse_dt(struct device *dev)
{
	struct device_node *np;

	if (!dev)
		return -ENODEV;

	np = dev->of_node;
	cstx_int_gpio = of_get_named_gpio(np, "cts,irq-gpio", 0);
	cst_rst_gpio = of_get_named_gpio(np, "cts,rst-gpio", 0);

	if (!gpio_is_valid(cstx_int_gpio) || !gpio_is_valid(cst_rst_gpio)) {
		printk("Invalid GPIO, irq-gpio:%d, rst-gpio:%d",
				cstx_int_gpio, cst_rst_gpio);
		return -EINVAL;
	}




	return 0;
}




/**
 * gt1x_request_io_port - Request gpio(INT & RST) ports.
 */
static s32 gt1x_request_io_port(void)
{
	s32 ret = 0;

	
	ret = gpio_request(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		printk("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_INT_PORT, ret);
		return ret;
	}

	GTP_GPIO_AS_INT(GTP_INT_PORT);
	g_i2c_client->irq = GTP_INT_IRQ;

	ret = gpio_request(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		printk("Failed to request GPIO:%d, ERRNO:%d", (s32) GTP_RST_PORT, ret);
		gpio_free(GTP_INT_PORT);
		return ret;
	}

	GTP_GPIO_OUTPUT(GTP_RST_PORT,1);
	return 0;
}


/**
 * gt1x_request_input_dev -  Request input device Function.
 * Return
 *      0: succeed, -1: failed.
 */
static s8 gt1x_request_input_dev(void)
{
	s8 ret = -1;
#if GTP_HAVE_TOUCH_KEY
	u8 idx = 0;
#endif


	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		printk("Failed to allocate input device.");
		return -ENOMEM;
	}

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#if GTP_ICS_SLOT_REPORT
	
#else
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);


#if GTP_CHANGE_X2Y
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TPD_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TPD_MAX_X, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, TPD_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, TPD_MAX_Y, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);


		#ifdef ICS_SLOT_REPORT
        clear_bit(BTN_TOUCH, input_dev->keybit);
		input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, 0, 15, 0, 0);
	    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, (TPD_MAX_FINGERS+1), 0, 0);
    	input_mt_init_slots(input_dev,TPD_MAX_FINGERS+1,INPUT_MT_DIRECT);
		#else 
		input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, (TPD_MAX_FINGERS+1), 0, 0);	
        #endif	



	input_set_abs_params(input_dev, ABS_X, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 255, 0, 0);

	input_dev->name =  "cts-ts";
	input_dev->phys = "input/ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x00;
	input_dev->id.product = 0x00;
	input_dev->id.version = 0x00;

#ifdef GTP_HAVE_TOUCH_KEY
    for (idx=0; idx<TPD_KEY_COUNT; idx++) {
        input_set_capability(input_dev, EV_KEY, touch_key_array[idx]);
    }
#endif	


	ret = input_register_device(input_dev);
	if (ret) {
		printk("Register %s input device failed", input_dev->name);
		return -ENODEV;
	}

	return 0;
}




static int cst3xx_tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	
	int ret=-1;
	int rc=-1;
	//int retry = 0;
	//unsigned char buf[4];

	
#ifdef GTP_HAVE_TOUCH_KEY
    s32 idx = 0;
#endif
	


    HYN_INF("  hyn is entering tpd_i2c_probe. \n");
	
	if(client->addr != CST3XX_I2C_ADDR)
	{
		client->addr = CST3XX_I2C_ADDR;
		HYN_INF("i2c_client_HYN->addr=%d.\n",client->addr);
	}
	g_i2c_client = client;


#ifdef CONFIG_OF	/* device tree support */
	if (client->dev.of_node) {
		ret = gt1x_parse_dt(&client->dev);
		if (ret)
			return ret;
	}
#endif

	ret = gt1x_request_io_port();
	if(ret < 0) {
		HYN_ERR(" cst3xx hyn gt1x_request_io_port failed.\n");
		return ret;
	}

	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	mdelay(100);
	
	HYN_INF(" hyn Device Tree get regulator! \n");
//	vdd_ana = regulator_get(&client->dev, "vtouch");
//	ret = regulator_set_voltage(vdd_ana, 2800000, 2800000);	/*set 2.8v*/
//	if (ret) {
//		HYN_ERR(" cst3xx regulator_set_voltage(%d) failed!\n", ret);
//		return -1;
//    }

//    ret = regulator_enable(vdd_ana);
	if (ret != 0)
		HYN_ERR(" cst3xx hyn Failed to enable reg-vgp2: %d\n", ret);
	
	mdelay(100);
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);	
//	tpd_gpio_as_int(GTP_INT_PORT);	
	mdelay(400);
	

	ret = cst3xx_i2c_test(client);
	if(ret < 0) {
		HYN_ERR(" cst3xx hyn cst3xx_i2c_test failed.\n");
		return ret;
	}
	
	
	ret = gt1x_request_input_dev();
	if (ret < 0) {
		printk("GTP request input dev failed");
	}
	
#ifdef ANDROID_TOOL_SURPORT
	ret = cst3xx_proc_fs_init();
	if(ret < 0) {
		HYN_ERR(" cst3xx hyn create cst3xx proc fs failed.\n");
	}
#ifdef HYN_SYSFS_NODE_EN
    hyn_create_sysfs(client);
#endif
	
#endif

	tpd_irq_registration();	
 


 	disable_irq(touch_irq);
	thread = kthread_run(cst3xx_touch_handler, 0, TPD_DEVICE_NAME);
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		HYN_ERR(" cst3xx hyn create touch event handler thread failed: %d.\n", ret);
		return ret;
	}
    enable_irq(touch_irq);
		


	HYN_INF("  hyn is endding tpd_i2c_probe .\n");

	return ret;
}

static int cst3xx_tpd_remove(struct i2c_client *client)
{
	HYN_INF("  cst3xx removed.\n");
	return 0;
}



static const struct i2c_device_id cst3xx_tpd_id[] = {{TPD_DEVICE_NAME,0},{}};


static const struct of_device_id tpd_of_match[] = {
	{.compatible = "mediatek,cap_touch_cst"},
	{.compatible = "mediatek,cap_touch"},
	{},
};

MODULE_DEVICE_TABLE(of, tpd_of_match);


static struct i2c_driver cst3xx_ts_driver = {

    .driver = {
    .name = TPD_DEVICE_NAME,
	.of_match_table = of_match_ptr(tpd_of_match),
  },

  .probe    = cst3xx_tpd_probe,
  .remove   = cst3xx_tpd_remove,
  .id_table = cst3xx_tpd_id,
//  .detect   = cst3xx_tpd_detect,
  //.address_list = (const unsigned short *)forces,
};
static int cst3xx_local_init(void)
{

//	int ret = 0;

	HYN_INF("  hyn is entering cst3xx_local_init .\n");


	if (i2c_add_driver(&cst3xx_ts_driver) != 0) {
		HYN_ERR(" cst3xx hyn unable to add i2c driver.\n");
		return -1;
	}




	HYN_INF("  hyn is end %s, %d\n", __FUNCTION__, __LINE__);
	
	return 0;
}






/* called when loaded into kernel */
static int __init cst3xx_ts_init(void)
{

	HYN_INF("  HYN_DRIVER_VERSION: %s.\n", HYN_DRIVER_VERSION);
	HYN_INF("  hyn is entering cst3xx_ts_init.\n");


//	tpd_get_dts_info();
	cst3xx_local_init();


	return 0;
}

/* should never be called */
static void __exit cst3xx_ts_exit(void)
{
	HYN_INF("  hyn is entering cst3xx_ts_exit.\n");

}

module_init(cst3xx_ts_init);
module_exit(cst3xx_ts_exit);

