/*
 *  apds9190.c - Linux kernel modules for ambient proximity sensor
 *
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <mach/board_lge.h>

#include <mach/gpio.h>

#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>
#include <linux/wakelock.h>
#include <mach/msm_i2ckbd.h>
#include <linux/spinlock.h>


#define APDS9190_DRV_NAME	"proximity_apds9190"
#define DRIVER_VERSION		"1.0.0"

/*
 * Defines
 */

#define APDS9190_ENABLE_REG		0x00
#define APDS9190_ATIME_REG		0x01 //ALS Non Use, Reserved
#define APDS9190_PTIME_REG		0x02
#define APDS9190_WTIME_REG		0x03
#define APDS9190_AILTL_REG		0x04 //ALS Non Use, Reserved
#define APDS9190_AILTH_REG		0x05 //ALS Non Use, Reserved
#define APDS9190_AIHTL_REG		0x06 //ALS Non Use, Reserved
#define APDS9190_AIHTH_REG		0x07 //ALS Non Use, Reserved
#define APDS9190_PILTL_REG		0x08
#define APDS9190_PILTH_REG		0x09
#define APDS9190_PIHTL_REG		0x0A
#define APDS9190_PIHTH_REG		0x0B
#define APDS9190_PERS_REG		0x0C
#define APDS9190_CONFIG_REG		0x0D
#define APDS9190_PPCOUNT_REG	0x0E
#define APDS9190_CONTROL_REG	0x0F
#define APDS9190_REV_REG		0x11
#define APDS9190_ID_REG			0x12
#define APDS9190_STATUS_REG		0x13
#define APDS9190_CDATAL_REG		0x14 //ALS Non Use, Reserved
#define APDS9190_CDATAH_REG		0x15 //ALS Non Use, Reserved
#define APDS9190_IRDATAL_REG	0x16 //ALS Non Use, Reserved
#define APDS9190_IRDATAH_REG	0x17 //ALS Non Use, Reserved
#define APDS9190_PDATAL_REG		0x18
#define APDS9190_PDATAH_REG		0x19

#define CMD_BYTE	0x80
#define CMD_WORD	0xA0
#define CMD_SPECIAL	0xE0

#define CMD_CLR_PS_INT	0xE5
#define CMD_CLR_ALS_INT	0xE6
#define CMD_CLR_PS_ALS_INT	0xE7

#define APDS9190_ENABLE_PIEN 	 0x20 
#define APDS9190_ENABLE_AIEN 	 0x00 //ALS Non Use, Reserved
#define APDS9190_ENABLE_WEN 	 0x08 
#define APDS9190_ENABLE_PEN 	 0x04 
#define APDS9190_ENABLE_AEN 	 0x00 //ALS Non Use, Reserved
#define APDS9190_ENABLE_PON 	 0x01


#define ATIME 	 	0x00 	// 27.2ms . minimum ALS integration time //ALS Non Use
#define WTIME 		0xf6 	// 27.2ms . minimum Wait time
#define PTIME 	 	0xff 	// 2.72ms . minimum Prox integration time
#define INT_PERS 	0x33


#define PPCOUNT 	7 		//prox pulse count
#define PDRIVE 	 	0xC0 	//12.5mA of LED Power
#define PDIODE 	 	0x20 	// IR Diode
#define PGAIN 	 	0x00 	//1x Prox gain
#define AGAIN 		0x00 	//1x ALS gain, Reserved ALS Non Use

#define APDS_PROXIITY_HIGH_THRESHHOLD	500
#define APDS_PROXIITY_LOW_THRESHHOLD	450

#define APDS9190_STATUS_PINT	0x20
#define APDS9190_STATUS_AINT	0x10 // ALS Interrupt STATUS ALS Non Use

#define PROX_SENSOR_DETECT_N	(0)

enum apds9190_input_event {
	PROX_INPUT_NEAR = 0,
	PROX_INPUT_FAR,
};


#define APDS900_SENSOR_DEBUG 0
 #if APDS900_SENSOR_DEBUG
 #define DEBUG_MSG(args...)  printk(args)
 #else
 #define DEBUG_MSG(args...)
 #endif


/*
 * Structs
 */

struct apds9190_data {
	struct i2c_client *client;
	struct mutex update_lock;

	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;

	unsigned int lth_prox;
	unsigned int hth_prox;
	unsigned int pDrive;
	
	unsigned int GA;
	unsigned int DF;
	unsigned int LPC;
	
	int irq;
	unsigned int isNear;
	unsigned int last_isNear;
	unsigned int sw_mode;
	spinlock_t lock;
	struct input_dev *input_dev;
	struct delayed_work dwork;
};
 
enum apds9190_dev_status {
	PROX_STAT_SHUTDOWN = 0,
	PROX_STAT_OPERATING,
};

static struct i2c_client *apds_9190_i2c_client = NULL;
static struct workqueue_struct *proximity_wq = NULL;
static int apds_9190_initlizatied = 0;
static int enable_status = 0;
static int methods = -1;

static int apds9190_set_command(struct i2c_client *client, int command)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;
		
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clearInt);
	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds9190_set_enable(struct i2c_client *client, int enable)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ENABLE_REG, enable);
	mutex_unlock(&data->update_lock);

	DEBUG_MSG("apds9190_set_enable = [%x] \n",enable);
	
	data->enable = enable;

	return ret;
}

static int apds9190_set_atime(struct i2c_client *client, int atime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}
static int apds9190_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds9190_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds9190_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_AILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->ailt = threshold;

	return ret;
}

static int apds9190_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_AIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->aiht = threshold;

	return ret;
}

static int apds9190_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_PILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->pilt = threshold;

	return ret;
}

static int apds9190_set_piht(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_PIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	
	data->piht = threshold;

	return ret;
}

static int apds9190_set_pers(struct i2c_client *client, int pers)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds9190_set_config(struct i2c_client *client, int config)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds9190_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PPCOUNT_REG, ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static int apds9190_set_control(struct i2c_client *client, int control)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;
	
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_CONTROL_REG, control);
	mutex_unlock(&data->update_lock);

	data->control = control;
	return ret;
}

static void apds_9190_initialize(void)
{
	struct apds9190_data *data = i2c_get_clientdata(apds_9190_i2c_client);
	u8 enable;

	data->pDrive = PDRIVE;
	
	enable = APDS9190_ENABLE_PIEN | APDS9190_ENABLE_AIEN | APDS9190_ENABLE_WEN | APDS9190_ENABLE_PEN | 
			APDS9190_ENABLE_AEN | APDS9190_ENABLE_PON;

	apds9190_set_enable(apds_9190_i2c_client,enable);

	apds9190_set_wtime(apds_9190_i2c_client, WTIME);
	apds9190_set_ptime(apds_9190_i2c_client, PTIME);

	apds9190_set_pers(apds_9190_i2c_client, 0x36); //Interrupt persistence

	apds9190_set_config(apds_9190_i2c_client, 0x00); // Wait long timer <- no needs so set 0
		
	apds9190_set_ppcount(apds_9190_i2c_client, PPCOUNT); // Pulse count for proximity
	
	apds9190_set_control(apds_9190_i2c_client, data->pDrive| PDIODE | PGAIN | AGAIN);

	apds9190_set_pilt(apds_9190_i2c_client, 1023); // init threshold for proximity
	apds9190_set_piht(apds_9190_i2c_client, 0);

	enable_status = enable;

	data->lth_prox = APDS_PROXIITY_LOW_THRESHHOLD;
	data->hth_prox = APDS_PROXIITY_HIGH_THRESHHOLD;
	data->pilt = APDS_PROXIITY_LOW_THRESHHOLD;
	data->piht = APDS_PROXIITY_HIGH_THRESHHOLD;
}


/*
 * SysFS support
 */
 
static ssize_t apds9190_status_show(struct device *dev, 
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	if(client != NULL)
		return sprintf(buf, "%d\n",data->last_isNear);
	else
		return -1;
}
static DEVICE_ATTR(show, S_IWUSR, apds9190_status_show, NULL);	

static ssize_t
apds9190_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->sw_mode);
}

static int apds9190_suspend(struct i2c_client *i2c_dev, pm_message_t state);
static int apds9190_resume(struct i2c_client *i2c_dev);


static ssize_t
apds9190_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *pdev = i2c_get_clientdata(client);
	pm_message_t dummy_state;
	int mode;

	dummy_state.event = 0;

	sscanf(buf, "%d", &mode);

	if ((mode != PROX_STAT_SHUTDOWN) && (mode != PROX_STAT_OPERATING)) {
		printk(KERN_INFO "Usage: echo [0 | 1] > enable");
		printk(KERN_INFO " 0: disable\n");
		printk(KERN_INFO " 1: enable\n");
		return count;
	}

	if (mode == pdev->sw_mode) {
		printk(KERN_INFO "mode is already %d\n", pdev->sw_mode);
		apds9190_suspend(client, dummy_state);
		apds9190_resume(client);
		return count;
	}

	if (mode) {
		apds9190_resume(client);
		printk(KERN_INFO "Power On Enable\n");
	}
	else {
		apds9190_suspend(client, dummy_state);
		printk(KERN_INFO "Power Off Disable\n");
	}

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR, apds9190_enable_show, apds9190_enable_store);	


static ssize_t apds9190_show_ptime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	if(client != NULL)
		return sprintf(buf, "%d\n",data->ptime);
	else
		return -1;
}

static ssize_t apds9190_store_ptime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	if(client != NULL)
		apds9190_set_ptime(client,rdata);
	else
		return -1;

	return count;
}

static DEVICE_ATTR(ptime, S_IWUSR, apds9190_show_ptime, apds9190_store_ptime);	

static ssize_t apds9190_show_wtime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	if(client != NULL)
		return sprintf(buf, "%d\n",data->wtime);
	else
		return -1;
}

static ssize_t apds9190_store_wtime(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	if(client != NULL)
		apds9190_set_wtime(client,rdata);
	else
		return -1;

	return count;
}

static DEVICE_ATTR(wtime, S_IWUSR, apds9190_show_wtime, apds9190_store_wtime);	

static ssize_t apds9190_show_ppcount(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	if(client != NULL)
		return sprintf(buf, "%d\n",data->ppcount);
	else
		return -1;
}

static ssize_t apds9190_store_ppcount(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);
		
	if(client != NULL)
		apds9190_set_ppcount(client,rdata);
	else
		return -1;

	return count;
}

static DEVICE_ATTR(ppcount, S_IWUSR, apds9190_show_ppcount, apds9190_store_ppcount);	

static ssize_t apds9190_show_pers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	if(client != NULL)
		return sprintf(buf, "%d\n",data->pers);
	else
		return -1;
}

static ssize_t apds9190_store_pers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	

	if(client != NULL)
		apds9190_set_pers(client,rdata);
	else
		return -1;

	return count;
}

static DEVICE_ATTR(pers, S_IWUSR, apds9190_show_pers, apds9190_store_pers);

static ssize_t apds9190_show_pilt(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	if(client != NULL)
		return sprintf(buf, "%d\n",data->lth_prox);
	else
		return -1;
}

static ssize_t apds9190_store_pilt(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);

	if(client != NULL)
		data->lth_prox = rdata;
	else
		return -1;

	return count;
}

static DEVICE_ATTR(pilt, S_IWUSR, apds9190_show_pilt, apds9190_store_pilt);	


static ssize_t apds9190_show_piht(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	if(client != NULL)
		return sprintf(buf, "%d\n",data->hth_prox);
	else
		return -1;
}

static ssize_t apds9190_store_piht(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);

	if(client != NULL)
		data->hth_prox = rdata;
	else
		return -1;

	return count;
}

static DEVICE_ATTR(piht, S_IWUSR, apds9190_show_piht, apds9190_store_piht);	


static ssize_t apds9190_show_pdata(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	if(client != NULL){
		int pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PDATAL_REG);		
	
		return sprintf(buf, "%d\n",pdata);
	}
	else{
		return -1;
	}
}

static DEVICE_ATTR(pdata, S_IWUSR, apds9190_show_pdata, NULL);	


static ssize_t apds9190_show_interrupt(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",enable_status);
}

static ssize_t apds9190_store_interrupt(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
// this value should be same with the value in sensors.cpp
#define STORE_INTERUPT_SELECT_PROXIMITY		0x02


	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);	
	int enable = (int)rdata;
	int val = 0;
	int ret;

	DEBUG_MSG("apds9190_store_interrupt = [%d] apds_9190_initlizatied [%d] \n",rdata, apds_9190_initlizatied);

	if(!apds_9190_initlizatied){
		apds_9190_initialize();
		apds_9190_initlizatied = 1;		
		enable_irq(data->irq);
	}

	disable_irq_nosync(data->irq);

	if(enable & STORE_INTERUPT_SELECT_PROXIMITY)
	{	
		if(enable & 0x01) // enable
		{
			data->enable |= (APDS9190_ENABLE_PIEN|APDS9190_ENABLE_PEN|APDS9190_ENABLE_PON); 	
		}
		else		//disable
		{
			data->enable &= ~(APDS9190_ENABLE_PIEN|APDS9190_ENABLE_PEN); 	
		}
	}

	if(data->enable == 1)
	{
		data->enable = 0;
	}

	ret = apds9190_set_enable(client, data->enable);

	enable_status = data->enable;

	enable_irq(data->irq);

	DEBUG_MSG("apds9190_store_interrupt enable_status = [%x] data->enable [%x] \n",enable_status, data->enable);

	if (ret < 0)
		return ret;
	
	return count;
}

static DEVICE_ATTR(interrupt, 0664,
		   apds9190_show_interrupt, apds9190_store_interrupt);		   


static ssize_t apds9190_show_pdrive(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);

	if(client != NULL)
		return sprintf(buf, "%d\n",data->pDrive);

	else
		return -1;
}

static ssize_t apds9190_store_pdrive(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9190_data *data = i2c_get_clientdata(client);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);

	if(client != NULL){
		data->pDrive= rdata;
		apds9190_set_control(client,(data->pDrive | PDIODE | PGAIN | AGAIN));
	}
	else
		return -1;

	return count;
}

static DEVICE_ATTR(pdrive, S_IWUSR, apds9190_show_pdrive, apds9190_store_pdrive);	



static struct attribute *apds9190_attributes[] = {
	&dev_attr_show.attr,
	&dev_attr_enable.attr,
	&dev_attr_ptime.attr,
	&dev_attr_wtime.attr,
	&dev_attr_ppcount.attr,
	&dev_attr_pers.attr,
	&dev_attr_pilt.attr,
	&dev_attr_piht.attr,
	&dev_attr_pdata.attr,
	&dev_attr_interrupt.attr,
	&dev_attr_pdrive.attr,
	NULL
};

static const struct attribute_group apds9190_attr_group = {
	.attrs = apds9190_attributes,
};

/*
 * Initialization function
 */

static int apds9190_init_client(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(apds_9190_i2c_client);
	int err;

	apds9190_set_enable(apds_9190_i2c_client, 0);

	mdelay(1);

	mutex_lock(&data->update_lock);
	err = i2c_smbus_read_byte_data(apds_9190_i2c_client, APDS9190_ENABLE_REG);
	mutex_unlock(&data->update_lock);

	if (err != 0)
		return -ENODEV;

	DEBUG_MSG("apds9190_init_client\n");

	data->enable = 0;

	return 0;
}

static void apds9190_event_report(int state)
{
	int input_state;
	struct apds9190_data *data = i2c_get_clientdata(apds_9190_i2c_client);	 

	input_report_abs(data->input_dev, ABS_DISTANCE, state);
	input_sync(data->input_dev);

	data->last_isNear = data->isNear;
}

void apds_9190_proximity_handler(struct apds9190_data *data) 	
{
	int pdata = i2c_smbus_read_word_data(apds_9190_i2c_client, CMD_WORD|APDS9190_PDATAL_REG);		
		
	if(pdata > data->hth_prox){
		apds9190_set_enable(apds_9190_i2c_client,0);
		data->isNear = 0;
		apds9190_set_pilt(apds_9190_i2c_client, data->lth_prox);
		apds9190_set_piht(apds_9190_i2c_client, 0);
		printk(KERN_INFO "prox sensor report NEAR\n");
	}
	else if(pdata < data->lth_prox)	{
		apds9190_set_enable(apds_9190_i2c_client,0);		
		data->isNear = 1;
		apds9190_set_pilt(apds_9190_i2c_client, 0);
		apds9190_set_piht(apds_9190_i2c_client, data->hth_prox);
		printk(KERN_INFO "prox sensor report FAR\n");
	}

	if(data->isNear != data->last_isNear){
		apds9190_event_report(data->isNear);
		data->last_isNear = data->isNear;
	}

}


#define PROX_LOW_TH		(0x6F) // far threshold
#define PROX_HIGH_TH	(0x70) // near threshold

void apds_9190_irq_work_func(struct work_struct *work) 	
{
   	struct apds9190_data *data = 
		container_of(work, struct apds9190_data, dwork.work);
	struct proximity_platform_data		*pdev = NULL;	
	int status, rdata;
	int org_enable = data->enable;
	int pdata, cdata, irdata;	
	int enable;
	
	pdev = data->client->dev.platform_data;

	if(NULL == pdev){
		printk(KERN_INFO "Platform data is NULL\n");
		return -1;
	}

	disable_irq_nosync(data->irq);
	
	if(pdev->methods){
		status = i2c_smbus_read_byte_data(apds_9190_i2c_client, CMD_BYTE|APDS9190_STATUS_REG);

		if(status & APDS9190_STATUS_PINT)
			apds_9190_proximity_handler(data);
		
		// ACK about interupt handling
		if(status & APDS9190_STATUS_PINT)
			apds9190_set_command(apds_9190_i2c_client,0);
		
	}
	else{
		mutex_lock(&data->update_lock);
	
		pdata = i2c_smbus_read_word_data(data->client, CMD_WORD|APDS9190_PDATAL_REG);

		i2c_smbus_write_byte(data->client, CMD_CLR_PS_INT);
		mutex_unlock(&data->update_lock);

		
		if(pdata > PROX_HIGH_TH) // near intr
			data->isNear = 0; 
		else
			data->isNear = 1; // far intr,  pdata<= PROX_INT_LOW_TH
		

		if(data->isNear != data->last_isNear)
			apds9190_event_report(data->isNear);

	}
		
	data->enable = org_enable;
		
	apds9190_set_control(apds_9190_i2c_client, PDRIVE | PDIODE | PGAIN | AGAIN);
	apds9190_set_enable(apds_9190_i2c_client,org_enable);
	enable_irq(data->irq);
	
}

static irqreturn_t apds_9190_irq_handler(int irq, void *dev_id)						   
{
 	struct apds9190_data *data = dev_id;
	struct proximity_platform_data		*pdata;	
	unsigned long delay;
	int status;
	
	pdata = data->client->dev.platform_data;
	spin_lock(&data->lock);
	
	delay = msecs_to_jiffies(pdata->irq_num);
	queue_delayed_work(proximity_wq, &data->dwork, delay);
	spin_unlock(&data->lock);
	return IRQ_HANDLED;
}	


/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver apds9190_driver;
static int __devinit apds9190_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9190_data *data;
	struct proximity_platform_data		*pdata;
	pm_message_t dummy_state;
	int err = 0;
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct apds9190_data), GFP_KERNEL);
	apds_9190_i2c_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	memset(data, 0x00, sizeof(struct apds9190_data));

	INIT_DELAYED_WORK(&data->dwork, apds_9190_irq_work_func);


	data->client = client;
	apds_9190_i2c_client = client;
	i2c_set_clientdata(data->client, data);
	
	data->input_dev = input_allocate_device();		

	data->input_dev->name = "proximity";
//	data->input_dev->phys = "proximity/input2";	
	set_bit(EV_ABS, data->input_dev->evbit);
	set_bit(EV_SYN, data->input_dev->evbit);
	input_set_abs_params(data->input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	
	err = input_register_device(data->input_dev);
	
	if (err) {
		DEBUG_MSG("Unable to register input device: %s\n",
		       data->input_dev->name);
		goto exit_input_register_device_failed;
	}
	
	pdata = data->client->dev.platform_data;
	if(NULL == pdata){
		printk(KERN_INFO "platform data is NULL");
		return -1;
	}

	methods = pdata->methods;
	
	data->irq = gpio_to_irq(pdata->irq_num);
	
	spin_lock_init(&data->lock);
	mutex_init(&data->update_lock);
	
	data->enable = 0;	/* default mode is standard */
	dev_info(&client->dev, "enable = %s\n", data->enable ? "1" : "0");

	pdata->power(1);
	mdelay(13);
	
	/* Initialize the APDS9190 chip */
	err = apds9190_init_client(apds_9190_i2c_client);
	apds_9190_initialize();	

	enable_irq(data->irq);

	if(request_irq(data->irq,apds_9190_irq_handler,IRQF_TRIGGER_FALLING,"proximity_irq", data) < 0){
		err = -EIO;
		goto exit_request_irq_failed;
	}
	
	data->sw_mode = PROX_STAT_OPERATING;

	err = set_irq_wake(data->irq,1);
	if(err)
		set_irq_wake(data->irq,0);


	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &apds9190_attr_group);
	if (err)
		goto exit_kfree;

	dummy_state.event = 0;
	apds9190_suspend(data->client, dummy_state);


	return 0;
exit_input_register_device_failed:	
exit_request_irq_failed:
exit_kfree:
	dev_info(&client->dev, "probe error\n");
	kfree(data);
exit:
	return err;
}

static int __devexit apds9190_remove(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);

	DEBUG_MSG("apds9190_remove\n");

	apds9190_set_enable(client, 0);

	set_irq_wake(data->irq, 0);
	free_irq(data->irq, NULL);
	input_unregister_device(data->input_dev);
	input_free_device(data->input_dev);

	kfree(data);		
	/* Power down the device */

	sysfs_remove_group(&client->dev.kobj, &apds9190_attr_group);

	return 0;
}

static int apds9190_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds9190_data *data = i2c_get_clientdata(apds_9190_i2c_client);	
	struct proximity_platform_data* pdata = NULL;
	int enable;
	
	printk("apds9190_suspend [%d]\n", data->enable);

	if(!data->sw_mode)
		return 0;

	pdata = data->client->dev.platform_data;

	if(NULL == pdata){
		printk(KERN_INFO "Platform data is NULL\n");
		return -1;
	}
	
	disable_irq_nosync(data->irq);

		
	if(!data->enable){
		enable =  APDS9190_ENABLE_PIEN | APDS9190_ENABLE_AIEN | APDS9190_ENABLE_WEN | APDS9190_ENABLE_PEN | 
					APDS9190_ENABLE_AEN | APDS9190_ENABLE_PON;
		apds9190_set_enable(client, enable);
	}
	else
		apds9190_set_enable(client, 0);

	cancel_delayed_work_sync(&data->dwork);
	flush_workqueue(proximity_wq);

	enable_status = enable;
	data->sw_mode = PROX_STAT_SHUTDOWN;
	
	pdata->power(0);
	set_irq_wake(data->irq, 0);
	
	return 0;
}

static int apds9190_resume(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(apds_9190_i2c_client);	
	struct proximity_platform_data* pdata = NULL;
	int ret;

	pdata = data->client->dev.platform_data;

	if(NULL == pdata){
		printk(KERN_INFO "Platform data is NULL");
		return -1;
	}
	
	printk("apds9190_resume [%d]\n",enable_status);
	if(data->sw_mode)
		return 0;
	

	pdata->power(1);

	apds_9190_initialize();

	mdelay(33);

	apds9190_event_report(PROX_SENSOR_DETECT_N);
	data->last_isNear = -1;

	enable_irq(data->irq);

	data->sw_mode = PROX_STAT_OPERATING;

	ret = set_irq_wake(data->irq, 1);
	if(ret)
		set_irq_wake(data->irq, 0);

	return 0;
}


static const struct i2c_device_id apds9190_id[] = {
	{ "proximity_apds9190", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds9190_id);

static struct i2c_driver apds9190_driver = {
	.driver = {
		.name	= APDS9190_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = apds9190_suspend,
	.resume	= apds9190_resume,
	.probe	= apds9190_probe,
	.remove	= __devexit_p(apds9190_remove),
	.id_table = apds9190_id,
};

static int __init apds9190_init(void)
{
	int err;
	proximity_wq = create_singlethread_workqueue("proximity_wq");

	if(NULL == proximity_wq)
		return -ENOMEM;
	err = i2c_add_driver(&apds9190_driver);
	if(err < 0){
		printk(KERN_INFO "Failed to i2c_add_driver \n");
		return err;
	}
	return 0;
}

static void __exit apds9190_exit(void)
{
	i2c_del_driver(&apds9190_driver);
	if(proximity_wq)
		destroy_workqueue(proximity_wq);		
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS9190 ambient proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9190_init);
module_exit(apds9190_exit);


