/* drivers/i2c/chips/cap_sense_express.c
 *
 * Copyright (C) 2010 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/msm_iomap.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/cap_sense.h>

#include "../../../arch/arm/mach-msm/proc_comm.h"

#define I2C_RETRY_COUNT 10

static struct workqueue_struct *cs_express_wq;
static int cs_express_thd;

struct cs_express_data {
	struct early_suspend	early_suspend;
	struct class *capsense_class;
	struct device *capsense_dev;
	int intr_gpio;
	int intr_irq;
	struct work_struct work;
	unsigned short tsa;
	uint8_t use_dsa; /* try dsa if tsa no response */
};

static struct i2c_client *this_client;

static int I2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		SAR_LOGE("%s retry over %d\n",
				__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		SAR_LOGE("%s retry over %d\n",
				__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int cse_set_threshold(uint32_t data)
{
	uint8_t reg[2];
	int err = 0;

	reg[0] = CSE_FINGER_TH_2;
	reg[1] = (data >> 8) & 0xFF;
	err = I2C_TxData(reg, 2);
	if (err) {
		SAR_LOGE("%s: I2C write fail. reg %d, err %d\n",
					__func__, CSE_FINGER_TH_2, err);
		return err;
	}

	cs_express_thd = data;
	return err;
}

static int cse_get_threshold(uint32_t *data)
{
	uint8_t reg = 0;
	int err = 0;

	reg = CSE_FINGER_TH_2;
	I2C_RxData(&reg, sizeof(reg));
	if (err) {
		SAR_LOGE("%s: I2C read fail. reg %d, err %d\n",
					__func__, CSE_FINGER_TH_2, err);
		return 0;
	}

	*data = reg;
	SAR_LOGD("%s thresold = 0x%x\n", __func__, *data);

	return err;
}

#define CSE_SIGNATURE 0x534152 /*"SAR" in ASCII*/
static int cse_set_kadc(void)
{
	uint32_t thd = 0;
	int err = 0;

	SAR_LOGI("csa_kvalue = (0x%x, 0x%x, 0x%x)\n",
		csa_kvalue1, csa_kvalue2, csa_kvalue3);

	if ((csa_kvalue3 & 0xFFFFFF) != CSE_SIGNATURE)
		return -EINVAL;

	thd = csa_kvalue2 & 0xFFFF;
	err = cse_set_threshold(thd);

	return err;
}

static int cse_get_devid(uint8_t *data)
{
	int err = 0;

	*data = CSE_DEVICE_ID;
	err = I2C_RxData(data, 1);
	return err;
}

static int cse_load_config(void)
{
	uint8_t CS_SETUP_MODE[] = {0xA0, 0x08};
	uint8_t CS_RELOAD_CONF[] = {0xA0, 0x06};
	uint8_t CS_DEF_CONFIG[] = {
		0xA0,0x03,0x00,0x00,0x04,0x00,0x02,0x00,0x00,0x00,
		0x00,0x00,0x06,0x00,0x00,0x02,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x82,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x64,0xA0,0x42,
		0x0A,0x03,0x14,0x14,0x20,0xFF,0xFF,0x00,0xFF,0xFF,
		0xFF,0xFF,0xFF,0xFF,0xFF,0x64,0x64,0x32,0x64,0x64,
		0x64,0x64,0x64,0x64,0x64,0x0A,0x0A,0x0A,0x0A,0x0A,
		0x0A,0x0A,0x0A,0x0A,0x0A,0x00,0x00,0x00,0x00,0xA0,
		0x23,0x00,0x20,0x00,0x09
	};
	int CONFIG_SIZE = ARRAY_SIZE(CS_DEF_CONFIG);
	int err = 0;

	SAR_LOGD("config[0]=0x%x, config[1]=0x%x, size=%d\n",
		CS_DEF_CONFIG[0], CS_DEF_CONFIG[1], CONFIG_SIZE);
	err = I2C_TxData(CS_DEF_CONFIG, CONFIG_SIZE);
	if (err)
		return err;

	msleep(400);
	err = I2C_TxData(CS_SETUP_MODE, 2);
	if (err)
		return err;
	msleep(5);
	err = I2C_TxData(CS_RELOAD_CONF, 2);
	if (err == 0)
		msleep(600);

	return err;
}

static void capsense_early_suspend(struct early_suspend *handler)
{
}

static void capsense_late_resume(struct early_suspend *handler)
{
}

static irqreturn_t cap_sense_irq_handler(int irq, void *data)
{
	struct cs_express_data *cd = (struct cs_express_data *)data;
	SAR_LOGD("%s in\n", __func__);

	queue_work(cs_express_wq, &cd->work);
	return IRQ_HANDLED;
}

static void capsense_work_func(struct work_struct *work)
{
	uint8_t reg = CSE_INPUT_PORT0;
	unsigned data = 0;
	int err = 0;

	err = I2C_RxData(&reg, 1);
	if (err) {
		SAR_LOGE("%s: I2C read fail. err %d\n",
						__func__, err);
		return;
	}

	data = (reg & (0x1 << 2)) ? 1 : 0;
	SAR_LOGI("%s: btn=%d, data=%d\n", __func__, reg, data);
	err = msm_proc_comm(PCOM_CUSTOMER_CMD1, &data, NULL);
	if (err)
		SAR_LOGE("%s: sent proc_comm fail, err=%d\n",
							__func__, err);
}

static ssize_t cse_reg_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	uint32_t data[2];
	uint8_t reg[2];
	int err = 0;

	sscanf(buf, "0x%x 0x%x", &data[0], &data[1]);
	SAR_LOGI("%s: reg=0x%x, value=0x%x\n", __func__, data[0], data[1]);
	reg[0] = (uint8_t)data[0];
	reg[1] = (uint8_t)data[1];
	err = I2C_TxData(reg, 2);
	if (err)
		SAR_LOGE("%s: I2C write fail. err %d\n",
						__func__, err);

	return count;
}
static ssize_t cse_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	char log[100];
	int i, p = 0;
	uint8_t data = 0;
	int err = 0;
	uint8_t reg_table[] = {0x00, 0x7A, 0x6D, 0x53, 0x7F,
				0x63, 0x63, 0x80, 0x50, 0x88,
				0x81, 0x82, 0x83, 0x86, 0x87,
				0x84, 0x85};
	int table_size = ARRAY_SIZE(reg_table);

	p = sprintf(log+p, "(");
	for (i = 0; i <= table_size ; i++) {
		data = reg_table[i];
		I2C_RxData(&data, sizeof(data));
		if (err) {
			SAR_LOGE("%s: I2C read fail. reg %d, err %d\n",
							__func__, i, err);
			return 0;
		}
		p += sprintf(log+p, "0x%x", data);
		if (i == table_size)
			p += sprintf(log+p, ")");
		else
			p += sprintf(log+p, ",");
	}
	return sprintf(buf, "%s\n", log);
}
static DEVICE_ATTR(reg, 0664, cse_reg_show, cse_reg_store);

static ssize_t cse_kadc_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	uint32_t data;
	int err = 0;

	sscanf(buf, "0x%x", &data);
	SAR_LOGI("%s: set kadc=0x%x\n", __func__, data);

	err = cse_set_threshold(data);
	if (err) {
		SAR_LOGE("Fail to set threshold\n");
		return 0;
	}
	return count;
}
static ssize_t cse_kadc_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint32_t thd = 0;
	int err = 0;

	err = cse_get_threshold(&thd);
	if (err) {
		SAR_LOGE("Fail to get threshold\n");
		return 0;
	}
	if (cs_express_thd != thd)
		cs_express_thd = thd;

	SAR_LOGI("%s: cs_express_thd = 0x%x\n", __func__, cs_express_thd);
	return sprintf(buf, "0x%x\n", cs_express_thd);
}
static DEVICE_ATTR(kadc, 0664, cse_kadc_show, cse_kadc_store);

static int cs_express_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct cs_express_data *cs;
	struct capsense_platform_data *pdata;
	int err = 0;
	uint8_t devID = 0;

	SAR_LOGD("%s: in\n", __func__);
	pdata = client->dev.platform_data;
	if (!pdata) {
		SAR_LOGE("Assign platform_data error!!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SAR_LOGE("check i2c fail !!!\n");
		err = -ENODEV;
		goto check_functionality_failed;
	}

	cs = kzalloc(sizeof(struct cs_express_data), GFP_KERNEL);
	if (!cs) {
		SAR_LOGE("kzalloc fail !!!\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, cs);
	this_client = client;

	do {
		if (cse_get_devid(&devID)) {
			if (cs->use_dsa) {
				SAR_LOGE("check device ID fail,"
							" err %d\n", err);
				goto check_id_failed;
			}
			SAR_LOGI("try default slave address\n");
			cs->tsa = this_client->addr;
			this_client->addr = CS_EXPRESS_DSA;
			cs->use_dsa = 1;
		} else {
			SAR_LOGI("%s: device ID = 0x%x\n", __func__, devID);
			cs->use_dsa = 0;
		}

		/* check if device has no default configuration */
		if (cs->use_dsa) {
			err = cse_get_devid(&devID);
			if (err) {
				SAR_LOGE("check device ID fail,"
							" err %d\n", err);
				goto check_id_failed;
			}
			err = cse_load_config();
			if (err) {
				SAR_LOGE("check device ID fail,"
							" err %d\n", err);
				goto check_id_failed;
			} else {
				SAR_LOGI("restore slave address\n");
				this_client->addr = cs->tsa;
			}
		}
	} while (cs->use_dsa);

	if (cse_set_kadc() != 0)
		SAR_LOGI("%s: device has no proper kadc\n", __func__);

	cs_express_wq = create_singlethread_workqueue("cs_express_wq");
	INIT_WORK(&cs->work, capsense_work_func);

	if (pdata->gpio_init)
		pdata->gpio_init();
	if (pdata->intr) {
		err = gpio_request(pdata->intr, "cse_intr");
		if (err < 0) {
			SAR_LOGE("gpio_request(intr) fail,"
				"err %d\n", err);
			goto request_gpio_failed;
		}
		cs->intr_gpio = pdata->intr;
		cs->intr_irq = gpio_to_irq(cs->intr_gpio);
		err = request_irq(cs->intr_irq, cap_sense_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"cse_irq", cs);
		if (err) {
			SAR_LOGE("request_irq failed for gpio %d,"
			" irq %d\n", cs->intr_gpio, cs->intr_irq);
			goto request_irq_failed;
		}
	}

	if (!cancel_work_sync(&cs->work))
		queue_work(cs_express_wq, &cs->work);

#ifdef CONFIG_HAS_EARLYSUSPEND
	cs->early_suspend.suspend = capsense_early_suspend;
	cs->early_suspend.resume  = capsense_late_resume;
	register_early_suspend(&cs->early_suspend);
#endif

	cs->capsense_class = class_create(THIS_MODULE, "cap_sense");
	if (IS_ERR(cs->capsense_class)) {
		err = PTR_ERR(cs->capsense_class);
		cs->capsense_class = NULL;
		SAR_LOGE("%s:fail to create class, err=%d\n", __func__, err);
	}

	cs->capsense_dev = device_create(cs->capsense_class,
				NULL, 0, "%s", "sar");
	if (unlikely(IS_ERR(cs->capsense_dev))) {
		err = PTR_ERR(cs->capsense_dev);
		cs->capsense_dev = NULL;
		SAR_LOGE("%s:fail to create device, err=%d\n", __func__, err);
	}
	err = device_create_file(cs->capsense_dev, &dev_attr_reg);
	if (err)
		SAR_LOGE("%s:fail to create attr_reg, err=%d\n", __func__, err);
	err = device_create_file(cs->capsense_dev, &dev_attr_kadc);
	if (err)
		SAR_LOGE("%s:fail to create attr_kadc, err=%d\n", __func__, err);

	return 0;

request_irq_failed:
	gpio_free(cs->intr_gpio);
request_gpio_failed:
check_id_failed:
	kfree(cs);
check_functionality_failed:
	return err;
}

static int cs_express_remove(struct i2c_client *client)
{
	struct cs_express_data *cs = i2c_get_clientdata(client);

	device_remove_file(cs->capsense_dev, &dev_attr_reg);
	kfree(cs);

	SAR_LOGI("%s in\n", __func__);
	return 0;
}

static const struct i2c_device_id cs_express_id[] = {
	{ CS_EXPRESS_NAME, 0 },
};

static struct i2c_driver cs_express_driver = {
	.probe		= cs_express_probe,
	.remove		= cs_express_remove,
	.id_table	= cs_express_id,
	.driver		= {
		.name = CS_EXPRESS_NAME,
	},
};

static int __init cs_express_init(void)
{
	SAR_LOGI("CapSense Express driver: init\n");
	return i2c_add_driver(&cs_express_driver);
}

static void __exit cs_express_exit(void)
{
	i2c_del_driver(&cs_express_driver);
}

module_init(cs_express_init);
module_exit(cs_express_exit);

MODULE_DESCRIPTION("capsense express driver");
MODULE_LICENSE("GPL");
