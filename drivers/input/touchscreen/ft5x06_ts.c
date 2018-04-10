/*
 *   Boundary Devices FTx06 touch screen controller.
 *
 *   Copyright (c) by Boundary Devices <info@boundarydevices.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/input/mt.h>

#define CONFIG_TOUCHSCREEN_FT5X06_SINGLE_TOUCH

#ifdef CONFIG_TOUCHSCREEN_FT5X06_SINGLE_TOUCH
#define USE_ABS_SINGLE
#else
#define USE_ABS_SINGLE
#define USE_ABS_MT
#endif
// For 5"
//#define FT5206
//#define FT5416

// For 7"
#define FT5306
//#define FT5306_800480

// For 10"  FT5X06 == FT5506
//#define FT5X06
//#define FT5526

//#define DEBUG

#define WORK_MODE	0
#define FACTORY_MODE	4

static int calibration[7] = {
	65536,0,0,
	0,65536,0,
	0
};
module_param_array(calibration, int, NULL, S_IRUGO | S_IWUSR);

static int screenres[2];
module_param_array(screenres, int, NULL, S_IRUGO | S_IWUSR);

#if defined(FT5416) || defined(FT5306_800480)
#define MAX_TOUCHES 7
#else
#define MAX_TOUCHES 12
#endif

static void translate(int *px, int *py)
{
	int x, y, x1, y1;
	if (calibration[6]) {
		x1 = *px;
		y1 = *py;

		x = calibration[0] * x1 +
			calibration[1] * y1 +
			calibration[2];
		x /= calibration[6];
		if (x < 0)
			x = 0;
		y = calibration[3] * x1 +
			calibration[4] * y1 +
			calibration[5];
		y /= calibration[6];
		if (y < 0)
			y = 0;
		*px = x ;
		*py = y ;
	}
}

struct point {
	int	x;
	int	y;
	int	id;
};

struct ft5x06_ts {
	struct i2c_client *client;
	struct input_dev	*idev;
	int			use_count;
	int			bReady;
	int			irq;
	struct gpio_desc	*wakeup_gpio;
	struct gpio_desc	*reset_gpio;
	struct proc_dir_entry  *procentry;
	unsigned		down_mask;
	unsigned		max_x;
	unsigned		max_y;
};
static const char *client_name = "ft5x06";

struct ft5x06_ts *gts;

static char const procentryname[] = {
   "ft5x06"
};

static int ts_startup(struct ft5x06_ts *ts);
static void ts_shutdown(struct ft5x06_ts *ts);

/*-----------------------------------------------------------------------*/
static void write_reg(struct ft5x06_ts *ts, int regnum, int value)
{
	u8 regnval[] = {
		regnum,
		value
	};
	struct i2c_msg pkt = {
		ts->client->addr, 0, sizeof(regnval), regnval
	};
	int ret = i2c_transfer(ts->client->adapter, &pkt, 1);
	if (ret != 1)
		printk(KERN_WARNING "%s: i2c_transfer failed\n", __func__);
	else
		printk(KERN_DEBUG "%s: set register 0x%02x to 0x%02x\n",
		       __func__, regnum, value);
}

static void set_mode(struct ft5x06_ts *ts, int mode)
{
	write_reg(ts, 0, (mode&7)<<4);
	printk(KERN_DEBUG "%s: changed mode to 0x%02x\n", __func__, mode);
}

static void release_slots(struct ft5x06_ts *ts, unsigned mask)
{
	struct input_dev *idev = ts->idev;

	while (mask) {
		int slot = __ffs(mask);

		mask &= ~(1 << slot);
		input_mt_slot(idev, slot);
		input_mt_report_slot_state(idev,  MT_TOOL_FINGER, 0);
	}
}

static inline void ts_evt_add(struct ft5x06_ts *ts,
			      unsigned buttons, struct point *p)
{
	struct input_dev *idev = ts->idev;
	unsigned down_mask = 0;
	unsigned tmp;
	int i;
	if (!buttons) {
		/* send release to user space. */
#ifdef USE_ABS_MT
		tmp = ts->down_mask;
		ts->down_mask = 0;
		release_slots(ts, tmp);
#endif
#ifdef USE_ABS_SINGLE
		input_report_abs(idev, ABS_PRESSURE, 0);
		input_report_key(idev, BTN_TOUCH, 0);
#endif
	} else {
#ifdef USE_ABS_MT
		for (i = 0; i < buttons; i++) {
			translate(&p[i].x, &p[i].y);
			input_mt_slot(idev, p[i].id);
			input_mt_report_slot_state(idev,  MT_TOOL_FINGER, 1);
			down_mask |= 1 << p[i].id;
#ifdef FT5206
			input_report_abs(idev, ABS_MT_POSITION_X, ts->max_x - p[i].y); //p[i].x	leavs patched for FT5206
			input_report_abs(idev, ABS_MT_POSITION_Y, ts->max_y - p[i].x); //p[i].y	leavs patched for FT5206
#endif
#ifdef FT5306
			input_report_abs(idev, ABS_MT_POSITION_X, ts->max_x - p[i].y); //p[i].x	leavs patched for FT5306
			input_report_abs(idev, ABS_MT_POSITION_Y, ts->max_y - p[i].x*600/768); //p[i].y leavs patched for FT5306
#endif
#ifdef FT5306_800480
			input_report_abs(idev, ABS_MT_POSITION_X, ts->max_x - p[i].y*800/1024);
			input_report_abs(idev, ABS_MT_POSITION_Y, ts->max_y - p[i].x*480/768); 
#endif
#ifdef FT5416
			input_report_abs(idev, ABS_MT_POSITION_X, p[i].x);
			input_report_abs(idev, ABS_MT_POSITION_Y, p[i].y); 
#endif
#ifdef FT5526
			input_report_abs(idev, ABS_MT_POSITION_X, p[i].x * (ts->max_x+1)/1024);//p[i].x leavs patched for FT5526
			input_report_abs(idev, ABS_MT_POSITION_Y, p[i].y * (ts->max_y+1)/600);//p[i].y leavs patched for FT5526
#endif
#ifdef FT5X06
			input_report_abs(idev, ABS_MT_POSITION_X, p[i].x);
			input_report_abs(idev, ABS_MT_POSITION_Y, p[i].y); 
#endif
		}
		tmp = ts->down_mask & ~down_mask;
		ts->down_mask = down_mask;
		release_slots(ts, tmp);
#else
		translate(&p[0].x, &p[0].y);
#endif
#ifdef USE_ABS_SINGLE
#ifdef FT5306
		input_report_abs(idev, ABS_X, ts->max_x - p[0].y);
		input_report_abs(idev, ABS_Y, ts->max_y - p[0].x*600/768);
		input_report_abs(idev, ABS_PRESSURE, 1);
		input_report_key(idev, BTN_TOUCH, 1);
#endif
#ifdef FT5X06
		input_report_abs(idev, ABS_X, p[0].x);
		input_report_abs(idev, ABS_Y, p[0].y);
		input_report_abs(idev, ABS_PRESSURE, 1);
		input_report_key(idev, BTN_TOUCH, 1);
#endif
#ifdef FT5526
                input_report_abs(idev, ABS_X, p[0].x * (ts->max_x+1)/1024);//p[i].x leavs patched for FT5526
                input_report_abs(idev, ABS_Y, p[0].y * (ts->max_y+1)/600);//p[i].y leavs patched for FT5526
		input_report_abs(idev, ABS_PRESSURE, 1);
		input_report_key(idev, BTN_TOUCH, 1);
#endif
#endif
	}
	input_sync(idev);
}

static int ts_open(struct input_dev *idev)
{
	struct ft5x06_ts *ts = input_get_drvdata(idev);
	return ts_startup(ts);
}

static void ts_close(struct input_dev *idev)
{
	struct ft5x06_ts *ts = input_get_drvdata(idev);
	ts_shutdown(ts);
}

static inline int ts_register(struct ft5x06_ts *ts)
{
	struct input_dev *idev; //定义input_dev结构体
	idev = input_allocate_device();  //调用input_allocate_device API分配input_dev结构体
	if (idev == NULL)
		return -ENOMEM;

	ts->max_x = 0x7ff;
	ts->max_y = 0x7ff;
	if (screenres[0])
		ts->max_x = screenres[0] - 1;
	else if (num_registered_fb > 0)
		ts->max_x = registered_fb[0]->var.xres - 1;
	if (screenres[1])
		ts->max_y = screenres[1] - 1;
	else if (num_registered_fb > 0)
		ts->max_y = registered_fb[0]->var.yres - 1; //由num_registered_fb获取分辨率

	pr_info("%s resolution is %dx%d\n", client_name, ts->max_x + 1, ts->max_y + 1);
	ts->idev = idev;	//填充idev结构体
	idev->name      = procentryname ;
	idev->id.bustype = BUS_I2C;
	idev->id.product = ts->client->addr;
	idev->open      = ts_open;
	idev->close     = ts_close;

	__set_bit(EV_ABS, idev->evbit); //申明触摸屏产生绝对坐标事件
	__set_bit(EV_KEY, idev->evbit); //申明触摸屏产生按键事件
	__set_bit(EV_SYN, idev->evbit); //申明触摸屏产生同步事件

#ifdef USE_ABS_MT
	input_mt_init_slots(idev, 16, 0); //初始化多点触摸的最大支持点数，第三个参数是一个标志
	input_set_abs_params(idev, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0); //x坐标的取值范围
	input_set_abs_params(idev, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0); 
	input_set_abs_params(idev, ABS_MT_TRACKING_ID, 0, MAX_TOUCHES, 0, 0); //用来支持硬件跟踪多点信息,即该点属于哪一条线等
#endif
#ifdef USE_ABS_SINGLE
	__set_bit(BTN_TOUCH, idev->keybit);
	input_set_abs_params(idev, ABS_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 1, 0, 0); //表示压力，取值范围[0,1) ABS_MT_TOUCH_MAJOR 手指触摸TP的椭圆长边直径/ ABS_MT_WIDTH_MAJOR 手指的近似椭圆长边直径
#endif

	input_set_drvdata(idev, ts);
	return input_register_device(idev);
}

static inline void ts_deregister(struct ft5x06_ts *ts)
{
	if (ts->idev) {
		input_unregister_device(ts->idev);
		input_free_device(ts->idev);
		ts->idev = NULL;
	}
}

#ifdef DEBUG
static void printHex(u8 const *buf, unsigned len)
{
	char hex[512];
	char *next = hex ;
	char *end = hex+sizeof(hex);

	while (len--) {
		next += snprintf(next, end-next, "%02x", *buf++);
		if (next >= end) {
			hex[sizeof(hex)-1] = '\0' ;
			break;
		}
	}
	printk(KERN_ERR "%s\n", hex);
}
#endif

static int proc_regnum = 0;
static int ft5x06_proc_read
	(struct file *f,
	 char __user *ubuf,
	 size_t count,
	 loff_t *off)
{
	int ret;
	unsigned char startch[1] = { (u8)proc_regnum };
	unsigned char buf[1];
	struct i2c_msg readpkt[2] = {
		{gts->client->addr, 0, 1, startch},
		{gts->client->addr, I2C_M_RD, sizeof(buf), buf}
	};
	ret = i2c_transfer(gts->client->adapter, readpkt,
			   ARRAY_SIZE(readpkt));
	if (ret != ARRAY_SIZE(readpkt)) {
		printk(KERN_WARNING "%s: i2c_transfer failed\n",
		       client_name);
	} else {
		printk (KERN_ERR "ft5x06[0x%02x] == 0x%02x\n", (u8)proc_regnum, buf[0]);
	}
	return 0 ;
}

static int
ft5x06_proc_write
	(struct file *file,
	 const char __user *buffer,
	 size_t count,
	 loff_t *data)
{
	proc_regnum = simple_strtoul(buffer,0,0);
	return count ;
}

struct file_operations proc_fops = {
	.read = ft5x06_proc_read,
	.write = ft5x06_proc_write,
};

static irqreturn_t ts_interrupt(int irq, void *id)
{
	struct ft5x06_ts *ts = id;
	int ret;
	struct point points[MAX_TOUCHES];
	unsigned char buf[3+(6*MAX_TOUCHES)];

	unsigned char startch[1] = { 0 };
	struct i2c_msg readpkt[2] = {
		{ts->client->addr, 0, 1, startch},
		{ts->client->addr, I2C_M_RD, sizeof(buf), buf}
	};
	int buttons = 0 ;

	while (gpiod_get_value(ts->wakeup_gpio)) {
		ts->bReady = 0;
		ret = i2c_transfer(ts->client->adapter, readpkt,
				   ARRAY_SIZE(readpkt));
		if (ret != ARRAY_SIZE(readpkt)) {
			printk(KERN_WARNING "%s: i2c_transfer failed\n",
			       client_name);
			msleep(1000);
		} else {
			int i;
			unsigned char *p = buf+3;
#ifdef DEBUG
			printHex(buf, sizeof(buf)); //buf 共75 (buf[3+(6*MAX_TOUCHES)],MAX_TOUCHES=12)个char(B),是i2c_transfer从触摸屏startch寄存器读取到的数据，该数据记录了触摸点数及坐标相关的值,前两个字节目前不清楚，第3个字节记录了目前触摸点数，一个点占6个字节，分别是4个字节记录p[0]-p[3],还有2个全为0的字节，最后还有12个字节目前不清楚，这就是为什么最大10点，却要写MAX_TOUCHES=12。
#endif
			buttons = buf[2];
			if (buttons > MAX_TOUCHES) {
				int interrupting = gpiod_get_value(ts->wakeup_gpio);
				if (interrupting) {
					printk(KERN_ERR
					       "%s: invalid button count 0x%02x\n",
					       __func__, buttons);
				} /* not garbage from POR */
				buttons = interrupting ? MAX_TOUCHES : 0;
			} else {
				for (i = 0; i < buttons; i++) {
#ifdef DEBUG
					printk(KERN_ERR "p[0]=%#x,\n "
							"p[1]=%#x,\n "
							"p[2]=%#x,\n "
							"p[3]=%#x\n ",p[0],p[1],p[2],p[3]);	
#endif
					points[i].x = (((p[0] & 0x0f) << 8)
						       | p[1]) & 0x7ff;
					points[i].id = (p[2]>>4);
					points[i].y = (((p[2] & 0x0f) << 8)
						       | p[3]) & 0x7ff;
					if (points[i].x > ts->max_x)
						points[i].x = ts->max_x;
#if defined(FT5306) || defined(FT5206) || defined(FT5306_800480)
#else
					if (points[i].y > ts->max_y)
						points[i].x = ts->max_y;
					if (points[i].y > 767)
						points[i].y = points[i].y*600/800+191;
#endif
					p += 6;
				}
			}
		}

#ifdef DEBUG
		printk(KERN_ERR "%s: buttons = %d, "
				"points[0].x = %d, "
				"points[0].y = %d\n",
		       client_name, buttons, points[0].x, points[0].y);
#endif
		ts_evt_add(ts, buttons, points);
	}
	return IRQ_HANDLED;
}

#define ID_G_THGROUP		0x80
#define ID_G_PERIODMONITOR	0x89
#define FT5X0X_REG_HEIGHT_B	0x8a
#define FT5X0X_REG_MAX_FRAME	0x8b
#define FT5X0X_REG_FEG_FRAME	0x8e
#define FT5X0X_REG_LEFT_RIGHT_OFFSET	0x92
#define FT5X0X_REG_UP_DOWN_OFFSET	0x93
#define FT5X0X_REG_DISTANCE_LEFT_RIGHT	0x94
#define FT5X0X_REG_DISTANCE_UP_DOWN	0x95
#define FT5X0X_REG_MAX_X_HIGH		0x98
#define FT5X0X_REG_MAX_X_LOW		0x99
#define FT5X0X_REG_MAX_Y_HIGH		0x9a
#define FT5X0X_REG_MAX_Y_LOW		0x9b
#define FT5X0X_REG_K_X_HIGH		0x9c
#define FT5X0X_REG_K_X_LOW		0x9d
#define FT5X0X_REG_K_Y_HIGH		0x9e
#define FT5X0X_REG_K_Y_LOW		0x9f

#define ID_G_AUTO_CLB	0xa0
#define ID_G_B_AREA_TH	0xae

#ifdef DEBUG
static void dumpRegs(struct ft5x06_ts *ts, unsigned start, unsigned end)
{
	u8 regbuf[512];
	unsigned char startch[1] = { start };
	int ret ;
	struct i2c_msg readpkt[2] = {
		{ts->client->addr, 0, 1, startch},
		{ts->client->addr, I2C_M_RD, end-start+1, regbuf}
	};
	ret = i2c_transfer(ts->client->adapter, readpkt, ARRAY_SIZE(readpkt));
	if (ret != ARRAY_SIZE(readpkt)) {
		printk(KERN_WARNING "%s: i2c_transfer failed\n", client_name);
	} else {
		printk(KERN_ERR "registers %02x..%02x\n", start, end);
		printHex(regbuf, end-start+1);
	}
}
#endif

static int ts_startup(struct ft5x06_ts *ts)
{
	int ret = 0;
	if (ts == NULL)
		return -EIO;

	if (ts->use_count++ != 0)
		goto out;

	ret = request_threaded_irq(ts->irq, NULL, ts_interrupt,
				     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     client_name, ts);
	if (ret) {
		pr_err("%s: error requesting irq %d\n", __func__, ts->irq);
		goto out;
	}

#ifdef DEBUG
	set_mode(ts, FACTORY_MODE);
	dumpRegs(ts, 0x4c, 0x4C);
	write_reg(ts, 0x4C, 0x05);
	dumpRegs(ts, 0, 0x4C);
#endif
	set_mode(ts, WORK_MODE);
#ifdef DEBUG
	dumpRegs(ts, 0x3b, 0x3b);
	dumpRegs(ts, 0x6a, 0x6a);
	dumpRegs(ts, ID_G_THGROUP, ID_G_PERIODMONITOR);
	dumpRegs(ts, FT5X0X_REG_HEIGHT_B, FT5X0X_REG_K_Y_LOW);
	dumpRegs(ts, ID_G_AUTO_CLB, ID_G_B_AREA_TH);
#endif
	set_mode(ts, WORK_MODE);

 out:
	if (ret)
		ts->use_count--;
	return ret;
}

/*
 * Release touchscreen resources.  Disable IRQs.
 */
static void ts_shutdown(struct ft5x06_ts *ts)
{
	if (ts) {
		if (--ts->use_count == 0) {
			free_irq(ts->irq, ts);
		}
	}
}
/*-----------------------------------------------------------------------*/

/* Return 0 if detection is successful, -ENODEV otherwise */
static int detect_ft5x06(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	char buffer;
	struct i2c_msg pkt = {
		client->addr,
		I2C_M_RD,
		sizeof(buffer),
		&buffer
	};
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;
	if (i2c_transfer(adapter, &pkt, 1) != 1)
		return -ENODEV;
	return 0;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int ts_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	int err = detect_ft5x06(client);
	if (!err)
		strlcpy(info->type, "ft5x06-ts", I2C_NAME_SIZE);
	return err;
}

static int ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct ft5x06_ts *ts;
	struct device *dev = &client->dev;
	struct gpio_desc *gp;

	if (gts) {
		printk(KERN_ERR "%s: Error gts is already allocated\n",
		       client_name);
		return -ENOMEM;
	}
	ts = kzalloc(sizeof(struct ft5x06_ts), GFP_KERNEL);
	if (!ts) {
		dev_err(dev, "Couldn't allocate memory for %s\n", client_name);
		return -ENOMEM;
	}
	ts->client = client;
	ts->irq = client->irq ;

	gp = devm_gpiod_get_index(dev, "reset", 0);
	dev_info(dev, "reset %p\n", gp);
	if (!IS_ERR(gp)) {
		/* release reset */
		ts->reset_gpio = gp;
		err = gpiod_direction_output(gp, 1);	/* doesn't use active_low flag */
		if (err)
			goto exit1;
		gpiod_set_value(gp, 0);
		msleep(1);
	}
	err = detect_ft5x06(client);
	if (err) {
		dev_err(dev, "Could not detect touch screen %d.\n", err);
		goto exit1;
	}
	ts->wakeup_gpio = devm_gpiod_get_index(dev, "wakeup", 0);
	pr_info("%s: wakeup %p\n", __func__, ts->wakeup_gpio);
	if (IS_ERR(ts->wakeup_gpio)) {
		err = -ENODEV;
		goto exit1;
	}

	printk(KERN_INFO "%s: %s touchscreen irq=%i, wakeup_irq=%i\n", __func__,
	       client_name, ts->irq, gpiod_to_irq(ts->wakeup_gpio));
	i2c_set_clientdata(client, ts);
	err = ts_register(ts);
	if (err == 0) {
		gts = ts;
		ts->procentry = proc_create(procentryname, 0x660, NULL,
					    &proc_fops);
		return 0;
	}

	printk(KERN_WARNING "%s: ts_register failed\n", client_name);
	ts_deregister(ts);
exit1:
	kfree(ts);
	return err;
}

static int ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts *ts = i2c_get_clientdata(client);
	remove_proc_entry(procentryname, 0);
	if (ts == gts) {
		gts = NULL;
		ts_deregister(ts);
	} else {
		printk(KERN_ERR "%s: Error ts!=gts\n", client_name);
	}
	if (ts->reset_gpio)
		gpiod_set_value(ts->reset_gpio, 1);
	kfree(ts);
	return 0;
}


/*-----------------------------------------------------------------------*/

static const struct i2c_device_id ts_idtable[] = {
	{ "ft5x06-ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts_idtable);

static const struct of_device_id ft5x06_dt_ids[] = {
       {
               .compatible = "ft5x06,ft5x06-touch",
       }, {
               /* sentinel */
       }
};
MODULE_DEVICE_TABLE(of, ft5x06_dt_ids);

static struct i2c_driver ts_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "ft5x06-ts",
		.of_match_table = ft5x06_dt_ids,
	},
	.id_table	= ts_idtable,
	.probe		= ts_probe,
	.remove		= ts_remove,
	.detect		= ts_detect,
};

module_i2c_driver(ts_driver);

MODULE_AUTHOR("Boundary Devices <info@boundarydevices.com>");
MODULE_DESCRIPTION("I2C interface for FocalTech ft5x06 touch screen controller.");
MODULE_LICENSE("GPL");
