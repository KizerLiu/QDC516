
#define pr_fmt(fmt)	"sgm41511: %s: " fmt, __func__
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/usb/phy.h>

#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/workqueue.h>

struct sgm41511_config{
	int	charge_voltage;
	int	charge_current;
	int boost_voltage;
	int boost_current;

	bool enable_term;
	int	term_current;

	bool enable_ico;
	bool use_absolute_iindpm;
};

struct sgm41511 {
	struct device *dev;
	struct i2c_client *client;
	
	int part_no;
	struct mutex i2c_rw_lock;
	bool charge_enabled;
	int charge_enable_gpio;
	struct delayed_work irq_work;

	struct sgm41511_config cfg;

	int usb_vbus_gpio;
	int usb_vbus_irq;
	
	struct power_supply *usb_psy;
	bool vbus_present;
};

static struct sgm41511 *g_sgm;

static int __sgm41511_read_byte(struct sgm41511 *sgm, u8 reg, u8 *data)
{
	int ret;

	ret = i2c_smbus_read_byte_data(sgm->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8)ret;
	
	return 0;

}

static int __sgm41511_write_byte(struct sgm41511 *sgm, u8 reg, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(sgm->client, reg, data);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				data, reg, ret);
		return ret;
	}
	return 0;

}

static int sgm41511_read_byte(struct sgm41511 *sgm, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm41511_read_byte(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);

	return ret;
}

static int sgm41511_write_byte(struct sgm41511 *sgm, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm41511_write_byte(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);
	return ret;
}

static int sgm41511_update_bits(struct sgm41511 *sgm, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&sgm->i2c_rw_lock);

	ret = __sgm41511_read_byte(sgm, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, reg=%d\n", reg, ret);
		goto out;
	}
		
	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sgm41511_write_byte(sgm, reg, tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, reg=%d\n", reg, ret);
	}
		
out:
	mutex_unlock(&sgm->i2c_rw_lock);
	return ret;
}

#define SGM41511_REG_0B               0x0B
#define SGM41511_PN_MASK             0x78
#define SGM41511_PN_SHIFT            3

static int sgm41511_detect_device(struct sgm41511 *sgm)
{
	int ret;
	u8 data;

	ret = sgm41511_read_byte(sgm, &data, SGM41511_REG_0B);
	if (ret == 0) {
		sgm->part_no = (data & SGM41511_PN_MASK) >> SGM41511_PN_SHIFT;
	}

	return ret;
}

#define SGM41511_REG_05               0x05
#define SGM41511_WDT_MASK          0x30
#define SGM41511_WDT_SHIFT            4
#define SGM41511_WDT_DISABLE         0
static int sgm41511_disable_watchdog_timer(struct sgm41511 *sgm)
{
	u8 val = SGM41511_WDT_DISABLE << SGM41511_WDT_SHIFT;

	return sgm41511_update_bits(sgm, SGM41511_REG_05, 
						SGM41511_WDT_MASK, val);
}

#define SGM41511_REG_01               0x01
#define SGM41511_WDT_RESET_MASK      0x40
#define SGM41511_WDT_RESET_SHIFT     6
#define SGM41511_WDT_RESET           1
static int sgm41511_reset_watchdog_timer(struct sgm41511 *sgm)
{
	u8 val = SGM41511_WDT_RESET << SGM41511_WDT_RESET_SHIFT;

	return sgm41511_update_bits(sgm, SGM41511_REG_01, 
						SGM41511_WDT_RESET_MASK, val);
}

#define SGM41511_REG_07              0x07
#define SGM41511_FORCE_IINDPM_MASK   0x80
#define SGM41511_FORCE_IINDPM_SHIFT  7
#define SGM41511_FORCE_IINDPM_ENABLE 1
#define SGM41511_FORCE_IINDPM_DISABLE 0
static int sgm41511_use_absolute_iindpm(struct sgm41511 *sgm, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = SGM41511_FORCE_IINDPM_ENABLE << SGM41511_FORCE_IINDPM_SHIFT;
	else
		val = SGM41511_FORCE_IINDPM_DISABLE << SGM41511_FORCE_IINDPM_SHIFT;

	ret = sgm41511_update_bits(sgm, SGM41511_REG_07, SGM41511_FORCE_IINDPM_MASK, val);

	return ret;

}

#define SGM41511_REG_06              0x06
#define SGM41511_VINDPM_MASK         0x0F
#define SGM41511_VINDPM_SHIFT        0
#define SGM41511_VINDPM_BASE         3900
#define SGM41511_VINDPM_LSB          100
static int sgm41511_set_input_volt_limit(struct sgm41511 *sgm, int volt)
{
	u8 val;
	val = (volt - SGM41511_VINDPM_BASE) / SGM41511_VINDPM_LSB;
	return sgm41511_update_bits(sgm, SGM41511_REG_06, 
						SGM41511_VINDPM_MASK, val << SGM41511_VINDPM_SHIFT);
}

#define SGM41511_REG_00              0x00
#define SGM41511_IINLIM_MASK		    0x1F
#define SGM41511_IINLIM_SHIFT		0
#define SGM41511_IINLIM_BASE         100
#define SGM41511_IINLIM_LSB          100
static int sgm41511_set_input_current_limit(struct sgm41511 *sgm, int curr)
{
	u8 val;

	val = (curr - SGM41511_IINLIM_BASE) / SGM41511_IINLIM_LSB;
	return sgm41511_update_bits(sgm, SGM41511_REG_00, SGM41511_IINLIM_MASK, 
						val << SGM41511_IINLIM_SHIFT);
}

/*static int sgm41511_get_input_current_limit(struct sgm41511 *sgm)
{
	u8 val;
	int cur;
	int ret;
	ret = sgm41511_read_byte(sgm, &val, SGM41511_REG_00);
	if (ret < 0) {
		dev_err(sgm->dev, "read input current limit failed :%d\n", ret);
		return ret;
	} else{
		cur = SGM41511_IINLIM_BASE + ((val & SGM41511_IINLIM_MASK) >> SGM41511_IINLIM_SHIFT) * SGM41511_IINLIM_LSB ;
		return cur;
	}
}*/

#define SGM41511_REG_04              0x04
#define SGM41511_VREG_MASK           0xF1
#define SGM41511_VREG_SHIFT          3
#define SGM41511_VREG_BASE           3856
#define SGM41511_VREG_LSB            32
static int sgm41511_set_chargevoltage(struct sgm41511 *sgm, int volt)
{
	u8 val;

	val = (volt - SGM41511_VREG_BASE)/SGM41511_VREG_LSB;
	return sgm41511_update_bits(sgm, SGM41511_REG_04, 
						SGM41511_VREG_MASK, val << SGM41511_VREG_SHIFT);
}

/*static int sgm41511_get_chargevoltage(struct sgm41511 *sgm)
{
	u8 val;
	int vol;
	int ret;
	ret = sgm41511_read_byte(sgm, &val, SGM41511_REG_04);
	if (ret < 0) {
		dev_err(sgm->dev, "read voltage failed :%d\n", ret);
		return ret;
	} else{
		vol = SGM41511_VREG_BASE + ((val & SGM41511_VREG_MASK) >> SGM41511_VREG_SHIFT) * SGM41511_VREG_LSB ;
		pr_info("wjh read voltage:%d\n",vol);
		return vol;
	}
}*/

#define SGM41511_REG_02              0x02
#define SGM41511_ICHG_MASK           0x3F
#define SGM41511_ICHG_SHIFT          0
#define SGM41511_ICHG_BASE           0
#define SGM41511_ICHG_LSB            60
static int sgm41511_set_chargecurrent(struct sgm41511 *sgm, int curr)
{
	u8 ichg;

	ichg = (curr - SGM41511_ICHG_BASE)/SGM41511_ICHG_LSB;
	return sgm41511_update_bits(sgm, SGM41511_REG_02, 
						SGM41511_ICHG_MASK, ichg << SGM41511_ICHG_SHIFT);

}

/*static int sgm41511_get_chargecurrent(struct sgm41511 *sgm)
{
	int ichg;
	u8 val;
	int ret;
	ret = sgm41511_read_byte(sgm, &val, SGM41511_REG_02);
	if (ret < 0) {
		dev_err(sgm->dev, "read current failed :%d\n", ret);
		return ret;
	} else{
		ichg = SGM41511_ICHG_BASE + ((val & SGM41511_ICHG_MASK) >> SGM41511_ICHG_SHIFT) * SGM41511_ICHG_LSB ;
		return ichg;
	}

}*/

#define SGM41511_REG_03                  0x03
#define SGM41511_ITERM_MASK          0x0F
#define SGM41511_ITERM_SHIFT         0
#define SGM41511_ITERM_BASE          60
#define SGM41511_ITERM_LSB           60
static int sgm41511_set_term_current(struct sgm41511 *sgm, int curr)
{
	u8 iterm;

	iterm = (curr - SGM41511_ITERM_BASE) / SGM41511_ITERM_LSB;

	return sgm41511_update_bits(sgm, SGM41511_REG_03, 
						SGM41511_ITERM_MASK, iterm << SGM41511_ITERM_SHIFT);
}

static int sgm41511_set_otg_current(struct sgm41511 *sgm, int curr)
{
	u8 temp;

	if (curr == 500)
		temp = 0;
	else if (curr == 1200)
		temp = 1;
	else
		temp = 1;

	return sgm41511_update_bits(sgm, 0x02, 0x80, temp << 7);
}

static int sgm41511_set_otg_volt(struct sgm41511 *sgm, int volt)
{
	u8 val = 0;
	if (volt == 4850)
		val = 0;
	else if (volt == 5000)
		val = 1;
	else if (volt == 5150)
		val = 2;
	else if(volt == 5300)
		val = 3;
	else
		val = 2;

	return sgm41511_update_bits(sgm, 0x06, 0x30, val << 4);

}

#define SGM41511_CHG_CONFIG_MASK     0x10
#define SGM41511_CHG_CONFIG_SHIFT    4
#define SGM41511_CHG_ENABLE          1
#define SGM41511_CHG_DISABLE         0
static int sgm41511_enable_charger(struct sgm41511 *sgm)
{
	int ret;
	u8 val = SGM41511_CHG_ENABLE << SGM41511_CHG_CONFIG_SHIFT;

	ret = sgm41511_update_bits(sgm, SGM41511_REG_01, 
						SGM41511_CHG_CONFIG_MASK, val);
	return ret;
}

/*static int sgm41511_disable_charger(struct sgm41511 *sgm)
{
	int ret;
	u8 val = SGM41511_CHG_DISABLE << SGM41511_CHG_CONFIG_SHIFT;

	ret = sgm41511_update_bits(sgm, SGM41511_REG_01, 
						SGM41511_CHG_CONFIG_MASK, val);
	return ret;
}*/

static int sgm41511_init_device(struct sgm41511 *sgm)
{
	int ret;
	
	sgm41511_reset_watchdog_timer(sgm);
	
	sgm41511_disable_watchdog_timer(sgm);
	
	sgm41511_use_absolute_iindpm(sgm, true);

	ret = sgm41511_set_input_volt_limit(sgm, 4500);
	if (ret < 0) {
		pr_err("Failed to set vindpm:%d\n",  ret);
	}
	
	ret = sgm41511_set_input_current_limit(sgm, 2400);
	if (ret < 0) {
		pr_err("Failed to set iindpm:%d\n",  ret);
	}
	
	ret = sgm41511_set_term_current(sgm, 180);
	if (ret < 0) {
		pr_err("Failed to set termination current:%d\n",  ret);
	}

	ret = sgm41511_set_chargevoltage(sgm, 4200);
	if (ret < 0) {
		pr_err("Failed to set charge voltage:%d\n",  ret);
	}
	
	ret = sgm41511_set_chargecurrent(sgm, 2040);
	if (ret < 0) {
		pr_err("Failed to set charge current:%d\n",  ret);
	}

	ret = sgm41511_set_otg_volt(sgm, 5150);
	if (ret < 0) {
		pr_err("Failed to set boost voltage:%d\n", ret);
	}

	ret = sgm41511_set_otg_current(sgm, 1200);
	if (ret < 0) {
		pr_err("Failed to set boost current:%d\n", ret);
	}

	ret = sgm41511_enable_charger(sgm);
	if (ret < 0) {
		pr_err("Failed to enable charger:%d\n",  ret);
		return ret;
	} else {
		sgm->charge_enabled = true;
		pr_err("charger enable\n");
	}

	return 0;
}

#define SGM41511_REG_0A              0x0A
#define SGM41511_VBUS_STAT_MASK      0x80
#define SGM41511_VBUS_STAT_SHIFT     7

#define SGM41511_REG_08               0x08
#define SGM41511_VBUS_TYPE_MASK      0xE0
#define SGM41511_VBUS_TYPE_SHIFT     5
#define SGM41511_CHRG_STAT_MASK      0x18
#define SGM41511_CHRG_STAT_SHIFT     3

static int sgm41511_get_vbus_type(struct sgm41511 *sgm)
{
	u8 status = 0;
	int vbus_type;

	sgm41511_read_byte(sgm, &status, SGM41511_REG_08);
	//pr_err("get vbus type=0x%x\n",status);
	vbus_type = (status&SGM41511_VBUS_TYPE_MASK) >> SGM41511_VBUS_TYPE_SHIFT;
	
	return vbus_type;
}

static int sgm41511_get_charge_status(struct sgm41511 *sgm)
{
	u8 status = 0;
	int chg_status;

	sgm41511_read_byte(sgm, &status, SGM41511_REG_08);
	//pr_err("get charge status=0x%x\n",status);
	chg_status = (status&SGM41511_CHRG_STAT_MASK) >> SGM41511_CHRG_STAT_SHIFT;
	
	return chg_status;
}

int sgm41511_enable_gpio_charger(int enable)
{
	int ret;
	
	ret = gpio_direction_output(g_sgm->charge_enable_gpio, enable);
	if (ret) {
		pr_err("disable charger fail!!!\n");
	}
	schedule_delayed_work(&g_sgm->irq_work, msecs_to_jiffies(5));
	
	return ret;
}


static void sgm41511_irq_work_start(struct work_struct *work)
{
	struct delayed_work *dw = to_delayed_work(work);
	struct sgm41511 *sgm =
		(struct sgm41511 *)container_of(dw, struct sgm41511, irq_work);
	union power_supply_propval propval;
	int vbus_type, chg_status;

    vbus_type = sgm41511_get_vbus_type(sgm);
	chg_status = sgm41511_get_charge_status(sgm);
    pr_err("get vbus_type=0x%x,charge status=0x%x\n", vbus_type, chg_status);
	if((vbus_type == 0x02) && (chg_status != 0) && (sgm->vbus_present == false)){
	  sgm->vbus_present = true;
	  propval.intval = POWER_SUPPLY_TYPE_USB_DCP;
	  power_supply_set_online(sgm->usb_psy, sgm->vbus_present);
	  sgm->usb_psy->set_property(sgm->usb_psy,
				POWER_SUPPLY_PROP_REAL_TYPE,
				&propval);
	}else if(((vbus_type == 0)||(chg_status == 0)) && (sgm->vbus_present == true)){
	sgm->vbus_present = false;
	propval.intval = POWER_SUPPLY_TYPE_UNKNOWN;
	power_supply_set_online(sgm->usb_psy, sgm->vbus_present);
	sgm->usb_psy->set_property(sgm->usb_psy,
			  POWER_SUPPLY_PROP_REAL_TYPE,
			  &propval);
	}
}

static irqreturn_t sgm41511_charger_interrupt(int irq, void *data)
{
	struct sgm41511 *sgm = data;
	pr_err("enter sgm41511 charger interrupt\n");
	schedule_delayed_work(&sgm->irq_work, msecs_to_jiffies(5));

	return IRQ_HANDLED;
}

static ssize_t sgm41511_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sgm41511 *sgm = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sgm41511");
	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = sgm41511_read_byte(sgm, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
						"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t sgm41511_store_register(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct sgm41511 *sgm = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf,"%x %x",&reg, &val);
	if (ret == 2 && reg <= 0x0B) {
		sgm41511_write_byte(sgm,(unsigned char)reg,(unsigned char)val);
	}

	return count;
}

static DEVICE_ATTR(registers, 0660, sgm41511_show_registers, sgm41511_store_register);

static struct attribute *sgm41511_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group sgm41511_attr_group = {
	.attrs = sgm41511_attributes,
};

static int sgm41511_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct sgm41511 *sgm;
	struct power_supply *usb_psy;
	int ret;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	sgm = devm_kzalloc(&client->dev, sizeof(struct sgm41511), GFP_KERNEL);
	if (!sgm) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}
	
	sgm->dev = &client->dev;
	sgm->client = client;
	sgm->usb_psy = usb_psy;
	i2c_set_clientdata(client, sgm);

	mutex_init(&sgm->i2c_rw_lock);

	sgm->vbus_present = false;
	
	ret = sgm41511_detect_device(sgm);
	if (!ret && sgm->part_no == 0x02) {
		pr_info("charger device SGM41511 detected, revision:%d\n",
									sgm->part_no);
	} else {
		pr_info("no SGM41511 charger device found:%d\n",ret);
		return -ENODEV;
	}
       
	ret = sgm41511_init_device(sgm);
	if (ret) {
		dev_err(sgm->dev, "device init failure: %d\n", ret);
		goto err_0;
	}

	sgm->charge_enable_gpio = of_get_named_gpio(client->dev.of_node,
					"sgm,charge-enable-gpio", 0);
	if (gpio_is_valid(sgm->charge_enable_gpio)) {
		ret = devm_gpio_request(&client->dev, sgm->charge_enable_gpio,
					"CHARGE_ENABLE");
		if (ret) {
			pr_err( "gpio req failed for gpio_%d\n", sgm->charge_enable_gpio);

		}
		ret = gpio_direction_output(sgm->charge_enable_gpio, 0);
		if (ret) {
			pr_err("Invalid input from GPIO_%d\n", sgm->charge_enable_gpio);
		}
	}

	if (of_find_property(sgm->dev->of_node, "sgm,interrupt-gpio", NULL)) {
		sgm->usb_vbus_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,interrupt-gpio", 0);
		if (!gpio_is_valid(sgm->usb_vbus_gpio)) {
			if (sgm->usb_vbus_gpio != -EPROBE_DEFER)
				pr_err("failed to get usb vbus gpio=%d\n",
						sgm->usb_vbus_gpio);
		}
		if (gpio_is_valid(sgm->usb_vbus_gpio)) {
			ret = devm_gpio_request(sgm->dev, sgm->usb_vbus_gpio, "interrupt-gpio");
			if (ret) {
				pr_err("failed to request usbid gpio\n");
				goto err_irq;
			}
			sgm->usb_vbus_irq = gpio_to_irq(sgm->usb_vbus_gpio);
		}
	}
	
	if (sgm->usb_vbus_irq) {
		ret = devm_request_threaded_irq(&client->dev, sgm->usb_vbus_irq, 
				NULL, sgm41511_charger_interrupt, 
				IRQF_TRIGGER_RISING |IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"sgm41511_charger_irq", sgm);
		if (ret) {
			pr_err("Request IRQ %d failed: %d\n", sgm->usb_vbus_irq, ret);
			goto err_irq;
		} else {
			pr_info("irq = %d\n", sgm->usb_vbus_irq);
		}
		enable_irq_wake(sgm->usb_vbus_irq);
	}
    INIT_DELAYED_WORK(&sgm->irq_work, sgm41511_irq_work_start);
	
	schedule_delayed_work(&sgm->irq_work, msecs_to_jiffies(5));

	ret = sysfs_create_group(&sgm->dev->kobj, &sgm41511_attr_group);
	if (ret) {
		pr_err("failed to register sysfs. err: %d\n", ret);
		goto err_sys;
	}
	g_sgm = sgm;
	pr_info(" probe successfully\n");
	
	return 0;

err_sys:
	sysfs_remove_group(&sgm->dev->kobj, &sgm41511_attr_group);
err_irq:
	devm_free_irq(sgm->dev, sgm->usb_vbus_irq, sgm);
err_0:
	mutex_destroy(&sgm->i2c_rw_lock);

	return ret;
}

static int sgm41511_charger_remove(struct i2c_client *client)
{
	struct sgm41511 *sgm = i2c_get_clientdata(client);

	mutex_destroy(&sgm->i2c_rw_lock);

	sysfs_remove_group(&sgm->dev->kobj, &sgm41511_attr_group);

	return 0;
}

static void sgm41511_charger_shutdown(struct i2c_client *client)
{
	pr_info("shutdown..");
}

static struct of_device_id sgm41511_charger_match_table[] = {
	{.compatible = "sgm41511",},
	{},
};


static const struct i2c_device_id sgm41511_charger_id[] = {
	{ "sgm41511", 0x00 },
	{},
};

MODULE_DEVICE_TABLE(i2c, sgm41511_charger_id);

static struct i2c_driver sgm41511_charger_driver = {
	.driver		= {
		.name	= "sgm41511",
		.of_match_table = sgm41511_charger_match_table,
		//.pm	= &sgm41511_pm_ops,
	},
	.id_table	= sgm41511_charger_id,

	.probe		= sgm41511_charger_probe,
	.shutdown  	= sgm41511_charger_shutdown,
	.remove		= sgm41511_charger_remove,
};
static int sgm41511_init(void)
{
	return i2c_add_driver(&sgm41511_charger_driver);
}		
static void sgm41511_exit(void)
{
	i2c_del_driver(&sgm41511_charger_driver);
}

late_initcall(sgm41511_init);

//module_i2c_driver(sgm41511_charger_driver);
module_exit(sgm41511_exit);

MODULE_DESCRIPTION("SGM41511 Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("jeffery");

