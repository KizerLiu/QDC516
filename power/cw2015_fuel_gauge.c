#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define CWFG_ENABLE_LOG 1
#define CWFG_I2C_BUSNUM 2
#define DOUBLE_SERIES_BATTERY 0

#define CW_PROPERTIES "battery"

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_VTEMPL              0xC
#define REG_VTEMPH              0xD
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)

#define queue_delayed_work_time  8000
#define BATTERY_CAPACITY_ERROR  40*1000
#define BATTERY_CHARGING_ZERO   1800*1000

#define UI_FULL 100
#define DECIMAL_MAX 70
#define DECIMAL_MIN 30

#define CHARGING_ON 1
#define NO_CHARGING 0


#define cw_printk(fmt, arg...)        \
({                                    \
	if(CWFG_ENABLE_LOG){              \
		printk("FG_CW2015 : %s : " fmt, __FUNCTION__ ,##arg);  \
	} else { }                           \
})

#define CWFG_NAME "cw2015"
#define SIZE_BATINFO    64

static unsigned char config_info[SIZE_BATINFO] = {
	0x17,0x67,0x6E,0x66,0x65,0x64,0x63,0x5F,
	0x5B,0x5D,0x53,0x55,0x5C,0x52,0x43,0x3C,
	0x33,0x2A,0x26,0x23,0x27,0x33,0x3F,0x4C,
	0x2E,0x28,0x0C,0xCD,0x29,0x49,0x4E,0x5F,
	0x6F,0x75,0x75,0x78,0x3E,0x1B,0x80,0x4E,
	0x0C,0x65,0x11,0x3D,0x6C,0x9C,0xBC,0x1F,
	0x40,0x78,0x9B,0xBB,0x80,0x80,0x97,0xCB,
	0x2F,0x00,0x64,0xA5,0xB5,0x0C,0xB8,0x11,
};

extern int sgm41511_enable_gpio_charger(int enable);

static struct timespec suspend_time_before;
static struct timespec after;
static int suspend_resume_mark = 0;

struct cw_battery {
	int charger_mode;
	int capacity;
	int voltage;
	int status;
	int change;
	struct i2c_client *client;
	struct power_supply batt_psy;
	struct power_supply *usb_psy;
	struct workqueue_struct *cwfg_workqueue;
	struct delayed_work battery_delay_work;
	bool chg_enabled;
};

static int g_cw2015_capacity = 0;
static int g_cw2015_vol = 0;

static int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	return i2c_smbus_read_i2c_block_data( client, reg, 1, buf);
}
	
static int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
	return i2c_smbus_write_i2c_block_data( client, reg, 1, &buf[0]);
}

static int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	return i2c_smbus_read_i2c_block_data( client, reg, 2, buf );
}

static int cw_update_config_info(struct cw_battery *cw_bat)
{
	int i;
	int ret;
	unsigned char reg_val;
	unsigned char reset_val;

	// make sure no in sleep mode
	ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0) {
		printk("cw_read Error!\n");
		return ret;
	}

	reset_val = reg_val;
	if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP)
		return -1;

	// update new battery info
	for (i = 0; i < SIZE_BATINFO; i++) {
		ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info[i]);
		if (ret < 0)
			return ret;
	}

	reg_val = 0x00;
	reg_val |= CONFIG_UPDATE_FLG;
	reg_val &= 0x07;
	reg_val |= ATHD;
	ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
	if(ret < 0)
		return ret;

	msleep(50);
	reg_val = 0x00;
	reset_val &= ~(MODE_RESTART);
	reg_val = reset_val | MODE_RESTART;
	ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0)
		return ret;

	msleep(10);

	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;

	msleep(100);
	cw_printk("cw2015 update config success!\n");

	return 0;
}

static int cw_get_voltage(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val[2];
	u16 value16, value16_1, value16_2, value16_3;
	int voltage;

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
		return ret;

	value16 = (reg_val[0] << 8) + reg_val[1];

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
		return ret;

	value16_1 = (reg_val[0] << 8) + reg_val[1];

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
		return ret;
	
	value16_2 = (reg_val[0] << 8) + reg_val[1];

	if (value16 > value16_1) {
		value16_3 = value16;
		value16 = value16_1;
		value16_1 = value16_3;
	}

	if (value16_1 > value16_2) {
		value16_3 =value16_1;
		value16_1 =value16_2;
		value16_2 =value16_3;
	}

	if (value16 >value16_1) {
		value16_3 =value16;
		value16 =value16_1;
		value16_1 =value16_3;
	}

	voltage = value16_1 * 625 / 2048;

	if (DOUBLE_SERIES_BATTERY)
		voltage = voltage * 2;

	return voltage;
}

static void cw_update_vol(struct cw_battery *cw_bat)
{
	int ret;

	ret = cw_get_voltage(cw_bat);
	if ((ret >= 0) && (cw_bat->voltage != ret)) {
		cw_bat->voltage = ret;
		cw_bat->change = 1;
	}
}

static int get_charge_state(struct cw_battery *cw_bat)
{
	union power_supply_propval val = {0, };
	int ret;
	int usb_online = 0;
	int usb_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (cw_bat->usb_psy) {
		ret = cw_bat->usb_psy->get_property(cw_bat->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &val);
		if (!ret)
			usb_online = val.intval;
	}

    val.intval = 0;
	if (cw_bat->usb_psy) {
		ret = cw_bat->usb_psy->get_property(cw_bat->usb_psy,
				POWER_SUPPLY_PROP_REAL_TYPE, &val);
		if (!ret)
			usb_type = val.intval;
	}
	pr_err("usb_online=%d usb_type=%d\n",usb_online,usb_type);
	if ((usb_online)&&((usb_type == POWER_SUPPLY_TYPE_USB_DCP)||(usb_type == POWER_SUPPLY_TYPE_USB)))
		return 1;

	return 0;
}
static void cw_update_charge_status(struct cw_battery *cw_bat)
{
	int cw_charger_mode;

	cw_charger_mode = get_charge_state(cw_bat);
	if (cw_bat->charger_mode != cw_charger_mode){
		cw_bat->charger_mode = cw_charger_mode;
		cw_bat->change = 1;
	}
}

static void cw_update_status(struct cw_battery *cw_bat)
{
	int status;

	if (cw_bat->charger_mode > 0) {
		if (cw_bat->capacity >= 100)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
		cw_bat->chg_enabled = 1;
	} else
		status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (cw_bat->status != status) {
		cw_bat->status = status;
		cw_bat->change = 1;
	}
}

static int cw_init_data(struct cw_battery *cw_bat)
{
	unsigned char reg_SOC[2];
	int real_SOC = 0;
	int digit_SOC = 0;
	int ret;
	int UI_SOC = 0;

	ret = cw_read_word(cw_bat->client, REG_SOC, reg_SOC);
	if (ret < 0)
		return ret;

	real_SOC = reg_SOC[0];
	digit_SOC = reg_SOC[1];
	UI_SOC = ((real_SOC * 256 + digit_SOC) * 100)/ (UI_FULL*256);

	cw_update_vol(cw_bat);
	cw_update_charge_status(cw_bat);
	cw_bat->capacity = UI_SOC;
	cw_update_status(cw_bat);

	return 0;
}

static int cw_init(struct cw_battery *cw_bat)
{
	int i;
	int ret;
	unsigned char reg_val = MODE_SLEEP;

	if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
		reg_val = MODE_NORMAL;
		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
		if (ret < 0)
			return ret;
	}

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0) {
		printk("cw_read Error!\n");
		return ret;
	}else{
		printk("cw_read ok:0x%x!\n",reg_val);
        }

	if ((reg_val & 0xf8) != ATHD) {
		reg_val &= 0x07;
		reg_val |= ATHD;
		ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
		if (ret < 0)
			return ret;
	}

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0) {
		printk("cw_read Error!\n");
		return ret;
	}else{
		printk("cw_read ok:0x%x!!\n",reg_val);
        }

	if (!(reg_val & CONFIG_UPDATE_FLG)) {
		cw_printk("update config flg is true, need update config\n");
		ret = cw_update_config_info(cw_bat);
		if (ret < 0) {
			printk("%s: update config fail\n", __func__);
			return ret;
		}
	} else {
		for (i = 0; i < SIZE_BATINFO; i++) {
			ret = cw_read(cw_bat->client, REG_BATINFO + i, &reg_val);
			if (ret < 0)
				return ret;
			if (config_info[i] != reg_val)
				break;
		}
		if (i != SIZE_BATINFO) {
			cw_printk("config didn't match, need update config\n");
			ret = cw_update_config_info(cw_bat);
			if (ret < 0)
				return ret;
		}
	}

	msleep(10);
	for (i = 0; i < 30; i++) {
		ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
		if (ret < 0)
			return ret;
		else if (reg_val <= 0x64)
			break;
		msleep(120);
	}

	if (i >= 30 ){
		reg_val = MODE_SLEEP;
		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
		cw_printk("cw2015 input unvalid power error, cw2015 join sleep mode\n");
		return -1;
	} 

	cw_printk("cw2015 init success!\n");

	return 0;
}

static int cw_por(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reset_val;

	reset_val = MODE_SLEEP;
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;

	reset_val = MODE_NORMAL;
	msleep(10);
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;

	ret = cw_init(cw_bat);
	if (ret)
		return ret;

	return 0;
}

static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int ui_100 = UI_FULL;
	int remainder = 0;
	int real_SOC = 0;
	int digit_SOC = 0;
	int UI_SOC = 0;
	int ret;
	unsigned char reg_val[2];
	static int reset_loop = 0;
	static int charging_zero_loop = 0;

	ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
	if (ret < 0)
		return ret;

	real_SOC = reg_val[0];
	digit_SOC = reg_val[1];

	if ((real_SOC < 0) || (real_SOC > 100)) {
		cw_printk("Error:  real_SOC = %d\n", real_SOC);
		reset_loop++;
		if (reset_loop > (BATTERY_CAPACITY_ERROR / queue_delayed_work_time)) {
			cw_por(cw_bat);
			reset_loop =0;
		}
		return cw_bat->capacity;
	} else
		reset_loop =0;

	if ((cw_bat->charger_mode > 0) &&(real_SOC == 0)) {
		charging_zero_loop++;
		if (charging_zero_loop > BATTERY_CHARGING_ZERO / queue_delayed_work_time) {
			cw_por(cw_bat);
			charging_zero_loop = 0;
		}
	} else if (charging_zero_loop != 0)
		charging_zero_loop = 0;

	UI_SOC = ((real_SOC * 256 + digit_SOC) * 100) / (ui_100*256);
	remainder = (((real_SOC * 256 + digit_SOC) * 100 * 100) / (ui_100*256)) % 100;
	if (UI_SOC >= 100) {
		printk(KERN_INFO "CW2015[%d]: UI_SOC = %d larger 100!!!!\n", __LINE__, UI_SOC);
		UI_SOC = 100;
	} else {
		if ((remainder > 70 || remainder < 30) &&
			UI_SOC >= cw_bat->capacity - 1 &&
			UI_SOC <= cw_bat->capacity + 1) {
			UI_SOC = cw_bat->capacity;
			printk(KERN_INFO "CW2015[%d]: UI_SOC = %d, cw_bat->capacity = %d\n",
				__LINE__, UI_SOC, cw_bat->capacity);
		}
	}

	if (suspend_resume_mark == 1)
		suspend_resume_mark = 0;

	return UI_SOC;
}

static void cw_update_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity = cw_get_capacity(cw_bat);

	if ((cw_capacity >= 0) && (cw_capacity <= 100) &&
		(cw_bat->capacity != cw_capacity)) {
		cw_bat->capacity = cw_capacity;
		cw_bat->change = 1;
	}
}

static void cw_bat_work(struct work_struct *work)
{
	int i = 0;
	int ret;
	unsigned char reg_val;
	struct delayed_work *delay_work;
	struct cw_battery *cw_bat;

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);

	ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0) {
		cw_bat->capacity = 100;
		cw_bat->voltage = 4200;
		cw_bat->change = 1;
	} else {
		if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
			for (i = 0; i < 5; i++) {
				if (cw_por(cw_bat) == 0)
					break;
			}
		}
		cw_update_vol(cw_bat);
		cw_update_charge_status(cw_bat);
		cw_update_capacity(cw_bat);
		cw_update_status(cw_bat);
	}
	cw_printk("charger_mod = %d\n", cw_bat->charger_mode);
	cw_printk("status = %d\n", cw_bat->status);
	cw_printk("capacity = %d\n", cw_bat->capacity);
	cw_printk("voltage = %d\n", cw_bat->voltage);

	if (suspend_resume_mark == 1)
		suspend_resume_mark = 0;

	if (cw_bat->change == 1) {
		power_supply_changed(&cw_bat->batt_psy);
		cw_bat->change = 0;
	}
	g_cw2015_capacity = cw_bat->capacity;
	g_cw2015_vol = cw_bat->voltage;

	queue_delayed_work(cw_bat->cwfg_workqueue,
		&cw_bat->battery_delay_work,
		msecs_to_jiffies(queue_delayed_work_time));
}

static int cw_battery_get_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct cw_battery *cw_bat = container_of(psy,
				struct cw_battery, batt_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = cw_bat->capacity;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = cw_bat->status; 
		break;                 
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval= POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = cw_bat->voltage <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = cw_bat->voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = cw_bat->chg_enabled;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int cw_battery_set_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	const union power_supply_propval *val)
{
	int ret = 0;
	struct cw_battery *cw_bat = container_of(psy,
				struct cw_battery, batt_psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		sgm41511_enable_gpio_charger(!val->intval);
		cw_bat->chg_enabled = val->intval;
		power_supply_changed(&cw_bat->batt_psy);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void cw_battery_changed(struct power_supply *psy)
{
	struct cw_battery *cw_bat = container_of(psy,
				struct cw_battery, batt_psy);

	cancel_delayed_work(&cw_bat->battery_delay_work);
	schedule_delayed_work(&cw_bat->battery_delay_work,
		msecs_to_jiffies(100));
}

static enum power_supply_property cw_battery_properties[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
};

static int cw_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int cw2015_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int loop = 0;
	struct cw_battery *cw_bat;
	struct power_supply *usb_psy;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat) {
		cw_printk("cw_bat create fail!\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, cw_bat);

	cw_bat->client = client;
	cw_bat->capacity = 1;
	cw_bat->voltage = 0;
	cw_bat->status = 0;
	cw_bat->charger_mode = NO_CHARGING;
	cw_bat->change = 0;
	cw_bat->usb_psy = usb_psy;

	ret = cw_init(cw_bat);
	while ((loop++ < 3) && (ret != 0)) {
		msleep(200);
		ret = cw_init(cw_bat);
	}
	if (ret) {
		printk("%s : cw2015 init fail!\n", __func__);
		return ret;	
	}

	ret = cw_init_data(cw_bat);
	if (ret) {
		printk("%s : cw2015 init data fail!\n", __func__);
		return ret;
	}

	cw_bat->batt_psy.name		= CW_PROPERTIES;
	cw_bat->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	cw_bat->batt_psy.get_property	= cw_battery_get_property;
	cw_bat->batt_psy.set_property	= cw_battery_set_property;
	cw_bat->batt_psy.properties	= cw_battery_properties;
	cw_bat->batt_psy.num_properties	= ARRAY_SIZE(cw_battery_properties);
	cw_bat->batt_psy.external_power_changed = cw_battery_changed;
	cw_bat->batt_psy.property_is_writeable = cw_battery_is_writeable;

	ret = power_supply_register(&client->dev, &cw_bat->batt_psy);
	if (ret < 0) {
		pr_err("Unable to register batt_psy ret = %d\n", ret);
        return ret;
	}

	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->cwfg_workqueue,
		&cw_bat->battery_delay_work, msecs_to_jiffies(50));

	cw_printk("cw2015 driver probe success!\n");

	return 0;
}

static int cw_bat_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	read_persistent_clock(&suspend_time_before);
	cancel_delayed_work(&cw_bat->battery_delay_work);

	return 0;
}

static int cw_bat_resume(struct device *dev)
{	
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

	suspend_resume_mark = 1;
	read_persistent_clock(&after);
	after = timespec_sub(after, suspend_time_before);
	queue_delayed_work(cw_bat->cwfg_workqueue,
	&cw_bat->battery_delay_work, msecs_to_jiffies(2));

	return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
	.suspend  = cw_bat_suspend,
	.resume   = cw_bat_resume,
};

static int cw2015_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id cw2015_id_table[] = {
	{ CWFG_NAME, 0 },
	{ }
};

static struct of_device_id cw2015_match_table[] = {
	{ .compatible = "cellwise,cw2015", },
	{ },
};

static struct i2c_driver cw2015_driver = {
	.driver 	  = {
		.name = CWFG_NAME,
		.pm     = &cw_bat_pm_ops,
		.owner	= THIS_MODULE,
		.of_match_table = cw2015_match_table,
	},
	.probe = cw2015_probe,
	.remove = cw2015_remove,
	.id_table = cw2015_id_table,
};

static int __init cw215_init(void)
{
	return i2c_add_driver(&cw2015_driver);
}

static void __exit cw215_exit(void)
{
	i2c_del_driver(&cw2015_driver);
}

late_initcall_sync(cw215_init);
module_exit(cw215_exit);
MODULE_LICENSE("GPL");
