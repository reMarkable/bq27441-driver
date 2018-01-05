#include <linux/device.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/power/bq27xxx_battery.h>

#include <linux/i2c.h>
#include <linux/debugfs.h>

#define BQ27441_CONTROL_STATUS  0x0000
#define BQ27441_DEVICE_TYPE     0x0001
#define BQ27441_FW_VERSION      0x0002
#define BQ27441_DM_CODE         0x0004
#define BQ27441_PREV_MACWRITE   0x0007
#define BQ27441_CHEM_ID         0x0008
#define BQ27441_BAT_INSERT      0x000C
#define BQ27441_BAT_REMOVE      0x000D
#define BQ27441_SET_HIBERNATE   0x0011
#define BQ27441_CLEAR_HIBERNATE 0x0012
#define BQ27441_SET_CFGUPDATE   0x0013
#define BQ27441_SHUTDOWN_ENABLE 0x001B
#define BQ27441_SHUTDOWN        0x001C
#define BQ27441_SEALED          0x0020
#define BQ27441_PULSE_SOC_INT   0x0023
#define BQ27441_RESET           0x0041
#define BQ27441_SOFT_RESET      0x0042

#define BQ27441_UNSEAL          0x8000

#define BQ27441_CONTROL_1       0x00
#define BQ27441_CONTROL_2       0x01
#define BQ27441_TEMPERATURE     0x02
#define BQ27441_VOLTAGE         0x04
#define BQ27441_FLAGS           0x06
#define BQ27441_FLAGS_CFGUPMODE (1 << 4)
#define BQ27441_FLAGS_ITPOR     (1 << 5)
#define BQ27441_NOMINAL_AVAIL_CAPACITY 0x08
#define BQ27441_FULL_AVAIL_CAPACITY    0x0a
#define BQ27441_REMAINING_CAPACITY     0x0c
#define BQ27441_FULL_CHG_CAPACITY      0x0e
#define BQ27441_AVG_CURRENT            0x10
#define BQ27441_STANDBY_CURRENT        0x12
#define BQ27441_MAXLOAD_CURRENT        0x14
#define BQ27441_AVERAGE_POWER          0x18
#define BQ27441_STATE_OF_CHARGE        0x1c
#define BQ27441_INT_TEMPERATURE        0x1e
#define BQ27441_STATE_OF_HEALTH        0x20

#define BQ27441_OPCONF_BATLOWEN (1 << 3)
#define BQ27441_OPCONF_GPIOPOL (1 << 3)

#define BQ27441_BLOCK_DATA_CHECKSUM 0x60
#define BQ27441_BLOCK_DATA_CONTROL  0x61
#define BQ27441_DATA_BLOCK_CLASS    0x3E
#define BQ27441_DATA_BLOCK          0x3F

#define BQ27441_OPCONFIG_1          0x40
#define BQ27441_OPCONFIG_2          0x41

#define BQ27441_DESIGN_CAPACITY_1   0x4A
#define BQ27441_DESIGN_CAPACITY_2   0x4B
#define BQ27441_DESIGN_ENERGY_1     0x4C
#define BQ27441_DESIGN_ENERGY_2     0x4D
#define BQ27441_TAPER_RATE_1        0x5B
#define BQ27441_TAPER_RATE_2        0x5C
#define BQ27441_TERMINATE_VOLTAGE_1 0x50
#define BQ27441_TERMINATE_VOLTAGE_2 0x51
#define BQ27441_V_CHG_TERM_1        0x41
#define BQ27441_V_CHG_TERM_2        0x42

#define BQ27441_BATTERY_LOW   15
#define BQ27441_BATTERY_FULL 100

#define BQ27441_MAX_REGS 0x7F

static struct bq27441_data {
	unsigned long full_capacity; /* in mAh */
	unsigned long full_energy; /* in mWh */
	unsigned long taper_rate;
	unsigned long terminate_voltage; /* in mV */
	unsigned long v_at_chg_term; /* in mV */
	const char *tz_name;
};

static struct bq27441_extended_cmd {
	u8 datablock[2];
	u8 command[32];
	u8 checksum;
	u16 wait_time;
};

static const struct bq27441_extended_cmd zerogravitas_golden_file[] = {
		{
			.datablock = {0x02, 0x00},
			.command = {
				0x02, 0x26, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xA5,
			.wait_time = 10,
		},
		{
			.datablock = {0x24, 0x00},
			.command = {
				0x00, 0x19, 0x28, 0x63, 0x5F, 0xFF, 0x62, 0x00,
				0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x69,
			.wait_time = 10,
		},
		{
			.datablock = {0x30, 0x00},
			.command = {
				0x0E, 0x74, 0xFD, 0xFF, 0x38, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x49,
			.wait_time = 10,
		},
		{
			.datablock = {0x31, 0x00},
			.command = {
				0x0A, 0x0F, 0x02, 0x05, 0x32, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xAD,
			.wait_time = 10,
		},
		{
			.datablock = {0x40, 0x00},
			.command = {
				0x25, 0xFC, 0x0F, 0x48, 0x00, 0x14, 0x04, 0x00,
				0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x66,
			.wait_time = 10,
		},
		{
			.datablock = {0x44, 0x00},
			.command = {
				0x05, 0x00, 0x32, 0x01, 0xC2, 0x14, 0x14, 0x00,
				0x03, 0x08, 0x98, 0x01, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x39,
			.wait_time = 10,
		},
		{
			.datablock = {0x50, 0x00},
			.command = {
				0x02, 0xBC, 0x01, 0x2C, 0x00, 0x1E, 0x00, 0xC8,
				0xC8, 0x14, 0x08, 0x00, 0x3C, 0x0E, 0x10, 0x00,
				0x0A, 0x46, 0x05, 0x14, 0x05, 0x0F, 0x03, 0x20,
				0x00, 0x64, 0x46, 0x50, 0x0A, 0x01, 0x90, 0x00},
			.checksum = 0xBB,
			.wait_time = 10,
		},
		{
			.datablock = {0x50, 0x01},
			.command = {
				0x64, 0x19, 0xDC, 0x5C, 0x60, 0x00, 0x7D, 0x00,
				0x04, 0x03, 0x19, 0x25, 0x0F, 0x14, 0x0A, 0x78,
				0x60, 0x28, 0x01, 0xF4, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x43, 0x80, 0x04, 0x01, 0x14, 0x00},
			.checksum = 0x2A,
			.wait_time = 10,
		},
		{
			.datablock = {0x50, 0x02},
			.command = {
				0x0B, 0x0B, 0xB8, 0x01, 0x2C, 0x0A, 0x01, 0x0A,
				0x00, 0x00, 0x00, 0xC8, 0x00, 0x64, 0x02, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xC1,
			.wait_time = 10,
		},
		{
			.datablock = {0x51, 0x00},
			.command = {
				0x00, 0xA7, 0x00, 0x64, 0x00, 0xFA, 0x00, 0x3C,
				0x3C, 0x01, 0xB3, 0xB3, 0x01, 0x90, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x8A,
			.wait_time = 10,
		},
		{
			.datablock = {0x52, 0x00},
			.command = {
				0x41, 0x8F, 0x00, 0x00, 0x00, 0x81, 0x0E, 0xDB,
				0x0E, 0xA8, 0x0B, 0xB8, 0x2B, 0x5C, 0x05, 0x3C,
				0x0C, 0x80, 0x00, 0xC8, 0x00, 0x32, 0x00, 0x14,
				0x03, 0xE8, 0x01, 0x01, 0x2C, 0x10, 0x04, 0x00},
			.checksum = 0xBD,
			.wait_time = 10,
		},
		{
			.datablock = {0x52, 0x01},
			.command = {
				0x0A, 0x10, 0x5E, 0xFF, 0xCE, 0xFF, 0xCE, 0x00,
				0x02, 0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xC8,
			.wait_time = 10,
		},
		{
			.datablock = {0x59, 0x00},
			.command = {
				0x00, 0x8A, 0x00, 0x8A, 0x00, 0x8A, 0x00, 0x98,
				0x00, 0x6B, 0x00, 0x58, 0x00, 0x5D, 0x00, 0x5E,
				0x00, 0x4F, 0x00, 0x46, 0x00, 0x59, 0x00, 0x69,
				0x00, 0xD1, 0x02, 0x26, 0x03, 0x6E, 0x00, 0x00},
			.checksum = 0x8A,
			.wait_time = 10,
		},
		{
			.datablock = {0x70, 0x00},
			.command = {
				0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xFF,
			.wait_time = 10,
		},

#if 0  /* Kaifa */
		{
			.datablock = {0x02, 0x00},
			.command = {
				0x02, 0x26, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xA5,
			.wait_time = 10,
		},
		{
			.datablock = {0x24, 0x00},
			.command = {
				0x00, 0x19, 0x28, 0x63, 0x5F, 0xFF, 0x62, 0x00,
				0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x69,
			.wait_time = 10,
		},
		{
			.datablock = {0x30, 0x00},
			.command = {
				0x0E, 0x74, 0xFD, 0xFF, 0x38, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x49,
			.wait_time = 10,
		},
		{
			.datablock = {0x31, 0x00},
			.command = {
				0x0A, 0x0F, 0x02, 0x05, 0x32, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xAD,
			.wait_time = 10,
		},
		{
			.datablock = {0x40, 0x00},
			.command = {
				0x25, 0xFC, 0x0F, 0x48, 0x00, 0x14, 0x04, 0x00,
				0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x66,
			.wait_time = 10,
		},
		{
			.datablock = {0x44, 0x00},
			.command = {
				0x05, 0x00, 0x32, 0x01, 0xC2, 0x14, 0x14, 0x00,
				0x03, 0x08, 0x98, 0x01, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x39,
			.wait_time = 10,
		},
		{
			.datablock = {0x50, 0x00},
			.command = {
				0x02, 0xBC, 0x01, 0x2C, 0x00, 0x1E, 0x00, 0xC8,
				0xC8, 0x14, 0x08, 0x00, 0x3C, 0x0E, 0x10, 0x00,
				0x0A, 0x46, 0x05, 0x14, 0x05, 0x0F, 0x03, 0x20,
				0x00, 0x64, 0x46, 0x50, 0x0A, 0x01, 0x90, 0x00},
			.checksum = 0xBB,
			.wait_time = 10,
		},
		{
			.datablock = {0x50, 0x01},
			.command = {
				0x64, 0x19, 0xDC, 0x5C, 0x60, 0x00, 0x7D, 0x00,
				0x04, 0x03, 0x19, 0x25, 0x0F, 0x14, 0x0A, 0x78,
				0x60, 0x28, 0x01, 0xF4, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x43, 0x80, 0x04, 0x01, 0x14, 0x00},
			.checksum = 0x2A,
			.wait_time = 10,
		},
		{
			.datablock = {0x50, 0x02},
			.command = {
				0x0B, 0x0B, 0xB8, 0x01, 0x2C, 0x0A, 0x01, 0x0A,
				0x00, 0x00, 0x00, 0xC8, 0x00, 0x64, 0x02, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xC1,
			.wait_time = 10,
		},
		{
			.datablock = {0x51, 0x00},
			.command = {
				0x00, 0xA7, 0x00, 0x64, 0x00, 0xFA, 0x00, 0x3C,
				0x3C, 0x01, 0xB3, 0xB3, 0x01, 0x90, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x8A,
			.wait_time = 10,
		},
		{
			.datablock = {0x52, 0x00},
			.command = {
				0x40, 0x00, 0x00, 0x00, 0x00, 0x81, 0x0E, 0xDB,
				0x0E, 0xA8, 0x0A, 0xF0, 0x28, 0x78, 0x05, 0x3C,
				0x0C, 0xE4, 0x00, 0xC8, 0x00, 0x32, 0x00, 0x14,
				0x03, 0xE8, 0x01, 0x01, 0x54, 0x10, 0x04, 0x00},
			.checksum = 0x71,
			.wait_time = 10,
		},
		{
			.datablock = {0x52, 0x01},
			.command = {
				0x0A, 0x10, 0x5E, 0xFF, 0xCE, 0xFF, 0xCE, 0x00,
				0x01, 0x02, 0xBC, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0x2E,
			.wait_time = 10,
		},
		{
			.datablock = {0x59, 0x00},
			.command = {
				0x00, 0x66, 0x00, 0x66, 0x00, 0x63, 0x00, 0x6B,
				0x00, 0x48, 0x00, 0x3B, 0x00, 0x3E, 0x00, 0x3F,
				0x00, 0x35, 0x00, 0x2F, 0x00, 0x3C, 0x00, 0x46,
				0x00, 0x8C, 0x01, 0x71, 0x02, 0x4C, 0x00, 0x00},
			.checksum = 0x33,
			.wait_time = 10,
		},
		{
			.datablock = {0x70, 0x00},
			.command = {
				0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
			.checksum = 0xFF,
			.wait_time = 10,
		},
#endif /* Kaifa */
};

static inline bool is_reg_valid(int reg)
{
	return (reg >= 0x00 && reg <= BQ27441_MAX_REGS);
}

static inline int read_byte(struct bq27xxx_device_info *di, int reg)
{
	if (!di || !is_reg_valid(reg))
		return -EINVAL;

	return di->bus.read(di, reg, true);
}

static inline int read_word(struct bq27xxx_device_info *di, int reg)
{
	if (!di || !is_reg_valid(reg))
		return -EINVAL;

	return di->bus.read(di, reg, false);
}

static inline int write_byte(struct bq27xxx_device_info *di, int reg,
		u8 data)
{
	int ret;
	if (!di || !is_reg_valid(reg))
		return -EINVAL;

	ret = di->bus.write(di, reg, &data, sizeof(data));
	usleep_range(100, 200);

	return ret;
}

static inline int write_word(struct bq27xxx_device_info *di, int reg,
		u16 data)
{
	int ret;
	unsigned char buf[2] = {(data & 0xff), (data >> 8)};

	if (!di || !is_reg_valid(reg))
		return -EINVAL;

	ret = di->bus.write(di, reg, buf, sizeof(buf));
	usleep_range(100, 200);

	return ret;
}

static inline int write_array(struct bq27xxx_device_info *di, int reg,
		const u8 *data, size_t len)
{
	int ret;

	if (!di || !is_reg_valid(reg))
		return -EINVAL;

	ret = di->bus.write(di, reg, data, len);;
	usleep_range(100, 200); /* This chip is slooooooow */

	return ret;
}

static inline int write_extended_cmd(struct bq27xxx_device_info *di,
		const struct bq27441_extended_cmd *cmd)
{
	int ret;
	u8 read_checksum;

	ret = write_array(di, BQ27441_DATA_BLOCK_CLASS, cmd->datablock,
			sizeof(cmd->datablock));
	if (ret < 0 || ret != sizeof(cmd->datablock) + 1) {
		dev_warn(di->dev,
				"Failed to write datablock to %02X-%02X (id %u), ret %d\n",
				cmd->datablock[0], cmd->datablock[1], cmd->datablock[0], ret);
		return ret;
	}

	ret = write_array(di, 0x40, cmd->command, sizeof(cmd->command));
	if (ret < 0 || ret != sizeof(cmd->command) + 1) {
		dev_warn(di->dev,
				"Failed to write command to %02X-%02X (id %u), ret %d\n",
				cmd->datablock[0], cmd->datablock[1], cmd->datablock[0], ret);
		return ret;
	}

	ret = write_array(di, BQ27441_BLOCK_DATA_CHECKSUM, &cmd->checksum,
			sizeof(cmd->checksum));
	if (ret < 0 || ret != sizeof(cmd->checksum) + 1) {
		dev_warn(di->dev,
				"Failed to write checksum to %02X-%02X (id: %u), ret %d\n",
				cmd->datablock[0], cmd->datablock[1], cmd->datablock[0], ret);
		return ret;
	}

	usleep_range(cmd->wait_time * 1000, cmd->wait_time * 1100);

	ret = write_array(di, BQ27441_DATA_BLOCK_CLASS, cmd->datablock,
			sizeof(cmd->datablock));
	if (ret < 0 || ret != sizeof(cmd->datablock) + 1) {
		dev_warn(di->dev,
				"Failed to write datablock second time to %02X-%02X (id: %u), ret %d\n",
				cmd->datablock[0], cmd->datablock[1], cmd->datablock[0], ret);
		return ret;
	}

	ret = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (ret < 0) {
		dev_warn(di->dev,
				"Failed to read checksum for %02X-%02X (id: %u), ret %d\n",
				cmd->datablock[0], cmd->datablock[1], cmd->datablock[0], ret);
		return ret;
	}

	read_checksum = ret & 0xFF;
	if (read_checksum != cmd->checksum) {
		dev_warn(di->dev,
				"Failed to write to %02X-%02X (id: %u), checksum %02x read back %02x\n",
				cmd->datablock[0], cmd->datablock[1], cmd->datablock[0],
				cmd->checksum, read_checksum);
		return -EINVAL;
	}

	dev_info(di->dev,
			"Happily wrote to %02X-%02X (id: %u)\n",
			cmd->datablock[0], cmd->datablock[1], cmd->datablock[0]);

	return 0;
}

static void of_bq27441_parse_platform_data(struct i2c_client *client,
				struct bq27441_data *pdata)
{
	u32 tmp;
	char const *pstr;
	struct device_node *np = client->dev.of_node;

	if (!of_property_read_u32(np, "design-capacity", &tmp))
		pdata->full_capacity = (unsigned long)tmp;

	if (!of_property_read_u32(np, "design-energy", &tmp))
		pdata->full_energy = (unsigned long)tmp;

	if (!of_property_read_u32(np, "taper-rate", &tmp))
		pdata->taper_rate = (unsigned long)tmp;

	if (!of_property_read_u32(np, "terminate-voltage", &tmp))
		pdata->terminate_voltage = (unsigned long)tmp;

	if (!of_property_read_u32(np, "v-at-chg-term", &tmp))
		pdata->v_at_chg_term = (unsigned long)tmp;

	if (!of_property_read_string(np, "tz-name", &pstr))
		pdata->tz_name = pstr;
	else
		dev_err(&client->dev, "Failed to read tz-name\n");

	dev_info(&client->dev, "Read device tree, got:\n"
			"    design-capacity %lu mAh\n"
			"    design-energy %lu mW\n"
			"    taper-rate %lu\n"
			"    terminate-voltage %lu mV\n"
			"    v-at-chg-term %lu mV\n"
			"    tz-name \"%s\"\n",
			pdata->full_capacity,
			pdata->full_energy,
			pdata->taper_rate,
			pdata->terminate_voltage,
			pdata->v_at_chg_term,
			pdata->tz_name);
}

static inline int config_mode_start(struct bq27xxx_device_info *di)
{
	int ret;
	int flags_lsb;
	int control_status;
	unsigned long timeout;

	ret = read_word(di, BQ27441_FLAGS);
	if (ret < 0)
		return ret;

	dev_info(di->dev, "Flags: 0x%04x\n", ret);
	flags_lsb = (ret & 0xff);

	if (flags_lsb & BQ27441_FLAGS_CFGUPMODE) {
		dev_info(di->dev, "Device already in config mode\n");
		return 0;
	}

	/* unseal the fuel gauge for data access if needed */

	ret = write_word(di, BQ27441_CONTROL_1, BQ27441_CONTROL_STATUS);
	if (ret < 0)
		return ret;

	ret = read_word(di, BQ27441_CONTROL_1);
	if (ret < 0)
		return ret;

	dev_info(di->dev, "Control status before seal: 0x%04x\n", ret);
	control_status = ret;

	if (control_status & 0x2000) {
		ret = write_word(di, BQ27441_CONTROL_1, BQ27441_UNSEAL);
		if (ret < 0)
			return ret;

		ret = write_word(di, BQ27441_CONTROL_1, BQ27441_UNSEAL);
		if (ret < 0)
			return ret;
	}
	else
		dev_info(di->dev, "Device already unsealed\n");

	usleep_range(1000, 2000);

	ret = write_word(di, BQ27441_CONTROL_1, BQ27441_CONTROL_STATUS);
	if (ret < 0)
		return ret;

	ret = read_word(di, BQ27441_CONTROL_1);
	if (ret < 0)
		return ret;

	dev_info(di->dev, "Control status after seal: 0x%04x\n", ret);

	/* Set fuel gauge in config mode */
	ret = write_word(di, BQ27441_CONTROL_1, BQ27441_SET_CFGUPDATE);
	if (ret < 0)
		return ret;

	/* Wait for config mode */
	timeout = jiffies + HZ;
	flags_lsb = 0;
	while (!flags_lsb) {
		ret = read_byte(di, BQ27441_FLAGS);
		if (ret < 0)
			return ret;

		flags_lsb = (ret & BQ27441_FLAGS_CFGUPMODE);
		dev_warn(di->dev, "flags_lsb %02x ret %02x\n", flags_lsb, ret);

		if (time_after(jiffies, timeout)) {
			dev_warn(di->dev, "Timeout waiting for cfg update\n");
			return -EIO;
		}
		usleep_range(1000, 2000);
	}

	return 0;
}

static inline int config_mode_stop(struct bq27xxx_device_info *di)
{
	int ret;
	int flags_lsb;
	unsigned long timeout = jiffies + HZ;

	ret = read_byte(di, BQ27441_FLAGS);
	if (ret < 0)
		return ret;

	if (ret & BQ27441_FLAGS_CFGUPMODE) {
		dev_info(di->dev, "Exiting config mode by soft reset\n");

		ret = write_word(di, BQ27441_CONTROL_1, BQ27441_SOFT_RESET);
		if (ret < 0)
			return ret;

		/* Wait for config mode to exit */
		timeout = jiffies + HZ;
		flags_lsb = BQ27441_FLAGS_CFGUPMODE;
		while (flags_lsb) {
			ret = read_byte(di, BQ27441_FLAGS);
			if (ret < 0)
				return ret;

			flags_lsb = (ret & BQ27441_FLAGS_CFGUPMODE);

			if (time_after(jiffies, timeout)) {
				dev_warn(di->dev, "Timeout waiting for cfg update stop\n");
				return -EIO;
			}
			usleep_range(1000, 2000);
		}
	}

	/* seal the fuel gauge */
	ret = write_word(di, BQ27441_CONTROL_1, BQ27441_SEALED);
	if (ret < 0)
		return ret;

	return 0;
}

#ifdef CONFIG_DEBUG_FS

static inline int toggle_gpiopol(struct bq27xxx_device_info *di)
{
	int ret;
	int old_csum;
	int new_csum;
	int temp_csum;
	u8 opconfig1;
	u8 old_opconfig1;

	dev_info(di->dev, "bq27441_toggle_gpiopol triggered\n");

	ret = config_mode_start(di);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to start config mode, ret %d\n", ret);
	}

	/* Enable block mode */
	ret = write_byte(di, BQ27441_BLOCK_DATA_CONTROL, 0x00);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to enable block mode, ret %d\n", ret);
		return ret;
	}

	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0040);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to change block, ret %d\n", ret);
		return ret;
	}

	usleep_range(100, 200);

	old_csum = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (old_csum < 0) {
		dev_warn(di->dev, "Unable to read old checksum, ret %d\n", ret);
		return old_csum;
	}

	ret = read_byte(di, BQ27441_OPCONFIG_1);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to read opconfig byte 1, ret %d\n", ret);
		return ret;
	}

	old_opconfig1 = (ret & 0xff);
	opconfig1 = (old_opconfig1 ^ BQ27441_OPCONF_GPIOPOL);

	ret = write_byte(di, BQ27441_OPCONFIG_1, opconfig1);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write opconfig 1 with temp config, ret %d\n", ret);
		return ret;
	}

	/* Write opconfig block check sum */
	temp_csum = (255 - old_csum - old_opconfig1) % 256;
	new_csum = 255 - ((temp_csum + opconfig1) % 256);

	ret = write_byte(di, BQ27441_BLOCK_DATA_CHECKSUM, new_csum);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write new checksum, ret %d\n", ret);
		return ret;
	}

	usleep_range(10000, 20000);

	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0040);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to change block, ret %d\n", ret);
		return ret;
	}

	usleep_range(100, 200);

	temp_csum = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (temp_csum < 0) {
		dev_warn(di->dev, "Unable to read back checksum, ret %d", ret);
		return temp_csum;
	}

	if (temp_csum != new_csum) {
		dev_warn(di->dev, "Checksum readback mismatch, want %02x has %02x\n",
				new_csum, temp_csum);
		return -EIO;
	}

	/* Approx pulse length */
	usleep_range(100000, 200000);

	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0040);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to change block, ret %d\n", ret);
		return ret;
	}

	ret = write_byte(di, BQ27441_OPCONFIG_1, old_opconfig1);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write opconfig byte 1 back to old value, ret %d\n", ret);
		return ret;
	}

	ret = write_byte(di, BQ27441_BLOCK_DATA_CHECKSUM, old_csum);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write checksum back to old value, ret %d\n", ret);
		return ret;
	}

	usleep_range(10000, 20000);

	/* Verify check sum */
	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0040);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to read back checksum of old value\n", ret);
		return ret;
	}

	temp_csum = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (temp_csum < 0)
		return temp_csum;

	if (temp_csum != old_csum) {
		dev_warn(di->dev, "Checksum readback mismatch, want %02x has %02x\n",
				new_csum, temp_csum);
		return -EIO;
	}

	ret = config_mode_stop(di);
	if (ret < 0)
		return ret;

	return 0;
}

static inline int toggle_gpout(struct bq27xxx_device_info *di)
{
	int ret;
	int old_csum;
	int new_csum;
	int temp_csum;
	u8 opconfig2;
	u8 old_opconfig2;

	dev_info(di->dev, "toggle_gpout triggered\n");

	ret = config_mode_start(di);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to start config mode, ret %d\n", ret);
	}

	/* Enable block mode */
	ret = write_byte(di, BQ27441_BLOCK_DATA_CONTROL, 0x00);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to enable block mode, ret %d\n", ret);
		return ret;
	}

	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0040);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to change block, ret %d\n", ret);
		return ret;
	}

	usleep_range(100, 200);

	old_csum = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (old_csum < 0) {
		dev_warn(di->dev, "Unable to read old checksum, ret %d\n", ret);
		return old_csum;
	}

	ret = read_byte(di, BQ27441_OPCONFIG_2);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to read opconfig byte 2, ret %d\n", ret);
		return ret;
	}

	old_opconfig2 = (ret & 0xff);
	opconfig2 = (old_opconfig2 & ~BQ27441_OPCONF_BATLOWEN);

	ret = write_byte(di, BQ27441_OPCONFIG_2, opconfig2);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write opconfig 2 with temp config, ret %d\n", ret);
		return ret;
	}

	/* Write opconfig block check sum */
	temp_csum = (255 - old_csum - (old_opconfig2)) % 256;
	new_csum = 255 - ((temp_csum + (opconfig2)) % 256);

	ret = write_byte(di, BQ27441_BLOCK_DATA_CHECKSUM, new_csum);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write new checksum, ret %d\n", ret);
		return ret;
	}

	usleep_range(10000, 20000);

	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0040);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to change block, ret %d\n", ret);
		return ret;
	}

	usleep_range(100, 200);

	temp_csum = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (temp_csum < 0) {
		dev_warn(di->dev, "Unable to read back checksum, ret %d", ret);
		return temp_csum;
	}

	if (temp_csum != new_csum) {
		dev_warn(di->dev, "Checksum readback mismatch, want %02x has %02x\n",
				new_csum, temp_csum);
		return -EIO;
	}

	/* Trigger pulse */
	ret = write_word(di, BQ27441_CONTROL_1, BQ27441_PULSE_SOC_INT);
	if (ret < 0)
		dev_warn(di->dev, "Could not trigger interrupt pulse, ret %d\n", ret);

	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0040);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to change datablock to 0x0040, ret %d", ret);
		return ret;
	}

	usleep_range(100, 200);

	opconfig2 |= BQ27441_OPCONF_BATLOWEN;
	ret = write_byte(di, BQ27441_OPCONFIG_2, opconfig2);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write opconfig byte 2 back to old value, ret %d\n", ret);
		return ret;
	}

	ret = write_byte(di, BQ27441_BLOCK_DATA_CHECKSUM, old_csum);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write checksum back to old value\n", ret);
		return ret;
	}

	usleep_range(10000, 20000);

	/* Verify check sum */
	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0040);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to read back checksum of old value\n", ret);
		return ret;
	}

	temp_csum = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (temp_csum < 0)
		return temp_csum;

	if (temp_csum != old_csum) {
		dev_warn(di->dev, "Checksum readback mismatch, want %02x has %02x\n",
				new_csum, temp_csum);
		return -EIO;
	}

	ret = config_mode_stop(di);
	if (ret < 0)
		return ret;

	return 0;
}

struct dentry *zero_dir, *toggle_file, *polarity_file;

static ssize_t toggle_gpout_debugfs(struct file *fp, const char __user *userbuf,
                                size_t count, loff_t *offset)
{
	int ret;
	struct bq27xxx_device_info *di = fp->private_data;

	if (!di)
		return -EIO;

	if (count < 1 || userbuf[0] != '1') {
		return -EINVAL;
	}

	mutex_lock(&di->lock);
	ret = toggle_gpout(di);
	mutex_unlock(&di->lock);

	if (ret >= 0)
		return count;
	else
		return ret;
}

static ssize_t toggle_polarity_debugfs(struct file *fp, const char __user *userbuf,
                                size_t count, loff_t *offset)
{
	int ret;
	struct bq27xxx_device_info *di = fp->private_data;

	if (!di)
		return -EIO;

	if (count < 1 || userbuf[0] != '1') {
		return -EINVAL;
	}

	mutex_lock(&di->lock);
	ret = toggle_gpiopol(di);
	mutex_unlock(&di->lock);

	if (ret >= 0)
		return count;
	else
		return ret;
}

static const struct file_operations polarity_fops = {
		.open = simple_open,
		.write = toggle_polarity_debugfs,
		.owner = THIS_MODULE,
};

static const struct file_operations toggle_fops = {
		.open = simple_open,
		.write = toggle_gpout_debugfs,
		.owner = THIS_MODULE,
};

static int bq27441_create_debugfs(struct bq27xxx_device_info *di)
{
	zero_dir = debugfs_create_dir("zero-gravitas", NULL);
	toggle_file = debugfs_create_file("toggle_gpout",
			S_IWUGO, zero_dir, di, &toggle_fops);
	polarity_file = debugfs_create_file("bq27441_toggle_polarity",
			S_IWUGO, zero_dir, di, &polarity_fops);

	return 0;
}
#endif /* CONFIG_DEBUG_FS */

static int check_fw_version(struct bq27xxx_device_info *di)
{
	/* Check firmware version
		W: AA 00 01 00
		C: AA 00 21 04
		W: AA 00 02 00
		C: AA 00 09 01
	 */

	int ret;
	int device_type;
	int fw_version;

	ret = write_word(di, BQ27441_CONTROL_1, BQ27441_DEVICE_TYPE);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);

	device_type = read_word(di, BQ27441_CONTROL_1);
	if (device_type < 0)
		return device_type;

	ret = write_word(di, BQ27441_CONTROL_1, BQ27441_FW_VERSION);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);

	fw_version = read_word(di, BQ27441_CONTROL_1);
	if (fw_version < 0)
		return fw_version;

	dev_info(di->dev, "Device type %04X, Firmware version %04X\n",
			device_type, fw_version);

	ret = 0;
	if (device_type != 0x0421) {
		dev_warn(di->dev, "Unsupported device type detected\n");
		ret = -EFAULT;
	}

	if (fw_version != 0x0109) {
		dev_warn(di->dev, "Unsupported firmware version detected\n");
		ret = -EFAULT;
	}

	return ret;
}

int bq27441_init(struct bq27xxx_device_info *di)
{
	int ret;
	int checksum;
	int design_capacity;
	int design_energy;
	int taper_rate;
	int terminate_voltage;
	int flags_lsb;
	int i;

	struct bq27441_data bdata = {0, 0, 0, 0, 0, NULL};
	struct i2c_client *client = to_i2c_client(di->dev);

	of_bq27441_parse_platform_data(client, &bdata);

	mutex_lock(&di->lock);

	flags_lsb = read_byte(di, BQ27441_FLAGS);

	if (check_fw_version(di) < 0)
		goto fail;

#ifdef CONFIG_DEBUG_FS
	if (bq27441_create_debugfs(di) < 0)
		dev_warn(di->dev, "Failed to create debugfs\n");
#endif /* CONFIG_DEBUG_FS */

	if (config_mode_start(di) < 0)
		goto fail;

	/* Enable block mode */
	if (write_byte(di, BQ27441_BLOCK_DATA_CONTROL, 0x00) < 0)
		goto fail;

	/* Dump configuration */
	for (i = 0; i < ARRAY_SIZE(zerogravitas_golden_file); i++) {
		write_extended_cmd(di, &zerogravitas_golden_file[i]);
	}

	/* Read back stuff */
	if (write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0052) < 0)
		goto fail;

	usleep_range(1000, 2000);
	checksum = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);

	/* read all the old values that we want to update */

	design_capacity = read_word(di, BQ27441_DESIGN_CAPACITY_1);
	design_energy = read_word(di, BQ27441_DESIGN_ENERGY_1);
	taper_rate = read_word(di, BQ27441_TAPER_RATE_1);
	terminate_voltage = read_word(di, BQ27441_TERMINATE_VOLTAGE_1);

	dev_info(di->dev,
			"Read current values on device:\n"
			"    capacity: %d mAh\n"
			"    energy: %d mW\n"
			"    taper_rate: %d\n"
			"    terminate_voltage: %d mV\n"
			"    itpor flag: %d\n"
			"    checksum: %x\n",
			be16_to_cpu(design_capacity),
			be16_to_cpu(design_energy),
			be16_to_cpu(taper_rate),
			be16_to_cpu(terminate_voltage),
			(flags_lsb & BQ27441_FLAGS_ITPOR),
			checksum);

	ret = config_mode_stop(di);
	mutex_unlock(&di->lock);
	return ret;

fail:
	dev_warn(di->dev, "Failed to initialize\n");
	mutex_unlock(&di->lock);
	return -EIO;
}
EXPORT_SYMBOL_GPL(bq27441_init);

void bq27441_exit(struct bq27xxx_device_info *di)
{
	debugfs_remove_recursive(zero_dir);
}
EXPORT_SYMBOL_GPL(bq27441_exit);

MODULE_AUTHOR("Lars <lars.ivar.miljeteig@remarkable.no>");
MODULE_DESCRIPTION("BQ27441 battery monitor driver");
MODULE_LICENSE("GPL");
