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

#include <linux/debugfs.h>

#include "bq27xxx_battery.h"

#define CONFIG_VERSION 5

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

static inline int control_read(struct bq27xxx_device_info *di, const u16 addr)
{
	int ret;
	ret = write_word(di, BQ27441_CONTROL_1, addr);
	if (ret < 0)
		return ret;

	usleep_range(100, 200);

	return read_word(di, BQ27441_CONTROL_1);
}

static inline int control_write(struct bq27xxx_device_info *di, const u16 cmd)
{
	return write_word(di, BQ27441_CONTROL_1, cmd);
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

static inline int read_extended_byte(struct bq27xxx_device_info *di,
		u8 dataclass, u8 datablock, u8 offset)
{
	int ret;
	u8 dataclassblock[] = {dataclass, datablock};

	if (offset >= 32)
		return -EINVAL;

	ret = write_array(di, BQ27441_DATA_BLOCK_CLASS, dataclassblock,
			sizeof(dataclassblock));
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	return read_byte(di, 0x40 + offset);
}

static inline int write_extended_byte(struct bq27xxx_device_info *di,
		u8 dataclass, u8 datablock, u8 offset, u8 data)
{
	int ret;
	u8 old_checksum;
	u8 read_checksum;
	u8 old_data;
	int new_checksum;
	int temp_checksum;
	u8 dataclassblock[] = {dataclass, datablock};

	if (offset >= 32)
		return -EINVAL;

	ret = write_array(di, BQ27441_DATA_BLOCK_CLASS, dataclassblock,
			sizeof(dataclassblock));
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	ret = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (ret < 0)
		return ret;

	old_checksum = ret & 0xff;

	ret = read_byte(di, 0x40 + offset);
	if (ret < 0)
		return ret;

	old_data = ret & 0xff;

	temp_checksum = (255 - old_checksum - old_data) % 256;
	new_checksum = 255 - ((temp_checksum + data) % 256);

	ret = write_byte(di, 0x40 + offset, data);
	if (ret < 0)
		return ret;

	ret = write_byte(di, BQ27441_BLOCK_DATA_CHECKSUM, new_checksum);
	if (ret < 0)
		return ret;

	usleep_range(10000, 11000);

	ret = write_array(di, BQ27441_DATA_BLOCK_CLASS, dataclassblock,
			sizeof(dataclassblock));
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);

	ret = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (ret < 0)
		return ret;

	read_checksum = ret & 0xFF;
	if (read_checksum != new_checksum) {
		dev_warn(di->dev,
				"Failed to write to %02X-%02X (id: %u), checksum %02x read back %02x\n",
				dataclass, datablock, dataclass,
				new_checksum, read_checksum);
		return -EINVAL;
	}

	dev_info(di->dev,
			"Happily wrote to %02X-%02X (id: %u)\n",
			dataclass, datablock, dataclass);

	return 0;
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

	ret = control_read(di, BQ27441_CONTROL_STATUS);
	if (ret < 0)
		return ret;

	dev_info(di->dev, "Control status before seal: 0x%04x\n", ret);
	control_status = ret;

	if (control_status & 0x2000) {
		ret = control_write(di, BQ27441_UNSEAL);
		if (ret < 0)
			return ret;

		ret = control_write(di, BQ27441_UNSEAL);
		if (ret < 0)
			return ret;
	}
	else
		dev_info(di->dev, "Device already unsealed\n");

	usleep_range(1000, 2000);

	ret = control_read(di, BQ27441_CONTROL_STATUS);
	if (ret < 0)
		return ret;

	dev_info(di->dev, "Control status after seal: 0x%04x\n", ret);

	/* Set fuel gauge in config mode */
	ret = control_write(di, BQ27441_SET_CFGUPDATE);
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

	/* Enable block mode */
	ret = write_byte(di, BQ27441_BLOCK_DATA_CONTROL, 0x00);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to enable block mode, ret %d\n", ret);
		return ret;
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

		ret = control_write(di, BQ27441_SOFT_RESET);
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
	ret = control_write(di, BQ27441_SEALED);
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
		return ret;
	}

	ret = read_extended_byte(di, 0x40, 0x00, 0);
	if (ret < 0)
		return ret;

	old_opconfig1 = (ret & 0xff);
	opconfig1 = old_opconfig1 ^ BQ27441_OPCONF_GPIOPOL;

	ret = write_extended_byte(di, 0x40, 0x00, 0, opconfig1);
	if (ret < 0)
		return ret;

	ret = config_mode_stop(di);
	if (ret < 0)
		return ret;

	return 0;
}

struct dentry *zero_dir, *polarity_file;

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

static int bq27441_create_debugfs(struct bq27xxx_device_info *di)
{
	zero_dir = debugfs_create_dir("zero-gravitas", NULL);
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

	device_type = control_read(di, BQ27441_DEVICE_TYPE);
	if (device_type < 0)
		return device_type;

	fw_version = control_read(di, BQ27441_FW_VERSION);
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

static int configure(struct bq27xxx_device_info *di)
{
	int ret;
	int checksum;
	int design_capacity;
	int design_energy;
	int taper_rate;
	int terminate_voltage;
	int flags_lsb;
	int i;

	flags_lsb = read_byte(di, BQ27441_FLAGS);

	ret = config_mode_start(di);
	if (ret < 0) {
		dev_warn(di->dev, "Failed to enter config mode\n");
		return ret;
	}

	/* Dump configuration */
	for (i = 0; i < ARRAY_SIZE(zerogravitas_golden_file); i++) {
		ret = write_extended_cmd(di, &zerogravitas_golden_file[i]);
		if (ret < 0)
			return ret;
	}

	/* Read back stuff */
	ret = write_word(di, BQ27441_DATA_BLOCK_CLASS, 0x0052);
	if (ret < 0) {
		dev_warn(di->dev, "Failed to read back data\n");
		return ret;
	}

	usleep_range(1000, 2000);
	checksum = read_byte(di, BQ27441_BLOCK_DATA_CHECKSUM);
	if (checksum < 0) {
		dev_warn(di->dev, "Failed to read back data\n");
		return ret;
	}

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

	ret = write_extended_byte(di, 0x40, 0x00, 3, CONFIG_VERSION);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to write BQ27441_DM_CODE, ret %d\n", ret);
		return ret;
	}

	usleep_range(1000, 2000);
	ret = control_read(di, BQ27441_DM_CODE);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to read back BQ27441_DM_CODE, ret %d\n", ret);
		return ret;
	}
	dev_info(di->dev, "BQ27441_DM_CODE read back %04X\n", ret);

	ret = config_mode_stop(di);
	return ret;
}

int bq27441_init(struct bq27xxx_device_info *di)
{
	int ret;

	mutex_lock(&di->lock);

	ret = check_fw_version(di);
	if (ret < 0)
		goto done;

#ifdef CONFIG_DEBUG_FS
	if (bq27441_create_debugfs(di) < 0)
		dev_warn(di->dev, "Failed to create debugfs\n");
#endif /* CONFIG_DEBUG_FS */

	ret = control_read(di, BQ27441_DM_CODE);
	if (ret < 0) {
		dev_warn(di->dev, "Unable to read BQ27441_DM_CODE, ret %d\n", ret);
		goto done;
	}
	dev_info(di->dev, "Configuration version %d\n", ret);

	if ((ret & 0xff) != CONFIG_VERSION) {
		ret = configure(di);
	}

done:
	mutex_unlock(&di->lock);

	if (ret < 0)
		dev_warn(di->dev, "Failed to initialize\n");

	return ret;
}
EXPORT_SYMBOL_GPL(bq27441_init);

void bq27441_exit(struct bq27xxx_device_info *di)
{
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(zero_dir);
#endif /* CONFIG_DEBUG_FS */
}
EXPORT_SYMBOL_GPL(bq27441_exit);

MODULE_AUTHOR("Lars <lars.ivar.miljeteig@remarkable.no>");
MODULE_DESCRIPTION("BQ27441 battery monitor driver");
MODULE_LICENSE("GPL");
