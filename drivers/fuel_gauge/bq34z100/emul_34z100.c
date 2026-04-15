#error NOT SUPPORTED YET

/**
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Emulator for bq34z100 fuel gauge
 */
#include <string.h>
#define DT_DRV_COMPAT ti_bq34z100

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(EMUL_BQ34Z100);

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/sys/byteorder.h>

#include "bq34z100.h"

/** Static configuration for the emulator */
struct bq34z100_emul_cfg {
	uint16_t i2c_addr;
};

static int emul_bq40z70_buffer_read(int reg, uint8_t *buf, size_t len)
{
	static const char manufacturer_name[] = "Texas Inst.";
	static const char device_name[] = "bq34z100";
	static const char device_chemistry[] = "LION";

	switch (reg) {
	case BQ34Z100_MANUFACTURERNAME:
		if (len < sizeof(manufacturer_name)) {
			return -EIO;
		}
		buf[0] = strlen(manufacturer_name);
		memcpy(&buf[1], manufacturer_name, strlen(manufacturer_name));
		break;

	case BQ34Z100_DEVICENAME:
		if (len < sizeof(device_name)) {
			return -EIO;
		}
		buf[0] = strlen(device_name);
		memcpy(&buf[1], device_name, strlen(device_name));
		break;

	case BQ34Z100_DEVICECHEMISTRY:
		if (len < sizeof(device_chemistry)) {
			return -EIO;
		}
		buf[0] = strlen(device_chemistry);
		memcpy(&buf[1], device_chemistry, strlen(device_chemistry));
		break;

	default:
		LOG_ERR("Buffer Read for reg 0x%x is not supported", reg);
		return -EIO;
	}

	return 0;
}

static int emul_bq34z100_write(const struct emul *target, uint8_t *buf, size_t len)
{
	LOG_ERR("Write operation is not currently supported");
	return -EIO;
}

static int emul_bq34z100_reg_read(const struct emul *target, int reg, uint16_t *val)
{
	switch (reg) {
	case BQ34Z100_MANUFACTURERACCESS:
		*val = 1;
		break;
	case BQ34Z100_ATRATE:
		*val = 0;
		break;
	case BQ34Z100_ATRATETIMETOEMPTY:
		*val = 0xFFFF;
		break;
	case BQ34Z100_TEMPERATURE:
		*val = 2980;
		break;
	case BQ34Z100_VOLTAGE:
		*val = 1;
		break;
	case BQ34Z100_BATTERYSTATUS:
		*val = 1;
		break;
	case BQ34Z100_CURRENT:
		*val = 1;
		break;
	case BQ34Z100_REMAININGCAPACITY:
		*val = 1;
		break;
	case BQ34Z100_FULLCHARGECAPACITY:
		*val = 1;
		break;
	case BQ34Z100_AVERAGECURRENT:
		*val = 1;
		break;
	case BQ34Z100_AVERAGETIMETOEMPTY:
		*val = 0xFFFF;
		break;
	case BQ34Z100_ATRATETIMETOFULL:
		*val = 0xFFFF;
		break;
	case BQ34Z100_BTPDISCHARGE:
		*val = 150;
		break;
	case BQ34Z100_BTPCHARGE:
		*val = 175;
		break;
	case BQ34Z100_CYCLECOUNT:
		*val = 1;
		break;
	case BQ34Z100_RELATIVESTATEOFCHARGE:
	case BQ34Z100_ABSOLUTESTATEOFCHARGE:
		*val = 100;
		break;
	case BQ34Z100_CHARGINGVOLTAGE:
	case BQ34Z100_CHARGINGCURRENT:
		*val = 1;
		break;
	case BQ34Z100_DESIGNCAPACITY:
		*val = 1;
		break;

	case BQ34Z100_DESIGNVOLTAGE:
		*val = 14400;
		break;

	case BQ34Z100_RUNTIMETOEMPTY:
		*val = 0xFFFF;
		break;

	case BQ34Z100_BATTERYMODE:
		*val = 0;
		break;

	case BQ34Z100_ATRATEOK:
		*val = 0;
		break;

	case BQ34Z100_REMAININGCAPACITYALARM:
		*val = 300;
		break;

	case BQ34Z100_REMAININGTIMEALARM:
		*val = 10;
		break;

	default:
		LOG_ERR("Unknown register 0x%x read", reg);
		return -EIO;
	}

	return 0;
}

static int emul_bq34z100_read(const struct emul *target, int reg, uint8_t *buf, size_t len)
{
	if (len <= 2) {
		/* Normal reads are maximum 1 or 2 bytes wide. */
		uint16_t val;
		int rc = emul_bq34z100_reg_read(target, reg, &val);

		if (rc) {
			return rc;
		}

		sys_put_le16(val, buf);
	} else {
		switch (reg) {
		case BQ34Z100_MANUFACTURERNAME:
		case BQ34Z100_DEVICECHEMISTRY:
		case BQ34Z100_DEVICENAME:
			return emul_bq40z70_buffer_read(reg, buf, len);
		default:
			LOG_ERR(" Buffer read only supported for string registers (i.e. "
				"manufacturer_name, device_chemistry, and device name)");
			return -EIO;
		}
	}

	return 0;
}

static int bq34z100_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs, int num_msgs,
				     int addr)
{
	int reg;
	int rc;
	const struct bq34z100_emul_cfg *cfg = (const struct bq34z100_emul_cfg *)(target->cfg);

	__ASSERT_NO_MSG(msgs && num_msgs);

	if (addr != cfg->i2c_addr) {
		LOG_ERR("I2C address (0x%2x) is not supported.", addr);
		return -EIO;
	}

	i2c_dump_msgs_rw(target->dev, msgs, num_msgs, addr, false);
	switch (num_msgs) {
	case 1:
		if (msgs->flags & I2C_MSG_READ) {
			LOG_ERR("Unexpected read");
			return -EIO;
		}

		return emul_bq34z100_write(target, msgs->buf, msgs->len);
	case 2:
		if (msgs->flags & I2C_MSG_READ) {
			LOG_ERR("Unexpected read");
			return -EIO;
		}
		if (msgs->len != 1) {
			LOG_ERR("Unexpected addr length %d", msgs->len);
			return -EIO;
		}
		reg = msgs->buf[0];

		/* Now process the 'read' part of the message */
		msgs++;
		if (msgs->flags & I2C_MSG_READ) {
			rc = emul_bq34z100_read(target, reg, msgs->buf, msgs->len);
			if (rc) {
				return rc;
			}
		} else {
			LOG_ERR("Second message must be an I2C read");
			return -EIO;
		}
		return rc;
	default:
		LOG_ERR("Invalid number of messages: %d", num_msgs);
		return -EIO;
	}

	return 0;
}

static const struct i2c_emul_api bq34z100_emul_api_i2c = {
	.transfer = bq34z100_emul_transfer_i2c,
};

/**
 * Set up a new emulator (I2C)
 *
 * @param target Emulation information
 * @param parent Device to emulate
 * @return 0 indicating success (always)
 */
static int emul_bq34z100_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(target);
	ARG_UNUSED(parent);

	return 0;
}

/*
 * Main instantiation macro.
 */
#define BQ34Z100_EMUL(n)                                                                            \
	static const struct bq34z100_emul_cfg bq34z100_emul_cfg_##n = {                              \
		.i2c_addr = DT_INST_REG_ADDR(n),                                                   \
	};                                                                                         \
	EMUL_DT_INST_DEFINE(n, emul_bq34z100_init, NULL, &bq34z100_emul_cfg_##n,                     \
			    &bq34z100_emul_api_i2c, NULL)

DT_INST_FOREACH_STATUS_OKAY(BQ34Z100_EMUL)
