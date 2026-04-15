#define DT_DRV_COMPAT ti_bq34z100

#include "bq34z100.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>



LOG_MODULE_REGISTER(BQ34Z100, CONFIG_FUEL_GAUGE_LOG_LEVEL);

/* Standard Commands (Direct Access) */
#define BQ34Z_REG_CONTROL      0x00 /* Subcommands write here */
#define BQ34Z_REG_SOC          0x02
#define BQ34Z_REG_TEMP         0x06
#define BQ34Z_REG_VOLT         0x08
#define BQ34Z_REG_CURR         0x0A

struct bq34z100_config {
	struct i2c_dt_spec i2c;
};

/* Helper: Read a 16-bit little-endian register */
static int bq34z_read_16(const struct device *dev, uint8_t reg, uint16_t *val)
{
	const struct bq34z100_config *config = dev->config;
	uint8_t data[2];
	int ret;

	ret = i2c_burst_read_dt(&config->i2c, reg, data, 2);
	if (ret < 0) return ret;

	*val = (data[1] << 8) | data[0];
	return 0;
}

/* Data Flash Interface Registers */
#define BQ34Z_REG_BLOCK_CONTROL    0x61
#define BQ34Z_REG_DATA_CLASS       0x3E
#define BQ34Z_REG_DATA_BLOCK       0x3F
#define BQ34Z_REG_BLOCK_DATA_START 0x40
#define BQ34Z_REG_BLOCK_CHECKSUM   0x60

/* Example Subclass IDs */
#define BQ34Z_SUBCLASS_STATE       82  /* Contains Cycle Count */
#define BQ34Z_SUBCLASS_REGISTERS   64  /* Contains Design Capacity */

static int bq34z_read_data_flash(const struct device *dev, uint8_t class_id, 
                                 uint8_t block_id, uint8_t *dest)
{
    const struct bq34z100_config *config = dev->config;
    int ret;

    /* 1. Enable Block Data Control */
    ret = i2c_reg_write_byte_dt(&config->i2c, BQ34Z_REG_BLOCK_CONTROL, 0x00);
    if (ret < 0) return ret;

    /* 2. Set the Class ID (Subclass) */
    ret = i2c_reg_write_byte_dt(&config->i2c, BQ34Z_REG_DATA_CLASS, class_id);
    if (ret < 0) return ret;

    /* 3. Set the Data Block (typically 0x00 for the first 32 bytes) */
    ret = i2c_reg_write_byte_dt(&config->i2c, BQ34Z_REG_DATA_BLOCK, block_id);
    if (ret < 0) return ret;

    /* 4. Small delay for clock stretching - the gauge is slow here */
    k_msleep(5);

    /* 5. Read the 32-byte block */
    return i2c_burst_read_dt(&config->i2c, BQ34Z_REG_BLOCK_DATA_START, dest, 32);
}

static int bq34z100_get_prop(const struct device *dev, fuel_gauge_prop_t prop,
			     union fuel_gauge_prop_val *val)
{
	int ret = 0;
	uint16_t val16;

	switch (prop) {
	case FUEL_GAUGE_VOLTAGE:
		ret = bq34z_read_16(dev, BQ34Z_REG_VOLT, &val16);
		/* Main branch expects mV for voltage_uv? 
                   Actually, check your specific header: usually it's uV. */
		val->voltage = val16 * 1000;
		break;

	case FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE:
		/* SOC is a single byte at 0x02 */
		uint8_t soc;
		ret = i2c_reg_read_byte_dt(&((struct bq34z100_config *)dev->config)->i2c, 
					   BQ34Z_REG_SOC, &soc);
		val->relative_state_of_charge = soc;
		break;

	case FUEL_GAUGE_TEMPERATURE:
		ret = bq34z_read_16(dev, BQ34Z_REG_TEMP, &val16);
		/* BQ34Z100 returns 0.1°K. Zephyr expects 0.1°C (decikelvins to decicelcius) */
		val->temperature = val16 - 2732;
		break;

	case FUEL_GAUGE_CURRENT:
		ret = bq34z_read_16(dev, BQ34Z_REG_CURR, &val16);
		/* Convert mA to uA. BQ34Z100 current is a signed 16-bit int */
		val->current = (int16_t)val16 * 1000;
		break;

	case FUEL_GAUGE_CYCLE_COUNT:
		/* Using the Data Flash helper we discussed */
		uint8_t block[32];
		ret = bq34z_read_data_flash(dev, BQ34Z_SUBCLASS_STATE, 0x00, block);
		if (ret == 0) {
			/* Cycle count is 2 bytes at offset 0 of subclass 82 */
			val->cycle_count = (block[0] << 8) | block[1];
		}
		break;

	default:
		return -ENOTSUP;
	}

	return ret;
}

/* Default TI Keys */
#define BQ34Z_KEY_UNSEAL_1    0x0414
#define BQ34Z_KEY_UNSEAL_2    0x3672
#define BQ34Z_KEY_SEAL        0x0020

static int bq34z100_set_seal_state(const struct device *dev, uint16_t key)
{
	const struct bq34z100_config *config = dev->config;
	uint8_t msg[2];

	msg[0] = key & 0xFF;        /* LSB */
	msg[1] = (key >> 8) & 0xFF; /* MSB */

	return i2c_write_dt(&config->i2c, msg, 2);
}

static int bq34z_write_data_flash_block(const struct device *dev, uint8_t class_id,
                                        uint8_t block_id, uint8_t *data)
{
	const struct bq34z100_config *config = dev->config;
	uint8_t checksum = 0;
	int ret;

	/* 1. Setup pointers as we did for reading */
	i2c_reg_write_byte_dt(&config->i2c, BQ34Z_REG_BLOCK_CONTROL, 0x00);
	i2c_reg_write_byte_dt(&config->i2c, BQ34Z_REG_DATA_CLASS, class_id);
	i2c_reg_write_byte_dt(&config->i2c, BQ34Z_REG_DATA_BLOCK, block_id);
	k_msleep(5);

	/* 2. Calculate Checksum: (255 - (sum of 32 bytes % 256)) */
	for (int i = 0; i < 32; i++) {
		checksum += data[i];
	}
	checksum = 255 - checksum;

	/* 3. Write the 32-byte block */
	ret = i2c_burst_write_dt(&config->i2c, BQ34Z_REG_BLOCK_DATA_START, data, 32);
	if (ret < 0) return ret;

	/* 4. Write the checksum to finalize */
	return i2c_reg_write_byte_dt(&config->i2c, BQ34Z_REG_BLOCK_CHECKSUM, checksum);
}



/* To Unseal: */
// bq34z100_set_seal_state(dev, BQ34Z_KEY_UNSEAL_1);
// bq34z100_set_seal_state(dev, BQ34Z_KEY_UNSEAL_2);


void float_to_xfloat(float val, uint8_t *dest)
{
    /* Handle zero case */
    if (val == 0.0f) {
        memset(dest, 0, 4);
        return;
    }

    /* Convert to absolute value for processing */
    float abs_val = (val < 0) ? -val : val;
    int exponent = 0;

    /* Normalize the value to get the exponent */
    while (abs_val >= 1.0f) {
        abs_val /= 2.0f;
        exponent++;
    }
    while (abs_val < 0.5f) {
        abs_val *= 2.0f;
        exponent--;
    }

    /* TI Exponent is biased by 128 */
    dest[0] = (uint8_t)(exponent + 128);

    /* Convert mantissa (24-bit) */
    uint32_t mantissa = (uint32_t)(abs_val * 16777216.0f); // 2^24

    /* Apply sign bit to the MSB of the mantissa */
    if (val < 0) {
        mantissa |= 0x800000;
    } else {
        mantissa &= 0x7FFFFF;
    }

    /* Store Big-Endian */
    dest[1] = (mantissa >> 16) & 0xFF;
    dest[2] = (mantissa >> 8) & 0xFF;
    dest[3] = mantissa & 0xFF;
}

#define BQ34Z_SUBCLASS_IDENT   48
#define BQ34Z_OFFSET_CHEM      0  /* Chemistry string offset */

static int bq34z100_get_buffer_prop(const struct device *dev, 
                                    fuel_gauge_prop_t prop,
                                    void *dst, size_t dst_len)
{
    uint8_t block[32];
    int ret;

    switch (prop) {
    case FUEL_GAUGE_DEVICE_CHEMISTRY:
        /* Read the Identification Subclass */
        ret = bq34z_read_data_flash(dev, BQ34Z_SUBCLASS_IDENT, 0, block);
        if (ret < 0) return ret;

        /* BQ34Z100 strings: block[0] is often the length, but 
           usually, for Chemistry, it's just a 4-byte null-terminated string 
           at a fixed offset. Double check your specific firmware version. */
        
        size_t copy_len = MIN(dst_len, 4); // Chemistry is usually 4 chars
        memcpy(dst, &block[BQ34Z_OFFSET_CHEM], copy_len);
        
        return copy_len;

    default:
        return -ENOTSUP;
    }
}

static int bq34z100_set_prop(const struct device *dev, fuel_gauge_prop_t prop,
                             union fuel_gauge_prop_val val)
{
    int ret;
    uint8_t block[32];

    switch (prop) {
    case FUEL_GAUGE_DESIGN_CAPACITY:
        /* 1. Unseal the gauge */
        bq34z100_set_seal_state(dev, BQ34Z_KEY_UNSEAL_1);
        bq34z100_set_seal_state(dev, BQ34Z_KEY_UNSEAL_2);

        /* 2. Read the current block for Subclass 82 */
        ret = bq34z_read_data_flash(dev, BQ34Z_SUBCLASS_REGISTERS, 0, block);
        if (ret < 0) return ret;

        /* 3. Update Design Capacity (Bytes 10-11 in Subclass 82, Big Endian) */
        block[10] = (val.design_cap >> 8) & 0xFF;
        block[11] = val.design_cap & 0xFF;

        /* 4. Write back and Seal */
        ret = bq34z_write_data_flash_block(dev, BQ34Z_SUBCLASS_REGISTERS, 0, block);
        bq34z100_set_seal_state(dev, BQ34Z_KEY_SEAL);
        return ret;

    default:
        return -ENOTSUP;
    }
}

static int bq34z100_init(const struct device *dev)
{
	const struct bq34z100_config *config = dev->config;
	if (!device_is_ready(config->i2c.bus)) return -ENODEV;
	return 0;
}

static DEVICE_API(fuel_gauge, bq34z100_driver_api) = {
	.get_property = &bq34z100_get_prop,
	.get_buffer_property = &bq34z100_get_buffer_prop,
	.set_property = &bq34z100_set_prop,
	/* .battery_cutoff = &bq34z100_battery_cutoff */
};

#define BQ34Z100_INIT(inst)                                                \
	static const struct bq34z100_config bq34z100_config_##inst = {     \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                         \
	};                                                                 \
	DEVICE_DT_INST_DEFINE(inst,                                        \
		&bq34z100_init,                                            \
		NULL,                                                      \
		NULL,                                                      \
		&bq34z100_config_##inst,                                   \
		POST_KERNEL,                                               \
		CONFIG_FUEL_GAUGE_INIT_PRIORITY,                           \
		&bq34z100_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BQ34Z100_INIT)
