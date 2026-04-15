#include "bq76952.h"

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(BQ76952, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT ti_bq76952

/* Direct Commands */
#define BQ769X2_CMD_SAFETY_ALERT_A   0x02
#define BQ769X2_CMD_TEMP_DIE         0x06
#define BQ769X2_CMD_VOLTAGE_CELL_1   0x14
#define BQ769X2_CMD_VOLTAGE_STACK    0x34
#define BQ769X2_CMD_CURRENT_CC2      0x3A
#define BQ769X2_CMD_SUBCMD_ADDR      0x3E
#define BQ769X2_CMD_SUBCMD_DATA      0x40
#define BQ769X2_CMD_TEMP_TS1         0x70
#define BQ769X2_CMD_FET_STATUS       0x7F
#define BQ769X2_CMD_SUBCMD_CHKSUM    0x60
#define BQ769X2_CMD_SUBCMD_LEN       0x61

/* Data Memory Addresses */
#define BQ769X2_ADDR_VCELL_MODE      0x922A
#define BQ769X2_ADDR_TS1_CONFIG      0x92FD

/* Custom Attributes */
#define BQ76952_ATTR_CELL_INDEX      SENSOR_ATTR_PRIV_START

struct bq76952_config {
	struct i2c_dt_spec i2c;
};

struct bq76952_data {
	uint16_t cell_voltages[16];
	uint16_t stack_voltage;
	int16_t current;
	int16_t die_temp;
	int16_t ts1_temp;
	uint8_t selected_cell; /* For Option A indexing */
	uint8_t fet_status;
};

static uint8_t bq76952_calc_checksum(uint16_t addr, uint8_t *data, size_t len)
{
	uint8_t sum = (addr & 0xFF) + (addr >> 8);
	for (size_t i = 0; i < len; i++) {
		sum += data[i];
	}
	return ~sum;
}

static int bq76952_write_datamem(const struct device *dev, uint16_t addr, uint8_t *data, size_t len)
{
	const struct bq76952_config *cfg = dev->config;
	uint8_t addr_buf[2] = { addr & 0xFF, addr >> 8 };
	uint8_t chksum = bq76952_calc_checksum(addr, data, len);

	i2c_burst_write_dt(&cfg->i2c, BQ769X2_CMD_SUBCMD_ADDR, addr_buf, 2);
	i2c_burst_write_dt(&cfg->i2c, BQ769X2_CMD_SUBCMD_DATA, data, len);
	i2c_reg_write_byte_dt(&cfg->i2c, BQ769X2_CMD_SUBCMD_CHKSUM, chksum);
	i2c_reg_write_byte_dt(&cfg->i2c, BQ769X2_CMD_SUBCMD_LEN, len + 4);
	k_usleep(1000); /* Processing time */
	return 0;
}

static int bq76952_control_mode(const struct device *dev, uint16_t mode_cmd, bool verify)
{
	const struct bq76952_config *cfg = dev->config;
	uint8_t buf[2];
	i2c_burst_write_dt(&cfg->i2c, BQ769X2_CMD_SUBCMD_ADDR, (uint8_t*)&mode_cmd, 2);
	if (!verify) return 0;

	for (int i = 0; i < 10; i++) {
		i2c_burst_read_dt(&cfg->i2c, 0x3E, buf, 2);
		if ((mode_cmd == 0x0090 && (buf[0] & 0x01)) || (mode_cmd == 0x0092 && !(buf[0] & 0x01))) {
			return 0;
		}
		k_msleep(5);
	}
	return -ETIMEDOUT;
}

static int bq76952_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct bq76952_config *cfg = dev->config;
	struct bq76952_data *data = dev->data;
	uint8_t buf[2];

	/* Fetch Cell Voltages (16 cells) */
	for (int i = 0; i < 16; i++) {
		i2c_burst_read_dt(&cfg->i2c, BQ769X2_CMD_VOLTAGE_CELL_1 + (i * 2), buf, 2);
		data->cell_voltages[i] = (buf[1] << 8) | buf[0];
	}

	/* Fetch Stack Voltage and Current */
	i2c_burst_read_dt(&cfg->i2c, BQ769X2_CMD_VOLTAGE_STACK, buf, 2);
	data->stack_voltage = (buf[1] << 8) | buf[0];

	i2c_burst_read_dt(&cfg->i2c, BQ769X2_CMD_CURRENT_CC2, buf, 2);
	data->current = (int16_t)((buf[1] << 8) | buf[0]);

	/* Fetch Temps (0.1K) */
	i2c_burst_read_dt(&cfg->i2c, BQ769X2_CMD_TEMP_DIE, buf, 2);
	data->die_temp = (buf[1] << 8) | buf[0];
	i2c_burst_read_dt(&cfg->i2c, BQ769X2_CMD_TEMP_TS1, buf, 2);
	data->ts1_temp = (buf[1] << 8) | buf[0];

	/* Fetch FET Status */
	i2c_reg_read_byte_dt(&cfg->i2c, BQ769X2_CMD_FET_STATUS, &data->fet_status);

	return 0;
}

static int bq76952_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct bq76952_data *data = dev->data;
	int32_t temp_01c;

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		/* Return currently indexed cell voltage */
		val->val1 = data->cell_voltages[data->selected_cell] / 1000;
		val->val2 = (data->cell_voltages[data->selected_cell] % 1000) * 1000;
		break;
	case SENSOR_CHAN_CURRENT:
		val->val1 = data->current / 1000;
		val->val2 = (data->current % 1000) * 1000;
		break;
	case SENSOR_CHAN_DIE_TEMP:
		temp_01c = data->die_temp - 2732;
		val->val1 = temp_01c / 10;
		val->val2 = (temp_01c % 10) * 100000;
		break;
	case SENSOR_CHAN_AMBIENT_TEMP: /* Used for TS1 */
		temp_01c = data->ts1_temp - 2732;
		val->val1 = temp_01c / 10;
		val->val2 = (temp_01c % 10) * 100000;
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static int bq76952_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	struct bq76952_data *data = dev->data;
	if (attr == BQ76952_ATTR_CELL_INDEX) {
		if (val->val1 < 1 || val->val1 > 16) return -EINVAL;
		data->selected_cell = val->val1 - 1;
		return 0;
	}
	return -ENOTSUP;
}

static int bq76952_init(const struct device *dev)
{
	const struct bq76952_config *cfg = dev->config;
	uint8_t ts_conf = 0x07; /* 10k NTC Cell Temp */
	uint8_t mask[2] = { 0xFF, 0x03 }; /* 10 Cells (0x03FF) */

	if (!device_is_ready(cfg->i2c.bus)) return -ENODEV;

	bq76952_control_mode(dev, 0x0090, true);  /* Enter Config */
	bq76952_write_datamem(dev, BQ769X2_ADDR_VCELL_MODE, mask, 2);
	bq76952_write_datamem(dev, BQ769X2_ADDR_TS1_CONFIG, &ts_conf, 1);
	bq76952_control_mode(dev, 0x0092, true);  /* Exit Config */

	return 0;
}

static const struct sensor_driver_api bq76952_api = {
	.sample_fetch = bq76952_sample_fetch,
	.channel_get = bq76952_channel_get,
	.attr_set = bq76952_attr_set,
};

#define BQ76952_INST(n) \
	static struct bq76952_data bq76952_data_##n; \
	static const struct bq76952_config bq76952_cfg_##n = { \
		.i2c = I2C_DT_SPEC_INST_GET(n), \
	}; \
	DEVICE_DT_INST_DEFINE(n, bq76952_init, NULL, &bq76952_data_##n, \
			      &bq76952_cfg_##n, POST_KERNEL, \
			      CONFIG_SENSOR_INIT_PRIORITY, &bq76952_api);

DT_INST_FOREACH_STATUS_OKAY(BQ76952_INST)




/*
example usage:

struct sensor_value cell_idx = { .val1 = 5 }; // We want Cell 5
struct sensor_value voltage;

/* 1. Point the driver to Cell 5 */
sensor_attr_set(bq_dev, SENSOR_CHAN_VOLTAGE, BQ76952_ATTR_CELL_INDEX, &cell_idx);

/* 2. Fetch all data from hardware */
sensor_sample_fetch(bq_dev);

/* 3. Get the voltage (will be Cell 5 specifically) */
sensor_channel_get(bq_dev, SENSOR_CHAN_VOLTAGE, &voltage);

printk("Cell 5 Voltage: %d.%06d V\n", voltage.val1, voltage.val2);
*/
