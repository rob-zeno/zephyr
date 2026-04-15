#ifndef ZEPHYR_DRIVERS_SENSOR_BATTERY_BQ76952_H_
#define ZEPHYR_DRIVERS_SENSOR_BATTERY_BQ76952_H_

#include <zephyr/drivers/sensor.h>

/* Custom attribute to select which cell (1-16) we are talking to */
#define BQ76952_ATTR_CELL_INDEX SENSOR_ATTR_PRIV_START

#endif
