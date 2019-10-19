/*
 * Copyright (c) 2018, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup sensortag-hdc-sensor
 * @{
 *
 * \file
 *        Driver for the Sensortag HDC2080 sensor.
 * \author
 *        Edvard Pettersen <e.pettersen@ti.com>
 */
/*---------------------------------------------------------------------------*/
#include "hdc-2080-sensor.h"

#include "contiki.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
#include <Board.h>

#include <ti/drivers/I2C.h>
#include "smartnet-i2c.h"

/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/*
 * Disable the entire file if sensors are disabled, as it could potentially
 * create compile errors with missing defines from either the Board file or
 * configuration defines.
 */
#if BOARD_SENSORS_ENABLE
/*---------------------------------------------------------------------------*/
#ifndef Board_HDC2080_ADDR
#error "Board file doesn't define the I2C address Board_HDC2080_ADDR"
#endif
/* Sensor I2C address */
#define HDC2080_I2C_ADDRESS        Board_HDC2080_ADDR
/*---------------------------------------------------------------------------*/
/* Registers */
#define HDC2080_REG_TEMP           0x00 /* Temperature */
#define HDC2080_REG_HUM            0x02 /* Humidity */
#define HDC2080_REG_CONFIG         0x0f /* Configuration */
#if 0
#define HDC2080_REG_SERID_H        0xFB /* Serial ID high */
#define HDC2080_REG_SERID_M        0xFC /* Serial ID middle */
#define HDC2080_REG_SERID_L        0xFD /* Serial ID low */
#endif
#define HDC2080_REG_MANF_ID        0xFC /* Manufacturer ID */
#define HDC2080_REG_DEV_ID         0xFE /* Device ID */
/*---------------------------------------------------------------------------*/
/* Fixed values */
#define HDC2080_VAL_MANF_ID        0x5449
#define HDC2080_VAL_DEV_ID         0x207d
#define HDC2080_VAL_CONFIG         0x00 /* 14 bit, acquired in sequence */
/*---------------------------------------------------------------------------*/
/* Byte swap of 16-bit register value */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define SWAP16(v) ((LO_UINT16(v) << 8) | HI_UINT16(v))

#define LSB16(v)  (LO_UINT16(v)), (HI_UINT16(v))
/*---------------------------------------------------------------------------*/
/* Raw data as returned from the sensor (Big Endian) */
typedef struct {
	uint16_t temp;
	uint16_t hum;
}HDC_2080_SensorData;

static HDC_2080_SensorData sensor_data;
/*---------------------------------------------------------------------------*/
static volatile HDC_2080_SENSOR_STATUS sensor_status = HDC_2080_SENSOR_STATUS_DISABLED;
/*---------------------------------------------------------------------------*/
/*
 * Maximum measurement durations in clock ticks. We use 14bit resolution, thus:
 * - Tmp: 6.35ms
 * - RH:  6.5ms
 */
#define MEASUREMENT_DURATION 2

/*
 * Wait SENSOR_STARTUP_DELAY clock ticks between activation and triggering a
 * reading (max 15ms)
 */
#define SENSOR_STARTUP_DELAY 3

static struct ctimer startup_timer;

/*---------------------------------------------------------------------------*/
/**
 * \brief   Initialize the HDC-2080 sensor driver.
 * \return  true if I2C operation successful; else, return false.
 */
static bool
sensor_init(void)
{

	if(i2c_handle == 0) {
		I2C_Params i2c_params;

		I2C_Params_init(&i2c_params);

		i2c_params.transferMode = I2C_MODE_BLOCKING;
		i2c_params.bitRate = I2C_400kHz;

		i2c_handle = I2C_open(Board_I2C0, &i2c_params);
		if(i2c_handle == NULL) {
			return false;
		}
	}
	/* Enable reading data in one operation */
	uint8_t config_data[] = {HDC2080_REG_CONFIG, HDC2080_VAL_CONFIG};

	return i2c_write(HDC2080_I2C_ADDRESS, config_data, sizeof(config_data));
}
/*---------------------------------------------------------------------------*/
/**
 * \brief   Start measurement.
 * \return  true if I2C operation successful; else, return false.
 */
static bool
start(void)
{
	uint8_t config_data[] = { HDC2080_REG_CONFIG, HDC2080_VAL_CONFIG + 1 };

	return i2c_write(HDC2080_I2C_ADDRESS, config_data, sizeof(config_data));
}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Convert raw data to temperature and humidity.
 * \param temp  Output variable to store converted temperature.
 * \param hum   Output variable to store converted humidity.
 */
static void
convert(int32_t *temp, int32_t *hum)
{

	int32_t raw_hum = sensor_data.hum;

	/* Convert temperature to degrees C */
	*temp = sensor_data.temp * 165;
	*temp *= 100;
	*temp /= 65535;
	*temp -= 4000;

	/* Convert relative humidity to a %RH value */
	*hum = raw_hum * 100 * 100 / 65536;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief  Callback when sensor is ready to read data from.
 */
static void
notify_ready(void *unused)
{
	/* Unused args */
	(void)unused;

	uint8_t config_data[] = {HDC2080_REG_TEMP};
	i2c_write_read(HDC2080_I2C_ADDRESS, config_data, sizeof(config_data), &sensor_data.temp, 2);
	config_data[0] = HDC2080_REG_HUM;
	if(i2c_write_read(HDC2080_I2C_ADDRESS, config_data, sizeof(config_data), &sensor_data.hum, 2)){
		sensor_status = HDC_2080_SENSOR_STATUS_READINGS_READY;
	} else {
		sensor_status = HDC_2080_SENSOR_STATUS_I2C_ERROR;
	}

	sensors_changed(&hdc_2080_sensor);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Returns a reading from the sensor.
 * \param type  HDC_2080_SENSOR_TYPE_TEMP or HDC_2080_SENSOR_TYPE_HUMID.
 * \return      Temperature (centi degrees C) or Humidity (centi %RH).
 */
static int
value(int type)
{
	int32_t temp = 0;
	int32_t hum = 0;

	if(sensor_status != HDC_2080_SENSOR_STATUS_READINGS_READY) {
		PRINTF("Sensor disabled or starting up (%d)\n", sensor_status);
		return HDC_2080_READING_ERROR;
	}

	switch(type) {
		case HDC_2080_SENSOR_TYPE_TEMP:
		case HDC_2080_SENSOR_TYPE_HUMID:
		convert(&temp, &hum);
		PRINTF("HDC: t=%d h=%d\n", (int)temp, (int)hum);

		if(type == HDC_2080_SENSOR_TYPE_TEMP) {
			return (int)temp;
		} else if(type == HDC_2080_SENSOR_TYPE_HUMID) {
			return (int)hum;
		} else {
			return HDC_2080_READING_ERROR;
		}

		default:
		PRINTF("Invalid type\n");
		return HDC_2080_READING_ERROR;
	}
}
/*---------------------------------------------------------------------------*/
/**
 * \brief         Configuration function for the HDC2080 sensor.
 * \param type    Activate, enable or disable the sensor. See below.
 * \param enable  Either enable or disable the sensor.
 *                When type == SENSORS_HW_INIT we turn on the hardware.
 *                When type == SENSORS_ACTIVE and enable==1 we enable the sensor.
 *                When type == SENSORS_ACTIVE and enable==0 we disable the sensor.
 */
static int
configure(int type, int enable)
{
	switch(type) {
		case SENSORS_HW_INIT:
		memset(&sensor_data, 0, sizeof(sensor_data));

		if(sensor_init()) {
			sensor_status = HDC_2080_SENSOR_STATUS_INITIALISED;
		} else {
			sensor_status = HDC_2080_SENSOR_STATUS_I2C_ERROR;
		}
		break;

		case SENSORS_ACTIVE:
		/* Must be initialised first */
		if(sensor_status == HDC_2080_SENSOR_STATUS_DISABLED) {
			break;
		}

		if(enable) {
			if(!start()) {
				sensor_status = HDC_2080_SENSOR_STATUS_I2C_ERROR;
				break;
			}
			ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
			sensor_status = HDC_2080_SENSOR_STATUS_TAKING_READINGS;
		} else {
			ctimer_stop(&startup_timer);
			sensor_status = HDC_2080_SENSOR_STATUS_INITIALISED;
		}
		break;

		default:
		break;
	}
	return sensor_status;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Returns the status of the sensor.
 * \param type  SENSORS_ACTIVE or SENSORS_READY.
 * \return      One of the SENSOR_STATUS_xyz defines.
 */
static int
status(int type)
{
	switch(type) {
		case SENSORS_ACTIVE:
		case SENSORS_READY:
		return sensor_status;

		default:
		return HDC_2080_SENSOR_STATUS_DISABLED;
	}
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(hdc_2080_sensor, "HDC2080", value, configure, status);
/*---------------------------------------------------------------------------*/
#endif /* BOARD_SENSORS_ENABLE */
/*---------------------------------------------------------------------------*/
/** @} */
