/*
 * board-conf.h
 *
 *  Created on: Oct 4, 2019
 */

#ifndef CONTIKI_NG_ARCH_PLATFORM_SIMPLELINK_CC13XX_CC26XX_SMARTNET_BOARD_CONF_H_
#define CONTIKI_NG_ARCH_PLATFORM_SIMPLELINK_CC13XX_CC26XX_SMARTNET_BOARD_CONF_H_

/**
 * Enable the three defines for sensors
 */
#define BOARD_CONF_HAS_SENSORS      1
#define BOARD_SENSORS_ENABLE		1
#define BOARD_CONF_SENSORS_ENABLE   1

/**
 * Configure I2C addresses
 */
#define Board_HDC2080_ADDR          0x40
#define Board_OPT3001_ADDR          0x44

/**
 * The platform has to have keys to be compilable
 */
#define PLATFORM_HAS_BUTTON           1
#define PLATFORM_SUPPORTS_BUTTON_HAL  1
// but only one key todo:find out where it is configured to a pin
#define BUTTON_HAL_ID_KEY      		0


#define PLATFORM_HAS_LEDS           1
#define LEDS_CONF_COUNT             1
#define LEDS_CONF_GREEN             1


#define BOARD_RF_LONGRANGE			1

#define SET_CCFG_MODE_CONF_XOSC_CAP_MOD				0
#define SET_CCFG_MODE_CONF_XOSC_CAPARRAY_DELTA		0xdb

#endif /* CONTIKI_NG_ARCH_PLATFORM_SIMPLELINK_CC13XX_CC26XX_SMARTNET_BOARD_CONF_H_ */
