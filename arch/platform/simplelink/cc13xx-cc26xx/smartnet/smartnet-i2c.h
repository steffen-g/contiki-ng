/*
 * smartnet-i2c.h
 *
 *  Created on: Oct 5, 2019
 */

#ifndef CONTIKI_NG_ARCH_PLATFORM_SIMPLELINK_CC13XX_CC26XX_SMARTNET_SMARTNET_I2C_H_
#define CONTIKI_NG_ARCH_PLATFORM_SIMPLELINK_CC13XX_CC26XX_SMARTNET_SMARTNET_I2C_H_


#include <ti/drivers/I2C.h>

extern I2C_Handle i2c_handle;
extern bool i2c_write_read(int addr, void *wbuf, size_t wcount, void *rbuf, size_t rcount);
extern bool i2c_write(int addr, void *wbuf, size_t wcount);
extern bool i2c_read(int addr, void *rbuf, size_t rcount);

#endif /* CONTIKI_NG_ARCH_PLATFORM_SIMPLELINK_CC13XX_CC26XX_SMARTNET_SMARTNET_I2C_H_ */
