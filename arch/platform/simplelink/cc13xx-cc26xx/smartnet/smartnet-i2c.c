/*
 * smartnet-i2c.c
 *
 *  Created on: Oct 5, 2019
 */

#include <ti/drivers/I2C.h>

I2C_Handle i2c_handle;

/**
 * \brief         Setup and peform an I2C transaction.
 * \param wbuf    Output buffer during the I2C transation.
 * \param wcount  How many bytes in the wbuf.
 * \param rbuf    Input buffer during the I2C transation.
 * \param rcount  How many bytes to read into rbuf.
 * \return        true if the I2C operation was successful;
 *                else, return false.
 */
bool
i2c_write_read(int addr, void *wbuf, size_t wcount, void *rbuf, size_t rcount)
{
  I2C_Transaction i2c_transaction = {
    .writeBuf = wbuf,
    .writeCount = wcount,
    .readBuf = rbuf,
    .readCount = rcount,
    .slaveAddress = addr,
  };

  return I2C_transfer(i2c_handle, &i2c_transaction);
}
/**
 * \brief         Peform a write only I2C transaction.
 * \param wbuf    Output buffer during the I2C transation.
 * \param wcount  How many bytes in the wbuf.
 * \return        true if the I2C operation was successful;
 *                else, return false.
 */
bool
i2c_write(int addr, void *wbuf, size_t wcount)
{
  return i2c_write_read(addr, wbuf, wcount, NULL, 0);
}
/**
 * \brief         Peform a read only I2C transaction.
 * \param rbuf    Input buffer during the I2C transation.
 * \param rcount  How many bytes to read into rbuf.
 * \return        true if the I2C operation was successful;
 *                else, return false.
 */
bool
i2c_read(int addr, void *rbuf, size_t rcount)
{
  return i2c_write_read(addr, NULL, 0, rbuf, rcount);
}
