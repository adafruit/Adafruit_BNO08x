/*!
 *  @file Adafruit_BNO08x.h
 *
 * 	I2C Driver for the Adafruit BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 * 	This is a library for the Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/products/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_BNO08x_H
#define _ADAFRUIT_BNO08x_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "sh2.h"
#include "sh2_err.h"

#define BNO08x_I2CADDR_DEFAULT 0x4A


/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the BNO08x 9-DOF Orientation IMU Fusion Breakout
 */
class Adafruit_BNO08x {
public:
  Adafruit_BNO08x(int8_t reset_pin = -1);
  ~Adafruit_BNO08x();

  bool begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensor_id = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id = 0);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensor_id = 0);
  
  void software_reset(void);
  void hardware_reset(void);


protected:
  virtual bool _init(int32_t sensor_id);

  int8_t _reset_pin = -1;
  sh2_Hal_t i2c_HAL;
};


uint32_t i2chal_getTimeUs(sh2_Hal_t *self);
int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
void i2chal_close(sh2_Hal_t *self);
int i2chal_open(sh2_Hal_t *self);
void i2chal_callback(void * cookie, sh2_AsyncEvent_t *pEvent);

#endif
