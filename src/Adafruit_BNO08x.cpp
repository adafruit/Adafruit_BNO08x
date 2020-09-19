/*!
 *  @file Adafruit_BNO08x.cpp
 *
 *  @mainpage Adafruit BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 * 	This is a library for the Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/product/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_BNO08x.h"

Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface
Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */
Adafruit_BNO08x::Adafruit_BNO08x(int8_t reset_pin) {
  _reset_pin = reset_pin;
}


/**
 * @brief Destroy the Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */
Adafruit_BNO08x::~Adafruit_BNO08x(void) {
  // if (temp_sensor)
  //   delete temp_sensor;

}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BNO08x::begin_I2C(uint8_t i2c_address, TwoWire *wire,
                               int32_t sensor_id) {
  spi_dev = NULL;
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  i2c_HAL.open = i2chal_open;
  i2c_HAL.close = i2chal_close;
  i2c_HAL.read = i2chal_read;
  i2c_HAL.write = i2chal_write;
  i2c_HAL.getTimeUs = i2chal_getTimeUs;


  return _init(sensor_id);
}

/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BNO08x::begin_SPI(uint8_t cs_pin, SPIClass *theSPI,
                               int32_t sensor_id) {
  i2c_dev = NULL;

  if (spi_dev) {
    delete spi_dev; // remove old interface
  }
  spi_dev = new Adafruit_SPIDevice(cs_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE0,             // data mode
                                   theSPI);
  if (!spi_dev->begin()) {
    return false;
  }

  return _init(sensor_id);
}

/*!
 *    @brief  Sets up the hardware and initializes software SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  sck_pin The arduino pin # connected to SPI clock
 *    @param  miso_pin The arduino pin # connected to SPI MISO
 *    @param  mosi_pin The arduino pin # connected to SPI MOSI
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BNO08x::begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                               int8_t mosi_pin, int32_t sensor_id) {
  i2c_dev = NULL;

  if (spi_dev) {
    delete spi_dev; // remove old interface
  }
  spi_dev = new Adafruit_SPIDevice(cs_pin, sck_pin, miso_pin, mosi_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE0);            // data mode
  if (!spi_dev->begin()) {
    return false;
  }
  return _init(sensor_id);
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_BNO08x::_init(int32_t sensor_id) {


  hardware_reset();
  // do any software reset or other initial setup

  // TODO: update for correct sensor types
  int success = sh2_open(&i2c_HAL, i2chal_callback, NULL);
  if (success != SH2_OK) {
    return false;
  }

  sh2_ProductIds_t prodIds;
  memset(&prodIds, 0, sizeof(prodIds));
  success = sh2_getProdIds(&prodIds);
  if (success != SH2_OK) {
    return false;
  }

  // Report the results
  for (int n = 0; n < prodIds.numEntries; n++) {
    Serial.printf("Part %d : Version %d.%d.%d Build %d\n",
                  prodIds.entry[n].swPartNumber,
                  prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, 
                  prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
  }

  return true;
}

void Adafruit_BNO08x::hardware_reset(void) {
  if (_reset_pin != -1) {
    Serial.println("BNO08x Hardware reset");

    pinMode(_reset_pin, OUTPUT);
    digitalWrite(_reset_pin, HIGH);
    delay(1);
    digitalWrite(_reset_pin, LOW);
    delay(1);
    digitalWrite(_reset_pin, HIGH);
    delay(100);
  }
}

/**
 * @brief Performs a software reset initializing registers to their power on
 * state
 *
 */
void Adafruit_BNO08x::software_reset(void) {

}

int i2chal_open(sh2_Hal_t *self) {
  Serial.println("I2C HAL open");
  return 0;
}


void i2chal_close(sh2_Hal_t *self) {
  Serial.println("I2C HAL close");
}

int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                uint32_t *t_us) {
  Serial.println("I2C HAL read");

  uint8_t header[4];
  if (! i2c_dev->read(header, 4) ) {
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  Serial.print("Read SHTP header. "); 
  Serial.print("Packet size: "); 
  Serial.print(packet_size);
  Serial.print(" & buffer size: "); 
  Serial.println(len);


  size_t i2c_buffer_max = i2c_dev->maxBufferSize();

  unsigned total_read_len = len;
  if (packet_size > len){
    // packet wouldn't fit in our buffer
    return 0;
  }
  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = min(i2c_buffer_max, cargo_remaining); 
    } else {
      read_size = min(i2c_buffer_max, cargo_remaining+4); 
    }

    Serial.print("Reading from I2C: "); Serial.println(read_size);
    Serial.print("Remaining to read: "); Serial.println(cargo_remaining);
    
    if (!i2c_dev->read(i2c_buffer, read_size)) {
      return 0;
    }

    if (first_read) {
      // this is the first read, so copy everything, including the header
      memcpy(pBuffer, i2c_buffer, read_size);
      // advance our pointer
      pBuffer += read_size;
      cargo_remaining -= read_size;
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of the local buffer to skip the header
      memcpy(pBuffer, i2c_buffer+4, read_size-4);      
      cargo_remaining -= read_size - 4;
    }
  }

  return packet_size;
}

int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = i2c_dev->maxBufferSize();

  Serial.print("I2C HAL write packet size: ");
  Serial.print(len);
  Serial.print(" & max buffer size: "); 
  Serial.println(i2c_buffer_max);

  uint16_t write_size = min(i2c_buffer_max, len); 
  if (!i2c_dev->write(pBuffer, write_size)) {
    return 0;
  }
  
  return write_size;
}

uint32_t i2chal_getTimeUs(sh2_Hal_t *self) {
  uint32_t t = millis() * 1000;
  // Serial.printf("I2C HAL get time: %d\n", t);
  return t;
}


void i2chal_callback(void * cookie, sh2_AsyncEvent_t *pEvent) {
  Serial.println("I2C HAL callback");
}
