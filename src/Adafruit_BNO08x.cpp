/*!
 *  @file Adafruit_BNO08x.cpp
 *
 *  @mainpage Adafruit BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the BNO08x 9-DOF Orientation IMU Fusion
 * Breakout
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

static Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface
static int8_t _int_pin, _reset_pin;

static Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
static HardwareSerial *uart_dev = NULL;

static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us);
static void i2chal_close(sh2_Hal_t *self);
static int i2chal_open(sh2_Hal_t *self);

static int uarthal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int uarthal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                        uint32_t *t_us);
static void uarthal_close(sh2_Hal_t *self);
static int uarthal_open(sh2_Hal_t *self);

static bool spihal_wait_for_int(void);
static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us);
static void spihal_close(sh2_Hal_t *self);
static int spihal_open(sh2_Hal_t *self);

static uint32_t hal_getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);
static void hal_hardwareReset(void);

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 * @param reset_pin The arduino pin # connected to the BNO Reset pin
 */
Adafruit_BNO08x::Adafruit_BNO08x(int8_t reset_pin) { _reset_pin = reset_pin; }

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
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    Serial.println(F("I2C address not found"));
    return false;
  }

  _HAL.open = i2chal_open;
  _HAL.close = i2chal_close;
  _HAL.read = i2chal_read;
  _HAL.write = i2chal_write;
  _HAL.getTimeUs = hal_getTimeUs;

  return _init(sensor_id);
}

/**
 *  @brief  Sets up the hardware and initializes UART
 *
 * @param serial Pointer to Stream (HardwareSerial/SoftwareSerial) interface
 * @param sensor_id
 *            The user-defined ID to differentiate different sensors
 * @return  true if initialization was successful, otherwise false.
 */
bool Adafruit_BNO08x::begin_UART(HardwareSerial *serial, int32_t sensor_id) {
  uart_dev = serial;

  _HAL.open = uarthal_open;
  _HAL.close = uarthal_close;
  _HAL.read = uarthal_read;
  _HAL.write = uarthal_write;
  _HAL.getTimeUs = hal_getTimeUs;

  return _init(sensor_id);
}

/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  int_pin The arduino pin # connected to BNO08x INT
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @param  sensor_id
 *            The user-defined ID to differentiate different sensors
 *    @return true if initialization was successful, otherwise false.
 */
bool Adafruit_BNO08x::begin_SPI(uint8_t cs_pin, uint8_t int_pin,
                                SPIClass *theSPI, int32_t sensor_id) {
  i2c_dev = NULL;

  _int_pin = int_pin;
  pinMode(_int_pin, INPUT_PULLUP);

  if (spi_dev) {
    delete spi_dev; // remove old interface
  }
  spi_dev = new Adafruit_SPIDevice(cs_pin,
                                   1000000,               // frequency
                                   SPI_BITORDER_MSBFIRST, // bit order
                                   SPI_MODE3,             // data mode
                                   theSPI);
  if (!spi_dev->begin()) {
    return false;
  }

  _HAL.open = spihal_open;
  _HAL.close = spihal_close;
  _HAL.read = spihal_read;
  _HAL.write = spihal_write;
  _HAL.getTimeUs = hal_getTimeUs;

  return _init(sensor_id);
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_BNO08x::_init(int32_t sensor_id) {
  int status;

  hardwareReset();

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) {
    return false;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return true;
}

/**
 * @brief Reset the device using the Reset pin
 *
 */
void Adafruit_BNO08x::hardwareReset(void) { hal_hardwareReset(); }

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool Adafruit_BNO08x::wasReset(void) {
  bool x = _reset_occurred;
  _reset_occurred = false;

  return x;
}

/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool Adafruit_BNO08x::getSensorEvent(sh2_SensorValue_t *value) {
  _sensor_value = value;

  value->timestamp = 0;

  sh2_service();

  if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool Adafruit_BNO08x::enableReport(sh2_SensorId_t sensorId,
                                   uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

/**************************************** I2C interface
 * ***********************************************************/

static int i2chal_open(sh2_Hal_t *self) {
  // Serial.println("I2C HAL open");
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (i2c_dev->write(softreset_pkt, 5)) {
      success = true;
      break;
    }
    delay(30);
  }
  if (!success)
    return -1;
  delay(300);
  return 0;
}

static void i2chal_close(sh2_Hal_t *self) {
  // Serial.println("I2C HAL close");
}

static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  // Serial.println("I2C HAL read");

  // uint8_t *pBufferOrig = pBuffer;

  uint8_t header[4];
  if (!i2c_dev->read(header, 4)) {
    return 0;
  }

  // Determine amount to read
  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  /*
  Serial.print("Read SHTP header. ");
  Serial.print("Packet size: ");
  Serial.print(packet_size);
  Serial.print(" & buffer size: ");
  Serial.println(len);
  */

  size_t i2c_buffer_max = i2c_dev->maxBufferSize();

  if (packet_size > len) {
    // packet wouldn't fit in our buffer
    return 0;
  }
  // the number of non-header bytes to read
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    // Serial.print("Reading from I2C: "); Serial.println(read_size);
    // Serial.print("Remaining to read: "); Serial.println(cargo_remaining);

    if (!i2c_dev->read(i2c_buffer, read_size)) {
      return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }

  /*
  for (int i=0; i<packet_size; i++) {
    Serial.print(pBufferOrig[i], HEX);
    Serial.print(", ");
    if (i % 16 == 15) Serial.println();
  }
  Serial.println();
  */

  return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = i2c_dev->maxBufferSize();

  /*
  Serial.print("I2C HAL write packet size: ");
  Serial.print(len);
  Serial.print(" & max buffer size: ");
  Serial.println(i2c_buffer_max);
  */

  uint16_t write_size = min(i2c_buffer_max, len);
  if (!i2c_dev->write(pBuffer, write_size)) {
    return 0;
  }

  return write_size;
}

/**************************************** UART interface
 * ***********************************************************/

static int uarthal_open(sh2_Hal_t *self) {
  // Serial.println("UART HAL open");
  uart_dev->begin(3000000);

  // flush input
  while (uart_dev->available()) {
    uart_dev->read();
    yield();
  }

  // send a software reset
  uint8_t softreset_pkt[] = {0x7E, 1, 5, 0, 1, 0, 1, 0x7E};
  for (int i = 0; i < sizeof(softreset_pkt); i++) {
    uart_dev->write(softreset_pkt[i]);
    delay(1);
  }

  return 0;
}

static void uarthal_close(sh2_Hal_t *self) {
  // Serial.println("UART HAL close");
  uart_dev->end();
}

static int uarthal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                        uint32_t *t_us) {
  uint8_t c;
  uint16_t packet_size = 0;

  // Serial.println("UART HAL read");

  // read packet start
  while (1) {
    yield();

    if (!uart_dev->available()) {
      continue;
    }
    c = uart_dev->read();
    // Serial.print(c, HEX); Serial.print(", ");
    if (c == 0x7E) {
      break;
    }
  }

  // read protocol id
  while (uart_dev->available() < 2) {
    yield();
  }
  c = uart_dev->read();
  // Serial.print(c, HEX); Serial.print(", ");
  if (c == 0x7E) {
    c = uart_dev->read();
    // Serial.print(c, HEX); Serial.print(", ");
    if (c != 0x01) {
      return 0;
    }
  } else if (c != 0x01) {
    return 0;
  }

  while (true) {
    yield();

    if (!uart_dev->available()) {
      continue;
    }
    c = uart_dev->read();
    // Serial.print(c, HEX); Serial.print(", ");
    if (c == 0x7E) {
      break;
    }
    if (c == 0x7D) {
      // escape!
      while (!uart_dev->available()) {
        continue;
      }
      c = uart_dev->read();
      c ^= 0x20;
    }
    pBuffer[packet_size] = c;
    packet_size++;
  }

  /*
  Serial.print("Read UART packet size: ");
  Serial.println(packet_size);
  for (int i=0; i<packet_size; i++) {
    Serial.print(pBuffer[i], HEX);
    Serial.print(", ");
    if (i % 16 == 15) Serial.println();
  }
  Serial.println();
  */

  return packet_size;
}

static int uarthal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  uint8_t c;

  // Serial.print("UART HAL write packet size: ");
  // Serial.println(len);

  // start byte
  uart_dev->write(0x7E);
  delay(1);
  // protocol id
  uart_dev->write(0x01);
  delay(1);

  for (int i = 0; i < len; i++) {
    c = pBuffer[i];
    if ((c == 0x7E) || (c == 0x7D)) {
      uart_dev->write(0x7D); // control
      delay(1);
      c ^= 0x20;
    }
    uart_dev->write(c);
    delay(1);
  }
  // end byte
  uart_dev->write(0x7E);

  return len;
}

/**************************************** UART interface
 * ***********************************************************/

static int spihal_open(sh2_Hal_t *self) {
  // Serial.println("SPI HAL open");

  spihal_wait_for_int();

  return 0;
}

static bool spihal_wait_for_int(void) {
  for (int i = 0; i < 500; i++) {
    if (!digitalRead(_int_pin))
      return true;
    // Serial.print(".");
    delay(1);
  }
  // Serial.println("Timed out!");
  hal_hardwareReset();

  return false;
}

static void spihal_close(sh2_Hal_t *self) {
  // Serial.println("SPI HAL close");
}

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
  // Serial.println("SPI HAL read");

  uint16_t packet_size = 0;

  if (!spihal_wait_for_int()) {
    return 0;
  }

  if (!spi_dev->read(pBuffer, 4, 0x00)) {
    return 0;
  }

  // Determine amount to read
  packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;

  /*
  Serial.print("Read SHTP header. ");
  Serial.print("Packet size: ");
  Serial.print(packet_size);
  Serial.print(" & buffer size: ");
  Serial.println(len);
  */

  if (packet_size > len) {
    return 0;
  }

  if (!spihal_wait_for_int()) {
    return 0;
  }

  if (!spi_dev->read(pBuffer, packet_size, 0x00)) {
    return 0;
  }

  return packet_size;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  // Serial.print("SPI HAL write packet size: ");
  // Serial.println(len);

  if (!spihal_wait_for_int()) {
    return 0;
  }

  spi_dev->write(pBuffer, len);

  return len;
}

/**************************************** HAL interface
 * ***********************************************************/

static void hal_hardwareReset(void) {
  if (_reset_pin != -1) {
    // Serial.println("BNO08x Hardware reset");

    pinMode(_reset_pin, OUTPUT);
    digitalWrite(_reset_pin, HIGH);
    delay(10);
    digitalWrite(_reset_pin, LOW);
    delay(10);
    digitalWrite(_reset_pin, HIGH);
    delay(10);
  }
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  uint32_t t = millis() * 1000;
  // Serial.printf("I2C HAL get time: %d\n", t);
  return t;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // Serial.println("Reset!");
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    Serial.println("BNO08x - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}
