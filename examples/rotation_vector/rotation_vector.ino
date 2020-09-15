// Basic demo for accelerometer readings from Adafruit BNO08x


// Basic demo for readings from Adafruit BNO08x
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define BNO08X_SCK 13
#define BNO08X_MISO 12
#define BNO08X_MOSI 11

Adafruit_BNO08x  bno08x; // TODO FIX NAME
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_SPI(BNO08X_CS)) {
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_SCK, BNO08X_MISO, BNO08X_MOSI)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
}


void loop() {

  delay(10);
}
