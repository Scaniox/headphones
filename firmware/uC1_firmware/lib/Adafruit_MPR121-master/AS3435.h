#ifndef ADAFRUIT_AS3435_H
#define ADAFRUIT_AS3435_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>

// The default I2C address
#define AS3435_I2CADDR_L 0x46        // left I2C address (swapped SCK and SDA)
#define AS3435_I2CADDR_R 0x47        // right I2C address

#define AS3435_MIC_GAIN_DEFAULT 0x00

#define _BV(bit) (1 << (bit))

/*!
 *  Device register map
 */
enum {
  AS3435_SYSTEM = 0x20,
  AS3435_PWR_SET = 0x21,
  AS3435_ANC_L2 = 0x10,
  AS3435_ANC_R2 = 0x11,
  AS3435_ANC_L3 = 0x12,
  AS3435_ANC_R3 = 0x13,
  AS3435_ANC_MODE = 0x14,
  AS3435_MON_MODE = 0x15,
  AS3435_PBO_MODE = 0x16,
  AS3435_ECO = 0x17,
  AS3435_ANC_L = 0x30,
  AS3435_ANC_R = 0x31,
  AS3435_MIC_MON_L = 0x32,
  AS3435_MIC_MON_R = 0x33,
  AS3435_MODE_1 = 0x34,
  AS3435_MODE_2 = 0x35,
  AS3435_EVAL = 0x3D,
  AS3435_CONFIG_1 = 0x3E,
  AS3435_CONFIG_2 = 0x3F,
};

/*!
 *  @brief  Class that stores state and functions for interacting with AS_3435
 */
class AS3435 {
public:
  // Hardware I2C
  AS3435();

  bool begin(uint8_t i2caddr = AS3435_I2CADDR_L, TwoWire *theWire = &Wire, 
              uint8_t ANC_L_mic_gain = AS3435_MIC_GAIN_DEFAULT, 
              uint8_t ANC_R_mic_gain = AS3435_MIC_GAIN_DEFAULT);

  uint8_t readRegister8(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);

  void anc_mode();
  void monitor_mode();
  void pbo_mode();
  void bypass_mode();

private:
  Adafruit_I2CDevice *i2c_dev = NULL;
};

#endif
