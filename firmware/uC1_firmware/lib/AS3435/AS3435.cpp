#include "AS3435.h"

// adapted from https://github.com/adafruit/Adafruit_MPR121


/*!
 *  @brief      Default constructor
 */
AS3435::AS3435() {}


/*!
 *  @brief    Begin an AS3435 object on a given I2C bus. This function resets
 *            the device and writes the default settings.
 *  @param    i2caddr
 *            the i2c address the device can be found on. Defaults to 0x46.
 *  @param    *theWire
 *            Wire object
 */
bool AS3435::begin(uint8_t i2caddr, TwoWire* theWire, 
                   uint8_t ANC_L_mic_gain, uint8_t ANC_R_mic_gain) {

    if (i2c_dev) {
        delete i2c_dev;
    }
    i2c_dev = new Adafruit_I2CDevice(i2caddr, theWire);

    if (!i2c_dev->begin()) {
        return false;
    }
    // pre startup reg dump
    // for (uint8_t i = 0; i < 0x7F; i++){
    //    Serial.print("$"); Serial.print(i, HEX);
    //    int v = readRegister8(i);
    //    Serial.printf(": 0x%x : %i\n", v, v);
    // }

    // set microphone gains
    writeRegister(AS3435_ANC_L, ANC_L_mic_gain);
    writeRegister(AS3435_ANC_R, ANC_R_mic_gain);

    // configure ANC mode
    uint8_t anc_settings = 0;
    anc_settings |= _BV(1); // OP1L_ON 
    anc_settings |= _BV(4); // MIX_ENABLE
    anc_settings |= _BV(0); // OP1R_ON
    anc_settings |= _BV(2); // OP2R_ON
    anc_settings |= 0b10 << 6; // HPH MUX: OP2 outputs connected to HPH input
    writeRegister(AS3435_ANC_MODE, anc_settings);

    // configure monitor mode
    uint8_t monitor_settings = 0;

    monitor_settings |= 0b00 << 6; // HPH MUX: QMIC outputs connected to HPH input
    writeRegister(AS3435_MON_MODE, monitor_settings);

    // configure PBO mode
    uint8_t pbo_settings = 0;
    writeRegister(AS3435_PBO_MODE, pbo_settings);

    // configure HP driver + I2C mode
    writeRegister(AS3435_MODE_2, 0x03);


    return true;
}


/*!
 *  @brief      Read the contents of an 8 bit device register.
 *  @param      reg the register address to read from
 *  @returns    the 8 bit value that was read.
 */
uint8_t AS3435::readRegister8(uint8_t reg) {
    Adafruit_BusIO_Register thereg = Adafruit_BusIO_Register(i2c_dev, reg, 1);

    return (thereg.read());
}


/*!
    @brief  Writes 8-bits to the specified destination register
    @param  reg the register address to write to
    @param  value the value to write
*/
void AS3435::writeRegister(uint8_t reg, uint8_t value) {
    Adafruit_BusIO_Register the_reg = Adafruit_BusIO_Register(i2c_dev, reg, 1);
    the_reg.write(value);
}


// sets eval register to enable anc mode
void AS3435::anc_mode() {
    uint8_t current_state = readRegister8(AS3435_EVAL);
    current_state &= ~(_BV(2) | _BV(3));
    writeRegister(AS3435_EVAL, current_state);
}


void AS3435::monitor_mode() {
    uint8_t current_state = readRegister8(AS3435_EVAL);
    current_state &= ~(_BV(2) | _BV(3));
    current_state |= _BV(3);
    writeRegister(AS3435_EVAL, current_state);
}


void AS3435::pbo_mode() {
    uint8_t current_state = readRegister8(AS3435_EVAL);
    current_state &= ~(_BV(2) | _BV(3));
    current_state |= _BV(2);
    writeRegister(AS3435_EVAL, current_state);
}