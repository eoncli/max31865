/**
 ******************************************************************************
 * @file    max31865.c
 * @author  Pablo Fuentes
 * @version V1.0.1
 * @date    2019
 * @brief   MAX31865 Functions
 ******************************************************************************
 */

#include "max31865.h"
#include "math.h"

/**
 ===============================================================================
              ##### Macros definition #####
 ===============================================================================
 */

// Register list
#define MAX31856_CONFIG_REG 0x00
#define MAX31856_CONFIG_BIAS 0x80
#define MAX31856_CONFIG_MODEAUTO 0x40
#define MAX31856_CONFIG_MODEOFF 0x00
#define MAX31856_CONFIG_1SHOT 0x20
#define MAX31856_CONFIG_3WIRE 0x10
#define MAX31856_CONFIG_24WIRE 0x00
#define MAX31856_CONFIG_FAULTSTAT 0x02
#define MAX31856_CONFIG_FILT50HZ 0x01
#define MAX31856_CONFIG_FILT60HZ 0x00

#define MAX31856_RTDMSB_REG 0x01
#define MAX31856_RTDLSB_REG 0x02
#define MAX31856_HFAULTMSB_REG 0x03
#define MAX31856_HFAULTLSB_REG 0x04
#define MAX31856_LFAULTMSB_REG 0x05
#define MAX31856_LFAULTLSB_REG 0x06
#define MAX31856_FAULTSTAT_REG 0x07

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

#define SELECT() gpio_reset(m->cs)
#define DESELECT() gpio_set(m->cs)

/**
 ===============================================================================
              ##### Private Functions #####
 ===============================================================================
 */

static void max31865_write(max31865_t *m, uint8_t address, uint8_t data) {
  SELECT();
  spi_write8(m->SPIx, (uint8_t)(address | 0x80));
  spi_write8(m->SPIx, data);
  DESELECT();
}

static uint8_t max31865_read(max31865_t *m, uint8_t address) {
  uint8_t value = 255;
  SELECT();
  spi_write8(m->SPIx, (uint8_t)(address & 0x7F));
  value = spi_write8(m->SPIx, 0xFF);
  DESELECT();
  return value;
}

static uint16_t max31865_read16(max31865_t *m, uint8_t address) {
  uint8_t val1, val2;
  uint16_t value = 255;
  SELECT();
  spi_write8(m->SPIx, (uint8_t)(address & 0x7F));
  val1 = spi_write8(m->SPIx, 0xFF);
  val2 = spi_write8(m->SPIx, 0xFF);
  DESELECT();
  value = val1;
  value <<= 8;
  value |= val2;
  return value;
}

// static uint8_t max31865_writeRaw(max31865_t *m, uint8_t val) {
//   return (uint8_t) spi_write8(m->SPIx, val);
// }

/**
 ===============================================================================
              ##### Public functions #####
 ===============================================================================
 */

void max31865_deselect(max31865_t *m) {
  gpio_mode(m->cs, OUTPUT_PP, NOPULL, SPEED_HIGH);
  DESELECT();
}

bool max31865_init(max31865_t *m, max31865_numwires_t wires) {
  gpio_mode(m->cs, OUTPUT_PP, NOPULL, SPEED_HIGH);
  DESELECT();

  max31865_write(m, MAX31856_CONFIG_REG, 0x00);
  delay(100);

  max31865_setWires(m, wires);
  max31865_enableBias(m, false);
  max31865_autoConvert(m, false);
  max31865_clearFault(m);

  return true;
}

void max31865_setWires(max31865_t *m, max31865_numwires_t wires) {
  uint8_t t = max31865_read(m, MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31856_CONFIG_3WIRE;
  } else { // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  }
  max31865_write(m, MAX31856_CONFIG_REG, t);
}

uint8_t max31865_readFault(max31865_t *m) {
  return max31865_read(m, MAX31856_FAULTSTAT_REG);
}

void max31865_clearFault(max31865_t *m) {
  uint8_t t = max31865_read(m, MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  max31865_write(m, MAX31856_CONFIG_REG, t);
}

void max31865_autoConvert(max31865_t *m, bool b) {
  uint8_t t = max31865_read(m, MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO; // disable autoconvert
  }
  max31865_write(m, MAX31856_CONFIG_REG, t);
}

void max31865_enableBias(max31865_t *m, bool b) {
  uint8_t t = max31865_read(m, MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS; // disable bias
  }
  max31865_write(m, MAX31856_CONFIG_REG, t);
}

uint16_t max31865_readRTD(max31865_t *m) {
  uint8_t t;
  uint16_t rtd;
  max31865_clearFault(m);
  max31865_enableBias(m, true);
  delay(10);
  t = max31865_read(m, MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;
  max31865_write(m, MAX31856_CONFIG_REG, t);
  delay(65);
  rtd = max31865_read16(m, MAX31856_RTDMSB_REG);

  // remove fault
  rtd >>= 1;

  return rtd;
}

float max31865_temperature(max31865_t *m, float RTDNominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp, rpoly;

  Rt = max31865_readRTD(m);
  Rt /= 32768;
  Rt *= refResistor;

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDNominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0) return temp;

  // ugh.
  Rt /= RTDNominal;
  Rt *= 100; // normalize to 100 ohm

  rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}