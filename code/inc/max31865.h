/**
 ******************************************************************************
 * @file    max31865.h
 * @author  Pablo Fuentes
 * @version V1.0.1
 * @date    2019
 * @brief   Header de MAX31865 Library
 ******************************************************************************
 */

#ifndef __MAX31865_H
#define __MAX31865_H

#include "eonOS.h"

typedef struct {
  SPI_TypeDef *SPIx;
  pin_t cs;
} max31865_t;

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define PT100_RREF 430.0
#define PT1000_RREF 4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define PT100_RNOMINAL 100.0
#define PT1000_RNOMINAL 1000.0

// Errors
#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04

typedef enum {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

/**
 ===============================================================================
              ##### FUNCIONES #####
 ===============================================================================
 */

void max31865_deselect(max31865_t *m);
bool max31865_init(max31865_t *m, max31865_numwires_t wires);
void max31865_setWires(max31865_t *m, max31865_numwires_t wires);
uint8_t max31865_readFault(max31865_t *m);
void max31865_clearFault(max31865_t *m);
void max31865_autoConvert(max31865_t *m, bool b);
void max31865_enableBias(max31865_t *m, bool b);
uint16_t max31865_readRTD(max31865_t *m);
float max31865_temperature(max31865_t *m, float RTDNominal, float refResistor);

#endif