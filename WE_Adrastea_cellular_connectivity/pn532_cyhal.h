/**************************************************************************
 *  @file     pn532_cyhal.h
 *  @author   Gemini Code Assist
 *  @license  BSD
 *
 *  This file contains the CYHAL-specific interface for the PN532 driver.
 *
 **************************************************************************/

#ifndef PN532_CYHAL_H
#define PN532_CYHAL_H

#include "cyhal_i2c.h"
#include "cyhal_spi.h"
#include "cyhal_gpio.h"
#include "pn532.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    cyhal_i2c_t *i2c;
    uint8_t address;
} pn532_cyhal_i2c_t;

typedef struct {
    cyhal_spi_t *spi; // Pointer to CYHAL SPI object
    cyhal_gpio_t ss;  // Slave select pin
} pn532_cyhal_spi_t;


void PN532_I2C_Init(PN532* pn532, pn532_cyhal_i2c_t *ctx);
void PN532_SPI_Init(PN532* pn532, pn532_cyhal_spi_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* PN532_CYHAL_H */