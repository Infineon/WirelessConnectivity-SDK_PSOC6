/**************************************************************************
 *  @file     pn532_stm32f1.c
 *  @author   Yehui from Waveshare
 *  @license  BSD
 *  
 *  This implements the peripheral interfaces.
 *  
 *  Check out the links above for our tutorials and wiring diagrams 
 *  These chips use SPI communicate.
 *  
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 **************************************************************************/

#include "cyhal_i2c.h"
#include "cyhal_spi.h"
#include "cyhal_gpio.h"
#include "cyhal_system.h"
#include "cyhal_lptimer.h"
#include "pn532.h"
#include "pn532_cyhal.h"
#define _SPI_STATREAD                   0x02
#define _SPI_DATAWRITE                  0x01
#define _SPI_DATAREAD                   0x03
#define _SPI_READY                      0x01

#define PN532_CYHAL_SPI_TIMEOUT_MS      10
// This indicates if the bits read/write should be reversed
#define _SPI_HARDWARE_LSB


#define PN532_RST_PIN                   P10_5
#define PN532_CYHAL_I2C_TIMEOUT_MS      10


/* Provide a local implementation for cyhal_system_get_tick() to resolve linker error. */
/** The LPTIMER object used for `cyhal_system_get_tick()` */
static cyhal_lptimer_t _cyhal_system_tick_obj;

uint32_t cyhal_system_get_tick(void)
{
    if (_cyhal_system_tick_obj.base == NULL) {
        cyhal_lptimer_init(&_cyhal_system_tick_obj);
    }
    return cyhal_lptimer_read(&_cyhal_system_tick_obj);
}

/**************************************************************************
 * Reset and Log implements
 **************************************************************************/
static int PN532_CYHAL_Reset(void) {
    cyhal_gpio_write(PN532_RST_PIN, true);
    cyhal_system_delay_ms(100);
    cyhal_gpio_write(PN532_RST_PIN, false);
    cyhal_system_delay_ms(500);
    cyhal_gpio_write(PN532_RST_PIN, true);
    cyhal_system_delay_ms(100);
    return PN532_STATUS_OK;
}

void PN532_Log(const char* log) {
    printf("%s\r\n", log);
}

/**************************************************************************
 * End: Reset and Log implements
 **************************************************************************/
/**************************************************************************
 * SPI
 **************************************************************************/
static uint8_t reverse_bit(uint8_t num) {
    uint8_t result = 0;
    for (uint8_t i = 0; i < 8; i++) {
        result <<= 1;
        result += (num & 1);
        num >>= 1;
    }
    return result;
}

static void cyhal_spi_rw(pn532_cyhal_spi_t *ctx, uint8_t* data, uint8_t count) {
    CY_ASSERT(NULL != ctx);
    cyhal_gpio_write(ctx->ss, 0); // Assert SS
    cyhal_system_delay_ms(1);
#ifndef _SPI_HARDWARE_LSB
    for (uint8_t i = 0; i < count; i++) {
        data[i] = reverse_bit(data[i]);
    }
#endif
    cyhal_spi_transfer(ctx->spi, data, count, data, count, PN532_CYHAL_SPI_TIMEOUT_MS);
#ifndef _SPI_HARDWARE_LSB
    for (uint8_t i = 0; i < count; i++) {
        data[i] = reverse_bit(data[i]);
    }
#endif
    cyhal_system_delay_ms(1);
    cyhal_gpio_write(ctx->ss, 1); // Deassert SS
}


static int PN532_CYHAL_SPI_ReadData(void *if_context, uint8_t* data, uint16_t count) {
    uint8_t frame[count + 1];
    pn532_cyhal_spi_t *ctx = (pn532_cyhal_spi_t*)if_context;
    frame[0] = _SPI_DATAREAD;
    cyhal_system_delay_ms(5);
    cyhal_spi_rw(ctx, frame, count + 1);
    for (uint8_t i = 0; i < count; i++) {
        data[i] = frame[i + 1];
    }
    return PN532_STATUS_OK;
}


static int PN532_CYHAL_SPI_WriteData(void *if_context, uint8_t *data, uint16_t count) {
    uint8_t frame[count + 1];
    pn532_cyhal_spi_t *ctx = (pn532_cyhal_spi_t*)if_context;
    frame[0] = _SPI_DATAWRITE;
    for (uint8_t i = 0; i < count; i++) {
        frame[i + 1] = data[i];
    }
    cyhal_spi_rw(ctx, frame, count + 1);
    return PN532_STATUS_OK;
}


static bool PN532_CYHAL_SPI_WaitReady(void *if_context, uint32_t timeout) {
    uint8_t status[] = {_SPI_STATREAD, 0x00};
    pn532_cyhal_spi_t *ctx = (pn532_cyhal_spi_t*)if_context;
    uint32_t tickstart = cyhal_system_get_tick();
    while ((cyhal_system_get_tick() - tickstart) < timeout) {
        cyhal_system_delay_ms(10);
        cyhal_spi_rw(ctx, status, sizeof(status));
        if (status[1] == _SPI_READY) {
            return true;
        } else {
            cyhal_system_delay_ms(5);
        }
    }
    return false;
}


static int PN532_CYHAL_SPI_Wakeup(void *if_context) {
    uint8_t data[] = {0x00};
    pn532_cyhal_spi_t *ctx = (pn532_cyhal_spi_t*)if_context;
    cyhal_system_delay_ms(1000);
    cyhal_gpio_write(ctx->ss, 0);
    cyhal_system_delay_ms(2); // T_osc_start
    cyhal_spi_rw(ctx, data, 1);
    cyhal_system_delay_ms(1000);
    cyhal_gpio_write(ctx->ss, 1);
    return PN532_STATUS_OK;
}


void PN532_SPI_Init(PN532* pn532, pn532_cyhal_spi_t *ctx) {
    pn532->_if_context = ctx;
    pn532->reset = PN532_CYHAL_Reset;
    pn532->read_data = PN532_CYHAL_SPI_ReadData;
    pn532->write_data = PN532_CYHAL_SPI_WriteData;
    pn532->wait_ready = PN532_CYHAL_SPI_WaitReady;
    pn532->wakeup = PN532_CYHAL_SPI_Wakeup;
    pn532->log = PN532_Log;
    pn532->wakeup(pn532->_if_context);
}
/**************************************************************************
 * End: SPI
 **************************************************************************/

/**************************************************************************
 * CYHAL I2C
 **************************************************************************/
static int PN532_CYHAL_I2C_ReadData(void *if_context, uint8_t* data, uint16_t count) {
    pn532_cyhal_i2c_t *ctx = (pn532_cyhal_i2c_t*)if_context;
    uint8_t status = 0x00;
    cy_rslt_t result = cyhal_i2c_master_read(ctx->i2c, ctx->address, &status, 1, PN532_CYHAL_I2C_TIMEOUT_MS, true);
    if (result != CY_RSLT_SUCCESS || status != PN532_I2C_READY) {
        return PN532_STATUS_ERROR;
    }
    uint8_t frame[count + 1];
    result = cyhal_i2c_master_read(ctx->i2c, ctx->address, frame, count + 1, PN532_CYHAL_I2C_TIMEOUT_MS, true);
    if (result != CY_RSLT_SUCCESS) {
        return PN532_STATUS_ERROR;
    }
    for (uint8_t i = 0; i < count; i++) {
        data[i] = frame[i + 1];
    }
    return PN532_STATUS_OK;
}

static int PN532_CYHAL_I2C_WriteData(void *if_context, uint8_t *data, uint16_t count) {
    pn532_cyhal_i2c_t *ctx = (pn532_cyhal_i2c_t*)if_context;
    cy_rslt_t result = cyhal_i2c_master_write(ctx->i2c, ctx->address, data, count, PN532_CYHAL_I2C_TIMEOUT_MS, true);
    if (result != CY_RSLT_SUCCESS) {
        return PN532_STATUS_ERROR;
    }
    return PN532_STATUS_OK;
}

static bool PN532_CYHAL_I2C_WaitReady(void *if_context, uint32_t timeout) {
    pn532_cyhal_i2c_t *ctx = (pn532_cyhal_i2c_t*)if_context;
    uint8_t status = 0x00;
    uint32_t tickstart = cyhal_system_get_tick();
    while ((cyhal_system_get_tick() - tickstart) < timeout) {
        cy_rslt_t result = cyhal_i2c_master_read(ctx->i2c, ctx->address, &status, 1, PN532_CYHAL_I2C_TIMEOUT_MS, true);
        if (result == CY_RSLT_SUCCESS && status == PN532_I2C_READY) {
            return true;
        }
        cyhal_system_delay_ms(5);
    }
    return false;
}

static int PN532_CYHAL_I2C_Wakeup(void *if_context) {
    // Wakeup is performed by sending a dummy I2C transaction.
    // The PN532 wakes up on any I2C start condition.
    pn532_cyhal_i2c_t *ctx = (pn532_cyhal_i2c_t*)if_context;
    uint8_t dummy = 0;
    cyhal_i2c_master_write(ctx->i2c, ctx->address, &dummy, 0, PN532_CYHAL_I2C_TIMEOUT_MS, true);
    cyhal_system_delay_ms(500); // Give it time to wake up
    return PN532_STATUS_OK;
}

void PN532_I2C_Init(PN532* pn532, pn532_cyhal_i2c_t *ctx) {
    pn532->_if_context = ctx;
    pn532->reset = PN532_CYHAL_Reset;
    pn532->read_data = PN532_CYHAL_I2C_ReadData;
    pn532->write_data = PN532_CYHAL_I2C_WriteData;
    pn532->wait_ready = PN532_CYHAL_I2C_WaitReady;
    pn532->wakeup = PN532_CYHAL_I2C_Wakeup;
    pn532->log = PN532_Log;
    pn532->wakeup(pn532->_if_context);
}
/**************************************************************************
 * End: CYHAL I2C
 **************************************************************************/
