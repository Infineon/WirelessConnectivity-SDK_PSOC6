/*
 ***************************************************************************************************
 * This file is part of WIRELESS CONNECTIVITY SDK
 *
 *
 * THE SOFTWARE INCLUDING THE SOURCE CODE IS PROVIDED “AS IS”. YOU ACKNOWLEDGE THAT WÜRTH ELEKTRONIK
 * EISOS MAKES NO REPRESENTATIONS AND WARRANTIES OF ANY KIND RELATED TO, BUT NOT LIMITED
 * TO THE NON-INFRINGEMENT OF THIRD PARTIES’ INTELLECTUAL PROPERTY RIGHTS OR THE
 * MERCHANTABILITY OR FITNESS FOR YOUR INTENDED PURPOSE OR USAGE. WÜRTH ELEKTRONIK EISOS DOES NOT
 * WARRANT OR REPRESENT THAT ANY LICENSE, EITHER EXPRESS OR IMPLIED, IS GRANTED UNDER ANY PATENT
 * RIGHT, COPYRIGHT, MASK WORK RIGHT, OR OTHER INTELLECTUAL PROPERTY RIGHT RELATING TO ANY
 * COMBINATION, MACHINE, OR PROCESS IN WHICH THE PRODUCT IS USED. INFORMATION PUBLISHED BY
 * WÜRTH ELEKTRONIK EISOS REGARDING THIRD-PARTY PRODUCTS OR SERVICES DOES NOT CONSTITUTE A LICENSE
 * FROM WÜRTH ELEKTRONIK EISOS TO USE SUCH PRODUCTS OR SERVICES OR A WARRANTY OR ENDORSEMENT
 * THEREOF
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS DRIVER PACKAGE.
 *
 * COPYRIGHT (c) 2025 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 */

/**
 * @file
 * @brief Contains code related to redirecting WE_DEBUG_PRINT() output to UART for debug reasons.
 */



#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>


#include "cyhal.h"
#include "cybsp.h"



#include "debug.h"




#define UART_TX_PIN CYBSP_DEBUG_UART_TX
#define UART_RX_PIN CYBSP_DEBUG_UART_RX

//Debug UART 115200 8N1
#define DEBUG_BAUD_RATE 115200

#if defined(WE_DEBUG) || defined(WE_DEBUG_INIT)

/**
 * @brief Debug ring buffer size.
 */
#define WE_DEBUG_BUFFER_SIZE 2048

/**
 * @brief UART used for debug output.
 */

static cyhal_uart_t debugUartObj;


/**
 * @brief Ring buffer for debug output (output is transmitted asynchronously).
 */
static uint8_t debugBuffer[WE_DEBUG_BUFFER_SIZE];

/**
 * @brief Current write position in ring buffer used for debug output
 * (next character to be queued).
 */
static uint16_t debugBufferWritePos = 0;

/**
 * @brief Current read position in ring buffer used for debug output
 * (next character to be transferred).
 */
static uint16_t debugBufferReadPos = 0;

static bool transferRunning = false;



void debug_uart_event_handler(void *callback_arg, cyhal_uart_event_t event)
{
	if (event & CYHAL_UART_IRQ_TX_DONE)
	{
		        if (debugBufferReadPos != debugBufferWritePos)
        {
            /* Ring buffer read and write positions differ -> transfer is active */

            /* Move to next position in ring buffer */
            debugBufferReadPos++;
            if (debugBufferReadPos >= WE_DEBUG_BUFFER_SIZE)
            {
                debugBufferReadPos = 0;
            }

            transferRunning = debugBufferReadPos != debugBufferWritePos;
            if (transferRunning)
            {
				/* More characters to be transferred -> transmit next character */
                cyhal_uart_putc(&debugUartObj,  debugBuffer[debugBufferReadPos]);
            }
        }
	}

}


/**
 * @brief Initializes UART and connects this interface to WE_DEBUG_PRINT().
 *
 * Note that it is not safe to call WE_DEBUG_PRINT() from different contexts (e.g.
 * inside main and inside ISRs) - there is no guarantee, that the debug output is
 * forwarded correctly in all circumstances.
 *
 * There are two preprocessor defines controlling debug behavior:
 * - WE_DEBUG: Initialize debug UART and enable printing of debug messages in drivers.
 * - WE_DEBUG_INIT: Initialize debug UART but disable printing of debug messages in drivers
 *   (adds support for debugging using printf in user/example code, but drivers don't print
 *   diagnostic/debug info).
 */
void WE_Debug_Init()
{
   
	cyhal_uart_cfg_t uart_config = {
		.data_bits = 8,
		.stop_bits = 1,
		.parity = CYHAL_UART_PARITY_NONE,
		.rx_buffer = NULL,
		.rx_buffer_size = 0
		
	};

	cyhal_uart_init(&debugUartObj, UART_TX_PIN, UART_RX_PIN, NC, NC, NULL, &uart_config);
	cyhal_uart_set_baud(&debugUartObj, DEBUG_BAUD_RATE, NULL);
	cyhal_uart_register_callback(&debugUartObj, debug_uart_event_handler, NULL);
	cyhal_uart_enable_event(&debugUartObj, CYHAL_UART_IRQ_TX_DONE, CYHAL_ISR_PRIORITY_DEFAULT, true);

}

/**
 * @brief Wait until any debug output pending to be written to UART has been transferred.
 */
void WE_Debug_Flush()
{
    while (transferRunning)
    {
    }
}




__attribute__((weak)) int32_t _write(int32_t fd, const cy_char8_t* ptr, int32_t len)
{
    /* Note that it is not safe to call this function from different contexts! */

    if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
    {
        /* Rudimentary check for concurrent access to this function */
        static bool lock = false;
        if (lock != false)
        {
            errno = EIO;
            return -1;
        }
        lock = true;

        uint16_t readPos = debugBufferReadPos;
        uint16_t writePos = debugBufferWritePos;

        /* Remaining space in ring buffer */
        uint16_t spaceRemaining = readPos > writePos ? readPos - writePos - 1 : WE_DEBUG_BUFFER_SIZE - writePos + readPos - 1;

        if (len > spaceRemaining)
        {
            /* Data to be written doesn't fit into ring buffer - limit to spaceRemaining */
            len = spaceRemaining;
        }

        /* Store data in ring buffer */
        int bytesWritten = 0;
        if (writePos > readPos)
        {
            /* Store data between writePos and end of buffer */

            int chunkSize = len;
            if (chunkSize > WE_DEBUG_BUFFER_SIZE - writePos)
            {
                chunkSize = WE_DEBUG_BUFFER_SIZE - writePos;
            }
            memcpy(debugBuffer + writePos, ptr, chunkSize);
            len -= chunkSize;
            writePos = (writePos + chunkSize) % WE_DEBUG_BUFFER_SIZE;
            bytesWritten += chunkSize;
        }

        if (len > 0)
        {
            /* Store remaining data between start of buffer and read pos */

            memcpy(debugBuffer + writePos, ptr + bytesWritten, len);
            writePos = (writePos + len) % WE_DEBUG_BUFFER_SIZE;
            bytesWritten += len;
        }

        debugBufferWritePos = writePos;

        /* Start transfer if not already running */
        if (!transferRunning)
        {
            transferRunning = true;
            cyhal_uart_putc(&debugUartObj,  *(debugBuffer + debugBufferReadPos));
        }

        lock = false;

        return bytesWritten;
    }

    errno = EBADF;
    return -1;
}
#endif // WE_DEBUG
