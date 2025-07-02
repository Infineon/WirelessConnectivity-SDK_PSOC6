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
 * @brief Contains global function definitions for the Wireless Connectivity SDK for PSOC6.
 */
#include <string.h>

#include "global.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cyhal_uart.h"
#include "global_types.h"



#ifdef __cplusplus
extern "C"
{
#endif


#define UART_TX_PIN P9_1
#define UART_RX_PIN P9_0

#define TEMP_RX_SIZE 1


cyhal_uart_t uart_obj;
uint8_t temp_rx[TEMP_RX_SIZE];

volatile uint32_t ms_ticks = 0;
volatile bool rx_done = false;
volatile bool tx_done = false;

static WE_UART_HandleRxByte_t *uartRxCallback ;

static void start_async_rx();

    /*              Functions              */

void SysTick_Handler(void)
{
	ms_ticks++; // Increment every 1 ms
}
    /**
 * @brief Initialise the microcontroller and setup system clock 
 */
void WE_Platform_Init(void)
{	
	/*Start the system tick timer*/
	Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, (8000000/1000));
	Cy_SysTick_SetCallback(0, SysTick_Handler);
	Cy_SysTick_Enable();
}




    /**
 * @brief Disables the interrupts
 */
void WE_Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
}

    /**
 * @brief Initialises the pins
 *
 * @param[in] pins: pins to be initialised
 * @param[in] numPins: number of pins
 *
 * @return true if request succeeded,
 *         false otherwise
 */
bool WE_InitPins(WE_Pin_t pins[], uint8_t numPins)
{
    for (uint8_t i = 0; i < numPins; i++)
    {
        if (IS_WE_PIN_UNDEFINED(pins[i]))
        {
            /* Unused */
            continue;
        }
		if (WE_Pin_Type_Output == pins[i].type)
		{
			cyhal_gpio_init((int)(uintptr_t)(pins[i].pin_def), CYHAL_GPIO_DIR_OUTPUT,
		                 CYHAL_GPIO_DRIVE_STRONG, WE_Pin_Level_Low);
		}
		if (WE_Pin_Type_Input == pins[i].type)
		{
		    cyhal_gpio_init((int)(uintptr_t)(pins[i].pin_def), CYHAL_GPIO_DIR_INPUT,
		                 CYHAL_GPIO_DRIVE_PULL_NONE, WE_Pin_Level_Low);
		}
    }

    return true;
}

bool WE_Reconfigure(WE_Pin_t pin)
{
    if (IS_WE_PIN_UNDEFINED(pin))
    {
        return false;
    }
    
    if (WE_Pin_Type_Output == pin.type)
	{
	    cyhal_gpio_init((int)(uintptr_t)pin.pin_def, CYHAL_GPIO_DIR_OUTPUT,
	                 CYHAL_GPIO_DRIVE_STRONG, WE_Pin_Level_Low);
	}
	if (WE_Pin_Type_Input == pin.type)
	{
	    cyhal_gpio_init((int)(uintptr_t)pin.pin_def, CYHAL_GPIO_DIR_INPUT,
	                 CYHAL_GPIO_DRIVE_PULL_NONE, WE_Pin_Level_Low);
	}
    return true;
}

    /**
 * @brief Deinitialises the pin to their default reset values
 *
 * @param[in] pin: pin to be deinitialised
 *
 * @return true if request succeeded,
 *         false otherwise
 */
bool WE_DeinitPin(WE_Pin_t pin)
{
	cyhal_gpio_free((int)(uintptr_t)pin.pin_def);
	return true;
}

    /**
 * @brief Switch pin to output high/low
 *
 * @param[in] pin Output pin to be set
 * @param[in] out Output level to be set
 * @return true if request succeeded, false otherwise
 */

bool WE_SetPin(WE_Pin_t pin, WE_Pin_Level_t out)
{
    if (IS_WE_PIN_UNDEFINED(pin))
    {
        return false;
    }
	cyhal_gpio_write((int)(uintptr_t)pin.pin_def, out);
    return true;
}

    /**
 * @brief Gets the pin level
 *
 * @param[in] pin: the pin to be checked
 *
 * @param[out] pin_levelP: the pin level
 *
 * @return true if request succeeded,
 *         false otherwise
 *         
 */
bool WE_GetPinLevel(WE_Pin_t pin, WE_Pin_Level_t* pin_levelP)
{
    if (IS_WE_PIN_UNDEFINED(pin) || (pin_levelP == NULL))
    {
        return false;
    }
    
    if(cyhal_gpio_read((int)(uintptr_t)pin.pin_def))
    {
		*pin_levelP = WE_Pin_Level_High;
	}else 
	{
		*pin_levelP = WE_Pin_Level_Low;
    }
    return true;
}

    /**
 * @brief Delays the microcontoller for the specified time.
 *
 * @param[in] sleepForMs: time in milliseconds.
 *         
 */
void WE_Delay(uint16_t sleepForMs)
{
    if (sleepForMs > 0)
    {
        cyhal_system_delay_ms((uint32_t)sleepForMs);
    }
}

    /**
 * @brief Delays the microcontoller for the specified time.
 *
 * @param[in] sleepForMs: time in microseconds.
 *         
 */
void WE_DelayMicroseconds(uint32_t sleepForUsec)
{
    /* Microsecond tick is disabled: round to ms */
    cyhal_system_delay_us(sleepForUsec);
}

    /**
 * @brief Gets the elapsed time since startup
 *
 * @return returns elapsed in ms
 *         
 */
uint32_t WE_GetTick() { return ms_ticks; }
    
    /**
 * @brief Gets the elapsed time since startup
 *
 * @return returns elapsed in microseconds
 *         
 */
uint32_t WE_GetTickMicroseconds()
{
    /* Microsecond tick is disabled: return ms tick * 1000 */
    return ms_ticks * 1000;
}
    /**
 * @brief Gets the Driver version
 *
 * @param[out] version: will contain the version value
 *
 * @return true if request succeeded,
 *         false otherwise        
 */
bool WE_GetDriverVersion(uint8_t* version)
{
    uint8_t help[3] = WE_WIRELESS_CONNECTIVITY_SDK_VERSION;
    memcpy(version, help, 3);
    return true;
}

void uart_event_handler(void *callback_arg, cyhal_uart_event_t event) {
  if (event & CYHAL_UART_IRQ_RX_DONE) {
    rx_done = true;
    (*uartRxCallback)(temp_rx, TEMP_RX_SIZE);
    start_async_rx();
  }
  if (event & CYHAL_UART_IRQ_TX_DONE) {
    tx_done = true;
  }
}

void start_async_rx()
{
    rx_done = false;
    memset(temp_rx,0,TEMP_RX_SIZE);
    cyhal_uart_read_async(&uart_obj, temp_rx, TEMP_RX_SIZE);
}


/**
 * @brief Initialize and start the UART.
 *
 * @param[in] baudrate Baud rate of the serial interface
 * @param[in] flowControl Enable/disable flow control
 * @param[in] parity Parity bit configuration
 * @param[in] rxByteHandlerP Pointer to the handle rx byte function inside the driver. (this function should be called by the ISR for uart on data reception)
 */
bool WE_UART1_Init(uint32_t baudrate, WE_FlowControl_t flowControl, WE_Parity_t parity, WE_UART_HandleRxByte_t *rxByteHandlerP)
{
	/*Configure UART*/
	cy_rslt_t result;
		cyhal_uart_cfg_t uart_config = {
		.data_bits = 8,
		.stop_bits = 1,
		.rx_buffer = NULL,
		.rx_buffer_size = 0
	};
	switch(parity)
	{
		case WE_Parity_None:
			uart_config.parity = CYHAL_UART_PARITY_NONE;
			break;

         case WE_Parity_Odd:
         	uart_config.parity = CYHAL_UART_PARITY_ODD;
            break;
                
         case WE_Parity_Even:
         	uart_config.parity = CYHAL_UART_PARITY_EVEN;
            break;
     }
    
    /*Initialize UART with the configuration*/
    result = cyhal_uart_init(&uart_obj, UART_TX_PIN, UART_RX_PIN, NC, NC, NULL, &uart_config);
   	
   	if (result != CY_RSLT_SUCCESS)
	{
		return false;
	}
    cyhal_uart_set_baud(&uart_obj, baudrate, NULL);
    
    switch (flowControl) {
    case WE_FlowControl_NoFlowControl:
    	break;
    case WE_FlowControl_RTSOnly:
    	cyhal_uart_enable_flow_control(&uart_obj, false, true);
    	break;
    case WE_FlowControl_CTSOnly:
        cyhal_uart_enable_flow_control(&uart_obj, true, false);
    	break;
    case WE_FlowControl_RTSAndCTS:
    	cyhal_uart_enable_flow_control(&uart_obj, true, true);
      break;
    }
    cyhal_uart_register_callback(&uart_obj, uart_event_handler, NULL);
    cyhal_uart_enable_event(&uart_obj, (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE | CYHAL_UART_IRQ_TX_DONE), CYHAL_ISR_PRIORITY_DEFAULT, true);
	cyhal_uart_set_async_mode(&uart_obj, CYHAL_ASYNC_SW, CYHAL_DMA_PRIORITY_DEFAULT);
	
	uartRxCallback = rxByteHandlerP;
	tx_done = true;
	rx_done = true;
	
	/*Start Receive*/
    start_async_rx();
	
	return true;
}

/**
 * @brief Deinitialize and stop the UART.
 */
 bool WE_UART1_DeInit()
 {
	cyhal_uart_free(&uart_obj);
	return true;
 }

/**
 * @brief Transmit data via UART.
 *
 * @param[in] data Pointer to data buffer (data to be sent)
 * @param[in] length Number of bytes to be sent
 */
bool WE_UART1_Transmit(const uint8_t *data, uint16_t length)
{
	cy_rslt_t result;
	
	if (data == NULL || length == 0)
		return false;

	while (!tx_done)
	{
		//Wait for previous TX to be completed
	}
	tx_done = false;
	result = cyhal_uart_write_async(&uart_obj, (void *)data, length);
	if (result != CY_RSLT_SUCCESS)
	{
		return false;
	}
	return true;
}
#ifdef __cplusplus
}
#endif
