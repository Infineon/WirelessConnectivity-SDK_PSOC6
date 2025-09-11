/*
 ***************************************************************************************************
 * This file is part of WIRELESS CONNECTIVITY SDK for PSOC6:
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
 
 
#include "cyhal.h"

#include "global.h"
#include "ATDeviceExamples.h"
#include "ATNetServiceExamples.h"
#include "ATMQTTExamples.h"
#include "ATHTTPExamples.h"
#include "ATSIMExamples.h"
#include "ATGNSSExamples.h"
#include "ATPowerExamples.h"
#include "AdrasteaI_Examples.h"

#define ADRASTEA_PSOC_RESET_PIN P9_5
#define ADRASTEA_PSOC_WAKE_UP_PIN P9_6


/**
 * @brief Definition of the control pins
 */
AdrasteaI_Pins_t AdrasteaI_pins = {
    .AdrasteaI_Pin_Reset = WE_PIN((void*)(uintptr_t)ADRASTEA_PSOC_RESET_PIN),
    .AdrasteaI_Pin_WakeUp = WE_PIN((void*)(uintptr_t)ADRASTEA_PSOC_WAKE_UP_PIN)
    };

/**
 * @brief Definition of the uart
 */
WE_UART_t AdrasteaI_uart;

/**
 * @brief Runs Adrastea examples.
 *
 * Comment/uncomment lines in this function to start the desired example.
 */
void AdrasteaI_Examples()
{
    AdrasteaI_uart.baudrate = 115200;
    AdrasteaI_uart.flowControl = WE_FlowControl_NoFlowControl;
    AdrasteaI_uart.parity = WE_Parity_None;
    AdrasteaI_uart.uartInit = WE_UART1_Init;
    AdrasteaI_uart.uartDeinit = WE_UART1_DeInit;
    AdrasteaI_uart.uartTransmit = WE_UART1_Transmit;
	
	/*Select one example to run by uncommenting the same. All the other examples should remain commented */
    //ATDeviceExample();
    //ATMQTTExample();
    ATMQTTGNSSExample();
    //ATPowerExample();
    //ATNetServiceExample();
    //ATHTTPExample();
    //ATGNSSExample();
    //ATSIMExample();
    return;
}

/**
 * @brief Prints the supplied string, prefixed with OK or NOK (depending on the success parameter).
 *
 * @param str String to print
 * @param success Variable indicating if action was ok
 */
void AdrasteaI_ExamplesPrint(char* str, bool success) { WE_DEBUG_PRINT("%s%s\r\n", success ? "OK    " : "NOK   ", str); }
