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

#include <stdio.h>

#include "ATDevice.h"
#include "ATEvent.h"
#include "ATGNSS.h"
#include "ATPacketDomain.h"
#include "AdrasteaI.h"
#include "AdrasteaI_Examples.h"

#define SAT_QUERY_REG_TIMEOUT_MS (30*1000)
#define SAT_QUERY_POLL_INTERVAL_MS (2000)


#define FIX_QUERY_TIMEOUT_MS (50*1000)
#define FIX_QUERY_POLL_INTERVAL_MS (5000)

void AdrasteaI_ATGNSS_EventCallback(char* eventText);
static bool AdrasteaI_getFix(AdrasteaI_ATGNSS_Fix_t *fix);

static AdrasteaI_ATPacketDomain_Network_Registration_Status_t status = {.state = 0};

static AdrasteaI_ATGNSS_Satellite_Count_t satelliteQueryCount = 0, satelliteQueryEventCount = 0;

/**
 * @brief This example configures the GNSS part of the Adrastea-I and tries to get a fix (current positional data)
 *
 */
void ATGNSSExample()
{
    WE_DEBUG_PRINT("*** Start of Adrastea-I ATGNSS example ***\r\n");

    if (!AdrasteaI_Init(&AdrasteaI_uart, &AdrasteaI_pins, &AdrasteaI_ATGNSS_EventCallback))
    {
        WE_DEBUG_PRINT("Initialization error\r\n");
        return;
    }
    bool ret = AdrasteaI_ATDevice_SetPhoneFunctionality(AdrasteaI_ATDevice_Phone_Functionality_Min, AdrasteaI_ATDevice_Phone_Functionality_Reset_Do_Not_Reset);
    AdrasteaI_ExamplesPrint("Set Phone Functionality", ret);

    AdrasteaI_ATGNSS_Fix_t fix;
    while (1)
    {
		AdrasteaI_getFix(&fix);
        WE_Delay(60000);
    }
}

static bool AdrasteaI_getFix(AdrasteaI_ATGNSS_Fix_t *fix)
{
    uint32_t elapsed = 0;
    satelliteQueryCount = 0;
    satelliteQueryEventCount = 0;
    
    bool  ret = AdrasteaI_ATGNSS_StartGNSS(AdrasteaI_ATGNSS_Start_Mode_Cold);
    AdrasteaI_ExamplesPrint("Start GNSS", ret);

    AdrasteaI_ATGNSS_Satellite_Systems_t satSystems = {.systems = {.GPS = AdrasteaI_ATGNSS_Runtime_Mode_State_Set, .GLONASS = AdrasteaI_ATGNSS_Runtime_Mode_State_Set}};
    ret = AdrasteaI_ATGNSS_SetSatelliteSystems(satSystems);
    AdrasteaI_ExamplesPrint("Set Satellite Systems", ret);
    
    WE_DEBUG_PRINT("Waiting for at least 4 satellites to be in the range\r\n");
   	while((satelliteQueryCount < 4) && (elapsed < SAT_QUERY_REG_TIMEOUT_MS))
	{
		AdrasteaI_ATGNSS_QueryGNSSSatellites(&satelliteQueryCount);
		WE_Delay(SAT_QUERY_POLL_INTERVAL_MS);
        elapsed += SAT_QUERY_POLL_INTERVAL_MS;
	}
	if(elapsed >= SAT_QUERY_REG_TIMEOUT_MS)
	{
		WE_DEBUG_PRINT("Satellite query timed out\r\n");		
		return false;
	}
    while (satelliteQueryCount != satelliteQueryEventCount)
    {
        WE_Delay(1000);
        WE_DEBUG_PRINT("Waiting for information from all visible satellites \r\n");
    }

	elapsed = 0;
	fix->fixType =  AdrasteaI_ATGNSS_Fix_Type_Invalid;
  	
	while ((elapsed < FIX_QUERY_TIMEOUT_MS) && (fix->fixType <= 0)) 
    {
		ret = AdrasteaI_ATGNSS_QueryGNSSFix(AdrasteaI_ATGNSS_Fix_Relavancy_Current, fix);
		WE_Delay(FIX_QUERY_POLL_INTERVAL_MS);
		elapsed += FIX_QUERY_POLL_INTERVAL_MS;
	}
    
    if(elapsed > FIX_QUERY_TIMEOUT_MS)
	{
		WE_DEBUG_PRINT("Fix timed out\r\n" );		
		return false;
	}

    WE_DEBUG_PRINT("Date: %u.%u.%d\r\n", fix->date.Day, fix->date.Month, fix->date.Year);
	WE_DEBUG_PRINT("Time: %u:%u:%u\r\n", fix->time.Hours, fix->time.Minutes, fix->time.Seconds);
	WE_DEBUG_PRINT("Fix Latitude: %ld.%06ld, Longitude: %ld.%06ld, Altitude: %ld.%06ld\r\n",
	       (long)fix->latitude, (long)((fix->latitude - (long)fix->latitude) * 1000000),
	       (long)fix->longitude, (long)((fix->longitude - (long)fix->longitude) * 1000000),
	       (long)fix->altitude, (long)((fix->altitude - (long)fix->altitude) * 1000000));
	return true;

}

void AdrasteaI_ATGNSS_EventCallback(char* eventText)
{
    AdrasteaI_ATEvent_t event;
    if (false == AdrasteaI_ATEvent_ParseEventType(&eventText, &event))
    {
        return;
    }

    switch (event)
    {
        case AdrasteaI_ATEvent_PacketDomain_Network_Registration_Status:
        {
            AdrasteaI_ATPacketDomain_ParseNetworkRegistrationStatusEvent(eventText, &status);
            break;
        }
        case AdrasteaI_ATEvent_GNSS_Satellite_Query:
        {
            AdrasteaI_ATGNSS_Satellite_t satellite;
            if (!AdrasteaI_ATGNSS_ParseSatelliteQueryEvent(eventText, &satellite))
            {
                return;
            }
            WE_DEBUG_PRINT("PRN: %d, Elevation: %d, Azimuth: %d, SNR: %d\r\n", satellite.prn, satellite.elevation, satellite.azimuth, satellite.snr);
            satelliteQueryEventCount++;
            break;
        }
        default:
            break;
    }
}
