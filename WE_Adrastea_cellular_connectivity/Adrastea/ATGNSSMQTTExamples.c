/*
 ***************************************************************************************************
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
#include <stdint.h>
#include <stdio.h>
#include <string.h>
 
#include "ATEvent.h"
#include "ATMQTT.h"
#include "ATGNSS.h"
#include "ATPacketDomain.h"
#include "ATGNSSMQTTExamples.h"
#include "AdrasteaI.h"
#include "AdrasteaI_Examples.h"
#include "ATDevice.h"
#include "ATNetService.h"
#include "credentials.h"


#define DEVICE_TYPE "ADRASTEA-I"
#define TELEMETRY_SEND_INTERVAL_MS (30*1000)

#define SAT_QUERY_REG_TIMEOUT_MS (30*1000)
#define SAT_QUERY_POLL_INTERVAL_MS (5000)


#define FIX_QUERY_TIMEOUT_MS (50*1000)
#define FIX_QUERY_POLL_INTERVAL_MS (5000)

static AdrasteaI_ATCommon_IP_Addr_t A1ServerAddress = WE_MQTT_HOST;
static AdrasteaI_ATCommon_Auth_Username_t userName = WE_MQTT_USERNAME;
static AdrasteaI_ATCommon_Auth_Password_t password = WE_MQTT_PASSWORD;
static AdrasteaI_ATMQTT_Topic_Name_t topicOperations = "s/ds";
static AdrasteaI_ATMQTT_Topic_Name_t topicError = "s/e";
static AdrasteaI_ATMQTT_Topic_Name_t pubTopic = "s/us";

static char payloadCreateDev[128];
static char payloadLocation[64];
static char payloadRSSI[32];

void AdrasteaI_ATGNSSMQTT_EventCallback(char* eventText);
static bool AdrasteaI_connectToServer();
static bool AdrasteaI_connectToNetwork();
static bool AdrasteaI_getFix(AdrasteaI_ATGNSS_Fix_t *fix);
static int8_t AdrasteaI_getRSSIindBm(uint8_t rssi);


static AdrasteaI_ATPacketDomain_Network_Registration_Status_t status = {.state = 0};
static AdrasteaI_ATMQTT_Connection_Result_t conResult = {.resultCode = -1};
static AdrasteaI_ATMQTT_Subscription_Result_t subResult = {.resultCode = -1};

static AdrasteaI_ATGNSS_Satellite_Count_t satelliteQueryCount = 0;
static volatile AdrasteaI_ATGNSS_Satellite_Count_t  satelliteQueryEventCount = 0;
static AdrasteaI_ATGNSS_Fix_t fix;


/**
 * @brief This example connects to the cellular network and sends current location to A1 cloud via MQTT
 *
 */
void ATGNSSMQTTExample()
{
	bool ret = false; 
    WE_DEBUG_PRINT("*** Start of Adrastea-I ATGNSS MQTT example ***\r\n");

    if (!AdrasteaI_Init(&AdrasteaI_uart, &AdrasteaI_pins, &AdrasteaI_ATGNSSMQTT_EventCallback))
    {
        WE_DEBUG_PRINT("Initialization error\r\n");
        return;
    }
	
	fix.latitude = 0;
	fix.longitude = 0;
	fix.altitude = 0;
	
    for(;;)
    {
		bool gotFix = AdrasteaI_getFix(&fix);
		
		//Switch to cellular network mode
		ret = AdrasteaI_ATDevice_SetPhoneFunctionality(AdrasteaI_ATDevice_Phone_Functionality_Full, AdrasteaI_ATDevice_Phone_Functionality_Reset_Do_Not_Reset);
   		AdrasteaI_ExamplesPrint("Set Phone Functionality", ret);
	
   		//Connect to the cellular network
   		AdrasteaI_connectToNetwork();
   		
		//Connect to the MQTT broker
		AdrasteaI_connectToServer();
		
		
		//Create and publish Signal strength data
		AdrasteaI_ATNetService_Signal_Quality_t sq;
		AdrasteaI_ATNetService_ReadSignalQuality(&sq);
		AdrasteaI_ExamplesPrint("Read Signal Quality", ret);
		if (ret)
		{
		    WE_DEBUG_PRINT("RSSI: %d, BER: %d\r\n", sq.rssi, sq.ber);
		}
		
		memset(payloadRSSI, 0, sizeof(payloadRSSI));
		
		sprintf(payloadRSSI, "210,%i,%d", AdrasteaI_getRSSIindBm(sq.rssi),sq.ber);
		WE_DEBUG_PRINT("Publish: %s\r\n", payloadRSSI);
	
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payloadRSSI, strlen(payloadRSSI));
		AdrasteaI_ExamplesPrint("Publish", ret);
		
		while (subResult.resultCode != AdrasteaI_ATMQTT_Event_Result_Code_Success)
		{
		    WE_Delay(10);
		}
		
		if(gotFix == true)
		{
			//Create and publish location data
			memset(payloadLocation, 0, sizeof(payloadLocation));
		
			sprintf(payloadLocation, "402,%ld.%06ld,%ld.%06ld, %ld.%06ld", 
			(long)fix.latitude, (long)((fix.latitude - (long)fix.latitude) * 1000000),  
			(long)fix.longitude, (long)((fix.longitude - (long)fix.longitude) * 1000000), 
			(long)fix.altitude, (long)((fix.altitude - (long)fix.altitude) * 1000000));
			
			WE_DEBUG_PRINT("Publish: %s\r\n", payloadLocation);
		
			ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payloadLocation, strlen(payloadLocation));
			AdrasteaI_ExamplesPrint("Publish", ret);
			
			while (subResult.resultCode != AdrasteaI_ATMQTT_Event_Result_Code_Success)
			{
			    WE_Delay(10);
			}
			
		}
		WE_Delay(TELEMETRY_SEND_INTERVAL_MS);
	}
}

static bool AdrasteaI_getFix(AdrasteaI_ATGNSS_Fix_t *fix)
{
    uint32_t elapsed = 0;
    satelliteQueryCount = 0;
    satelliteQueryEventCount = 0;
    
    bool ret = AdrasteaI_ATDevice_SetPhoneFunctionality(AdrasteaI_ATDevice_Phone_Functionality_Min, AdrasteaI_ATDevice_Phone_Functionality_Reset_Do_Not_Reset);
    AdrasteaI_ExamplesPrint("Set Phone Functionality", ret);
    
    ret = AdrasteaI_ATGNSS_StartGNSS(AdrasteaI_ATGNSS_Start_Mode_Cold);
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
		WE_Delay(FIX_QUERY_POLL_INTERVAL_MS);
		elapsed += FIX_QUERY_POLL_INTERVAL_MS;
		ret = AdrasteaI_ATGNSS_QueryGNSSFix(AdrasteaI_ATGNSS_Fix_Relavancy_Current, fix);

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

bool AdrasteaI_connectToNetwork()
{
	uint32_t elapsed = 0;
	bool ret = AdrasteaI_ATPacketDomain_SetNetworkRegistrationResultCode(AdrasteaI_ATPacketDomain_Network_Registration_Result_Code_Enable_with_Location_Info);

    AdrasteaI_ExamplesPrint("Set Network Registration Result Code", ret);

    while (status.state != AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Home_Network &&
           status.state != AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Roaming &&
           elapsed < NETWORK_REG_TIMEOUT_MS)
    {
        WE_Delay(NETWORK_REG_POLL_INTERVAL_MS);
        elapsed += NETWORK_REG_POLL_INTERVAL_MS;
    }

    if (elapsed >= NETWORK_REG_TIMEOUT_MS)
    {
        WE_DEBUG_PRINT("Timeout waiting for network registration.\r\n");
        return false;
    }
    else
    {
        WE_DEBUG_PRINT("Network registered: %d\r\n", status.state);
    }
    return true;
}

bool AdrasteaI_connectToServer()
{
	 bool ret = false;
	/*Read the IMEI number, this will be used as the client ID*/
    AdrasteaI_ATDevice_IMEI_t imei = "";
    ret = AdrasteaI_ATDevice_RequestIMEI(&imei);
    AdrasteaI_ExamplesPrint("Request IMEI", ret);
    WE_DEBUG_PRINT("IMEI: %s\r\n", imei);

	/*Enable all MQTT events*/
    ret = AdrasteaI_ATMQTT_SetMQTTUnsolicitedNotificationEvents(AdrasteaI_ATMQTT_Event_All, 1);
    AdrasteaI_ExamplesPrint("MQTT Unsolicited Notification Events", ret);
	
	AdrasteaI_ATMQTT_Client_ID_t clientID = "";
	
	strncpy(clientID, imei, 15);
	/*Configure MQTT client*/
    ret = AdrasteaI_ATMQTT_ConfigureNodes(AdrasteaI_ATMQTT_Conn_ID_1,clientID, A1ServerAddress, userName, password);
    AdrasteaI_ExamplesPrint("Configure Nodes", ret);

    /*Configure MQTT client*/
    ret = AdrasteaI_ATMQTT_ConfigureProtocol(AdrasteaI_ATMQTT_Conn_ID_1, 1200, 1);
    AdrasteaI_ExamplesPrint("Configure Protocol", ret);

    /*Connect to MQTT broker*/
    ret = AdrasteaI_ATMQTT_Connect(AdrasteaI_ATMQTT_Conn_ID_1);
    AdrasteaI_ExamplesPrint("Connect", ret);

    while (conResult.resultCode != AdrasteaI_ATMQTT_Event_Result_Code_Success)
    {
        WE_Delay(10);
    }
    
	/*Create the device manually*/
	memset(payloadCreateDev, 0, sizeof(payloadCreateDev));
		
	sprintf(payloadCreateDev, "100,%s,%s", imei, DEVICE_TYPE);
	
	ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payloadCreateDev, strlen(payloadCreateDev));
	AdrasteaI_ExamplesPrint("Device registration", ret);
	if(ret == false)
	{
		return ret;
	}
	
	/*Subscribe to predefined topics*/
	ret = AdrasteaI_ATMQTT_Subscribe(AdrasteaI_ATMQTT_Conn_ID_1, AdrasteaI_ATMQTT_QoS_At_Most_Once, topicOperations);
	AdrasteaI_ExamplesPrint("Subscribe", ret);
	
	ret = AdrasteaI_ATMQTT_Subscribe(AdrasteaI_ATMQTT_Conn_ID_1, AdrasteaI_ATMQTT_QoS_At_Most_Once, topicError);
	AdrasteaI_ExamplesPrint("Subscribe", ret);
    return ret;
}

int8_t AdrasteaI_getRSSIindBm(uint8_t rssi)
{
    if (rssi == 0)
    {
        return -113;
    }
    else if (rssi == 1)
    {
        return -111;
    }
    else if (rssi >= 2 && rssi <= 30)
    {
        return -113 + 2 * rssi;
    }
    else if (rssi == 31)
    {
        return -51;
    }
    else if (rssi == 99)
    {
        return 99; // or a special value like INT8_MIN to indicate "unknown"
    }
    else
    {
        return 99; // or handle as an error
    }
}

void AdrasteaI_ATGNSSMQTT_EventCallback(char* eventText)
{
    AdrasteaI_ATEvent_t event;
    if (false == AdrasteaI_ATEvent_ParseEventType(&eventText, &event))
    {
        return;
    }

    switch (event)
    {
        case AdrasteaI_ATEvent_MQTT_Connection_Confirmation:
        {
            AdrasteaI_ATMQTT_ParseConnectionConfirmationEvent(eventText, &conResult);
            break;
        }
        case AdrasteaI_ATEvent_MQTT_Subscription_Confirmation:
        {
            AdrasteaI_ATMQTT_ParseSubscriptionConfirmationEvent(eventText, &subResult);
            break;
        }
        case AdrasteaI_ATEvent_MQTT_Publication_Received:
        {
            AdrasteaI_ATMQTT_Publication_Received_Result_t result;
            char payload[128];
            result.payload = payload;
            result.payloadMaxBufferSize = sizeof(payload);
            if (!AdrasteaI_ATMQTT_ParsePublicationReceivedEvent(eventText, &result))
            {
                return;
            }
            WE_DEBUG_PRINT("Connection ID: %d, Message ID: %d, Topic Name: %s, Payload Size: %d, Payload: %s\r\n", result.connID, result.msgID, result.topicName, result.payloadSize, result.payload);
            break;
        }
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
