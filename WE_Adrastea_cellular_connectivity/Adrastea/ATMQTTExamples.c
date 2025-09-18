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

#include "cyhal.h"
#include "cybsp.h"
 
#include "ATSIM.h"
#include "ATEvent.h"
#include "ATMQTT.h"
#include "ATPacketDomain.h"
#include "ATMQTTExamples.h"
#include "AdrasteaI.h"
#include "AdrasteaI_Examples.h"
#include "ATDevice.h"
#include "ATNetService.h"

#include "ATGNSS.h"
#include "ATPacketDomain.h"


#include "xensiv_dps3xx_mtb.h"
#include "mtb_bmi270.h"
#include "mtb_bmm350.h"

#include "mtb_ssd1306.h"
#include "GUI.h"

#include "pn532_cyhal.h"

#define DEVICE_TYPE "ADRASTEA-I"
#define TELEMETRY_SEND_INTERVAL_MS (30*1000)

AdrasteaI_ATCommon_IP_Addr_t A1ServerAddress = "avl.iotstg.a1.digital";
//AdrasteaI_ATCommon_Auth_Username_t userName = "<Tenant ID>/<user_name>" ;
//AdrasteaI_ATCommon_Auth_Password_t password = "<password>";
AdrasteaI_ATCommon_Auth_Username_t userName = "t99024930/Miniaturized-ADAS" ;
AdrasteaI_ATCommon_Auth_Password_t password = "mqtt4MADAS2025!";
AdrasteaI_ATMQTT_Topic_Name_t topicOperations = "s/ds";
AdrasteaI_ATMQTT_Topic_Name_t topicError = "s/e";
AdrasteaI_ATMQTT_Topic_Name_t pubTopic = "s/us";


char payload[128];

void AdrasteaI_ATMQTT_EventCallback(char* eventText);
int8_t AdrasteaI_getRSSIindBm(uint8_t rssi);

static AdrasteaI_ATPacketDomain_Network_Registration_Status_t status = {.state = 0};
static AdrasteaI_ATMQTT_Connection_Result_t conResult = {.resultCode = -1};
static AdrasteaI_ATMQTT_Subscription_Result_t subResult = {.resultCode = -1};

// Sensor and I2C definition
xensiv_dps3xx_t pressure_sensor;
mtb_bmi270_t imu;
mtb_bmm350_t magnetometer;

PN532 pn532;

cyhal_i2c_t i2c;
cyhal_i2c_cfg_t i2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = 400000
};

// PINs for SDA and SCL
#define DPS_I2C_SDA (CYBSP_I2C_SDA) 
#define DPS_I2C_SCL (CYBSP_I2C_SCL) 

#define BMM_I2C_SDA (CYBSP_I2C_SDA)
#define BMM_I2C_SCL (CYBSP_I2C_SCL)

#define IMU_I2C_SDA (CYBSP_I2C_SDA)
#define IMU_I2C_SCL (CYBSP_I2C_SCL)


void bytes_to_hex_string(const uint8_t *data, size_t len, char *out_str);
uint64_t bytes_to_uint64(const uint8_t *data, size_t len);

/**
 * @brief This example connects to the cellular network and accesses mosquitto.org via MQTT
 *
 */
void ATMQTTExample()
{
    bool ret;

    WE_DEBUG_PRINT("*** Start of Adrastea-I ATMQTT example ***\r\n");

    if (!AdrasteaI_Init(&AdrasteaI_uart, &AdrasteaI_pins, &AdrasteaI_ATMQTT_EventCallback))
    {
        WE_DEBUG_PRINT("Initialization error\r\n");
        return;
    }
    
     /* Initialize i2c for pressure sensor */
    ret = cyhal_i2c_init(&i2c, DPS_I2C_SDA, DPS_I2C_SCL, NULL);
    CY_ASSERT(ret == CY_RSLT_SUCCESS);
    ret = cyhal_i2c_configure(&i2c, &i2c_cfg);
    CY_ASSERT(ret == CY_RSLT_SUCCESS);

    /* Initialize pressure sensor */
    ret = xensiv_dps3xx_mtb_init_i2c(&pressure_sensor, &i2c, XENSIV_DPS3XX_I2C_ADDR_DEFAULT);
    CY_ASSERT(ret == CY_RSLT_SUCCESS);
    



    AdrasteaI_ATSIM_Lock_Status_t lockStatus;
    ret = AdrasteaI_ATSIM_ReadFacilityLock(AdrasteaI_ATSIM_Facility_SC, &lockStatus);
    AdrasteaI_ExamplesPrint("Read Facility Lock", ret);
    if(ret)
    {
		WE_DEBUG_PRINT("Lockstatus: %i", lockStatus);
		if(lockStatus == 1)
		{
			WE_DEBUG_PRINT("Error: SIM locked");
			return;
		}
	}
    
    ret = AdrasteaI_ATPacketDomain_SetNetworkRegistrationResultCode(AdrasteaI_ATPacketDomain_Network_Registration_Result_Code_Enable_with_Location_Info);

    AdrasteaI_ExamplesPrint("Set Network Registration Result Code", ret);

    while (!((status.state == AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Roaming)||(status.state == AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Home_Network)))
    {
        switch(status.state){
			case AdrasteaI_ATPacketDomain_Network_Registration_State_Invalid: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Invalid\r\n");break;
			case AdrasteaI_ATPacketDomain_Network_Registration_State_Not_Registered_Not_Searching:WE_DEBUG_PRINT("Networkstatus: Registration_State_Not_Registered_Not_Searching\r\n");break;
    	   	case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Home_Network:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_Home_Network\r\n");break;
       		case AdrasteaI_ATPacketDomain_Network_Registration_State_Not_Registered_Searching:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Not_Registered_Searching\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registration_Denied:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registration_Denied\r\n");break;
     		case AdrasteaI_ATPacketDomain_Network_Registration_State_Unknown: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Unknown\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Roaming:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_Roaming\r\n");break;
     		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_SMS_Only_Home_Network: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_SMS_Only_Home_Network\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_SMS_Only_Roaming: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_SMS_Only_Roaming\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Attached_For_Emergency_Bearer_Services_Only:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Attached_For_Emergency_Bearer_Services_Only\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_For_CSFB_Not_Preferred_Home_Network:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_For_CSFB_Not_Preferred_Home_Network\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_For_CSFB_Not_Preferred_Roaming:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_For_CSFB_Not_Preferred_Roaming\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_NumberOfValues: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_NumberOfValues\r\n");break;
      		default:WE_DEBUG_PRINT("Networkstatus: unknown state\r\n");break;
		}
		
        
        WE_Delay(500);   
    }
    WE_DEBUG_PRINT("Networkstatus: Registered\r\n");
    
    /*Read the IMEI number, this will be used as the client ID*/
    AdrasteaI_ATDevice_IMEI_t imei;
    ret = AdrasteaI_ATDevice_RequestIMEI(&imei);
    AdrasteaI_ExamplesPrint("Request IMEI", ret);
    WE_DEBUG_PRINT("IMEI: %s\r\n", imei);

	/*Enable all MQTT events*/
    ret = AdrasteaI_ATMQTT_SetMQTTUnsolicitedNotificationEvents(AdrasteaI_ATMQTT_Event_All, 1);
    AdrasteaI_ExamplesPrint("MQTT Unsolicited Notification Events", ret);
	
	AdrasteaI_ATMQTT_Client_ID_t clientID="";
	
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
		
		switch(conResult.resultCode){
		        case AdrasteaI_ATMQTT_Event_Result_Code_Invalid: WE_DEBUG_PRINT("ATMQTT_Event_Result: AdrasteaI_ATMQTT_Event_Result_Code_Invalid\r\n");break;
        		case AdrasteaI_ATMQTT_Event_Result_Code_Success: WE_DEBUG_PRINT("ATMQTT_Event_Result: AdrasteaI_ATMQTT_Event_Result_Code_Success\r\n");break;
        		case AdrasteaI_ATMQTT_Event_Result_Code_Fail: WE_DEBUG_PRINT("ATMQTT_Event_Result: AdrasteaI_ATMQTT_Event_Result_Code_Fail\r\n");break;
        		case AdrasteaI_ATMQTT_Event_Result_Code_NumberOfValues: WE_DEBUG_PRINT("ATMQTT_Event_Result: AdrasteaI_ATMQTT_Event_Result_Code_NumberOfValues\r\n");break;
        		default: WE_DEBUG_PRINT("ATMQTT_Event_Result: unknown state\r\n");break;        		
		}
        WE_Delay(500);
    }
    
    
    /*Create the device manually*/
    memset(payload, 0, sizeof(payload));
		
	sprintf(payload, "100,%s,%s", imei, DEVICE_TYPE);
	
    ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
	AdrasteaI_ExamplesPrint("Device registration", ret);
	if(ret == false)
	{
		return;
	}
    
    /*Subscribe to predefined topics*/
    ret = AdrasteaI_ATMQTT_Subscribe(AdrasteaI_ATMQTT_Conn_ID_1, AdrasteaI_ATMQTT_QoS_At_Most_Once, topicOperations);
    AdrasteaI_ExamplesPrint("Subscribe", ret);
    
    ret = AdrasteaI_ATMQTT_Subscribe(AdrasteaI_ATMQTT_Conn_ID_1, AdrasteaI_ATMQTT_QoS_At_Most_Once, topicError);
    AdrasteaI_ExamplesPrint("Subscribe", ret);
    

    
    for(;;)
    {
		AdrasteaI_ATNetService_Signal_Quality_t sq;
		AdrasteaI_ATNetService_ReadSignalQuality(&sq);
		AdrasteaI_ExamplesPrint("Read Signal Quality", ret);
		if (ret)
		{
		    WE_DEBUG_PRINT("RSSI: %d, BER: %d\r\n", sq.rssi, sq.ber);
		}
		
		memset(payload, 0, sizeof(payload));
		
		sprintf(payload, "210,%i,%d", AdrasteaI_getRSSIindBm(sq.rssi),sq.ber);  // MQTT Format: https://cumulocity.com/docs/smartrest/mqtt-static-templates/#210
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
	
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);
		
		//Get  the pressure and temperature data and print the results to the UART and MQTT
    	float pressure, temperature;
    	xensiv_dps3xx_read(&pressure_sensor, &pressure, &temperature);

		AdrasteaI_ExamplesPrint("Read Pressure & Temperature", ret);
		if (ret)
		{
		        	WE_DEBUG_PRINT("Pressure   : %f\r\n", pressure); 
    				WE_DEBUG_PRINT("Temperature: %f\r\n\r\n", temperature);
		}
		
		memset(payload, 0, sizeof(payload));
		
		sprintf(payload, "211,%f", temperature); // MQTT Format: https://cumulocity.com/docs/smartrest/mqtt-static-templates/#211
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);                                                                                                                                                                  
		
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);
		
		memset(payload, 0, sizeof(payload));
		
		sprintf(payload, "200,Pressure,P,%f, pascal", pressure); // MQTT Format: https://cumulocity.com/docs/smartrest/mqtt-static-templates/#200
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
		
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);
		while (subResult.resultCode != AdrasteaI_ATMQTT_Event_Result_Code_Success)
		{
		    WE_Delay(10);
		}
        WE_DEBUG_PRINT("Wait for %d seconds\r\n", TELEMETRY_SEND_INTERVAL_MS/1000);
		WE_Delay(TELEMETRY_SEND_INTERVAL_MS);

	}


}

void ATMQTTGNSSExample()
{
    AdrasteaI_ATGNSS_Satellite_Count_t satelliteQueryCount = 0;
    bool ret;
    char progress='-';


    WE_DEBUG_PRINT("*** Start of Adrastea-I GNSS + MQTT example ***\r\n");

    if (!AdrasteaI_Init(&AdrasteaI_uart, &AdrasteaI_pins, &AdrasteaI_ATMQTT_EventCallback)) {
      WE_DEBUG_PRINT("Initialization error\r\n");
      return;
    }

  

     /* Initialize i2c for pressure sensor */
    ret = cyhal_i2c_init(&i2c, DPS_I2C_SDA, DPS_I2C_SCL, NULL);
    CY_ASSERT(ret == CY_RSLT_SUCCESS);
    ret = cyhal_i2c_configure(&i2c, &i2c_cfg);
    CY_ASSERT(ret == CY_RSLT_SUCCESS);

    /* Initialize pressure sensor */
    ret = xensiv_dps3xx_mtb_init_i2c(&pressure_sensor, &i2c, XENSIV_DPS3XX_I2C_ADDR_DEFAULT);
    CY_ASSERT(ret == CY_RSLT_SUCCESS);

    mtb_bmi270_data_t bmi270_data;
   /* Initialize the sensor and configure default settings */
    ret = mtb_bmi270_init_i2c(&imu, &i2c, MTB_BMI270_ADDRESS_DEFAULT);
    if (CY_RSLT_SUCCESS != ret)
    {
        CY_ASSERT(0);
    }
    ret = mtb_bmi270_config_default(&imu);
    if (CY_RSLT_SUCCESS != ret)
    {
        CY_ASSERT(0);
    }

    mtb_bmm350_data_t bmm350_data;
      /* Initialize the sensor */
    ret = mtb_bmm350_init_i2c(&magnetometer, &i2c, MTB_BMM350_ADDRESS_SEC);
    if (CY_RSLT_SUCCESS != ret)
    {
        CY_ASSERT(0);
    }


     /* Initialize the OLED display */
    ret = mtb_ssd1306_init_i2c(&i2c);
    
    CY_ASSERT(ret == CY_RSLT_SUCCESS);


    GUI_Init();
    GUI_DispString("GNSS + MQTT Demo\n");


/* Initialize PN532 with I2C interface */
    pn532_cyhal_i2c_t i2c_if_ctx = {
        .i2c = &i2c,
        .address = PN532_I2C_ADDRESS
    };
    PN532_I2C_Init(&pn532, &i2c_if_ctx);

    uint8_t version[4];
    if (PN532_GetFirmwareVersion(&pn532, version) == PN532_STATUS_OK) {
        printf("Found PN532 with firmware version: %d.%d\r\n", version[1], version[2]);
    } else {
        printf("PN532 not found.\r\n");
        while(1);
    }

    // Configure PN532 to communicate with MiFare cards
    PN532_SamConfiguration(&pn532);



    AdrasteaI_ATSIM_Lock_Status_t lockStatus;
    ret = AdrasteaI_ATSIM_ReadFacilityLock(AdrasteaI_ATSIM_Facility_SC, &lockStatus);
    AdrasteaI_ExamplesPrint("Read Facility Lock", ret);
    if(ret)
    {
		WE_DEBUG_PRINT("Lockstatus: %i", lockStatus);
		if(lockStatus == 1)
		{
			WE_DEBUG_PRINT("Error: SIM locked");
			return;
		}
	}
    



    ret = AdrasteaI_ATPacketDomain_SetNetworkRegistrationResultCode(AdrasteaI_ATPacketDomain_Network_Registration_Result_Code_Enable_with_Location_Info);
    AdrasteaI_ExamplesPrint("Set Network Registration Result Code", ret);

    while (!((status.state == AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Home_Network) || (status.state == AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Roaming)))
    {
        switch(status.state){
			case AdrasteaI_ATPacketDomain_Network_Registration_State_Invalid: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Invalid\r\n");break;
			case AdrasteaI_ATPacketDomain_Network_Registration_State_Not_Registered_Not_Searching:WE_DEBUG_PRINT("Networkstatus: Registration_State_Not_Registered_Not_Searching\r\n");break;
    	   	case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Home_Network:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_Home_Network\r\n");break;
       		case AdrasteaI_ATPacketDomain_Network_Registration_State_Not_Registered_Searching:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Not_Registered_Searching\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registration_Denied:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registration_Denied\r\n");break;
     		case AdrasteaI_ATPacketDomain_Network_Registration_State_Unknown: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Unknown\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_Roaming:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_Roaming\r\n");break;
     		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_SMS_Only_Home_Network: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_SMS_Only_Home_Network\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_SMS_Only_Roaming: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_SMS_Only_Roaming\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Attached_For_Emergency_Bearer_Services_Only:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Attached_For_Emergency_Bearer_Services_Only\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_For_CSFB_Not_Preferred_Home_Network:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_For_CSFB_Not_Preferred_Home_Network\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_Registered_For_CSFB_Not_Preferred_Roaming:WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_Registered_For_CSFB_Not_Preferred_Roaming\r\n");break;
      		case AdrasteaI_ATPacketDomain_Network_Registration_State_NumberOfValues: WE_DEBUG_PRINT("Networkstatus: Network_Registration_State_NumberOfValues\r\n");break;
      		default:WE_DEBUG_PRINT("Networkstatus: unknown state\r\n");break;
		}
		
        
        WE_Delay(500);   
    }

    GUI_DispString("LTE ");

     // Read the IMEI number, this will be used as the client ID
    AdrasteaI_ATDevice_IMEI_t imei;
    ret = AdrasteaI_ATDevice_RequestIMEI(&imei);
    AdrasteaI_ExamplesPrint("Request IMEI", ret);
    WE_DEBUG_PRINT("IMEI: %s\r\n", imei);

	// Enable all MQTT events
    ret = AdrasteaI_ATMQTT_SetMQTTUnsolicitedNotificationEvents(AdrasteaI_ATMQTT_Event_All, 1);
    AdrasteaI_ExamplesPrint("MQTT Unsolicited Notification Events", ret);
	
	AdrasteaI_ATMQTT_Client_ID_t clientID = "";
	
	strncpy(clientID, imei, 15);
	// Configure MQTT client
    ret = AdrasteaI_ATMQTT_ConfigureNodes(AdrasteaI_ATMQTT_Conn_ID_1,clientID, A1ServerAddress, userName, password);
    AdrasteaI_ExamplesPrint("Configure Nodes", ret);

    //Configure MQTT client
    ret = AdrasteaI_ATMQTT_ConfigureProtocol(AdrasteaI_ATMQTT_Conn_ID_1, 1200, 1);
    AdrasteaI_ExamplesPrint("Configure Protocol", ret);

    //Connect to MQTT broker
    ret = AdrasteaI_ATMQTT_Connect(AdrasteaI_ATMQTT_Conn_ID_1);
    AdrasteaI_ExamplesPrint("Connect", ret);

    while (conResult.resultCode != AdrasteaI_ATMQTT_Event_Result_Code_Success)
    {
		
		switch(conResult.resultCode){
		        case AdrasteaI_ATMQTT_Event_Result_Code_Invalid: WE_DEBUG_PRINT("ATMQTT_Event_Result: AdrasteaI_ATMQTT_Event_Result_Code_Invalid\r\n");break;
        		case AdrasteaI_ATMQTT_Event_Result_Code_Success: WE_DEBUG_PRINT("ATMQTT_Event_Result: AdrasteaI_ATMQTT_Event_Result_Code_Success\r\n");break;
        		case AdrasteaI_ATMQTT_Event_Result_Code_Fail: WE_DEBUG_PRINT("ATMQTT_Event_Result: AdrasteaI_ATMQTT_Event_Result_Code_Fail\r\n");break;
        		case AdrasteaI_ATMQTT_Event_Result_Code_NumberOfValues: WE_DEBUG_PRINT("ATMQTT_Event_Result: AdrasteaI_ATMQTT_Event_Result_Code_NumberOfValues\r\n");break;
        		default: WE_DEBUG_PRINT("ATMQTT_Event_Result: unknown state\r\n");break;        		
		}
        WE_Delay(500);
    }
    
    GUI_DispString("MQTT\n");

    //Create the device manually
    memset(payload, 0, sizeof(payload));
		
	sprintf(payload, "100,%s,%s", imei, DEVICE_TYPE);
	
    ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
	AdrasteaI_ExamplesPrint("Device registration", ret);
	if(ret == false)
	{
		return;
	}
    
    //Subscribe to predefined topics
    ret = AdrasteaI_ATMQTT_Subscribe(AdrasteaI_ATMQTT_Conn_ID_1, AdrasteaI_ATMQTT_QoS_At_Most_Once, topicOperations);
    AdrasteaI_ExamplesPrint("Subscribe", ret);
    
    ret = AdrasteaI_ATMQTT_Subscribe(AdrasteaI_ATMQTT_Conn_ID_1, AdrasteaI_ATMQTT_QoS_At_Most_Once, topicError);
    AdrasteaI_ExamplesPrint("Subscribe", ret);
    
        AdrasteaI_ATNetService_Signal_Quality_t sq;
		AdrasteaI_ATNetService_ReadSignalQuality(&sq);
		AdrasteaI_ExamplesPrint("Read Signal Quality", ret);
		if (ret)
		{
		    WE_DEBUG_PRINT("RSSI: %d, BER: %d\r\n", sq.rssi, sq.ber);
            sprintf(payload, "RSSI: %i BER: %i\n", AdrasteaI_getRSSIindBm(sq.rssi),sq.ber);
            GUI_DispStringAt(payload,0,2*8);
		}
		
		memset(payload, 0, sizeof(payload));
		
		sprintf(payload, "210,%i,%d", AdrasteaI_getRSSIindBm(sq.rssi),sq.ber);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
	
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);
    

    uint8_t uid[MIFARE_UID_MAX_LENGTH];
    int uid_len;
    char uid_str[MIFARE_UID_MAX_LENGTH * 3];
    uint64_t uid_num ;

    printf("Waiting for a card...\r\n");

    // Wait for a card to be present
    uid_len = PN532_ReadPassiveTarget(&pn532, uid, PN532_MIFARE_ISO14443A, 1000);

    if (uid_len > 0)
    {
        // Card found
        bytes_to_hex_string(uid, uid_len, uid_str);
        printf("Card found! UID: %s\r\n", uid_str);
        
        uid_num = bytes_to_uint64(uid, uid_len);
        printf("Card found! UID (as number): %llu\r\n", (unsigned long long)uid_num);


         memset(payload, 0, sizeof(payload));

		sprintf(payload, "200,RFID,ID, %llu, id",  (unsigned long long)uid_num);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
		
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

    }
    else if (uid_len == PN532_STATUS_ERROR)
    {
        printf("Error reading card.\r\n");
    }
    else
    {
        // No card found
    }

   

    // LTE Usage for GNSS
	ret = AdrasteaI_ATGNSS_DownloadCEPFile(AdrasteaI_ATGNSS_CEP_Number_of_Days_1Day);
    AdrasteaI_ExamplesPrint("Download CEP File", ret);

    AdrasteaI_ATGNSS_CEP_Status_t cepStatus;
    ret = AdrasteaI_ATGNSS_QueryCEPFileStatus(&cepStatus);
    AdrasteaI_ExamplesPrint("Query CEP Status", ret);
    if (ret)
    {
        WE_DEBUG_PRINT("Validity: %d, Remaining Days: %d, Hours: %d, Minutes:%d\r\n", cepStatus.validity, cepStatus.remDays, cepStatus.remHours, cepStatus.remMinutes);
    }
	

 for(;;)
    {	

        // Disconnect the MQTT Connection
        AdrasteaI_ATMQTT_Disconnect(AdrasteaI_ATMQTT_Conn_ID_1);
     
        ret = AdrasteaI_ATDevice_SetPhoneFunctionality(AdrasteaI_ATDevice_Phone_Functionality_Min, AdrasteaI_ATDevice_Phone_Functionality_Reset_Do_Not_Reset);
        AdrasteaI_ExamplesPrint("Set Phone Functionality", ret);

        ret = AdrasteaI_ATGNSS_StartGNSS(AdrasteaI_ATGNSS_Start_Mode_Hot);
        AdrasteaI_ExamplesPrint("Start GNSS", ret);
        GUI_DispStringAt("GNSS        \n",0,1*8);

        AdrasteaI_ATGNSS_Satellite_Systems_t satSystems = {.systems = {.GPS = AdrasteaI_ATGNSS_Runtime_Mode_State_Set, .GLONASS = AdrasteaI_ATGNSS_Runtime_Mode_State_Set}};
        ret = AdrasteaI_ATGNSS_SetSatelliteSystems(satSystems);
        AdrasteaI_ExamplesPrint("Set Satellite Systems", ret);

        AdrasteaI_ATGNSS_Fix_t fix;
        fix.fixType= AdrasteaI_ATGNSS_Fix_Type_Invalid;
        
        satelliteQueryCount = 0;

        while (fix.fixType== AdrasteaI_ATGNSS_Fix_Type_No_Fix || fix.fixType== AdrasteaI_ATGNSS_Fix_Type_Invalid||fix.accuracy>10 ){

            ret = AdrasteaI_ATGNSS_QueryGNSSSatellites(&satelliteQueryCount);
            AdrasteaI_ExamplesPrint("Query GNSS Satellites", ret);
            WE_DEBUG_PRINT("Satellites Count: %d\r\n", satelliteQueryCount);

            ret = AdrasteaI_ATGNSS_QueryGNSSFix(AdrasteaI_ATGNSS_Fix_Relavancy_Current, &fix);
            AdrasteaI_ExamplesPrint("Query GNSS Fix", ret);

            if (ret && fix.fixType != AdrasteaI_ATGNSS_Fix_Type_No_Fix)
            {
                WE_DEBUG_PRINT("Fix Latitude: %f, Longitude: %f, Altitude: %.1f, Accuracy: %.1f\r\n", fix.latitude, fix.longitude, fix.altitude, fix.accuracy);
                
                sprintf(payload, "Fix:  %.1f   \n", fix.accuracy);
                GUI_DispStringAt(payload,0,5*8);

                sprintf(payload, "Time: %02d:%02d:%02d\n", fix.time.Hours,fix.time.Minutes,fix.time.Seconds);
                GUI_DispStringAt(payload,0,6*8);
            }
            else
            {
                WE_DEBUG_PRINT("No Fix. Satellites Count: %d\r\n", satelliteQueryCount);
                
                sprintf(payload, "Sat Count: %d  %c\n", satelliteQueryCount,progress);
                GUI_DispStringAt(payload,0,5*8);

                if(progress=='-'){
                    progress='+';
                }else{
                     progress='-';
                }

                sprintf(payload, "Time: %02d:%02d:%02d\n", fix.time.Hours,fix.time.Minutes,fix.time.Seconds);
                GUI_DispStringAt(payload,0,6*8);
                WE_Delay(1000); 

            }
        }

        ret = AdrasteaI_ATDevice_SetPhoneFunctionality(AdrasteaI_ATDevice_Phone_Functionality_Full, AdrasteaI_ATDevice_Phone_Functionality_Reset_Do_Not_Reset);
        AdrasteaI_ExamplesPrint("Set Phone Functionality", ret);
        
        // Stop GNSS
        AdrasteaI_ATGNSS_StopGNSS();	
        WE_Delay(2000); // GNSS Stop needs some delay before directly connecting to the MQTT 
        GUI_DispStringAt("LTE MQTT\n",0,1*8);
        
        
	
	
	
	
	
	
	
	
        
        //Connect to MQTT broker
        ret = AdrasteaI_ATMQTT_Connect(AdrasteaI_ATMQTT_Conn_ID_1); 
        AdrasteaI_ExamplesPrint("Connect", ret);

        while (conResult.resultCode != AdrasteaI_ATMQTT_Event_Result_Code_Success)
        {
            WE_Delay(2000);
        }    
        
   
		memset(payload, 0, sizeof(payload));
	
		sprintf(payload, "401,%f,%f,%f,%f", fix.latitude, fix.longitude, fix.altitude, fix.accuracy);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);                                                                                                                                                                  
		
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

         AdrasteaI_ATNetService_Signal_Quality_t sq;
		AdrasteaI_ATNetService_ReadSignalQuality(&sq);
		AdrasteaI_ExamplesPrint("Read Signal Quality", ret);
		if (ret)
		{
		    WE_DEBUG_PRINT("RSSI: %d, BER: %d\r\n", sq.rssi, sq.ber);
            sprintf(payload, "RSSI: %i BER: %i\n", AdrasteaI_getRSSIindBm(sq.rssi),sq.ber);
            GUI_DispString(payload);
            GUI_DispStringAtCEOL(payload,0,2*8);
		}
		
		memset(payload, 0, sizeof(payload));
		
		sprintf(payload, "210,%i,%d", AdrasteaI_getRSSIindBm(sq.rssi),sq.ber);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
	
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);
		
		//Get  the pressure and temperature data and print the results to the UART
       	float pressure, temperature;
       	xensiv_dps3xx_read(&pressure_sensor, &pressure, &temperature);

		AdrasteaI_ExamplesPrint("Read Pressure & Temperature", ret);
		if (ret)
		{
		        WE_DEBUG_PRINT("Pressure   : %d\r\n", (int)pressure); 
   				WE_DEBUG_PRINT("Temperature: %d\r\n\r\n", (int)temperature);
		}
		
		memset(payload, 0, sizeof(payload));
		
		sprintf(payload, "211,%d", (int)temperature);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);                                                                                                                                                                  
		
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);
		
		memset(payload, 0, sizeof(payload));
		
		sprintf(payload, "200,Pressure,P,%d, pascal", (int) pressure);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
		
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        memset(payload, 0, sizeof(payload));

		sprintf(payload, "200,Satellites,count,%d, count", satelliteQueryCount);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
		
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);


        memset(payload, 0, sizeof(payload));

		sprintf(payload, "200,Accuracy,m,%f, m", fix.accuracy);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
		
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        /* Get the accelerometer and gyroscope data and print the results to the UART */
        ret = mtb_bmi270_read(&imu, &bmi270_data);
        printf("\nAcc:  X:%6d Y:%6d Z:%6d\r\n", bmi270_data.sensor_data.acc.x, bmi270_data.sensor_data.acc.y,
               bmi270_data.sensor_data.acc.z);
        printf("Gyro: X:%6d Y:%6d Z:%6d\r\n", bmi270_data.sensor_data.gyr.x, bmi270_data.sensor_data.gyr.y,
               bmi270_data.sensor_data.gyr.z);

        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Gyroscope,X,%d, rad/s", bmi270_data.sensor_data.gyr.x);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Gyroscope,Y,%d, rad/s", bmi270_data.sensor_data.gyr.y);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
		ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Gyroscope,Z,%d, rad/s", bmi270_data.sensor_data.gyr.z);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
        ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Accelerometer,X,%d, m/s^2", bmi270_data.sensor_data.acc.x);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
        ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Accelerometer,Y,%d, m/s^2", bmi270_data.sensor_data.acc.y);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
        ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Accelerometer,Z,%d, m/s^2", bmi270_data.sensor_data.acc.z);
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
        ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);


             /* Get the magnetometer data and temperature, and print the results to the UART */
        ret = mtb_bmm350_read(&magnetometer, &bmm350_data);
        printf("\nMag: %f, %f, %f \r\n", bmm350_data.sensor_data.x, bmm350_data.sensor_data.y,
               bmm350_data.sensor_data.z);
        printf("Temp: %f\r\n\n", bmm350_data.sensor_data.temperature);


        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Magnetometer,X,%d, uT", (int)(bmm350_data.sensor_data.x));
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
        ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Magnetometer,Y,%d, uT", (int)(bmm350_data.sensor_data.y));
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
        ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

        memset(payload, 0, sizeof(payload));
		sprintf(payload, "200,Magnetometer,Z,%d, uT", (int)(bmm350_data.sensor_data.z));
		WE_DEBUG_PRINT("Publish: %s\r\n", payload);
        ret = AdrasteaI_ATMQTT_Publish(AdrasteaI_ATMQTT_Conn_ID_1, 0, 0, pubTopic, payload, strlen(payload));
		AdrasteaI_ExamplesPrint("Publish", ret);

		while (subResult.resultCode != AdrasteaI_ATMQTT_Event_Result_Code_Success)
		{
		    WE_Delay(10);
		}
		WE_Delay(5000);
	}
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

void AdrasteaI_ATMQTT_EventCallback(char* eventText)
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
        default:
            break;
    }
}



/**
 * @brief Converts a byte array to a hex string.
 *
 * @param data The byte array to convert.
 * @param len The length of the byte array.
 * @param out_str The output buffer to store the hex string.
 *                The buffer must be large enough (at least len * 3).
 */
void bytes_to_hex_string(const uint8_t *data, size_t len, char *out_str)
{
    if (len == 0)
    {
        *out_str = '\0';
        return;
    }

    for (size_t i = 0; i < len; i++)
    {
        // Each byte takes 2 hex chars + 1 space.
        sprintf(out_str + (i * 3), "%02X ", data[i]);
    }
    // Remove the trailing space and add null terminator
    out_str[len * 3 - 1] = '\0';
}


/**
 * @brief Converts a byte array to a hex string without spaces.
 *
 * @param data The byte array to convert.
 * @param len The length of the byte array.
 * @param out_str The output buffer to store the hex string.
 *                The buffer must be large enough (at least len * 2 + 1).
 */
void bytes_to_hex_string_no_space(const uint8_t *data, size_t len, char *out_str)
{
    if (len == 0)
    {
        *out_str = '\0';
        return;
    }

    for (size_t i = 0; i < len; i++)
    {
        // Each byte takes 2 hex chars.
        sprintf(out_str + (i * 2), "%02X", data[i]);
    }
}

/**
 * @brief Converts a byte array to a uint64_t integer.
 * Assumes big-endian byte order in the array.
 *
 * @param data The byte array to convert.
 * @param len The length of the byte array (max 8).
 * @return The converted 64-bit unsigned integer. Returns 0 if len > 8 or data is NULL.
 */
uint64_t bytes_to_uint64(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || len > 8) {
        return 0;
    }
    uint64_t result = 0;
    for (size_t i = 0; i < len; i++) {
        result = (result << 8) | data[i];
    }
    return result;
}