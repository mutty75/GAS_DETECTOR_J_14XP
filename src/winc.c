/*
 * winc.c
 *
 * Created: 14/07/2016 09.21.41
 *  Author: ROBERTOC
 */ 

#include <asf.h>
#include "portmacro.h"
#include "eepromemul.h"
#include "winc.h"
#include "RTC.h"
#include "taskslist.h"

#include "eepromemul.h"

#define WINC_TASK_PRIORITY      (tskIDLE_PRIORITY + 1)


#define AP_MODE

#ifdef AP_MODE

/** Wi-Fi Settings */
//#define MAIN_WLAN_SSID                    "OpenSpace" /**< Destination SSID */
#define MAIN_WLAN_SSID                    "NocoSystem" /**< Destination SSID */
#define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WEP /**< Security manner */
//#define MAIN_WLAN_PSK                     "OpenSpaceBtsr$$" /**< Password for Destination SSID */
#define MAIN_WLAN_PSK                     "NocoSystem123" /**< Password for Destination SSID */
#define MAIN_WIFI_M2M_PRODUCT_NAME        "BTSRWifi"
#define MAIN_WIFI_M2M_SERVER_IP           0xFFFFFFFF /* 255.255.255.255 */
#define MAIN_WIFI_M2M_SERVER_PORT         (1114)
#define MAIN_WIFI_M2M_HTTP_PORT           (80)

#define MAIN_WLAN_DEVICE_NAME			  "WIFIDIRECT_TEST"
#define MAIN_WLAN_CHANNEL (0) /* < Channel number */

#else

/** Wi-Fi Settings */
//#define MAIN_WLAN_SSID                    "OpenSpace" /**< Destination SSID */
#define MAIN_WLAN_SSID                    "OpenSpace" /**< Destination SSID */
#define MAIN_WLAN_AUTH                    M2M_WIFI_SEC_WPA_PSK /**< Security manner */
//#define MAIN_WLAN_PSK                     "OpenSpaceBtsr$$" /**< Password for Destination SSID */
#define MAIN_WLAN_PSK                     "OpenSpaceBtsr$$" /**< Password for Destination SSID */
#define MAIN_WIFI_M2M_PRODUCT_NAME        "BTSRWifi"
#define MAIN_WIFI_M2M_SERVER_IP           0xFFFFFFFF /* 255.255.255.255 */
#define MAIN_WIFI_M2M_SERVER_PORT         (1114)
#define MAIN_WIFI_M2M_HTTP_PORT           (80)

#endif

struct sockaddr_in addr;
struct sockaddr_in UDPstrRemoteAddr;

typedef enum {
	WAIT_CONNECT,
	CONNECTED,
	APOINT_MODE,
	MEMORY_ERROR,
	FS_MEMORY_ERROR,
	BUSY,
} xStatusEnum;

xStatusEnum xStatus;


/** Socket for TCP communication */
SOCKET tcp_server_socket = -1;
SOCKET tcp_client_socket = -1;

/** Socket for HTTP communication */
SOCKET http_server_socket = -1;
SOCKET http_client_socket = -1;

/** Wi-Fi connection state */
static uint8_t wifi_connected;

static int16_t sMTUSize;
//static bool xTxResult;
static bool xTCPTxResult;
//static bool xUDPTxResult;

static void winc_task(void *);
static void cmd_task(void *);

//Controllare BOOT
//Error EEPROM 
//Errori nella call back

////////////////////////////////////////////////////////////////////////////////////////////////////

void vLoadConfiguration()
{

 //if (eeprom_emulator_read_buffer(EEPROM_CONFIGFILE_OFFSET,(uint8_t *)&xConfigData,sizeof(xConfigData))==STATUS_OK) return;
 //
 //memset(&xConfigData,0x00,sizeof(xConfigData));
 //strcpy(xConfigData.cSSID,"OpenSpace");
 //strcpy(xConfigData.cPassword,"OpenSpaceBtsr$$");
 //strcpy(xConfigData.cHostName,"BtsrWiFi");
 //xConfigData.u32StaticIP=0xc0a801c0;
 //xConfigData.u32SubnetMask=0xFFFFFE00;
 //xConfigData.u32Gateway=0xc0a801FE;
 //xConfigData.xSecurityType=M2M_WIFI_SEC_WPA_PSK;
 //xConfigData.xDHCP=false;
 //xConfigData.xBaudrate=BR115200;
 //xConfigData.xConnectionMode=TCP;
 //
 //eeprom_emulator_write_buffer(EEPROM_CONFIGFILE_OFFSET,(uint8_t*)&xConfigData,sizeof(xConfigData));
 //eeprom_emulator_commit_page_buffer();

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void vSaveConfiguration()
{
 //eeprom_emulator_read_buffer(EEPROM_CONFIGFILE_OFFSET,(uint8_t *)HTTPBuffer,sizeof(xConfigData));
 //if (memcmp(HTTPBuffer,(uint8_t *)&xConfigData,sizeof(xConfigData)))
 //{
  //eeprom_emulator_write_buffer(EEPROM_CONFIGFILE_OFFSET,(uint8_t *)&xConfigData,sizeof(xConfigData));
  //eeprom_emulator_commit_page_buffer();	
 //}
 //xSwitchModeFlag=true;
}



////////////////////////////////////////////////////////////////////////////////////////////////////

static void vSendTCPReply(char *pcRxString, uint16_t usLen)
{
	 portDISABLE_INTERRUPTS();
	 xTCPTxResult=false;
	 if (send(tcp_client_socket,pcRxString,usLen,0)==SOCK_ERR_NO_ERROR)
	 {
	  vTaskDelay(TASK_DELAY_MS(WIFI_TIMEOUT_SEND));
	  if (!xTCPTxResult) return;
	 }
	 else {portENABLE_INTERRUPTS();vTaskDelay(TASK_DELAY_MS(10));}
}



///////////////////////////////////////////////////////////////////////////////////////////////////

int16_t sGetSize()
{
	return sMTUSize;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

//void vCloseHTTP()
//{
 //close(http_client_socket)
//}


////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) 
	{
	/* Socket bind */
	case SOCKET_MSG_BIND:
	{
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
		if (sock==tcp_server_socket)
	    {
		 if (pstrBind && pstrBind->status == 0) listen(tcp_server_socket, 0);
	     else 
		 {
		  close(tcp_server_socket);
		  tcp_server_socket = -1;
		 }
		}
	}
	break;

	/* Socket listen */
	case SOCKET_MSG_LISTEN:
	{
		tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg *)pvMsg;
		if (sock==tcp_server_socket)
	    {
		 if (pstrListen && pstrListen->status == 0) break;
		 close(tcp_server_socket);
		 tcp_server_socket = -1;
		}	
	}
	break;

	/* Connect accept */
	case SOCKET_MSG_ACCEPT:
	{
		tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
		if (sock==tcp_server_socket)
	    {
		 if (pstrAccept) 
		 {
		  if (tcp_client_socket!=-1) close(tcp_client_socket);
		  accept(tcp_server_socket, NULL, NULL);
		  tcp_client_socket = pstrAccept->sock;
		  recv(tcp_client_socket, TCPBuffer, sizeof(TCPBuffer), 0);
		 } 
		 else close(pstrAccept->sock);
		}
	}
	break;
	
	/* Message send */
	case SOCKET_MSG_SEND:
	
	{
     int16_t sLen=*(int16_t *)pvMsg;

	 if (sock==tcp_client_socket) 
	 {
	  if (sLen>0)
	  {
	   xTCPTxResult=true;
	   xTaskAbortDelay(xCmdTaskH);
	  }
	  else
	  {
	   xTCPTxResult=false;
	   xTaskAbortDelay(xCmdTaskH);		  	  
	  } 
	 }
	}

	break;
	
	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
		if (sock==tcp_client_socket) 
		{
		 if (pstrRecv && pstrRecv->s16BufferSize > 0) 
		 {
		  BaseType_t xHigherPriorityTaskWoken=pdFALSE;
		  //send(sock,TCPBuffer,pstrRecv->s16BufferSize,0);
		  //port_pin_set_output_level(LED_0_PIN,true);
		   
		  //xNotify xNotifyValue;
		  //xNotifyValue.xTXFlag=1;
		  //xNotifyValue.xTXLen=pstrRecv->s16BufferSize;
		  xTaskNotifyFromISR(xCmdTaskH,pstrRecv->s16BufferSize,eSetValueWithOverwrite,&xHigherPriorityTaskWoken); 
		  
		 } 
		 else 
		 {
		  switch (pstrRecv->s16BufferSize)
		  {
			  case SOCK_ERR_CONN_ABORTED: close(tcp_client_socket);tcp_client_socket=-1; break;
			  default:  recv(tcp_client_socket, TCPBuffer, sizeof(TCPBuffer), 0); break;
		  }
		 }
		}
	}
	break;

	default: break;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{

switch (u8MsgType)
{
case M2M_WIFI_RESP_CON_STATE_CHANGED:
{
tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
m2m_wifi_request_dhcp_client();
} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
//printf("Wi-Fi disconnected\r\n");
}
break;
}
case M2M_WIFI_REQ_DHCP_CONF:
{
uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
//printf("Wi-Fi connected\r\n");
//printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
//pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
break;
}
default:
{
break;
}
}
}
//
//static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
//{
	//switch (u8MsgType) {
	//case M2M_WIFI_RESP_CON_STATE_CHANGED:
	//{
		//tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		//if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) 
		//{
			//wifi_connected = 1;
			////if (xStatus==WAIT_CONNECT) xStatus=CONNECTED;
		//}
		//if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) 
		//{
	      //wifi_connected = 0;
		  //if (xStatus==APOINT_MODE)
		  //{
			  //socketDeinit();
			  //tcp_server_socket = -1;
			  //tcp_client_socket = -1;
//
			  //http_server_socket = -1;
			  //http_client_socket = -1;
			  //
			  //socketInit();
			  //registerSocketCallback(socket_cb, NULL);
			  //break;
		  //}
          //tstrM2MIPConfig ip_client; 
          //ip_client.u32StaticIP = _htonl(xConfigData.u32StaticIP); //corresponds to 192.168.0.192
          //ip_client.u32SubnetMask = _htonl(xConfigData.u32SubnetMask); //corresponds to 255.255.254.0
          //ip_client.u32Gateway = _htonl(xConfigData.u32Gateway);  //corresponds to 192.168.1.254
          //m2m_wifi_set_static_ip(&ip_client);		  
		  ////if (xStatus==CONNECTED) xStatus=WAIT_CONNECT;
		  ////m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		  //m2m_wifi_connect((char *)xConfigData.cSSID, sizeof(xConfigData.cSSID), xConfigData.xSecurityType, (char *)xConfigData.cPassword, M2M_WIFI_CH_ALL);
		//}
	//}
	//break;
//
	//case M2M_WIFI_REQ_DHCP_CONF:
	//{
		////uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		//wifi_connected = 1;
		//////printf("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
				////pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	//}
	//break;
//
	//default:
		//break;
	//}
//}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vSwitchToAPMode(void)
{
  tstrWifiInitParam param;
  
  socketDeinit();
  m2m_wifi_deinit(NULL);
  //nm_bsp_init();
  param.pfAppWifiCb = wifi_cb;
  int8_t ret = m2m_wifi_init(&param);
  if (M2M_SUCCESS != ret) while (1) vTaskDelay(TASK_DELAY_MS(1000));
	
  tstrM2MAPConfig strM2MAPConfig;	
  /* Initialize AP mode parameters structure with SSID, channel and OPEN security type. */
  memset(&strM2MAPConfig, 0x00, sizeof(tstrM2MAPConfig));
  strcpy((char *)&strM2MAPConfig.au8SSID, MAIN_WLAN_SSID);
  strM2MAPConfig.u8SsidHide = SSID_MODE_VISIBLE;
  strM2MAPConfig.u8ListenChannel = 1;
  strM2MAPConfig.u8SecType = M2M_WIFI_SEC_OPEN;

  strM2MAPConfig.au8DHCPServerIP[0] = 192;
  strM2MAPConfig.au8DHCPServerIP[1] = 168;
  strM2MAPConfig.au8DHCPServerIP[2] = 1;
  strM2MAPConfig.au8DHCPServerIP[3] = 1;
  
  /** Socket for TCP communication */
  tcp_server_socket = -1;
  tcp_client_socket = -1;

/** Socket for HTTP communication */
  http_server_socket = -1;
  http_client_socket = -1;
  
  wifi_connected = 0;

	/* Bring up AP mode with parameters structure. */
 ret = m2m_wifi_enable_ap(&strM2MAPConfig);
 if (M2M_SUCCESS != ret) while (1) vTaskDelay(TASK_DELAY_MS(1000));

 socketInit();
 registerSocketCallback(socket_cb, NULL); 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vSwitchToClientMode(void)
{
	tstrWifiInitParam param;
	
	socketDeinit();
	m2m_wifi_deinit(NULL);
	//nm_bsp_init();
	param.pfAppWifiCb = wifi_cb;
	int8_t ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) while (1) vTaskDelay(TASK_DELAY_MS(1000));
	
    /** Socket for TCP communication */
    tcp_server_socket = -1;
    tcp_client_socket = -1;

    /** Socket for HTTP communication */
    http_server_socket = -1;
    http_client_socket = -1;
  
    wifi_connected = 0;	
	
    socketInit();
    registerSocketCallback(socket_cb, NULL);

     /* Connect to router. */
	 if (!xConfigData.xDHCP)
	 {
      m2m_wifi_enable_dhcp(0);
      tstrM2MIPConfig ip_client; 
      ip_client.u32StaticIP = _htonl(xConfigData.u32StaticIP); //corresponds to 192.168.0.192
      ip_client.u32SubnetMask = _htonl(xConfigData.u32SubnetMask); //corresponds to 255.255.254.0
      ip_client.u32Gateway = _htonl(xConfigData.u32Gateway);  //corresponds to 192.168.1.254
      m2m_wifi_set_static_ip(&ip_client);
	 }
	 else m2m_wifi_enable_dhcp(1);
 
    //portDISABLE_INTERRUPTS();
    m2m_wifi_connect((char *)xConfigData.cSSID, sizeof(xConfigData.cSSID), 
                    xConfigData.xSecurityType, (char *)xConfigData.cPassword, M2M_WIFI_CH_ALL);
				
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vCreateWincTask(void)
{
	
	xTaskCreate(winc_task,(const char *)"Winc",2*configMINIMAL_STACK_SIZE,NULL,WINC_TASK_PRIORITY,&xWincTaskH);
    xTaskCreate(cmd_task,(const char *)"Comm",2*configMINIMAL_STACK_SIZE,NULL,WINC_TASK_PRIORITY,&xCmdTaskH);
	

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void winc_task(void *params)
{
 tstrWifiInitParam param;
 uint32_t ulNotifiedValue;

 //xIntSemaphore = xSemaphoreCreateCounting( 20, 0 );
 xAcceptQueue = xQueueCreate( 7, sizeof( SOCKET ) );
 //xStatus = WAIT_CONNECT;
 
 /* Initialize Wi-Fi parameters structure. */
 memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

 /* Initialize Wi-Fi driver with data and status callbacks. */
 param.pfAppWifiCb = wifi_cb;
 int8_t ret = m2m_wifi_init(&param);
 if (M2M_SUCCESS != ret) while (1) vTaskDelay(TASK_DELAY_MS(1000));
 
  //memset(&xConfigData,0x00,sizeof(xConfigData));
  //strcpy(xConfigData.cSSID,"Roby");
  //strcpy(xConfigData.cPassword,"pipppero");
  //strcpy(xConfigData.cHostName,"BtsrWiFi");
  //xConfigData.u32StaticIP=0xc0a800c0;
  //xConfigData.u32SubnetMask=0xFFFFFF00;
  //xConfigData.u32Gateway=0xc0a80005;
  //xConfigData.xSecurityType=M2M_WIFI_SEC_WPA_PSK;
  //xConfigData.xDHCP=false;
  //xConfigData.xBaudrate=BR115200;
  //xConfigData.xConnectionMode=TCP;
  
    memset(&xConfigData,0x00,sizeof(xConfigData));
    strcpy(xConfigData.cSSID,"OpenSpace");
    strcpy(xConfigData.cPassword,"OpenSpaceBtsr$$");
    strcpy(xConfigData.cHostName,"BtsrWiFi");
    xConfigData.u32StaticIP=0xc0a801c0;
    xConfigData.u32SubnetMask=0xFFFFFE00;
    xConfigData.u32Gateway=0xc0a801FE;
    xConfigData.xSecurityType=M2M_WIFI_SEC_WPA_PSK;
    xConfigData.xDHCP=false;
    xConfigData.xBaudrate=BR115200;
    xConfigData.xConnectionMode=TCP;
 
 //vTaskSuspend(NULL);
 vTaskDelay(TASK_DELAY_MS(4000));
 
 ret = m2m_wifi_set_device_name((uint8_t *)MAIN_WLAN_DEVICE_NAME,strlen(MAIN_WLAN_DEVICE_NAME)); if (M2M_SUCCESS != ret) while (1) vTaskDelay(TASK_DELAY_MS(1000));
 
 
 ret = m2m_wifi_p2p(0);
 if (M2M_SUCCESS != ret) while (1) vTaskDelay(TASK_DELAY_MS(1000));
  //#ifdef  DEBUG
  //
  //vSwitchToAPMode();
  xStatus=APOINT_MODE;
  
  //#else
 
 ///* Initialize socket module */
 //socketInit();
 //registerSocketCallback(socket_cb, NULL);
//
 ///* Connect to router. */
 //if (!xConfigData.xDHCP)
 //{
  //m2m_wifi_enable_dhcp(0);
  //tstrM2MIPConfig ip_client; 
  //ip_client.u32StaticIP = _htonl(xConfigData.u32StaticIP); //corresponds to 192.168.0.192
  //ip_client.u32SubnetMask = _htonl(xConfigData.u32SubnetMask); //corresponds to 255.255.254.0
  //ip_client.u32Gateway = _htonl(xConfigData.u32Gateway);  //corresponds to 192.168.1.254
  //m2m_wifi_set_static_ip(&ip_client);	
 //}
 //else m2m_wifi_enable_dhcp(1);
 //
 ////portDISABLE_INTERRUPTS();
 //m2m_wifi_connect((char *)xConfigData.cSSID, sizeof(xConfigData.cSSID), 
                 //xConfigData.xSecurityType, (char *)xConfigData.cPassword, M2M_WIFI_CH_ALL);
				 
//#endif

 for (;;)
 {
  
  //xSemaphoreTake(xIntSemaphore,portMAX_DELAY);
  ulNotifiedValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
  
  while (ulNotifiedValue>0)
  {//vTaskDelay(2);
   ulNotifiedValue--;
   m2m_wifi_handle_events(NULL);
  }
  
   if (wifi_connected == M2M_WIFI_CONNECTED) 
   {
    if (tcp_server_socket < 0)
    {
     /* Open TCP server socket */
	 //if (xConfigData.xConnectionMode==TCP)
	 //{
      if ((tcp_server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) continue;
    
      /* Bind service*/
	  /* Initialize socket address structure. */
      addr.sin_family = AF_INET;
      addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
      addr.sin_addr.s_addr = 0;
	  bind(tcp_server_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
	 //}
	 //else
	 //{
      //if ((tcp_server_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) continue;
    //
      ///* Bind service*/
	  ///* Initialize socket address structure. */
      //addr.sin_family = AF_INET;
      //addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
      //addr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP);
	  //bind(tcp_server_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));		 
	 //}
   }
//#ifndef DEBUG
    //if ((http_server_socket < 0) && (xStatus==APOINT_MODE))
//#else
    //if (http_server_socket < 0)
//#endif
    //{
     ///* Open HTTP server socket */
     //if ((http_server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) continue;
    //
     ///* Bind service*/
	  ///* Initialize socket address structure. */
     //addr.sin_family = AF_INET;
     //addr.sin_port = _htons(MAIN_WIFI_M2M_HTTP_PORT);
     //addr.sin_addr.s_addr = 0;
	 //bind(http_server_socket, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    //}
   }
  //}
 }
	
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void cmd_task(void *params)
{//uint32_t ulNotifiedValue;
	
 for (;;)
 {
  
  //ulNotifiedValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
  ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
  
  switch (TCPBuffer[0])
  {
	case 0x01: //Recv Program
	{
     UINT uWrt;
	 memcpy(&xProgramList[TCPBuffer[1]-0x01],&TCPBuffer[2],sizeof(xProgram));
	 f_open(&file_object,"0:/PrgList.bin",FA_WRITE);
	 f_write(&file_object,xProgramList,sizeof(xProgramList),&uWrt);
	 f_close(&file_object);
	
	 TCPBuffer[0]=0x01;
	 TCPBuffer[1]=0x00;
	 TCPBuffer[2]=0x00;
	 vSendTCPReply(TCPBuffer,0x03);
	}
	break; 
	
	case 0x02:
	{
	 UINT uWrt;
	 xSetup xTempSetup;
	 time_t xSaveTime;
	 	 
	 memcpy (&xTempSetup,&TCPBuffer[1],sizeof(xTempSetup));
	 xSaveTime=xTempSetup.xDateTime;
	 xTempSetup.xDateTime=0x00;
	 
	 if (memcmp(&xMachineSetup,&xTempSetup,sizeof(xMachineSetup)))
	 {
	  memcpy(&xMachineSetup,&xTempSetup,sizeof(xMachineSetup));
	  //vSaveSetup();
	  f_open(&file_object,"0:/Setup.bin",FA_WRITE);
	  f_write(&file_object,&xMachineSetup,sizeof(xMachineSetup),&uWrt);
	  f_close(&file_object);		 
	 }

	struct tm* timeinf=localtime (&xSaveTime);
	struct rtc_calendar_time time;
	xRTCStruct xRTCValue;
	
	time.year   = timeinf->tm_year+1900;
	time.month  = timeinf->tm_mon+1;
	time.day    = timeinf->tm_mday;
	time.hour   = timeinf->tm_hour;
	time.minute = timeinf->tm_min;
	time.second = timeinf->tm_sec;
	rtc_calendar_set_time(&rtc_instance, &time);
	
	xRTCValue.xSec=time.second % 10;
	xRTCValue.xSec10=time.second / 10;
	xRTCValue.xStart=0;
	xRTCValue.xMin=time.minute % 10;
	xRTCValue.xMin10=time.minute / 10;
    xRTCValue.xHour=time.hour % 10;
	xRTCValue.xHour10=time.hour / 10;
	xRTCValue.x1224=0;
	xRTCValue.xCalSign=0;
    xRTCValue.xDay=1;
	xRTCValue.xBatEN=1;
	xRTCValue.xVBat=0;
	xRTCValue.xOSCon=0;			
	xRTCValue.xDate=time.day % 10;
	xRTCValue.xDate10=time.day / 10;	
	xRTCValue.xMonth=time.month % 10;
	xRTCValue.xMonth10=time.month / 10;
    xRTCValue.xLeap=0;
	xRTCValue.xYear=(time.year-2000) % 10;
	xRTCValue.xYear10=(time.year-2000) / 10;
	RTC_WriteRegisters(&xRTCValue);
	RTC_WriteRegister(0x00,0x00);
	RTC_WriteRegister(0x01,(1<<7)+(xRTCValue.xSec10<<4)+xRTCValue.xSec);	
	
	f_open(&file_object,"0:/Labels.bin",FA_READ);
    f_lseek(&file_object,(uint32_t)xMachineSetup.ucLanguage*sizeof(LCDlabels));
    f_read(&file_object,&LCDlabels,sizeof(LCDlabels),&uWrt);
    f_close(&file_object);	
	
	TCPBuffer[0]=0x02;
	TCPBuffer[1]=0x00;
	TCPBuffer[2]=0x00;
	vSendTCPReply(TCPBuffer,0x03);	  
	}
	
	break;
	
	case 0x03:
	{
	TCPBuffer[0]=0x03;
	TCPBuffer[1]=(uint8_t)xMachineState;
	TCPBuffer[2]=(uint8_t)xAlarm;
	memcpy(&TCPBuffer[3],&ulProgramRunTime,sizeof(ulProgramRunTime));
	memcpy(&TCPBuffer[7],&ulProgramTimeToEnd,sizeof(ulProgramRunTime));
	memcpy(&TCPBuffer[11],&ulMachineWorkTime,sizeof(ulProgramRunTime));
	TCPBuffer[15]=1;
	TCPBuffer[16]=0;
	TCPBuffer[17]=0;
	TCPBuffer[18]=0;
	time_t xTmpTime;
	memcpy(&TCPBuffer[19],&xTmpTime,sizeof(time_t));
	memset(&TCPBuffer[23],0x00,20);
    vSendTCPReply(TCPBuffer,0x03);
	}
	
	break;
	
	case 0x04:
	{
	 TCPBuffer[0]=0x04;
     memcpy(&TCPBuffer[1],&xLogTable,sizeof(xLogTable));
     vSendTCPReply(TCPBuffer,sizeof(xLogTable)+0x01);
	}
		
	break;
	
	case 0x05:
	{  
	 UINT uSize;
	 uint8_t ucTempPage=TCPBuffer[2];
	 sprintf(TempString,"0:/Logs/Log%03d.bin",TCPBuffer[1]);
	 f_open(&file_object,TempString,FA_READ);
	 f_lseek(&file_object,(uint32_t)ucTempPage*sizeof(xLogData)*50);
     memset(TCPBuffer,0x00,sizeof(TCPBuffer));
	 f_read(&file_object,&TCPBuffer[2],sizeof(xLogData)*50,&uSize);
	 f_close(&file_object);		
	 TCPBuffer[0]=5;
	 TCPBuffer[1]=ucTempPage;
	 vSendTCPReply(TCPBuffer,sizeof(xLogData)*50+0x02);
	}
	
	break;	
	
	case 0x06:
	{
     UINT uSize;		
	
	 memcpy(&xTags[TCPBuffer[1]],&TCPBuffer[2],sizeof(xTagData));
	
	 f_open(&file_object,"0:/TagList.bin",FA_WRITE);
	 f_lseek(&file_object,(uint32_t)TCPBuffer[1]*sizeof(xTagData));
	 f_write(&file_object,&TCPBuffer[2],sizeof(xTagData),&uSize); 
	 f_close(&file_object);	
	 
	 TCPBuffer[0]=6;
	 TCPBuffer[1]=0x00;
	 vSendTCPReply(TCPBuffer,0x02);	 	 
	 	
	}
	break;
	
	case 0x07:
	break;
	
    case 0x08:
	{
     UINT uSize;		
	
	 f_open(&file_object,"0:/Labels.bin",FA_WRITE);
	 f_lseek(&file_object,(uint32_t)TCPBuffer[1]*sizeof(LCDlabels)+(uint32_t)TCPBuffer[2]*MAXLSIZE);
	 f_write(&file_object,&TCPBuffer[3],MAXLSIZE,&uSize); 
	 f_close(&file_object);
	 
	 f_open(&file_object,"0:/Labels.bin",FA_READ);
	 f_read(&file_object,LCDlabels,sizeof(LCDlabels),&uSize);
	 f_close(&file_object);
	 
	 TCPBuffer[0]=8;
	 TCPBuffer[1]=0x00;
	 vSendTCPReply(TCPBuffer,0x02);	 
	 
	}
	break;
	
    case 0x09:
    {
	    UINT uSize;
	    
        ulMachineWorkTime=(uint32_t)xMachineSetup.usRevTime*60;
        f_open(&file_object,"0:/RunTime.bin",FA_CREATE_ALWAYS | FA_WRITE);
        f_write(&file_object,&ulMachineWorkTime,sizeof(ulMachineWorkTime),&uSize);
        f_close(&file_object);
	    
	    TCPBuffer[0]=9;
	    TCPBuffer[1]=0x00;
	    vSendTCPReply(TCPBuffer,0x02);
	    
    }
    break;	
	
	default:break;   
  }
  
  recv(tcp_client_socket, TCPBuffer, sizeof(TCPBuffer), 10000);
  
 }
	
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////