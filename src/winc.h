/*
 * winc.h
 *
 * Created: 14/07/2016 09.22.28
 *  Author: ROBERTOC
 */ 

#include "asf.h"
#include "FreeRTOS.h"
#include "nm_common.h"
#include "m2m_wifi.h"
#include "socket.h"


#ifndef WINC_H_
#define WINC_H_

#define MAIN_WIFI_M2M_BUFFER_SIZE          1460

#define WIFI_TIMEOUT_RECV				   250
#define WIFI_TIMEOUT_SEND				   10000

typedef enum {BR56700,BR115200,BR345600} xBRates;
typedef enum {UDP,TCP} xConn;

/** Message format definitions. */
typedef struct s_msg_wifi_product {
	uint8_t name[9];
} t_msg_wifi_product;

typedef struct {
  char cSSID[30];
  char cPassword[30];
  char cHostName[30];
  uint32_t u32StaticIP;
  uint32_t u32SubnetMask; //corresponds to 255.255.254.0
  uint32_t u32Gateway;  //corresponds to 192.168.1.254
  tenuM2mSecType xSecurityType;
  xBRates xBaudrate;
  xConn xConnectionMode;
  bool xDHCP;
  char Spare[50];
} xWifiConfig;

TaskHandle_t xWincTaskH;
TaskHandle_t xCmdTaskH;

char HTTPBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

COMPILER_ALIGNED(32)
char TCPBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

//SemaphoreHandle_t xIntSemaphore;

void vSwitchToAPMode(void);
void vSwitchToClientMode(void);
void vCreateWincTask(void);
void vCloseHTTP(void);
//void vSendUDPReply(char *, uint16_t );
//void vSendHTTPReply(char *, uint16_t );
//void vSendTCPReply(char *, uint16_t );
void vLoadConfiguration(void);
void vSaveConfiguration(void);
int16_t sGetSize(void);

extern SOCKET http_client_socket;
extern SOCKET tcp_client_socket;

QueueHandle_t xAcceptQueue;
xWifiConfig xConfigData;


#endif /* WINC_H_ */