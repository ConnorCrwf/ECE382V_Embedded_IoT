/*
 * main.c - Example project for UT.6.02x Embedded Systems - Shape the World
 * Jonathan Valvano and Ramesh Yerraballi
 * July 29 2021 (both weathermap and ifttt email tested 7/29/2021)
 * Hardware requirements 
     MSP432 LaunchPad, switches, LEDs, debugging output to PC (UART)
     CC3100 wifi booster and 
     an internet access point with OPEN, WPA, or WEP security
   TI CC3100 simplelink SDK v1.3.0 release 
 * Software requirements (not MSP432 specific)
          ..\cc3100-sdk\simplelink\source\device.c 
          ..\cc3100-sdk\simplelink\source\driver.c 
          ..\cc3100-sdk\simplelink\source\flowcont.c 
          ..\cc3100-sdk\simplelink\source\fs.c 
          ..\cc3100-sdk\simplelink\source\netapp.c 
          ..\cc3100-sdk\simplelink\source\netcfg.c 
          ..\cc3100-sdk\simplelink\source\nonos.c 
          ..\cc3100-sdk\simplelink\source\socket.c
          ..\cc3100-sdk\simplelink\source\spawn.c 
          ..\cc3100-sdk\simplelink\source\wlan.c 
          ..\utils\ustdlib.c 
          ..\driverlib\sysctl.c 
          ..\driverlib\interrupt.c 
          ..\driverlib\fpu.c enables floating point and lazy stack
          ..\driverlib\cpu.c
          ..\inc\SimpleLinkEventHandlers.c
 * Software requirements (MSP432 specific)
      cc3100-sdk\platform\msp432p\board.c (edited from a MSP430 version) handles 
          sets clock to 48MHz (calls ClockSystem)
          P2.5 IRQ input from CC3100 causes edge interrupts 
          P4.1 nHIB output to CC3100 hiberate 
      ..\cc3100-sdk\platform\msp432p\spi.c (edited from a MSP430 version) handles communication serial with CC3100
          P2.5 IRQ input from CC3100 causes edge interrupts 
          P3.0 SPI_CS  (GPIO output, active low)
          P1.5 SPI_CLK (UCB0, 12 MHz)
          P1.6 SPI_MOSI(UCB0)
          P1.7 SPI_MISO(UCB0)
          P4.1 nHIB output to CC3100 hiberate 
          P5.1 WLAN_LOG_TX UART log data arrives from CC3100 into MSP432, but this line is not used
          P2.3 NWP_LOG_TX UART log data arrives into MSP432, but this line is not used
          P3.3 UART1_RX to CC3100 set to weak pullup (not used)
          P3.2 UART1_TX UART communication arrives from CC3100 into MSP432, but this line is not used
          P5.6 UART1_CTS from MSP432 to CC3100, not initialized, this line is initially low, but then goes high
          P6.6 UART1_RTS from CC3100 to MSP432, not used, this line is initially low, but then goes high
      ..\inc\LaunchPad.c outputs to LEDs, inputs from switches on the MSP432 LaunchPad
          P1.1 P1.4 negative logic switch inputs
          P2.2, P2.1, P2.0 positive logic LED outputs
      ..\inc\Clock.c manages bus clocks, set to 48 MHz, SMCLK to 12 MHz, ACLK to 32.786kHz
      ..\inc\UART0.c debugging output to serial port to PC
          P1.2 UCA0RXD (VCP receive) 
          P1.3 UCA0TXD (VCP transmit) 
      ..\cc3100-sdk\platform\msp432p\timer_tick.c enables TimerA2 for simplelink time-stamping feature

* main.c - get weather details sample application
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Application Name     -   Get weather
 * Application Overview -   This is a sample application demonstrating how
 *                          to connect to openweathermap.org server and request
 *                          for weather details of a city. The application\
 *                          opens a TCP socket w/ the server and sends a HTTP
 *                          Get request to get the weather details. The received
 *                          data is processed and displayed on the console
 * Application Details  -   http://processors.wiki.ti.com/index.php/CC31xx_Get_Weather_Application
 *                          doc\examples\get_weather.pdf
 */

/* CC3100 booster pack MSP432 connections (unused pins can be used by user application)
Pin  Signal         Direction   Pin   Signal      Direction
P1.1  3.3  VCC        IN        P2.1  Gnd   GND       IN
P1.2  P6.0 UNUSED     NA        P2.2  P2.5  IRQ       OUT
P1.3  P3.2 UART1_TX   OUT       P2.3  P3.0  SPI_CS    IN
P1.4  P3.3 UART1_RX   IN        P2.4  P5.7  UNUSED    NA
P1.5  P4.1 nHIB       IN        P2.5  Reset nRESET    IN
P1.6  P4.3 UNUSED     NA        P2.6  P1.6  SPI_MOSI  IN
P1.7  P1.5 SPI_CLK    IN        P2.7  P1.7  SPI_MISO  OUT
P1.8  P4.6 UNUSED     NA        P2.8  P5.0  UNUSED    NA
P1.9  P6.5 UNUSED     NA        P2.9  P5.2  UNUSED    NA
P1.10 P6.4 UNUSED     NA        P2.10 P3.6  UNUSED    NA

Pin   Signal        Direction   Pin   Signal        Direction
P3.1  +5   +5V        IN        P4.1  P2.7  UNUSED      OUT
P3.2  Gnd  GND        IN        P4.2  P2.6  UNUSED      OUT
P3.3  P6.1 UNUSED     NA        P4.3  P2.4  UNUSED      NA
P3.4  P4.0 UNUSED     NA        P4.4  P5.6  UART1_CTS   IN
P3.5  P4.2 UNUSED     NA        P4.5  P6.6  UART1_RTS   OUT
P3.6  P4.4 UNUSED     NA        P4.6  P6.7  UNUSED      NA
P3.7  P4.5 UNUSED     NA        P4.7  P2.3  NWP_LOG_TX  OUT
P3.8  P4.7 UNUSED     NA        P4.8  P5.1  WLAN_LOG_TX OUT
P3.9  P5.4 UNUSED     NA        P4.9  P3.5  UNUSED      IN (see R74)
P3.10 P5.5 UNUSED     NA        P4.10 P3.7  UNUSED      OUT(see R75)

// UCA0RXD (VCP receive) connected to P1.2, debugging output
// UCA0TXD (VCP transmit) connected to P1.3

*/
#include "simplelink.h"
#include "sl_common.h"
#include "UART0.h"
#include "debug.h"
#include "fpu.h"
#include "rom.h"
#include "sysctl.h"
#include <stdio.h>
#include "application_commands.h"
#include "LaunchPad.h"
#include <string.h>
// edit sl_common.h file with access point information
//    you will find it in \cc3100-sdk\examples\common



/* Application specific status/error codes */
#define SL_STOP_TIMEOUT     0xFF

typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        // Choosing this number to avoid overlap with host-driver's error codes
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

/*
 * GLOBAL VARIABLES -- Start
 */
_u32  g_Status = 0;

/*
 * GLOBAL VARIABLES -- End
 */


/*
 * STATIC FUNCTION PROTOTYPES -- Start
 */
_i32 establishConnectionWithAP(void);
_i32 disconnectFromAP(void);
_i32 configureSimpleLinkToDefaultState(void);

/*
 * STATIC FUNCTION PROTOTYPES -- End
 */

#define MAX_RECV_BUFF_SIZE  1024
#define MAX_SEND_BUFF_SIZE  512
#define SUCCESS             0

/*
 * GLOBAL VARIABLES -- Start
 */

char Recvbuff[MAX_RECV_BUFF_SIZE];
char SendBuff[MAX_SEND_BUFF_SIZE];
unsigned long DestinationIP;
int SockID;


/*
 * GLOBAL VARIABLES -- End
 */

void Crash(uint32_t time){
  printf(" Failed\n");
  while(1){
    for(int i=time;i;i--){};
    LaunchPad_LED(1);    // toggle
    LaunchPad_Output(0);
    for(int i=time;i;i--){};
    LaunchPad_LED(0);    // toggle
    LaunchPad_Output(RED);
  }
}
/*
 * GetWeather parameters
 */
// 1) change Austin Texas to your city
// 2) you can change metric to imperial if you want temperature in F
#define WELCOME "\nFetching weather from openweathermap.org"
#define WEBPAGE "api.openweathermap.org"
#define REQUEST "GET /data/2.5/weather?q=Austin&APPID=1234567890abcdef1234567890abcdef&units=metric HTTP/1.1\nHost:api.openweathermap.org\nAccept: */*\n\n"
// 1) go to http://openweathermap.org/appid#use 
// 2) Register on the Sign up page
// 3) get an API key (APPID) replace the 1234567890abcdef1234567890abcdef with your APPID

/*
 * IFTTT send email parameters
 */
//#define WELCOME "\nIFTTT trigger email"
//#define WEBPAGE "maker.ifttt.com"
//#define REQUEST "POST /trigger/button_pressed/with/key/1234567890abcdef1234567890abcdef HTTP/1.1\nHost: maker.ifttt.com\nUser-Agent: CCS/9.0.1\nConnection: close\nContent-Type: application/json\nContent-Length: 68\n\n{\"value1\" : \"TI-RSLK MAX\", \"value2\" : \"Hello\", \"value3\" : \"World!\" }\n\n"
// 1) create an account on IFTTT (record your key)
// 2) follow directions in Lab 20 to create an IFTTT applet called button_pressed that triggers on Webhooks and then sends you email
// 3) replace 1234567890abcdef1234567890abcdef with your
int main(void){int32_t retVal;  
int32_t ASize = 0; SlSockAddrIn_t  Addr;
  initClk();           // 48 MHz
  UART0_Initprintf();  // Send data to PC, 115200 bps
  LaunchPad_Init();    // initialize LaunchPad I/O
  printf(WELCOME);
  printf("\nStarting configureSimpleLinkToDefaultState(); ...");
  retVal = configureSimpleLinkToDefaultState();  
  if(retVal < 0)Crash(4000000);
  printf(" Completed\nStarting sl_Start(0, 0, 0); ...");
  retVal = sl_Start(0, 0, 0);
  if((retVal < 0) || (ROLE_STA != retVal) ) Crash(8000000);
  printf(" Completed\nStarting establishConnectionWithAP(); ...");
  retVal = establishConnectionWithAP();
  if(retVal < 0)Crash(1000000);
  printf(" Connected\n");
  while(1){
    printf("Starting sl_NetAppDnsGetHostByName(%s) ...",WEBPAGE);
    retVal = sl_NetAppDnsGetHostByName((_i8 *)WEBPAGE,
             strlen(WEBPAGE),&DestinationIP, SL_AF_INET);
    if(retVal == 0){
      printf(" Completed\nCreating a socket ...");
      Addr.sin_family = SL_AF_INET;
      Addr.sin_port = sl_Htons(80);
      Addr.sin_addr.s_addr = sl_Htonl(DestinationIP);// IP to big endian 
      ASize = sizeof(SlSockAddrIn_t);
      SockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
      if( SockID >= 0 ){
        printf(" Completed\nConnecting to this socket ...");
        retVal = sl_Connect(SockID, ( SlSockAddr_t *)&Addr, ASize);
      }
      if((SockID >= 0)&&(retVal >= 0)){
        strcpy(SendBuff,REQUEST); 
        printf(" Completed\nSending this TCP payload to socket\n%s",SendBuff);
        sl_Send(SockID, SendBuff, strlen(SendBuff), 0);   // Send the HTTP GET/POST
        sl_Recv(SockID, Recvbuff, MAX_RECV_BUFF_SIZE, 0); // Receive response
        sl_Close(SockID);
        LaunchPad_Output(GREEN);
        printf("Received this response from server\n%s\n",Recvbuff);
      }else{
        printf(" Failed\n");
      }
    }else{
        printf(" Failed\n");
    }
    printf("Push LaunchPad switch to run again\n");
    while(LaunchPad_Input()==0){}; // wait for touch
    LaunchPad_Output(0);
  }
}

/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
*/
_i32 configureSimpleLinkToDefaultState(void){
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    _u8           val = 1;
    _u8           configOpt = 0;
    _u8           configLen = 0;
    _u8           power = 0;

    _i32          retVal = -1;
    _i32          mode = -1;

    mode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(mode);

    /* If the device is not in station-mode, try configuring it in station-mode */
    if (ROLE_STA != mode)
    {
        if (ROLE_AP == mode)
        {
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
        }

        /* Switch to STA role and restart */
        retVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Stop(SL_STOP_TIMEOUT);
        ASSERT_ON_ERROR(retVal);

        retVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(retVal);

        /* Check if the device is in station again */
        if (ROLE_STA != retVal)
        {
            /* We don't want to proceed if the device is not coming up in station-mode */
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    /* Get the device's version-information */
    configOpt = SL_DEVICE_GENERAL_VERSION;
    configLen = sizeof(ver);
    retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
    ASSERT_ON_ERROR(retVal);

    /* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
    retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove all profiles */
    retVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(retVal);

    /*
     * Device in station-mode. Disconnect previous connection if any
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    /* Enable DHCP client*/
    retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
    ASSERT_ON_ERROR(retVal);

    /* Disable scan */
    configOpt = SL_SCAN_POLICY(0);
    retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
    power = 0;
    retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
    ASSERT_ON_ERROR(retVal);

    /* Set PM policy to normal */
    retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(retVal);

    /* Unregister mDNS services */
    retVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(retVal);

    /* Remove  all 64 filters (8*8) */
    pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(retVal);

    retVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(retVal);

  //  retVal = initializeAppVariables();
  //  ASSERT_ON_ERROR(retVal);

    return retVal; /* Success */
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
*/
_i32 establishConnectionWithAP(void){
    SlSecParams_t secParams = {0};
    _i32 retVal = 0;

    secParams.Key = PASSKEY;
    secParams.KeyLen = PASSKEY_LEN;
    secParams.Type = SEC_TYPE;

    retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(retVal);

    /* Wait */
    while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

    return SUCCESS;
}

/*!
    \brief Disconnecting from a WLAN Access point

    This function disconnects from the connected AP

    \param[in]      None

    \return         none

    \note

    \warning        If the WLAN disconnection fails, we will be stuck in this function forever.
*/
_i32 disconnectFromAP(void){
    _i32 retVal = -1;

    /*
     * The function returns 0 if 'Disconnected done', negative number if already disconnected
     * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
     */
    retVal = sl_WlanDisconnect();
    if(0 == retVal)
    {
        /* Wait */
        while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
    }

    return SUCCESS;
}

/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */

/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent){
  if(pWlanEvent == NULL)
    printf(" [WLAN EVENT] NULL Pointer Error \n\r");
  switch(pWlanEvent->Event)    {
    case SL_WLAN_CONNECT_EVENT:
    {
      SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

            /*
             * Information about the connected AP (like name, MAC etc) will be
             * available in 'slWlanConnectAsyncResponse_t' - Applications
             * can use it if required
             *
             * slWlanConnectAsyncResponse_t *pEventData = NULL;
             * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
             *
             */
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            /* If the user has initiated 'Disconnect' request, 'reason_code' is SL_USER_INITIATED_DISCONNECTION */
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                printf(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                printf(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            printf(" [WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent){
    if(pNetAppEvent == NULL)
        printf(" [NETAPP EVENT] NULL Pointer Error \n\r");

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

            /*
             * Information about the connected AP's IP, gateway, DNS etc
             * will be available in 'SlIpV4AcquiredAsync_t' - Applications
             * can use it if required
             *
             * SlIpV4AcquiredAsync_t *pEventData = NULL;
             * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
             * <gateway_ip> = pEventData->gateway;
             *
             */
        }
        break;

        default:
        {
            printf(" [NETAPP EVENT] Unexpected event \n\r");
        }
        break;
    }
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
*/
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse){
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
  printf(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent){
    /*
     * Most of the general errors are not FATAL and are to be handled
     * appropriately by the application
     */
    printf(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock){
    if(pSock == NULL)
        printf(" [SOCK EVENT] NULL Pointer Error \n\r");

    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        {
            /*
            * TX Failed
            *
            * Information about the socket descriptor and status will be
            * available in 'SlSockEventData_t' - Applications can use it if
            * required
            *
            * SlSockEventData_u *pEventData = NULL;
            * pEventData = & pSock->socketAsyncEvent;
            */
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE:
                    printf(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    printf(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            printf(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */




