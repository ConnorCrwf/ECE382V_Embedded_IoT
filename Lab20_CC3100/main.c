/*
 * main.c - Example project for TI-RSLK MAX
 * Jonathan Valvano and Ramesh Yerraballi
 * June 21, 2019
 * Hardware requirements 
     MSP432 LaunchPad, switches, LEDs, debugging output to PC (UART)
     TI-RSLK MAX negative logic bumper switches on P4.7, P4.6, P4.5, P4.3, P4.2, and P4.0
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
      ..\inc\bump.c, hardware in negative logic and software in positive logic
          P4.7 Bump5, left side of robot
          P4.6 Bump4
          P4.5 Bump3
          P4.3 Bump2
          P4.2 Bump1
          P4.0 Bump0, right side of robot
       ../inc/TimerA1.c, used to run odometry in background
* derived from main.c - get weather details sample application
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
#include "bump.h"
#include "clock.h"
#include "..\inc\TimerA1.h"
#include "..\inc\Motor.h"
#include "..\inc\Tachometer.h"
#include "..\inc\SSD1306.h"
#include "..\inc\odometry.h"
#include "..\inc\blinker.h"

//enum outputtype CurrentOutputType = OLED;
extern int32_t MyX,MyY;               // position in 0.0001cm
extern int32_t MyTheta;               // direction units 2*pi/16384 radians (-pi to +pi)
extern enum RobotState Action;

// edit sl_common.h file with access point information
//    you will find it in \cc3100-sdk\examples\common
// MAX_SEND_BUFF_SIZE and MAX_RECV_BUFF_SIZE must greater than 200++3*MAX_VALUE_BUFF_SIZE
#define SL_STOP_TIMEOUT        0xFF
#define MAX_RECV_BUFF_SIZE  1024
#define MAX_SEND_BUFF_SIZE  1024
#define SUCCESS             0
#define MAX_VALUE_BUFF_SIZE 256
char Value1String[MAX_VALUE_BUFF_SIZE];
char Value2String[MAX_VALUE_BUFF_SIZE];
char Value3String[MAX_VALUE_BUFF_SIZE];
int32_t DataCount;
char LogMessage[100+3*MAX_VALUE_BUFF_SIZE];
/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

_u32  g_Status = 0;


_i32 establishConnectionWithAP(void);
_i32 disconnectFromAP(void);
_i32 configureSimpleLinkToDefaultState(void);

char Recvbuff[MAX_RECV_BUFF_SIZE];
char SendBuff[MAX_SEND_BUFF_SIZE];
unsigned long DestinationIP;
int SockID;

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
int Running; // 0 means stopped
void CheckBumper(void){
  if(Running){
    if(Bump_Read()){
      Motor_Stop(); // stop on bump switch
      Running = 0;
    }
  }
}
uint32_t RacingStatus; // true when last goal meet
uint32_t bWifi=0;      // true if using Wifi
void Racing(void){
  UpdatePosition();
  RacingStatus = CheckGoal();
}

#define WEBPAGE "maker.ifttt.com"
#define REQUEST "POST /trigger/log_data/with/key/1234567890abcdef1234567890abcdef HTTP/1.1\nHost: maker.ifttt.com\nUser-Agent: CCS/9.0.1\nConnection: close\nContent-Type: application/json\nContent-Length: "
// 1) create an account on IFTTT (record your key)
// 2) create an IFTTT applet called log_data that triggers on Webhooks and then adds a row to your googlesheet
// 3) replace 1234567890abcdef1234567890abcdef with your IFTTT key
void ClearData(void){
  Value1String[0] = Value2String[0] = Value3String[0] = 0; // empty strings
  DataCount = 0;
}
void LogData(int32_t x, int32_t y, int32_t th){
  if(strlen(Value1String)>(MAX_VALUE_BUFF_SIZE-32)) return;
  if(strlen(Value2String)>(MAX_VALUE_BUFF_SIZE-32)) return;
  if(strlen(Value3String)>(MAX_VALUE_BUFF_SIZE-32)) return;
  if(DataCount > 0){
    strcat(Value1String,", "); // comma separated values
    strcat(Value2String,", ");
    strcat(Value3String,", ");
  }
  sprintf(LogMessage,"%d",x/1000); // 1 mm
  strcat(Value1String,LogMessage);
  sprintf(LogMessage,"%d",y/1000); // 1 mm
  strcat(Value2String,LogMessage);
  sprintf(LogMessage,"%d",(180*th)/8192); // 1 deg
  strcat(Value3String,LogMessage);
  DataCount++;
}
void SendData(void){int32_t retVal;
int32_t ASize = 0; SlSockAddrIn_t  Addr; int32_t logSize;
  LaunchPad_Output(0);
  if(bWifi==0) return;
  printf("Creating a socket ...");
  Addr.sin_family = SL_AF_INET;
  Addr.sin_port = sl_Htons(80);
  Addr.sin_addr.s_addr = sl_Htonl(DestinationIP);// IP to big endian
  ASize = sizeof(SlSockAddrIn_t);
  SockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
  if( SockID >= 0 ){
    printf(" Completed\nConnecting to this socket ...");
    retVal = sl_Connect(SockID, ( SlSockAddr_t *)&Addr, ASize);
  }
  if((SockID >= 0)&&(retVal >= 0)){ // distances in mm
    sprintf(LogMessage,"{\"value1\" : \"%s\", \"value2\" : \"%s\", \"value3\" : \"%s\" }",Value1String,Value2String,Value3String);
    logSize = strlen(LogMessage);
    sprintf(SendBuff,"%s%d\n\n%s\n\n",REQUEST,logSize,LogMessage);
    printf(" Completed\nSending this TCP payload to socket\n%s",SendBuff);
    sl_Send(SockID, SendBuff, strlen(SendBuff), 0);   // Send the HTTP GET
    sl_Recv(SockID, Recvbuff, MAX_RECV_BUFF_SIZE, 0); // Receive response
    sl_Close(SockID);
    LaunchPad_Output(GREEN);
    printf("Received this response from server\n%s\n",Recvbuff);
  }else{
    printf(" Failed\n");
  }
}

uint32_t result;
int main(void){int32_t retVal;
  initClk();        // 48 MHz
  Bump_Init();      // RSLK bump switches
  LaunchPad_Init();      // initialize LaunchPad I/O

  LaunchPad_Output(BLUE);
  Motor_Init();
  Motor_Stop();
  Blinker_Init();
  Tachometer_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_Clear(); SSD1306_SetCursor(0,0);
  if(LaunchPad_Input()==0){
    Odometry_SetPower(7000,3000); ///< PWM for fast motions, out of 15000
    SSD1306_OutString("RSLK MAX, Fast");
  }else{
    Odometry_SetPower(4000,2000); ///< PWM for fast motions, out of 15000
    SSD1306_OutString("RSLK MAX, Valvano");
  }
  SSD1306_SetCursor(0,1); SSD1306_OutString("Lab 20 Wi-Fi");
  SSD1306_SetCursor(0,2); SSD1306_OutString("North at (0,0)");
  if(Bump_Read()){
    bWifi=0;
    SSD1306_SetCursor(0,3); SSD1306_OutString("Wifi is off");
  }else{
    bWifi = 1;
    UART0_Initprintf();    // Send data to PC, 115200 bps
    printf("Lab 20 Barrel racing\nStarting configureSimpleLinkToDefaultState(); ...");
    retVal = configureSimpleLinkToDefaultState();
    if(retVal < 0)Crash(4000000);
    printf(" Completed\nStarting sl_Start(0, 0, 0); ...");
    retVal = sl_Start(0, 0, 0);
    if((retVal < 0) || (ROLE_STA != retVal) ) Crash(8000000);
    printf(" Completed\nStarting establishConnectionWithAP(); ...");
    retVal = establishConnectionWithAP();
    if(retVal < 0)Crash(1000000);
    printf("Connected\n");
    SSD1306_SetCursor(0,3); SSD1306_OutString("Connected");
    printf("\nStarting sl_NetAppDnsGetHostByName(%s) ...",WEBPAGE);
    retVal = sl_NetAppDnsGetHostByName((_i8 *)WEBPAGE,
              strlen(WEBPAGE),&DestinationIP, SL_AF_INET);
    if(retVal == 0){
     printf(" Completed\n");
    }else{
      printf(" Failed\n");
      bWifi = 0;
    }
  }
  SSD1306_SetCursor(0,4); SSD1306_OutString("Hit bump to start"); printf("Hit bump to start\n");
  TimerA1_Init(&Racing,20000); // every 40ms
  /* logging test code
  ClearData();
  for(int32_t i=10;i<50;i=i+3){
     LogData(i*1000,(i+1)*1000,((i+2)*8192+90)/180,ISSTOPPED);
  }
  SendData();
  */
  WaitUntilBumperTouched();
  while(1){
    ClearData();
    Odometry_Init(0,0,NORTH); UpdatePosition(); // facing North
    Display(); LogData(MyX,MyY,MyTheta);
    ForwardUntilYStart(400000);       // 0,40 cm
    RacingStatus = 0; while(RacingStatus==0){}; LogData(MyX,MyY,MyTheta); Display();
    SoftLeftUntilThStart(WEST);       // 180 or -180
    RacingStatus = 0; while(RacingStatus==0){}; LogData(MyX,MyY,MyTheta); Display();
    ForwardUntilXStart(-400000);      // -40,40 cm
    RacingStatus = 0; while(RacingStatus==0){}; LogData(MyX,MyY,MyTheta); Display();
    SoftLeftUntilThStart(SOUTH);      // -90
    RacingStatus = 0; while(RacingStatus==0){}; LogData(MyX,MyY,MyTheta); Display();
    ForwardUntilYStart(0);            // -40,0 cm
    RacingStatus = 0; while(RacingStatus==0){}; LogData(MyX,MyY,MyTheta); Display();
    SoftLeftUntilThStart(EAST);       // 0
    RacingStatus = 0; while(RacingStatus==0){}; LogData(MyX,MyY,MyTheta); Display();
    ForwardUntilXStart(0);            // 0,0 cm
    RacingStatus = 0; while(RacingStatus==0){}; LogData(MyX,MyY,MyTheta); Display();
    SoftLeftUntilThStart(NORTH);      // 90
    RacingStatus = 0; while(RacingStatus==0){}; Motor_Stop(); LogData(MyX,MyY,MyTheta); Display();
    SendData();
    StopUntilBumperTouched();
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
     * Most of the general errors are not FATAL are are to be handled
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







