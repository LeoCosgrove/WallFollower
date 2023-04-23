/* Based on LAB21 Solution
 * TODO:
 * Web interface
 * Tune wall following
 * Set delay time after collision
 */
//comment to test

#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "./inc/Clock.h"
#include "./inc/I2CB1.h"
#include "./inc/CortexM.h"
#include "./inc/LPF.h"
#include "./inc/opt3101.h"
#include "./inc/LaunchPad.h"
#include "./inc/Bump.h"
#include "./inc/Motor.h"
#include "./inc/UART0.h"
#include "./inc/FFT.h"
#include "./inc/UART1.h"
#include "./driverlib/MSP432P4xx/driverlib.h"
#include "./simplelink/include/simplelink.h"
#include "sl_common.h"
#include "./mqtt/MQTTClient.h"
#include "./inc/Tachometer.h"
#include "./inc/odometry.h"

uint16_t avg(uint16_t *array, int length)
{
  int i;
  uint32_t sum = 0;

  for(i=0; i<length; i=i+1)
  {
    sum = sum + array[i];
  }
  return (sum/length);
}

bool runFlag = true;

uint16_t ActualL;                        // actual rotations per minute
uint16_t ActualR;                        // actual rotations per minute
#define TACHBUFF 10                      // number of elements in tachometer array
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)

/*
 * Values for below macros shall be modified per the access-point's (AP) properties
 * SimpleLink device will connect to following AP when the application is executed
 */
#define SSID_NAME       "iPhone (2)."       /* Access point name to connect to. */
#define SEC_TYPE        SL_SEC_TYPE_WPA_WPA2     /* Security type of the Access piont */
#define PASSKEY         "Higginsj"   /* Password in case of secure AP */
#define PASSKEY_LEN     pal_Strlen(PASSKEY)  /* Password length in case of secure AP */

/*
 * MQTT server and topic properties that shall be modified per application
 */
#define MQTT_BROKER_SERVER  "broker.hivemq.com"
#define SUBSCRIBE_TOPIC "scribeTopic"
#define PUBLISH_TOPIC "CyberJ"

// MQTT message buffer size
#define BUFF_SIZE 32


#define APPLICATION_VERSION "1.0.0"

#define MCLK_FREQUENCY 48000000
#define PWM_PERIOD 255

#define SL_STOP_TIMEOUT        0xFF

#define SMALL_BUF           32
#define MAX_SEND_BUF_SIZE   512
#define MAX_SEND_RCV_SIZE   1024

/* Application specific status/error codes */
typedef enum{
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap with host-driver's error codes */
    HTTP_SEND_ERROR = DEVICE_NOT_IN_STATION_MODE - 1,
    HTTP_RECV_ERROR = HTTP_SEND_ERROR - 1,
    HTTP_INVALID_RESPONSE = HTTP_RECV_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define min(X,Y) ((X) < (Y) ? (X) : (Y))

/*
 * GLOBAL VARIABLES -- Start
 */
/* Button debounce state variables */
volatile unsigned int S1buttonDebounce = 0;
volatile unsigned int S2buttonDebounce = 0;
volatile int publishID = 0;

unsigned char macAddressVal[SL_MAC_ADDR_LEN];
unsigned char macAddressLen = SL_MAC_ADDR_LEN;

char macStr[18];        // Formatted MAC Address String
char uniqueID[9];       // Unique ID generated from TLV RAND NUM and MAC Address

Network n;
Client hMQTTClient;     // MQTT Client

_u32  g_Status = 0;
struct{
    _u8 Recvbuff[MAX_SEND_RCV_SIZE];
    _u8 SendBuff[MAX_SEND_BUF_SIZE];

    _u8 HostName[SMALL_BUF];
    _u8 CityName[SMALL_BUF];

    _u32 DestinationIP;
    _i16 SockID;
}g_AppData;

/* Port mapper configuration register */
const uint8_t port_mapping[] =
{
    //Port P2:
    PM_TA0CCR1A, PM_TA0CCR2A, PM_TA0CCR3A, PM_NONE, PM_TA1CCR1A, PM_NONE, PM_NONE, PM_NONE
};

/* TimerA UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_8,          // SMCLK/8 = 6MHz
        90000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/*
 * GLOBAL VARIABLES -- End
 */

/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
static _i32 establishConnectionWithAP();
static _i32 configureSimpleLinkToDefaultState();
static _i32 initializeAppVariables();
static void displayBanner();
static void messageArrived(MessageData*);
static void generateUniqueID();


/*
 * STATIC FUNCTION DEFINITIONS -- End
 */

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

void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
        CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");

    switch(pWlanEvent->Event)
    {
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
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                CLI_Write(" Device disconnected from the AP on application's request \n\r");
            }
            else
            {
                CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
            }
        }
        break;

        default:
        {
            CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
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
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
        CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");

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
            CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
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
                                  SlHttpServerResponse_t *pHttpResponse)
{
    /*
     * This application doesn't work with HTTP server - Hence these
     * events are not handled here
     */
    CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /*
     * Most of the general errors are not FATAL are are to be handled
     * appropriately by the application
     */
    CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
*/
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
        CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");

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
            * SlSockEventData_t *pEventData = NULL;
            * pEventData = & pSock->EventData;
            */
            switch( pSock->EventData.status )
            {
                case SL_ECLOSE:
                    CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
                break;


                default:
                    CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
                break;
            }
        }
        break;

        default:
            CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
        break;
    }
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */

// this batch configures for UART link to PC
#include "./inc/UART0.h"
void UartSetCur(uint8_t newX, uint8_t newY){
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};
#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec
#define OutSDec UART0_OutSDec

uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t Noises[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
bool pollDistanceSensor(void){
  if(OPT3101_CheckDistanceSensor()){
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}

// calibrated for 500mm track
// right is raw sensor data from right sensor
// return calibrated distance from center of Robot to right wall
int32_t Right(int32_t right){
  return  (right*(59*right + 7305) + 2348974)/32768;
}
// left is raw sensor data from left sensor
// return calibrated distance from center of Robot to left wall
int32_t Left(int32_t left){
  return (1247*left)/2048 + 22;
}

#define N 1024
uint32_t Data[N];
#define M 1024
uint16_t Histogram[M];
uint32_t Sum;      // sum of data
uint32_t Sum2;     // sum of (data-average)^2
uint32_t Average;  // average of data = sum/N
uint32_t Variance; // =sum2/(N-1)
uint32_t Sigma;    // standard deviation = sqrt(Variance)

// assumes track is 500mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Error;
int32_t Ki=1800;  // integral controller gain
int32_t Kp=1;  // proportional controller gain //was 4
int32_t UR, UL;  // PWM duty 0 to 14,998

#define TOOCLOSE 325 //was 200
#define DESIRED 400 //was 250
int32_t SetPoint = DESIRED; // mm //was 250
int32_t LeftDistance,CenterDistance,RightDistance; // mm
#define TOOFAR 550 // was 400

#define PWMNOMINAL 5000 // was 2500
#define SWING 2050 //was 1000
#define PWMMIN (PWMNOMINAL-SWING)
#define PWMMAX (PWMNOMINAL+SWING)

void Controller(void){ // runs at 100 Hz
  if(Mode){
    if((LeftDistance>DESIRED)&&(RightDistance>DESIRED)){
      SetPoint = (LeftDistance+RightDistance)/2;
    }else{
      SetPoint = DESIRED;
    }
    if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }
 //   UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UL = PWMNOMINAL-Kp*Error; // proportional control
    if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;
//    Motor_Forward(UL,UR);

  }
}

void Controller_Right(void){ // runs at 100 Hz
  if(Mode){
    if((RightDistance>DESIRED)){
      SetPoint = (RightDistance)/2;
    }else{
      SetPoint = DESIRED;
    }
    /*if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }*/

    Error = SetPoint-RightDistance;
    //UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UR = UR + Ki*Error;      // adjust right motor
    UL = PWMNOMINAL-Kp*Error; // proportional control
    if(UR < (PWMNOMINAL-SWING)) UR = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)) UR = PWMNOMINAL+SWING;
    if(UL < (PWMNOMINAL-SWING)) UL = PWMNOMINAL-SWING; // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)) UL = PWMNOMINAL+SWING;

    //turns left if the center measurement and right measurement is small enough that we will hit the wall if we don't turn
    if((RightDistance<250) && (CenterDistance <250)){
        UL = 0;
        UR = PWMNOMINAL;
    }

    Motor_Forward(UL,UR);

  }
}

void Pause(void){int i;
/*
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(200); LaunchPad_Output(0); // off
    Clock_Delay1ms(200); LaunchPad_Output(1); // red
  }
  while(Bump_Read()==0){// wait for touch
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(3); // red/green
  }
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(4); // blue
  }
  */
//  for(i=1000;i>100;i=i-200){
//    Clock_Delay1ms(i); LaunchPad_Output(0); // off
//    Clock_Delay1ms(i); LaunchPad_Output(2); // green
//  }
  // restart Jacki
  UR = UL = PWMNOMINAL;    // reset parameters
  Mode = 1;

}

void main(void){ // wallFollow wall following implementation
  printf("Hello, World\n");
  int i = 0;
  char charIn = 'g';
  int32_t xcoord, ycoord, heading;
  uint32_t channel = 1;
  DisableInterrupts();
  Tachometer_Init();
  Clock_Init48MHz();
  Odometry_Init(0,0,0);
  Bump_Init();
  LaunchPad_Init(); // built-in switches and LEDs
  Motor_Init();
  UART1_Init();     // initialize UART
  Motor_Stop(); // initialize and stop
  Mode = 0;
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz
  Init();
  Clear();
  OutString("OPT3101");
  SetCursor(0, 1);
  OutString("L=");
  SetCursor(0, 2);
  OutString("C=");
  SetCursor(0, 3);
  OutString("R=");
  SetCursor(0, 4);
  OutString("Wall follow");
  SetCursor(0, 5);
  OutString("SP=");
  SetCursor(0, 6);
  OutString("Er=");
  SetCursor(0, 7);
  OutString("U =");
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);
  TxChannel = 3;
  OPT3101_StartMeasurementChannel(channel);
  LPF_Init(100,8);
  LPF_Init2(100,8);
  LPF_Init3(100,8);
  UR = UL = PWMNOMINAL; //initial power
  Pause();
  EnableInterrupts();
  bool crash = false;

  _i32 retVal = -1;

  retVal = initializeAppVariables();
  ASSERT_ON_ERROR(retVal);

  /* Stop WDT and initialize the system-clock of the MCU */
  stopWDT();
  initClk();

  /* Configure command line interface */
  CLI_Configure();

  int q = 0;

  displayBanner();

  /*
   * Following function configures the device to default state by cleaning
   * the persistent settings stored in NVMEM (viz. connection profiles &
   * policies, power policy etc)
   *
   * Applications may choose to skip this step if the developer is sure
   * that the device is in its default state at start of application
   *
   * Note that all profiles and persistent settings that were done on the
   * device will be lost
   */
  retVal = configureSimpleLinkToDefaultState();
  if(retVal < 0)
  {
      if (DEVICE_NOT_IN_STATION_MODE == retVal)
          CLI_Write(" Failed to configure the device in its default state \n\r");

      LOOP_FOREVER();
  }

  CLI_Write(" Device is configured in default state \n\r");

  /*
   * Assumption is that the device is configured in station mode already
   * and it is in its default state
   */
  retVal = sl_Start(0, 0, 0);
  if ((retVal < 0) ||
      (ROLE_STA != retVal) )
  {
      CLI_Write(" Failed to start the device \n\r");
      LOOP_FOREVER();
  }

  CLI_Write(" Device started as STATION \n\r");

  /* Connecting to WLAN AP */
  retVal = establishConnectionWithAP();
  if(retVal < 0)
  {
      CLI_Write(" Failed to establish connection w/ an AP \n\r");
      LOOP_FOREVER();
  }

  CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

  // Obtain MAC Address
  sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(unsigned char *)macAddressVal);

  // Print MAC Addres to be formatted string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
          macAddressVal[0], macAddressVal[1], macAddressVal[2], macAddressVal[3], macAddressVal[4], macAddressVal[5]);

  // Generate 32bit unique ID from TLV Random Number and MAC Address
  generateUniqueID();

  int rc = 0;
  unsigned char buf[100];
  unsigned char readbuf[100];

  NewNetwork(&n);
  rc = ConnectNetwork(&n, MQTT_BROKER_SERVER, 1883);

  if (rc != 0) {
      CLI_Write(" Failed to connect to MQTT broker \n\r");
      LOOP_FOREVER();
  }
  CLI_Write(" Connected to MQTT broker \n\r");

  MQTTClient(&hMQTTClient, &n, 1000, buf, 100, readbuf, 100);
  MQTTPacket_connectData cdata = MQTTPacket_connectData_initializer;
  cdata.MQTTVersion = 3;
  cdata.clientID.cstring = uniqueID;
  rc = MQTTConnect(&hMQTTClient, &cdata);

  if (rc != 0) {
      CLI_Write(" Failed to start MQTT client \n\r");
      LOOP_FOREVER();
  }
  CLI_Write(" Started MQTT client successfully \n\r");

  rc = MQTTSubscribe(&hMQTTClient, SUBSCRIBE_TOPIC, QOS0, messageArrived);

  if (rc != 0) {
      CLI_Write(" Failed to subscribe to /msp/cc3100/demo topic \n\r");
      LOOP_FOREVER();
  }
  CLI_Write(" Subscribed to /msp/cc3100/demo topic \n\r");

  rc = MQTTSubscribe(&hMQTTClient, uniqueID, QOS0, messageArrived);

  if (rc != 0) {
      CLI_Write(" Failed to subscribe to uniqueID topic \n\r");
      LOOP_FOREVER();
  }
  CLI_Write(" Subscribed to uniqueID topic \n\r");

  while(1){
    rc = MQTTYield(&hMQTTClient, 500);
    if (rc != 0) {
        CLI_Write(" MQTT failed to yield \n\r");
        LOOP_FOREVER();
    }

    Tachometer_Get(&LeftTach[q], &LeftDir, &LeftSteps, &RightTach[q], &RightDir, &RightSteps);
    q = q + 1;

    if(q >= TACHBUFF){
        q = 0;
        ActualL = 2000000/avg(LeftTach, TACHBUFF);
        ActualR = 2000000/avg(RightTach, TACHBUFF);
    }

    if(!runFlag){
        ActualL = 0;
        ActualR = 0;
    }

    if(runFlag){
        Odometry_Update(LeftSteps,RightSteps);
        xcoord = Odometry_GetX();
        ycoord = Odometry_GetY();
        heading = Odometry_GetAngle();
    }

//    printf("\n%u", FilteredDistances[0]);
//    printf("\n%u", &FilteredDistances[0]);
//    printf("\n%d", q);

    //mqtt
    MQTTMessage ch0;
    ch0.dup = 0;
    ch0.id = 0;
    ch0.qos = QOS0;
    ch0.retained = 0;
    ch0.payload = &FilteredDistances[0];
    ch0.payloadlen = sizeof(FilteredDistances[0]);
    rc = MQTTPublish(&hMQTTClient, "distCH0", &ch0);

    MQTTMessage ch1;
    ch1.dup = 0;
    ch1.id = 0;
    ch1.qos = QOS0;
    ch1.retained = 0;
    ch1.payload = &FilteredDistances[1];
    ch1.payloadlen = sizeof(FilteredDistances[1]);
    rc = MQTTPublish(&hMQTTClient, "distCH1", &ch1);

    MQTTMessage ch2;
    ch2.dup = 0;
    ch2.id = 0;
    ch2.qos = QOS0;
    ch2.retained = 0;
    ch2.payload = &FilteredDistances[2];
    ch2.payloadlen = sizeof(FilteredDistances[2]);
    rc = MQTTPublish(&hMQTTClient, "distCH2", &ch2);

    MQTTMessage lrpm;
    lrpm.dup = 0;
    lrpm.id = 0;
    lrpm.qos = QOS0;
    lrpm.retained = 0;
    lrpm.payload = &ActualL;
    lrpm.payloadlen = sizeof(ActualL);
    rc = MQTTPublish(&hMQTTClient, "leftRPM", &lrpm);

    MQTTMessage rrpm;
    rrpm.dup = 0;
    rrpm.id = 0;
    rrpm.qos = QOS0;
    rrpm.retained = 0;
    rrpm.payload = &ActualR;
    rrpm.payloadlen = sizeof(ActualR);
    rc = MQTTPublish(&hMQTTClient, "rightRPM", &rrpm);

    MQTTMessage xcoor;
    xcoor.dup = 0;
    xcoor.id = 0;
    xcoor.qos = QOS0;
    xcoor.retained = 0;
    xcoor.payload = &xcoord;
    xcoor.payloadlen = sizeof(xcoord);
    rc = MQTTPublish(&hMQTTClient, "xLoc", &xcoor);

    MQTTMessage ycoor;
    ycoor.dup = 0;
    ycoor.id = 0;
    ycoor.qos = QOS0;
    ycoor.retained = 0;
    ycoor.payload = &ycoord;
    ycoor.payloadlen = sizeof(ycoord);
    rc = MQTTPublish(&hMQTTClient, "yLoc", &ycoor);

    MQTTMessage head;
    head.dup = 0;
    head.id = 0;
    head.qos = QOS0;
    head.retained = 0;
    head.payload = &heading;
    head.payloadlen = sizeof(heading);
    rc = MQTTPublish(&hMQTTClient, "heading", &head);

    // read BLE
    if(UART1_InStatus() != 0){
        charIn = UART1_InChar();
        switch(charIn){
                case 'g':
                    runFlag = true;
                    break;
                case 's':
                    runFlag = false;
                    Motor_Stop();
                    break;
        }
    }

    if(runFlag)
    {
        uint8_t test = Bump_Read();
        if(Bump_Read()){ // collision
            Mode = 0;
            Motor_Stop();
            Pause();
            crash = 1;
//            report crash
            MQTTMessage crsh;
            crsh.dup = 0;
            crsh.id = 0;
            crsh.qos = QOS0;
            crsh.retained = 0;
            crsh.payload = &crash;
            crsh.payloadlen = sizeof(crash);
            rc = MQTTPublish(&hMQTTClient, "crashBool", &crsh);

            //wait 1 second
            Clock_Delay1ms(1000);

            //recover from crash
            Motor_Backward(9000, 9000);
            Clock_Delay1ms(1000);
            Motor_Stop();

            crash = false;
        }
        else{
            MQTTMessage crsh;
            crsh.dup = 0;
            crsh.id = 0;
            crsh.qos = QOS0;
            crsh.retained = 0;
            crsh.payload = &crash;
            crsh.payloadlen = sizeof(crash);
            rc = MQTTPublish(&hMQTTClient, "crashBool", &crsh);
        }
//        Delay(10);

        if(TxChannel <= 2){ // 0,1,2 means new data
          if(TxChannel==0){
            if(Amplitudes[0] > 1000){
              LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
            }else{
              LeftDistance = FilteredDistances[0] = 500;
            }
          }else if(TxChannel==1){
            if(Amplitudes[1] > 1000){
              CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
            }else{
              CenterDistance = FilteredDistances[1] = 500;
            }
          }else {
            if(Amplitudes[2] > 1000){
              RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
            }else{
              RightDistance = FilteredDistances[2] = 500;
            }
          }
          SetCursor(2, TxChannel+1);
          OutUDec(FilteredDistances[TxChannel]); OutChar(','); OutUDec(Amplitudes[TxChannel]);
          TxChannel = 3; // 3 means no data
          channel = (channel+1)%3;
          OPT3101_StartMeasurementChannel(channel);
          i = i + 1;
        }
        Controller_Right();
        if(i >= 100){
          i = 0;
//          SetCursor(3, 5);
//          OutUDec(SetPoint);
//          SetCursor(3, 6);
//          OutSDec(Error);
//          SetCursor(3, 7);
//          OutUDec(UL); OutChar(','); OutUDec(UR);
        }

        WaitForInterrupt();
      }
    else{
        Motor_Stop();
    }
//    Delay(10);
  }
}

static void generateUniqueID() {
    CRC32_setSeed(TLV->RANDOM_NUM_1, CRC32_MODE);
    CRC32_set32BitData(TLV->RANDOM_NUM_2);
    CRC32_set32BitData(TLV->RANDOM_NUM_3);
    CRC32_set32BitData(TLV->RANDOM_NUM_4);
    int i;
    for (i = 0; i < 6; i++)
    CRC32_set8BitData(macAddressVal[i], CRC32_MODE);

    uint32_t crcResult = CRC32_getResult(CRC32_MODE);
    sprintf(uniqueID, "%06X", crcResult);
}

//****************************************************************************
//
//!    \brief MQTT message received callback - Called when a subscribed topic
//!                                            receives a message.
//! \param[in]                  data is the data passed to the callback
//!
//! \return                        None
//


static void messageArrived(MessageData* data) {
    char buf[BUFF_SIZE];

    // Check for buffer overflow
    if (data->topicName->lenstring.len >= BUFF_SIZE) {
        // UART_PRINT("Topic name too long!\n\r");
        return;
    }
    if (data->message->payloadlen >= BUFF_SIZE) {
        // UART_PRINT("Payload too long!\n\r");
        return;
    }

    strncpy(buf, data->topicName->lenstring.data,
        min(BUFF_SIZE, data->topicName->lenstring.len));
    buf[data->topicName->lenstring.len] = '\0';  // Ensure null terminator

    strncpy(buf, data->message->payload,
        min(BUFF_SIZE, data->message->payloadlen));
    buf[data->message->payloadlen] = '\0';  // Ensure null terminator

    printf("\nbuf: \"%s\"\n", buf);  // Print buf contents

    if (strcmp(buf, "go") == 0) {
        runFlag = true;
    } else if (strcmp(buf, "stop") == 0) {
        runFlag = false;
        Motor_Stop();
    }

    return;
}

///*
// * Port 1 interrupt handler. This handler is called whenever the switch attached
// * to P1.1 is pressed.
// */
//void PORT1_IRQHandler(void)
//{
//    uint32_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
//    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
//
//    if (status & GPIO_PIN1)
//    {
//        if (S1buttonDebounce == 0)
//        {
//            S1buttonDebounce = 1;
//
//            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
//
//            // Publish the unique ID
//            publishID = 1;
//
//            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
//        }
//    }
//    if (status & GPIO_PIN4)
//    {
//        if (S2buttonDebounce == 0)
//        {
//            S2buttonDebounce = 1;
//
//            CLI_Write(" MAC Address: \n\r ");
//            CLI_Write(macStr);
//            CLI_Write("\n\r");
//
//            MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
//        }
//    }
//}
//
//void TA1_0_IRQHandler(void)
//{
//    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
//    if (P1IN & GPIO_PIN1)
//    {
//        S1buttonDebounce = 0;
//    }
//    if (P1IN & GPIO_PIN4)
//    {
//        S2buttonDebounce = 0;
//    }
//
//    if ((P1IN & GPIO_PIN1) && (P1IN & GPIO_PIN4))
//    {
//        Timer_A_stopTimer(TIMER_A1_BASE);
//    }
//    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
//                TIMER_A_CAPTURECOMPARE_REGISTER_0);
//}
//
//
///*!
//    \brief This function configure the SimpleLink device in its default state. It:
//           - Sets the mode to STATION
//           - Configures connection policy to Auto and AutoSmartConfig
//           - Deletes all the stored profiles
//           - Enables DHCP
//           - Disables Scan policy
//           - Sets Tx power to maximum
//           - Sets power policy to normal
//           - Unregisters mDNS services
//           - Remove all filters
//
//    \param[in]      none
//
//    \return         On success, zero is returned. On error, negative is returned
//*/
static _i32 configureSimpleLinkToDefaultState()
{
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

    retVal = initializeAppVariables();
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
static _i32 establishConnectionWithAP()
{
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
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
*/
static _i32 initializeAppVariables()
{
    g_Status = 0;
    pal_Memset(&g_AppData, 0, sizeof(g_AppData));

    return SUCCESS;
}

/*!
    \brief This function displays the application's banner

    \param      None

    \return     None
*/
static void displayBanner()
{
    CLI_Write("\n\r\n\r");
    CLI_Write(" MQTT Twitter Controlled RGB LED - Version ");
    CLI_Write(APPLICATION_VERSION);
    CLI_Write("\n\r*******************************************************************************\n\r");
}
