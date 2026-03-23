#include <Arduino.h>
/*
 * UDP Autosteer code for Teensy 4.1
 * For AgOpenGPS and CANBUS Autosteer ready tractors
 * 4 Feb 2021, Brian Tischler
 * Like all Arduino code - copied from somewhere else :)
 * So don't claim it as your own
 */

//----------------------------------------------------------

// Tony / @Commonrail Version 18.02.2024

// GPS forwarding mode: (Serial Bynav etc)
// - GPS to Serial3, Forward to AgIO via UDP
// - Forward Ntrip from AgIO (Port 2233) to Serial3

// Panda Mode
// - GPS to Serial3, Forward to AgIO as Panda via UDP
// - Forward Ntrip from AgIO (Port 2233) to Serial3
// - BNO08x Data sent with Panda data

// This CAN setup is for CANBUS based steering controllers as below:
// Danfoss PVED-CL & PVED-CLS (Claas, JCB, Massey Fergerson, CaseIH, New Holland, Valtra, Deutz, Lindner)
// Fendt SCR, S4, Gen6, FendtOne Models need Part:ACP0595080 3rd Party Steering Unlock Installed
// Late model Valtra & Massey with PVED-CC valve (Steering controller in Main Tractor ECU)
//!!Model is selected via serial monitor service tool!! (One day we will will get a CANBUS setup page in AgOpen)

// For engage & disengage via CAN or Button on PCB, select "Button" as switch option in AgOpen
// For engage via AgOpen tablet & disengage via CAN, select "None" as switch option and make sure "Remote" is on the steering wheel icon
// For engage & disengage via PCB switch only select "Switch" as switch option

// PWM value drives set curve up & down, so you need to set the PWM settings in AgOpen
// Normal settings P=15, Max=254, Low=5, Min=1 - Note: New version of AgOpen "LowPWM" is removed and "MinPWM" is used as Low for CANBUS setups (MinPWM hardcoded in .ino coded to 1)
// Some tractors have very fast valves, this smooths out the setpoint from AgOpen

// Workswitch can be operated via PCB or CAN (Will need to setup CAN Messages in ISOBUS section)
// 17.09.2021 - If Pressure Sensor selected, Work switch will be operated when hitch is less than pressure setting (0-250 x 0.4 = 0-100%)
//              Note: The above is temporary use of unused variable, as one day we will get hitch % added to AgOpen
//              Note: There is a AgOpenGPS on MechanicTony GitHub with these two labels & picture changed

// Fendt K-Bus - (Not FendtOne models) Note: This also works with Claas thanks to Ryan
// Big Go/End is operated via hitch control in AgOpen
// Arduino Hitch settings must be enableded and sent to module
//"Invert Relays" Uses section 1 to trigger hitch (Again temporary)

//----------------------------------------------------------

String inoVersion = ("\r\nAndy's board, CANBUS/TM171 INO");

#define useLED 1
////////////////// User Settings /////////////////////////

// How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 0x5422

// Not Connected for Cytron, Right PWM for IBT2
#define STEER_RELAY_5V 9 // D9

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 6 // PD6
#define REMOTE_PIN 8  // PB0
#define WORKSW_PIN 7  // PD7

#define CONST_180_DIVIDED_BY_PI 57.2957795130823
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

// #include <Wire.h>
#include <EEPROM.h>
#include "zNMEAParser.h"
#include "BNO08x_AOG.h"
/* A parser is declared with 3 handlers at most */
NMEAParser<3> parser;

// Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype
extern float tempmonGetTemp(void);
elapsedMillis tempChecker;

//----Teensy 4.1 Ethernet--Start---------------------
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP
{
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 1;
};
ConfigIP networkAddress; // 3 bytes

// Module IP Address / Port
IPAddress ip = {0, 0, 0, 126};
unsigned int localPort = 8888;
unsigned int NtripPort = 2233;

// AOG IP Address / Port
static uint8_t ipDestination[] = {0, 0, 0, 255};
unsigned int AOGPort = 9999;

// MAC address
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x7E};

// Buffer For Receiving UDP Data
byte udpData[128]; // Incomming Buffer
byte NtripData[512];

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
EthernetUDP NtripUdp;

//----Teensy 4.1 Ethernet--End---------------------

//----Teensy 4.1 CANBus--Start---------------------

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> K_Bus;   // Tractor / Control Bus
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> ISO_Bus; // ISO Bus
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> V_Bus;   // Steering Valve Bus

uint8_t Brand = 1;              // Variable to set brand via serial monitor.
uint8_t gpsMode = 1;            // Variable to set GPS mode via serial monitor.
uint8_t CANBUS_ModuleID = 0x1C; // Used for the Module CAN ID

bool reverse_MT = 0;

uint32_t Time;           // Time Arduino has been running
uint32_t relayTime;      // Time to keep "Button Pressed" from CAN Message
boolean engageCAN = 0;   // Variable for Engage from CAN
boolean workCAN = 0;     // Variable for Workswitch from CAN
uint8_t RearHitch = 250; // Variable for hitch height from ISOBUS (Fendt) or KBus (MF/Case) (0-250 *0.4 = 0-100%)
boolean Service = 0;     // Variable for Danfoss Service Tool Mode
boolean ShowCANData = 0; // Variable for Showing CAN Data

boolean goDown = false, endDown = false, bitState = false, bitStateOld = false; // CAN Hitch Control
byte hydLift = 0;
byte goPress[8] = {0x15, 0x20, 0x06, 0xCA, 0x80, 0x01, 0x00, 0x00};  //  press big go
byte goLift[8] = {0x15, 0x20, 0x06, 0xCA, 0x00, 0x02, 0x00, 0x00};   //  lift big go
byte endPress[8] = {0x15, 0x21, 0x06, 0xCA, 0x80, 0x03, 0x00, 0x00}; //  press big end
byte endLift[8] = {0x15, 0x21, 0x06, 0xCA, 0x00, 0x04, 0x00, 0x00};  //  lift big end
// byte goPress[8]        = {0x15, 0x22, 0x06, 0xCA, 0x80, 0x01, 0x00, 0x00} ;    //  press little go
// byte goLift[8]         = {0x15, 0x22, 0x06, 0xCA, 0x00, 0x02, 0x00, 0x00} ;    //  lift little go
// byte endPress[8]       = {0x15, 0x23, 0x06, 0xCA, 0x80, 0x03, 0x00, 0x00} ;    //  press little end
// byte endLift[8]        = {0x15, 0x23, 0x06, 0xCA, 0x00, 0x04, 0x00, 0x00} ;    //  lift little end

// byte csm1Press[8]        = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A} ; // CLAAS CSM1 button press Stage5 tractors
// byte csm2Press[8]        = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22} ; // CLAAS CSM2 button press Stage5 tractors

byte csm1Press[8] = {0xF1, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x67}; // CLAAS CSM1 button press pre MR tractors
byte csm2Press[8] = {0xF4, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F}; // CLAAS CSM2 button press pre MR tractors

uint16_t setCurve = 32128; // Variable for Set Curve to CAN
uint16_t estCurve = 32128; // Variable for WAS from CAN
int16_t FendtEstCurve = 0; // Variable for WAS from CAN (Fendt Only)
int16_t FendtSetCurve = 0; // Variable for Set Curve to CAN CAN (Fendt Only)

// WAS Calabration
float inputWAS[] = {-50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0}; // Input WAS do not adjust
float outputWAS[] = {-50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0};
float outputWASFendt[] = {-60.00, -54.0, -48.0, -42.3, -36.1, -30.1, -23.4, -17.1, -11.0, -5.5, 0, 5.5, 11.0, 17.1, 23.4, 30.1, 36.1, 42.3, 48.0, 54.0, 60.0}; // Fendt 720 SCR, CPD = 80

uint8_t steeringValveReady = 0; // Variable for Steering Valve State from CAN
boolean intendToSteer = 0;      // Do We Intend to Steer?

//----Teensy 4.1 CANBus--End-----------------------

// Main loop time variables in microseconds
const uint16_t LOOP_TIME = 40; // 25Hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

// IMU data
const uint16_t GYRO_LOOP_TIME = 20; // 50Hz IMU
uint32_t lastGyroTime = GYRO_LOOP_TIME;
uint32_t IMU_currentTime;

bool blink;

// IMU data
float roll = 0;
float pitch = 0;
float yaw = 0;

// GPS Data
bool sendGPStoISOBUS = false;
double pivotLat, pivotLon, fixHeading, pivotAltitude;
float utcTime, geoidalGGA;
uint8_t fixTypeGGA, satsGGA;
float hdopGGA, rtkAgeGGA;

uint8_t N2K_129029_Data[48];

// Swap BNO08x roll & pitch? - Note this is now sent from AgOpen

elapsedMillis bnoTimer;
bool bnoTrigger = false;

// booleans to see what mode BNO08x
bool useBNO08x = false;

// BNO08x address
uint8_t bno08xAddress = 0x4A;
BNO080 bno08x;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

// Parsing PGN
bool isPGNFound = false, isHeaderFound = false;
uint8_t pgn = 0, dataLength = 0, idx = 0;
int16_t tempHeader = 0;

// show life in AgIO - v5.5
uint8_t helloAgIO[] = {0x80, 0x81, 0x7f, 0xC7, 1, 0, 0x47};
uint8_t helloCounter = 0;

// Heart beat hello AgIO - v5.6
uint8_t helloFromIMU[] = {128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71};
uint8_t helloFromAutoSteer[] = {128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71};
int16_t helloSteerPosition = 0;

// fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t AOG[] = {0x80, 0x81, 0x7f, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int16_t AOGSize = sizeof(AOG);

// fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = {0x80, 0x81, 0x7f, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
uint8_t pressureReading;
uint8_t currentReading;

// EEPROM
int16_t EEread = 0;

// Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

// Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

// On Off
uint8_t guidanceStatus = 0;
uint8_t previousStatus = 0;

// speed sent as *10
float gpsSpeed = 0;

// steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; // the desired angle from AgOpen
int16_t steeringPosition = 0; // from steering sensor
float steerAngleError = 0;    // setpoint - actual

// pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

// Steer switch button
uint8_t currentState = 1, reading, previous = 0;
bool encEnable = false; // debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

// Variables for settings
struct Storage
{
  uint8_t Kp = 15;    // proportional gain
  uint8_t lowPWM = 5; // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 1;
  uint8_t highPWM = 250; // max PWM value
  float steerSensorCounts = 80;
  float AckermanFix = 1; // sent as percent
};
Storage steerSettings; // 11 bytes

// Variables for settings - 0 is false
struct Setup
{
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0; // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0; // 1 if switch selected
  uint8_t SteerButton = 0; // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 1; // 0 = use X Axis, 1 = use Y axis
};
Setup steerConfig; // 9 bytes

// Variables for config - 0 is false - Machine Config
struct Config
{
  uint8_t raiseTime = 2;
  uint8_t lowerTime = 4;
  uint8_t enableToolLift = 0;
  uint8_t isRelayActiveHigh = 0; // if zero, active low (default)
};
Config aogConfig; // 4 bytes

elapsedMillis elapsedGPSLED;
elapsedMillis elapsedIMULED;
#ifdef useLED
#define LED_TEENSY 13 // Teensy conversation with AOG
#define LED_GPS 28    // GPS status
#define LED_IMU 29    // IMU status
#endif

// TM171 stuff
constexpr int serial_buffer_size = 512;
// roll moyenne flottante
#include "RunningAverage.h"
RunningAverage myRA(7);
int samples = 0;
float avg = 0;
elapsedMillis TM171lastData;
elapsedMillis imuTimer;
bool imuTrigger = false;
bool useTM171 = false;
#include "TM171.ino"

//*******************************************************************************
// #include "zHandlers.ino"
#include "CAN_All_Brands.ino"
#include "CAN_Service_Tool.ino"
#include "j1939.ino"
#include "GPS.ino"
#include "_Utils.ino"
#include "AutosteerPID.ino"
#include "UDPReceive.ino"
#include "Relays.ino"

// Rob Tillaart, https://github.com/RobTillaart/MultiMap
// error on next line re "error 't' does not name a type? This line should be as one
// AUTOFORMAT WILL BREAK THIS
// template <typename T> T multiMap(T value, T *_in, T *_out, uint8_t size)
template <typename T> T multiMap(T value, T *_in, T *_out, uint8_t size)
{
  // take care the value is within range
  // value = constrain(value, _in[0], _in[size-1]);
  if (value <= _in[0])
    return _out[0];
  if (value >= _in[size - 1])
    return _out[size - 1];

  // search right interval
  uint8_t pos = 1; // _in[0] already tested
  while (value > _in[pos])
    pos++;

  // this will handle all exact "points" in the _in array
  if (value == _in[pos])
    return _out[pos];

  // interpolate in the right segment for the rest
  return (value - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}
void imuHandler(); // forward declaration

IntervalTimer ledIMUTimer;
IntervalTimer ledGPSTimer;

#ifdef useLED
void ledBlinkIMUISR()
{
  if (elapsedIMULED < 500)
  {
    digitalWrite(LED_IMU, !digitalRead(LED_IMU));
    elapsedIMULED = 0;
  }
}
void ledBlinkGPSISR()
{
  if (elapsedGPSLED < 500)
  {
    digitalWrite(LED_GPS, !digitalRead(LED_GPS));
    elapsedGPSLED = 0;
  }
}
#endif

/////////////////////////////// SETUP /////////////////////////////////////

void setup()
{
  delay(1000);              // Small delay so serial can monitor start up
  set_arm_clock(450000000); // Set CPU speed to 450mhz
  Serial.print("CPU speed set to: ");
  Serial.println(F_CPU_ACTUAL);

  pinMode(STEER_RELAY_5V, OUTPUT);
  digitalWrite(STEER_RELAY_5V, LOW);

#ifdef useLED
  pinMode(LED_TEENSY, OUTPUT);
#endif

  Serial.begin(115200);

  delay(2000);

#ifdef useLED
  ledIMUTimer.begin(ledBlinkIMUISR, 500000); // 100us interval
  ledGPSTimer.begin(ledBlinkGPSISR, 500000); // 100us interval
#endif

  // Check BNO08x IMU
  uint8_t error;
  Serial.println("Checking for BNO08x IMU");
  Wire.begin();
  Wire.beginTransmission(bno08xAddress);
  error = Wire.endTransmission();

  if (error == 0)
  {
    Serial.println("Error = 0");
    Serial.print("BNO08X ADDRESs: 0x");
    Serial.println(bno08xAddress, HEX);
    Serial.println("BNO08X Ok.");

    // Initialize BNO080 lib
    if (bno08x.begin(bno08xAddress))
    {
      Wire.setClock(400000); // Increase I2C data rate to 400kHz

      delay(300);

      // Use GameRotationVector
      bno08x.enableGameRotationVector(GYRO_LOOP_TIME);

      useBNO08x = true;
    }
    else
    {
      Serial.println("BNO08x not detected at given I2C address.");
    }
  }
  else
  {
    Serial.println("Error = " + String(error));
    Serial.println("i2c BNO08x not Connected or Found");
    Serial.println("Giving it some time for TM171 to stabilise (if present)");
    delay(2000);
    Serial.println("Checking for TM171");
    TM171setup();
    delay(1000);

    TM171process();
    if (TM171lastData <= 80)
    {
      Serial.println("Received data from TM171");
      imuHandler();
      useTM171 = true;
    }
    else
    {
      Serial.println("No fresh data from TM171 - you have powered it with the jumper, haven't you?????");
    }
  }

  EEPROM.get(0, EEread); // read identifier

  if (EEread != EEP_Ident) // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(6, aogConfig); // Machine
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, networkAddress);
    EEPROM.update(70, Brand);
    EEPROM.update(72, gpsMode);
    // EEPROM.put(80, outputWAS);
  }
  else
  {
    EEPROM.get(6, aogConfig);      // Machine
    EEPROM.get(10, steerSettings); // read the Settings
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, networkAddress);
    // EEPROM.get(80, outputWAS);
    Brand = EEPROM.read(70);
    gpsMode = EEPROM.read(72);
  }

  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;

  //----Teensy 4.1 Ethernet--Start---------------------

  delay(500);

  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("\r\nEthernet cable is not connected - Who cares we will start ethernet anyway.");
  }

  Ethernet.begin(mac, 0); // Start Ethernet with IP 0.0.0.0

  // grab the ip from EEPROM
  ip[0] = networkAddress.ipOne;
  ip[1] = networkAddress.ipTwo;
  ip[2] = networkAddress.ipThree;

  ipDestination[0] = networkAddress.ipOne;
  ipDestination[1] = networkAddress.ipTwo;
  ipDestination[2] = networkAddress.ipThree;

  Ethernet.setLocalIP(ip); // Change IP address to IP set by user
  Serial.println("\r\nEthernet status OK");
  Serial.print("IP set Manually: ");
  Serial.println(Ethernet.localIP());

  Udp.begin(localPort);
  NtripUdp.begin(NtripPort);

  GPS_setup();

  //----Teensy 4.1 Ethernet--End---------------------

  //----Teensy 4.1 CANBus--Start---------------------

  Serial.println("\r\nStarting CAN-Bus Ports");
  if (Brand == 0)
    Serial.println("Brand = Claas (Set Via Service Tool)");
  else if (Brand == 1)
    Serial.println("Brand = Valtra / Massey (Set Via Service Tool)");
  else if (Brand == 2)
    Serial.println("Brand = CaseIH / New Holland (Set Via Service Tool)");
  else if (Brand == 3)
    Serial.println("Brand = Fendt SCR,S4,Gen6 (Set Via Service Tool)");
  else if (Brand == 4)
    Serial.println("Brand = JCB (Set Via Service Tool)");
  else if (Brand == 5)
    Serial.println("Brand = FendtOne (Set Via Service Tool)");
  else if (Brand == 6)
    Serial.println("Brand = Lindner (Set Via Service Tool)");
  else if (Brand == 7)
    Serial.println("Brand = AgOpenGPS (Set Via Service Tool)");
  else if (Brand == 8)
    Serial.println("Brand = Cat MT Late (Set Via Service Tool)");
  else if (Brand == 9)
    Serial.println("Brand = Cat MT Early (Set Via Service Tool)");
  else
    Serial.println("No Tractor Brand Set, Set Via Service Tool");

  Serial.println("\r\nGPS Mode:");
  if (gpsMode == 1)
    Serial.println("GPS Forwarding @ 115200 (Set Via Service Tool)");
  else if (gpsMode == 2)
    Serial.println("GPS Forwarding @ 460800 (Set Via Service Tool)");
  else if (gpsMode == 3)
    Serial.println("Panda Mode @ 115200 (Set Via Service Tool)");
  else if (gpsMode == 4)
    Serial.println("Panda Mode @ 460800 (Set Via Service Tool)");
  else
    Serial.println("No GPS mode selected - Set Via Service Tool");

  delay(3000);
  CAN_setup(); // Run the Setup void (CAN page)
  if ((!useBNO08x) && (!useTM171))
    sendHardwareMessage("No IMU found", 2);

  //----Teensy 4.1 CANBus--End---------------------

  Serial.print(inoVersion);
  Serial.println("\r\nSetup complete, waiting for AgOpenGPS");
  Serial.println("\r\nTo Start AgOpenGPS CANBUS Service Tool Enter 'S'");
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
#ifdef useLED
  pinMode(LED_IMU, OUTPUT);
  digitalWrite(LED_IMU, LOW);
  pinMode(LED_GPS, OUTPUT);
  digitalWrite(LED_GPS, LOW);
  pinMode(LED_TEENSY, OUTPUT);
  digitalWrite(LED_TEENSY, LOW);
#endif
}
// End of Setup

/////////////////////////////// LOOP /////////////////////////////////////
void loop()
{

  currentTime = millis();

  //--Main Timed Loop----------------------------------
  if (currentTime - lastTime >= LOOP_TIME)
  {
    lastTime = currentTime;

    // reset debounce
    encEnable = true;

    // If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250)
      watchdogTimer = WATCHDOG_FORCE_VALUE;

    // read all the switches
    workSwitch = digitalRead(WORKSW_PIN); // read work switch (PCB pin)
    if (workCAN == 1)
      workSwitch = 0; // If CAN workswitch is on, set workSwitch ON

    // Engage steering via 1 PCB Button or 2 Tablet or 3 CANBUS

    // 1 PCB Button pressed?
    reading = digitalRead(STEERSW_PIN);
    // 2 Has tablet button been pressed?
    if (previousStatus != guidanceStatus)
    {
      if (guidanceStatus == 1) // Must have changed Off >> On
      {
        Time = millis();
        engageCAN = 1;
        relayTime = ((millis() + 1000));

        currentState = 1;
      }
      else
      {
        currentState = 1;
        steerSwitch = 1;
      }

      previousStatus = guidanceStatus;
    }

    // 3 Has CANBUS button been pressed?
    if (engageCAN == 1)
      reading = 0; // CAN Engage is ON (Button is Pressed)

    // Arduino software switch code
    if (reading == LOW && previous == HIGH)
    {
      if (currentState == 1)
      {
        if (Brand == 3)
          steeringValveReady = 16; // Fendt Valve Ready To Steer
        if (Brand == 5)
          steeringValveReady = 16; // FendtOne Valve Ready To Steer
        currentState = 0;
        steerSwitch = 0;
      }
      else
      {
        currentState = 1;
        steerSwitch = 1;
      }
    }
    previous = reading;

    //--------CAN CutOut--------------------------
    if (steeringValveReady != 20 && steeringValveReady != 16)
    {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = HIGH;
    }
    remoteSwitch = digitalRead(REMOTE_PIN); // read auto steer enable switch open = 0n closed = Off
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); // put remote in bit 2
    switchByte |= (steerSwitch << 1);  // put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    // get steering position

    // DETERMINE ACTUAL STEERING POSITION  *********From CAN-Bus************
    if (Brand == 7)
    {
    }
    else
    {
      if (intendToSteer == 0)
        setCurve = estCurve; // Not steering so setCurve = estCurve
      steeringPosition = (setCurve - 32128 + steerSettings.wasOffset);
      if (Brand == 3)
        steerAngleActual = (float)(steeringPosition) / (steerSettings.steerSensorCounts * 10); // Fendt Only
      else if (Brand == 5)
        steerAngleActual = (float)(steeringPosition) / (steerSettings.steerSensorCounts * 10); // Fendt Only
      else
        steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    }
    // Serial.print("Steering Angle Actual: ");
    // Serial.println(steerAngleActual);
    // Ackerman fix
    if (steerAngleActual < 0)
      steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    // Map WAS
    float mappedWAS;
    if (Brand == 3)
      mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWASFendt, 21);
    else if (Brand == 5)
      mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWASFendt, 21);
    else
      mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWAS, 21);
    steerAngleActual = mappedWAS;

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      // We are good to steer
      digitalWrite(STEER_RELAY_5V, 1);

      steerAngleError = steerAngleActual - steerAngleSetPoint; // calculate the steering error
      // if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      if (Brand != 7)
      {
        calcSteeringPID(); // do the pid
        motorDrive();      // out to motors the pwm value
      }
      intendToSteer = 1; // CAN Curve Inteeded for Steering
    }
    else
    {
      // we've lost the comm to AgOpenGPS, or just stop request
      //****** If CAN engage is ON (1), don't turn off saftey valve ******
      if (engageCAN == 0)
      {
        digitalWrite(STEER_RELAY_5V, 0);
      }

      intendToSteer = 0; // CAN Curve NOT Inteeded for Steering
      if (Brand != 7)
      {
        pwmDrive = 0; // turn off steering motor
        motorDrive(); // out to motors the pwm value
      }
    }

    //-------CAN Set Curve ---------------

    VBus_Send();

    //-------CAN Hitch Control---------------

    if (Brand == 3)
      SetRelaysFendt(); // If Brand = Fendt run the hitch control bottom of this page
    if (Brand == 0)
      SetRelaysClaas(); // If Brand = Claas run the hitch control bottom of this page

    // send empty pgn to AgIO to show activity
    if (++helloCounter > 10)
    {
      Udp.beginPacket(ipDestination, AOGPort);
      Udp.write(helloAgIO, sizeof(helloAgIO));
      Udp.endPacket();
      helloCounter = 0;
    }
  } // end of main timed loop

  // This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense, CAN data etc
  // delay(1);

  //--CAN--Start--
  VBus_Receive();
  ISO_Receive();
  K_Receive();

  if ((millis()) > relayTime)
  {
    engageCAN = 0;
  }

  // Service Tool
  if (Serial.available())
  { // Read Data From Serial Monitor
    byte b = Serial.read();

    while (Serial.available())
    {
      Serial.read(); // Clear the serial buffer
    }

    if (b == 'S')
    {
      Service = 1;
      Service_Tool();
    }
  }

  //--CAN--End-----

  //**GPS**
  if (useTM171)
  {
    TM171process();
    if (imuTimer > 70 && imuTrigger)
    {
      imuTrigger = false;
      imuHandler();
    }
  }
  else if (useBNO08x)
  {
    Read_IMU();
  }

  if (gpsMode == 1 || gpsMode == 2)
  {
    Forward_GPS();
  }
  else
  {
    Panda_GPS();
  }

  Forward_Ntrip();

  // Check for UDP Packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // Serial.println("UDP Data Avalible");
    udpSteerRecv(packetSize);
  }

} // end of main loop

//********************************************************************************
