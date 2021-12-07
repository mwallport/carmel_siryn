#ifndef __CARMEL_SIRYN__
#define __CARMEL_SIRYN__

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <LiquidCrystal.h>    // LCD interface library
#include <Adafruit_MAX31865.h>
#include <controlProtocol.h>
#include <polySci.h>        // polySci chiller communication library
#include "common.h"
#include "handler.h"
#include "ctrlr_commands.h"
#include "deviceHandler.h"



//#define __DEBUG_MODBUS_CMDS__
//#define __DEBUG_MODBUS_TXRX__


// this is for important, error condition debug output
#define __DEBUG_VIA_SERIAL__

// this is for frivilous debug output
//#define __DEBUG2_VIA_SERIAL__

// initial PVOF value
#define __INIT_PVOF__  0x01FA

//
// uncomment for the siryn project
//
//#define __USING_CHILLER__

//
// these are the RS485 bus Ids of the entities on the bus
// need to match the actual physical assignments
//
uint8_t   ASIC_ID     = 1;
uint8_t   DDR_ID      = 2;
uint8_t   RTD_MUX_ID  = 3;
uint32_t  val;


//
// modbus communication
//
static const uint8_t MAX_BUFF_LENGTH_MODBUS = 255;
uint8_t tx_buff[MAX_BUFF_LENGTH_MODBUS];
uint8_t rx_buff[MAX_BUFF_LENGTH_MODBUS];

deviceHandler rs485Bus(Serial3, tx_buff, MAX_BUFF_LENGTH_MODBUS, rx_buff, MAX_BUFF_LENGTH_MODBUS);


//
// huber chiller communication
//
#if defined(__USING_CHILLER__)
polySci chiller(9600);  // hard programmed to use Serial2 SERIAL_8N1
#endif

//
// control PC communication
// assuming they will be address 0 and this program will be address 1
//
controlProtocol cp(1, 0, 57600);  // slave address (arduino) is 1, control address is 0
                  // hard coded to use Serial1 

//
// Adafruit RTDs - do we have enough pins for all these ?
//
// TODO: set these up w/ the correct pins
//
Adafruit_MAX31865 ASIC_RTD          = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 ASIC_Chiller_RTD  = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 DDR1_RTD          = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 DDR2_RTD          = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 DDR3_RTD          = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 DDR_Chiller_RTD   = Adafruit_MAX31865(10, 11, 12, 13);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


//
// constants - using #define - have limited space on Arduino
//
#define GET_STATUS_INTERVAL   10000
#define GET_HUMIDITY_INTERVAL 2500
#define BUTTON_PERIOD         5000
#define HUMIDITY_THRESHOLD    80
#define HUMIDITY_BUFFER       10
#define PIN_HW_ENABLE_n       8
#define SWITCH_PIN            9
#define MAX_BUFF_LENGHT       10
#define MAX_ACU_ADDRESS       2
#define MIN_ACU_ADDRESS       1
#define MAX_SHUTDOWN_ATTEMPTS 1
#define MAX_RTD_ADDRESS       7


#define SYSTEM_NRML_OFFSET    0   // 2 msgs, good and bad
#define SYSTEM_FAIL_OFFSET    1   // 2 msgs, good and bad
#define ACU_NRML_OFFSET       2   // 2 msgs, good and bad
#define ACU_FAIL_OFFSET       3   // 2 msgs, good and bad
#define ASIC_RTD_NRML_OFFSET  4
#define ASIC_RTD_FAIL_OFFSET  5
#define DDR_RTD_NRML_OFFSET   6
#define DDR_RTD_FAIL_OFFSET   7
#if defined(__USING_CHILLER__)
#define CHILLER_NRML_OFFSET   8   // 2 msgs, good and bad
#define CHILLER_FAIL_OFFSET   9   // 2 msgs, good and bad
#define MAX_LCD_MSGS      (CHILLER_FAIL_OFFSET + 1)
#else
#define MAX_LCD_MSGS      (DDR_RTD_FAIL_OFFSET + 1)
#endif
#define MAX_MSG_DISPLAY_TIME  1500  // 1.5 minimum seconds per message



//
// functions to paint the LCD screen
//
// enums to be indexes into the lcdDisplay array
//
enum {
  // no status to display
  no_Status = 0,
  
  // sys LCD status
  sys_Initializing, sys_Ready, sys_Running, sys_Shutdown, sys_StartFailed, sys_Failure,

  // ACU status
  ACU_Stopped, ACU_Running, ACU_ComFailure,

  // ASIC RTD status
  ASIC_RTD_Running, ASIC_RTD_Failure,

  // DDR RTD status
  DDR_RTD_Running, DDR_RTD_Failure,

#if defined(__USING_CHILLER__)
  // chiller
  chiller_Running, chiller_Stopped, chiller_ComFailure,
#endif
  MAX_LCD_FUNC
};

// sys
void lcd_initializing();  // power-on, LCD splash screen
void lcd_ready();         // system was started
void lcd_running();       // system was started
void lcd_shutdown();      // system was shutdown
void lcd_startFailed();   // not all devices present or something
void lcd_systemFailure(); // some run-time fail, loss of comm w/ device, etc.

// ACU
void lcd_ACUsStopped();   // set point and current temp
void lcd_ACUsRunning();   // set point and current temp
void lcd_ACUComFailure(); // set point and current temp

// ASIC RTDs
void lcd_ASIC_RTDs_Running();
void lcd_ASIC_RTDs_Failure();

// DDR RTDs
void lcd_DDR_RTDs_Running();
void lcd_DDR_RTDs_Failure();

#if defined(__USING_CHILLER__)
// chiller
void lcd_chillerRunning();    //set point and current temp
void lcd_chillerStopped();    // running or stopped or fail
void lcd_chillerComFailure();   // can't communicate with the chiller
#endif


typedef void (*lcdFunc)(void);   

lcdFunc lcdFaces[MAX_LCD_FUNC] = 
{
  0,          // no_Status to show
  lcd_initializing,   // system is starting
  lcd_ready,      // started and waiting for startUp command
  lcd_running,    // system is running
  lcd_shutdown,     // shutdown has been done either implicitely or explicitely
  lcd_startFailed,  // not all devices present upon startup
  lcd_systemFailure,  // some run-time failure - check ACU, chiller, or humidity status
  lcd_ACUsStopped,  // stopped and 3 set points 
  lcd_ACUsRunning,  // running and 3 set points
  lcd_ACUComFailure,  // can't communicate with one of the ACUs asterisks and which ACU
  lcd_ASIC_RTDs_Running,
  lcd_ASIC_RTDs_Failure,
  lcd_DDR_RTDs_Running,
  lcd_DDR_RTDs_Failure,
#if defined(__USING_CHILLER__)
  lcd_chillerRunning, // running - pump is on, etc. and temps
  lcd_chillerStopped, // not running - pump is off and temps
  lcd_chillerComFailure,  // can't communicate with the chiller
#endif
};


//
// running status for components - updated by getStatus and set commands
//
typedef enum { offline, online, running, stopped, shutdown } runningStates;

//
// system status
//
typedef enum { SHUTDOWN, READY, RUNNING, UNKNOWN } systemStatus;

#if defined(__USING_CHILLER__)
typedef struct _chillerState
{
  runningStates online;         // online or offline
  runningStates state;          // running or stopped
  float         setpoint;       // current set point temperature
  float         temperature;    // current temperature
} chillerState;
#endif

typedef struct _ACUState
{
  runningStates   online;       // online or offline
  runningStates   state;        // running or stopped
  float           setpoint;     // current set point temperature
  float           temperature;  // current temperature
} ACUState;

typedef struct _RTDState
{
  runningStates   online;       // online or offline
  runningStates   state;        // running or stopped
  uint8_t         fault;        // tightly coupled to adafruit .. boo .. can display on the LCD fault face which could be good-ish ..?
  uint16_t        rtd;          // same same - tightly coupled
  float           temperature;  // current temperature
} RTDState;

typedef struct _LCDState
{
  int16_t       lcdFacesIndex[MAX_LCD_MSGS];// index into lcd faces array
  int16_t       index;            // index into lcdFacesIndex array
  unsigned long prior_millis;         // prior time in millis()
} LCDState;

typedef struct _systemState
{
#if defined (__USING_CHILLER__)
  chillerState  chiller;
#endif
  ACUState    ACU[MAX_ACU_ADDRESS];
  RTDState    ASIC_RTD;             // ASIC chip temp
  RTDState    ASIC_Chiller_RTD;     // ASIC chiller temp
  RTDState    DDR1_RTD;             // 1st DDR chip temp
  RTDState    DDR2_RTD;             // 2nd DDR chip temp
  RTDState    DDR3_RTD;             // 3rd DDR chip temp
  RTDState    DDR_Chiller_RTD;      // DDR chiller temp
  LCDState    lcd;
  systemStatus  sysStatus;
} systemState;


//
// system status - updated by getStats and by set commands from control
//
systemState sysStates;


//
// configure the fault/no-fault LED
//
const int FAULT_LED   = 4;
const int NO_FAULT_LED  = 6;


//
// configure the start/stop button
//
// The pin number attached to the button.
const int BUTTON_PIN = 3;
const int BUTTON_LED = 7;
bool currentButtonOnOff = false;
volatile bool buttonOnOff = false;


//
// LCD display
//
const int rs = 8, en = 10, d4 = 11, d5 = 12, d6 = 13, d7 = 42, rw=9;
LiquidCrystal lcd(rs, rw, en, d4, d5, d6, d7);


//
// splash screen conents - shown during boot while the system is coming on-line
//
const char deftDevise[16] = "deftDevise   ";
const char buildInfo[16]  = "210103       ";

#endif
