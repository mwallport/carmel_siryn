#ifndef __CARMEL_SIRYN__
#define __CARMEL_SIRYN__
#include <LiquidCrystal.h>
#include <Adafruit_MAX31865.h>
#include <DS3231.h>
#include <Wire.h>
#include <SHTSensor.h>
#include "controlProtocol.h"
#include "polySci.h"
#include "common.h"
#include "handler.h"
#include "ctrlr_commands.h"
#include "deviceHandler.h"
#include "eventlog.h"
#include "events.h"



// this is for important, error condition debug output
#define __DEBUG_VIA_SERIAL__

// this is for frivilous debug output
//#define __DEBUG2_VIA_SERIAL__

//#define __DEBUG_RTD_READS__

// peripheral component speeds
#define CONTROL_PROTO_SPEED   19200
#define CHILLER_PROTO_SPEED   9600
#define RS485_MOD_BUS_SPEED   19200


// timeout for wait for menu command
#define CTRL_TIMEOUT          250


// millis() between DDR RTD samples while in RUNNING state
unsigned long timeBetweenSamples;


// uncomment for the siryn project as it will use chiller
//#define __USING_CHILLER__

// uncomment to use the Humidity sensor in the software
//#define __USING_HUMIDITY__

//
// these are the RS485 bus Ids of the entities on the bus
// need to match the actual physical assignments
//
// the RS485 for each of the accuthermos
// assuming ASIC will be RS485 id 1, DDR will be RS485 id 2
#define   ASIC_RS485_ID   1
#define   DDR_RS485_ID    2

// index of the accuthermo in sysState ACU array - ease of display the lcd output
#define   ASIC_ACU_IDX    0
#define   DDR_ACU_IDX     1


//
// LCD display
//
const int rs = 8, en = 7, d4 = 11, d5 = 12, d6 = 13, d7 = 42;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


DS3231* rtc_clock;  // need the Wire.h library loaded and Wire.begin() before instantiating clock

//
// control PC communication
// assuming they will be address 0 and this program will be address 1
//
controlProtocol cp(Serial1, 1, 0);  // slave address (arduino) is 1, control address is 0
                  // hard coded to use Serial1 

//
// chiller communication
//
#if defined(__USING_CHILLER__)
polySci chiller(9600);  // hard programmed to use Serial2 SERIAL_8N1
#define CHILLER_BUFF_LEN  10
char chiller_buff[CHILLER_BUFF_LEN + 1];
#endif

#if defined(__USING_HUMIDITY__)
SHTSensor*  p_sht;  // need the Wire.h library loaded and Wire1.begin() before instantiating SHT
#endif

#define GET_HUMIDITY_INTERVAL   2000
#define HUMIDITY_THRESHOLD      80
#define HUMIDITY_BUFFER         10


//
// modbus communication
//
static const uint8_t MAX_BUFF_LENGTH_MODBUS = 255;
uint8_t tx_buff[MAX_BUFF_LENGTH_MODBUS];
uint8_t rx_buff[MAX_BUFF_LENGTH_MODBUS];

deviceHandler RS485Bus(Serial3, tx_buff, MAX_BUFF_LENGTH_MODBUS, rx_buff, MAX_BUFF_LENGTH_MODBUS);


//
// Adafruit RTDs - do we have enough pins for all these ?
//
Adafruit_MAX31865 DDR1_RTD(4);                      // #1
Adafruit_MAX31865 DDR2_RTD(52);                     // #2
Adafruit_MAX31865 ASIC_Chiller_RTD(10);             // #3
Adafruit_MAX31865 DDR_Chiller_RTD(26, 24, 22, 34);  // #4 - not on the SPI bus

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


#define HZ_POWER_SWITCHPIN        38

// for #1
#define RTD_DDR1_ISR_PIN          32
// for #2
//#define RTD_DDR2_ISR_PIN          2   
// for #3
//#define ASIC_Chiller_RTD_ISR_PIN  30
// for #4
//#define DDR_Chiller_RTD_ISR_PIN   28

volatile bool RTD_DDR1_DRDY   = false;
volatile unsigned long  RTD_DDR1_DRDY_StartTime = 0;
unsigned long DRDY_GUARD_TIMER  = 7000;

//
// contants
//
unsigned long  status_interval;
#define GET_STATUS_INTERVAL   5000
#define GET_STATUS_INTERVAL_RUNNING 3500

#define BUTTON_PERIOD         250
#define BUTTON_COUNT_FOR_AT1  5000
//#define PIN_HW_ENABLE_n       8
//#define SWITCH_PIN            9
//#define MAX_BUFF_LENGHT       10
#define MAX_ACU_ADDRESS       2
#define MIN_ACU_ADDRESS       1
#define MAX_SHUTDOWN_ATTEMPTS 1
//#define MAX_RTD_ADDRESS       7


#define SYSTEM_NRML_OFFSET    0   // 2 msgs, good and bad
#define SYSTEM_FAIL_OFFSET    1   // 2 msgs, good and bad
#define ACU_NRML_OFFSET       2   // 2 msgs, good and bad
#define ACU_FAIL_OFFSET       3   // 2 msgs, good and bad
#define ASIC_RTD_NRML_OFFSET  4
#define ASIC_RTD_FAIL_OFFSET  5
#define DDR_RTD_NRML_OFFSET   6
#define DDR_RTD_FAIL_OFFSET   7

//#if defined(__USING_CHILLER__)
#define CHILLER_NRML_OFFSET   8   // 2 msgs, good and bad
#define CHILLER_FAIL_OFFSET   9   // 2 msgs, good and bad
#define HUMIDITY_NRML_OFFSET    10   // 2 msgs, good and bad
#define HUMIDITY_FAIL_OFFSET    11  // 2 msgs, good and bad
#define MAX_LCD_MSGS            (HUMIDITY_FAIL_OFFSET + 1)

//#else
//#define MAX_LCD_MSGS      (DDR_RTD_FAIL_OFFSET + 1)
//#endif
#define MAX_MSG_DISPLAY_TIME  4000  // 1.5 minimum seconds per message

// move this - and ask Rick what the correct initial settings should be
// high RTD chiller temperature, if hit this, go to SHUTDOWN state
// and don't start if RTD chiller temperature is this
float ASIC_HIGH   = 35.5;
float DDR_HIGH    = 35.5;


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

//#if defined(__USING_CHILLER__)
  // chiller
  chiller_Running, chiller_Stopped, chiller_ComFailure,
//#endif
  sensor_humidityAndThreshold,
  sensor_HighHumidity, sensor_Failure,
  
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

//#if defined(__USING_CHILLER__)
// chiller
void lcd_chillerRunning();    //set point and current temp
void lcd_chillerStopped();    // running or stopped or fail
void lcd_chillerComFailure();   // can't communicate with the chiller
//#endif

// sensor
void lcd_humidityAndThreshold();    // current humidity and threshold
void lcd_highHumidity();    // humidity alert, or mechanical failure
void lcd_sensorFailure();   // unable to communicate with the sensor


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
//#if defined(__USING_CHILLER__)
  lcd_chillerRunning, // running - pump is on, etc. and temps
  lcd_chillerStopped, // not running - pump is off and temps
  lcd_chillerComFailure,  // can't communicate with the chiller
//#endif
  lcd_humidityAndThreshold, // normal humidity
  lcd_highHumidity,   // high humidity
  lcd_sensorFailure
};


//
// running status for components - updated by getStatus and set commands
//
typedef enum { offline, online, running, stopped, shutdown } runningStates;

//
// system status
//
typedef enum { SHUTDOWN, READY, RUNNING, UNKNOWN } systemStatus;

//#if defined(__USING_CHILLER__)
typedef struct _chillerState
{
  runningStates online;         // online or offline
  runningStates state;          // running or stopped
  runningStates prior_online;   // eventlog guard
  runningStates prior_state;    // eventlog guard
  float         setpoint;       // current set point temperature, the SV
  float         temperature;    // current temperature, the PV
} chillerState;
//#endif

const int MAX_HUMIDITY_SAMPLES  = 6;
typedef struct _humiditySamples
{
    int    index;
    float  sample[MAX_HUMIDITY_SAMPLES];
} humiditySamples_t;

typedef struct _humidityState
{
    runningStates       online;
    runningStates       prior_online; // eventlog guard
    float               prior_humidity; // guard against logging same humidity event
    float               humidity;
    uint16_t            threshold;
    humiditySamples_t   sampleData;
} humidityState;

typedef struct _ACUState
{
  runningStates   online;       // online or offline
  runningStates   state;        // running or stopped
  runningStates   prior_online; // event log guard
  runningStates   prior_state;  // event log guard
  float           setpoint;     // current set point temperature ( SV )
  float           temperature;  // current temperature ( PV )
} ACUState;

typedef struct _RTDState
{
  runningStates   online;       // online or offline
  runningStates   state;        // running or stopped
  uint8_t         prior_fault;  // tightly coupled to adafruit .. boo .. can display on the LCD fault face which could be good-ish ..?
  uint8_t         fault;        // tightly coupled to adafruit .. boo .. can display on the LCD fault face which could be good-ish ..?
  uint16_t        rtd;          // same same - tightly coupled
  float           prior_temperature; // event log guard
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
//#if defined (__USING_CHILLER__)
  chillerState  chiller;
//#endif
  humidityState sensor;
  ACUState    ACU[MAX_ACU_ADDRESS];
  RTDState    ASIC_RTD;             // ASIC chip temp - this RTD is connected to ASIC Accutermo - can't get fault
  RTDState    ASIC_Chiller_RTD;     // ASIC chiller temp - fault is reflected in status menu cmd
  RTDState    DDR_RTD;              // DDR chip temp - this RTD is connected to the DDR Accuthermo - can't get fault
  RTDState    DDR1_RTD;             // 2rd DDR chip temp - fault is reflected in status menu cmd
  RTDState    DDR2_RTD;             // 2rd DDR chip temp - fault is reflected in status menu cmd
  RTDState    DDR_Chiller_RTD;      // DDR chiller temp - fault is reflected in status menu cmd
  float       highRTDTemp;          // will be the temperature of the hottest RTD in the DDR group
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
const int FAULT_LED     = 36;
const int NO_FAULT_LED  = 53;


//
// configure the start/stop button
//
// The pin number attached to the button.
const int BUTTON_PIN = 5;
const int BUTTON_LED = 6;
bool currentButtonOnOff = false;
static unsigned long  buttonLastInterruptTime = 0;
volatile bool buttonOnOff = false;
volatile int bp_count = -2;  // button press 1
#define   LONG_PRESS_BP_COUNT   1500000



//
// splash screen conents - shown during boot while the system is coming on-line
//
const char deftDevise[16] = "deftDevise   ";
const char buildInfo[16]  = "220117       ";

#endif
