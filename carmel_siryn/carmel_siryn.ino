#include <SPI.h>
#include <Controllino.h>
#include <LiquidCrystal.h>    // LCD interface library
#include <HardwareSerial.h>
#include <stdio.h>
#include <inttypes.h>
#include "carmel_siryn.h"


void setup(void)
{

  //
  // start the system components and Serial port if running debug
  //
  initSystem();


  //
  // TODO: remove this
  // banner - used to easilyeasily  find system restarts in log file
  //
  #ifdef __DEBUG_VIA_SERIAL__
  Serial.println(" -----------------------------------------------------------"); Serial.flush();
  Serial.println(" -------------------------> setup() <-----------------------"); Serial.flush();
  Serial.println(" -----------------------------------------------------------"); Serial.flush();
  #endif


  //
  // always start the chiller
  //
  startChiller();
  
  //
  // get all statuses at startup
  // normally gotten via getStatus() which runs on a period
  //
  handleChillerStatus();
  handleACUStatus();
}


void loop(void)
{
  //
  // write the PVOF
  //
  uint16_t currPVOF = writePVOF(ASIC_ID, __INIT_PVOF__);

  #ifdef __DEBUG_VIA_SERIAL__
  Serial.print("PVOF read back : "); Serial.print(currPVOF, 16); Serial.println("");
  #endif

  
  //
  // getStatus will update LCD and sysStats data structure
  //
  getStatus();


  //
  // take commands from the
  // - the switch on front panel
  // - the controlling PC software
  //
  handleMsgs();


  //
  // update the LCD
  //
  manageLCD();
}



//-------------------------------------------------------------
//
//
void initSystem(void)
{
  int count = 0;


  //
  // start the Serial port if running debug
  //
  #if defined __DEBUG_VIA_SERIAL__ || defined __DEBUG2_VIA_SERIAL__
  Serial.begin(19200);
  delay(1000);
  #endif

  //
  // start the Serial3 for the RS485 bus
  //
  Controllino_RS485Init();
  Serial3.begin(19200);
  Controllino_RS485RxEnable();

  while( (!Serial3) && (count < 10) )
  {
    delay(500);
    count += 1;
  }


  if( (!Serial3) )
  {
    #if defined __DEBUG_VIA_SERIAL__ || defined __DEBUG2_VIA_SERIAL__
    Serial.println("unable to start Serial3, exiting");
    #endif
    exit(1);
  }
  

  //
  // initialize the system states /stats - these are
  // used to hold temperatures, humidity, etc. and
  // used in responses to getStatusCmd from control
  // and holds the LCD messages
  //
  initSysStates(sysStates);

  //
  // start the LCD and paint system initializing
  //
  //  startLCD();

  //
  // register the ISR for the digital encoder
  //
  //enableRotaryEncoder();

  //
  // initialize the start/stop button
  //
  configureButton();

  //
  // initialize the fault/no-fault LED(s)
  //
  configureFaultNoFault();

  //
  // let the Serial port settle after initButton()
  //
  delay(1000);
}


//
// initialize the system status data to reflect clean start up
// i.e. no failures found yet
//
void initSysStates(systemState& states)
{
#if defined(__USING_CHILLER__)
  // chiller starts offline until queried via getStatus()
  states.chiller.online    = offline;
  states.chiller.state     = stopped;
  states.chiller.temperature = 0;
  states.chiller.setpoint  = 0;
#endif


  // ACUs starts offline until discovered to be online via getStatus()
  for(int i = MIN_ACU_ADDRESS; i <= MAX_ACU_ADDRESS; i++)
  {
    states.ACU[(i - MIN_ACU_ADDRESS)].online      = offline;
    states.ACU[(i - MIN_ACU_ADDRESS)].state       = stopped;
    states.ACU[(i - MIN_ACU_ADDRESS)].setpoint    = 0;
    states.ACU[(i - MIN_ACU_ADDRESS)].temperature   = 0;
  }

  // lcd - initialize all messages
  for(int i = 0; i < MAX_LCD_MSGS; i++)
    sysStates.lcd.lcdFacesIndex[i] = 0;

  // pick up the millis() 'now' - something fishy about this, what will happen when
  // the counter rolls over ?  This program is suppossed to run for a long time..
  // TODO: find a better way - use timeofday or the RTC.. ?
  sysStates.lcd.prior_millis = millis();

  //
  // setup the 'good' lcd functions
  //
  sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Initializing;
  sysStates.lcd.lcdFacesIndex[ACU_NRML_OFFSET]     = ACU_Stopped;
#if defined(__USING_CHILLER__)
  sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]   = chiller_Stopped;
#endif
  sysStates.lcd.index = 0;
  sysStates.sysStatus = UNKNOWN;
}


void configureFaultNoFault(void)
{
  pinMode(FAULT_LED, OUTPUT);
  digitalWrite(FAULT_LED, HIGH);
  pinMode(NO_FAULT_LED, OUTPUT);
  digitalWrite(NO_FAULT_LED, LOW);
}



//
// use the global sysStates.lcd data structures to paint
// messages on the LCD
//
void manageLCD(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif

  //
  // if enough time has passed, display the current index
  //   
  if( (MAX_MSG_DISPLAY_TIME <= (millis() - sysStates.lcd.prior_millis)) )
  {
    // set up the LCD's number of columns and rows:
    lcd.noDisplay(); lcd.begin(16, 2); lcd.noDisplay(); lcd.clear(); lcd.home();

    //
    // update the LCD with the next message, if the next message
    // had been removed, advance to the next message
    //
    if( (0 == sysStates.lcd.lcdFacesIndex[sysStates.lcd.index]) ||      // no status, i.e. no fail status
      (0 == lcdFaces[sysStates.lcd.lcdFacesIndex[(sysStates.lcd.index)]]) ) // this face has been removed
    {
      //
      // find next non-zero message, adjust sysStates.lcd.index
      //
      for(int i = ((((sysStates.lcd.index) + 1) >= MAX_LCD_MSGS) ? 0 : (sysStates.lcd.index + 1));
        (i != sysStates.lcd.index);
        i = (((i + 1) >= MAX_LCD_MSGS) ? 0 : (i + 1)) )
      {
        if( (0 != sysStates.lcd.lcdFacesIndex[i]) )
        {
          //
          // updagte the index and call the LCD function
          //
          sysStates.lcd.index = i;
          break;
        }
      }
    }

    lcdFaces[sysStates.lcd.lcdFacesIndex[(sysStates.lcd.index)]]();


    //
    // reload the count down timer
    //
    sysStates.lcd.prior_millis  = millis();

    //
    // and advance the index to the next message
    //
    sysStates.lcd.index = (((sysStates.lcd.index + 1) >= MAX_LCD_MSGS) ? 0 : (sysStates.lcd.index + 1));
  }
}



// all LCD calls are void, no way to tell if the LCD is up .. 
bool startLCD(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print(deftDevise);
  lcd.setCursor(0,1);
  lcd.print(buildInfo);
  lcd.display();

  return(true);
}


// ----------------------------------------------------------
// return true only if
// - LCD is up
// - SHTSensor is up
// - Meersteters are up
// - chiller is running
//
bool startUp(void)
{
  bool retVal = true;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  //
  // paint 'Starting' on LCD
  //
  lcd_starting();


  //
  // start the chiller and the ACUs
  //
  if( !(startChiller()) )
    retVal  = false;

  // only start the ACUs is the chiller is running
  else if( !(startACUs()) )
    retVal = false;

  if( (true == retVal) )
  {
    // set the LCD to running
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Running;
  } // else leave the FacesIndex what it was .. ?

  return(retVal);
}


//
// this function updates the LCD banner w/ failure messages
// to reflect problems
//
// run every 20 seconds
//
void getStatus(void)
{
  static unsigned long  lastGetStatusTime     = 0;
  static unsigned long  lastGetHumidityTime   = 0;
  unsigned long       currentGetStatusTime  = millis();


  //
  // get the chiller and ACU's status every GET_STATUS_INTERVAL seconds
  //
  if( (GET_STATUS_INTERVAL < (currentGetStatusTime - lastGetStatusTime)) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("- check ACUs and chiller --------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif
    
    lastGetStatusTime = currentGetStatusTime;
    handleChillerStatus();
    handleACUStatus();
  }



  //
  // set the LCD state for the overall system based on the
  // gathered information and attach/detach knob interrupts as
  // needed, i.e. shutdown or not - save the priorStatus to
  // handle transition to SHUTDOWN (above)
  //
  setSystemStatus();
}


//
// shut everything down, update the system and LCD status
//
bool shutDownSys(bool stopChillerCmd)
{
  bool  retVal  = true;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  //
  // paint system shutdown on LCD
  //
  lcd_shuttingDown();

  //
  // turn off the chiller only if there is a humidity failure or stopChillerCmd is true
  //
#if defined(__USING_CHILLER__)
  if( (stopChillerCmd) )
  {
    for(uint8_t i = 0; i < MAX_SHUTDOWN_ATTEMPTS; i++)
    {
      if( !(chiller.StopChiller()) )
        retVal  = false;
      else
      {
        retVal  = true;
        break;
      }
    }
  }
#endif
      
  //
  // turn off the ACUs - not checking return value here as we are dying anyway ??
  //
  for(uint8_t Address = MIN_ACU_ADDRESS; (Address <= MAX_ACU_ADDRESS); Address++)
  {
    if( !(StopACU(Address)) )
      retVal  = false;
  }


  //
  // update the LCD to reflect system shutDown
  //
  sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]   = sys_Shutdown;

  return(retVal);
}


// ----------------------------------------------------------
// turn off echo and continuous output
// turn on the chiller
// return success if all these happen
//
bool startChiller(void)
{
#if defined(__USING_CHILLER__)

  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  bool retVal = true;



  if( !(chiller.StartChiller()) )
  {
    retVal  = false;
 
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__); Serial.println(" unable to start chiller");
    #endif
  }

  //
  // let handleChillerStatus do all the house keeping
  //
  handleChillerStatus();


  //
  // update the system state
  //
  setSystemStatus();

  return(retVal);
#else
  return(true);
#endif
}


// ---------------------------------------
// find all ACUs, there should be 3
// - verify their addresses by fetching hw version
//      expecting ID s2, 3, and 4
// - turn them 'on'
// return true if all these things happen
//
bool startACUs(void)
{
  bool  retVal    = true;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  //
  // only start the ACUs if the chiller is running
  // only in the SHUTDOWN state is the chiller not running (fucking Yoda?)
  //
#if defined (__USING_CHILLER__)
  if( (running == sysStates.chiller.state) )
  {
#endif
    //
    // expecting the addresses to be 1, 2, and 3
    // the instance for these commands to be 1
    //
    // initialize the ACU LCD state
    for(uint8_t Address = MIN_ACU_ADDRESS; Address <= MAX_ACU_ADDRESS; Address++)
    {
      if( !(StartACU(Address)) )
      {
        retVal = false;
        
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" unable to start ACU: ");
        Serial.println(Address, DEC);
        Serial.flush();
        #endif
      }
    }

    //
    // let handleACUStatus() derive the ACU's status
    //
    handleACUStatus();

    //
    // update the system state
    //
    setSystemStatus();

#if defined (__USING_CHILLER__)
  } else
  {
    retVal = false;
    
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(": not starting ACUs chiller is not running");
    Serial.flush();
    #endif
  }
#endif
  
  return(retVal);
}


bool stopACUs(void)
{
  bool  retVal    = true;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  //
  // expecting the addresses to be 2, 3, and 4
  // the instance for these commands to be 1
  //
  sysStates.lcd.lcdFacesIndex[ACU_NRML_OFFSET]   = ACU_Stopped;
  sysStates.lcd.lcdFacesIndex[ACU_FAIL_OFFSET]   = no_Status;
  for(uint8_t Address = MIN_ACU_ADDRESS; (Address <= MAX_ACU_ADDRESS); Address++)
  {
    if( !(StopACU(Address)) )
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.print("stopACUs unable to stop ACU ");
      Serial.println(Address, DEC);
      #endif

      retVal    = false;

      //
      // update the LCD state at least
      //
      sysStates.lcd.lcdFacesIndex[ACU_FAIL_OFFSET]  = ACU_ComFailure;
      sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online       = offline;
      sysStates.ACU[(Address - MIN_ACU_ADDRESS)].state        = stopped;  // we don't know, or don't change this

    } else
    {
      sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online   = online;
      sysStates.ACU[(Address - MIN_ACU_ADDRESS)].state  = stopped;
    }
  }

  return(retVal);
}

/*
bool setACUTemp(uint16_t ACUAddress, float temp)
{
  bool retVal = false;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  if( (MAX_ACU_ADDRESS >= ACUAddress) )
  {
    if( (ms.SetACUTemp(ACUAddress, temp)) )
    {
      retVal  = true;
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__);
      Serial.print(" ERROR: failed to set temp for ACU at addr: ");
      Serial.println(ACUAddress);
      Serial.flush();
    #endif
    }
  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" ERROR: failed to set temp for ACU addr out of range: ");
    Serial.println(ACUAddress);
    Serial.flush();
  #endif
  }

  return(retVal);
}
*/


bool setChillerSetPoint(char* temp)
{
#if defined (__USING_CHILLER__)
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif


  return(chiller.SetSetPoint(temp));
#else
  return(true);
#endif
}


void handleMsgs(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  if( (currentButtonOnOff != buttonOnOff) )
  {
    currentButtonOnOff = buttonOnOff;
      
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("button press happened, switching ");
    if( (true == currentButtonOnOff) ) Serial.println("on"); else Serial.println("off");
    Serial.flush();
    #endif

    if( (true == currentButtonOnOff) )
    {
      //
      // adjust the button LED
      //
      digitalWrite(BUTTON_LED, HIGH);
      startUp();
    } else
    {
      //
      // adjust the button LED
      //
      digitalWrite(BUTTON_LED, LOW);
      shutDownSys(false);
    }
  } else
  {
    //
    // check for message from control PC
    //
    if( (getMsgFromControl()) )
    {
      //
      // cp.m_buff has the message just received, process that message
      //
      switch( (cp.getMsgId()) )
      {
        case startUpCmd:        // start the chiller if present, and ACUs
        {
          handleStartUpCmd();
          break;
        }
  
        case shutDownCmd:         // shutdown the chiller, ACUs, and sensor
        {
          handleShutDownCmd();
          break;
        }
/*  
        case getStatusCmd:         // fetch the status of chiller, all ACUs, and humidity sensor
        {
          handleGetStatusCmd();
          break;
        };
  
        case setHumidityThreshold:     // get the humidity threshold
        {
          handleSetHumidityThreshold();
          break;
        };
  
        case getHumidityThreshold:     // set the humidity threshold
        {
          handleGetHumidityThreshold();
          break;
        };
  
        case getHumidity:        // get current humidity and temperature
        {
          handleGetHumidity();
          break;
        };
  
        case setACUTemperature:      // target ACU m_address and temp
        {
          handleSetACUTemperature();
          break;
        };
  
        case getACUTemperature:      // target ACU m_address and temp
        {
          handleGetACUTemperature(false);
          break;
        };
  
        case getACUObjTemperature:
        {
          handleGetACUTemperature(true);
          break;
        };
#if defined(__USING_CHILLER__)
        case startChillerMsg:
        {
          handleStartChillerMsg();
          break;
        };
  
        case stopChiller:
        {
          handleStopChiller();
          break;
        };
  
        case getChillerInfo:
        {
          handleGetChillerInfo();
          break;
        };
  
        case setChillerTemperature:    // target ACU m_address and temp
        {
          handleSetChillerTemperature();
          break;
        };
  
        case getChillerTemperature:    // target ACU m_address and temp
        {
          handleGetChillerTemperature(true);
          break;
        };
  
        case getChillerObjTemperature:    // target ACU m_address and temp
        {
          handleGetChillerTemperature(false);
          break;
        };
#endif
        case getACUInfoMsg:
        {
          handlGetACUInfo();
          break;
        };
  
        case enableACUs:         // turn on all ACUs
        {
          handleEnableACUs();
          break;
        };
  
        case disableACUs:        // turn off all ACUs
        {
          handleDisableACUs();
          break;
        };
*/
        default:
        {
          // send NACK
          sendNACK();
          break;
        }
      }
    } // else no message from control
  }
}
  

void lcd_initializing(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,1);
  lcd.print("SYS initializing");
  lcd.display();

}


void lcd_starting(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("**** SYSTEM ****");
  lcd.setCursor(0,1);
  lcd.print("*** STARTING ***");
  lcd.display();
}


void lcd_ready(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,1);
  lcd.print("SYS ready");
  lcd.display();
}


void lcd_running(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,1);
  lcd.print("SYS running");
  lcd.display();
}


void lcd_shutdown(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,1);
  lcd.print("SYS shutdown");
  lcd.display();
}


void lcd_shuttingDown(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("**** SYSTEM ****");
  lcd.setCursor(0,1);
  lcd.print("*SHUTTING DOWN *");
  lcd.display();
}


void lcd_startFailed(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,1);
  lcd.print("SYS start fail");
  lcd.display();
}


void lcd_systemFailure(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("SYS fail");
  lcd.setCursor(0,1);
  lcd.print("failure");
  lcd.display();
}


void lcd_ACUsRunning(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("ACU run     ");
  lcd.setCursor(9,0);
  lcd.print(sysStates.ACU[0].setpoint,1);
  lcd.setCursor(0,1);
  lcd.print(sysStates.ACU[1].setpoint,1);
  lcd.setCursor(9,1);
  lcd.print(sysStates.ACU[2].setpoint,1);
  lcd.display();
}


void lcd_ACUsStopped(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("ACU stop    ");
  lcd.setCursor(9,0);
  lcd.print(sysStates.ACU[0].setpoint,1);
  lcd.setCursor(0,1);
  lcd.print(sysStates.ACU[1].setpoint,1);
  lcd.setCursor(9,1);
  lcd.print(sysStates.ACU[2].setpoint,1);
  lcd.display();
}


void lcd_ACUComFailure(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("ACU COMM");
  lcd.setCursor(0,1);
  lcd.print("FAILURE");
  lcd.display();
}


#if defined(__USING_CHILLER__)
void lcd_chillerRunning(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("CHL RUNNING");
  lcd.setCursor(0,1);
  lcd.print("S: ");
  lcd.setCursor(3,1);
  lcd.print(sysStates.chiller.setpoint);
  lcd.setCursor(9,1);
  lcd.print("T: ");
  lcd.setCursor(12,1);
  lcd.print(sysStates.chiller.temperature);
  lcd.display();
}


void lcd_chillerStopped(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("CHL STOPPED");
  lcd.setCursor(0,1);
  lcd.print("S: ");
  lcd.setCursor(3,1);
  lcd.print(sysStates.chiller.setpoint);
  lcd.setCursor(9,1);
  lcd.print("T: ");
  lcd.setCursor(12,1);
  lcd.print(sysStates.chiller.temperature);
  lcd.display();
}


void lcd_chillerComFailure(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,1);
  lcd.print("CHL COM FAILURE");
  lcd.display();
}
#endif

void lcd_sensorFailure(void)
{
  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,1);
  lcd.print("SENSOR FAILURE ");
  lcd.display();
}


bool getMsgFromControl(void)
{
  bool      retVal  = false;

  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif

  //
  // receive a command, wait 1 seconds for a command
  //
  if( (cp.doRxCommand(1000)) )
  {
    retVal  = true;
  }

  return(retVal);
}



void handleStartUpCmd(void)
{
  startUpCmd_t* pstartUpCmd = reinterpret_cast<startUpCmd_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a startUpCmdCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pstartUpCmd->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_startUpCmd_t,
          ntohs(pstartUpCmd->crc), ntohs(pstartUpCmd->eop))) )
    {
      //
      // start the ACUs, chiller, sensor ...
      //
      if( (startUp()) )
      {
        result  = 1;
        
        //
        // adjust the button
        //
        buttonOnOff     = true;
        currentButtonOnOff  = buttonOnOff;

        //
        // adjust the button LED
        //
        digitalWrite(BUTTON_LED, HIGH);
        
      } else
        setSystemStatus();  // derive the button state
        

      respLength = cp.Make_startUpCmdResp(cp.m_peerAddress, cp.m_buff,
        result, pstartUpCmd->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pstartUpCmd->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pstartUpCmd->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleShutDownCmd(void)
{
  shutDownCmd_t* pshutDownCmd = reinterpret_cast<shutDownCmd_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a shutDownCmdCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pshutDownCmd->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_shutDownCmd_t,
            ntohs(pshutDownCmd->crc), ntohs(pshutDownCmd->eop))) )
    {
      //
      // call shutDown()
      //
      if( (shutDownSys(false)) )
      {
        result  = 1;
        
        //
        // adjust the button
        //
        buttonOnOff     = false;
        currentButtonOnOff  = buttonOnOff;

        //
        // adjust the button LED
        //
        digitalWrite(BUTTON_LED, LOW);
        
      } else
        setSystemStatus();  // derive the button state


      respLength = cp.Make_shutDownCmdResp(cp.m_peerAddress, cp.m_buff,
        result, pshutDownCmd->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pshutDownCmd->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pshutDownCmd->header.address.address));
    Serial.flush();
  #endif
  }
}


//
// all message handlers expect the controlProtocol object
// to have the received message
//
/*
void handleGetStatusCmd(void)
{
  getStatus_t*  pgetStatus = reinterpret_cast<getStatus_t*>(cp.m_buff);
  uint16_t    respLength; 


  //
  // verify the received packet, here beause this is a getStatusCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetStatus->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getStatus_t,
              ntohs(pgetStatus->crc), ntohs(pgetStatus->eop))) )
    {
      //
      // use the sysStates conentent to respond, send back the received seqNum
      //
      respLength = cp.Make_getStatusResp(cp.m_peerAddress, cp.m_buff,
        0, // humidity alert
        ((true == ACUsRunning()) ? 1 : 0),                  // ACUs running
        ((running == sysStates.chiller.state) ? 1 : 0),           // chiller running
        pgetStatus->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this functoin usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pgetStatus->crc), HEX);
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet: ");
    Serial.println(ntohs(pgetStatus->header.address.address));
    Serial.flush();
  #endif
  }
}



void handleSetACUTemperature(void)
{
  setACUTemperature_t* psetACUTemperature = reinterpret_cast<setACUTemperature_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  ACUAddress;
  uint16_t  result = 0;
  float     setPoint;


  //
  // verify the received packet, here beause this is a setACUTemperatureCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(psetACUTemperature->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_setACUTemperature_t,
              ntohs(psetACUTemperature->crc), ntohs(psetACUTemperature->eop))) )
    {
      //
      // pick up the new temperature
      //
      // if the ACUs addresses is out of range, send back failure
      //
      setPoint = atof(reinterpret_cast<char*>(psetACUTemperature->temperature));
      ACUAddress = ntohs(psetACUTemperature->ACU_address);

      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print( " found setPoint:ACUAddress ");
      Serial.print(setPoint,2);
      Serial.print(":");
      Serial.println(ACUAddress);
      Serial.flush();
      #endif

      if( (MAX_ACU_ADDRESS >= ACUAddress) )
      {
        if( (setACUTemp(ACUAddress, setPoint)) )
        {
          #ifdef __DEBUG2_VIA_SERIAL__
          Serial.print(__PRETTY_FUNCTION__); Serial.print( " success set temp for ACU: ");
          Serial.println(ACUAddress);
          Serial.flush();
          #endif
          result  = 1;

          //
          // TODO: DO NOT update the ACU's set point temperature ! 
          // rather, getStatus() queries the Meerstetter device
          //
          // TODO: document this behavior, update may not show on LCD
          // for 'a few seconds' until getStatus() queries the device
          //
          //sysStates.ACU[(ACUAddress - MIN_ACU_ADDRESS)].setpoint = setPoint;

        } else
        {
          #ifdef __DEBUG_VIA_SERIAL__
          Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: unable to temp for ACU: ");
          Serial.println(ACUAddress);
          #endif
          result  = 0;
        }
      } else
      {
        //
        // send back failure
        //
        result  = 0;

        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: ACU_address out of range: ");
        Serial.println(ACUAddress);
        #endif
      }

      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print( " sending back response for ACU: ");
      Serial.println(ACUAddress);
      #endif

      Serial.flush();
      
      //
      // use the sysStates conentent to respond, send back the received seqNum
      //
      respLength = cp.Make_setACUTemperatureResp(cp.m_peerAddress, cp.m_buff,
        ACUAddress, result, psetACUTemperature->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this functoin usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(psetACUTemperature->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(psetACUTemperature->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleGetACUTemperature(bool getObjTemp)
{
  getACUTemperature_t* pgetACUTemperature = reinterpret_cast<getACUTemperature_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  ACUAddress;
  uint16_t  result;


  //
  // verify the received packet, here beause this is a getACUTemperatureCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetACUTemperature->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getACUTemperature_t,
                ntohs(pgetACUTemperature->crc), ntohs(pgetACUTemperature->eop))) )
    {
      //
      // if the ACUs addresses is out of range, send back failure
      //
      ACUAddress = ntohs(pgetACUTemperature->ACU_address);

      if( (MAX_ACU_ADDRESS < ACUAddress) )
      {
        //
        // send back failure
        //
        result  = 0;

        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: ACU_address out of range: ");
        Serial.println(ACUAddress);
        #endif
      } else
      {
        result = 1;
      }
      
      //
      // use the sysStates conentent to respond, send back the received seqNum
      //
      if( (true == getObjTemp) )
        respLength = cp.Make_getACUObjTemperatureResp(cp.m_peerAddress, cp.m_buff,
          ACUAddress, result, sysStates.ACU[(ACUAddress - MIN_ACU_ADDRESS)].temperature,
          pgetACUTemperature->header.seqNum);
      else
        respLength = cp.Make_getACUTemperatureResp(cp.m_peerAddress, cp.m_buff,
          ACUAddress, result, sysStates.ACU[(ACUAddress - MIN_ACU_ADDRESS)].setpoint,
          pgetACUTemperature->header.seqNum);

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" sent response: ");
        Serial.println(sysStates.ACU[(ACUAddress - MIN_ACU_ADDRESS)].temperature, 2);
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pgetACUTemperature->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetACUTemperature->header.address.address));
    Serial.flush();
  #endif
  }
}
*/

#if defined (__USING_CHILLER__)
void handleStartChillerMsg(void)
{
  startChillerMsg_t* pstartChillerMsg = reinterpret_cast<startChillerMsg_t*>(cp.m_buff);
  uint16_t  respLength;
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a startChillerMsgCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pstartChillerMsg->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_startChillerMsg_t,
          ntohs(pstartChillerMsg->crc), ntohs(pstartChillerMsg->eop))) )
    {
      //
      // start the ACUs, chiller, sensor ...
      //
      if( (startChiller()) )
      {
        result  = 1;

      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: start chiller failed");
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_startChillerMsgResp(cp.m_peerAddress, cp.m_buff,
        result, pstartChillerMsg->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pstartChillerMsg->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pstartChillerMsg->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleStopChiller(void)
{
  stopChiller_t* pstopChiller = reinterpret_cast<stopChiller_t*>(cp.m_buff);
  uint16_t  respLength;
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a stopChillerCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pstopChiller->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_stopChiller_t,
          ntohs(pstopChiller->crc), ntohs(pstopChiller->eop))) )
    {
      //
      // stop chiller is effectively a shutDownSys() as the ACUs cannot
      // be running w/o the chiller running
      //
      if(shutDownSys(true))
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: stop chiller failed");
        Serial.flush();
        #endif
      }

      respLength = cp.Make_stopChillerResp(cp.m_peerAddress, cp.m_buff,
        result, pstopChiller->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pstopChiller->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pstopChiller->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleGetChillerInfo(void)
{
  getChillerInfo_t* pgetChillerInfo = reinterpret_cast<getChillerInfo_t*>(cp.m_buff);
  uint16_t  respLength;
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a getChillerInfoCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetChillerInfo->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getChillerInfo_t,
          ntohs(pgetChillerInfo->crc), ntohs(pgetChillerInfo->eop))) )
    {
      result  = 1;

      respLength = cp.Make_getChillerInfoResp(cp.m_peerAddress, cp.m_buff,
        result, reinterpret_cast<uint8_t*>(const_cast<char*>(chiller.GetSlaveName())),
        MAX_SLAVE_NAME_LENGTH, pgetChillerInfo->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pgetChillerInfo->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetChillerInfo->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleSetChillerTemperature(void)
{
  setChillerTemperature_t* psetChillerTemperature = reinterpret_cast<setChillerTemperature_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a setChillerTemperatureCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(psetChillerTemperature->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_setChillerTemperature_t,
                ntohs(psetChillerTemperature->crc), ntohs(psetChillerTemperature->eop))) )
    {
      //
      // set the new chiller temperature
      //
      if( (setChillerSetPoint(reinterpret_cast<char*>(psetChillerTemperature->temperature))) )
      {
        #ifdef __DEBUG2_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" success setChillerSetPoint");
        Serial.flush();
        #endif
        result = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to setChillerSetPoint to: ");
        Serial.println(reinterpret_cast<char*>(psetChillerTemperature->temperature));
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_setChillerTemperatureResp(cp.m_peerAddress, cp.m_buff,
        result, psetChillerTemperature->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(psetChillerTemperature->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(psetChillerTemperature->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleGetChillerTemperature(bool GetSetPoint)
{
  getChillerTemperature_t* pgetChillerTemperature = reinterpret_cast<getChillerTemperature_t*>(cp.m_buff);
  uint16_t  respLength; 


  //
  // verify the received packet, here beause this is a getChillerTemperatureCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetChillerTemperature->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getChillerTemperature_t,
                ntohs(pgetChillerTemperature->crc), ntohs(pgetChillerTemperature->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      if( (true == GetSetPoint) )
        respLength = cp.Make_getChillerTemperatureResp(cp.m_peerAddress, cp.m_buff,
          sysStates.chiller.setpoint, pgetChillerTemperature->header.seqNum);
      else
        respLength = cp.Make_getChillerObjTemperatureResp(cp.m_peerAddress, cp.m_buff,
          sysStates.chiller.temperature, pgetChillerTemperature->header.seqNum);

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pgetChillerTemperature->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetChillerTemperature->header.address.address));
    Serial.flush();
  #endif
  }
}
#endif

/*
void handlGetACUInfo(void)
{
  getACUInfoMsg_t*   pgetACUInfo = reinterpret_cast<getACUInfoMsg_t*>(cp.m_buff);
  uint16_t    respLength;
  uint16_t    result = 0;
  uint32_t    deviceType  = 0;
  uint32_t    hwVersion   = 0;
  uint32_t    fwVersion   = 0;
  uint32_t    serialNumber= 0;


  if( (ntohs(pgetACUInfo->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getACUInfoMsg_t,
                ntohs(pgetACUInfo->crc), ntohs(pgetACUInfo->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      if( (getACUInfo(ntohs(pgetACUInfo->ACU_address), &deviceType,
                &hwVersion, &fwVersion, &serialNumber)) )
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to startACUs");
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_getACUInfoMsgResp(cp.m_peerAddress, cp.m_buff, htons(pgetACUInfo->ACU_address), result,
                deviceType, hwVersion, fwVersion, serialNumber, pgetACUInfo->header.seqNum);

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pgetACUInfo->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetACUInfo->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleEnableACUs(void)
{
  enableACUs_t* penableACUs = reinterpret_cast<enableACUs_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a enableACUsCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(penableACUs->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_enableACUs_t,
                ntohs(penableACUs->crc), ntohs(penableACUs->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      if( (startACUs()) )
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to startACUs");
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_enableACUsResp(cp.m_peerAddress, cp.m_buff,
        result, penableACUs->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(penableACUs->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(penableACUs->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleDisableACUs(void)
{
  disableACUs_t* pdisableACUs = reinterpret_cast<disableACUs_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a disableACUsCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pdisableACUs->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_disableACUs_t,
                ntohs(pdisableACUs->crc), ntohs(pdisableACUs->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      if( (stopACUs()) )
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: failed to stopACUs");
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_disableACUsResp(cp.m_peerAddress, cp.m_buff,
        result, pdisableACUs->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this function usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #ifdef __DEBUG2_VIA_SERIAL__
      } else
      {
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pdisableACUs->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pdisableACUs->header.address.address));
    Serial.flush();
  #endif
  }
}

*/

void sendNACK(void)
{
  msgHeader_t*  pmsgHeader = reinterpret_cast<msgHeader_t*>(cp.m_buff);
  uint16_t    respLength; 


  respLength = cp.Make_NACK(cp.m_peerAddress, cp.m_buff, pmsgHeader->seqNum);

  //
  // use the CP object to send the response back
  // this function usese the cp.m_buff created above, just
  // need to send the lenght into the function
  //
  if( !(cp.doTxResponse(respLength)))
  {
    Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
    Serial.flush();
  #ifdef __DEBUG2_VIA_SERIAL__
  } else
  {
    Serial.println(__PRETTY_FUNCTION__); Serial.print(" sent response");
    Serial.flush();
  #endif
  }
}


bool ACUsRunning(void)
{
  bool retVal = true;


  for(int i = MIN_ACU_ADDRESS; i <= MAX_ACU_ADDRESS; i++)
  {
    if( (running != sysStates.ACU[(i - MIN_ACU_ADDRESS)].state) )
      retVal = false;
  }

  return(retVal);
}


void handleChillerStatus(void)
{
#if defined(__USING_CHILLER__)

  bool  retVal  = false;

  
  //
  // get all chiller information
  //
  if( (offline == sysStates.chiller.online) )
  {
    digitalWrite(FAULT_LED, HIGH);
    retVal = chiller.GetAllChillerInfo();
  } else
  {
    retVal  = chiller.getChillerStatus();
  }
  
  if( (false == retVal) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" WARINING: unable to GetAllChillerInfo");
    #endif

    //
    // update the chilller state to stopped (as it is not running)
    //
    sysStates.chiller.online              = offline;
    sysStates.chiller.state               = stopped;  // offline, we don't know
    sysStates.chiller.temperature             = 0;
    sysStates.chiller.setpoint              = 0;
    sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]  = chiller_Stopped;
    sysStates.lcd.lcdFacesIndex[CHILLER_FAIL_OFFSET]  = chiller_ComFailure;

  } else
  {
    //
    // update sysStates with what was feACUhed from GetAllChillerInfo
    //
    sysStates.chiller.online              = online;
    if( ('O' == chiller.GetTempCtrlMode()) )
    {
      sysStates.chiller.state = stopped;
      sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]  = chiller_Stopped;
    }
    else
    {
      sysStates.chiller.state = running;
      sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]  = chiller_Running;
    }

    sysStates.chiller.temperature             = chiller.GetInternalTempFloat();
    sysStates.chiller.setpoint              = chiller.GetSetPointFloat();
    sysStates.lcd.lcdFacesIndex[CHILLER_FAIL_OFFSET]  = no_Status;

    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("stored temperatures "); Serial.print(sysStates.chiller.temperature);
    Serial.print(" : "); Serial.println(sysStates.chiller.setpoint);
    Serial.print("chiller.GetTempCtrlMode(): "); Serial.println(chiller.GetTempCtrlMode());
    Serial.flush();
    #endif
  }
#endif
}


void handleACUStatus(void)
{
  bool  ACUsOnline  = true;
  bool  ACUsRunning = true;


  //
  // check all ACUs running - get Device Status, possible status are
  //
  // assume they are running, if one if found not running, mark the group
  // as not running
  //
  // before going to chat w/ the ACUs - enable the FAUL_LED so it is on 
  // when there is a fault and on while we could be timing out chatting
  // to a dead ACU
  for(uint8_t Address = MIN_ACU_ADDRESS; (Address <= MAX_ACU_ADDRESS); Address++)
  {
    if( (sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online == offline) )
    {
    digitalWrite(FAULT_LED, HIGH);
    break;
    }
  }

  for(uint8_t Address = MIN_ACU_ADDRESS; (Address <= MAX_ACU_ADDRESS); Address++)
  {
    sysStates.ACU[(Address - MIN_ACU_ADDRESS)].state      = running;
    sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online     = online;

    //
    // always fetch the ACU's set point and object temperatures
    //
    if( !(GetACUTemp(Address,
      &sysStates.ACU[(Address - MIN_ACU_ADDRESS)].setpoint,
      &sysStates.ACU[(Address - MIN_ACU_ADDRESS)].temperature)) )
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:ACU ");
      Serial.print(Address, DEC); Serial.println(" unable to get temps");
      Serial.flush();
      #endif

      sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online   = offline;
      sysStates.ACU[(Address - MIN_ACU_ADDRESS)].state  = stopped;
      #ifdef __DEBUG2_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" found ");
      Serial.print(sysStates.ACU[(Address - MIN_ACU_ADDRESS)].setpoint, 2);
      Serial.print(" : "); Serial.println(sysStates.ACU[(Address - MIN_ACU_ADDRESS)].temperature, 2);
      Serial.flush();
      #endif
    }

    if( !(ACURunning(Address)) )
    {
      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: ACU ");
      Serial.print(Address); Serial.println(" is not running");
      Serial.flush();
      #endif

      //
      // keep track of whether all ACUs are running
      //
      ACUsRunning = false;

      // update sysStates
      sysStates.ACU[(Address - MIN_ACU_ADDRESS)].state = stopped;

      //
      // check if we can still communicate with ACUs
      //
      if( !(ACUPresent(Address)) )
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:ACU ");
        Serial.print(Address, DEC); Serial.println(" is not on-line");
        Serial.flush();
        #endif

        ACUsOnline  = false;

        // update the sysStates
        sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online = offline;
      }
    }
  }


  //
  // if one ACU is down or bad, the overall status is bad
  //
  if( !(ACUsOnline) )
    sysStates.lcd.lcdFacesIndex[ACU_FAIL_OFFSET]   = ACU_ComFailure;
  else
    sysStates.lcd.lcdFacesIndex[ACU_FAIL_OFFSET]   = no_Status;

  if( !(ACUsRunning) )
    sysStates.lcd.lcdFacesIndex[ACU_NRML_OFFSET]   = ACU_Stopped;
  else
    sysStates.lcd.lcdFacesIndex[ACU_NRML_OFFSET]   = ACU_Running;
}


/* this will be replaces w/ a get info for the accuthermo
// careful . . ACU_address is a
bool getACUInfo(uint8_t ACU_address, uint32_t* deviceType, uint32_t* hwVersion,
                    uint32_t* fwVersion, uint32_t* serialNumber)
{
  bool retVal = true;


  if( !(ms.GetACUInfo(ACU_address, deviceType, hwVersion, fwVersion, serialNumber)) )
  {
    retVal  = false;
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println(__PRETTY_FUNCTION__); Serial.println(" getACUInfo failed");
    #endif
  }

  return(retVal);
}
*/

systemStatus setSystemStatus(void)
{
  systemStatus  retVal    = RUNNING;
  bool      ACUsOnline  = true;
  bool      ACUsRunning = true;
  bool      ACUMismatch = false;
  

  //
  // use the chiller and ACUs status
  //
  // the other 'bad' status for the other components shoudl already be
  // updated
  //
  // SHUTDOWN - if one thing is offline, or the chiller is not running - shutdown
  //


  //
  // accumulate the ACUs status'
  //
  for(int i = MIN_ACU_ADDRESS; i <= MAX_ACU_ADDRESS; i++)
  {
    if( (offline == sysStates.ACU[(i - MIN_ACU_ADDRESS)].online) )
      ACUsOnline = false;

    if( (stopped == sysStates.ACU[(i - MIN_ACU_ADDRESS)].state) )
      ACUsRunning = false;

    //
    // check for ACU on/off line or running mismatch, shutdown if present
    // can/could happen if a ACU fails or is reset
    //
    if( (sysStates.ACU[(i - MIN_ACU_ADDRESS)].online != sysStates.ACU[0].online) ||
      (sysStates.ACU[(i - MIN_ACU_ADDRESS)].state != sysStates.ACU[0].state) )
    {
      ACUMismatch = true;
    }
  }

  //
  // special case check - if the chiller is not running and the ACUs are running, shutdown the ACUs
  //
#if defined(__USING_CHILLER__)
  if( (sysStates.chiller.state != running && ACUsRunning == true) ||
    (ACUMismatch || offline == sysStates.chiller.online || false== ACUsOnline) ) {
#else
  if( (ACUMismatch || false== ACUsOnline) ) {
#endif
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Shutdown;
    retVal  = SHUTDOWN;

    if( (SHUTDOWN != sysStates.sysStatus) )
    {
      buttonOnOff         = false;
      currentButtonOnOff  = buttonOnOff;
      shutDownSys(false);
    }

    //
    // ALWAYS adjust the button, knobs, and LEDs
    //

    //
    // adjust the button LED
    //
    digitalWrite(BUTTON_LED, LOW);
  
    //
    // adjust the FAULT/NO-FAULT LEDs
    //
    // want this LED to blink
    if( (HIGH == digitalRead(FAULT_LED)) )
      digitalWrite(FAULT_LED, LOW);
    else
      digitalWrite(FAULT_LED, HIGH);
      
    digitalWrite(NO_FAULT_LED, LOW);
  

  //
  // READY - else everythig is online, ACUs are not running
  //
  }
#if defined(__USING_CHILLER__)
  else if( (online == sysStates.chiller.online && true == ACUsOnline)
        && (running != sysStates.chiller.state || false == ACUsRunning)) )
#else
  else if( ((true == ACUsOnline) && (false == ACUsRunning)) )
#endif
  {
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Ready;
    retVal  = READY;

    if( (READY != sysStates.sysStatus) )
    {
      buttonOnOff     = false;
      currentButtonOnOff  = buttonOnOff;

      //
      // adjust the button LED
      //
      digitalWrite(BUTTON_LED, LOW);
  
      //
      // adjust the FAULT/NO-FAULT LEDs
      //
      digitalWrite(FAULT_LED, LOW);
      digitalWrite(NO_FAULT_LED, HIGH);
    }
  //
  // else the system is running
  //
  } else
  {
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Running;
    retVal  = RUNNING;

    if( (RUNNING != sysStates.sysStatus) )
    {
      buttonOnOff     = true;
      currentButtonOnOff  = buttonOnOff;
      //
      // adjust the button LED
      //
      digitalWrite(BUTTON_LED, HIGH);
  
      //
      // adjust the FAULT/NO-FAULT LEDs
      //
      digitalWrite(FAULT_LED, LOW);
      digitalWrite(NO_FAULT_LED, LOW);
   
    }
  }

  sysStates.sysStatus = retVal;
  return(retVal);
}


void configureButton(void)
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // status LED - start as off
  pinMode(BUTTON_LED, OUTPUT);
  digitalWrite(BUTTON_LED, LOW);
}



void enableButtonISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
}


void disableButtonISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
  pinMode(BUTTON_PIN, OUTPUT);
}


void buttonISR(void)
{
  static unsigned long  buttonLastInterruptTime = 0;
  unsigned long     interruptTime   = millis();

  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

    
  if( (BUTTON_PERIOD < (interruptTime - buttonLastInterruptTime)) )
  {
    buttonOnOff = !buttonOnOff;
    buttonLastInterruptTime = interruptTime;
  }
}


//-------------------------------------------------------------
//
// - use global rs486Bus to write PVOF
// - read the value back
// - return the value read
//
uint16_t writePVOF(uint8_t id, uint16_t val)
{
  uint16_t  retVal  = 0x0000;

  
  // do the write
  cmdResp writePVOF = rs485Bus.writeProcess(id, PVOF, val);
  if( (false == writePVOF.retCode()) )
  {
    Serial.println("writePVOF fail");
  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.println("writePVOF success");
  #endif
  }


  // do the read, request 2 bytes back
  cmdResp readPVOF = rs485Bus.readProcess(id, PVOF, 2);
  if( (false == readPVOF.retCode()) )
  {
    Serial.print("readPVOF fail");
  } else
  {
    retVal = htons((*(reinterpret_cast<uint16_t*>(readPVOF.buff()))));
  #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("readPVOF success");
    Serial.print("readProcess(ASIC_ID, SV, 2) success, got "); Serial.print(readPVOF.bufflen()); Serial.println(" bytes returned");
    Serial.print(" read value is: "); Serial.println(retVal, 16);
  #endif
  }

  return(retVal);
}


//-------------------------------------------------------------
//
// - use global rs486Bus to do something for now
// - this will eventually be the 'start' cmd the ACU
// - read the value back
// - return the value read
// - return true if the value read back is what is expected .. 
//
bool StartACU(uint8_t id)
{
  // for now, just call writePVOF w/ the __INIT_PVOF__ and return true
  writePVOF(id, __INIT_PVOF__);

  return(true);
}


//-------------------------------------------------------------
//
// - use global rs486Bus to do something for now
// - this will eventually be the 'stop' cmd the ACU
// - read the value back
// - return the value read
// - return true if the value read back is what is expected .. 
//
bool StopACU(uint8_t id)
{
  // for now, just call writePVOF w/ the __INIT_PVOF__ and return true
  writePVOF(id, __INIT_PVOF__);

  return(true);
}


//-------------------------------------------------------------
//
// - use global rs486Bus to do something for now
// - this will eventually be the 'stop' cmd the ACU
// - read the value back
// - return the value read
// - return true if the value read back is what is expected .. 
//
bool ACURunning(uint8_t id)
{
  // for now, just call writePVOF w/ the __INIT_PVOF__ and return true
  writePVOF(id, __INIT_PVOF__);

  return(true);
}


//-------------------------------------------------------------
//
// - use global rs486Bus to do something for now
// - this will eventually be the 'stop' cmd the ACU
// - read the value back
// - return the value read
// - return true if the value read back is what is expected .. 
//
bool ACUPresent(uint8_t id)
{
  // for now, just call writePVOF w/ the __INIT_PVOF__ and return true
  writePVOF(id, __INIT_PVOF__);

  return(true);
}


//-------------------------------------------------------------
//
// - use global rs486Bus to do something for now
// - this will eventually be the 'stop' cmd the ACU
// - read the value back
// - return the value read
// - return true if the value read back is what is expected .. 
//
bool GetACUTemp(uint8_t id, float* setpoint, float* temperature)
{
  uint16_t  setp  = 0x0000;
  uint16_t  temp  = 0x0000;


  // do the read, request 2 bytes back
  cmdResp readPVPVOF = rs485Bus.readProcess(id, PVPVOF, 2);
  if( (false == readPVPVOF.retCode()) )
  {
    Serial.println("readPVPVOF fail");
  } else
  {
    temp = htons((*(reinterpret_cast<uint16_t*>(readPVPVOF.buff()))));
  #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("readPVPVOF success");
    Serial.print("readProcess(ASIC_ID, SV, 2) success, got "); Serial.print(readPVPVOF.bufflen()); Serial.println(" bytes returned");
    Serial.print(" read value is: "); Serial.println(temp, 16);
  #endif
  }


  // do the read, request 2 bytes back
  cmdResp readSV = rs485Bus.readProcess(id, SV, 2);
  if( (false == readSV.retCode()) )
  {
    Serial.println("readSV fail");
  } else
  {
    setp = htons((*(reinterpret_cast<uint16_t*>(readSV.buff()))));
  #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("readSV success");
    Serial.print("readProcess(ASIC_ID, SV, 2) success, got "); Serial.print(readSV.bufflen()); Serial.println(" bytes returned");
    Serial.print(" read value is: "); Serial.println(setp, 16);
  #endif
  }

  // convert the read values to float
  *setpoint     = (float)setp / (float)10;
  *temperature  = (float)temp / (float)10;

  return(true);
}
