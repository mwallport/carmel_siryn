#include "carmel_siryn.h"


int logEvent(uint16_t event_id, uint32_t inst = 0, uint32_t d0 = 0, uint32_t d1 = 0, uint32_t d2 = 0, uint32_t d3 = 0);


void setup(void)
{
  //
  // start the system components and Serial port if running debug
  //
  initSystem();

  //
  // banner - used to easily find system restarts in log file
  //
  #if defined( __DEBUG_VIA_SERIAL__) || defined(__DEBUG2_VIA_SERIAL__)
  Serial.println(" -----------------------------------------------------------"); Serial.flush();
  Serial.println(" -------------------------> setup() <-----------------------"); Serial.flush();
  Serial.println(" -----------------------------------------------------------"); Serial.flush();
  #endif


  //
  // TODO: testing only, remove - using this for random temps for RTD testing
  //
  randomSeed(analogRead(0));


  //
  // always start the chiller
  //
  #ifdef __USING_CHILLER__
  startChiller();
  #endif
  
  //
  // these are normally gotten via getStatus() which runs on a period
  // at start up, i.e. now, the period will not have lapsed, so get them now
  // before getting in loop
  //
  #ifdef __USING_CHILLER__
  handleChillerStatus();
  #endif
  handleACUStatus();
  handleRTDStatus();
}


void loop(void)
{
  //
  // getStatus will update LCD and sysStats data structure
  //
  getStatus();

  //
  // determine the hot RTD temperature
  //
  setHotRTD();

  //
  // determine the hot RTD, update DDR Accuthermo PVOF
  // calculateAndWritePVOF is using stored data fetched by
  // getStatus() and setHotRTD() . . so those funcitons must
  // be called prior to calling calculateAndWritePVOF
  //
  calculateAndWritePVOF();

  // this has its own loop and exists when sysStates.sysStatus != RUNNING
  //handleRunningState();

  //
  // update the LCD
  //
  manageLCD(false);

  //
  // take commands from the
  // - the buffon on front panel
  // - or the controlling PC software
  //
  // - handleMsgs(tmo) is the throttle of the loop
  // - RUNNING state the tmo for input is 3ms
  // - Otherwise the tmo for input is 3000ms
  //
  if( (RUNNING == sysStates.sysStatus) )
    handleMsgs(CTRL_TIMEOUT_RUNNING_STATE); // tmo is 3ms
  else
    handleMsgs(CTRL_TIMEOUT);               // tmo is 3000ms
}



//-------------------------------------------------------------
//
//
void initSystem(void)
{
  //
  // disable the ISRs
  //
  handleRTDISRs(false);
 
  //
  // start the LCD and paint system initializing
  //
  startLCD();
   
  //
  // initialize the system states /stats - these are
  // used to hold temperatures, humidity, etc. and
  // used in responses to getStatusCmd from control
  // and holds the LCD messages
  //
  initSysStates(sysStates);


  
  //
  // start the Serial ports
  //
  #if defined __DEBUG_VIA_SERIAL__ || defined __DEBUG2_VIA_SERIAL__
  Serial.begin(19200);
  #endif

  // control protocol uses Serial1
  Serial1.begin(CONTROL_PROTO_SPEED);

  // chiller protocol uses Serial2
  #if defined(__USING_CHILLER__)
  Serial2.begin(CHILLER_PROTO_SPEED
  #endif

  // RS485 uses Serial3
  Serial3.begin(RS485_MOD_BUS_SPEED);

  //
  // wait for Serial ports to become ready - wait forever, can't start without them
  //
  while( (!Serial1) )
  {
    #if defined __DEBUG_VIA_SERIAL__
    Serial.println("waiting for Serial1 to become ready");
    #endif
    delay(250);
  }

  #if defined(__USING_CHILLER__)
  while( (!Serial2) )
  {
    #if defined __DEBUG_VIA_SERIAL__
    Serial.println("waiting for Serial2 to become ready");
    #endif
    delay(250);
  }
  #endif
  
  while( (!Serial3) )
  {
    #if defined __DEBUG_VIA_SERIAL__
    Serial.println("waiting for Serial3 to become ready");
    #endif
    delay(250);
  }



  //
  // initialize the start/stop button
  //
//  configureButton();

  //
  // initialize the fault/no-fault LED(s)
  //
  configureFaultNoFault();


  //
  // initialize the Adafruits
  //
  DDR1_RTD.begin(MAX31865_4WIRE);
  DDR2_RTD.begin(MAX31865_4WIRE);
  ASIC_Chiller_RTD.begin(MAX31865_4WIRE);
  DDR_Chiller_RTD.begin(MAX31865_4WIRE);


  //
  // initialize the Accuthermos
  //
  //
  // clear the PVOF
  //
  writePVOF(ASIC_RS485_ID, 0);
  writePVOF(DDR_RS485_ID, 0);

  //
  // set Celcius
  //
  writeUNIT(ASIC_RS485_ID, 0x001C);
  writeUNIT(DDR_RS485_ID, 0x001C);


  //
  // calculate the time between samples for the adafruits
  //
  timeBetweenSamples  = (1000 - (80 * DDR_RTDS_SAMPLES_PER_SEC)) / DDR_RTDS_SAMPLES_PER_SEC;
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
    states.ACU[(i - MIN_ACU_ADDRESS)].temperature = 0;
  }

  // RTDs all start offline until discovered to be online via getStatus()
  states.ASIC_RTD.online      = offline;
  states.ASIC_RTD.state       = stopped;
  states.ASIC_RTD.temperature = 0;

  states.ASIC_Chiller_RTD.online      = offline;
  states.ASIC_Chiller_RTD.state       = stopped;
  states.ASIC_Chiller_RTD.temperature = 0;

  states.DDR_RTD.online      = offline;
  states.DDR_RTD.state       = stopped;
  states.DDR_RTD.temperature = 0;

  states.DDR1_RTD.online      = offline;
  states.DDR1_RTD.state       = stopped;
  states.DDR1_RTD.temperature = 0;

  states.DDR2_RTD.online      = offline;
  states.DDR2_RTD.state       = stopped;
  states.DDR2_RTD.temperature = 0;

  states.DDR_Chiller_RTD.online      = offline;
  states.DDR_Chiller_RTD.state       = stopped;
  states.DDR_Chiller_RTD.temperature = 0;


  // lcd - initialize all messages
  for(int i = 0; i < MAX_LCD_MSGS; i++)
    sysStates.lcd.lcdFacesIndex[i] = no_Status; // no_Status is 0, so same as NULL .. sort of ..

  // pick up the millis() 'now' - something fishy about this, what will happen when
  // the counter rolls over ?  This program is suppossed to run for a long time..
  // TODO: find a better way - use timeofday or the RTC.. ?
  sysStates.lcd.prior_millis = millis();

  //
  // setup the 'good' lcd functions - the error faces are 0
  //
  sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]   = sys_Initializing;
  sysStates.lcd.lcdFacesIndex[ACU_NRML_OFFSET]      = ACU_Stopped;
  sysStates.lcd.lcdFacesIndex[ASIC_RTD_NRML_OFFSET] = ASIC_RTD_Running;
  sysStates.lcd.lcdFacesIndex[DDR_RTD_NRML_OFFSET]  = DDR_RTD_Running;
#if defined(__USING_CHILLER__)
  sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]  = chiller_Stopped;
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
void manageLCD(bool now)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif

  //
  // if enough time has passed, display the current index
  //   
  if( (now) || (MAX_MSG_DISPLAY_TIME < (millis() - sysStates.lcd.prior_millis)) )
  {
    // set up the LCD's number of columns and rows:
    lcd.noDisplay(); lcd.begin(20, 4); lcd.noDisplay(); lcd.clear(); lcd.home();

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
  lcd.begin(20, 4);
  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print(deftDevise);
  lcd.setCursor(0,1);
  lcd.print(buildInfo);
  lcd.setCursor(0,2);
  lcd.print(deftDevise);
  lcd.setCursor(0,3);
  lcd.print(buildInfo);
  lcd.display();

  delay(2000);

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
void getStatus()
{
  static unsigned long  lastGetStatusTime     = 0;
  unsigned long         currentGetStatusTime  = millis();


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
    #ifdef __USING_CHILLER__
    handleChillerStatus();
    #endif
    handleACUStatus();
    handleRTDStatus();
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
  //setSystemStatus();

  return(retVal);
#else
  return(true);
#endif
}


// ---------------------------------------
// find all ACUs, there should be 2
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

#if defined (__USING_CHILLER__)
  } else
  {
    retVal = false;
    
    #ifdef __DEBUG_VIA_SERIAL__
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


bool setACUTemp(uint16_t ACUAddress, float temp)
{
  bool retVal = true;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  if( (MAX_ACU_ADDRESS >= ACUAddress) )
  {
    if( (SetACUSetPointValue(ACUAddress, temp)) )
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


void handleMsgs(uint16_t tmo)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  if( (currentButtonOnOff != buttonOnOff) )
  {
    currentButtonOnOff = buttonOnOff;
      
    #ifdef __DEBUG2_VIA_SERIAL__
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
    if( (getMsgFromControl(tmo)) )
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
  
        case getStatusCmd:         // fetch the status of chiller, all ACUs, and humidity sensor
        {
          handleGetStatusCmd();
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
        case setRTCCmd:
        {
            handleSetRTCCmd();
            break;
        }

        case getRTCCmd:
        {
            handleGetRTCCmd();
            break;
        }

        case clrEventLogCmd:
        {
            handleClrEventLogCmd();
            break;
        }

        case getEventLogCmd:
        {
            handleGetEventLogCmd();
            break;
        }

        default:
        {
          // send NACK
          sendNACK();
          break;
        }
      }
    } // else no message from control
    #ifdef __DEBUG2_VIA_SERIAL__
    else 
    Serial.println("did not get a command");
    #endif
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
  lcd.setCursor(2,1);
  lcd.print("**** SYSTEM ****");
  lcd.setCursor(2,2);
  lcd.print("**SHUTTING DOWN**");
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
  lcd.print("Temp Ctrl RUNNING");
  lcd.setCursor(0,1);
  lcd.print("ASIC Sv      Pv     ");
  lcd.setCursor(0,3);
  lcd.print("DDR  Sv      Pv     ");

  // if the SV is negative, scoot left one position
  if( (sysStates.ACU[0].setpoint < 0) )
    lcd.setCursor(7,1);
  else
    lcd.setCursor(8,1);
  lcd.print(sysStates.ACU[0].setpoint,1);

  // if the PV is negative, scoot left one position
  if( (sysStates.ACU[0].temperature < 0) )
    lcd.setCursor(15,1);
  else
    lcd.setCursor(16,1);
  lcd.print(sysStates.ACU[0].temperature,1);
    

  // if the SV is negative, scoot left one position
  if( (sysStates.ACU[1].setpoint < 0) )
    lcd.setCursor(7,3);
  else
    lcd.setCursor(8,3);
  lcd.print(sysStates.ACU[1].setpoint,1);

  // if the PV is negative, scoot left one position
  if( (sysStates.ACU[1].temperature < 0) )
    lcd.setCursor(15,3);
  else
    lcd.setCursor(16,3);
  lcd.print(sysStates.ACU[1].temperature,1);
  
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
  lcd.print("Temp Ctrl STOPPED");
  lcd.setCursor(0,1);
  lcd.print("ASIC Sv      Pv     ");
  lcd.setCursor(0,3);
  lcd.print("DDR  Sv      Pv     ");

  // if the SV is negative, scoot left one position
  if( (sysStates.ACU[0].setpoint < 0) )
    lcd.setCursor(7,1);
  else
    lcd.setCursor(8,1);
  lcd.print(sysStates.ACU[0].setpoint,1);

  // if the PV is negative, scoot left one position
  if( (sysStates.ACU[0].temperature < 0) )
    lcd.setCursor(15,1);
  else
    lcd.setCursor(16,1);
  lcd.print(sysStates.ACU[0].temperature,1);
    

  // if the SV is negative, scoot left one position
  if( (sysStates.ACU[1].setpoint < 0) )
    lcd.setCursor(7,3);
  else
    lcd.setCursor(8,3);
  lcd.print(sysStates.ACU[1].setpoint,1);

  // if the PV is negative, scoot left one position
  if( (sysStates.ACU[1].temperature < 0) )
    lcd.setCursor(15,3);
  else
    lcd.setCursor(16,3);
  lcd.print(sysStates.ACU[1].temperature,1);
  
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
  lcd.print("TMP CTRL COM FAILURE");

  // print the ASIC controll status
  lcd.setCursor(0,1);
  if( (offline == sysStates.ACU[ASIC_ACU_IDX].online) )
    lcd.print("ASIC OFFLINE");
  else
    lcd.print("ASIC ONLINE");

  // print the DDR controller status
  lcd.setCursor(0,3);
  if( (offline == sysStates.ACU[DDR_ACU_IDX].online) )
    lcd.print("DDR OFFLINE");
  else
    lcd.print("DDR ONLINE");
  
  lcd.display();
}


void lcd_ASIC_RTDs_Running(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif 

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("ASIC RTDs     ");
  lcd.setCursor(0,2);
  lcd.print("1. ");
  lcd.setCursor(3,2);
  lcd.print(sysStates.ASIC_RTD.temperature, 1);
  lcd.setCursor(11, 2);
  lcd.print("H2O");
  lcd.setCursor(15, 2);
  lcd.print(sysStates.ASIC_Chiller_RTD.temperature,1);

  //
  // put an exclamation point if there is a fault w/ a DDR
  //
  if( (sysStates.ASIC_RTD.fault) )
  {
    lcd.setCursor(7,2);
    lcd.print("!");
  }

  if( (sysStates.ASIC_Chiller_RTD.fault) )
  {
    lcd.setCursor(19,2);
    lcd.print("!");
  }

  lcd.display();
}


void lcd_ASIC_RTDs_Failure(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("ASIC RTD FAILURE");
  lcd.setCursor(0,2);
  lcd.print("1. 0x");
  lcd.setCursor(5,2);
  lcd.print(sysStates.ASIC_RTD.fault, HEX);
  lcd.setCursor(10, 2);
  lcd.print("H2O 0x");
  lcd.setCursor(16, 2);
  lcd.print(sysStates.ASIC_Chiller_RTD.fault, HEX);

  lcd.display();
}


void lcd_DDR_RTDs_Running(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif  

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("DDR RTDs     ");
  lcd.setCursor(0,1);
  lcd.print("1. ");
  lcd.setCursor(3,1);
  lcd.print(sysStates.DDR_RTD.temperature, 1);
  lcd.setCursor(10, 1);
  lcd.print("2. ");
  lcd.setCursor(14, 1);
  lcd.print(sysStates.DDR1_RTD.temperature, 1);
  lcd.setCursor(0,3);
  lcd.print("3. ");
  lcd.setCursor(3,3);
  lcd.print(sysStates.DDR2_RTD.temperature,1);
  lcd.setCursor(10, 3);
  lcd.print("H2O");
  lcd.setCursor(14, 3);
  lcd.print(sysStates.DDR_Chiller_RTD.temperature, 1);  

  //
  // put an asterisk after the temperature being used for the PVOF calculation
  //
  // the DDRs are checked in order of 1, 2, 3 for the hottest, so set the
  // asterisk by checking for match to the hottest in the same order
  //
  if( (sysStates.highRTDTemp == sysStates.DDR_RTD.temperature) )
    lcd.setCursor(7,1);
  else if( (sysStates.highRTDTemp == sysStates.DDR1_RTD.temperature) )
    lcd.setCursor(18,1);
  else // gotta be DDR2 ..
    lcd.setCursor(7,3);

  lcd.print("*");

  //
  // put an exclamation point if there is a fault w/ a DDR
  //
  if( (sysStates.DDR_RTD.fault) )
  {
    lcd.setCursor(8,1);
    lcd.print("!");
  }

  if( (sysStates.DDR1_RTD.fault) )
  {
    lcd.setCursor(19,1);
    lcd.print("!");
  }

  if( (sysStates.DDR2_RTD.fault) )
  {
    lcd.setCursor(8,3);
    lcd.print("!");
  }

  lcd.display();
}


void lcd_DDR_RTDs_Failure(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("DDR RTD FAILURE");
  lcd.setCursor(0,1);
  lcd.print("1. 0x");
  lcd.setCursor(5,1);
  lcd.print(sysStates.DDR_RTD.fault, HEX);
  lcd.setCursor(10, 1);
  lcd.print("2. 0x");
  lcd.setCursor(15, 1);
  lcd.print(sysStates.DDR1_RTD.fault, HEX);
  lcd.setCursor(0,3);
  lcd.print("3. 0x");
  lcd.setCursor(5,3);
  lcd.print(sysStates.DDR2_RTD.fault, HEX);
  lcd.setCursor(10, 3);
  lcd.print("H2O 0x");
  lcd.setCursor(16, 3);
  lcd.print(sysStates.DDR_Chiller_RTD.fault, HEX);  

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


bool getMsgFromControl(uint16_t tmo)
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
  if( (cp.doRxCommand(tmo)) )
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
/*        
      } else
      {
        setSystemStatus();  // derive the button state
*/
      }

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
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
        #endif
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
/*        
      } else
      {
        setSystemStatus();  // derive the button state\
*/
      }

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
      #if defined(__USING_CHILLER__)
      respLength = cp.Make_getStatusResp(cp.m_peerAddress, cp.m_buff,
        ((true == checkRTDStatus()) ? 1 : 0),           // enough RTDs running that have not ShutDown .. 
        ((true == ACUsRunning()) ? 1 : 0),              // ACUs running
        ((running == sysStates.chiller.state) ? 1 : 0), // chiller running
        pgetStatus->header.seqNum
      );
      #else
      respLength = cp.Make_getStatusResp(cp.m_peerAddress, cp.m_buff,
        ((true == checkRTDStatus()) ? 1 : 0),           // enough RTDs running that have not ShutDown .. 
        ((true == ACUsRunning()) ? 1 : 0),              // ACUs running
        0,                                              // aint got no chiller running
        pgetStatus->header.seqNum
      );
      #endif

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
      ACUAddress = ntohs(psetACUTemperature->acu_address);

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
        Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: acu_address out of range: ");
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
      ACUAddress = ntohs(pgetACUTemperature->acu_address);

      if( (MAX_ACU_ADDRESS < ACUAddress) )
      {
        //
        // send back failure
        //
        result  = 0;

        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: acu_address out of range: ");
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
      if( (getACUInfo(ntohs(pgetACUInfo->acu_address), &deviceType,
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

      respLength = cp.Make_getACUInfoMsgResp(cp.m_peerAddress, cp.m_buff, htons(pgetACUInfo->acu_address), result,
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
      // TODO: make the MODE some in the menu command
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


void handleSetRTCCmd(void)
{
    setRTCCmd_t* psetRTCCmd = reinterpret_cast<setRTCCmd_t*>(cp.m_buff);
    uint16_t    respLength;
    uint16_t    result = 0;


    //
    // verify the received packet, here beause this is a setRTCCmdCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(psetRTCCmd->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_setRTCCmd_t,
                                ntohs(psetRTCCmd->crc), ntohs(psetRTCCmd->eop))) )
        {
            //
            // set the RTC - don't know if there is a 'bad' return code from this call
            //
   /* TODO: put the RTC we decide to use
            Controllino_SetTimeDate(psetRTCCmd->tv.mday, psetRTCCmd->tv.wday, psetRTCCmd->tv.mon,
              psetRTCCmd->tv.year, psetRTCCmd->tv.hour, psetRTCCmd->tv.min, psetRTCCmd->tv.sec);
   */
            result  = 1;

            respLength = cp.Make_setRTCCmdResp(cp.m_peerAddress, cp.m_buff,
                result, psetRTCCmd->header.seqNum
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
            Serial.println(ntohs(psetRTCCmd->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(psetRTCCmd->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleGetRTCCmd(void)
{
    getRTCCmd_t* pgetRTCCmd = reinterpret_cast<getRTCCmd_t*>(cp.m_buff);
    uint16_t    respLength;
    uint16_t    result = 0;
    timeind     RTCTime;


    //
    // verify the received packet, here beause this is a getRTCCmdCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetRTCCmd->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getRTCCmd_t,
                                ntohs(pgetRTCCmd->crc), ntohs(pgetRTCCmd->eop))) )
        {
            //
            // get the RTC - don't know if there is a 'bad' return code from this call
            //
            /* TODO: fix this when we get the RTC figured out
            result  = getControllinoTime(&RTCTime);
            */
            result = 1;

            respLength = cp.Make_getRTCCmdResp(cp.m_peerAddress, cp.m_buff,
                &RTCTime, result, pgetRTCCmd->header.seqNum
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
            Serial.println(ntohs(pgetRTCCmd->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetRTCCmd->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleClrEventLogCmd(void)
{
    clrEventLogCmd_t* pclrEventLogCmd = reinterpret_cast<clrEventLogCmd_t*>(cp.m_buff);
    uint16_t    respLength;
    uint16_t    result = 1;


    //
    // verify the received packet, here beause this is a clrEventLogCmdCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pclrEventLogCmd->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_clrEventLogCmd_t,
                                ntohs(pclrEventLogCmd->crc), ntohs(pclrEventLogCmd->eop))) )
        {
            //
            // get the RTC - don't know if there is a 'bad' return code from this call
            //
            clrEventLog();

            respLength = cp.Make_clrEventLogCmdResp(cp.m_peerAddress, cp.m_buff,
                result, pclrEventLogCmd->header.seqNum
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
            Serial.println(ntohs(pclrEventLogCmd->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pclrEventLogCmd->header.address.address));
        Serial.flush();
    #endif
    }
}


void handleGetEventLogCmd(void)
{
    getEventLogCmd_t* pgetEventLogCmd = reinterpret_cast<getEventLogCmd_t*>(cp.m_buff);
    uint16_t    respLength;
    uint16_t    result = 1;


    //
    // verify the received packet, here beause this is a getEventLogCmdCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetEventLogCmd->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getEventLogCmd_t,
                                ntohs(pgetEventLogCmd->crc), ntohs(pgetEventLogCmd->eop))) )
        {
            //
            // get the RTC - don't know if there is a 'bad' return code from this call
            //
            respLength = cp.Make_getEventLogCmdResp(cp.m_peerAddress, cp.m_buff,
                result, getEventLog(), pgetEventLogCmd->header.seqNum
            );

            //
            // use the CP object to send the response back
            // this function usese the cp.m_buff created above, just
            // need to send the lenght into the function
            //

            if( !(cp.doTxResponse(respLength)))
            {
                Serial.print(__PRETTY_FUNCTION__); Serial.println(" ERROR: failed to send response");
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
            Serial.println(ntohs(pgetEventLogCmd->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetEventLogCmd->header.address.address));
        Serial.flush();
    #endif
    }
}


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


// don't log events here, those are handled in setSystemStatus
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
    sysStates.chiller.temperature         = 0;
    sysStates.chiller.setpoint            = 0;
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
    // initialize to running and online
    sysStates.ACU[(Address - MIN_ACU_ADDRESS)].state      = running;
    sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online     = online;

    //
    // get the ACU's set point and object temperatures
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

      ACUsRunning = false;
      ACUsOnline  = false;

      sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online = offline;
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

    //
    // get the value of ENAB, the current run mode
    //
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
    }

    //
    // ASIC_RTD is connected to the Accuthermo RS485 Id 1
    // DDR_RTD is connected to the Accuthermo RS485 Id 2
    //
    // translate the retrieved data and temp from the Accuthermos to these RTDs
    //
    if( (ASIC_RS485_ID == Address) )
    {
      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.println("Translating ASIC ACU data to ASIC_RTD");
      #endif      

      resetRTDState(sysStates.ASIC_RTD);
      
      // pick up the temperature and faul, the online and state are updated in handleRTDStatus
      sysStates.ASIC_RTD.temperature  = sysStates.ACU[(Address - MIN_ACU_ADDRESS)].temperature;

      Serial.print("ASIC_RTD.temperature: "); Serial.println(sysStates.ASIC_RTD.temperature, 2);
      Serial.print("souce temperature: "); Serial.println(sysStates.ACU[(Address - MIN_ACU_ADDRESS)].temperature, 2);
      

      if( (offline == sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online) )
      {
        sysStates.ASIC_RTD.online = offline;
        sysStates.ASIC_RTD.state  = stopped;    
        sysStates.ASIC_RTD.fault  = 0xFF;
      }

    } else if( (DDR_RS485_ID == Address) )
    {
      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.println("Translating DDR ACU data to DDR_RTD");
      #endif      
      
      resetRTDState(sysStates.DDR_RTD);

      // pick up the temperature and faul, the online and state are updated in handleRTDStatus
      sysStates.DDR_RTD.temperature  = sysStates.ACU[(Address - MIN_ACU_ADDRESS)].temperature;

      if( (offline == sysStates.ACU[(Address - MIN_ACU_ADDRESS)].online) )
      {
        sysStates.DDR_RTD.online  = offline;
        sysStates.DDR_RTD.state   = stopped;    
        sysStates.DDR_RTD.fault   = 0xFF;
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


/* this will be replaces w/ a get info for the accuthermo */
bool getACUInfo(uint8_t acu_address, uint32_t* deviceType, uint32_t* hwVersion,
                    uint32_t* fwVersion, uint32_t* serialNumber)
{
  bool retVal = true;

/* TODO: put this back <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  if( !(ms.GetACUInfo(acu_address, deviceType, hwVersion, fwVersion, serialNumber)) )
  {
    retVal  = false;
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println(__PRETTY_FUNCTION__); Serial.println(" getACUInfo failed");
    #endif
  }
*/
  return(retVal);
}


void handleRTDStatus()
{
  bool  NON_DDR_RTDsRunning  = true;  // becomes false is at least one RTD has fault
  bool  DDR_RTDsRunning   = true;  // becomes false is at least one RTD has fault


  
  //
  // check all RTds running
  //
  // assume they are running, if one if found not running, mark the group
  // as not running
  //
  // before going to chat w/ the RTDs - enable the FAUL_LED if already in fault so it is on 
  // while we could be timing out chatting up a dead RTD
  //
  if( !(checkRTDStatus()) )
  {
    digitalWrite(FAULT_LED, HIGH);
  }
  
  //
  // get all the ASIC, ASIC chiller and DDR chiller RTD data
  //
  getNonDDRRTDData();

  //
  // get all DDR RTDs temperatures
  //
  getDDRRTDData(true);

  //
  // if any have fault . . shut it down ?
  //

  //
  // check the non DDR RTDs
  //
  if( (sysStates.ASIC_RTD.fault) )
  {
    // this RTD is connected to the Accuthermo Id 1 and
    // this data is updated for this RTD in handleACUStatus()
//    sysStates.ASIC_RTD.online  = offline;
//    sysStates.ASIC_RTD.state   = stopped;    
    NON_DDR_RTDsRunning  = false;
  } else
  {
    sysStates.ASIC_RTD.prior_fault  = 0; 
  }

  if( (sysStates.ASIC_Chiller_RTD.fault) )
  {
    sysStates.ASIC_Chiller_RTD.online  = offline;
    sysStates.ASIC_Chiller_RTD.state   = stopped;
    NON_DDR_RTDsRunning  = false;  
  } else
  {
    sysStates.ASIC_Chiller_RTD.prior_fault = 0;
  }

  if( (sysStates.DDR_Chiller_RTD.fault) )
  {
    sysStates.DDR_Chiller_RTD.online  = offline;
    sysStates.DDR_Chiller_RTD.state   = stopped;    
    NON_DDR_RTDsRunning  = false;  
  } else
  {
    sysStates.DDR_Chiller_RTD.prior_fault = 0;
  }

  // 
  // check the DDR RTDs
  //
  if( (sysStates.DDR_RTD.fault) )
  {
    // this RTD is connected to the Accuthermo Id 2 and
    // this data is updated for this RTD in handleACUStatus()
//    sysStates.DDR_RTD.online  = offline;
//    sysStates.DDR_RTD.state   = stopped;    
    DDR_RTDsRunning  = false;  
  } else
  {
    sysStates.DDR_RTD.prior_fault = 0;
  }

  if( (sysStates.DDR1_RTD.fault) )
  {
    sysStates.DDR1_RTD.online  = offline;
    sysStates.DDR1_RTD.state   = stopped;    
    DDR_RTDsRunning  = false;  
  } else
  {
    sysStates.DDR1_RTD.prior_fault = 0;
  }

  if( (sysStates.DDR2_RTD.fault) )
  {
    sysStates.DDR2_RTD.online  = offline;
    sysStates.DDR2_RTD.state   = stopped;    
    DDR_RTDsRunning  = false;  
  } else
  {
    sysStates.DDR2_RTD.prior_fault = 0;
  }


  //
  // update the LCD face for ASIC RTDs
  //
  if( !(NON_DDR_RTDsRunning) )
  {
    // TODO: fix this for NON_DDR_RTDs vs DDR_RTDs
    sysStates.lcd.lcdFacesIndex[ASIC_RTD_NRML_OFFSET]   = no_Status;
    sysStates.lcd.lcdFacesIndex[ASIC_RTD_FAIL_OFFSET]   = ASIC_RTD_Failure;
  }
  else
  {
    sysStates.lcd.lcdFacesIndex[ASIC_RTD_NRML_OFFSET]   = ASIC_RTD_Running;
    sysStates.lcd.lcdFacesIndex[ASIC_RTD_FAIL_OFFSET]   = no_Status;
  }


  //
  // update the LCD face for DDR RTDs
  //
  if( !(DDR_RTDsRunning) )
  {
    sysStates.lcd.lcdFacesIndex[DDR_RTD_NRML_OFFSET]   = no_Status;
    sysStates.lcd.lcdFacesIndex[DDR_RTD_FAIL_OFFSET]   = DDR_RTD_Failure;
  }
  else
  {
    sysStates.lcd.lcdFacesIndex[DDR_RTD_NRML_OFFSET]   = DDR_RTD_Running;
    sysStates.lcd.lcdFacesIndex[DDR_RTD_FAIL_OFFSET]   = no_Status;
  }
}


void getNonDDRRTDData()
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif

  
  //
  // reset the stats/data for each RTD - set them online and running
  // and zero the rtd, fault, and temperature
  //
  resetRTDState(sysStates.ASIC_Chiller_RTD);
  resetRTDState(sysStates.DDR_Chiller_RTD);

  getAdafruitRTDData(ASIC_Chiller_RTD, sysStates.ASIC_Chiller_RTD, true);
  getAdafruitRTDData(DDR_Chiller_RTD, sysStates.DDR_Chiller_RTD, true);
}


// get the data for the RTDs in the DDR part
void getDDRRTDData(bool getAllData)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif

  
  //
  // reset the stats/data for each RTD - set them online and running
  // and zero the rtd, fault, and temperature
  //
  resetRTDState(sysStates.DDR1_RTD);
  resetRTDState(sysStates.DDR2_RTD);

  getAdafruitRTDData(DDR1_RTD, sysStates.DDR1_RTD, getAllData);
  getAdafruitRTDData(DDR2_RTD, sysStates.DDR2_RTD, getAllData);
}



// this data is processed in the LCD and setSystemStatus functions - only getting it here
void getAdafruitRTDData(Adafruit_MAX31865& afmaxRTD, RTDState& state, bool getAllData)
{

                                                            // hard coded delay(x)

  // only get these on a period - i.e. not ALL the time
  if( (true == getAllData) )
  {
    state.rtd         = afmaxRTD.readRTD(); // this one has the 75ms hard coded delay AND IS CALLED by temperature, don't call this everytime
    state.fault       = afmaxRTD.readFault(); // reads a register
  }

  // always get temperature
  state.temperature = afmaxRTD.temperature(RNOMINAL, RREF); // calls readRTD() where the are .075 seconds of


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.print("fault: "); Serial.print(state.fault); Serial.print(" rtd: "); Serial.print(state.rtd);
  Serial.print(" temperature: "); Serial.println(state.temperature, 1);
  #endif
}


// only called by getSatus() - keep it that way !
systemStatus setSystemStatus(void)
{
  systemStatus  retVal      = RUNNING;
  bool      ACUsOnline      = true;
  bool      ACUsRunning     = true;
  bool      ACUMismatch     = false;
  bool      RTDsRunning     = true;  
  

  Serial.println("entered setSystemStatus.....");

  //
  // ACUs
  //
  // accumulate the ACUs status' - if one ACU is bad, they are all bad
  //
  for(int i = MIN_ACU_ADDRESS; i <= MAX_ACU_ADDRESS; i++)
  {
    if( (offline == sysStates.ACU[(i - MIN_ACU_ADDRESS)].online) )
    {
      ACUsOnline = false;

      // if sysStatus is SHUTDOWN, would have already logged this
      if( (SHUTDOWN != sysStates.sysStatus) )
      {
        logEvent(ACUNotOnLine, i);

        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("logging ACU not online");
        #endif
      }
    }

    if( (stopped == sysStates.ACU[(i - MIN_ACU_ADDRESS)].state) )
    {
      ACUsRunning = false;

      // if RUNNING, log this event, about to change state because ACUsRunning is false
      if( (RUNNING == sysStates.sysStatus) )
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("logging ACU not running");
        #endif
        
        logEvent(ACUNotRunning, i);
      }
    }

    //
    // check for ACU on/off line or running mismatch, shutdown if present
    // can/could happen if a ACU fails or is reset
    //
    // checking against index 0 because they should all be the same
    //
    if( (sysStates.ACU[(i - MIN_ACU_ADDRESS)].online != sysStates.ACU[0].online) ||
      (sysStates.ACU[(i - MIN_ACU_ADDRESS)].state != sysStates.ACU[0].state) )
    {
      ACUMismatch = true;

      if( (SHUTDOWN != sysStates.sysStatus) )
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("logging ACU is mismatch");
        #endif

        logEvent(ACUIsMismatch);
      }
    }
  }

  
  // RTDs
  //
  //
  // accumulate the RTD's status - return 'bad' status according to Rick's instruction
  //
  if( !(checkRTDStatus()) )
    RTDsRunning = false;


  //
  // log any RTS faults if new fault is not same as prior fault
  //
  // check the non DDR RTDs
  //
  if( (sysStates.ASIC_RTD.fault) )
  {
    sysStates.ASIC_RTD.online  = offline;
    sysStates.ASIC_RTD.state   = stopped;    

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.ASIC_RTD.fault != sysStates.ASIC_RTD.prior_fault) )
    {
      sysStates.ASIC_RTD.prior_fault = sysStates.ASIC_RTD.fault;
      logEvent(ASIC_RTDFault, 0, sysStates.ASIC_RTD.fault);
    }
  } else
  {
    sysStates.ASIC_RTD.prior_fault  = 0; 
  }

  if( (sysStates.ASIC_Chiller_RTD.fault) )
  {
    sysStates.ASIC_Chiller_RTD.online  = offline;
    sysStates.ASIC_Chiller_RTD.state   = stopped;

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.ASIC_Chiller_RTD.fault != sysStates.ASIC_Chiller_RTD.prior_fault) )
    {
      sysStates.ASIC_Chiller_RTD.prior_fault = sysStates.ASIC_Chiller_RTD.fault;
      logEvent(ASIC_Chiller_RTDFault, 0, sysStates.ASIC_Chiller_RTD.fault);
    }
  } else
  {
    sysStates.ASIC_Chiller_RTD.prior_fault = 0;
  }

  if( (sysStates.DDR_Chiller_RTD.fault) )
  {
    sysStates.DDR_Chiller_RTD.online  = offline;
    sysStates.DDR_Chiller_RTD.state   = stopped;    

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.DDR_Chiller_RTD.fault != sysStates.DDR_Chiller_RTD.prior_fault) )
    {
      sysStates.DDR_Chiller_RTD.prior_fault = sysStates.DDR_Chiller_RTD.fault;
      logEvent(DDR_Chiller_RTDFault, 0, sysStates.DDR_Chiller_RTD.fault);
    }
  } else
  {
    sysStates.DDR_Chiller_RTD.prior_fault = 0;
  }

  // 
  // check the DDR RTDs
  //
  if( (sysStates.DDR_RTD.fault) )
  {
    sysStates.DDR_RTD.online  = offline;
    sysStates.DDR_RTD.state   = stopped;    

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.DDR_RTD.fault != sysStates.DDR_RTD.prior_fault) )
    {
      sysStates.DDR_RTD.prior_fault = sysStates.DDR_RTD.fault;
      logEvent(DDR_RTDFault, 1, sysStates.DDR_RTD.fault);
    }
  } else
  {
    sysStates.DDR_RTD.prior_fault = 0;
  }

  if( (sysStates.DDR1_RTD.fault) )
  {
    sysStates.DDR1_RTD.online  = offline;
    sysStates.DDR1_RTD.state   = stopped;    

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.DDR1_RTD.fault != sysStates.DDR1_RTD.prior_fault) )
    {
      sysStates.DDR1_RTD.prior_fault = sysStates.DDR1_RTD.fault;
      logEvent(DDR_RTDFault, 2, sysStates.DDR1_RTD.fault);
    }
  } else
  {
    sysStates.DDR1_RTD.prior_fault = 0;
  }

  if( (sysStates.DDR2_RTD.fault) )
  {
    sysStates.DDR2_RTD.online  = offline;
    sysStates.DDR2_RTD.state   = stopped;    

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.DDR2_RTD.fault != sysStates.DDR2_RTD.prior_fault) )
    {
      sysStates.DDR2_RTD.prior_fault = sysStates.DDR2_RTD.fault;
      logEvent(DDR_RTDFault, 3, sysStates.DDR2_RTD.fault);
    }
  } else
  {
    sysStates.DDR2_RTD.prior_fault = 0;
  }


  // check the ASIC RTD chiller temperatures - log event if needed
  // if temp is high and not already in SHUTDOWN, log the event as we are
  // about to go into SHUTDOWN
  if( (HIGH_CHILLER_TEMP < sysStates.ASIC_Chiller_RTD.temperature) &&
      (SHUTDOWN != sysStates.sysStatus) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("ASIC RTD chiller temp too high logging event");
    #endif
    
    logEvent(ASIC_Chiller_RTDHot, 0, sysStates.ASIC_Chiller_RTD.temperature);
  }

  // check the ASIC RTD chiller temperatures - log event if needed
  // if temp is high and not already in SHUTDOWN, log the event as we are
  // about to go into SHUTDOWN
  if( (HIGH_CHILLER_TEMP < sysStates.DDR_Chiller_RTD.temperature) &&
      (SHUTDOWN != sysStates.sysStatus) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("DDR RTD chiller temp too high logging event");
    #endif
    
    logEvent(DDR_Chiller_RTDHot, 0, sysStates.DDR_Chiller_RTD.temperature);
  }

  
  //
  // special case check - if the chiller is not running and the ACUs are running, shutdown the ACUs
  //
#if defined(__USING_CHILLER__)
  if( (sysStates.chiller.state != running && ACUsRunning == true) ||
    (ACUMismatch || offline == sysStates.chiller.online || false== ACUsOnline) || false == RTDsRunning) {
#else
  if( (ACUMismatch || false== ACUsOnline || false == RTDsRunning) ) {
#endif
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Shutdown;
    retVal  = SHUTDOWN;

    if( (SHUTDOWN != sysStates.sysStatus) )
    {
      // disable the MAX31865 ISRs
      handleRTDISRs(false);

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
    //digitalWrite(BUTTON_LED, LOW);
  
    //
    // adjust the FAULT/NO-FAULT LEDs
    //


/* TODO: revisit this 
    // want this LED to blink
    if( (HIGH == digitalRead(FAULT_LED)) )
      digitalWrite(FAULT_LED, LOW);
    else
      digitalWrite(FAULT_LED, HIGH);
      
    digitalWrite(NO_FAULT_LED, LOW);
*/
  }
#if defined(__USING_CHILLER__)
  //
  // TODO: verify this fucking logic is right ..
  //
  else if( ((online == sysStates.chiller.online && true == ACUsOnline)
        && (running != sysStates.chiller.state && false == ACUsRunning) && (true == RTDsRunning)) )
#else
  else if( ((true == ACUsOnline) && (false == ACUsRunning) && (true == RTDsRunning)) )
#endif
  {
    //
    // READY - else everythig is online, ACUs are not running
    //
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Ready;
    retVal  = READY;

    if( (READY != sysStates.sysStatus) )
    {
      buttonOnOff         = false;
      currentButtonOnOff  = buttonOnOff;

/*
      //
      // adjust the button LED
      //
      digitalWrite(BUTTON_LED, LOW);
  
      //
      // adjust the FAULT/NO-FAULT LEDs
      //
      digitalWrite(FAULT_LED, LOW);
      digitalWrite(NO_FAULT_LED, HIGH);
*/
      // disable the MAX31865 ISRs
      handleRTDISRs(false);
    }
  } else
  {
    //
    // else the system is running
    //
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Running;
    retVal  = RUNNING;

    if( (RUNNING != sysStates.sysStatus) )
    {
      buttonOnOff     = true;
      currentButtonOnOff  = buttonOnOff;

      
/*      
      //
      // adjust the button LED
      //
      digitalWrite(BUTTON_LED, HIGH);
  
      //
      // adjust the FAULT/NO-FAULT LEDs
      //
      digitalWrite(FAULT_LED, LOW);
      digitalWrite(NO_FAULT_LED, LOW);
*/
      // enable the MAX31865 ISRs
      handleRTDISRs(true);
    }
  }

  sysStates.sysStatus = retVal;
  return(retVal);
}



/*
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

*/

//-------------------------------------------------------------
//
// - use global rs486Bus to write PVOF
// - read the value back
// - return the value read
//
// - TODO: how to return failure . . is 0xffff ok for failure ?
// - same for all these read and write functions
//
uint16_t writePVOF(uint8_t id, uint16_t val)
{
  uint16_t  retVal  = 0x0000;

  
  // do the write
  cmdResp writePVOF = RS485Bus.writeProcess(id, PVOF, val);
  if( (false == writePVOF.retCode()) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("writePVOF fail to id: "); Serial.println(id);
    #endif
    return(0xFFFF);
    
  #ifdef __DEBUG2_VIA_SERIAL__
  } else
  {
    Serial.print("writePVOF success to id: "); Serial.println(id);
  #endif
  }

  return(retVal);
}


//-------------------------------------------------------------
//
// - use global rs486Bus to read PVOF
// - return the value read
//
// - TODO: how to return failure . . is 0xffff ok for failure ?
// - same for all these read and write functions
//
uint16_t readPVOF(uint8_t id)
{
  cmdResp readPVOF = RS485Bus.readProcess(id, PVOF, 2);
  
  if( ((true == readPVOF.retCode()) && (0 < readPVOF.bufflen())) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" returns PVOF 0x"); Serial.println(readPVOF.buff()[0], 16);
    #endif
  
    return(readPVOF.buff()[0]);
    
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("readPVOF fail from id: "); Serial.println(id);
    #endif

    return(0xFFFF);
  }
}


uint16_t writeUNIT(uint8_t id, uint16_t val)
{
  uint16_t  retVal  = 0x0000;

  
  // do the write
  cmdResp writeUNIT = RS485Bus.writeProcess(id, UNIT, val);
  if( (false == writeUNIT.retCode()) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("writeUNIT fail to id: "); Serial.println(id);
    #endif
    return(0xFFFF);
    
  #ifdef __DEBUG2_VIA_SERIAL__
  } else
  {
    Serial.print("writeUNIT success to id: "); Serial.println(id);
  #endif
  }

  return(retVal);
}

//-------------------------------------------------------------
//
//
bool StartACU(uint8_t id)
{
  cmdResp writeENAB = RS485Bus.writeProcess(id, ENAB, MODE);

  if( (true == writeENAB.retCode()) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.println(" is started");
    #endif

    // test TODO: remove this
    ACURunning(id);
  
    return(true); // started
  }

  #ifdef __DEBUG_VIA_SERIAL__
  Serial.print("ACU: "); Serial.print(id); Serial.println(" is not started, unable to write cmd");
  #endif
  
  return(true); // not started, unable to write
}


//-------------------------------------------------------------
//
//
bool StopACU(uint8_t id)
{
  cmdResp writeENAB = RS485Bus.writeProcess(id, ENAB, OFF);

  if( (true == writeENAB.retCode()) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.println(" is stopped");
    #endif
  
    return(true); // not running
  }

  #ifdef __DEBUG_VIA_SERIAL__
  Serial.print("ACU: "); Serial.print(id); Serial.println(" is not stopped, unable to write cmd");
  #endif
  
  return(true); // is AT1, AT2, MPWR, SPON, PROG, HOLD
}


//-------------------------------------------------------------
//
//
bool ACURunning(uint8_t id)
{
  cmdResp readENAB = RS485Bus.readProcess(id, ENAB, 2);

  if( ((true == readENAB.retCode()) && (0 < readENAB.bufflen()) && (OFF == readENAB.buff()[0])) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" is not running: 0x"); Serial.println(readENAB.buff()[0], 16);
    #endif
  
    return(false); // not running
  }

  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.print("ACU: "); Serial.print(id); Serial.print(" is running: 0x"); Serial.println(readENAB.buff()[0], 16);
  #endif
  
  return(true); // is AT1, AT2, MPWR, SPON, PROG, HOLD
}


//-------------------------------------------------------------
//
// try to read the SV, return the retCode from the deviceHandler
// - false means there was a communication failure - device not present
// - true means there is no communication failure - device present
//
bool ACUPresent(uint8_t id)
{
  cmdResp readSV = RS485Bus.readProcess(id, SV, 2);
  return(readSV.retCode());
}


//-------------------------------------------------------------
//
//
bool GetACUTemp(uint8_t id, float* sv, float* pv)
{
  bool retVal = true;
  
  if( !(GetACUTempSV(id, sv)) )
    retVal = false;
    
  if( !(GetACUTempPV(id, pv)) )
    retVal = false;

  return(retVal);
}


//-------------------------------------------------------------
//
//
bool GetACUTempPV(uint8_t id, float* pv)
{
  uint16_t  temp  = 0x0000;


  // do the read, request 2 bytes back
  cmdResp readPVPVOF = RS485Bus.readProcess(id, PVPVOF, 2); // get response w/ PV and PVOF

  if( (false == readPVPVOF.retCode()) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("readPVPVOF fail");
    #endif
    return(false);
  } else
  {
    *pv = htons(*(reinterpret_cast<uint16_t*>(&readPVPVOF.buff()[0])));
    *pv = (float)*pv / (float)10;
    
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("readPVPVOF success");
    Serial.print("readProcess PVPVOF from id: "); Serial.print(id); Serial.print(" success, got "); Serial.print(readPVPVOF.bufflen()); Serial.println(" bytes returned");
    Serial.print("read value is: 0x"); Serial.print(*pv); Serial.print(", "); Serial.println(htons((*(reinterpret_cast<uint16_t*>(readPVPVOF.buff()[1])))));
    #endif
  }

  return(true);
}


//-------------------------------------------------------------
//
//
bool GetACUTempSV(uint8_t id, float* sv)
{


  // do the read, request 2 bytes back
  cmdResp readSVSVOF = RS485Bus.readProcess(id, SVSVOF, 2);  // get response w/ SV and SVOF
  if( (false == readSVSVOF.retCode()) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("readSVSVOF fail");
    #endif
    
    return(false);
    
  } else
  {
    *sv = htons(*(reinterpret_cast<uint16_t*>(&readSVSVOF.buff()[0])));
    *sv = (float)*sv / (float)10;

    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("readSV success");
    Serial.print("readProcess SVSVOF from id: "); Serial.print(id); Serial.print(" success, got "); Serial.print(readSVSVOF.bufflen()); Serial.println(" bytes returned");
    Serial.print("read value is: 0x"); Serial.print(*sv);  Serial.print(", "); Serial.println(htons((*(reinterpret_cast<uint16_t*>(readSVSVOF.buff()[1])))));
    #endif
  }

  return(true);
}


//
// write the SV register
//
bool SetACUSetPointValue(uint16_t id, float temp)
{
  bool      retCode = true;
  uint16_t  val, retVal;


  //
  // temp is a float, may have 2 decimal places
  // we only want 1 decimal place
  // then we multiply by 10 to no decimal
  //
  val = (uint16_t) ((float)temp * (float)10);

    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("SetACUSetPointValue got input temp: "); Serial.println(temp);
    Serial.print("atttempting to write this to register: 0x"); Serial.println(val,16);
    #endif
  
  // do the write
  cmdResp writeSV = RS485Bus.writeProcess(id, SV, val);
  if( (false == writeSV.retCode()) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("writeSV fail");
    #endif
  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.println("writeSV success");
  #endif
  }


  // do the read, request 2 bytes back
  cmdResp readSV = RS485Bus.readProcess(id, SV, 2);
  if( (false == readSV.retCode()) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("readSV fail");
    #endif
  } else
  {
    retVal = htons((*(reinterpret_cast<uint16_t*>(readSV.buff()))));
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("readSV success");
    Serial.print("readProcess SV from id: "); Serial.print(id); Serial.print(" success, got "); Serial.print(readSV.bufflen()); Serial.println(" bytes returned");
    Serial.print(" read value is: 0x"); Serial.println(retVal, 16);
    #endif

    if( (retVal != val) )
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.println("write and read values don't match, fail ...");
      #endif

      retCode = false;
    }
  }

  return(retCode);
}


void resetRTDState(RTDState& rtdState)
{
  rtdState.state          = running;
  rtdState.online         = online;
  rtdState.fault          = 0;
  rtdState.rtd            = 0;
  rtdState.temperature    = 0;
}


// don't log events here, those are handled in setSystemStatus
bool checkRTDStatus(void)
{
  bool retVal = true;

  
  //
  // per Rick .. return false (bad status) only if
  // - any chiller RTD is fault
  // - both RTDs not connected to the accuthermo are bad 'at the same time'
  //
  // - and if either of the chiller RTDs get above HIGH_CHILLER_TEMP, return bad status
  //
  if( (sysStates.ASIC_Chiller_RTD.fault || sysStates.DDR_Chiller_RTD.fault || sysStates.DDR_RTD.fault ||
      (sysStates.DDR1_RTD.fault && sysStates.DDR2_RTD.fault)) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("checkRTDStatus returning bad status");
    #endif
    
    retVal = false;
  }

  if( (HIGH_CHILLER_TEMP < sysStates.ASIC_Chiller_RTD.temperature) || 
      (HIGH_CHILLER_TEMP < sysStates.DDR_Chiller_RTD.temperature) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("RTD chiller temp too high returning bad status");
    Serial.print(sysStates.ASIC_Chiller_RTD.temperature,2); Serial.print(":"); Serial.println(sysStates.DDR_Chiller_RTD.temperature,2);
    #endif

    retVal = false;
  }

  return(retVal);
}


float setHotRTD(void)
{
  float hotRTD = sysStates.DDR_RTD.temperature; // this is the RTD connected to the DDR Accuthermo ( Accuthermo address 2 )


  if( (hotRTD < sysStates.DDR1_RTD.temperature) )
    hotRTD = sysStates.DDR1_RTD.temperature;

  if( (hotRTD < sysStates.DDR2_RTD.temperature) )
    hotRTD = sysStates.DDR2_RTD.temperature;

  sysStates.highRTDTemp = hotRTD;
  
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.print("DDR_RTD: "); Serial.print(sysStates.DDR_RTD.temperature);
  Serial.print(" DDR1_RTD: "); Serial.print(sysStates.DDR1_RTD.temperature);
  Serial.print(" DDR2_RTD: "); Serial.println(sysStates.DDR2_RTD.temperature);
  Serial.print(" hotRTD: "); Serial.println(sysStates.highRTDTemp);
  #endif


  return(sysStates.highRTDTemp);
}


//---------------------------------------------------
// the idea with this function to heavily favor the update of the SVPVOF in DDR accuthermo while in RUNNING state
//
// rather than have several millis() calculations to control the frequency of these :
// - chiller RTD sample rate
// - handle menu command rate
// - lcd update rate
// - getStatus rate
//
// the frequency of them will be controlled by the number of times the SVPVOF is written
//
// the SVPVOF needs to be written at 10Hz, so the the frequency of the other procoesses will
// be a function of the count of SVPVOF updates
//
// this is to try to guarantee 10Hz sample rate as best we can for the SVPVOF update frequency
//
// for example, the chiller RTS sample rate will be after 2 SVPVOF updates, i.e 2x per second
// the menu sample rate will be after 3 SVPVOF updates, i.e. 3x per second
// the lcd update rate will be after 4 SVPVOF updates, i.e .4x per second
// the getStatus rate will be after 5 updates, i.e. 5x per second
//
// this loop only runs while in the RUNNING state, ways out of this loop
//
// 1. the DoRTD10HzSamples() function will return false if the Accuthermo are found not running or not reachable, this 
// will cause loop exit
// 2. the getStatus() every 5 seconds will also cause loop exist as it can change the sysStates.sysStatus from RUNNING
// 3. the handleMenu() every 3 seconds can cause loop exist as it can cause sysStates.sysStatus state change
// 4. the push button can also cause sysStates.sysStatus change
//
void handleRunningState(void)
{
  uint8_t non_ddr_rtd_chiller_cnt = 0;
  uint8_t handle_menu_cnt         = 0;
  uint8_t handle_lcd_cnt          = 0;
  uint8_t get_status_cnt          = 0;

  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif
  
  while( (RUNNING == sysStates.sysStatus) )
  {
    if( (false == handleDDRRTDSamples()) )
      break;

    // increment counters
    non_ddr_rtd_chiller_cnt++; handle_menu_cnt++; handle_lcd_cnt++; get_status_cnt++;
    
    if( (NON_DDR_RTD_CHILLER_CNT == non_ddr_rtd_chiller_cnt) )
    {
      getNonDDRRTDData();   // the RTDs not involved in the DDR_RTD_10HzSample()
      non_ddr_rtd_chiller_cnt = 0;
    }

    if( (HANDLE_MENU_CNT == handle_menu_cnt) )
    {
      handleMsgs(CTRL_TIMEOUT_RUNNING_STATE);
      handle_menu_cnt = 0;
    }

    if( (HANDLE_LCD_CNT == handle_lcd_cnt) )
    {
      manageLCD(true);
      handle_lcd_cnt = 0;
    }

    if( (GET_STATUS_CNT == get_status_cnt) )
    {
      getStatus();
      get_status_cnt = 0;
    }
  }
}


//
// - function runs for 1 second
// - 
bool handleDDRRTDSamples(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif

  unsigned long startTime;
  uint16_t  pvof;

  
  for(int i = 0; i < DDR_RTDS_SAMPLES_PER_SEC; i++)
  {
    startTime = millis();

    // get the RTD temps for the DDR RTDs
    getDDRRTDData(false);


    // pick the hot RTD temperature
    setHotRTD(); 


    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("hotP "); Serial.println(sysStates.highRTDTemp);
    Serial.flush();
    #endif
  
    //
    // sysStates.DDR_RTD is the RTD connected to the Accuthermo Id 2 and
    // sysStates.DDR_RTD.temperature is the PV of Accuthermo Id 2
    //
    pvof = (uint16_t)(sysStates.highRTDTemp * 10) - (uint16_t)(sysStates.DDR_RTD.temperature * 10);
    
    if( (0xFFFF == writePVOF(DDR_RS485_ID, (pvof))) )
      return(false);

    do
    {
      // need 1 to ensure the code executes, i.e. actually delays if non-zero, i.e. if 0 does the delay call simply return ?
      delay(0);
    } while( (millis() - startTime) < timeBetweenSamples);
  }
  Serial.println("+");

  return(true);
}


int logEvent(uint16_t event_id, uint32_t inst, uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3)
{
  timeind   tstamp;
  elogentry event;


  // get the time
  getRTCTime(tstamp);

  // set up the eventlog record
  clrEventLogEntry(&event);
  event.id  = inst << 16 | event_id;
  event.ts  = tstamp;
  event.data[0] = d0;
  event.data[1] = d1;
  event.data[2] = d2;
  event.data[3] = d3;
  addEventLogEntry(&event);
}


uint16_t getRTCTime(timeind& rtcTime)
{
/*
    rtcTime->sec    = Controllino_GetSecond();
    rtcTime->min    = Controllino_GetMinute();
    rtcTime->hour   = Controllino_GetHour();
    rtcTime->mday   = Controllino_GetDay();
    rtcTime->mon    = Controllino_GetMonth();
    rtcTime->year   = Controllino_GetYear();
    rtcTime->wday   = Controllino_GetWeekDay();
    rtcTime->fill = 0x00; // fill to keep the buff length 

    // if any of them are 0xff (-1) the get failed
    if( (0xff == RTCTime->sec) || (0xff == RTCTime->min) || (0xff == RTCTime->hour) ||
      (0xff == RTCTime->mday) || (0xff == RTCTime->mon) || (0xff == RTCTime->year) ||
      (0xff == RTCTime->wday) )
    {    
      return(0);  // bad
    }    

    return(1);    // good
*/

// TODO: fix after the RTC decision has been made

    rtcTime.sec    = 0;
    rtcTime.min    = 0;
    rtcTime.hour   = 0;
    rtcTime.mday   = 0;
    rtcTime.mon    = 0;
    rtcTime.year   = 0;
    rtcTime.wday   = 0;
    rtcTime.fill = 0x00; // fill to keep the buff length 

  
  return(1);
}


bool calculateAndWritePVOF(void)
{
  uint16_t  pvof;


  if( (RUNNING == sysStates.sysStatus) )
  {
    //
    // float math .. ugh ..
    //
    pvof  = (uint16_t)(10 * ((float)sysStates.highRTDTemp - (float)sysStates.DDR_RTD.temperature));
  
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("highRTDTemp is set to : "); Serial.println(sysStates.highRTDTemp, 1);
    Serial.print("DDR_RTD PV is : "); Serial.println(sysStates.DDR_RTD.temperature, 1);
    Serial.print("pvof to write is : "); Serial.println(pvof, 1);
    #endif

    if( (0xffff == writePVOF(DDR_RS485_ID, pvof)) )
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.print("FAILED to writePVOF of "); Serial.println(pvof, 16);
      #endif

      return(false);
    }

    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("SUCCESS writePVOF of "); Serial.println(pvof, 16);
    #endif
    
  } else
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("not in RUNNING state, writing PVOF 0");
    #endif
    
    if( (0xffff == writePVOF(DDR_RS485_ID, 0)) )
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.println("FAILED to writePVOF of 0");
      #endif

      return(false);
    }   
  }

  return(true);
}


void enableRTD_DDR1_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  pinMode(RTD_DDR1_ISR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTD_DDR1_ISR_PIN), RTD_DDR1_ISR, FALLING);
}


void disableRTD_DDR1_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  detachInterrupt(digitalPinToInterrupt(RTD_DDR1_ISR_PIN));
  pinMode(RTD_DDR1_ISR_PIN, OUTPUT);
}


void RTD_DDR1_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  RTD_DDR1_DRDY = true;
}


void enableRTD_DDR2_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  pinMode(RTD_DDR2_ISR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTD_DDR2_ISR_PIN), RTD_DDR2_ISR, FALLING);
}


void disableRTD_DDR2_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  detachInterrupt(digitalPinToInterrupt(RTD_DDR2_ISR_PIN));
  pinMode(RTD_DDR2_ISR_PIN, OUTPUT);
}


void RTD_DDR2_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  RTD_DDR2_DRDY = true;
}


void enableASIC_Chiller_RTD_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  pinMode(ASIC_Chiller_RTD_ISR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ASIC_Chiller_RTD_ISR_PIN), ASIC_Chiller_RTD_ISR, FALLING);
}


void disableASIC_Chiller_RTD_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  detachInterrupt(digitalPinToInterrupt(ASIC_Chiller_RTD_ISR_PIN));
  pinMode(ASIC_Chiller_RTD_ISR_PIN, OUTPUT);
}


void ASIC_Chiller_RTD_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  ASIC_Chiller_RTD_DRDY = true;
}


void enableDDR_Chiller_RTD_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  pinMode(DDR_Chiller_RTD_ISR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DDR_Chiller_RTD_ISR_PIN), DDR_Chiller_RTD_ISR, FALLING);
}


void disableDDR_Chiller_RTD_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  detachInterrupt(digitalPinToInterrupt(DDR_Chiller_RTD_ISR_PIN));
  pinMode(DDR_Chiller_RTD_ISR_PIN, OUTPUT);
}


void DDR_Chiller_RTD_ISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  DDR_Chiller_RTD_DRDY = true;
}


void handleRTDISRs(bool on)
{
/*
  // disable the MAX31865 ISRs
  RTD_DDR1_DRDY          = false;
  RTD_DDR2_DRDY          = false;
  ASIC_Chiller_RTD_DRDY  = false;
  DDR_Chiller_RTD_DRDY   = false;

  if( (false == on) )
  {
    disableRTD_DDR1_ISR();
    disableRTD_DDR2_ISR();
    disableASIC_Chiller_RTD_ISR();
    disableDDR_Chiller_RTD_ISR();
  } else
  {
    enableRTD_DDR1_ISR();
    enableRTD_DDR2_ISR();
    enableASIC_Chiller_RTD_ISR();
    enableDDR_Chiller_RTD_ISR();
  }
*/
}
