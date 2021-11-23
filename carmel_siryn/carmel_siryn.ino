#include <SPI.h>
#include <Controllino.h>
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

/*

  //
  // always start the chiller
  //
  startChiller();
  
  //
  // get all statuses at startup
  // normally gotten via getStatus() which runs on a period
  //
  handleChillerStatus();
  handleTECStatus();
  getHumidityLevel();
*/
}


void loop(void)
{


  cmdResp writeSV = rs485Bus.writeProcess(ASIC_ID, SV, 0x0220);
  if( (false == writeSV.retCode()) )
  {
    Serial.println("writeProcess(ASIC_ID, SV, 0x0220) failed");
  } else
  {
    Serial.println("writeProcess(ASIC_ID, SV, 0x0220) success");
  }
  
  cmdResp readSV = rs485Bus.readProcess(ASIC_ID, SV, 2);
  if( (false == readSV.retCode()) )
  {
    Serial.print("readProcess(ASIC_ID, SV, 2) failed");
  } else
  {
    val = htons((*(reinterpret_cast<uint16_t*>(readSV.buff()))));
    Serial.print("readProcess(ASIC_ID, SV, 2) success, got "); Serial.print(readSV.bufflen()); Serial.println(" bytes returned");
    Serial.print(" read value is: "); Serial.println(val, 16);
  }

  delay(1000);
/*
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
*/
}


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
//  configureButton();

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
  // chiller starts offline until queried via getStatus()
  states.chiller.online    = offline;
  states.chiller.state     = stopped;
  states.chiller.temperature = 0;
  states.chiller.setpoint  = 0;

  // sensor starts offline until discovered to be online via getStatus()
  states.sensor.humidity   = 0;
  states.sensor.threshold  = HUMIDITY_THRESHOLD;
  states.sensor.online     = offline;
  states.sensor.sampleData.index = 0;
  for(int i = 0; i < MAX_HUMIDITY_SAMPLES; i++)
    states.sensor.sampleData.sample[i] = 0.0;

  // tecs starts offline until discovered to be online via getStatus()
  for(int i = MIN_TEC_ADDRESS; i <= MAX_TEC_ADDRESS; i++)
  {
    states.tec[(i - MIN_TEC_ADDRESS)].online      = offline;
    states.tec[(i - MIN_TEC_ADDRESS)].state       = stopped;
    states.tec[(i - MIN_TEC_ADDRESS)].setpoint    = 0;
    states.tec[(i - MIN_TEC_ADDRESS)].temperature   = 0;
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
  sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]     = tec_Stopped;
  sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]   = chiller_Stopped;
  sysStates.lcd.lcdFacesIndex[HUMIDITY_NRML_OFFSET]  = sensor_humidityAndThreshold;

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


/*
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

    //
    // update the LCD screen
    //
    // special case the humidity sensor - show humidity status in case of:
    // - humidity is too high
    // - SHT sensor failure
    // in either case the system will be in shutdown mode already or very soon
    //
    if( (humidityHigh()) )
    {
      // high or sensor failure
      if( (offline == sysStates.sensor.online) )
        lcdFaces[sensor_Failure]();
      else
        lcdFaces[sensor_HighHumidity]();

    } else // paint the current LCD screen
    {
      lcdFaces[sysStates.lcd.lcdFacesIndex[(sysStates.lcd.index)]]();
    }

    //
    // reload the count down timer
    //
    sysStates.lcd.prior_millis  = millis();

    //
    // and advance the index to the next message
    //
    sysStates.lcd.index =
      (((sysStates.lcd.index + 1) >= MAX_LCD_MSGS) ? 0 : (sysStates.lcd.index + 1));
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


bool startSHTSensor(void)
{
  bool retVal = false;

  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  if( (sht.init()) && (sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM)) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println("started SHT sensor");
    Serial.flush();
    #endif

    
    sysStates.sensor.online = online;
    sysStates.lcd.lcdFacesIndex[HUMIDITY_NRML_OFFSET]  = sensor_humidityAndThreshold;
    retVal  = true;
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" ERROR: unable to start sensor");
    Serial.flush();
    #endif

    // update the sysStates
    sysStates.sensor.online = offline;

    // update the LCD status for the sensor
    sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET]  = sensor_Failure;

    retVal  = false;
  }

  return(retVal);
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
  // if humidity is too high, or there is a humidity sensor failure,
  // do not start the chiller - return failure
  //
  if( (humidityHigh()) )
    return(false);  
  
  //
  // paint 'Starting' on LCD
  //
  lcd_starting();


  //
  // start the chiller and the TECs
  //
  if( !(startChiller()) )
    retVal  = false;

  // only start the TECs is the chiller is running
  else if( !(startTECs()) )
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
  // get the chiller and TEC's status every GET_STATUS_INTERVAL seconds
  //
  if( (GET_STATUS_INTERVAL < (currentGetStatusTime - lastGetStatusTime)) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("- check TECs and chiller --------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif
    
    lastGetStatusTime = currentGetStatusTime;
    handleChillerStatus();
    handleTECStatus();
  }


  //
  // get the chiller and TEC's status every GET_HUMIDITY_INTERVAL seconds
  //
  if( (GET_HUMIDITY_INTERVAL < (currentGetStatusTime - lastGetHumidityTime)) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("- check humidity ----------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    lastGetHumidityTime = currentGetStatusTime;
    getHumidityLevel();
    
    //
    // if humidity too high, shutdown the chiller
    //
    // TODO: shutdown just the chiller or the TECs too ?
    //
    if( (humidityHigh()) )
    { 
      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.println("- !!! humidity is high !!!! ----------------------");
      Serial.print("sysStates.sysStatus is: "); Serial.println(sysStates.sysStatus);
      Serial.println(__PRETTY_FUNCTION__);
      #endif

      // disable the button ISR, the uC queues up the
      // interrupts, so if button is presssed while system is
      // shutdown due to high humidity, that button press could
      // cause the system to start up when the humidity failure clears
      disableButtonISR();

      // stop the chiller, stop the TECs
      if( (SHUTDOWN != sysStates.sysStatus) )
        shutDownSys(false);
    } else
    {
      enableButtonISR();
    }
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
  if( (stopChillerCmd) || (humidityHigh()) )
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
      
  //
  // turn off the TECs - not checking return value here as we are dying anyway ??
  //
  for(uint8_t Address = MIN_TEC_ADDRESS; (Address <= MAX_TEC_ADDRESS); Address++)
  {
    if( !(ms.StopTEC(Address)) )
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
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  bool retVal = true;


  //
  // if humidity is too high, or there is a humidity sensor failure,
  // do not start the chiller - return failure
  //
  if( (humidityHigh()) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" WARNING: not starting chiller, humidity too high or sensor failure");
    #endif

    retVal  = false;

  } else
  {
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
  }

  return(retVal);
}


// ---------------------------------------
// find all TECs, there should be 3
// - verify their addresses by fetching hw version
//      expecting ID s2, 3, and 4
// - turn them 'on'
// return true if all these things happen
//
bool startTECs(void)
{
  bool  retVal    = true;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  //
  // only start the TECs if the chiller is running
  // only in the SHUTDOWN state is the chiller not running (fucking Yoda?)
  //
  if( (running == sysStates.chiller.state) )
  {
    //
    // expecting the addresses to be 1, 2, and 3
    // the instance for these commands to be 1
    //
    // initialize the TEC LCD state
    for(uint8_t Address = MIN_TEC_ADDRESS; Address <= MAX_TEC_ADDRESS; Address++)
    {
      if( !(ms.StartTEC(Address)) )
      {
        retVal = false;
        
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" unable to start TEC: ");
        Serial.println(Address, DEC);
        Serial.flush();
        #endif
      }
    }

    //
    // let handleTECStatus() derive the TEC's status
    //
    handleTECStatus();

    //
    // update the system state
    //
    setSystemStatus();
    
  } else
  {
    retVal = false;
    
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(": not starting TECs chiller is not running");
    Serial.flush();
    #endif
  }
  
  return(retVal);
}


bool stopTECs(void)
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
  sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]   = tec_Stopped;
  sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]   = no_Status;
  for(uint8_t Address = MIN_TEC_ADDRESS; (Address <= MAX_TEC_ADDRESS); Address++)
  {
    if( !(ms.StopTEC(Address)) )
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.print("stopTECs unable to stop TEC ");
      Serial.println(Address, DEC);
      #endif

      retVal    = false;

      //
      // update the LCD state at least
      //
      sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]  = tec_ComFailure;
      sysStates.tec[(Address - MIN_TEC_ADDRESS)].online       = offline;
      sysStates.tec[(Address - MIN_TEC_ADDRESS)].state        = stopped;  // we don't know, or don't change this

    } else
    {
      sysStates.tec[(Address - MIN_TEC_ADDRESS)].online   = online;
      sysStates.tec[(Address - MIN_TEC_ADDRESS)].state  = stopped;
    }
  }

  return(retVal);
}


bool setTECTemp(uint16_t tecAddress, float temp)
{
  bool retVal = false;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  if( (MAX_TEC_ADDRESS >= tecAddress) )
  {
    if( (ms.SetTECTemp(tecAddress, temp)) )
    {
      retVal  = true;
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__);
      Serial.print(" ERROR: failed to set temp for TEC at addr: ");
      Serial.println(tecAddress);
      Serial.flush();
    #endif
    }
  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__);
    Serial.print(" ERROR: failed to set temp for TEC addr out of range: ");
    Serial.println(tecAddress);
    Serial.flush();
  #endif
  }

  return(retVal);
}


bool setChillerSetPoint(char* temp)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  Serial.flush();
  #endif


  return(chiller.SetSetPoint(temp));
}


void handleMsgs(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  if( (currentButtonOnOff != buttonOnOff) )
  {
    if( (!humidityHigh()) )
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
        case startUpCmd:        // start the chiller, TECs, and sensor
        {
          handleStartUpCmd();
          break;
        }
  
        case shutDownCmd:         // shutdown the chiller, TECs, and sensor
        {
          handleShutDownCmd();
          break;
        }
  
        case getStatusCmd:         // fetch the status of chiller, all TECs, and humidity sensor
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
  
        case setTECTemperature:      // target TEC m_address and temp
        {
          handleSetTECTemperature();
          break;
        };
  
        case getTECTemperature:      // target TEC m_address and temp
        {
          handleGetTECTemperature(false);
          break;
        };
  
        case getTECObjTemperature:
        {
          handleGetTECTemperature(true);
          break;
        };
  
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
  
        case setChillerTemperature:    // target TEC m_address and temp
        {
          handleSetChillerTemperature();
          break;
        };
  
        case getChillerTemperature:    // target TEC m_address and temp
        {
          handleGetChillerTemperature(true);
          break;
        };
  
        case getChillerObjTemperature:    // target TEC m_address and temp
        {
          handleGetChillerTemperature(false);
          break;
        };
  
        case getTECInfoMsg:
        {
          handlGetTECInfo();
          break;
        };
  
        case enableTECs:         // turn on all TECs
        {
          handleEnableTECs();
          break;
        };
  
        case disableTECs:        // turn off all TECs
        {
          handleDisableTECs();
          break;
        };
  
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


void lcd_tecsRunning(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("TEC run     ");
  lcd.setCursor(9,0);
  lcd.print(sysStates.tec[0].setpoint,1);
  lcd.setCursor(0,1);
  lcd.print(sysStates.tec[1].setpoint,1);
  lcd.setCursor(9,1);
  lcd.print(sysStates.tec[2].setpoint,1);
  lcd.display();
}


void lcd_tecsStopped(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("TEC stop    ");
  lcd.setCursor(9,0);
  lcd.print(sysStates.tec[0].setpoint,1);
  lcd.setCursor(0,1);
  lcd.print(sysStates.tec[1].setpoint,1);
  lcd.setCursor(9,1);
  lcd.print(sysStates.tec[2].setpoint,1);
  lcd.display();
}


void lcd_tecComFailure(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("TEC COMM");
  lcd.setCursor(0,1);
  lcd.print("FAILURE");
  lcd.display();
}


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


void lcd_humidityAndThreshold(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  lcd.noDisplay();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Humidity:     %");
  lcd.setCursor(11,0);
  lcd.print(sysStates.sensor.humidity);
  lcd.setCursor(0,1);
  lcd.print("Threshold:    %");
  lcd.setCursor(11,1);
  lcd.print(sysStates.sensor.threshold);
  lcd.display();
}


void lcd_highHumidity(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(0,0);
  lcd.print("*** HUMIDITY ***");
  lcd.setCursor(0,1);
  lcd.print("**** ALERT *****");
  lcd.display();
}


void lcd_sensorFailure(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


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
      // start the TECs, chiller, sensor ...
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
        ((humidityHigh()) ? 1 : 0), // humidity alert
        ((true == TECsRunning()) ? 1 : 0),                  // TECs running
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



void handleSetHumidityThreshold(void)
{
  setHumidityThreshold_t* psetHumidityThreshold = reinterpret_cast<setHumidityThreshold_t*>(cp.m_buff);
  uint16_t        respLength;
  uint16_t        result = 0;


  //
  // verify the received packet, here beause this is a setHumidityThreshold
  // check this is my address and the CRC is correct
  //
  if( (ntohs(psetHumidityThreshold->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_setHumidityThreshold_t,
      ntohs(psetHumidityThreshold->crc), ntohs(psetHumidityThreshold->eop))) )
    {
      //
      // pick up the new threshold if not running
      //
      if( (RUNNING != sysStates.sysStatus) )
      {
        sysStates.sensor.threshold  = ntohs(psetHumidityThreshold->threshold);
        result = 1;
        #ifdef __DEBUG2_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print( " set humidity threshold to: ");
        Serial.println(sysStates.sensor.threshold);
        #endif
      }
      
      //
      // use the sysStates conentent to respond, send back the received seqNum
      //
      respLength = cp.Make_setHumidityThresholdResp(cp.m_peerAddress, cp.m_buff, result,
        psetHumidityThreshold->header.seqNum
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
      Serial.println(ntohs(psetHumidityThreshold->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(psetHumidityThreshold->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleGetHumidityThreshold(void)
{
  getHumidityThreshold_t* pgetHumidityThreshold = reinterpret_cast<getHumidityThreshold_t*>(cp.m_buff);
  uint16_t        respLength; 


  //
  // verify the received packet, here beause this is a getHumidityThresholdCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetHumidityThreshold->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getHumidityThreshold_t,
              ntohs(pgetHumidityThreshold->crc), ntohs(pgetHumidityThreshold->eop))) )
    {
      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print( " returning humidity threshold: ");
      Serial.println(sysStates.sensor.threshold);
      #endif
      
      //
      // use the sysStates conentent to respond, send back the received seqNum
      //
      respLength = cp.Make_getHumidityThresholdResp(cp.m_peerAddress, cp.m_buff,
        sysStates.sensor.threshold,
        pgetHumidityThreshold->header.seqNum
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
      Serial.println(ntohs(pgetHumidityThreshold->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetHumidityThreshold->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleGetHumidity(void)
{
  getHumidity_t* pgetHumidity = reinterpret_cast<getHumidity_t*>(cp.m_buff);
  uint16_t     respLength; 


  //
  // verify the received packet, here beause this is a getHumidityCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetHumidity->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getHumidity_t,
            ntohs(pgetHumidity->crc), ntohs(pgetHumidity->eop))) )
    {
      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print( " returning humidity: ");
      Serial.println(sysStates.sensor.humidity);
      #endif
      
      //
      // use the sysStates content to respond, send back the received seqNum
      //
      respLength = cp.Make_getHumidityResp(cp.m_peerAddress, cp.m_buff,
        sysStates.sensor.humidity,
        pgetHumidity->header.seqNum
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
      Serial.println(ntohs(pgetHumidity->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetHumidity->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleSetTECTemperature(void)
{
  setTECTemperature_t* psetTECTemperature = reinterpret_cast<setTECTemperature_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  tecAddress;
  uint16_t  result = 0;
  float     setPoint;


  //
  // verify the received packet, here beause this is a setTECTemperatureCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(psetTECTemperature->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_setTECTemperature_t,
              ntohs(psetTECTemperature->crc), ntohs(psetTECTemperature->eop))) )
    {
      //
      // pick up the new temperature
      //
      // if the TECs addresses is out of range, send back failure
      //
      setPoint = atof(reinterpret_cast<char*>(psetTECTemperature->temperature));
      tecAddress = ntohs(psetTECTemperature->tec_address);

      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print( " found setPoint:tecAddress ");
      Serial.print(setPoint,2);
      Serial.print(":");
      Serial.println(tecAddress);
      Serial.flush();
      #endif

      if( (MAX_TEC_ADDRESS >= tecAddress) )
      {
        if( (setTECTemp(tecAddress, setPoint)) )
        {
          #ifdef __DEBUG2_VIA_SERIAL__
          Serial.print(__PRETTY_FUNCTION__); Serial.print( " success set temp for TEC: ");
          Serial.println(tecAddress);
          Serial.flush();
          #endif
          result  = 1;

          //
          // TODO: DO NOT update the TEC's set point temperature ! 
          // rather, getStatus() queries the Meerstetter device
          //
          // TODO: document this behavior, update may not show on LCD
          // for 'a few seconds' until getStatus() queries the device
          //
          //sysStates.tec[(tecAddress - MIN_TEC_ADDRESS)].setpoint = setPoint;

        } else
        {
          #ifdef __DEBUG_VIA_SERIAL__
          Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: unable to temp for TEC: ");
          Serial.println(tecAddress);
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
        Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: tec_address out of range: ");
        Serial.println(tecAddress);
        #endif
      }

      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print( " sending back response for TEC: ");
      Serial.println(tecAddress);
      #endif

      Serial.flush();
      
      //
      // use the sysStates conentent to respond, send back the received seqNum
      //
      respLength = cp.Make_setTECTemperatureResp(cp.m_peerAddress, cp.m_buff,
        tecAddress, result, psetTECTemperature->header.seqNum
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
      Serial.println(ntohs(psetTECTemperature->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(psetTECTemperature->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleGetTECTemperature(bool getObjTemp)
{
  getTECTemperature_t* pgetTECTemperature = reinterpret_cast<getTECTemperature_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  tecAddress;
  uint16_t  result;


  //
  // verify the received packet, here beause this is a getTECTemperatureCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetTECTemperature->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getTECTemperature_t,
                ntohs(pgetTECTemperature->crc), ntohs(pgetTECTemperature->eop))) )
    {
      //
      // if the TECs addresses is out of range, send back failure
      //
      tecAddress = ntohs(pgetTECTemperature->tec_address);

      if( (MAX_TEC_ADDRESS < tecAddress) )
      {
        //
        // send back failure
        //
        result  = 0;

        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print( " ERROR: tec_address out of range: ");
        Serial.println(tecAddress);
        #endif
      } else
      {
        result = 1;
      }
      
      //
      // use the sysStates conentent to respond, send back the received seqNum
      //
      if( (true == getObjTemp) )
        respLength = cp.Make_getTECObjTemperatureResp(cp.m_peerAddress, cp.m_buff,
          tecAddress, result, sysStates.tec[(tecAddress - MIN_TEC_ADDRESS)].temperature,
          pgetTECTemperature->header.seqNum);
      else
        respLength = cp.Make_getTECTemperatureResp(cp.m_peerAddress, cp.m_buff,
          tecAddress, result, sysStates.tec[(tecAddress - MIN_TEC_ADDRESS)].setpoint,
          pgetTECTemperature->header.seqNum);

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
        Serial.println(sysStates.tec[(tecAddress - MIN_TEC_ADDRESS)].temperature, 2);
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pgetTECTemperature->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetTECTemperature->header.address.address));
    Serial.flush();
  #endif
  }
}


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
      // start the TECs, chiller, sensor ...
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
      // stop chiller is effectively a shutDownSys() as the TECs cannot
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


void handlGetTECInfo(void)
{
  getTECInfoMsg_t*   pgetTECInfo = reinterpret_cast<getTECInfoMsg_t*>(cp.m_buff);
  uint16_t    respLength;
  uint16_t    result = 0;
  uint32_t    deviceType  = 0;
  uint32_t    hwVersion   = 0;
  uint32_t    fwVersion   = 0;
  uint32_t    serialNumber= 0;


  if( (ntohs(pgetTECInfo->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getTECInfoMsg_t,
                ntohs(pgetTECInfo->crc), ntohs(pgetTECInfo->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      if( (getTECInfo(ntohs(pgetTECInfo->tec_address), &deviceType,
                &hwVersion, &fwVersion, &serialNumber)) )
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to startTECs");
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_getTECInfoMsgResp(cp.m_peerAddress, cp.m_buff, htons(pgetTECInfo->tec_address), result,
                deviceType, hwVersion, fwVersion, serialNumber, pgetTECInfo->header.seqNum);

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
      Serial.println(ntohs(pgetTECInfo->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetTECInfo->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleEnableTECs(void)
{
  enableTECs_t* penableTECs = reinterpret_cast<enableTECs_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a enableTECsCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(penableTECs->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_enableTECs_t,
                ntohs(penableTECs->crc), ntohs(penableTECs->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      if( (startTECs()) )
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to startTECs");
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_enableTECsResp(cp.m_peerAddress, cp.m_buff,
        result, penableTECs->header.seqNum
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
      Serial.println(ntohs(penableTECs->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(penableTECs->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleDisableTECs(void)
{
  disableTECs_t* pdisableTECs = reinterpret_cast<disableTECs_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a disableTECsCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pdisableTECs->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_disableTECs_t,
                ntohs(pdisableTECs->crc), ntohs(pdisableTECs->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      if( (stopTECs()) )
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: failed to stopTECs");
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_disableTECsResp(cp.m_peerAddress, cp.m_buff,
        result, pdisableTECs->header.seqNum
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
      Serial.println(ntohs(pdisableTECs->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pdisableTECs->header.address.address));
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





bool getHumidityLevel(void)
{
  bool retVal = true;


  //
  // update status and take a reading
  //
  if( (sysStates.sensor.online == offline) )
    digitalWrite(FAULT_LED, HIGH);
    
  sysStates.sensor.online = online;

  if( (sht.readSample()) )
  {
    //
    // update the sysStates for humidity - take an average to smooth spikes
    //
    sysStates.sensor.sampleData.sample[sysStates.sensor.sampleData.index] = sht.getHumidity();
    sysStates.sensor.humidity = 0;  // this will eventually be an average when have enough samples
    for(int i = 0; i < MAX_HUMIDITY_SAMPLES; i++)
    {
      if( ( 0 != sysStates.sensor.sampleData.sample[i]) )
        sysStates.sensor.humidity += (sysStates.sensor.sampleData.sample[i]);
      else
      {
        sysStates.sensor.humidity = 0;
        break;
      }
    }

    if( (0 != sysStates.sensor.humidity) )
    {
      // have enough samples, make an average
      sysStates.sensor.humidity /= (float)MAX_HUMIDITY_SAMPLES;

      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" took an average for humidity: ");
      Serial.print(sysStates.sensor.humidity, 2); Serial.println("%"); Serial.flush();
      #endif

    } else
      // not enough samples - take the raw reading
      sysStates.sensor.humidity = sysStates.sensor.sampleData.sample[sysStates.sensor.sampleData.index];

    //
    // update the index for the next reading
    //
    sysStates.sensor.sampleData.index += 1;

    if( (sysStates.sensor.sampleData.index >= MAX_HUMIDITY_SAMPLES) )
      sysStates.sensor.sampleData.index = 0;

    if (humidityHigh())
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: found high humidity: ");
      Serial.print(sysStates.sensor.humidity, 2); Serial.println("%"); Serial.flush();
      #endif

      // update the LCD
      sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET]   = sensor_HighHumidity;
      retVal  = false;
    } else
    {
      sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET]   = no_Status;
    }
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__); Serial.println(" ERROR: sensor not on-line");
    #endif

    //
    // update sysStates
    //
    sysStates.sensor.online = offline;

    // update the LCD
    sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET]   = sensor_Failure;

    retVal = false;
  }

  return(retVal);
}


bool TECsRunning(void)
{
  bool retVal = true;


  for(int i = MIN_TEC_ADDRESS; i <= MAX_TEC_ADDRESS; i++)
  {
    if( (running != sysStates.tec[(i - MIN_TEC_ADDRESS)].state) )
      retVal = false;
  }

  return(retVal);
}


void enableRotaryEncoder(void)
{
  // encoder
  pinMode(pinA, INPUT);
  digitalWrite(pinA, HIGH);
  pinMode(pinB, INPUT);
  digitalWrite(pinB, HIGH);

  attachInterrupt(digitalPinToInterrupt(pinA), digitalEncoderISR, LOW);

  virtualPosition = sysStates.sensor.threshold;
}


void disableRotaryEncoder(void)
{
  detachInterrupt(digitalPinToInterrupt(pinA));
}


void digitalEncoderISR(void)
{
  static unsigned long  lastInterruptTime = 0;
  unsigned long     interruptTime   = millis();


  // pick up the system value - may have changed from ctrl PC
  virtualPosition = sysStates.sensor.threshold;

  if( (5 < (interruptTime - lastInterruptTime)) )
  {
    // read the pins
    if( (LOW == digitalRead(pinB)) )
      --virtualPosition;
    else
      ++virtualPosition;

    virtualPosition = min(100, max(0, virtualPosition));

    lastInterruptTime = interruptTime;

    // update the system variable
    sysStates.sensor.threshold = virtualPosition;

    // update the LCD
    lcd.noDisplay();
    lcd.clear(); lcd.home();
    lcd.setCursor(0,0);  lcd.print("Humidity");
    lcd.setCursor(0,1);  lcd.print("Threshold: ");
    lcd.setCursor(11,1); lcd.print(virtualPosition);
    lcd.display();
  }
}


void handleChillerStatus(void)
{
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
    // update sysStates with what was feteched from GetAllChillerInfo
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
}


void handleTECStatus(void)
{
  bool  TECsOnline  = true;
  bool  TECsRunning = true;


  //
  // check all TECs running - get Device Status, possible status are
  //
  // assume they are running, if one if found not running, mark the group
  // as not running
  //
  // before going to chat w/ the TECs - enable the FAUL_LED so it is on 
  // when there is a fault and on while we could be timing out chatting
  // to a dead TEC
  for(uint8_t Address = MIN_TEC_ADDRESS; (Address <= MAX_TEC_ADDRESS); Address++)
  {
    if( (sysStates.tec[(Address - MIN_TEC_ADDRESS)].online == offline) )
    {
    digitalWrite(FAULT_LED, HIGH);
    break;
    }
  }

  for(uint8_t Address = MIN_TEC_ADDRESS; (Address <= MAX_TEC_ADDRESS); Address++)
  {
    sysStates.tec[(Address - MIN_TEC_ADDRESS)].state      = running;
    sysStates.tec[(Address - MIN_TEC_ADDRESS)].online     = online;

    //
    // always fetch the TEC's set point and object temperatures
    //
    if( !(ms.GetTECTemp(Address,
      &sysStates.tec[(Address - MIN_TEC_ADDRESS)].setpoint,
      &sysStates.tec[(Address - MIN_TEC_ADDRESS)].temperature)) )
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:TEC ");
      Serial.print(Address, DEC); Serial.println(" unable to get temps");
      Serial.flush();
      #endif

      sysStates.tec[(Address - MIN_TEC_ADDRESS)].online   = offline;
      sysStates.tec[(Address - MIN_TEC_ADDRESS)].state  = stopped;
      #ifdef __DEBUG2_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" found ");
      Serial.print(sysStates.tec[(Address - MIN_TEC_ADDRESS)].setpoint, 2);
      Serial.print(" : "); Serial.println(sysStates.tec[(Address - MIN_TEC_ADDRESS)].temperature, 2);
      Serial.flush();
      #endif
    }

    if( !(ms.TECRunning(Address)) )
    {
      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: TEC ");
      Serial.print(Address); Serial.println(" is not running");
      Serial.flush();
      #endif

      //
      // keep track of whether all TECs are running
      //
      TECsRunning = false;

      // update sysStates
      sysStates.tec[(Address - MIN_TEC_ADDRESS)].state = stopped;

      //
      // check if we can still communicate with TECs
      //
      if( !(ms.TECPresent(Address)) )
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:TEC ");
        Serial.print(Address, DEC); Serial.println(" is not on-line");
        Serial.flush();
        #endif

        TECsOnline  = false;

        // update the sysStates
        sysStates.tec[(Address - MIN_TEC_ADDRESS)].online = offline;
      }
    }
  }


  //
  // if one TEC is down or bad, the overall status is bad
  //
  if( !(TECsOnline) )
    sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]   = tec_ComFailure;
  else
    sysStates.lcd.lcdFacesIndex[TEC_FAIL_OFFSET]   = no_Status;

  if( !(TECsRunning) )
    sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]   = tec_Stopped;
  else
    sysStates.lcd.lcdFacesIndex[TEC_NRML_OFFSET]   = tec_Running;
}


// careful . . tec_address is a
bool getTECInfo(uint8_t tec_address, uint32_t* deviceType, uint32_t* hwVersion,
                    uint32_t* fwVersion, uint32_t* serialNumber)
{
  bool retVal = true;


  if( !(ms.GetTECInfo(tec_address, deviceType, hwVersion, fwVersion, serialNumber)) )
  {
    retVal  = false;
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println(__PRETTY_FUNCTION__); Serial.println(" getTECInfo failed");
    #endif
  }

  return(retVal);
}


systemStatus setSystemStatus(void)
{
  systemStatus  retVal    = RUNNING;
  bool      TECsOnline  = true;
  bool      TECsRunning = true;
  bool      TECMismatch = false;
  

  //
  // use the humidity sensor, chiller, and TECs status
  //
  // the other 'bad' status for the other components shoudl already be
  // updated
  //
  // SHUTDOWN - if high humidity, or one thing is offline, or the chiller is not running - shutdown
  //


  //
  // accumulate the TECs status'
  //
  for(int i = MIN_TEC_ADDRESS; i <= MAX_TEC_ADDRESS; i++)
  {
    if( (offline == sysStates.tec[(i - MIN_TEC_ADDRESS)].online) )
      TECsOnline = false;

    if( (stopped == sysStates.tec[(i - MIN_TEC_ADDRESS)].state) )
      TECsRunning = false;

    //
    // check for TEC on/off line or running mismatch, shutdown if present
    // can/could happen if a TEC fails or is reset
    //
    if( (sysStates.tec[(i - MIN_TEC_ADDRESS)].online != sysStates.tec[0].online) ||
      (sysStates.tec[(i - MIN_TEC_ADDRESS)].state != sysStates.tec[0].state) )
    {
      TECMismatch = true;
    }
  }

  //
  // special case check - if the chiller is not running and the TECs are running, shutdown the TECs
  //
  if( (sysStates.chiller.state != running && TECsRunning == true) ||
    (humidityHigh()) ||
    (TECMismatch || offline == sysStates.sensor.online || offline == sysStates.chiller.online || false== TECsOnline) ) {
      
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Shutdown;
    retVal  = SHUTDOWN;

    if( (SHUTDOWN != sysStates.sysStatus) )
    {
      buttonOnOff     = false;
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
  
    // enable the humidity threshold knob
    enableRotaryEncoder();

  //
  // READY - else everythig is online, chiller is running, TECs are not running
  //  
  } else if( ((online == sysStates.sensor.online &&
         online == sysStates.chiller.online &&
         true   == TECsOnline)
        && (running != sysStates.chiller.state || false == TECsRunning)) )
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
  
      // enable the humidity threshold knob
      enableRotaryEncoder();
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
   
      // disable the humidity threshold knob
      disableRotaryEncoder();
    }
  }

  sysStates.sysStatus = retVal;
  return(retVal);
}


void configureButton(void)
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);     // TODO: INPUT_PULLUP or just INPUT

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

  if( ! (humidityHigh()) )
  {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  }
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


bool humidityHigh(void)
{
  bool retVal = false;


  if( ((sysStates.sensor.humidity > sysStates.sensor.threshold) 
    || (offline == sysStates.sensor.online)) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" WARNING: humidity too high or sensor failure");
    #endif

    retVal = true;
  }

  return(retVal);
}


void setInitialHumidityThreshold(void)
{
  //
  // fetch current humidity 2x w/ breif delay inbetween
  //
  getHumidityLevel();

  delay(1000);
  getHumidityLevel();

  //
  // set threshold to ambient + 10
  //
  sysStates.sensor.threshold = sysStates.sensor.humidity + HUMIDITY_BUFFER;
}

*/
