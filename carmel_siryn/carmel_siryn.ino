#include "carmel_siryn.h"


void setup(void)
{
  //
  // start the system components and Serial port if running debug
  //
  initSystem();

  #ifdef __USING_HUMIDITY__
  //
  // set the humidity threshold to ambient + 10%
  //
  setInitialHumidityThreshold();
  #endif

  //
  // banner - used to easily find system restarts in log file
  //
  #if defined __DEBUG_VIA_SERIAL__ || defined __DEBUG2_VIA_SERIAL__ || defined __DEBUG_RTD_READS__
  Serial.println(" -----------------------------------------------------------"); Serial.flush();
  Serial.println(" -------------------------> setup() <-----------------------"); Serial.flush();
  Serial.println(" -----------------------------------------------------------"); Serial.flush();
  #endif
 
  //
  // always start the chiller
  //
  #ifdef __USING_CHILLER__
  startChiller();
  #endif
  
  //
  // shutdown on boot up, get to a known state
  //
  shutDownSys(false); // false is 'do not stop chiller' ( if chiller is present )

  //
  // get the adafruits into known good state too
  //
  configureAdafruitsBySysteState(SHUTDOWN);


  //
  // these are normally gotten via getStatus() which runs on a period
  // at start up, i.e. now, the period will not have lapsed, so get them now
  // before getting in loop
  //
  #ifdef __USING_CHILLER__
  handleChillerStatus();
  #endif
  handleACUStatus();
  handleRTDStatus(true, false); // true get fautls too, false get all RTDs temps
}


void loop(void)
{
  //
  // getStatus will update LCD and sysStats data structure
  //
  getStatus();

  //
  // update the LCD
  //
  manageLCD(false);

  //
  // take commands from the
  // - the button on front panel
  // - or the controlling PC software
  //
  // - handleMsgs(tmo) is the throttle of the loop
  // - RUNNING state the tmo for input is 3ms
  // - Otherwise the tmo for input is 3000ms
  //
  handleMsgs(CTRL_TIMEOUT);               // tmo is 3000ms
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


//-------------------------------------------------------------
//
//
void initSystem(void)
{

  //
  // initialize the fault/no-fault LED(s)
  //
  configureFaultNoFault();


  Wire1.begin();  // for the RTC; the Wire.begin() is called only if using SHT


  //
  // the RTC
  //
  rtc_clock = new DS3231(Wire1);  // clock was taken, rtc was taken, so we go with rtc_clock
  rtc_clock->setClockMode(false);

  //
  // initialize the system states /stats - these are
  // used to hold temperatures, humidity, etc. and
  // used in responses to getStatusCmd from control
  // and holds the LCD messages
  //
  initSysStates(sysStates);



  //
  // remove the LCD faces that are not going to be used due to compile time
  // options for chiller and humidity
  //
//#if defined(__USING_CHILLER__)
  // chiller
/*
  chiller_Running, chiller_Stopped, chiller_ComFailure,
//#endif
  sensor_humidityAndThreshold,
  sensor_HighHumidity, sensor_Failure,
*/
  #ifndef __USING_CHILLER__
  lcdFaces[chiller_Running]     = 0;
  lcdFaces[chiller_Stopped]     = 0;
  lcdFaces[chiller_ComFailure]  = 0;
  sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET] = 0;
  sysStates.lcd.lcdFacesIndex[CHILLER_FAIL_OFFSET] = 0;
  #endif

  #ifndef __USING_HUMIDITY__
  lcdFaces[sensor_humidityAndThreshold] = 0;
  lcdFaces[sensor_HighHumidity]         = 0;
  lcdFaces[sensor_Failure]              = 0;
  sysStates.lcd.lcdFacesIndex[HUMIDITY_NRML_OFFSET] = 0;
  sysStates.lcd.lcdFacesIndex[HUMIDITY_FAIL_OFFSET] = 0;
  #endif
  
  // start the LCD and paint system initializing
  //
  startLCD();
  

  //
  // start the Serial ports
  //
  #if defined __DEBUG_VIA_SERIAL__ || defined __DEBUG2_VIA_SERIAL__ || defined __DEBUG_RTD_READS__ || defined __DEBUG_RTD_READS2__
  Serial.begin(115200);
  #endif

  // control protocol uses Serial1
  Serial1.begin(CONTROL_PROTO_SPEED);

  // chiller protocol uses Serial2
  #if defined(__USING_CHILLER__)
  Serial2.begin(CHILLER_PROTO_SPEED);
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


#ifdef __USING_HUMIDITY__
  //   
  // start the humidity sensor - call this only after Wire.begin()
  //   
  startSHTSensor();
#endif

  //
  // initialize the start/stop button
  //
  configureButton();


  //
  // read the 50/60 Hz switch
  //
  pinMode(HZ_POWER_SWITCHPIN, INPUT_PULLUP);
  int mode = digitalRead(HZ_POWER_SWITCHPIN);
  
  #ifdef __DEBUG2_VIA_SERIAL__
  if( (HIGH == mode) )
    Serial.println("50 60 Hz switch is HIGH");
  else if( (LOW == mode ) )
    Serial.println("50 60 Hz switch is LOW");
  else
    Serial.println("50 60 Hz switch not HIGH and not LOW");
  #endif


  //
  // initialize the Adafruits
  //
  DDR1_RTD.begin(MAX31865_4WIRE);
  DDR2_RTD.begin(MAX31865_4WIRE);
  ASIC_Chiller_RTD.begin(MAX31865_4WIRE);
  DDR_Chiller_RTD.begin(MAX31865_4WIRE);

  if( (HIGH == mode) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("enabling 50hz on Adafruilt MAX31865s");
    #endif
    DDR1_RTD.enable50Hz(true);
    DDR2_RTD.enable50Hz(true);
    ASIC_Chiller_RTD.enable50Hz(true);
    DDR_Chiller_RTD.enable50Hz(true);
    
  } else if( (LOW == mode) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("enabling 60hz on Adafruilt MAX31865s");
    #endif
    DDR1_RTD.enable50Hz(false);
    DDR2_RTD.enable50Hz(false);
    ASIC_Chiller_RTD.enable50Hz(false);
    DDR_Chiller_RTD.enable50Hz(false);
    
  } else // default to 60Hz
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("defaulting to 60hz on Adafruilt MAX31865s");
    #endif
    DDR1_RTD.enable50Hz(false);
    DDR2_RTD.enable50Hz(false);
    ASIC_Chiller_RTD.enable50Hz(false);
    DDR_Chiller_RTD.enable50Hz(false);
  }

  //
  // set Celcius on the Accuthermos
  //
  writeUNIT(ASIC_RS485_ID, 0x001C);
  writeUNIT(DDR_RS485_ID, 0x001C);

  //
  // clear PVOF
  //
  writePVOF(ASIC_RS485_ID, 0);
  writePVOF(DDR_RS485_ID, 0);

  //
  // set the status_interval
  //
  status_interval = GET_STATUS_INTERVAL;
}


//
// initialize the system status data to reflect clean start up
//
void initSysStates(systemState& states)
{
//#if defined(__USING_CHILLER__)
  // chiller starts offline until queried via getStatus()
  states.chiller.online    = offline;
  states.chiller.state     = stopped;
  states.chiller.temperature = 0;
  states.chiller.setpoint  = 0;
//#endif

  // sensor starts offline until discovered to be online via getStatus()
  states.sensor.humidity     = 0; 
  states.sensor.threshold    = HUMIDITY_THRESHOLD;
  states.sensor.online       = offline;
  states.sensor.sampleData.index = 0; 
  for(int i = 0; i < MAX_HUMIDITY_SAMPLES; i++) 
    states.sensor.sampleData.sample[i] = 0.0; 



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
//#if defined(__USING_CHILLER__)
  sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]  = chiller_Stopped;
  sysStates.lcd.lcdFacesIndex[HUMIDITY_NRML_OFFSET]  = sensor_humidityAndThreshold;
//#endif
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

  #ifdef __USING_HUMIDITY__
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
  #else
    lcdFaces[sysStates.lcd.lcdFacesIndex[(sysStates.lcd.index)]]();
  #endif

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

  delay(5000);

  return(true);
}


//--------------------------------------------------------------
// must be called AFTER Wire.begin();
// only called if #define __USING_HUMIDITY__
//
bool startSHTSensor(void)
{
    bool retVal = true;

#ifdef __USING_HUMIDITY__

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif

    Wire.begin();   // for SHT humidity sensor
    p_sht = new SHTSensor;

    if( (p_sht->init()) && (p_sht->setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM)) )
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

  #endif

    return(retVal);
}


// ----------------------------------------------------------
// return true only if
// - LCD is up
// - SHTSensor is up
// - Meersteters are up
// - chiller is running
//
bool startUp(bool startAT)
{
  bool retVal = true;


  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif


  #ifdef __USING_HUMIDITY__
  //
  // if humidity is too high, or there is a humidity sensor failure,
  // do not start the chiller - return failure
  //
  if( (humidityHigh()) )
    return(false);
  #endif

  //
  // start the chiller and the ACUs
  //
  #ifdef __USING_CHILLER__
  if( !(startChiller()) )
    return(false);
  #endif
  

  if( (startACUs(startAT)) )
  {
    //
    // paint 'Starting' on LCD
    //
    if( (true == startAT) )
      lcd_starting_AT();
    else
      lcd_starting();
  
    // allow the LCD to show what is starting for at least 3 seconds
    delay(2000);
  
    // set the LCD to running
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Running;
  } // else leave the FacesIndex what it was .. ?
  else
  {
    retVal = false;
    // and don't change the lcdFaces
  }

  return(retVal);
}


void getStatus(void)
{
  static unsigned long  lastGetStatusTime         = 0;
  static unsigned long  lastGetStatusTimeRunning  = 0;
  static unsigned long  lastGetHumidityTime       = 0;
  unsigned long         currentGetStatusTime  = millis();


  //
  // get the chiller and ACU's status every GET_STATUS_INTERVAL seconds
  //
  //  TODO: test with a count rather than the millis(), millis() is trange on Due..
  //
  if((status_interval < (currentGetStatusTime - lastGetStatusTime)) )
  {
    lastGetStatusTime = currentGetStatusTime;

    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("---------- CHECK all ACUs and all RTDs -----------");
    #endif
    
    #ifdef __USING_CHILLER__
    handleChillerStatus();
    #endif
    handleACUStatus();            // read everything, running status, SV and PV for both controllers
    handleRTDStatus(true, false); // true means read the faults also, false get all RTD temps

    //
    // determine the hot RTD temperature
    //
    setHotRTD();

    if( (RUNNING == sysStates.sysStatus) )
    {
      //
      // determine the hot RTD, update DDR Accuthermo PVOF
      // calculateAndWritePVOF is using stored data fetched by
      // getStatus() and setHotRTD() . . so those funcitons must
      // be called prior to calling calculateAndWritePVOF
      //
      calculateAndWritePVOF();
      
    } else
    {
      #ifdef __DEBUG_RTD_READS__
      Serial.println("not in RUNNING state, not writing PVOF");
      #endif
    }
  }

/*
  if( (RUNNING == sysStates.sysStatus) &&
            (GET_STATUS_INTERVAL_RUNNING < (currentGetStatusTime - lastGetStatusTimeRunning)) )
  {
    lastGetStatusTimeRunning = currentGetStatusTime;
    
    #ifdef __DEBUG_VIA_SERIAL_
    Serial.println("----------- NOT check all ACUs and all RTDs ----------");
    #endif
    
    //
    // get only the RTD temperatures and the DDR Accuthermo PV
    // as these are needed for the HotRTD calculation
    //
    if( (true == handleRTDStatus(false, true)) ) // false means just get the temp, don't read faults, true get only DDR RTD temps
    {
      // return of true from handleRTDStatus means the RTDs were read
      // go read the DDR Accuthermo's PV
      // as that PV value is needed for the HotRTD calculation
      handleACUTempStatus(DDR_RS485_ID, true); // true is read Pv only

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
    }
  }
*/

#ifdef __USING_HUMIDITY__
  //   
  // get the chiller and TEC's status every GET_HUMIDITY_INTERVAL seconds
  //   
  if( (GET_HUMIDITY_INTERVAL < (currentGetStatusTime - lastGetHumidityTime)) )
  {    
    #ifdef __DEBUG_VIA_SERIAL__
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

      // if haven't logged high humidity, log it
      if( (sysStates.sensor.prior_humidity < sysStates.sensor.threshold) )
      {
        logEvent(HumidityHigh, 0);
      }
        
      // stop the chiller, stop the TECs
      if( (SHUTDOWN != sysStates.sysStatus) )
      {
          shutDownSys(false);
      }
    } else 
    {
        //enableButtonISR();
    }

    sysStates.sensor.prior_humidity = sysStates.sensor.humidity;
  }
#endif

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

#if defined(__USING_HUMIDITY__) && defined(__USING_CHILLER__)
  if( (stopChillerCmd) || (humidityHigh()) )
  {
    //Serial.println("have stopChillerCmd || humidityHigh()");
#elif defined(__USING_CHILLER__) &&  ! defined(__USING_HUMIDITY__)
  if( (stopChillerCmd) )
  {
    //Serial.println("have stopChillerCmd");
#endif
#if defined(__USING_CHILLER__)
    for(uint8_t i = 0; i < MAX_SHUTDOWN_ATTEMPTS; i++)
    {
      //Serial.println("stopping the chiller...");
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
  #ifdef __DEBUG_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  bool retVal = true;
#if defined(__USING_CHILLER__)


  #ifdef __USING_HUMIDITY__
  //
  // if humidity is too high, or there is a humidity sensor failure,
  // do not start the chiller - return failure
  //
  if( (humidityHigh()) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.println(" WARNING: not starting chiller, humidity too high or sensor failure");
    #endif

    return(false);
  }
  #endif


  if( !(chiller.StartChiller()) )
  {
    retVal  = false;
 
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__); Serial.println(" unable to start chiller");
    #endif
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__); Serial.println(" chiller started");
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

#endif

  return(retVal);
}


// ---------------------------------------
// find all ACUs, there should be 2
// - verify their addresses by fetching hw version
//      expecting ID s2, 3, and 4
// - turn them 'on'
// return true if all these things happen
//
bool startACUs(bool startAT)
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
      if( !(StartACU(Address, startAT)) )
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
      sysStates.lcd.lcdFacesIndex[ACU_FAIL_OFFSET]          = ACU_ComFailure;
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
    #ifdef __USING_HUMIDITY__
    if( (!humidityHigh()) )
    {
    #endif
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
  
        if( (LONG_PRESS_BP_COUNT < bp_count) )
          startUp(true);
        else
          startUp(false);
          
        bp_count = 0;
      } else
      {
        //
        // adjust the button LED
        //
        digitalWrite(BUTTON_LED, LOW);
        shutDownSys(false);
        bp_count = 0;
      }

    #ifdef __USING_HUMIDITY__
    }
    #endif
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
  
        case startUpATCmd:        // start the chiller if present, and ACUs
        {
          handleStartUpATCmd();
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
        
//#if defined(__USING_CHILLER__)
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
//#endif
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

        case setH20AlarmASIC:
        {
            handleSetH20AlarmASIC();
            break;
        }
        
        case setH20AlarmDDR:
        {
            handleSetH20AlarmDDR();
            break;
        }
        
        case getH20AlarmASIC:
        {
            handleGetH20AlarmASIC();
            break;
        }
        
        case getH20AlarmDDR:
        {
            handleGetH20AlarmDDR();
            break;
        }

//#ifdef __USING_HUMIDITY__
        case setHumidityThreshold:       // get the humidity threshold
        {
            handleSetHumidityThreshold();
            break;
        };

        case getHumidityThreshold:       // set the humidity threshold
        {
            handleGetHumidityThreshold();
            break;
        };

        case getHumidity:                // get current humidity and temperature
        {
            handleGetHumidity();
            break;
        };

        case getTempCmd:
        {
            handleGetTempCmd();
            break;
        }
//#endif
        default:
        {
          // send NACK
          sendNACK();
          break;
        }
      }
    } // else no message from control
    #ifdef __DEBUG2_VIA_SERIAL__
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
  lcd.setCursor(1,1);
  lcd.print("***** SYSTEM *****");
  lcd.setCursor(1,2);
  lcd.print("**** STARTING ****");
  lcd.display();
}

void lcd_starting_AT(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  lcd.noDisplay();
  lcd.clear();
  lcd.home();
  lcd.setCursor(1,1);
  lcd.print("***** SYSTEM *****");
  lcd.setCursor(1,2);
  lcd.print("*** STARTING AT ***");
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

  delay(2000); 
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
  lcd.print("Sys Comms ONLINE");
  lcd.setCursor(0,1);
  lcd.print("ASIC Sv      Pv     ");
  lcd.setCursor(0,3);
  lcd.print("DDR  Sv      Pv     ");

  // if the SV is negative, scoot left one position
  if( (sysStates.ACU[ASIC_ACU_IDX].setpoint < 0) )
    lcd.setCursor(7,1);
  else
    lcd.setCursor(8,1);
  lcd.print(sysStates.ACU[ASIC_ACU_IDX].setpoint,1);

  // if the PV is negative, scoot left one position
  if( (sysStates.ACU[ASIC_ACU_IDX].temperature < 0) )
    lcd.setCursor(15,1);
  else
    lcd.setCursor(16,1);
  lcd.print(sysStates.ACU[ASIC_ACU_IDX].temperature,1);
    

  // if the SV is negative, scoot left one position
  if( (sysStates.ACU[DDR_ACU_IDX].setpoint < 0) )
    lcd.setCursor(7,3);
  else
    lcd.setCursor(8,3);
  lcd.print(sysStates.ACU[DDR_ACU_IDX].setpoint,1);

  // if the PV is negative, scoot left one position
  if( (sysStates.ACU[DDR_ACU_IDX].temperature < 0) )
    lcd.setCursor(15,3);
  else
    lcd.setCursor(16,3);
  lcd.print(sysStates.ACU[DDR_ACU_IDX].temperature,1);
  
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
  lcd.print("Sys Comms OFFLINE");
  lcd.setCursor(0,1);
  lcd.print("ASIC Sv      Pv     ");
  lcd.setCursor(0,3);
  lcd.print("DDR  Sv      Pv     ");

  // if the SV is negative, scoot left one position
  if( (sysStates.ACU[ASIC_ACU_IDX].setpoint < 0) )
    lcd.setCursor(7,1);
  else
    lcd.setCursor(8,1);
  lcd.print(sysStates.ACU[ASIC_ACU_IDX].setpoint,1);

  // if the PV is negative, scoot left one position
  if( (sysStates.ACU[ASIC_ACU_IDX].temperature < 0) )
    lcd.setCursor(15,1);
  else
    lcd.setCursor(16,1);
  lcd.print(sysStates.ACU[ASIC_ACU_IDX].temperature,1);
    

  // if the SV is negative, scoot left one position
  if( (sysStates.ACU[DDR_ACU_IDX].setpoint < 0) )
    lcd.setCursor(7,3);
  else
    lcd.setCursor(8,3);
  lcd.print(sysStates.ACU[DDR_ACU_IDX].setpoint,1);

  // if the PV is negative, scoot left one position
  if( (sysStates.ACU[DDR_ACU_IDX].temperature < 0) )
    lcd.setCursor(15,3);
  else
    lcd.setCursor(16,3);
  lcd.print(sysStates.ACU[DDR_ACU_IDX].temperature,1);
  
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

  
//#if defined(__USING_CHILLER__)
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
//#endif

void lcd_humidityAndThreshold(void)
{
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("---------------------------------------");
    Serial.println(__PRETTY_FUNCTION__);
    #endif


    lcd.noDisplay();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Humidity:       %");
    lcd.setCursor(11,0);
    lcd.print(sysStates.sensor.humidity);
    lcd.setCursor(0,1);
    lcd.print("Threshold:      %");
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
      if( (startUp(false)) )
      {
        result  = 1;
        
        //
        // adjust the button
        //
        buttonOnOff         = true;
        currentButtonOnOff  = buttonOnOff;

        //
        // adjust the button LED
        //
        digitalWrite(BUTTON_LED, HIGH);

      } else
      {
        setSystemStatus();  // derive the button state
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
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" sent response");
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


void handleStartUpATCmd(void)
{
  startUpATCmd_t* pstartUpATCmd = reinterpret_cast<startUpATCmd_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;


  //
  // verify the received packet, here beause this is a startUpATCmdCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pstartUpATCmd->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_startUpATCmd_t,
          ntohs(pstartUpATCmd->crc), ntohs(pstartUpATCmd->eop))) )
    {
      //
      // start the ACUs, chiller, sensor ...
      //
      if( (startUp(true)) )
      {
        result  = 1;
        
        //
        // adjust the button
        //
        buttonOnOff         = true;
        currentButtonOnOff  = buttonOnOff;

        //
        // adjust the button LED
        //
        digitalWrite(BUTTON_LED, HIGH);

      } else
      {
        setSystemStatus();  // derive the button state
      }

      respLength = cp.Make_startUpATCmdResp(cp.m_peerAddress, cp.m_buff,
        result, pstartUpATCmd->header.seqNum
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
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" sent response");
        Serial.flush();
      #endif
      }
    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
      Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR: dropping packet bad CRC: ");
      Serial.println(ntohs(pstartUpATCmd->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pstartUpATCmd->header.address.address));
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
        buttonOnOff         = false;
        currentButtonOnOff  = buttonOnOff;

        //
        // adjust the button LED
        //
        digitalWrite(BUTTON_LED, LOW);
        
      } else
      {
        setSystemStatus();  // derive the button state
      }

      respLength = cp.Make_shutDownCmdResp(cp.m_peerAddress, cp.m_buff,
        result, pshutDownCmd->header.seqNum);

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
  uint16_t      respLength; 
  uint16_t      chiller_humidity = 0; // top 8 bits are chiller, bottom 8 bits are humidity
  

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
      if(offline == sysStates.chiller.online)
      {
        //Serial.println("      setting chiller offline...");
        chiller_humidity |= 0x1000;
      } //else
        //Serial.println("      setting chiller online...");
        
      if(running == sysStates.chiller.state)
      {
        chiller_humidity |= 0x0100;
        //Serial.println("      setting chiller running...");
      } //else
        //Serial.println("      setting chiller not running...");
      #endif

      #if defined(__USING_HUMIDITY__)
      if(offline == sysStates.sensor.online)
      {
        chiller_humidity  |= 0x0010;
        //Serial.println("      setting humidity offline...");
      } //else
        //Serial.println("      stting humidity online...");
          
      if( sysStates.sensor.humidity > sysStates.sensor.threshold )
      {
        chiller_humidity  |= 0x0001;
        //Serial.println("      setting humidity high...");
      } //else
        //Serial.println("      setting humidity low...");
      #endif
      
      respLength = cp.Make_getStatusResp(cp.m_peerAddress, cp.m_buff,
        getRTDErrors(),            // return bit map of RTDErrors 
        getACUErrorsAndRunState(), // ACUs running
        chiller_humidity,          // values are set if using them
        pgetStatus->header.seqNum
      );

      //
      // use the CP object to send the response back
      // this functoin usese the cp.m_buff created above, just
      // need to send the lenght into the function
      //
      if( !(cp.doTxResponse(respLength)))
      {
        //Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        //Serial.flush();
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
      #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.print(" ERROR: failed to send response");
        Serial.flush();
      #endif
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
  uint32_t    OutL      = 0;  // OUTL - a bunch of stuff ..
  uint32_t    WkErno    = 0;  // WKERNO - some program control and/or error no ..  
  uint32_t    Ver       = 0;  // VER - h/w and f/w version
  uint32_t    SerialNo  = 0;  // SERIAL_NH and SERIAL_NL - serial number


  if( (ntohs(pgetACUInfo->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getACUInfoMsg_t,
                ntohs(pgetACUInfo->crc), ntohs(pgetACUInfo->eop))) )
    {
      if( (getACUInfo(ntohs(pgetACUInfo->acu_address), &OutL,
                &WkErno, &Ver, &SerialNo)) )
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: failed to getACUInfo");
        Serial.flush();
        #endif
        result  = 0;
      }

      respLength = cp.Make_getACUInfoMsgResp(cp.m_peerAddress, cp.m_buff, htons(pgetACUInfo->acu_address), result,
                OutL, WkErno, Ver, SerialNo, pgetACUInfo->header.seqNum);

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
      if( (startACUs(false)) )
      {
        result  = 1;
      } else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println(__PRETTY_FUNCTION__); Serial.println(" ERROR: failed to startACUs");
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


//#if defined (__USING_CHILLER__)
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
/*
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
        result, "not supported",
        MAX_SLAVE_NAME_LENGTH, pgetChillerInfo->header.seqNum
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
*/
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
//#endif


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
      #ifdef __DEBUG2_VIA_SERIAL__
          Serial.print("year:mon:mday:wday:hour:min:sec");
          Serial.print(psetRTCCmd->tv.year); Serial.print(":");
          Serial.print(psetRTCCmd->tv.mon); Serial.print(":");
          Serial.print(psetRTCCmd->tv.mday); Serial.print(":");
          Serial.print(psetRTCCmd->tv.wday); Serial.print(":");
          Serial.print(psetRTCCmd->tv.hour); Serial.print(":");
          Serial.print(psetRTCCmd->tv.min); Serial.print(":");
          Serial.println(psetRTCCmd->tv.sec);
      #endif
            //
            // set the RTC - don't know if there is a 'bad' return code from this call
            //
          rtc_clock->setClockMode(false); // false == 24hr, true == 12hr
          rtc_clock->setYear(psetRTCCmd->tv.year);
          rtc_clock->setMonth(psetRTCCmd->tv.mon);
          rtc_clock->setDate(psetRTCCmd->tv.mday);
          rtc_clock->setDoW(psetRTCCmd->tv.wday);
          rtc_clock->setHour(psetRTCCmd->tv.hour);
          rtc_clock->setMinute(psetRTCCmd->tv.min);
          rtc_clock->setSecond(psetRTCCmd->tv.sec);
            
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
    bool        h12, PM_flag, Century;


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
            RTCTime.year = rtc_clock->getYear();
            RTCTime.mon  = rtc_clock->getMonth(Century);
            RTCTime.mday = rtc_clock->getDate();
            RTCTime.wday = rtc_clock->getDoW();
            RTCTime.hour = rtc_clock->getHour(h12, PM_flag);
            RTCTime.min  = rtc_clock->getMinute();
            RTCTime.sec  = rtc_clock->getSecond();
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
              #ifdef __DEBUG_VIA_SERIAL__
                Serial.print(__PRETTY_FUNCTION__); Serial.println(" ERROR: failed to send response");
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

  bool  retVal  = true;
  float temp    = 0;
  char* ptr     = 0;

  
  //
  // get all chiller information
  //
  if( (offline == sysStates.chiller.online) )
  {
    digitalWrite(FAULT_LED, HIGH);
    digitalWrite(NO_FAULT_LED, LOW);
    retVal = chiller.ChillerPresent(0);
  }
  
  if( (false == retVal) )  // call to chiller.ChillerPresent(0) failed
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
    sysStates.chiller.online = online;
    sysStates.lcd.lcdFacesIndex[CHILLER_FAIL_OFFSET] = no_Status;

    //
    // get the run state and temps
    //
    memset(chiller_buff, '\0', sizeof(chiller_buff));

    if( chiller.ReadStatus(chiller_buff) )
    {
      if( ('1' == chiller_buff[0]) )
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("chiller is running...");
        #endif
        sysStates.chiller.state = running;
        sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]  = chiller_Running;
      }
      else
      {
        #ifdef __DEBUG_VIA_SERIAL__
        Serial.println("chiller is not running...");
        #endif
        sysStates.chiller.state = stopped;
        sysStates.lcd.lcdFacesIndex[CHILLER_NRML_OFFSET]  = chiller_Stopped;
      }

      //
      // get the set point temp
      //
      memset(chiller_buff, '\0', sizeof(chiller_buff));
      chiller.ReadSetPointTemperature(chiller_buff);

      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print("reported setpoint temp: "); Serial.println(chiller_buff);
      #endif
      
      temp = strtof(chiller_buff, &ptr);

      if( ((ptr != 0) && ( (*ptr == '\0') || (*ptr == '\r') || (*ptr == '\n'))) || (ptr == 0) )
      {
        sysStates.chiller.setpoint = temp;
      #ifdef __DEBUG_VIA_SERIAL__
      } else
      {
        Serial.println("failed to get the chiller setpoint temp");
      #endif
      }

      // get the current temp
      memset(chiller_buff, '\0', sizeof(chiller_buff));
      chiller.ReadTemperature(chiller_buff);

      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print("reported temp: "); Serial.println(chiller_buff);
      #endif

      temp = strtof(chiller_buff, &ptr);

      if( ((ptr != 0) && ( (*ptr == '\0') || (*ptr == '\r') || (*ptr == '\n'))) || (ptr == 0) )
      {
        sysStates.chiller.temperature = temp;
      #ifdef __DEBUG_VIA_SERIAL__
      } else
      {
        Serial.println("failed to get the chiller temp");
      #endif
      }

      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.print("stored temperatures "); Serial.print(sysStates.chiller.temperature);
      Serial.print(" : "); Serial.println(sysStates.chiller.setpoint);
      #endif
    } else
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.print(__PRETTY_FUNCTION__);
      Serial.println(" WARINING: unable to chiller ReadStatus() ");
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
    }
  }
#endif
}


//--------------------------------
// get all data for all ACUs
//
void handleACUStatus(void)
{
  //Serial.println("handleACUStatus called...");
  
  // initialize to running and online
  sysStates.ACU[ASIC_ACU_IDX].state      = running;
  sysStates.ACU[ASIC_ACU_IDX].online     = online;
  sysStates.ACU[DDR_ACU_IDX].state       = running;
  sysStates.ACU[DDR_ACU_IDX].online      = online;


  // always get the running status
  handleACURunningStatus(ASIC_RS485_ID);
  handleACURunningStatus(DDR_RS485_ID);

  // if RUNNING state only get the PV, else get PV and SV
/*
  if( (RUNNING == sysStates.sysStatus) )
  {
    handleACUTempStatus(ASIC_RS485_ID, true); // get PV only if RUNNING state, menu won't let 'em change it
    handleACUTempStatus(DDR_RS485_ID, true);  // same
  } else
  {
*/
    handleACUTempStatus(ASIC_RS485_ID, false);
    handleACUTempStatus(DDR_RS485_ID, false);
//  }
}


//------------------------------------------
//
// get the running state of the input Accuthermo id
// id = 1 ASIC_RS485_ID
// id = 2 DDR_RS485_ID
//
void handleACURunningStatus(uint8_t id)
{
  bool  ACUsOnline  = true;
  bool  ACUsRunning = true;
  bool  Running     = false;
  uint8_t idx       = id - 1;  // array index is id - 1

  //Serial.println("handleACURunningStatus called...");
  
  //
  // assume they are running, if one if found not running, mark the group
  // as not running
  //
  // before going to chat w/ the ACUs - enable the FAUL_LED so it is on 
  // when there is a fault and on while we could be timing out chatting
  // to a dead ACU
  if( (sysStates.ACU[idx].online == offline) )
  {
    digitalWrite(FAULT_LED, HIGH);
    digitalWrite(NO_FAULT_LED, LOW);
  }
 
  //
  // get the value of ENAB, the current run mode
  //
  // returns false if can't talk to ACU
  // updates formal parameter Running if can talk to ACU
  //
  Running = false;
  if( (false == ACURunning(id, Running)) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:ACU ");
    Serial.print(id, DEC); Serial.println(" unable to get running");
    Serial.flush();
    #endif

    ACUsOnline  = false;
    ACUsRunning = false;

    sysStates.ACU[idx].online = offline;
    sysStates.ACU[idx].state  = stopped;

  } else if( (false == Running) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: ACU ");
    Serial.print(id); Serial.println(" is not running");
    Serial.flush();
    #endif
    //
    // keep track of whether all ACUs are running
    //
    ACUsOnline  = true;
    ACUsRunning = false;

    // update sysStates
    sysStates.ACU[idx].online = online;
    sysStates.ACU[idx].state  = stopped;
    
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: ACU ");
    Serial.print(id); Serial.println(" is running");
    Serial.flush();
    #endif

    ACUsOnline  = true;
    ACUsRunning = true;

    sysStates.ACU[idx].online = online;
    sysStates.ACU[idx].state  = running;
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


//------------------------------------------
//
// get the temperature of the input Accuthermo id
// id = 1 ASIC_RS485_ID
// id = 2 DDR_RS485_ID
//
void handleACUTempStatus(uint8_t id, bool GetPVOnly)
{
  bool  ACUsOnline  = true;
  bool  ACUsRunning = true;
  bool  Running     = false;
  bool  RetVal      = false;
  uint8_t idx       = id - 1; // don't f this up, no bounds checking

  //
  // check all ACUs running - get Device Status, possible status are
  //
  // assume they are running, if one if found not running, mark the group
  // as not running
  //
  // before going to chat w/ the ACUs - enable the FAUL_LED so it is on 
  // when there is a fault and on while we could be timing out chatting
  // to a dead ACU
  if( (sysStates.ACU[idx].online == offline) )
  {
    digitalWrite(FAULT_LED, HIGH);
    digitalWrite(NO_FAULT_LED, LOW);
  }

  //
  // get the ACU's set point and object temperatures
  //
  if( (true == GetPVOnly) )
    RetVal = GetACUTempPV(id, &sysStates.ACU[idx].temperature);
  else
    RetVal  = GetACUTemp(id, &sysStates.ACU[idx].setpoint, &sysStates.ACU[idx].temperature);
    
  if((false == RetVal) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" ERROR:ACU ");
    Serial.print(id, DEC); Serial.println(" unable to get temps");
    Serial.flush();
    #endif

    ACUsRunning = false;
    ACUsOnline  = false;

    sysStates.ACU[idx].online = offline;
    sysStates.ACU[idx].state  = stopped;
    #ifdef __DEBUG2_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" found ");
    Serial.print(sysStates.ACU[idx].setpoint, 2);
    Serial.print(" : "); Serial.println(sysStates.ACU[idx].temperature, 2);
    Serial.flush();
    #endif
  }

  //
  // ASIC_RTD is connected to the Accuthermo RS485 Id 1
  // DDR_RTD is connected to the Accuthermo RS485 Id 2
  //
  // translate the retrieved data and temp from the Accuthermos to these RTDs
  //
  if( (ASIC_RS485_ID == id) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("Translating ASIC ACU data to ASIC_RTD");
    #endif      
    resetRTDState(sysStates.ASIC_RTD);
    
    // pick up the temperature and faul, the online and state are updated in handleRTDStatus
    sysStates.ASIC_RTD.temperature  = sysStates.ACU[idx].temperature;

    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ASIC_RTD.temperature: "); Serial.println(sysStates.ASIC_RTD.temperature, 2);
    Serial.print("souce temperature: "); Serial.println(sysStates.ACU[idx].temperature, 2);
    #endif      

    if( (offline == sysStates.ACU[idx].online) )
    {
      sysStates.ASIC_RTD.online = offline;
      sysStates.ASIC_RTD.state  = stopped;    
      sysStates.ASIC_RTD.fault  = 0xFF;
    }

  } else if( (DDR_RS485_ID == id) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("Translating DDR ACU data to DDR_RTD");
    #endif      
      
    resetRTDState(sysStates.DDR_RTD);

    // pick up the temperature and faul, the online and state are updated in handleRTDStatus
    sysStates.DDR_RTD.temperature  = sysStates.ACU[idx].temperature;

    if( (offline == sysStates.ACU[idx].online) )
    {
      sysStates.DDR_RTD.online  = offline;
      sysStates.DDR_RTD.state   = stopped;    
      sysStates.DDR_RTD.fault   = 0xFF;
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
bool getACUInfo(uint8_t acu_address, uint32_t* OutL, uint32_t* WkErno,
                    uint32_t* Ver, uint32_t* SerialNo)
{
  bool retVal = false;


  if( ((ASIC_RS485_ID != acu_address) && (DDR_RS485_ID != acu_address)) )
    return(false);


  // attempt to get the data from the input address - if can't communicate with
  // target accuthermo, just return false
  *OutL     = readOUTL(acu_address);
  *WkErno   = readWKERNO(acu_address);
  *Ver      = readVER(acu_address);
  *SerialNo = readSERIAL_NH(acu_address);
  *SerialNo = readSERIAL_NL(acu_address);

  // the SerialNo high and low are returning succesful packets, but the reported 
  // serial no is -1 .. so can't do this check . . 
//  if( (0 == (-1 == *OutL) || (-1 == *WkErno) || (-1 == *Ver) || (-1 == *SerialNo)) )
    retVal  = true;

  return(retVal);
}


bool handleRTDStatus(bool getFaults, bool getDDROnly)
{
  bool  NON_DDR_RTDsRunning = true;  // becomes false if at least one RTD has fault
  bool  DDR_RTDsRunning     = true;  // becomes false if at least one RTD has fault
  bool  retVal              = false; // true if the RTDs were read

  
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
    digitalWrite(NO_FAULT_LED, LOW);
  }
  
  //
  // use the DRDY - return retVal true if a read was performed, false otherwise
  //
  retVal = getRTDDataDRDY(getFaults, getDDROnly);
  

  //
  // check the non DDR RTDs
  //
  if( (sysStates.ASIC_RTD.fault) )
  {
    // this RTD is connected to the Accuthermo Id 1 and
    // this data is updated for this RTD in handleACUStatus()
    sysStates.ASIC_RTD.online  = offline;
    sysStates.ASIC_RTD.state   = stopped;    
    NON_DDR_RTDsRunning  = false;
    
  } else
  {
    sysStates.ASIC_RTD.prior_fault  = 0; 
  }

  if( (sysStates.ASIC_Chiller_RTD.fault) )
  {
    sysStates.ASIC_Chiller_RTD.online  = offline;
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
    sysStates.DDR_RTD.online  = offline;
    sysStates.DDR_RTD.state   = stopped;    
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

  return(retVal);
}


// this data is processed in the LCD and setSystemStatus functions - only getting it here
void getAdafruitRTDData(Adafruit_MAX31865& afmaxRTD, RTDState& state, bool getAllData)
{

  // only get these on a period - i.e. not ALL the time
  if( (true == getAllData) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("GETTING ALL DATA");
    #endif
//    state.rtd         = afmaxRTD.readRTD(); // this one has the 75ms hard coded delay AND IS CALLED by temperature, don't call this everytime
    state.fault       = afmaxRTD.readFault(); // reads a register
  }

  // always get temperature
  if( (RUNNING == sysStates.sysStatus) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("IN ACTIVE STATE calling temperature_dd");
    #endif
    state.temperature = afmaxRTD.temperature_dd(RNOMINAL, RREF);  // this call is optimized for deftDevise
    
  } else
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("NOT IN ACTIVE STATE calling temperature");
    #endif
    state.temperature = afmaxRTD.temperature(RNOMINAL, RREF);     // this call is standard Adafruite, delays and all
  }

  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.print("fault: "); Serial.print(state.fault); Serial.print(" rtd: "); Serial.print(state.rtd);
  Serial.print(" temperature: "); Serial.println(state.temperature, 1);
  #endif
}



//-----------------------------------------------
//
//
bool getRTDDataDRDY(bool getFaults, bool getDDROnly)
{
  unsigned long newStartTime  = millis();   // they all get the same start time when starting
  unsigned long currTime      = millis();  
  uint8_t t;
  bool  retVal  = false;
  bool  guard_timer = false;
  

  guard_timer = (DRDY_GUARD_TIMER < (currTime - RTD_DDR1_DRDY_StartTime)) ? true : false;

  if( (true == RTD_DDR1_DRDY) || (true == guard_timer) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    if( (true == RTD_DDR1_DRDY) )
      Serial.println("-----------> RTD_DDR1_DRDY is true, getting temperature"); 
    else
      Serial.println("-----------> guard_timer expired, getting temperature");
    #endif

    // always get the DDR tempteratures
    resetRTDState(sysStates.DDR1_RTD);
    resetRTDState(sysStates.DDR2_RTD);
    getAdafruitRTDData(DDR1_RTD, sysStates.DDR1_RTD, getFaults);
    getAdafruitRTDData(DDR2_RTD, sysStates.DDR2_RTD, getFaults);

    // get the other if getDDROnly is false
    if( (false == getDDROnly) )
    {
      #ifdef __DEBUG2_VIA_SERIAL__
      Serial.println("getDDROnly is false, getting ASCI or DDR H2O temperatures too"); 
      #endif
      resetRTDState(sysStates.DDR_Chiller_RTD);
      resetRTDState(sysStates.ASIC_Chiller_RTD);
      getAdafruitRTDData(DDR_Chiller_RTD, sysStates.DDR_Chiller_RTD, getFaults);
      getAdafruitRTDData(ASIC_Chiller_RTD, sysStates.ASIC_Chiller_RTD, getFaults);

      //
      // reset the adafruit for the next conversion cycle
      //
      handleRTDISRs(false);
      RTD_DDR1_DRDY_StartTime = newStartTime;
      RTD_DDR1_DRDY = false;
      DDR1_RTD.setOneShot_dd();
      DDR1_RTD.readRTD_dd();
      handleRTDISRs(true);
     
    } else   // current implementation, this else won't get hit, but leaving here just in case
    {
      #ifdef __DEBUG_VIA_SERIAL__
      Serial.println("getDDROnly is true, NOT getting CHILLER temperatures too"); 
      #endif

      //
      // getDDRonly is true when in RUNNING state
      // the ISR is on/used only when in RUNNING state
      // so, trigger another read 'now' rather than waiting to come back to
      // the top of this function
      //
      handleRTDISRs(false);
      RTD_DDR1_DRDY_StartTime = newStartTime;
      RTD_DDR1_DRDY = false;
      DDR1_RTD.setOneShot_dd();
      DDR1_RTD.readRTD_dd();
      handleRTDISRs(true);
    }

    retVal = true;
  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.println("!!!!!! DRDY NOT READY !!!!!"); 
  #endif    
  }

  return(retVal);
}


// only called by getSatus() - keep it that way !
systemStatus setSystemStatus(void)
{
  systemStatus  retVal      = RUNNING;
  bool      ACUsOnline      = true; // becomes false
  bool      ACUsRunning     = true; // 
  bool      ACUMismatch     = false;  // becomes false
  bool      RTDsRunning     = true;  
  

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

      if( (sysStates.ACU[(i - MIN_ACU_ADDRESS)].prior_online != sysStates.ACU[(i - MIN_ACU_ADDRESS)].online) )
        logEvent(ACUNotOnLine, i);        
    }

    if( (stopped == sysStates.ACU[(i - MIN_ACU_ADDRESS)].state) )
    {
      ACUsRunning = false;

      if( (RUNNING == sysStates.sysStatus) )
      {
        if( (sysStates.ACU[(i - MIN_ACU_ADDRESS)].prior_state != sysStates.ACU[(i - MIN_ACU_ADDRESS)].state) )
          logEvent(ACUNotRunning, i);
      }
    }

    sysStates.ACU[(i - MIN_ACU_ADDRESS)].prior_online = sysStates.ACU[(i - MIN_ACU_ADDRESS)].online;
    sysStates.ACU[(i - MIN_ACU_ADDRESS)].prior_state = sysStates.ACU[(i - MIN_ACU_ADDRESS)].state;
    

    //
    // check for ACU on/off line or running mismatch, shutdown if present
    // can/could happen if a ACU fails or is reset
    //
    // checking against index 0 because they should all be the same
    //
    if( (sysStates.ACU[(i - MIN_ACU_ADDRESS)].online != sysStates.ACU[ASIC_ACU_IDX].online) ||
      (sysStates.ACU[(i - MIN_ACU_ADDRESS)].state != sysStates.ACU[ASIC_ACU_IDX].state) )
    {
      ACUMismatch = true;
      logEvent(ACUIsMismatch);  // this should not repeat in the eventlog as 
                                // the ACUs are about the stopped below which will
                                // put them both back in the same state
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
      logEvent(ASIC_RTDFault, 0, sysStates.ASIC_RTD.fault);
    }
  }
  sysStates.ASIC_RTD.prior_fault  = sysStates.ASIC_RTD.fault; 


  if( (sysStates.ASIC_Chiller_RTD.fault) )
  {
    sysStates.ASIC_Chiller_RTD.online  = offline;
    sysStates.ASIC_Chiller_RTD.state   = stopped;

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.ASIC_Chiller_RTD.fault != sysStates.ASIC_Chiller_RTD.prior_fault) )
    {
      logEvent(ASIC_Chiller_RTDFault, 0, sysStates.ASIC_Chiller_RTD.fault);
    }
  }
  sysStates.ASIC_Chiller_RTD.prior_fault = sysStates.ASIC_Chiller_RTD.fault;


  if( (sysStates.DDR_Chiller_RTD.fault) )
  {
    sysStates.DDR_Chiller_RTD.online  = offline;
    sysStates.DDR_Chiller_RTD.state   = stopped;    

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.DDR_Chiller_RTD.fault != sysStates.DDR_Chiller_RTD.prior_fault) )
    {
      logEvent(DDR_Chiller_RTDFault, 0, sysStates.DDR_Chiller_RTD.fault);
    }
  }
  sysStates.DDR_Chiller_RTD.prior_fault = sysStates.DDR_Chiller_RTD.fault;
  

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
      logEvent(DDR_RTDFault, 1, sysStates.DDR_RTD.fault);
    }
  }
  sysStates.DDR_RTD.prior_fault = sysStates.DDR_RTD.fault;


  if( (sysStates.DDR1_RTD.fault) )
  {
    sysStates.DDR1_RTD.online  = offline;
    sysStates.DDR1_RTD.state   = stopped;    

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.DDR1_RTD.fault != sysStates.DDR1_RTD.prior_fault) )
    {
      logEvent(DDR_RTDFault, 2, sysStates.DDR1_RTD.fault);
    }
  }
  sysStates.DDR1_RTD.prior_fault = sysStates.DDR1_RTD.fault;
  

  if( (sysStates.DDR2_RTD.fault) )
  {
    sysStates.DDR2_RTD.online  = offline;
    sysStates.DDR2_RTD.state   = stopped;    

    // if this fault is not the same as the prior fault, log it and
    // update prior_fault
    if( (sysStates.DDR2_RTD.fault != sysStates.DDR2_RTD.prior_fault) )
    {
      logEvent(DDR_RTDFault, 3, sysStates.DDR2_RTD.fault);
    }
  }
  sysStates.DDR2_RTD.prior_fault = sysStates.DDR2_RTD.fault;
  

  // check the ASIC RTD chiller temperatures - log event if needed
  // if temp is high and not already in SHUTDOWN, log the event as we are
  // about to go into SHUTDOWN
  if( (ASIC_HIGH < sysStates.ASIC_Chiller_RTD.temperature) &&
      (ASIC_HIGH > sysStates.ASIC_Chiller_RTD.prior_temperature) )
  {
    logEvent(ASIC_Chiller_RTDHot, 0, sysStates.ASIC_Chiller_RTD.temperature);
  }
  sysStates.ASIC_Chiller_RTD.prior_temperature = sysStates.ASIC_Chiller_RTD.temperature;
  

  // check the ASIC RTD chiller temperatures - log event if needed
  // if temp is high and not already in SHUTDOWN, log the event as we are
  // about to go into SHUTDOWN
  if( (DDR_HIGH < sysStates.DDR_Chiller_RTD.temperature) &&
      (DDR_HIGH > sysStates.DDR_Chiller_RTD.prior_temperature) )
  {
    logEvent(DDR_Chiller_RTDHot, 0, sysStates.DDR_Chiller_RTD.temperature);
  }
  sysStates.DDR_Chiller_RTD.prior_temperature = sysStates.DDR_Chiller_RTD.temperature;


  //
  // log chiller events if have chiller and states have changed
  //
  #ifdef __USING_CHILLER__
  if( (offline == sysStates.chiller.online) && (offline != sysStates.chiller.prior_online) )
  {
    logEvent(ChillerOffline);
  }
  sysStates.chiller.prior_online = sysStates.chiller.online;
  
  if( (running != sysStates.chiller.state) && (running == sysStates.chiller.prior_state) )
  {
    logEvent(ChillerNotRunning);
  } 
  sysStates.chiller.prior_state = sysStates.chiller.state;
  #endif
  
  //
  // log humidity sensor failure if using
  //
  #ifdef __USING_HUMIDITY__
  if( (online != sysStates.sensor.online) && (online == sysStates.sensor.prior_online) )
  {
    logEvent(HumiditySensorFail);
  } 
  sysStates.sensor.prior_online = sysStates.sensor.online;
  #endif
  
  //
  // special case check - if the chiller is not running and the ACUs are running, shutdown the ACUs
  //
  if( (SHUTDOWN == sysStates.sysStatus) && (true == ACUsRunning) )
  {
    shutDownSys(false);
    sysStates.sysStatus = UNKNOWN;
  }
  
  if( (UNKNOWN == sysStates.sysStatus) ||
     ( (true == ACUMismatch || false == ACUsOnline) || (false == RTDsRunning) )
#if defined(__USING_CHILLER__)
    || (running != sysStates.chiller.state || offline == sysStates.chiller.online)
#endif
#if defined(__USING_HUMIDITY__)
    || (humidityHigh())
#endif
  )
  {
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Shutdown;
    retVal  = SHUTDOWN;

    if( (SHUTDOWN != sysStates.sysStatus || UNKNOWN == sysStates.sysStatus) )
    {
      disableButtonISR();

      digitalWrite(FAULT_LED, HIGH);
      delay(250);
      digitalWrite(NO_FAULT_LED, LOW);
      delay(250);
      digitalWrite(BUTTON_LED, LOW);

      // disable the MAX31865 ISRs
      configureAdafruitsBySysteState(SHUTDOWN);

      //
      // clear PVOF
      //
      writePVOF(ASIC_RS485_ID, 0);
      writePVOF(DDR_RS485_ID, 0);
      
      buttonOnOff         = false;
      currentButtonOnOff  = buttonOnOff;
      shutDownSys(false); // sets sysStates.sysStatus to SHUTDOWN

      //
      // put the status_interval back to the not-running-state rate
      //
      status_interval = GET_STATUS_INTERVAL;
    }

    //
    // ALWAYS adjust the button, knobs, and LEDs
    //
    digitalWrite(FAULT_LED, HIGH);
    delay(250);
    digitalWrite(NO_FAULT_LED, LOW);
    delay(250);
    digitalWrite(BUTTON_LED, LOW);
  }

  else if(
     ( (false == ACUsRunning && false == ACUMismatch && true == ACUsOnline) )
#if defined(__USING_CHILLER__)
    && (running == sysStates.chiller.state && online == sysStates.chiller.online)
#endif
#if defined(__USING_HUMIDITY__)
    && (false == humidityHigh())
#endif
  )
  {
    //
    // READY - else everythig is online, ACUs are not running
    //
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Ready;
    retVal  = READY;

    if( (READY != sysStates.sysStatus) )
    {
      //
      // turn off the ACUs
      //
      for(uint8_t Address = MIN_ACU_ADDRESS; (Address <= MAX_ACU_ADDRESS); Address++)
        StopACU(Address);
      
      enableButtonISR();
      
      // disable the MAX31865 ISRs
      configureAdafruitsBySysteState(READY);

      //
      // clear PVOF
      //
      writePVOF(ASIC_RS485_ID, 0);
      writePVOF(DDR_RS485_ID, 0);

      buttonOnOff         = false;
      currentButtonOnOff  = buttonOnOff;

      //
      // adjust the  LEDs
      //
      digitalWrite(FAULT_LED, LOW);
      delay(250);
      digitalWrite(NO_FAULT_LED, HIGH);
      delay(250);
      digitalWrite(BUTTON_LED, LOW);

      //
      // put the status_interval back to the not-running-state rate
      //
      status_interval = GET_STATUS_INTERVAL;
    }
    
    //
    // always adjust the  LEDs
    //
    digitalWrite(FAULT_LED, LOW);
    delay(250);
    digitalWrite(NO_FAULT_LED, HIGH);
    delay(250);
    digitalWrite(BUTTON_LED, LOW);
  }

  else if(
     ( (true == ACUsRunning && false == ACUMismatch && true == ACUsOnline) )
#if defined(__USING_CHILLER__)
    && (running == sysStates.chiller.state && online == sysStates.chiller.online)
#endif
#if defined(__USING_HUMIDITY__)
    && (false == humidityHigh())
#endif
  )
  {
    //
    // else the system is running
    //
    sysStates.lcd.lcdFacesIndex[SYSTEM_NRML_OFFSET]  = sys_Running;
    retVal  = RUNNING;

    if( (RUNNING != sysStates.sysStatus) )
    {
      enableButtonISR();

      // disable the MAX31865 ISRs
      configureAdafruitsBySysteState(RUNNING);

      //
      // clear PVOF
      //
      writePVOF(ASIC_RS485_ID, 0);
      writePVOF(DDR_RS485_ID, 0);

      buttonOnOff     = true;
      currentButtonOnOff  = buttonOnOff;
     

      //
      // adjust the  LEDs
      //
      digitalWrite(FAULT_LED, LOW);
      digitalWrite(NO_FAULT_LED, LOW);
      digitalWrite(BUTTON_LED, HIGH);

      //
      // put the status_interval back to the not-running-state rate
      //
      status_interval = GET_STATUS_INTERVAL_RUNNING;
    }
    
    //
    // always adjust the  LEDs
    //
    digitalWrite(FAULT_LED, LOW);
    digitalWrite(NO_FAULT_LED, LOW);
    digitalWrite(BUTTON_LED, HIGH);
  }

  // set the system status
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
  #ifdef __DEBUG_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  noInterrupts();

  buttonLastInterruptTime = millis();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, LOW);  // one button press and the ISR is continuously hit - i.e. system freeze

  interrupts();
}


void disableButtonISR(void)
{
  #ifdef __DEBUG_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
  pinMode(BUTTON_PIN, OUTPUT);
}


//-----------------------------------------
// increment holdCount 
//
void buttonISR(void)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  // de-bounce
  unsigned long         interruptTime   = millis();

  bp_count += 1;

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
    
  #ifdef __DEBUG_RTD_READS__
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
  
    return(readPVOF.buff()[0]);  // TODO: does this need to be ntohs(unit16_t) ?
    
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


uint16_t readOUTL(uint8_t id)
{
  cmdResp readOUTL = RS485Bus.readProcess(id, OUTL, 2);
  
  if( ((true == readOUTL.retCode()) && (0 < readOUTL.bufflen())) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" returns OUTL 0x"); Serial.println(readOUTL.buff()[0], 16);
    #endif
  
    return(ntohs(*(reinterpret_cast<uint16_t*>(&(readOUTL.buff()[0])))));
    
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("readOUTL fail from id: "); Serial.println(id);
    #endif

    return(0xFFFF);
  }
}

uint16_t readWKERNO(uint8_t id)
{
  cmdResp readWKERNO = RS485Bus.readProcess(id, WKERNO, 2);
  
  if( ((true == readWKERNO.retCode()) && (0 < readWKERNO.bufflen())) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" returns WKERNO 0x"); Serial.println(readWKERNO.buff()[0], 16);
    #endif
  
    return(ntohs(*(reinterpret_cast<uint16_t*>(&(readWKERNO.buff()[0])))));
    
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("readWKERNO fail from id: "); Serial.println(id);
    #endif

    return(0xFFFF);
  }
}


uint16_t readVER(uint8_t id)
{
  cmdResp readVER = RS485Bus.readProcess(id, VER, 2);
  
  if( ((true == readVER.retCode()) && (0 < readVER.bufflen())) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" returns VER 0x"); Serial.println(readVER.buff()[0], 16);
    #endif
  
    return(ntohs(*(reinterpret_cast<uint16_t*>(&(readVER.buff()[0])))));
    
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("readVER fail from id: "); Serial.println(id);
    #endif

    return(0xFFFF);
  }
}


uint16_t readSERIAL_NH(uint8_t id)
{
  cmdResp readSERIAL_NH = RS485Bus.readProcess(id, SERIAL_NH, 2);
  
  if( ((true == readSERIAL_NH.retCode()) && (0 < readSERIAL_NH.bufflen())) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" returns SERIAL_NH 0x"); Serial.println(readSERIAL_NH.buff()[0], 16);
    #endif
  
    return(ntohs(*(reinterpret_cast<uint16_t*>(&(readSERIAL_NH.buff()[0])))));
    
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("readSERIAL_NH fail from id: "); Serial.println(id);
    #endif

    return(0xFFFF);
  }
}


uint16_t readSERIAL_NL(uint8_t id)
{
  cmdResp readSERIAL_NL = RS485Bus.readProcess(id, SERIAL_NL, 2);
  
  if( ((true == readSERIAL_NL.retCode()) && (0 < readSERIAL_NL.bufflen())) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" returns SERIAL_NL 0x"); Serial.println(readSERIAL_NL.buff()[0], 16);
    #endif
  
    return(ntohs(*(reinterpret_cast<uint16_t*>(&(readSERIAL_NL.buff()[0])))));
    
  } else
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("readSERIAL_NL fail from id: "); Serial.println(id);
    #endif

    return(0xFFFF);
  }
}


//-------------------------------------------------------------
//
//
bool StartACU(uint8_t id, bool startAT)
{
  cmdResp writeENAB;
  
  
  if( (true == startAT) )
    writeENAB = RS485Bus.writeProcess(id, ENAB, AT1);
  else
    writeENAB = RS485Bus.writeProcess(id, ENAB, SPON);

  if( (true == writeENAB.retCode()) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.println(" is started");
    #endif

    return(true); // started
  }

  #ifdef __DEBUG_VIA_SERIAL__
  Serial.print("ACU: "); Serial.print(id); Serial.println(" is not started, unable to write cmd");
  #endif
  
  return(false); // not started, unable to write
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

  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.print("ACU: "); Serial.print(id); Serial.println(" is not stopped, unable to write cmd");
  #endif
  
  return(true); // is AT1, AT2, MPWR, SPON, PROG, HOLD
}


//-------------------------------------------------------------
//
//
bool ACURunning(uint8_t id, bool& Running)
{
  cmdResp readENAB = RS485Bus.readProcess(id, ENAB, 2);


  Running = false;

  if( (false == readENAB.retCode() || 0 == readENAB.bufflen()) )
  {
    // unable to communicate with the unit
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.println(" is not communicating");
    #endif

    Running = false;
    return(false);
  }
  
  if( (OFF == ntohs(*(reinterpret_cast<uint16_t*>(readENAB.buff())))) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" is not running: 0x"); Serial.println(ntohs(*(reinterpret_cast<uint16_t*>(readENAB.buff()))), 16);
    #endif

    Running = false; // not running
    
  } else
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.print("ACU: "); Serial.print(id); Serial.print(" is running: 0x"); Serial.println(ntohs(*(reinterpret_cast<uint16_t*>(readENAB.buff()))), 16);
    #endif

    Running = true;
  }
  
  return(true);
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
// Rick says !
// - we don't need to pump in the PVOF real quick
// - we can take some time here to calculate the true PV
// - the crap software on them accuthermos is allegedly capable of reporting
//    just the PV via PV0 - but that don't work, that cmd returns the SV !
// - no kidding I totally checked the bytes
// - so because we have more processing time on our hands, going to
//    get the PV+PVOF, get the PVOF, do the math of PV = PV+PVOF - PVOF
//    and convert that number to float and return 'it'.
// - lame.
//
bool GetACUTempPV(uint8_t id, float* pv)
{
  uint16_t  pvpvof  = 0;
  uint32_t  pvof    = 0;


  // get the pv+pvof - this returns 2 bytes of PVPVOF
  cmdResp readPVPVOF = RS485Bus.readProcess(id, PVPVOF, 2);

  if( (false == readPVPVOF.retCode()) )
  {
    #ifdef __DEBUG_RTD_READS__
    Serial.println("readPVPVOF fail");
    #endif
    
    return(false);
    
  } else
  {
    pvpvof = htons(*(reinterpret_cast<uint16_t*>(&readPVPVOF.buff()[0])));
    
    #ifdef __DEBUG_RTD_READS__
    Serial.println("readPVPVOF a success");
    Serial.print("readProcess PVPVOF from id: "); Serial.print(id); Serial.print(" success, got "); Serial.print(readPVPVOF.bufflen()); Serial.println(" bytes returned");
    Serial.print("read value is: 0x"); Serial.print(pvpvof, 16); Serial.print(", "); Serial.println(htons((*(reinterpret_cast<uint16_t*>(&readPVPVOF.buff()[0])))));
    #endif
  }


  // get the pvof - this returns 4 bytes of PVOF ( whether you request 2 or 4 bytes )
  cmdResp readPVOF = RS485Bus.readProcess(id, PVOF, 2);

  if( (false == readPVOF.retCode()) )
  {
    #ifdef __DEBUG2_VIA_SERIAL__
    Serial.println("readPVOF fail");
    #endif
    
    return(false);
    
  } else
  {
    pvof = htons(*(reinterpret_cast<uint16_t*>(&readPVOF.buff()[0])));
    
    #ifdef __DEBUG_RTD_READS__
    Serial.println("readPVOF a success");
    Serial.print("readProcess PVOF from id: "); Serial.print(id); Serial.print(" success, got "); Serial.print(readPVOF.bufflen()); Serial.println(" bytes returned");
    Serial.print("read value is: 0x"); Serial.print(pvof, 16); Serial.print(", "); Serial.println(htons((*(reinterpret_cast<uint16_t*>(&readPVOF.buff()[0])))));
    #endif
  }

  *pv = pvpvof - pvof;
  *pv = (float)*pv / (float)10;

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
  // - and if either of the chiller RTDs get above ASIC_HIGH or DDR_HIGH, return bad status
  //
  if( (sysStates.ASIC_Chiller_RTD.fault || sysStates.DDR_Chiller_RTD.fault || sysStates.DDR_RTD.fault ||
      (sysStates.DDR1_RTD.fault && sysStates.DDR2_RTD.fault)) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("checkRTDStatus returning bad status");
    #endif
    
    retVal = false;
  }

  if( (ASIC_HIGH < sysStates.ASIC_Chiller_RTD.temperature) || 
      (DDR_HIGH < sysStates.DDR_Chiller_RTD.temperature) )
  {
    #ifdef __DEBUG_VIA_SERIAL__
    Serial.println("RTD chiller temp too high returning bad status");
    Serial.print(sysStates.ASIC_Chiller_RTD.temperature,2); Serial.print(":"); Serial.println(sysStates.DDR_Chiller_RTD.temperature,2);
    #endif

    retVal = false;
  }

  return(retVal);
}


uint16_t getRTDErrors(void)
{
  uint16_t  rtd_error_map = 0;
  
  //
  // RTDErrorMap is uint16
  // we have 4 RTDs, each RTD gets 4 bits
  // for now, just setting 1 bit for fail for an RTD
  //
  // the getStatus() accumulates the RTD faults
  // the event log has those faults
  // all we're doing here is indicating whether a fault is present 'now' by
  //  setting a bit for an RTD
  //

  rtd_error_map |= ( 0 == sysStates.ASIC_Chiller_RTD.fault  ? 0 : 1 << 12 ); // high
  rtd_error_map |= ( 0 == sysStates.DDR1_RTD.fault          ? 0 : 1 << 8 );
  rtd_error_map |= ( 0 == sysStates.DDR2_RTD.fault          ? 0 : 1 << 4 );
  rtd_error_map |= ( 0 == sysStates.DDR_Chiller_RTD.fault   ? 0 : 1 );

  return(rtd_error_map);
}


uint16_t getACUErrorsAndRunState(void)
{
  uint16_t  acu_error_run_state_map = 0;
  
  //
  // have two ACUs and 16 bits, each ACU gets 8 bits
  //  4 bits for run state
  //  4 bits for error state
  // 
  // check if the ACUs are offline - set that bit
  if( (offline == sysStates.ACU[ASIC_ACU_IDX].online) )
    acu_error_run_state_map |= (1 << 12); // high

  if( (running != sysStates.ACU[ASIC_ACU_IDX].state) )
    acu_error_run_state_map |= (1 << 8);
    
  if( (offline == sysStates.ACU[DDR_ACU_IDX].online) )
    acu_error_run_state_map |= (1 << 4); // high

  if( (running != sysStates.ACU[DDR_ACU_IDX].state) )
    acu_error_run_state_map |= 1;
    

  return(acu_error_run_state_map);
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

//  Serial.print("added event.id: "); Serial.println(event.id, 16);
}


uint16_t getRTCTime(timeind& rtcTime)
{
    bool h12, PM_Time, Century;

    
    rtcTime.sec    = rtc_clock->getSecond();
    rtcTime.min    = rtc_clock->getMinute();
    rtcTime.hour   = rtc_clock->getHour(h12, PM_Time); 
    rtcTime.mday   = rtc_clock->getDate();
    rtcTime.mon    = rtc_clock->getMonth(Century);
    rtcTime.year   = rtc_clock->getYear();
    rtcTime.wday   = rtc_clock->getDoW();
    rtcTime.fill = 0x00; // fill to keep the buff length 
  
  return(1);
}


bool calculateAndWritePVOF(void)
{
  uint16_t  pvof = 0;
  static uint16_t prior_pvof = 0;  


  //
  // if the HotRTD is the RTD attached to the Accuthermo, no need to write an PVOF
  //
  if( (sysStates.highRTDTemp != sysStates.DDR_RTD.temperature) )
  {
    #if defined(__DEBUG_RTD_READS__) || defined(__DEBUG_VIA_SERIAL__)
    Serial.println("DOING writePVOF----------------");
    #endif
    
    // calculate the PV offset as the (hight temp RTD) - (DDR-Accuthermo-PV)
    pvof  = (uint16_t)(10 * (sysStates.highRTDTemp - sysStates.DDR_RTD.temperature));
  } else
  {
    // clear a prior pvof
    pvof = 0;
  }

  #if defined(__DEBUG_RTD_READS2__) || defined(__DEBUG_VIA_SERIAL__)
  Serial.print("highRTDTemp is: "); Serial.print(sysStates.highRTDTemp, 1);
  Serial.print(" DDR_RTD PV is : "); Serial.println(sysStates.DDR_RTD.temperature, 1);
  Serial.print("pvof to write is : "); Serial.println(pvof, 1);
  #endif

  if( (prior_pvof != pvof) )
  {
    prior_pvof = pvof;
    
    if( (0xffff == writePVOF(DDR_RS485_ID, pvof)) )
    {
      #if defined(__DEBUG_RTD_READS__) || defined(__DEBUG_VIA_SERIAL__)
      Serial.print("FAILED to writePVOF of "); Serial.println(pvof, 16);
      #endif
  
      return(false);
    }
  
    #if defined(__DEBUG_RTD_READS2__) || defined(__DEBUG_VIA_SERIAL__)
    Serial.println("SUCCESS writePVOF");
    #endif
  } else
  {
    #if defined(__DEBUG_RTD_READS2__) || defined(__DEBUG_VIA_SERIAL__)
    Serial.println("no need to writePVOF, same as last write");
    #endif
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
  RTD_DDR1_DRDY = true;
}


void handleRTDISRs(bool on)
{
  #ifdef __DEBUG2_VIA_SERIAL__
  Serial.println("---------------------------------------");
  Serial.println(__PRETTY_FUNCTION__);
  #endif

  
  // set the the flag to known state  
  RTD_DDR1_DRDY = false;

  if( (false == on) )
  {
    disableRTD_DDR1_ISR();
  } else
  {
    enableRTD_DDR1_ISR();
  }
}


void configureAdafruitsBySysteState(systemStatus sysStatus)
{
  // disable the MAX31865 ISRs
  handleRTDISRs(false);

  if( (RUNNING == sysStatus) )
  {
    #ifdef __DEBUG_RTD_READS__
    Serial.println("RUNNING, enabling Vbias for all MAX31865");    
    #endif

    DDR1_RTD.enableBias(true);
    DDR2_RTD.enableBias(true);
    ASIC_Chiller_RTD.enableBias(true);
    DDR_Chiller_RTD.enableBias(true);

    DDR1_RTD.autoConvert(true);
    DDR2_RTD.autoConvert(true);
    ASIC_Chiller_RTD.autoConvert(true);
    DDR_Chiller_RTD.autoConvert(true);

//    RTD_DDR1_DRDY_StartTime = 0;  // TODO: this is suuuuper lame..
    
//    delay(10);  // do we need this when enabling bias? the adafruit driver does this ..

  } else
  {
    #ifdef __DEBUG_RTD_READS__
    Serial.println("not RUNNING, turning off Vbias for all MAX31865");    
    #endif
    
    // disable the Vbia and the autoconvert
    DDR1_RTD.enableBias(false);
    DDR2_RTD.enableBias(false);
    ASIC_Chiller_RTD.enableBias(false);
    DDR_Chiller_RTD.enableBias(false);

    DDR1_RTD.autoConvert(false);
    DDR2_RTD.autoConvert(false);
    ASIC_Chiller_RTD.autoConvert(false);
    DDR_Chiller_RTD.autoConvert(false);
  }

   RTD_DDR1_DRDY_StartTime = 0; // triggers behavior
   delay(10);  // do we need this when enabling bias? the adafruit driver does this ..
}


void handleSetH20AlarmASIC(void)
{
  setH20AlarmASIC_t* psetH20AlarmASIC = reinterpret_cast<setH20AlarmASIC_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;
  char*     ptr = 0;
  float     temp  = 0;

  //
  // verify the received packet, here beause this is a setH20AlarmASICCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(psetH20AlarmASIC->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_setH20AlarmASIC_t,
                ntohs(psetH20AlarmASIC->crc), ntohs(psetH20AlarmASIC->eop))) )
    {
      //
      // set the new H20 alarm ASIC
      //
      temp = strtof((char*)psetH20AlarmASIC->temperature, &ptr);

      if( (0 == temp) && (0 != ptr) )
      {
        result = 0; // failed to convert
      } else
      {
        result = 1; // convert good
        ASIC_HIGH = temp;
      }
      
      //Serial.print("ASIC_HIGH: "); Serial.println(ASIC_HIGH, 2);

      respLength = cp.Make_setH20AlarmASICResp(cp.m_peerAddress, cp.m_buff,
        result, psetH20AlarmASIC->header.seqNum
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
      Serial.println(ntohs(psetH20AlarmASIC->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(psetH20AlarmASIC->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleGetH20AlarmASIC()
{
  getH20AlarmASIC_t* pgetH20AlarmASIC = reinterpret_cast<getH20AlarmASIC_t*>(cp.m_buff);
  uint16_t  respLength; 


  //
  // verify the received packet, here beause this is a getH20AlarmASICCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetH20AlarmASIC->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getH20AlarmASIC_t,
                ntohs(pgetH20AlarmASIC->crc), ntohs(pgetH20AlarmASIC->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      respLength = cp.Make_getH20AlarmASICResp(cp.m_peerAddress, cp.m_buff,
        ASIC_HIGH, pgetH20AlarmASIC->header.seqNum);

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
      Serial.println(ntohs(pgetH20AlarmASIC->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetH20AlarmASIC->header.address.address));
    Serial.flush();
  #endif
  }
}




void handleSetH20AlarmDDR(void)
{
  setH20AlarmDDR_t* psetH20AlarmDDR = reinterpret_cast<setH20AlarmDDR_t*>(cp.m_buff);
  uint16_t  respLength; 
  uint16_t  result = 0;
  char*     ptr = 0;
  float     temp  = 0;

  //
  // verify the received packet, here beause this is a setH20AlarmDDRCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(psetH20AlarmDDR->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_setH20AlarmDDR_t,
                ntohs(psetH20AlarmDDR->crc), ntohs(psetH20AlarmDDR->eop))) )
    {
      //
      // set the new H20 alarm ASIC
      //
      temp = strtof((char*)psetH20AlarmDDR->temperature, &ptr);

      if( (0 == temp) && (0 != ptr) )
      {
        result = 0; // failed to convert
      } else
      {
        result = 1; // convert good
        DDR_HIGH = temp;
      }
      
      //Serial.print("DDR_HIGH: "); Serial.println(DDR_HIGH, 2);
      respLength = cp.Make_setH20AlarmDDRResp(cp.m_peerAddress, cp.m_buff,
        result, psetH20AlarmDDR->header.seqNum
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
      Serial.println(ntohs(psetH20AlarmDDR->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(psetH20AlarmDDR->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleGetH20AlarmDDR()
{
  getH20AlarmDDR_t* pgetH20AlarmDDR = reinterpret_cast<getH20AlarmDDR_t*>(cp.m_buff);
  uint16_t  respLength; 


  //
  // verify the received packet, here beause this is a getH20AlarmDDRCmd
  // check this is my address and the CRC is correct
  //
  if( (ntohs(pgetH20AlarmDDR->header.address.address)) == cp.m_myAddress)
  {
    //
    // verify the CRC
    //
    if( (cp.verifyMessage(len_getH20AlarmDDR_t,
                ntohs(pgetH20AlarmDDR->crc), ntohs(pgetH20AlarmDDR->eop))) )
    {
      //
      // chiller informaion is gotton during getStatus
      //
      respLength = cp.Make_getH20AlarmDDRResp(cp.m_peerAddress, cp.m_buff,
        DDR_HIGH, pgetH20AlarmDDR->header.seqNum);

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
      Serial.println(ntohs(pgetH20AlarmDDR->crc));
      Serial.flush();
    #endif
    }

  #ifdef __DEBUG_VIA_SERIAL__
  } else
  {
    Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
    Serial.println(ntohs(pgetH20AlarmDDR->header.address.address));
    Serial.flush();
  #endif
  }
}


void handleSetHumidityThreshold(void)
{
    setHumidityThreshold_t* psetHumidityThreshold = reinterpret_cast<setHumidityThreshold_t*>(cp.m_buff);
    uint16_t                respLength;
    uint16_t                result = 0;
    
    
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
    uint16_t                respLength;


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
    uint16_t       respLength;


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


void handleGetTempCmd(void)
{
    getTempCmd_t* pgetTempCmd = reinterpret_cast<getTempCmd_t*>(cp.m_buff);
    uint16_t      respLength;
    float         temp  = 0;
    uint16_t      itemp = 0;
    int16_t       fraction;
    int16_t       base;


    //
    // verify the received packet, here beause this is a getTempCmdCmd
    // check this is my address and the CRC is correct
    //
    if( (ntohs(pgetTempCmd->header.address.address)) == cp.m_myAddress)
    {
        //
        // verify the CRC
        //
        if( (cp.verifyMessage(len_getTempCmd_t,
                        ntohs(pgetTempCmd->crc), ntohs(pgetTempCmd->eop))) )
        {
            //
            // use the sysStates content to respond, send back the received seqNum
            //
            temp = getTemperature();

            base = temp;  // explicit conversion to int to peel off the fraction part
            fraction  = ((float)(temp - base))*100;
            itemp     = base << 8;
            itemp     |= fraction & 0x00ff;

            #ifdef __DEBUG_VIA_SERIAL__
            Serial.print("p_sht->getTemperature() returned: "); Serial.println(temp, 2);
            Serial.print("returning representation: "); Serial.println(itemp, HEX);
            #endif

            respLength = cp.Make_getTempCmdResp(cp.m_peerAddress, cp.m_buff, itemp, pgetTempCmd->header.seqNum);

            //
            // use the CP object to send the response back
            // this functoin usese the cp.m_buff created above, just
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
            Serial.println(ntohs(pgetTempCmd->crc));
            Serial.flush();
        #endif
        }

    #ifdef __DEBUG_VIA_SERIAL__
    } else
    {
        Serial.print(__PRETTY_FUNCTION__); Serial.print(" WARNING: bad address, dropping packet");
        Serial.println(ntohs(pgetTempCmd->header.address.address));
        Serial.flush();
    #endif
    }
}


bool getHumidityLevel(void)
{
    bool retVal = true;

  #ifdef __USING_HUMIDITY__
    //
    // update status and take a reading
    //
    if( (sysStates.sensor.online == offline) )
    {
      digitalWrite(FAULT_LED, HIGH);
      digitalWrite(NO_FAULT_LED, LOW);
    }

    sysStates.sensor.online = online;

    if( (p_sht->readSample()) )
    {
        //
        // update the sysStates for humidity - take an average to smooth spikes
        //
        sysStates.sensor.sampleData.sample[sysStates.sensor.sampleData.index] = p_sht->getHumidity();
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
  #endif
    return(retVal);
}


float getTemperature(void)
{
    float retVal = 0;
  #ifdef __USING_HUMIDITY__
    p_sht->readSample();
    retVal = p_sht->getTemperature();
  #endif
    return(retVal); // get a free conversion to int ..?
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
