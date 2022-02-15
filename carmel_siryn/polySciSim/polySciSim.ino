//#include <SoftwareSerial.h>
#include <stdint.h>


// chiller state
typedef enum {running, standby} State;
State state = standby;

// chiller setpoint and temperature
float setPoint  = 30.5;
float temp      = 29.5;


const char* readStatusCmd   = "RW\r";
const char* runStatusAck    = "1\r";
const char* stdbyStatusAck  = "0\r";

const char* setCmdEchoOn    = "SE1\r";
const char* setCmdEchoOff   = "SE0\r";
const char* setOnCmd        = "SO1\r";
const char* setOffCmd       = "SO0\r";
const char* setSetPoint     = "SSxxx\r";  // add setPoint
const char* readSetPoint    = "RS\r";
const char* readTemperature = "RT\r";     // add temp
const char* readStatus      = "RW\r";
const char* readFaulStatus  = "RF\r";
const char* outputContOn    = "RD1\r";
const char* outputContOff   = "RD0\r";
const char* simpleAck       = "!\r";
const char* simpleNack      = "?\r";

const uint8_t MAX_BUFF_SIZE  = 10;
char buff[MAX_BUFF_SIZE];

void setup()
{
  Serial.begin(115200);
  state = standby;  // chiller start lift stdby (which is off)
  Serial2.begin(9600);
  Serial2.setTimeout(2000);

  randomSeed(analogRead(0));
}


void loop()
{
  // read a command
  memset(buff, '\0', MAX_BUFF_SIZE);
  Serial2.readBytesUntil('\r', (uint8_t*)buff, MAX_BUFF_SIZE);

  Serial.print("got command: "); Serial.println(buff);
/*  for(int i = 0; i < 10; i++)
    Serial.print((char)buff[i]); */

  if( (0 == strcmp(setOnCmd, buff)) )
  {
    state = running;
    Serial2.write(simpleAck, strlen(simpleAck));
    
  } else if( (0 == strcmp(setOffCmd, buff)) )
  {
    state = standby;
    Serial2.write(simpleAck, strlen(simpleAck));
    
  } else if( (0 == strcmp(readStatusCmd, buff)) )
  {
    if( (running == state) )
      Serial2.write(runStatusAck, strlen(runStatusAck));
    else
      Serial2.write(stdbyStatusAck, strlen(stdbyStatusAck));
      
  } else if( ((0 == strcmp(setCmdEchoOn, buff)) ||
              (0 == strcmp(setCmdEchoOff, buff)) ||
              (0 == strcmp(outputContOn, buff)) ||
              (0 == strcmp(outputContOn, buff))) )
  {
    Serial2.write(simpleAck, strlen(simpleAck));

  } else if( (0 == strcmp(setSetPoint, buff)) )
  {
    scanf(buff, "SE%f\r", &setPoint);
    setPoint /= 10;
    Serial.println("got setPoint command for: "); Serial.println(setPoint, 1);
    Serial2.write(simpleAck, strlen(simpleAck));
    
  } else if( (0 == strcmp(readSetPoint, buff)) )
  {
    memset(buff, '\0', MAX_BUFF_SIZE);
    sprintf(buff, "-%2.1f\r", setPoint);
    Serial.print("got readSetPoint command, sending back: "); Serial.println(buff);
    Serial2.write(buff);
    
  } else if( (0 == strcmp(readTemperature, buff)) )
  {
    memset(buff, '\0', MAX_BUFF_SIZE);
    temp = random(90, 320);
    temp = (float)temp / (float)10;
    sprintf(buff, "-%2.1f\r", temp);
    Serial.print("got readSetPoint command, sending back: "); Serial.println(buff);
    Serial2.write(buff);   
    
  } else if( (0 == strcmp(readStatus, buff)) )
  {
    memset(buff, '\0', MAX_BUFF_SIZE);

    if( (standby == state) )
      sprintf(buff, "0\r");
    else
      sprintf(buff, "1\r");
      
    Serial.print("got readStatus command, sending back: "); Serial.println(buff);
    Serial2.write(buff);   
    
  } else if( (0 == strcmp(readFaulStatus, buff)) )
  {
    Serial.println("got readFaultStatus, replying simple ack");
    Serial2.write(simpleAck);    
  }
  {
    Serial2.write(simpleNack, strlen(simpleNack));
  }
}
