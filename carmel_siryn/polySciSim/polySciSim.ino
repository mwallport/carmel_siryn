//#include <SoftwareSerial.h>
#include <stdint.h>


// chiller state
typedef enum {running, standby} State;
State state = standby;

// chiller setpoint and temperature
float setPoint  = 30.5;
float temp      = 29.5;


const char* runStatusAck    = "1\r";
const char* stdbyStatusAck  = "0\r";
const char* simpleAck       = "!\r";
const char* simpleNack      = "?\r";

const char* readStatusCmd   = "RW";
const char* setCmdEchoOn    = "SE1";
const char* setCmdEchoOff   = "SE0";
const char* setOnCmd        = "SO1";
const char* setOffCmd       = "SO0";
const char* setSetPoint     = "SS";  // add setPoint
const char* readSetPoint    = "RS";
const char* readTemperature = "RT";     // add temp
const char* readStatus      = "RW";
const char* readFaulStatus  = "RF";
const char* outputContOn    = "RD1";
const char* outputContOff   = "RD0";

const uint8_t MAX_BUFF_SIZE  = 10;
uint8_t idx;
uint8_t in;
char buff[MAX_BUFF_SIZE];

void setup()
{
  Serial.begin(115200);
  state = standby;  // chiller start lift stdby (which is off)
  Serial2.begin(9600);
//  Serial2.setTimeout(2000);

  randomSeed(analogRead(0));
}


void loop()
{
  // read a command
  memset(buff, '\0', MAX_BUFF_SIZE);

  Serial2.readBytesUntil('\r', buff, MAX_BUFF_SIZE);
 
  if( strlen(buff) )
  {
    Serial.print("got command: "); Serial.print((char*)buff); Serial.print(": ");
    for(int i = 0; i < 10; i++)
      Serial.print((uint8_t)buff[i], 16);
    Serial.println("");
  
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
      {
        Serial.println("got readStatusCmd sending running");
        Serial2.write(runStatusAck, strlen(runStatusAck));
        
      } else
      {
        Serial.println("got readStatusCmd sending standby");
        Serial2.write(stdbyStatusAck, strlen(stdbyStatusAck));
      } 
    } else if( ((0 == strcmp(setCmdEchoOn, buff)) ||
                (0 == strcmp(setCmdEchoOff, buff)) ||
                (0 == strcmp(outputContOn, buff)) ||
                (0 == strcmp(outputContOff, buff))) )
    {
      Serial2.write(simpleAck, strlen(simpleAck));
  
    } else if( (0 == strncmp(setSetPoint, buff, 2)) )
    {
//      scanf(buff, "SS%-+2.1f\r", &setPoint);
      setPoint = atof(&buff[2]);
      Serial.println("got setPoint command for: "); Serial.println(setPoint, 1);
      Serial2.write(simpleAck, strlen(simpleAck));
      
    } else if( (0 == strcmp(readSetPoint, buff)) )
    {
      memset(buff, '\0', MAX_BUFF_SIZE);
      dtostrf(setPoint, 2, 1, buff);
        
      strcat(buff, "\r");
      Serial.print("got readSetPoint command, sending back: "); Serial.println(buff);
      Serial2.write(buff);
      
    } else if( (0 == strcmp(readTemperature, buff)) )
    {
      memset(buff, '\0', MAX_BUFF_SIZE);
      temp = random(90, 320);
      temp = (float)temp / (float)10;
      dtostrf(temp, 2, 1, buff);
      strcat(buff, "\r");
      Serial.print("got readSTemperature command, sending back: "); Serial.println(buff);
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
    } else
    {
      Serial.println("replying simple NACK");
      Serial2.write(simpleNack, strlen(simpleNack));
    }
  }
}
