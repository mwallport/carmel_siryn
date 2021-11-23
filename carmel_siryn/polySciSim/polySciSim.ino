#include <SoftwareSerial.h>
#include <stdint.h>

typedef enum {on, off} OnOff;
typedef enum {running, standby} State;

State state;

const char* readStatusCmd   = "RW\r";
const char* runStatusAck    = "1\r";
const char* stdbyStatusAck  = "0\r";

const char* setOnCmd        = "SO1\r";
const char* setOffCmd       = "SO0\r";
const char* setCmdEchoOn    = "SE1\r";
const char* setCmdEchoOff   = "SE0\r";
const char* outputContOn    = "RD1\r";
const char* outputContOff   = "RD0\r";
const char* simpleAck       = "!\r";
const char* simpleNack      = "?\r";

const uint8_t MAX_BUFF_SIZE  = 10;
uint8_t buff[MAX_BUFF_SIZE];

void setup()
{
  state = standby;  // chiller start lift stdby (which is off)
  Serial.begin(9600);
  Serial.setTimeout(1000);
}


void loop()
{
  // read a command
  memset(buff, '\0', MAX_BUFF_SIZE);
  Serial.readBytesUntil('\r', buff, MAX_BUFF_SIZE);

  //
  // onyl handling two commands
  // 1. enable/disable
  // 2. get status
  //
  if( (0 == strcmp(setOnCmd, buff)) )
  {
    state = running;
    Serial.write(simpleAck, strlen(simpleAck));
    
  } else if( (0 == strcmp(setOffCmd, buff)) )
  {
    state = standby;
    Serial.write(simpleAck, strlen(simpleAck));
    
  } else if( (0 == strcmp(readStatusCmd, buff)) )
  {
    if( (running == state) )
      Serial.write(runStatusAck, strlen(runStatusAck));
    else
      Serial.write(stdbyStatusAck, strlen(stdbyStatusAck));
      
  } else if( ((0 == strcmp(setCmdEchoOn, buff)) ||
              (0 == strcmp(setCmdEchoOff, buff)) ||
              (0 == strcmp(outputContOn, buff)) ||
              (0 == strcmp(outputContOn, buff))) )
  {
    Serial.write(simpleAck, strlen(simpleAck));

  } else
  {
    Serial.write(simpleNack, strlen(simpleNack));
  }
}
