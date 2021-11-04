#include <SPI.h>
#include <Controllino.h>

#include <HardwareSerial.h>
#include <stdio.h>
#include <inttypes.h>
//#include <string>
#include "handler.h"
#include "ctrlr_commands.h"
#include "deviceHandler.h"

static const uint8_t MAX_BUFF_LENGTH    = 64;

uint8_t tx_buff[MAX_BUFF_LENGTH];
uint8_t rx_buff[MAX_BUFF_LENGTH];

uint8_t   ASIC_ID     = 1;
uint8_t   DDR_ID      = 2;
uint8_t   RTD_MUX_ID  = 3;




void setup() {
  // put your setup code here, to run once:
  Controllino_RS485Init();
  Serial3.begin(19200);
  Controllino_RS485RxEnable();

  Serial.begin(19200);

  while( (!Serial) && (!Serial3) )
    delay(500);

  //let me open the Serial monitor .. !
  Serial.println("delaying...");
  delay(5000);
  Serial.println("starting...");
  

  if( (!Serial3) )
  {
    //fprintf(stderr, "unable to start the serial port\n");
    exit(1);
  }

  deviceHandler rs485Bus(Serial3, tx_buff, MAX_BUFF_LENGTH, rx_buff, MAX_BUFF_LENGTH);

  rs485Bus.writeProcess(ASIC_ID, SV, 0x2002);
/*  rs485Bus.readProcess(ASIC_ID, PVPVOF, 12);

  rs485Bus.readProcess(DDR_ID, PVPVOF, 12);
  rs485Bus.writeProcess(DDR_ID, PVPVOF, 12);

  rs485Bus.readProcess(RTD_MUX_ID, PVPVOF, 12);
  rs485Bus.writeProcess(RTD_MUX_ID, PVPVOF, 12);
*/
}


void loop() {
  // put your main code here, to run repeatedly:

}
