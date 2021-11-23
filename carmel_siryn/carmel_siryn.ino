#include <SPI.h>
#include <Controllino.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include <inttypes.h>
#include "common.h"
#include "handler.h"
#include "ctrlr_commands.h"
#include "deviceHandler.h"



// these need to match the actual physical assignments
uint8_t   ASIC_ID     = 1;
uint8_t   DDR_ID      = 2;
uint8_t   RTD_MUX_ID  = 3;
uint32_t  val;


//
// buffer used by the modbus handler
//
static const uint8_t MAX_BUFF_LENGTH    = 255;
uint8_t tx_buff[MAX_BUFF_LENGTH];
uint8_t rx_buff[MAX_BUFF_LENGTH];


void setup() {
  // put your setup code here, to run once:

  Controllino_RS485Init();
  Serial3.begin(19200);
  Controllino_RS485RxEnable();

  Serial.begin(19200);

  while( (!Serial) && (!Serial3) )
    delay(500);

  //let me open the Serial monitor .. !
  Serial.println("");
  Serial.println("delaying...");
  delay(5000);
  Serial.println("starting...");
  

  if( (!Serial3) )
  {
    //fprintf(stderr, "unable to start the serial port\n");
    exit(1);
  }

  deviceHandler rs485Bus(Serial3, tx_buff, MAX_BUFF_LENGTH, rx_buff, MAX_BUFF_LENGTH);

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


  
  //
  // check the error pkt returns
  //
  // try to write a read only - try to write to PCPVOF
  // 
  // get a short packet back something like 5 bytes: 1 86 2 C3 A1
  //
/*
  cmdResp writePVPVOF = rs485Bus.writeProcess(ASIC_ID, PVPVOF, 0x0220);
  if( (false == writePVPVOF.retCode()) )
    Serial.println("writeProcess(ASIC_ID, PVPVOF, 0x0220) failed");
  else
    Serial.println("writeProcess(ASIC_ID, PVPVOF, 0x0220) success");
*/
  //
  // request a read of 0x03fe byte count
  //
  // - the unit will return all registers, which is cool can get them all in 1 shot
  //
  /*
  cmdResp readSVbig = rs485Bus.readProcess(ASIC_ID, SV, 0x03fe);
  if( (false == readSVbig.retCode()) )
    Serial.println("big readProcess(ASIC_ID, SV, 0x03fe) failed");
  else
    Serial.println("big readProcess(ASIC_ID, SV, 0x03fe) success");
*/
/*
  // request a read of function that doesn't exist 0x101C - this succeed, must be undocumented parameter ?
  cmdResp read_bad = rs485Bus.readProcess(ASIC_ID, 0x101C, 0x02);
  if( (false == read_bad.retCode()) )
    Serial.println("bad readProcess(ASIC_ID, 0x101C, 0x02) failed");
  else
    Serial.println("bad readProcess(ASIC_ID, 0x101C, 0x02) success");

  // request a write of function that doesn't exist 0x0060
  //
  // this resulted in short pkt 5 bytes : 1 86 2 C3 A1
  //
  cmdResp write_bad2 = rs485Bus.writeProcess(ASIC_ID, 0x0060, 0x007f);
  if( (false == write_bad2.retCode()) )
    Serial.println("bad writeProcess(ASIC_ID, 0x0060, 0x0060) failed");
  else
    Serial.println("bad writeProcess(ASIC_ID, 0x0060, 0x0060) success");

  //
  // request a write of to the wrong id
  //
  // get no response - which is expected, there is no id 2 on the bus
  // 
  cmdResp writeSVbadaddr = rs485Bus.writeProcess(ASIC_ID + 1, SV, 0x0220);
  if( (false == writeSVbadaddr.retCode()) )
    Serial.println("badaddr writeProcess(ASIC_ID + 1, SV, 0x0220) failed");
  else
    Serial.println("badaddr writeProcess(ASIC_ID + 1, SV, 0x0220) success");

  //
  // request read from bad addr
  //
  // - returns a pkt of all 0's
  //
  cmdResp readSVbadaddr = rs485Bus.readProcess(ASIC_ID + 1, SV, 2);
  if( (false == readSVbadaddr.retCode()) )
    Serial.print("bad readProcess(ASIC_ID, SV, 2) failed");
  else
    Serial.print("bad readProcess(ASIC_ID, SV, 2) success");
*/
}


void loop() {
  // put your main code here, to run repeatedly:
 

}
