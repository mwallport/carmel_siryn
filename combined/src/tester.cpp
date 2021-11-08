#include <stdio.h>
#include <inttypes.h>
#include <string>
#include "handler.h"
#include "ctrlr_commands.h"
#include "deviceHandler.h"

static const uint8_t MAX_BUFF_LENGTH    = 64;

uint8_t tx_buff[MAX_BUFF_LENGTH];
uint8_t rx_buff[MAX_BUFF_LENGTH];

uint8_t   ASIC_ID     = 1;
uint8_t   DDR_ID      = 2;
uint8_t   RTD_MUX_ID  = 3;

int main(int arc, char** argv)
{
  std::string port("/dev/ttyUSB0");
  Serial so(port, 19200);

/*
  if( (false == so.begin()) )
  {
    fprintf(stderr, "unable to start the serial port\n");
    exit(1);
  }
*/

  deviceHandler rs485Bus(so, tx_buff, MAX_BUFF_LENGTH, rx_buff, MAX_BUFF_LENGTH);

  //rs485Bus.writeProcess(ASIC_ID, SV, 0x226); <<- 0x226 is transposed
  rs485Bus.writeProcess(ASIC_ID, SV, 0x1e02);
/*
  rs485Bus.readProcess(ASIC_ID, PVPVOF, 2);

  rs485Bus.readProcess(DDR_ID, PVPVOF, 2);
  rs485Bus.writeProcess(DDR_ID, PVPVOF, 2);

  rs485Bus.readProcess(RTD_MUX_ID, PVPVOF, 2);
  rs485Bus.writeProcess(RTD_MUX_ID, PVPVOF, 2);
*/
  return(0);
}

