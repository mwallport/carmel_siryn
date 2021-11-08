#ifndef __HANDLER__
#define __HANDLER__
#include <inttypes.h>
#include "Serial.h"
#include "commands.h"

extern "C"
{

class handler
{
  public:
  handler();
  virtual ~handler();

  // send a command and do return rcvReadResp
  cmdResp readProcess(Serial& so, uint8_t id, uint16_t param_addr, uint16_t byte_cnt,
                      uint8_t* tx_buff, uint8_t tx_buff_size, uint8_t* rx_buff, uint8_t rx_buff_size);

  // send a command and do return rcvWriteResp
  cmdResp writeProcess(Serial& so, uint8_t id, uint16_t param_addr, uint16_t data,
                      uint8_t* tx_buff, uint8_t tx_buff_size, uint8_t* rx_buff, uint8_t rx_buff_size);

  protected:
  // send a command return the count of bytes written
  int sndCmd(Serial& so, uint8_t*, uint8_t);
  
  // handle reading a response to a write command
  // read into the m_rx_buff and return bytes read
  int rcvWriteResp(Serial& so, size_t min_pkt_size, uint8_t* rx_buff, uint8_t rx_buff_size);
  
  // handle reading a response to a read command - handle the ByteCount in the Rx'ed packet
  // read into the m_rx_buff and return bytes read
  int rcvReadResp(Serial& so, size_t min_pkt_size, uint8_t* rx_buff, uint8_t rx_buff_size);
};

}

#endif

