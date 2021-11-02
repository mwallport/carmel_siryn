#ifndef __HANDLER__
#define __HANDLER__
#include <inttypes.h>
#include "Serial.h"
#include "commands.h"


class handler
{
  public:
  handler(uint8_t id, uint8_t* tx_buff, uint8_t tx_buff_len, uint8_t* rx_buff, uint8_t rx_buff_len);
  virtual ~handler();

  // send a command and do return rcvReadResp
  cmdResp readProcess(Serial& so, uint16_t param_addr, uint16_t byte_cnt);

  // send a command and do return rcvWriteResp
  cmdResp writeProcess(Serial& so, uint16_t param_addr, uint16_t data);

  protected:
  // send a command return the count of bytes written
  ssize_t sndCmd(Serial& so, cmd& command);
  
  // handle reading a response to a write command
  // read into the m_rx_buff and return bytes read
  ssize_t rcvWriteResp(Serial& so, size_t min_pkt_size);
  
  // handle reading a response to a read command - handle the ByteCount in the Rx'ed packet
  // read into the m_rx_buff and return bytes read
  ssize_t rcvReadResp(Serial& so, size_t min_pkt_size);

  uint8_t   m_id;
  uint8_t*  m_tx_buff;
  size_t    m_tx_buff_size;
  uint8_t*  m_rx_buff;
  size_t    m_rx_buff_size;
};

#endif

