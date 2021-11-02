#include <inttypes.h>
#include "handler.h"


handler::handler(uint8_t id, uint8_t* tx_buff, uint8_t tx_buff_size, uint8_t* rx_buff, uint8_t rx_buff_size)
  : m_id(id), m_tx_buff(tx_buff), m_tx_buff_size(tx_buff_size), m_rx_buff(rx_buff), m_rx_buff_size(rx_buff_size)
{}

handler::~handler() {};

extern "C"
{
// build and send a read command - read the read command response
cmdResp handler::readProcess(Serial& so, uint16_t param_addr, uint16_t byte_cnt)
{
  cmd c;
  ssize_t  bytesWritten  = 0;
  ssize_t  bytesRead     = 0;


  // bulid the command in the m_tx_buff
  c.buildReadCmd(m_tx_buff, m_tx_buff_size, byte_cnt, m_id, param_addr);

  // the command that is in the m_tx_buff
  bytesWritten = sndCmd(so, c);

  if( (c.cmdLength() == bytesWritten) )
  {
    // read the response into the m_rx_buff
    bytesRead = rcvReadResp(so, READ_RESP_PKT_LEN);   // return function style cast

    if( (-1 != bytesRead) )
    {
      return(c.buildReadResp(m_rx_buff, bytesRead, m_id));
    } else
    {
      fprintf(stderr, "rcvReadResp failed for id [%" PRIu8 "], returns [%" PRId64 "] bytes read\n",
        m_id, bytesRead);
    }
  }

  fprintf(stderr, "sndCmd failed for id [%" PRIu8 "], returns [%" PRId64 "] bytes written\n",
    m_id, bytesWritten);

  return(cmdResp(false, 0, 0));
}


// build and send a write command - read the write command response
cmdResp handler::writeProcess(Serial& so, uint16_t param_addr, uint16_t byte_cnt)
{
  cmd c;
  ssize_t  bytesWritten  = 0;
  ssize_t  bytesRead     = 0;


  // bulid the command in the m_tx_buff
  c.buildWriteCmd(m_tx_buff, m_tx_buff_size, byte_cnt, m_id, param_addr);

  // the command that is in the m_tx_buff
  bytesWritten = sndCmd(so, c);

  if( (c.cmdLength() == bytesWritten) )
  {
    // read the response into the m_rx_buff
    bytesRead = rcvWriteResp(so, WRITE_RESP_PKT_LEN);   // return function style cast

    if( (-1 != bytesRead) )
    {
      return(c.buildWriteResp(m_rx_buff, bytesRead, m_id));
    } else
    {
      fprintf(stderr, "rcvWriteResp failed for id [%" PRIu8 "], returns [%" PRIu64 "] bytes read\n",
        m_id, bytesRead);
    }
  }

  fprintf(stderr, "sndCmd failed for id [%" PRIu8 "], returns [%" PRIu64 "] bytes written\n",
    m_id, bytesWritten);

  return(cmdResp(false, 0, 0));
}
} // end extern "C"


// use the so object to send the packet
ssize_t handler::sndCmd(Serial& so, cmd& c)
{
  // discard rx and tx queue
  so.flush(TCIOFLUSH);

  // write the bytes, handle the return code in the caller
  return(so.writeByte(m_tx_buff, c.cmdLength()));
}


// read a write response into the m_rx_buff
// this is a write response, so expecting to get 8 bytes
ssize_t handler::rcvWriteResp(Serial& so, size_t min_pkt_size)
{
  uint8_t bytesReceived = 0;


  memset(m_rx_buff, '\0', m_rx_buff_size);

  //
  // want to read min_pkt_size bytes and less than m_rx_buff_size
  //
  // this is where we go Arduino and use their Serial object
  //
  // TODO: add the Serial read here with the millis() implementation
  // to try to read for some period of time before timing out
  //
  // bytesReceived = Serial3.read() ... bla bla bla
  //
  // if all good proceed to bulidWriteResp
  //
  return(bytesReceived);
}


// read a read response into the m_rx_buff
// this is a read response, so expecting to get 8 bytes or more depending
// on ByteCnt
ssize_t handler::rcvReadResp(Serial& so, size_t min_pkt_size)
{
  uint8_t bytesReceived = 0;


  memset(m_rx_buff, '\0', m_rx_buff_size);

  //
  // want to read min_pkt_size bytes and less than m_rx_buff_size
  //
  // this is where we go Arduino and use their Serial object
  //
  // TODO: add the Serial read here with the millis() implementation
  // to try to read for some period of time before timing out
  //
  // bytesReceived = Serial3.read() ... bla bla bla
  //
  // if all good proceed to bulidWriteResp
  //
  return(bytesReceived);
}


