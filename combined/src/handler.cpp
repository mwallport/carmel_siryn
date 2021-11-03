#include <inttypes.h>
#include "handler.h"


extern "C"
{

handler::handler() {}
handler::~handler() {};


// build and send a read command - read the read command response
cmdResp handler::readProcess(Serial& so, uint8_t id, uint16_t param_addr, uint16_t byte_cnt,
                uint8_t* tx_buff, uint8_t tx_buff_size, uint8_t* rx_buff, uint8_t rx_buff_size)
{
  cmd c;
  ssize_t  bytesWritten  = 0;
  ssize_t  bytesRead     = 0;


  // bulid the command in the tx_buff
  c.buildReadCmd(tx_buff, tx_buff_size, byte_cnt, id, param_addr);

  fprintf(stdout,"readProcess build cmd: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
    tx_buff[0], tx_buff[1], tx_buff[2], tx_buff[3], tx_buff[4], tx_buff[5], tx_buff[6], tx_buff[7]);

  // the command that is in the tx_buff
  bytesWritten = sndCmd(so, c);

  if( (c.cmdLength() == bytesWritten) )
  {
    // read the response into the m_rx_buff
    bytesRead = rcvReadResp(so, READ_RESP_PKT_LEN, rx_buff, rx_buff_size);   // return function style cast

    if( (-1 != bytesRead) )
    {
      return(c.buildReadResp(rx_buff, bytesRead, id));
    } else
    {
      fprintf(stdout, "rcvReadResp failed for id [%" PRIu8 "], returns [%" PRId64 "] bytes read\n",
        id, bytesRead);
    }
  }

  fprintf(stdout, "sndCmd failed for id [%" PRIu8 "], returns [%" PRId64 "] bytes written\n",
    id, bytesWritten);

  return(cmdResp(false, 0, 0));
}


// build and send a write command - read the write command response
cmdResp handler::writeProcess(Serial& so, uint8_t id, uint16_t param_addr, uint16_t data,
                  uint8_t* tx_buff, uint8_t tx_buff_size, uint8_t* rx_buff, uint8_t rx_buff_size)
{
  cmd c;
  ssize_t  bytesWritten  = 0;
  ssize_t  bytesRead     = 0;


  // bulid the command in the tx_buff
  c.buildWriteCmd(tx_buff, tx_buff_size, data, id, param_addr);

  fprintf(stdout,"writeProcess build cmd: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
    tx_buff[0], tx_buff[1], tx_buff[2], tx_buff[3], tx_buff[4], tx_buff[5], tx_buff[6], tx_buff[7]);

  // the command that is in the tx_buff
  bytesWritten = sndCmd(so, c);

  if( (c.cmdLength() == bytesWritten) )
  {
    // read the response into the m_rx_buff
    bytesRead = rcvWriteResp(so, WRITE_RESP_PKT_LEN, rx_buff, rx_buff_size);   // return function style cast

    if( (-1 != bytesRead) )
    {
      return(c.buildWriteResp(rx_buff, bytesRead, id));
    } else
    {
      fprintf(stdout, "rcvWriteResp failed for id [%" PRIu8 "], returns [%" PRIu64 "] bytes read\n",
        id, bytesRead);
    }
  }

  fprintf(stdout, "sndCmd failed for id [%" PRIu8 "], returns [%" PRIu64 "] bytes written\n",
    id, bytesWritten);

  return(cmdResp(false, 0, 0));
}


// use the so object to send the packet
ssize_t handler::sndCmd(Serial& so, cmd& c)
{
  // discard rx and tx queue
  // TODO: undo this debug BS
  // so.flush();

  // write the bytes, handle the return code in the caller
  //return(so.writeByte(tx_buff, c.cmdLength()));
  //
  // TODO: undo this debug BS
  return(c.cmdLength());
}


// read a write response into the m_rx_buff
// this is a write response, so expecting to get 8 bytes
ssize_t handler::rcvWriteResp(Serial& so, size_t min_pkt_size, uint8_t* rx_buff, uint8_t rx_buff_size)
{
  uint8_t bytesReceived = 0;


  memset(rx_buff, '\0', rx_buff_size);

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
  // only for testing - copy the tx_buff into m_rx_buff and return
  //
  // TODO: implement the read
  //
  // TODO: fudge this better
  //
  //memcpy(tx_buff, m_rx_buff, min_pkt_size);
  bytesReceived = min_pkt_size;
  return(bytesReceived);
}


// read a read response into the m_rx_buff
// this is a read response, so expecting to get 8 bytes or more depending
// on ByteCnt
ssize_t handler::rcvReadResp(Serial& so, size_t min_pkt_size, uint8_t* rx_buff, uint8_t rx_buff_size)
{
  uint8_t bytesReceived = 0;


  memset(rx_buff, '\0', rx_buff_size);

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
  // for testing only copy bytes in the m_rx_buff and return its length
  //
  // TODO: implement the read
  //
  //memcpy(rx_buff, tx_buff, min_pkt_size);  // TODO: fudge this better
  bytesReceived = min_pkt_size;
  return(bytesReceived);
}

} // end extern "C"

