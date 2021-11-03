#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "commands.h"
#include "ctrlr_commands.h"
#include "crc16.h"


extern "C"
{

cmd::cmd() : m_paramAddr(0), m_cmdLength(0), m_data(0) {}
cmd::~cmd() {}
uint16_t  cmd::paramAddr() const { return(m_paramAddr); }
uint8_t   cmd::cmdLength() const { return(m_cmdLength); }
uint8_t   cmd::dataLength() const { return(m_data); }


// validate pkt Rx'ed from the temp controller
bool cmd::validateCtrlrRxPkt(uint8_t* buff, uint8_t bufflen, uint8_t id, bool isWriteCmd)
{
  uint16_t  crc;


  //
  // check if this is a runt or an error packet
  //
  if( (CTRLR_MIN_RX_PKT_LEN > bufflen) )
  {
    fprintf(stdout, "short packet received len [%" PRIu8 "]\n", bufflen);
    return(false);
  }

  //
  // check if this is a function error, address error, or data error repsonse
  // and barf those bytes up in the response
  // for now, check if this is the packet expected to be decoding
  // and if not return false and barf up the whole packet
  //
  if( (false == isWriteCmd) )
  {
    // must be same id, a read function, and the whole packet must be present
    // i.e. byte count + 6 where is the count of bytes other than the 2 bytes at CTRLR_READ_BYTE_CNT_OFFSET
    if( ((id != buff[CTRLR_ID_OFFSET]) || (CTRLR_READ_FUNC != buff[CTRLR_FUNC_OFFSET]) ||
        ((*(reinterpret_cast<uint16_t*>(&buff[CTRLR_READ_BYTE_CNT_OFFSET]))) + 6) != bufflen) )
    {
      // return the buff with only the returned data bytes from the controller
      // return those data byte length
      fprintf(stdout, "unexpected read response expected id:func:pktlen %" PRIx8 ":%" PRIx8 ":%" PRIx16 " got id:func:pktlen %" PRIx8 ":%" PRIx8":%" PRIx8"\n",
        id, CTRLR_READ_FUNC, bufflen,
        buff[CTRLR_ID_OFFSET], buff[CTRLR_FUNC_OFFSET], (*(reinterpret_cast<uint16_t*>(&buff[CTRLR_READ_BYTE_CNT_OFFSET])) + 6));

      return(false);
    }
  } else // is a write command, verify the data echo'ed is what was sent
  {
    if( ((id != buff[CTRLR_ID_OFFSET]) || (CTRLR_WRITE_FUNC != buff[CTRLR_FUNC_OFFSET]) ||
        (m_data != *(reinterpret_cast<uint16_t*>(&buff[CTRLR_WRITE_DATA_OFFSET])))) )
    {
      fprintf(stdout, "unexpected write response expected id:func:data %" PRIx8 ":%" PRIx8 ":%" PRIx8 " got id:func:data %" PRIx8 ":%" PRIx8":%" PRIx16"\n",
        id, CTRLR_WRITE_FUNC, m_data,
        buff[CTRLR_ID_OFFSET], buff[CTRLR_FUNC_OFFSET], *(reinterpret_cast<uint16_t*>(&buff[CTRLR_WRITE_DATA_OFFSET])));

      return(false);
    }
  }

  //
  // verify the CRC
  //
  crc = calcCRC16(buff, bufflen - 2);   // where bufflen is the number of bytes
                                        // read NOT the absolute size of the buffer

  //
  // the crc is the last word ( last two bytes )
  //
  // and in the event of a read response, but CRC could be further away than
  // CTRLR_CRC_OFFSET, so check bufflen - 2
  //
  if( (*(reinterpret_cast<uint16_t*>(&buff[bufflen - 2])) != crc) )
  {
    fprintf(stdout, "bad CRC received [%" PRIx16 "] expected [%" PRIx16 "]\n",
      *(reinterpret_cast<uint16_t*>(&buff[bufflen - 2])), crc);

    return(false);
  }

  return(true);
}


//-----------------------------------------------------------------------------
// READ COMMANDS
//
uint8_t cmd::buildReadCmd(uint8_t* buff, uint8_t bufflen, uint16_t dataLength, uint8_t id, uint16_t param_addr)
{
  // initialize buff
  memset(buff, '\0', bufflen);


  // build the commnad in the buff as a read operation
  buff[CTRLR_ID_OFFSET]   = id;
  *(reinterpret_cast<uint16_t*>(&buff[CTRLR_FUNC_OFFSET])) = CTRLR_READ_FUNC;  // TODO: htons needed here on Due ?
  *(reinterpret_cast<uint16_t*>(&buff[CTRLR_PARAM_ADDR])) = param_addr;        // TODO: htons needed here on Due ?
  *(reinterpret_cast<uint16_t*>(&buff[CTRLR_READ_DATA_CNT_OFFSET])) = dataLength; // TODO: same ..
  *(reinterpret_cast<uint16_t*>(&buff[CTRLR_CRC_OFFSET])) = calcCRC16(buff, CTRLR_READ_DATA_CNT_OFFSET + 2);

  //uint16_t crc = calcCRC16(buff, CTRLR_READ_DATA_CNT_OFFSET + 2);
  //fprintf(stdout, "adding crc [%" PRIx16 "]\n", crc);

  m_paramAddr   = param_addr;
  m_data        = dataLength; // save the request data length ( byte count )
  m_cmdLength   = CTRLR_CRC_OFFSET + 2;
  return(m_cmdLength); // huh ?  this is always going to be 8
}


//
// build the response for the PVPVOF command
// the buff has the Rx'ed pkt from the temp controller
//
cmdResp cmd::buildReadResp(uint8_t* buff, uint8_t bufflen, uint8_t id)
{
  // validate the Rx'ed packet
  if( (!validateCtrlrRxPkt(buff, bufflen, id, false)) )
  {
    return(cmdResp(false, buff, bufflen));
  }

  // return the address of the beginning of the returned data bytes . . 
  // or just return the whole packet . . ?
  return(cmdResp(true,                                                 // success
         &buff[CTRLR_READ_DATA_CNT_OFFSET],                            // the data
         *(reinterpret_cast<uint16_t*>(&buff[CTRLR_READ_BYTE_CNT_OFFSET])))); // the length of the returned data
}




//-----------------------------------------------------------------------------
// WRITE COMMANDS
//
uint8_t cmd::buildWriteCmd(uint8_t* buff, uint8_t bufflen, uint16_t data, uint8_t id, uint16_t param_addr)
{
  // initialize buff
  memset(buff, '\0', bufflen);

  // build the commnad in the buff as a read operation
  buff[CTRLR_ID_OFFSET]   = id;
  *(reinterpret_cast<uint16_t*>(&buff[CTRLR_FUNC_OFFSET])) = CTRLR_WRITE_FUNC;  // TODO: htons needed here on Due ?
  *(reinterpret_cast<uint16_t*>(&buff[CTRLR_PARAM_ADDR])) = param_addr;         // TODO: htons needed here on Due ?
  *(reinterpret_cast<uint16_t*>(&buff[CTRLR_WRITE_DATA_OFFSET])) = data;        // TODO: same ..
  *(reinterpret_cast<uint16_t*>(&buff[CTRLR_CRC_OFFSET])) = calcCRC16(buff, CTRLR_WRITE_DATA_OFFSET + 2);

  //uint16_t crc = calcCRC16(buff, CTRLR_WRITE_DATA_OFFSET + 2);
  //fprintf(stdout, "adding crc [%" PRIx16 "]\n", crc);

  m_paramAddr   = param_addr;
  m_data        = data; // save the request data length ( byte count )
  m_cmdLength   = CTRLR_CRC_OFFSET + 2;
  return(m_cmdLength); // huh ?  this is always going to be 8
}


//
// build the response for the PVPVOF command
// the buff has the Rx'ed pkt from the temp controller
//
cmdResp cmd::buildWriteResp(uint8_t* buff, uint8_t bufflen, uint8_t id)
{
  // validate the Rx'ed packet
  if( (!validateCtrlrRxPkt(buff, bufflen, id, true)) )
  {
    return(cmdResp(false, buff, bufflen));
  }

  // return the address of the beginning of the returned data bytes . . 
  // or just return the whole packet . . ?
  return(cmdResp(true, 0, 0));
}

} 
