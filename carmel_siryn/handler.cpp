//#include <SPI.h>
//#include <Controllino.h>
#include <inttypes.h>
#include "handler.h"
#include "common.h"
#include "ctrlr_commands.h"
#include "Arduino.h"

extern "C"
{

handler::handler() {}
handler::~handler() {};


// build and send a read command - read the read command response
cmdResp handler::readProcess(HardwareSerial& so, uint8_t id, uint16_t param_addr, uint16_t byte_cnt,
        uint8_t* tx_buff, uint8_t tx_buff_size, uint8_t* rx_buff, int8_t rx_buff_size)
{
  cmd c;
  int  bytesWritten = 0;
  int  bytesRead    = 0;


  // bulid the command in the tx_buff
  c.buildReadCmd(tx_buff, tx_buff_size, byte_cnt, id, param_addr);

  #ifdef __DEBUG_MODBUS_CMDS__
  Serial.print("readProcess command is : ");
  for(int i = 0; i < c.cmdLength(); i++) {  Serial.print(tx_buff[i], 16); Serial.print(" "); }
  Serial.println("");
  #endif

  // the command that is in the tx_buff
  bytesWritten = sndCmd(so, tx_buff, c.cmdLength());

  if( (c.cmdLength() == bytesWritten) )
  {
  // read the response into the m_rx_buff
    bytesRead = rcvReadResp(so, READ_RESP_PKT_LEN, rx_buff, rx_buff_size);   // return function style cast

    if( (-1 != bytesRead) )
    {
      #ifdef __DEBUG_MODBUS_CMDS__
      Serial.print("rvcReadResp got "); Serial.print(bytesRead); Serial.print(" bytes: ");
      for(int i = 0; i < bytesRead; i++) {  Serial.print(rx_buff[i], 16); Serial.print(" "); }
      Serial.println("");
      #endif
    
      return(c.buildReadResp(rx_buff, bytesRead, id));
      
    } else
    {
      #ifdef __DEBUG_MODBUS_CMDS__
      Serial.print("rvcReadResp failed");
      #endif
    }
  } else
  {
    #ifdef __DEBUG_MODBUS_CMDS__
    Serial.print("not all bytes written");
    #endif    
  }

  #ifdef __DEBUG_MODBUS_CMDS__
  Serial.print("readProcess sndCmd failed");
  #endif

  return(cmdResp(false, 0, 0));
}


// build and send a write command - read the write command response
cmdResp handler::writeProcess(HardwareSerial& so, uint8_t id, uint16_t param_addr, uint16_t data,
          uint8_t* tx_buff, uint8_t tx_buff_size, uint8_t* rx_buff, int8_t rx_buff_size)
{
  cmd c;
  int  bytesWritten = 0;
  int  bytesRead    = 0;


  // bulid the command in the tx_buff
  c.buildWriteCmd(tx_buff, tx_buff_size, data, id, param_addr);


  #ifdef __DEBUG_MODBUS_CMDS__
  Serial.print("writeProcess command is : ");
  for(int i = 0; i < c.cmdLength(); i++) {  Serial.print(tx_buff[i], 16); Serial.print(" "); }
  Serial.println("");
  #endif

  // the command that is in the tx_buff
  bytesWritten = sndCmd(so, tx_buff, c.cmdLength());

  if( (c.cmdLength() == bytesWritten) )
  {
  // read the response into the m_rx_buff
  bytesRead = rcvWriteResp(so, WRITE_RESP_PKT_LEN, rx_buff, rx_buff_size);   // return function style cast


  if( (-1 != bytesRead) )
  {
    #ifdef __DEBUG_MODBUS_CMDS__
    Serial.print("rvcWriteResp got "); Serial.print(bytesRead); Serial.print(" bytes: ");
    for(int i = 0; i < bytesRead; i++) {  Serial.print(rx_buff[i], 16); Serial.print(" "); }
    Serial.println("");
    #endif
    
    return(c.buildWriteResp(rx_buff, bytesRead, id));
  } else
  {
    #ifdef __DEBUG_MODBUS_CMDS__
    Serial.println("rvcWriteResp failed");
    #endif
  }
  }

  #ifdef __DEBUG_MODBUS_CMDS__
  Serial.print("writeProcess sndCmd failed");
  #endif

  return(cmdResp(false, 0, 0));
}


// use the so object to send the packet
int handler::sndCmd(HardwareSerial& so, uint8_t* tx_buff, int8_t bufflen)
{
  int retVal;
  volatile int writeDelay  = 3000;   // 

  
  // write the bytes, handle the return code in the caller
  #ifdef __DEBUG_MODBUS_TXRX__
  Serial.println("setting pin RS485_WRITE_ENABLE HIGH and sending bytes...");
  #endif
  
  digitalWrite(RS485_WRITE_ENABLE, HIGH);
  
  retVal = so.write(tx_buff, bufflen);
  so.flush();

  while( (writeDelay-- > 0) );  // delay to let the board complete Tx

  digitalWrite(RS485_WRITE_ENABLE, LOW);

  #ifdef __DEBUG_MODBUS_TXRX__
  Serial.print("set RS485_WRITE_ENABLE LOW and sent "); Serial.print(retVal); Serial.println(" bytes...");
  #endif
  
  return(retVal);
}


// read a write response into the m_rx_buff
// this is a write response, so expecting to get 8 bytes
int handler::rcvWriteResp(HardwareSerial& so, int8_t min_pkt_size, uint8_t* rx_buff, int8_t rx_buff_size)
{
  int bytes_read  = 0;
  bool timed_out  = false;
  unsigned long start_time = millis();


  memset(rx_buff, '\0', rx_buff_size);

  //Controllino_RS485RxEnable();

  // always going to try to read 8 bytes
  // TODO: generate fail return from unit and verify it is also 8 bytes
  while( ((!timed_out) && (bytes_read < min_pkt_size) && (bytes_read < rx_buff_size)) )
  {
    if( ((millis() - start_time) > _READ_TIME_OUT_) )  // 3 seconds
    {
      #ifdef __DEBUG_MODBUS_TXRX__
      Serial.println("rcvWriteResp timed out");
      #endif
      
      timed_out = true;
      break;
    }

    if( (0 < so.available()) )
    {
      rx_buff[bytes_read] = so.read();

      if( (0 == bytes_read) && (0 == rx_buff[bytes_read]) )
      {
        #ifdef __DEBUG_MODBUS_TXRX__
        Serial.println("rcvWriteResp dropping 1st byte 0");
        #endif

        continue;
      }
      
      #ifdef __DEBUG_MODBUS_TXRX__
      Serial.print("rcvWriteResp read byte: 0x"); Serial.print(rx_buff[bytes_read], 16); Serial.print(" into index: "); Serial.println(bytes_read);
      #endif

      bytes_read ++;
      
    } else
    {
      // delay, let the unit respond
      delay(100);
    }
  }

  //Controllino_RS485TxEnable();
  return(bytes_read);
}


// read a read response into the m_rx_buff
// this is a read response, so expecting to get 8 bytes or more depending
// on ByACUnt
int handler::rcvReadResp(HardwareSerial& so, int8_t min_pkt_size, uint8_t* rx_buff, int8_t rx_buff_size)
{
  int bytes_read  = 0;
  bool timed_out  = false;
  int length    = min_pkt_size;  // this will change during reading due to byte count
  uint16_t byte_cnt = 0;
  unsigned long start_time = millis();


  memset(rx_buff, '\0', rx_buff_size);

  //Controllino_RS485RxEnable();

  // always going to try to read 8 bytes
  // TODO: generate fail return from unit and verify it is also 8 bytes
  while( ((!timed_out) && (bytes_read < length) && (bytes_read < rx_buff_size)) )
  {
    if( ((millis() - start_time) > _READ_TIME_OUT_) )  // 3 seconds
    {
      #ifdef __DEBUG_MODBUS_TXRX__
      Serial.println("rcvReadResp timed out");
      #endif
      
      timed_out = true;
      break;
    }

    if( (0 < so.available()) )
    {
      rx_buff[bytes_read] = so.read();

      if( (0 == bytes_read) && (0 == rx_buff[bytes_read]) )
      {
        #ifdef __DEBUG_MODBUS_TXRX__
        Serial.println("rcvReadResp dropping 1st byte 0");
        #endif

        continue;
      }

      #ifdef __DEBUG_MODBUS_TXRX__
      Serial.print("rcvReadResp read byte: 0x"); Serial.print(rx_buff[bytes_read], 16); Serial.print(" into index: "); Serial.println(bytes_read);
      #endif

      // handle CTRLR_READ_BYTE_CNT_OFFSET data
      // there is a word in the pac ket that is the count of bytes after it
      // then there is the CRC word after those bytes
      if( (CTRLR_READ_BYTE_CNT_OFFSET + 1 == bytes_read) )
      {
        byte_cnt = (*(reinterpret_cast<uint16_t*>(&rx_buff[bytes_read])));

        if( (byte_cnt > 2) )
        {
          length = _PKT_BYTS_EXCEPT_BYTE_CNT_BYTES_ + byte_cnt;

          #ifdef __DEBUG_MODBUS_TXRX__
          Serial.print("rcvReadResp found byte_cnt: "); Serial.print(byte_cnt, 16); Serial.print(" modifying expected length to : "); Serial.println(length);
          #endif
        } else
        {
          // else expected length remains 8
          #ifdef __DEBUG_MODBUS_TXRX__
          Serial.print("rcvReadResp found byte_cnt of: "); Serial.print(byte_cnt, 16); Serial.println(" not adjusting expected length");
          #endif
        }
      }

      bytes_read++;
      
    } else
    {
      // delay, let the unit respond
      delay(100);
    }
  }

  //Controllino_RS485TxEnable();
  return(bytes_read);
}

} // end extern "C"
