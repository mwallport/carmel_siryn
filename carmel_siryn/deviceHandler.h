#ifndef __DEVICE_HANDLER__
#define __DEVICE_HANDLER__
#include <inttypes.h>
//#include "HardwareSerial.h"
#include "handler.h"
#include "commands.h"


const uint8_t   MAX_BUFF_SIZE = 64;


class deviceHandler
{
  public:
  deviceHandler(HardwareSerial& so, uint8_t* tx_buff, uint8_t tx_buff_size, uint8_t* rx_buff, uint8_t rx_buff_size);
  virtual ~deviceHandler();

  cmdResp readProcess(uint8_t id, uint16_t param_addr, uint16_t byte_cnt);
  cmdResp writeProcess(uint8_t id, uint16_t param_addr, uint16_t data);

  protected:
  HardwareSerial& m_Serial;
  handler m_handler;
  uint8_t* m_tx_buff;
  uint8_t  m_tx_buff_size;
  uint8_t* m_rx_buff;
  uint8_t  m_rx_buff_size;
};

#endif
