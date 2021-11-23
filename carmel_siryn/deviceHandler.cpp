#include <inttypes.h>
#include "deviceHandler.h"


deviceHandler::deviceHandler(HardwareSerial& so, uint8_t* tx_buff, uint8_t tx_buff_size,
                              uint8_t* rx_buff, uint8_t rx_buff_size)
  : m_Serial(so), m_tx_buff(tx_buff), m_tx_buff_size(tx_buff_size), m_rx_buff(rx_buff),
    m_rx_buff_size(rx_buff_size) {}


deviceHandler::~deviceHandler() {}

cmdResp deviceHandler::readProcess(uint8_t id, uint16_t param_addr, uint16_t byte_cnt)
{
  return(m_handler.readProcess(m_Serial, id, param_addr, byte_cnt, m_tx_buff, MAX_BUFF_SIZE, m_rx_buff, MAX_BUFF_SIZE));
}


cmdResp deviceHandler::writeProcess(uint8_t id, uint16_t param_addr, uint16_t data)
{
  return(m_handler.writeProcess(m_Serial, id, param_addr, data, m_tx_buff, MAX_BUFF_SIZE, m_rx_buff, MAX_BUFF_SIZE));
}
