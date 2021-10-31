#ifndef __MODBUS_HANDLER__
#define __MODBUS_HANDLER__
//#include "common.h"
#include <cstdint>


extern class  Serial;
extern class  CmdBase;
extern enum   Rs485BusId;

class modbusHandler
{
  public:
  modbusHandler(uint8_t* rx_buff, uint8_t* tx_buff, Rs485BusId);
  virtual ~modbusHandler();
  modbusHandler() = delete;
  modbusHandler(const modbusHandler&) = delete;
  modbusHandler& operator=(const modbusHandler&) = delete;

  read(Serial&, CmdBase&, uint8_t);   // execute modbus read command
  write(Serial&, CmdBase&, uint8_t);  // execute modbus write command

  protected:

  private:

};

#endif

