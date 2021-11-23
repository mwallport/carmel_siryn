#ifndef __COMMANDS__
#define __COMMANDS__
#include <inttypes.h>
#include <stddef.h>

// temp controller read and write cmds
static const uint16_t   CTRLR_READ_FUNC   = 0x03;
static const uint16_t   CTRLR_WRITE_FUNC  = 0x06;
static const int8_t   WRITE_RESP_PKT_LEN  = 8;
static const int8_t   READ_RESP_PKT_LEN   = 8;


class cmdResp
{
  public:
  cmdResp() : m_retCode(false), m_buff(0), m_bufflen(0) {};
  cmdResp(bool retCode, uint8_t* buff, uint8_t bufflen)
  : m_retCode(retCode), m_buff(buff), m_bufflen(bufflen) {}
  cmdResp& operator=(const cmdResp& _cmdResp)
  {
  m_retCode = _cmdResp.m_retCode; m_buff = _cmdResp.m_buff; m_bufflen = _cmdResp.m_bufflen;
  return(*this);      
  }

  virtual ~cmdResp() {};

  bool retCode() { return m_retCode; };
  uint8_t* buff() { return m_buff; };
  uint16_t bufflen() { return (m_bufflen); };

  protected:
  bool    m_retCode;  // true is success, false is failed
  uint8_t*  m_buff;   // for read cmd, this is the data that came back, there are m_byteCnt bytes present
  uint16_t  m_bufflen;  // the lenght of the data in the buff
};


class cmd
{
  public:
  cmd();
  virtual ~cmd();
  uint8_t buildReadCmd(uint8_t* buff, uint8_t bufflen, uint16_t dataCnt, uint8_t id, uint16_t param_addr);
  uint8_t buildWriteCmd(uint8_t* buff, uint8_t bufflen, uint16_t dataCnt, uint8_t id, uint16_t param_addr);
  cmdResp buildReadResp(uint8_t*, uint8_t, uint8_t);
  cmdResp buildWriteResp(uint8_t*, uint8_t, uint8_t);

  cmd(const cmd&) = delete;
  cmd& operator=(const cmd&) = delete;

  bool validateCtrlrRxPkt(uint8_t* buff, uint8_t bufflen, uint8_t id, bool=false);
  uint16_t  paramAddr() const;
  uint8_t   cmdLength() const;
  uint16_t  dataLength() const;

  protected:
  uint16_t  m_paramAddr;
  uint8_t   m_cmdLength;  // length of the cmd, should always be 8
  uint16_t  m_data;     // for read cmds is the requested byte count
              // for write cmds is the data to be written

  private:
};

#endif
