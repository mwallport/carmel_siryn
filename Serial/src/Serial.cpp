#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <inttypes.h>
#include "Serial.h"


extern "C"
{

//
// pick up member variable values
//
Serial::Serial(const std::string& port, const int speed)
  : m_fd(0), m_FILE(0), m_port(port), m_speed(speed) {};


//
// close the m_fd if not 0
//
Serial::~Serial()
{
  if( (0 != m_fd) )  // pretty sure m_fd will never EVER be stdin
    close(m_fd);
}


//
// open the port, configure it for use
//
bool Serial::begin()
{
  struct  termios options;


  m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY);

  if( (m_fd == -1) )
  {
    fprintf(stderr, "unable to open %s\n", m_port.c_str());
    return(false);
  }

  //
  // get a FILE* to support the peek function - do this now or at bottom ?
  //
  if( (NULL == (m_FILE = fdopen(m_fd, "r+"))) )
  {
    fprintf(stderr, "unable to fdopen for %s\n", m_port.c_str());
    close(m_fd);
    m_fd  = 0;
    return(false);
  }

  //
  // get the current serial port configuration and change it for 'our' purposes
  //
  memset(&options, '\0', sizeof(options));
  if( (0 != tcgetattr(m_fd, &options)) )
  {
    fprintf(stderr, "unable to tcgetattr for %s\n", m_port.c_str());
    return(false);
  }

  // set the input speed
  switch(m_speed)
  {
      case 19200:
          cfsetispeed(&options, B19200);
          cfsetospeed(&options, B19200);
          break;

      case 38400:
          cfsetispeed(&options, B38400);
          cfsetospeed(&options, B38400);
          break;

      case 57600:
          cfsetispeed(&options, B57600);
          cfsetospeed(&options, B57600);
          break;

      default:
      case 9600:
          cfsetispeed(&options, B9600);
          cfsetospeed(&options, B9600);
          break;
  }

  // Enable the receiver and set local mode...
  options.c_cflag |= (CLOCAL | CREAD);

  // 8N1
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // no hardware flow control
  options.c_cflag &= ~CRTSCTS;

  // use raw input
  options.c_lflag &= ~(ICANON | ISIG | ECHO | ECHOE);

  // no software flow control
  options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

  // raw output
  options.c_oflag &= ~(OPOST | ONLCR | OCRNL);

  options.c_cc[VMIN] = 0; // always want 1 byte
  options.c_cc[VTIME]= 10; // wait 100th of a second for it

  //Set the new options for the port...
  if( (0 != tcsetattr(m_fd, TCSANOW, &options)) )
  {
    fprintf(stderr, "unable to tcgetattr for %s\n", m_port.c_str());
    return(false);
  }


  return(true);
}


//
// read a byte
//
uint8_t Serial::readByte(void)
{
  uint8_t byte;
  ssize_t retVal;

  retVal = read(m_fd, (void*)&byte, 1);

  if( (-1 == retVal) )
  {
    return(-1);
  }

  return(byte);
}


uint8_t Serial::readByte(uint8_t* buff, const ssize_t len)
{
  ssize_t nread  = 0;
  ssize_t retVal = 0;

  fprintf(stderr,"entering [%s]\n", __PRETTY_FUNCTION__);

  if( (NULL == buff) || (0 >= len) )
    return(0);

  do
  {
    retVal = read(m_fd, (void*)&buff[nread], (len - nread));
    fprintf(stderr,"read [%" PRId64 "] bytes..\n", retVal);

    if( (0 == retVal) )
      break;

    nread += retVal;

  } while( (0 < nread) && (len != nread) );

  if( (-1 == nread) )
  {
    fprintf(stderr, "error writing to [%s], errno [%d], errorstr [%s]\n",
      m_port.c_str(), errno, strerror(errno));

    return(-1);
  }

  fprintf(stderr,"exiting [%s]\n", __PRETTY_FUNCTION__);

  return(nread);
}


//
// return the count of bytes available to read
//
const size_t Serial::available()
{
  int bytes = 0;


  if( (0 != ioctl(m_fd, FIONREAD, &bytes)) )
  {
    fprintf(stderr, "unable to get available bytes for %s\n", m_port.c_str());
    return(false);
  }

  return(bytes);
}


//
// returns the count of bytes written
//
const size_t Serial::writeByte(const uint8_t byte)
{
  ssize_t retVal;

  retVal = write(m_fd, (const void*)&byte, 1);

  if( (-1 == retVal) )
  {
    fprintf(stderr, "write failed on [%s] with error [%d], [%s]\n",
      m_port.c_str(), errno, strerror(errno));

    return(-1);
  }

  return(retVal);
}


//
// returns the count of bytes written
//
const size_t Serial::writeByte(const uint8_t* p_byte)
{
  if( (0 == p_byte) )
    return(0);

  return(writeByte(*p_byte));
}

//
// returns the count of bytes written
//
const size_t Serial::writeByte(const uint8_t* p_byte, const ssize_t len)
{
  ssize_t nwritten  = 0;


  if( (NULL == p_byte) )
    return(0);

  do
  {
    nwritten = write(m_fd, (const void*)&p_byte[nwritten], (len - nwritten));
  } while( (-1 != nwritten) && (len != nwritten) );

  if( (-1 == nwritten) )
  {
    fprintf(stderr, "error writing to [%s], errno [%d], errorstr [%s]\n",
      m_port.c_str(), errno, strerror(errno));
  }

  return(nwritten);
}


// queu is 
//    TCIOFLUSH - discard Rx and Tx buff 
//    TCIFLUSH  - discard Rx buff 
//    TCOFLUSH  - discard Tx buff 
int Serial::flush(const int queue)
{
  return(tcflush(m_fd, queue));
}

// waits until all output written to the object referred to by fd has been transmitted
int Serial::drain()
{
  return(tcdrain(m_fd));
}

}

