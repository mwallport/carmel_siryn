// file controlProtocol.cpp
#include <stdio.h>
#include <stdlib.h>

#ifdef __RUNNING_ON_CONTROLLINO__
    #if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
    #else
        #include "WProgram.h"
    #endif
#endif

#include "controlProtocol.h"



// crc16.cpp
uint16_t getCRC16(uint16_t CRC, uint8_t byte)
{
    CRC = ( (CRC % 256) << 8 ) ^ ( CRC16_table_C[ (CRC >> 8) ^ byte ] );
    return (CRC);
}


uint16_t calcCRC16(uint8_t* pBuff, uint16_t length)
{
    uint16_t    CRC = 0;


    for(uint16_t i = 0; i < length; i++)
    {
        CRC = getCRC16(CRC, pBuff[i]);
    }

    return(CRC);
}
// end crc16.cpp


bool controlProtocol::openUSBPort(const char* usbPort, uint32_t Speed)
{
    bool    retVal  = false;


#if defined(__USING_LINUX_USB__)

    struct  termios options;


    m_fd = open(usbPort, O_RDWR | O_NOCTTY | O_NDELAY);
    if( (m_fd == -1) )
    {
        fprintf(stderr, "unable to open %s\n", usbPort);
        retVal  = false;
    } else
    {
        // FNDELAY makes the fd non-blocking, doing a blocking calls
        fcntl(m_fd, F_SETFL, 0);
        retVal  = true;
 
        //
        // and set 9600N81
        //
        tcgetattr(m_fd, &options);
    
        switch(Speed)
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
    
        // 8 data bits
        //options.c_cflag &= ~CSIZE; /* Mask the character size bits */
        //options.c_cflag |= CS8;    /* Select 8 data bits */
    
        // no parity
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    
        // no hardware flow control
        options.c_cflag &= ~CRTSCTS;
    
        // use raw input
        //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_lflag &= ~(ICANON | ISIG);
    
        // no software flow control
        options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    
        // raw output
        options.c_oflag &= ~(OPOST | ONLCR);
    
        // have 5 second timeout
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME]= 50;
    
        //Set the new options for the port...
        tcsetattr(m_fd, TCSANOW, &options);
    }

#endif

#ifdef __USING_WINDOWS_USB__

    // Initializing DCB structure
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    m_fd = CreateFile(usbPort,                //port name
                      GENERIC_READ | GENERIC_WRITE, //Read/Write
                      0,                            // No Sharing
                      NULL,                         // No Security
                      OPEN_EXISTING,// Open existing port only
                      0,            // Non Overlapped I/O
                      NULL);        // Null for Comm Devices
                      
    if (m_fd == INVALID_HANDLE_VALUE)
        fprintf(stderr, "unable to open %s\n", usbPort);
    else
    {
        retVal  = true;
        
        if(true == GetCommState(m_fd, &dcbSerialParams) )
        {
            // set the attributes we so desire ..  mm get some !
            // speed
            switch(Speed)
            {
                case 14400:
                    dcbSerialParams.BaudRate = CBR_14400;
                    break;
        
                case 19200:
                    dcbSerialParams.BaudRate = CBR_19200;
                    break;
        
                case 38400:
                    dcbSerialParams.BaudRate = CBR_38400;
                    break;
        
                case 57600:
                    dcbSerialParams.BaudRate = CBR_57600;
                    break;
        
                default:
                case 9600:
                    dcbSerialParams.BaudRate = CBR_9600;
                    break;
            }
            
            // set 8N1 parity
            dcbSerialParams.ByteSize = 8;         // Setting ByteSize = 8
            dcbSerialParams.StopBits = ONESTOPBIT;// Setting StopBits = 1
            dcbSerialParams.Parity   = NOPARITY;  // Setting Parity = None
            
            // binary mode
            dcbSerialParams.fBinary = true;

            // no software and hardware flow control
            dcbSerialParams.fOutxCtsFlow        = false;
            dcbSerialParams.fOutxDsrFlow        = false;
            dcbSerialParams.fDtrControl         = DTR_CONTROL_DISABLE;
            dcbSerialParams.fDsrSensitivity     = false;
            dcbSerialParams.fTXContinueOnXoff   = false;
            dcbSerialParams.fOutX               = false;
            dcbSerialParams.fInX                = false;
            dcbSerialParams.fErrorChar          = false;
            dcbSerialParams.fErrorChar          = false;
            dcbSerialParams.fRtsControl         = RTS_CONTROL_DISABLE;
            dcbSerialParams.fAbortOnError       = false;
            //XonLim                          = 0; // TODO: what is this
            //XoffLim                         = 0; // TODO: what is this
            //XonChar                       = ?; // not changing
            //XoffChar                      = ?; // not changing
            //ErrorChar                     = ?; // not changing
            //EofChar                       = ?; // not changing
            //EvtChar                       = ?; // not changing

            // set the new serial comm settings
            if( (false == (SetCommState(m_fd, &dcbSerialParams))) )
            {
                fprintf(stderr, "unable to SetCommState\n");
                retVal = false;
            }
        } else
        {
            fprintf(stderr, "unable to GetCommState\n");
            retVal = false;
        }
    }

#endif
    return(retVal);
}


//
// TODO: throw if this fails
//
controlProtocol::controlProtocol(uint16_t myAddress, uint16_t peerAddress, const char* usbPort, uint32_t Speed)
    : m_seqNum(0x0000), m_myAddress(myAddress), m_peerAddress(peerAddress)
{
    //
    // only coding this for the master for now .. 
    //
    TxCommand   = &controlProtocol::TxCommandUSB;
    RxResponse  = &controlProtocol::RxResponseUSB;

    //
    // assuming this always passes TODO: don't assume
    //
    openUSBPort(usbPort, Speed);
};


//
// TODO: throw if this fails
//
controlProtocol::controlProtocol(uint16_t myAddress, uint16_t peerAddress, uint32_t Speed)
    : m_seqNum(0x0000), m_myAddress(myAddress), m_peerAddress(peerAddress)
{
    //
    // for now this will always be the Controllino
    //
    RxCommand   = &controlProtocol::RxCommandSerial;
    TxResponse  = &controlProtocol::TxResponseSerial;

    #ifdef __RUNNING_ON_CONTROLLINO__
    Serial1.begin(Speed, SERIAL_8N1);
    #endif
};


controlProtocol::~controlProtocol()
{
#if defined(__USING_LINUX_USB__)
    close(m_fd);
#endif

#ifdef __USING_WINDOWS_USB__
    CloseHandle(m_fd);
#endif
};


bool controlProtocol::TxCommandUSB(uint16_t length)
{
    #ifdef __DEBUG_CTRL_PROTO__
    printf(__PRETTY_FUNCTION__);
    printf("\n");
    printf("writing %u bytes: ", length);
    for(int i = 0; i < length; i++)
    {
        printf("0x%02X ", m_buff[i]);
    }
    printf("\n");
    #endif
    
    #if defined(__USING_LINUX_USB__)
    int n = write(m_fd, m_buff, length);

    if( (n < 0) )
    {
        fprintf(stderr, "%s write() failed\n", __PRETTY_FUNCTION__);
    }
    #endif

    
    #ifdef __USING_WINDOWS_USB__
    DWORD dNoOfBytesWritten = 0;     // No of bytes written to the port
    
    if( false == WriteFile(m_fd, m_buff, length, &dNoOfBytesWritten, NULL) )
    {
        fprintf(stderr, "%s WriteFile() failed\n", __PRETTY_FUNCTION__);
    }
    #endif

    return(true);
}


bool controlProtocol::RxResponseUSB(uint16_t timeout)
{
#if defined(__USING_LINUX_USB__)
    uint32_t        nbytes  = 0;
    uint32_t        length;
    uint8_t*        bufptr;
    msgHeader_t*    pmsgHeader;
    struct  termios options;


    //
    // set the passed in timeout - VTIME wants 10ths of a second
    // TODO: assuming the caller will provide milliseconds !
    //
    tcgetattr(m_fd, &options);          // get the attribures (all)
    options.c_cc[VMIN] = 0;             // update the VMIN and VTIME
    options.c_cc[VTIME]= timeout / 100; 
    tcsetattr(m_fd, TCSANOW, &options); // set the attributes

    //
    // clear da buffa' bra'
    //
    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);

    //
    // read the message header
    //
    bufptr = m_buff;
    while ((nbytes = read(m_fd, bufptr, m_buff + sizeof(msgHeader_t) - bufptr)) > 0)
    {
        bufptr += nbytes;
    }

    //
    // read the rest of the message using length
    //
    pmsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
    length  = pmsgHeader->length;
    while( (nbytes = read(m_fd, bufptr, length - (bufptr - m_buff))) > 0)
    {
        bufptr += nbytes;
    }

#endif

#ifdef __USING_WINDOWS_USB__
    DWORD           nbytes  = 0;
    DWORD           length;
    uint8_t*        bufptr;
    msgHeader_t*    pmsgHeader;
    COMMTIMEOUTS    timeouts = { 0 };


    timeouts.ReadIntervalTimeout         = timeout; // in milliseconds
    timeouts.ReadTotalTimeoutConstant    = timeout; // in milliseconds
    timeouts.ReadTotalTimeoutMultiplier  = timeout; // in milliseconds
    timeouts.WriteTotalTimeoutConstant   = timeout; // in milliseconds
    timeouts.WriteTotalTimeoutMultiplier = timeout; // in milliseconds

    SetCommTimeouts(m_fd, &timeouts);  // assuming this works 'everytime' !

    //
    // clear da buffa' bra'
    //
    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);

    //
    // read the message header
    //
    bufptr = m_buff;
    while( (ReadFile(m_fd, static_cast<void*>(bufptr),
        m_buff + sizeof(msgHeader_t) - bufptr, &nbytes, NULL)) )
    {
        bufptr += nbytes;
        nbytes  = 0;
        if( 0 == (m_buff + sizeof(msgHeader_t) - bufptr)) break;
    }

    //
    // read the rest of the message using length
    //
    pmsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
    length  = pmsgHeader->length;
    nbytes  = 0;
    while( (ReadFile(m_fd, bufptr, length - (bufptr - m_buff), &nbytes, NULL)) )
    {
        bufptr += nbytes;
        nbytes  = 0;
        if( (0 == (length - (bufptr - m_buff)))) break;
    }

#endif

    return(true);
}


bool controlProtocol::RxCommandSerial(uint16_t TimeoutMs)
{
    bool retVal             = false;
#ifdef __RUNNING_ON_CONTROLLINO__
    bool done               = false;
    bool gotSTX             = false;     // TODO: for now dont' find a defined start char
    bool gotLength          = false;
    bool timedOut           = false;
    int32_t bytes_read      = 0;
    int32_t length          = MAX_BUFF_LENGTH_CP;   // adjusts when pkt length is read
    const uint8_t STX       = COMMAND;
    unsigned long startTime = millis();


    memset(reinterpret_cast<void*>(m_buff), '\0', MAX_BUFF_LENGTH_CP + 1);

    // try to read a packet for a total of TimeoutMs milliseconds
    while( (!done) && (!timedOut) &&
            ((bytes_read < (length)) && (bytes_read < MAX_BUFF_LENGTH_CP)) )
    {
        if( ((millis() - startTime) > TimeoutMs) )
        {
            timedOut = true;
        } else
        {
            if( (Serial1.available()) )
            {
                m_buff[bytes_read] = Serial1.read();

                //
                // look for start of frame
                //
                if( (!gotSTX) )
                {
                    if( (STX == m_buff[bytes_read]) )
                    {
                        // TODO: restart startTime here, give more time to get the packet?
                        gotSTX = true;
                        bytes_read += 1;
                    } // else don't increment bytes_read effectively discarding this byte

                    continue;
                }

                //
                // length is byte 2 (1 for zero based count)
                //
                if( (!gotLength) )
                {
                    gotLength  = true;
                    length = m_buff[bytes_read++];
                    #ifdef __DEBUG_CONTROL_PKT_RX__
                    Serial.print("RxCommandSerial found length: ");
                    Serial.print(length, HEX);
                    Serial.println("");
                    Serial.flush();
                    #endif
                    continue;
                }


                //
                // read the rest of the packet
                //
                bytes_read += 1;

            } else
            {
                // TODO: too long, too short ?
                // no data available, wait a bit before checking again
                //Serial.println("Serial1 no bytes available");
                delay(100);
            }
        }
    }

    // always null terminate just in case we want to dump out for debug
    m_buff[bytes_read] = 0;


    // debug stuff
    #ifdef __DEBUG_CONTROL_PKT_RX__
    Serial.print(__PRETTY_FUNCTION__);
    Serial.flush();
    Serial.print(" received ");
    Serial.flush();
    Serial.print(bytes_read, DEC);
    Serial.flush();
    Serial.println(" bytes");
    Serial.flush();
    for(int i = 0; i < bytes_read; i++)
    {
        Serial.print(m_buff[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
    Serial.flush();
    #endif

    if( (length == (bytes_read)) )
        retVal = true;

    #ifdef __DEBUG_CONTROL_ERROR__
    if( !(retVal) )
    {
        Serial.println("RxCommand found bad formatted packet");
        Serial.flush();
    }
    #endif

#endif
    return(retVal);
}


bool controlProtocol::TxResponseSerial(uint16_t length)
{
    bool    retVal  = true;
#ifdef __RUNNING_ON_CONTROLLINO__
    uint8_t lenWritten;

    // class member Buff is filled in by the member functions
    lenWritten = Serial1.write(m_buff, length);
    Serial1.flush();

    if( (lenWritten != length) )
    {
        #ifdef __DEBUG_HUBER_ERROR__
        Serial.flush();
        Serial.println("TxCommand failed");
        #endif
        retVal  = false;
    #ifdef __DEBUG_CONTROL_PKT_TX__
    } else
    {
        Serial.flush();
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" sent: ");
        for(int i = 0; i < length; i++)
        {
            Serial.print(reinterpret_cast<uint8_t>(m_buff[i]), HEX);
            Serial.print(" ");
        }
        Serial.println("");
        Serial.flush();
    #endif
    }

#endif

    return(retVal);
}


bool controlProtocol::GetStatus(uint16_t destAddress, uint16_t* humidityAlert,
                            uint16_t* ACUsRunning,  uint16_t* chillerOnLine)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getStatusResp_t*    pgetStatusResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getStatus(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getStatusResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getStatusResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetStatusResp = reinterpret_cast<getStatusResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getStatusResp_t, ntohs(pgetStatusResp->crc),
                                            seqNum, ntohs(pgetStatusResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getStatusResp(m_buff, humidityAlert, ACUsRunning, chillerOnLine, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet humidityAlert %u ACUsRunning %u, chillerOnLine %u, seqNumer 0x%02x\n",
                *humidityAlert, *ACUsRunning, *chillerOnLine, seqNum);
            #endif

            retVal  = true;
        } else
        {
            fprintf(stderr, "ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        fprintf(stderr, "ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::SetACUTemperature(uint16_t destAddress, uint16_t acu_address, float temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    setACUTemperatureResp_t*    psetACUTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_setACUTemperature(destAddress, m_buff, acu_address, temperature))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(setACUTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (setACUTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            psetACUTemperatureResp = reinterpret_cast<setACUTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_setACUTemperatureResp_t, ntohs(psetACUTemperatureResp->crc),
                                        seqNum, ntohs(psetACUTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_setACUTemperatureResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "ERROR: did not get a m_buffer back\n");
        }

    } else
    {
        fprintf(stderr, "ERROR: unable to Make_getStatus\n");
    }

    return(retVal);
}


bool controlProtocol::GetACUTemperature(uint16_t destAddress, uint16_t acu_address, uint16_t* result, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getACUTemperatureResp_t*    pgetACUTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getACUTemperature(destAddress, m_buff, acu_address))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getACUTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getACUTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetACUTemperatureResp = reinterpret_cast<getACUTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getACUTemperatureResp_t, ntohs(pgetACUTemperatureResp->crc),
                                            seqNum, ntohs(pgetACUTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getACUTemperatureResp(m_buff, result, temperature, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet temperature %lf seqNumer 0x%02x\n", *temperature, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}

bool controlProtocol::GetACUObjTemperature(uint16_t destAddress, uint16_t acu_address, uint16_t* result, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getACUObjTemperatureResp_t*    pgetACUObjTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getACUObjTemperature(destAddress, m_buff, acu_address))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getACUObjTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getACUObjTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetACUObjTemperatureResp = reinterpret_cast<getACUObjTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getACUObjTemperatureResp_t, ntohs(pgetACUObjTemperatureResp->crc),
                                            seqNum, ntohs(pgetACUObjTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getACUObjTemperatureResp(m_buff, result, temperature, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet temperature %lf seqNumer 0x%02x\n", *temperature, seqNum);
            #endif 

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}

#if defined __USING_CHILLER__
bool controlProtocol::StartChiller(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    startChillerMsgResp_t* pstartChillerMsgResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_startChillerMsg(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(startChillerMsgResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (startChillerMsgResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pstartChillerMsgResp = reinterpret_cast<startChillerMsgResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_startChillerMsgResp_t, ntohs(pstartChillerMsgResp->crc),
                                            seqNum, ntohs(pstartChillerMsgResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_startChillerMsgResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::StopChiller(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    stopChillerResp_t*  pstopChillerResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_stopChiller(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(stopChillerResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (stopChillerResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pstopChillerResp = reinterpret_cast<stopChillerResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_stopChillerResp_t, ntohs(pstopChillerResp->crc),
                                            seqNum, ntohs(pstopChillerResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_stopChillerResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::SetChillerTemperature(uint16_t destAddress, float temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    setChillerTemperatureResp_t*    psetChillerTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_setChillerTemperature(destAddress, m_buff, temperature))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(setChillerTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (setChillerTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            psetChillerTemperatureResp = reinterpret_cast<setChillerTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_setChillerTemperatureResp_t, ntohs(psetChillerTemperatureResp->crc),
                                            seqNum, ntohs(psetChillerTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_setChillerTemperatureResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetChillerTemperature(uint16_t destAddress, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getChillerTemperatureResp_t*    pgetChillerTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getChillerTemperature(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getChillerTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getChillerTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetChillerTemperatureResp = reinterpret_cast<getChillerTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getChillerTemperatureResp_t, ntohs(pgetChillerTemperatureResp->crc),
                                            seqNum, ntohs(pgetChillerTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getChillerTemperatureResp(m_buff, temperature, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet temperature %lf seqNumer 0x%02x\n", *temperature, seqNum);
            #endif

            retVal  = true;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetChillerObjTemperature(uint16_t destAddress, float* temperature)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getChillerObjTemperatureResp_t*    pgetChillerObjTemperatureResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getChillerObjTemperature(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getChillerObjTemperatureResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getChillerObjTemperatureResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetChillerObjTemperatureResp = reinterpret_cast<getChillerObjTemperatureResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getChillerObjTemperatureResp_t, ntohs(pgetChillerObjTemperatureResp->crc),
                                            seqNum, ntohs(pgetChillerObjTemperatureResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getChillerObjTemperatureResp(m_buff, temperature, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet temperature %lf seqNumer 0x%02x\n", *temperature, seqNum);
            #endif

            retVal  = true;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetChillerInfo(uint16_t destAddress, char* info, uint8_t length)
{
    bool                retVal  = false;
    uint16_t            result;
    uint16_t            seqNum;
    msgHeader_t*        pMsgHeader;
    getChillerInfoResp_t*    pgetChillerInfoResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getChillerInfo(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getChillerInfoResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getChillerInfoResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetChillerInfoResp = reinterpret_cast<getChillerInfoResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getChillerInfoResp_t, ntohs(pgetChillerInfoResp->crc),
                                            seqNum, ntohs(pgetChillerInfoResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getChillerInfoResp(m_buff, &result, info, length, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet info \'%s\' seqNumer 0x%02x\n", reinterpret_cast<char*>(info), seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}
#endif


bool controlProtocol::EnableACUs(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    enableACUsResp_t*    penableACUsResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_enableACUs(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(enableACUsResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (enableACUsResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            penableACUsResp = reinterpret_cast<enableACUsResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_enableACUsResp_t, ntohs(penableACUsResp->crc),
                                        seqNum, ntohs(penableACUsResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_enableACUsResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::DisableACUs(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    disableACUsResp_t*  pdisableACUsResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_disableACUs(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(disableACUsResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (disableACUsResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pdisableACUsResp = reinterpret_cast<disableACUsResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_disableACUsResp_t, ntohs(pdisableACUsResp->crc),
                                        seqNum, ntohs(pdisableACUsResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_disableACUsResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::GetACUInfo(uint16_t destAddress, uint16_t acu_address,
                    uint32_t* deviceType, uint32_t* hwVersion, uint32_t* fwVersion, uint32_t* serialNum)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    getACUInfoMsgResp_t*    pgetACUInfoMsgResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_getACUInfoMsg(destAddress, m_buff, acu_address))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(getACUInfoMsgResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (getACUInfoMsgResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pgetACUInfoMsgResp = reinterpret_cast<getACUInfoMsgResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_getACUInfoMsgResp_t, ntohs(pgetACUInfoMsgResp->crc),
                                            seqNum, ntohs(pgetACUInfoMsgResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_getACUInfoMsgResp(m_buff, &result, deviceType, hwVersion, fwVersion, serialNum, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d, deviceType 0x%02X, hwVersion 0x%02X, fwVersion 0x%02X, \
                serialNum 0x%02X seqNumer 0x%02x\n", result, *deviceType, *hwVersion, *fwVersion, *serialNum, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;

        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::StartUpCmd(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    startUpCmdResp_t*   pstartUpCmdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_startUpCmd(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(startUpCmdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (startUpCmdResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pstartUpCmdResp = reinterpret_cast<startUpCmdResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_startUpCmdResp_t, ntohs(pstartUpCmdResp->crc),
                                        seqNum, ntohs(pstartUpCmdResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_startUpCmdResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


bool controlProtocol::ShutDownCmd(uint16_t destAddress)
{
    bool                retVal  = false;
    uint16_t            seqNum;
    uint16_t            result;
    msgHeader_t*        pMsgHeader;
    shutDownCmdResp_t*  pshutDownCmdResp;


    //
    // increment the sequence number for this transaction
    //
    ++m_seqNum;

    if( (doTxCommand(Make_shutDownCmd(destAddress, m_buff))) )
    {
        //
        // save the seqNum
        //
        pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
        seqNum  = pMsgHeader->seqNum;

        // get the return packet
        if( (doRxResponse(COMM_TIMEOUT)) )
        {
            #ifdef __DEBUG_CTRL_PROTO__
            //
            // dump out what we got
            //
            for(uint16_t i = 0; i < sizeof(shutDownCmdResp_t); i++)
            {
                printf("0x%02X ", m_buff[i]);
            }
            printf("\n");
            #endif

            //
            // check got the expected message number
            //
            pMsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);
            if( (shutDownCmdResp != pMsgHeader->msgNum) )
            {
                fprintf(stderr, "ERROR: %s got unexpected msg %hu\n",
                    __PRETTY_FUNCTION__, pMsgHeader->msgNum);

                //
                // no need to continue processing
                //
                return(false);
            }

            //
            // cast into the buffer, pick up the CRC
            //
            pshutDownCmdResp = reinterpret_cast<shutDownCmdResp_t*>(m_buff);

            //
            // verify seqNum and CRC
            //
            if( !(verifyMessage(len_shutDownCmdResp_t, ntohs(pshutDownCmdResp->crc),
                                    seqNum, ntohs(pshutDownCmdResp->eop))) )
            {
                // TODO: drop the packet
                fprintf(stderr, "ERROR: %s CRC bad, seqNum mismatch, or wrong address\n",
                        __PRETTY_FUNCTION__);

                //
                // no need to continue processing
                //
                return(false);
            }


            //
            // report the health
            //
            Parse_shutDownCmdResp(m_buff, &result, &seqNum);

            #ifdef __DEBUG_CTRL_PROTO__
            printf("found in packet result %d seqNumer 0x%02x\n", result, seqNum);
            #endif

            if( (result) )
                retVal  = true;
            else
                retVal  = false;
        } else
        {
            fprintf(stderr, "%s ERROR: did not get a m_buffer back\n", __PRETTY_FUNCTION__);
        }

    } else
    {
        fprintf(stderr, "%s ERROR: unable to Make_getStatus\n", __PRETTY_FUNCTION__);
    }

    return(retVal);
}


uint16_t controlProtocol::Make_startUpCmd(uint16_t Address, uint8_t* pBuff)
{
    startUpCmd_t*    msg  = reinterpret_cast<startUpCmd_t*>(pBuff);
    uint16_t      CRC  = 0;


    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);
    // create the startUpCmd message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(startUpCmd_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = startUpCmd;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_startUpCmd_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(startUpCmd_t));
}


uint16_t controlProtocol::Make_startUpCmdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    startUpCmdResp_t*  msg  = reinterpret_cast<startUpCmdResp_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the startUpCmdResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(startUpCmdResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = startUpCmdResp;

    // set the result
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_startUpCmdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(startUpCmdResp_t));
}


void controlProtocol::Parse_startUpCmdResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    startUpCmdResp_t* pResponse = reinterpret_cast<startUpCmdResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_shutDownCmd(uint16_t Address, uint8_t* pBuff)
{
    shutDownCmd_t* msg  = reinterpret_cast<shutDownCmd_t*>(pBuff);
    uint16_t    CRC  = 0;


    // create the shutDownCmdResp message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(shutDownCmd_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = shutDownCmd;

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_shutDownCmd_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(shutDownCmd_t));
}


uint16_t controlProtocol::Make_shutDownCmdResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    shutDownCmdResp_t* msg  = reinterpret_cast<shutDownCmdResp_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the shutDownCmdResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(shutDownCmdResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = shutDownCmdResp;

    // set the result
    msg->result             = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_shutDownCmdResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(shutDownCmdResp_t));
}


void controlProtocol::Parse_shutDownCmdResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    shutDownCmdResp_t* pResponse = reinterpret_cast<shutDownCmdResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


//
// send a getStatus message to controllino at Address - future expandability
// TODO: protect against short pBuff by having caller include length ?
//
uint16_t controlProtocol::Make_getStatus(uint16_t Address, uint8_t* pBuff)
{
    getStatus_t*    msg  = reinterpret_cast<getStatus_t*>(pBuff);
    uint16_t        CRC  = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getStatus_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getStatusCmd;

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_getStatus_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getStatus_t));
}


uint16_t controlProtocol::Make_getStatusResp(uint16_t Address, uint8_t* pBuff, uint16_t humidityAlert,
                    uint16_t ACUsRunning, uint16_t chillerOnLine, uint16_t SeqNum)
{
    getStatusResp_t*    msg  = reinterpret_cast<getStatusResp_t*>(pBuff);
    uint16_t            CRC  = 0;


    // create the getStatusResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getStatusResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getStatusResp;
    msg->status.humidityAlert   = htons(humidityAlert);
    msg->status.ACUsRunning     = htons(ACUsRunning);
    msg->status.chillerOnLine   = htons(chillerOnLine);

    // calculate the CRC
    CRC = calcCRC16(pBuff, len_getStatusResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getStatusResp_t));
}


void controlProtocol::Parse_getStatusResp(uint8_t* m_buff, uint16_t* humidityAlert,
        uint16_t* ACUsRunning, uint16_t* chillerOnLine, uint16_t* pSeqNum)
{
    getStatusResp_t* pResponse = reinterpret_cast<getStatusResp_t*>(m_buff);

    //
    // fill in the pReport
    //
    *humidityAlert  = ntohs(pResponse->status.humidityAlert);
    *ACUsRunning    = ntohs(pResponse->status.ACUsRunning);
    *chillerOnLine  = ntohs(pResponse->status.chillerOnLine);
    *pSeqNum        = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_setACUTemperature(uint16_t Address, uint8_t* pBuff, uint16_t acu_address, float temperature)
{
    setACUTemperature_t* msg = reinterpret_cast<setACUTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(setACUTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = setACUTemperature;
    msg->acu_address            = htons(acu_address);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_ACU_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_ACU_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setACUTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setACUTemperature_t));
}


uint16_t controlProtocol::Make_setACUTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t acu_address, uint16_t result, uint16_t SeqNum)
{
    setACUTemperatureResp_t* msg = reinterpret_cast<setACUTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(setACUTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = setACUTemperatureResp;
    msg->acu_address            = htons(acu_address);
    msg->result                 = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setACUTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setACUTemperatureResp_t));
}


void controlProtocol::Parse_setACUTemperatureResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    setACUTemperatureResp_t* pResponse = reinterpret_cast<setACUTemperatureResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getACUTemperature(uint16_t Address, uint8_t* pBuff, uint16_t acu_address)
{
    getACUTemperature_t* msg = reinterpret_cast<getACUTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getACUTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getACUTemperature;
    msg->acu_address            = htons(acu_address);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getACUTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getACUTemperature_t));
}


uint16_t controlProtocol::Make_getACUTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t acu_address, uint16_t result, float temperature, uint16_t SeqNum)
{
    getACUTemperatureResp_t* msg = reinterpret_cast<getACUTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getACUTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getACUTemperatureResp;
    msg->acu_address            = htons(acu_address);
    msg->result                 = htons(result);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_ACU_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_ACU_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getACUTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getACUTemperatureResp_t));
}

void controlProtocol::Parse_getACUTemperatureResp(uint8_t* m_buff, uint16_t* result, float* temperature, uint16_t* pSeqNum)
{
    getACUTemperatureResp_t* pResponse = reinterpret_cast<getACUTemperatureResp_t*>(m_buff);


    *result = ntohs(pResponse->result);

    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getACUObjTemperature(uint16_t Address, uint8_t* pBuff, uint16_t acu_address)
{
    getACUObjTemperature_t* msg = reinterpret_cast<getACUObjTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getACUObjTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getACUObjTemperature;
    msg->acu_address            = htons(acu_address);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getACUObjTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getACUObjTemperature_t));
}


uint16_t controlProtocol::Make_getACUObjTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t acu_address, uint16_t result, float temperature, uint16_t SeqNum)
{
    getACUObjTemperatureResp_t* msg = reinterpret_cast<getACUObjTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getACUObjTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getACUObjTemperatureResp;
    msg->acu_address            = htons(acu_address);
    msg->result                 = htons(result);
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_ACU_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_ACU_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getACUObjTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getACUObjTemperatureResp_t));
}

void controlProtocol::Parse_getACUObjTemperatureResp(uint8_t* m_buff, uint16_t* result, float* temperature, uint16_t* pSeqNum)
{
    getACUObjTemperatureResp_t* pResponse = reinterpret_cast<getACUObjTemperatureResp_t*>(m_buff);


    *result = ntohs(pResponse->result);

    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getACUInfoMsg(uint16_t Address, uint8_t* pBuff, uint16_t acu_address)
{
    getACUInfoMsg_t* msg = reinterpret_cast<getACUInfoMsg_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getACUInfoMsg_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getACUInfoMsg;
    msg->acu_address            = htons(acu_address);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getACUInfoMsg_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getACUInfoMsg_t));
}


uint16_t controlProtocol::Make_getACUInfoMsgResp(uint16_t Address, uint8_t* pBuff, uint16_t acu_address,
        uint16_t result, uint32_t deviceType, uint32_t hwVersion, uint32_t fwVersion,
        uint32_t serialNumber, uint16_t SeqNum)
{
    getACUInfoMsgResp_t* msg = reinterpret_cast<getACUInfoMsgResp_t*>(pBuff);
    uint16_t CRC = 0;

    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getACUInfoMsgResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getACUInfoMsgResp;
    msg->result                 = htons(result);
    msg->acu_address            = htons(acu_address);
    msg->deviceType             = htonl(deviceType);
    msg->hwVersion              = htonl(hwVersion);
    msg->fwVersion              = htonl(fwVersion);
    msg->serialNumber           = htonl(serialNumber);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getACUInfoMsgResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getACUInfoMsgResp_t));
}


void controlProtocol::Parse_getACUInfoMsgResp(uint8_t* m_buff, uint16_t* result, uint32_t* deviceType,
        uint32_t* hwVersion, uint32_t* fwVersion, uint32_t* serialNumber, uint16_t* pSeqNum)
{
    getACUInfoMsgResp_t* pResponse = reinterpret_cast<getACUInfoMsgResp_t*>(m_buff);


    *result         = ntohs(pResponse->result);
    *deviceType     = pResponse->deviceType;
    *hwVersion      = pResponse->hwVersion;
    *fwVersion      = pResponse->fwVersion;
    *serialNumber   = pResponse->serialNumber;
    *pSeqNum        = pResponse->header.seqNum;
}



uint16_t controlProtocol::Make_enableACUs(uint16_t Address, uint8_t* pBuff)
{
    enableACUs_t* msg = reinterpret_cast<enableACUs_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(enableACUs_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = enableACUs;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_enableACUs_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(enableACUs_t));
}


uint16_t controlProtocol::Make_enableACUsResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    enableACUsResp_t* msg = reinterpret_cast<enableACUsResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(enableACUsResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = enableACUsResp;
    msg->result                 = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_enableACUsResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(enableACUsResp_t));
}


void controlProtocol::Parse_enableACUsResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    enableACUsResp_t* pResponse = reinterpret_cast<enableACUsResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_disableACUs(uint16_t Address, uint8_t* pBuff)
{
    disableACUs_t* msg = reinterpret_cast<disableACUs_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(disableACUs_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = disableACUs;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_disableACUs_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(disableACUs_t));
}


uint16_t controlProtocol::Make_disableACUsResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    disableACUsResp_t* msg = reinterpret_cast<disableACUsResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(disableACUsResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = disableACUsResp;
    msg->result                 = htons(result);

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_disableACUsResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(disableACUsResp_t));
}


void controlProtocol::Parse_disableACUsResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    disableACUsResp_t* pResponse = reinterpret_cast<disableACUsResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}

#if defined __USING_CHILLER__
uint16_t controlProtocol::Make_startChillerMsg(uint16_t Address, uint8_t* pBuff)
{
    startChillerMsg_t* msg  = reinterpret_cast<startChillerMsg_t*>(pBuff);
    uint16_t      CRC  = 0;


    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);
    // create the startChillerMsg message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(startChillerMsg_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = startChillerMsg;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_startChillerMsg_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(startChillerMsg_t));
}


uint16_t controlProtocol::Make_startChillerMsgResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    startChillerMsgResp_t* msg  = reinterpret_cast<startChillerMsgResp_t*>(pBuff);
    uint16_t      CRC  = 0;


    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);
    // create the startChillerMsgResp message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(startChillerMsgResp_t);
    msg->header.address.address = htons(Address);
    msg->result                 = htons(result);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = startChillerMsgResp;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_startChillerMsgResp_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(startChillerMsgResp_t));
}


void controlProtocol::Parse_startChillerMsgResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    startChillerMsgResp_t* pResponse = reinterpret_cast<startChillerMsgResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_stopChiller(uint16_t Address, uint8_t* pBuff)
{
    stopChiller_t* msg  = reinterpret_cast<stopChiller_t*>(pBuff);
    uint16_t      CRC  = 0;


    memset(m_buff, '\0', MAX_BUFF_LENGTH_CP + 1);
    // create the stopChiller message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(stopChiller_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = stopChiller;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_stopChiller_t);

    // put the CRC
    msg->crc                = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(stopChiller_t));
}


uint16_t controlProtocol::Make_stopChillerResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    stopChillerResp_t* msg = reinterpret_cast<stopChillerResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(stopChillerResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = stopChillerResp;
    msg->result                 = htons(result);
    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_stopChillerResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(stopChillerResp_t));
}


void controlProtocol::Parse_stopChillerResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    stopChillerResp_t* pResponse = reinterpret_cast<stopChillerResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_setChillerTemperature(uint16_t Address, uint8_t* pBuff, float temperature)
{
    setChillerTemperature_t* msg = reinterpret_cast<setChillerTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(setChillerTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = setChillerTemperature;
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_CHILLER_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setChillerTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setChillerTemperature_t));
}


uint16_t controlProtocol::Make_setChillerTemperatureResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint16_t SeqNum)
{
    setChillerTemperatureResp_t* msg = reinterpret_cast<setChillerTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(setChillerTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = setChillerTemperatureResp;
    msg->result                 = htons(result);
    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_setChillerTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(setChillerTemperatureResp_t));
}


void controlProtocol::Parse_setChillerTemperatureResp(uint8_t* m_buff, uint16_t* result, uint16_t* pSeqNum)
{
    setChillerTemperatureResp_t* pResponse = reinterpret_cast<setChillerTemperatureResp_t*>(m_buff);


    *result     = ntohs(pResponse->result);
    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getChillerTemperature(uint16_t Address, uint8_t* pBuff)
{
    getChillerTemperature_t* msg = reinterpret_cast<getChillerTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getChillerTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getChillerTemperature;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerTemperature_t));
}

uint16_t controlProtocol::Make_getChillerTemperatureResp(uint16_t Address, uint8_t* pBuff, float temperature, uint16_t SeqNum)
{
    getChillerTemperatureResp_t* msg = reinterpret_cast<getChillerTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getChillerTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getChillerTemperatureResp;
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_CHILLER_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerTemperatureResp_t));
}

void controlProtocol::Parse_getChillerTemperatureResp(uint8_t* m_buff, float* temperature, uint16_t* pSeqNum)
{
    getChillerTemperatureResp_t* pResponse = reinterpret_cast<getChillerTemperatureResp_t*>(m_buff);


    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getChillerObjTemperature(uint16_t Address, uint8_t* pBuff)
{
    getChillerObjTemperature_t* msg = reinterpret_cast<getChillerObjTemperature_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getChillerObjTemperature_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getChillerObjTemperature;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerObjTemperature_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerObjTemperature_t));
}

uint16_t controlProtocol::Make_getChillerObjTemperatureResp(uint16_t Address, uint8_t* pBuff, float temperature, uint16_t SeqNum)
{
    getChillerObjTemperatureResp_t* msg = reinterpret_cast<getChillerObjTemperatureResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getChillerObjTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getChillerObjTemperatureResp;
    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // use the dostrf function, left justified, max 7 characters total, max 2 decimal places
    //
    dtostrf(temperature, -(MAX_CHILLER_TEMP_LENGH), 2, reinterpret_cast<char*>(msg->temperature));
    #else
    //
    // use the snprintf function, 
    //
    snprintf(reinterpret_cast<char*>(msg->temperature), MAX_CHILLER_TEMP_LENGH, "%-+3.2f", temperature);
    #endif

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerObjTemperatureResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerObjTemperatureResp_t));
}

void controlProtocol::Parse_getChillerObjTemperatureResp(uint8_t* m_buff, float* temperature, uint16_t* pSeqNum)
{
    getChillerObjTemperatureResp_t* pResponse = reinterpret_cast<getChillerObjTemperatureResp_t*>(m_buff);


    #ifdef __RUNNING_ON_CONTROLLINO__
    //
    // on uC use atof, sscanf support is dodgy
    //
    *temperature = atof(reinterpret_cast<char*>(pResponse->temperature));
    #else
    sscanf(reinterpret_cast<char*>(pResponse->temperature), "%f", temperature);
    #endif

    *pSeqNum    = pResponse->header.seqNum;
}


uint16_t controlProtocol::Make_getChillerInfo(uint16_t Address, uint8_t* pBuff)
{
    getChillerInfo_t* msg = reinterpret_cast<getChillerInfo_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = COMMAND;
    msg->header.length          = sizeof(getChillerInfo_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = m_seqNum;
    msg->header.msgNum          = getChillerInfo;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerInfo_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerInfo_t));
}


uint16_t controlProtocol::Make_getChillerInfoResp(uint16_t Address, uint8_t* pBuff, uint16_t result, uint8_t* info, uint8_t length, uint16_t SeqNum)
{
    getChillerInfoResp_t* msg = reinterpret_cast<getChillerInfoResp_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getChillerInfoResp_t);
    msg->header.address.address = htons(Address);
    msg->result                 = htons(result);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = getChillerInfoResp;

    strncpy(reinterpret_cast<char*>(msg->info), reinterpret_cast<char*>(info),
        length < MAX_CHILLER_INFO_LENGTH ? length : MAX_CHILLER_INFO_LENGTH); 

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_getChillerInfoResp_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?


    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(getChillerInfoResp_t));
}


void controlProtocol::Parse_getChillerInfoResp(uint8_t* m_buff, uint16_t* result, char* info, uint8_t length, uint16_t* pSeqNum)
{
    getChillerInfoResp_t* pResponse = reinterpret_cast<getChillerInfoResp_t*>(m_buff);


    *result = htons(pResponse->result);

    strncpy(info, reinterpret_cast<char*>(pResponse->info),
        (length < MAX_CHILLER_INFO_LENGTH ? length : MAX_CHILLER_INFO_LENGTH)); 

    *pSeqNum    = pResponse->header.seqNum;
}
#endif


uint16_t controlProtocol::Make_NACK(uint16_t Address, uint8_t* pBuff, uint16_t SeqNum)
{
    NACK_t* msg = reinterpret_cast<NACK_t*>(pBuff);
    uint16_t CRC = 0;


    // create the getStatus message in pBuff and CRC16 checksum it
    msg->header.control         = RESPONSE;
    msg->header.length          = sizeof(getChillerTemperatureResp_t);
    msg->header.address.address = htons(Address);
    msg->header.seqNum          = SeqNum;
    msg->header.msgNum          = NACK;

    // calculate the CRC
    CRC = calcCRC16(pBuff,  len_NACK_t);

    // put the CRC
    msg->crc    = htons(CRC);   // TODO: need htons ?

    // put the end of transmission byte
    msg->eop                = htons(EOP_VAL);

    return(sizeof(NACK_t));
}


//
// uses class members:
// 1. m_buff    - has the received message in it
// 2. m_seqNum  - is the current sequence number expected in this reply
//
// the calling function supplies the length of the message in m_buff
//
bool controlProtocol::verifyMessage(uint16_t buffLength, uint16_t pktCRC, uint16_t expSeqNum, EOP eot)
{
    if( (verifyMessageSeqNum(buffLength, expSeqNum))    // seqNum
        && (verifyMessageCRC(buffLength, pktCRC))       // CRC
        && (verifyMessageLength(eot)) )                 // length
    {
        return(true);
    } else
    {
        return(false);
    }
}


bool controlProtocol::verifyMessage(uint16_t buffLength, uint16_t pktCRC, EOP eot)
{
        if( (verifyMessageCRC(buffLength, pktCRC))       // CRC
        && (verifyMessageLength(eot)) )                 // length
    {
        return(true);
    } else
    {
        return(false);
    }
}


bool controlProtocol::verifyMessageSeqNum(uint16_t buffLength, uint16_t expSeqNum)
{
    bool            retVal      = true;
    msgHeader_t*    pMsgHeader  = reinterpret_cast<msgHeader_t*>(m_buff);


    //
    // check seqNum match
    //
    if( (expSeqNum != pMsgHeader->seqNum) )
    {
        fprintf(stderr, "ERROR: %s seqNum mismatch\n", __PRETTY_FUNCTION__);
        retVal  = false;
    }

    return(retVal);
}


bool controlProtocol::verifyMessageCRC(uint16_t buffLength, uint16_t pktCRC)
{
    bool            retVal      = true;
    uint16_t        CRC         = calcCRC16(m_buff, buffLength);


    if( (CRC != pktCRC) ) 
    {
        #ifndef __RUNNING_ON_CONTROLLINO__
        fprintf(stderr, "ERROR: %s CRC mismatch 0x%x:0x%x\n", __PRETTY_FUNCTION__, CRC, pktCRC);
        #else
        Serial.print(__PRETTY_FUNCTION__);
        Serial.print(" ERROR: CRC mismatch: 0x");
        Serial.print(CRC, HEX);
        Serial.print(":");
        Serial.println(pktCRC, HEX);
        Serial.flush();
        #endif
        retVal  = false;
    }

    return(retVal);
}


bool controlProtocol::verifyMessageLength(EOP eot)
{
    //
    // should be an EOP_VAL at the end of the buffer
    //
    if( (EOP_VAL == eot) )
        return(true);
    else
        return(false);
}

uint16_t controlProtocol::getMsgId()
{
    msgHeader_t* pmsgHeader = reinterpret_cast<msgHeader_t*>(m_buff);

    return(pmsgHeader->msgNum);
}
