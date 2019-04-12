#include "deviceio_hid_over_i2c.h"

/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <time.h>

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 *
 * If you need this, please have your distro update the kernel headers.
 */
#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "logmanager.h"
#include "deviceexception.h"
#include "deviceio_hid_over_i2c.h"
#include "packet.h"
#include <list>
#include <errno.h>

#define CMD_F1_RETRY_MAX    5
#define OUTPUT_IGNORE       4


/* cmd info */
// HID_I2C_OUTPUT_REGISTER, HID_I2C_OUTPUT_WO_LEN, report id & seq # & STX, ETX, CMD, Length
// for Read, HID_I2C_OUTPUT_REGISTER was excluded.

#define CMD_HW_RESET                    (0xD8)
#define CMD_MAIN_CMD                    (0x13)
#define CMD_CU_ERASE                    (0xB3)
#define CMD_CU_WRITE                    (0xB4)

#define CMD_FW_VER                      (0xF1)

#define G2_SUB_0x13_GOTOBOOT            0x80
#define G2_SUB_0x13_FWDOWNREADY         0x81
#define G2_SUB_0x13_FWERASE             0x82
#define G2_SUB_0x13_FWWRITE             0x83
#define G2_SUB_0x13_FLASHCHKSUM         0x84
#define G2_SUB_0x13_FLASHFINISH         0x85
#define G2_SUB_0x13_SYSTEMRESET         0x90
#define G2_SUB_0x13_FLASH_ERASE         0x05
#define G2_SUB_0x13_FLASH_WRITE         0x06
#define G2_SUB_0x13_FLASH_BOOT          0x00
#define G2_SUB_0x13_FLASH_CU            0x01
#define G2_SUB_0x13_FLASH_TSMINFO       0x02
#define G2_SUB_0x13_FLASH_DEADCELL      0x03
#define G2_SUB_0x13_FLASH_MAINAPP       0x10
#define G2_SUB_0x13_FLASH_READ          0x08
#define G2_SUB_0x13_FW_DOWNLOAD_READY   0x10
#define G2_SUB_0x13_GOTO_FW_MODE        0x11
#define G2_SUB_0x13_DOWNLOAD_FINISH     0x14

#define HID_I2C_OUTPUT_REGISTER     (0x00E7)

#define TOKEN_STX1 0x02
#define TOKEN_STX2 0xA3

#define TOKEN_ETX1 0x03
#define TOKEN_ETX2 0xb3

#define FW_START_POS 0x8000
#define FW_VID_POS 0x220
#define FW_PID_POS 0x230

#define CU_START_POS 0x4000
#define BOOT_VER_POS 0x400
#define BOOT_FILE_SIZE 0x4000
#define CU_FILE_SIZE 0xb00
#define BOOT_VERSION_SIZE 0x40

#define DUMP_SIZE 0x100

using namespace G2;
using namespace G2::DeviceIO;

#define TAG "deviceio_hid_over_i2c"

DeviceIO_hid_over_i2c::DeviceIO_hid_over_i2c(CArgHandler *argHandler) :
    hid_Type(0),  HID_OUTPUT_MAX_LEN(0), HID_OUT_REPORT_ID(0), m_fd(0),
    out_buffer(0), in_buffer(0), rxdbgbuf(0), tmpRxUnit(0),
    index(0), m_packetlength(0), dbgidx_push(0), dbgidx_pop(0),
    m_bOpened(false), read_pos(0), buf_pos(0), target_type(""),file_type("")
{
    /* malloc out_buffer, in_buffer */
    out_buffer = new unsigned char[HID_OUTPUT_MAX_LEN_USB];
    in_buffer = new unsigned char[HID_INPUT_MAX_LEN];
    rxdbgbuf = new unsigned char[RXDBGBUFSIZE];
    index =0;
    m_packetlength=0;
    tmpRxUnit = new rxUnit();
    dbgidx_push = 0;
    dbgidx_pop = 0;
}

DeviceIO_hid_over_i2c::~DeviceIO_hid_over_i2c()
{
    closeDevice();
    delete [] out_buffer;
    delete [] in_buffer;
    delete [] rxdbgbuf;
    delete tmpRxUnit;
}

void
DeviceIO_hid_over_i2c::initBuffer()
{
    memset(out_buffer, 0x0, sizeof(unsigned char) * HID_OUTPUT_MAX_LEN);
}

#ifdef USE_EXCEPTION
void
DeviceIO_hid_over_i2c::throwNotSupportException(std::string functionName)
{
	std::string msg = "";
	char errorMsg[1024] = "";
	sprintf(errorMsg, "G2DeviceIO_hid_over_i2c, not support : %s", functionName.c_str());
	msg.append(errorMsg);
	throw G2DeviceException(msg, -1);
}
#endif

int
DeviceIO_hid_over_i2c::openDevice(string hidRawName)
{
    char* cptr;
    cptr = new char[hidRawName.length() + 1];
    strcpy(cptr, hidRawName.c_str());
    const char* devName = cptr;

    LOG_G2_D(CLog::getLogOwner(), TAG, "devName=%s", devName);

    if(m_fd >= 0)
    {
        close(m_fd);
        m_fd = -1;

        LOG_G2_I(CLog::getLogOwner(), TAG, "opendevice-close\n" );
    }

    if(isUsingIOCTL())
    {
        LOG_G2_I(CLog::getLogOwner(), TAG, "isUsingIOCTL : true");
        m_fd = open(devName, O_RDWR | O_NONBLOCK); // ioctl : O_NONBLOCK flag has no effect
    }
    else
    {
        LOG_G2_I(CLog::getLogOwner(), TAG, "isUsingIOCTL : false");
        m_fd = open(devName, O_RDWR | O_NONBLOCK);
    }

    delete [] cptr;

    /* handle error */
    if (m_fd < 0)
    {
        std::string msg = "";
        char errorMsg[1024] = "";
        sprintf(errorMsg, "openDevice, unable to open device, errno=%d (%s)", errno, strerror(errno));
        msg.append(errorMsg);

#ifdef USE_EXCEPTION
        throw G2DeviceException(msg, -1);
#else
        LOG_G2_E(CLog::getLogOwner(), TAG, "%s", msg.c_str());
#endif

        return m_fd;
    }
    else
    {
    	m_bOpened = true;
    }

    return m_fd;
}

int
DeviceIO_hid_over_i2c::closeDevice()
{
    LOG_G2_I(CLog::getLogOwner(), TAG, "closeDevice, m_fd : %d",  m_fd);

    if( m_fd < 0 )
    {
        return 0;
    }

    int ret = 0;
    if(m_fd >= 0)
    {
        ret = close(m_fd);
        m_fd = -1;
    }

    if (ret < 0)
    {
        std::string msg = "";
        char errorMsg[1024] = "";
        sprintf(errorMsg, "closeDevice, unable to close device, errno=%d (%s)", errno, strerror(errno));
        msg.append(errorMsg);
#ifdef USE_EXCEPTION
        throw G2DeviceException(msg, -1);
#else
        LOG_G2_E(CLog::getLogOwner(), TAG, "%s", msg.c_str());
#endif

        return m_fd;
    }

    return ret;
}

int
DeviceIO_hid_over_i2c::writeData(unsigned char * buf, int size, int timeoutMsec = 0)
{
    int ret = 0;

    if(isUsingIOCTL())
    {
        ret = ioctl(m_fd, HIDIOCSFEATURE(size), buf);
    }
    else
    {
        ret = write(m_fd, buf, size);
        //ret = write(m_fd, buf + OUTPUT_IGNORE, size - OUTPUT_IGNORE);
    }

    return ret;
}

int
DeviceIO_hid_over_i2c::ReadDataPush( unsigned char * buf, int size)
{
    int i, head;
    int ret = 0;

    head = dbgidx_push;

    // SKIP 2bytes ... Report ID[1byte], Index[1byte]
    for(i = 2; i < size; i++)
    {
        rxdbgbuf[head&(RXDBGBUFSIZE-1)] = buf[i];
        ++head;
        head = head & (RXDBGBUFSIZE-1);
    }
    dbgidx_push = head;
    return ret;
}

int
DeviceIO_hid_over_i2c::ParsePushedData( unsigned char * buf, int size, int Maincommand, int Subcommand, int chk_ack = 0)
{
    unsigned int i=0, len=0, head=0, tail=0, cnt=0;
    int ret = 0;
    unsigned char tmpbuf[RXDBGBUFSIZE];

    head = dbgidx_push;
    tail = dbgidx_pop;

    if(head == tail)
    {
        ret = 0;
        goto end;
    }
    else if(head > tail)
    {
		memcpy(tmpbuf, &rxdbgbuf[tail], head-tail);
		cnt = head - tail;
    }
    else
    {
		memcpy(tmpbuf, &rxdbgbuf[tail], RXDBGBUFSIZE-tail);
		memcpy(&tmpbuf[RXDBGBUFSIZE-tail],rxdbgbuf,head);
		cnt = (RXDBGBUFSIZE-tail) + head;
    }

    if(cnt < 7)
    {
        ret = 0;
        goto end;
    }

    len = 0;
    for(i=0; i<cnt; i++)
    {
        if((tmpbuf[i] == 0x02) && (tmpbuf[i+1] == 0xA3 ))
        {
            len = (tmpbuf[i+3]<<8) + tmpbuf[i+4];

            if(((cnt-i) < 7) || ((cnt-i-7) < len)){ 
                ret = 0;
                goto end;
            }

            if((tmpbuf[i+len+5] != 0x03) || (tmpbuf[i+len+6] != 0xB3)) { 
                LOG_G2_D(CLog::getLogOwner(), TAG, "Packet Corrupted, i=%d, len=%d, etx=[%02X, %02X]", i, len, tmpbuf[i+len+5], tmpbuf[i+len+6]);
                i += 7;
            }
            else
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "MainCmd=%02X/SubCmd=%02X - idx[Push=%d/Pop=%d]", Maincommand, Subcommand, dbgidx_push, dbgidx_pop);
                LOG_G2_D(CLog::getLogOwner(), TAG, "Packet Matched. cmd[%02X]/len[%02X]/body[%02X/%02X/%02X/%02X/%02X]",
                    tmpbuf[i+2], len, tmpbuf[i+5], tmpbuf[i+6], tmpbuf[i+7], tmpbuf[i+8], tmpbuf[i+9]);

                ret = len+1;
                if(chk_ack && Maincommand == tmpbuf[i+2])
                {
                    LOG_G2_D(CLog::getLogOwner(), TAG, "ACK.. MainCmd=%02X/SubCmd=%02X - idx[Push=%d/Pop=%d]", Maincommand, Subcommand, dbgidx_push, dbgidx_pop);
                    LOG_G2_D(CLog::getLogOwner(), TAG, "ACK.. Packet Matched. cmd[%02X]/len[%02X]/body[%02X/%02X/%02X/%02X/%02X]",
                        tmpbuf[i+2], len, tmpbuf[i+5], tmpbuf[i+6], tmpbuf[i+7], tmpbuf[i+8], tmpbuf[i+9]);
                    LOG_G2_D(CLog::getLogOwner(), TAG, "buf-body-2[%02X/%02X/%02X/%02X/%02X/%02X/%02X/%02X]",
                        buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
                }
                break;
            }
        }
    }

    if(ret)
    {
        dbgidx_pop = (tail + i + len+7) & (RXDBGBUFSIZE-1);
        if(chk_ack && Maincommand != tmpbuf[i+2]) ret = 0;
    }
    else if(!len)
        dbgidx_pop = (tail + i) & (RXDBGBUFSIZE-1);

end:
	return ret;
}

int
DeviceIO_hid_over_i2c::readData( unsigned char * buf, int size, int Maincommand, int Subcommand)
{
    int ret = -2;
    int stx_etx_ret=0;

    ret = read(m_fd, buf, size);

    if( ret > 0 )
    {
        if( (buf[2] == TOKEN_STX1) && (buf[3] == TOKEN_STX2)  && (buf_pos == 0) )
        {
            m_packetlength = (((unsigned int)(buf[5] << 8) + (unsigned int)buf[6]))& 0xffff;

            if((Maincommand == 0xF1) && (buf[4] == 0x51))
            {
                if(strstr((char *)&buf[7],"Not Supprot Command(0xF1") != NULL )
                {
                    LOG_G2_E(CLog::getLogOwner(), TAG, "Stayed in Bootloader.. ");
                }
                else if(strstr((char *)&buf[7],"USB bootloader - Not Supported Command(F1") != NULL )
                {
                    LOG_G2_E(CLog::getLogOwner(), TAG, "Stayed in USB Bootloader.. ");
                }
                else
                {
                    LOG_G2_D(CLog::getLogOwner(), TAG, "version get.. strange response %02x %02x ", buf[4], Maincommand);
                    return -2; //error
                }
            }

            else if((buf[4] != Maincommand) && (Maincommand != 0))
            {
                LOG_G2_E(CLog::getLogOwner(), TAG, "MainCommend different : %02x %02x ", buf[4], Maincommand);
                return 0; //error
            }

            if((buf[7] != Subcommand) && (Subcommand != 0))
            {
                LOG_G2_E(CLog::getLogOwner(), TAG, "SubCommand different : %02x %02x ", buf[7], Subcommand);
                return 0;  //error
            }

            if(m_packetlength <= ONE_PACKET_MAXBODY )
            {
                if(( buf[7+m_packetlength] == TOKEN_ETX1) && (buf[8+m_packetlength] == TOKEN_ETX2))
                {
                    stx_etx_ret = 1;
                }
            }
            else if((int)m_packetlength == ((ret-ONE_PACKET_EXCEPT_BODY)-1))
            {
                buf_pos +=((ret-ONE_PACKET_EXCEPT_BODY)-1);

                if(( buf[ret-1] == TOKEN_ETX1))
                {
                    stx_etx_ret = 1;
                }
            }
            else if((int)m_packetlength >= (ret-ONE_PACKET_EXCEPT_BODY))
            {
                buf_pos +=(ret-ONE_PACKET_EXCEPT_BODY);
                stx_etx_ret = 1;
            }
            else
            {
                LOG_G2_E(CLog::getLogOwner(), TAG, "end packet different1 : %02x %02x ", buf[7+m_packetlength] , buf[8+m_packetlength] );
                m_packetlength=0;
                return -2; //errors
            }
        }
        else if(buf_pos <= m_packetlength)
        {
            if(m_packetlength == (buf_pos +1)) //divede to ext1, etx2
            {
                if(( buf[ret-1] == TOKEN_ETX1))
                {
                    buf_pos+=(ret-REPORT_ID_INDEX_SIZE);
                    stx_etx_ret = 1;
                }
                else
                {
                    stx_etx_ret = 0;
                }
            }
            else
            {
                buf_pos+=(ret-REPORT_ID_INDEX_SIZE);
                stx_etx_ret = 1;

                if(buf_pos > m_packetlength)
                {
                    int nPos_etx = ((m_packetlength+ONE_PACKET_EXCEPT_BODY)%(ret-REPORT_ID_INDEX_SIZE));

                    if(nPos_etx == 1)
                    {
                        if(( buf[2] == TOKEN_ETX2))
                        {
                            stx_etx_ret = 1;
                        }
                    }
                    else if(( buf[nPos_etx] == TOKEN_ETX1) && (buf[nPos_etx+1] == TOKEN_ETX2))
                    {
                        stx_etx_ret = 1;
                    }
                    else
                    {
                        LOG_G2_E(CLog::getLogOwner(), TAG, "nPos_etx : %02x end packet different : %02x %02x ",nPos_etx, buf[nPos_etx] , buf[nPos_etx+1] );
                        LOG_G2_E(CLog::getLogOwner(), TAG, "buf_pos : %02x m_packetlength : %02x ",buf_pos, m_packetlength);
                        stx_etx_ret = 0;
                    }
                }
            }
        }

        if(stx_etx_ret == 0)
            ret = 0;
        /* --- */
    }
    else if(ret <= 0)
    {
        LOG_G2_I(CLog::getLogOwner(), TAG, "No Read Data !!");
        return -2;
    }

    return ret;
}

bool
DeviceIO_hid_over_i2c::isUsingIOCTL()
{
	return false;
}

bool DeviceIO_hid_over_i2c::isDeviceOpened()
{
	return m_bOpened;
}

//TODO: HID_I2C_OUTPUT_REGISTER, HID_I2C_OUTPUT_WO_LEN should be determined based on HID descriptor
//TODO: Report ID should be get from HID descriptor
//TODO: sequence number should be increased
int DeviceIO_hid_over_i2c::CreateCmdBuffer(unsigned char cmd, unsigned char* data, int data_len)
{
    int idx = 0;

    memset(out_buffer, 0, HID_OUTPUT_MAX_LEN);
    out_buffer[idx++] = HID_OUT_REPORT_ID;
    out_buffer[idx++] = index++;

    out_buffer[idx++] = TOKEN_STX1;
    out_buffer[idx++] = TOKEN_STX2;
    out_buffer[idx++] = cmd;
    out_buffer[idx++] = (data_len >> 8) & 0xff;
    out_buffer[idx++] = data_len & 0xff;

    for(int i=0; i<data_len; i++)
    {
        out_buffer[idx++] = data[i];
    }

    out_buffer[idx++] = TOKEN_ETX1;
    out_buffer[idx++] = TOKEN_ETX2;

    if(hid_Type == TYPE_USB)
    {
        if(idx != HID_OUTPUT_MAX_LEN)
        {
            for(; idx < (HID_OUTPUT_MAX_LEN);idx++){
                out_buffer[idx] = 0;
            }
        }
    }
    return idx;
}

int DeviceIO_hid_over_i2c::waitRxData(int &fd, int uSec)
{
	fd_set rfds;
	struct timeval tv;
	int retval;

	FD_ZERO(&rfds); /* clear the set */
	FD_SET(fd, &rfds); /* add our file descriptor to the set */

	LOG_G2_I(CLog::getLogOwner(), TAG, "waitIOReady : %d(uSec)", uSec);

	tv.tv_sec = 0;
	tv.tv_usec = uSec;

	retval = select(fd + 1, &rfds, NULL, NULL, &tv);

    //recover return value
	if (retval == -1)
	{
		LOG_G2_D(CLog::getLogOwner(), TAG, "! select, errno=%d (%s)", errno, strerror(errno)); /* an error accured */
		return errno;
	}
	else if (retval == 0)
	{
		LOG_G2_D(CLog::getLogOwner(), TAG, "! IO timeout, uSecWait=%d", uSec); /* a timeout occured */
		return -1;
	}
	else
	{
		LOG_G2_I(CLog::getLogOwner(), TAG, "IO data ready"); /* there was data to read */
		return 0;
	}
}


// TODO: read size should be changed based on descriptor
// TODO: architecture change
int DeviceIO_hid_over_i2c::TxSingleCmdWaitAck(unsigned char cmd, unsigned char* data, int data_len, int uSecWait)
{
    int nReadSize = -1;
    unsigned char *tmp_read = new unsigned char[HID_INPUT_MAX_LEN];

    do{
        waitRxData(m_fd, uSecWait);
        nReadSize = readData(tmp_read, HID_INPUT_MAX_LEN, cmd, 0);
    }while(nReadSize == 0);
    LOG_G2_I(CLog::getLogOwner(), TAG, "nReadSize : %d", nReadSize);
    ////////////////////////////////////////////////////////////////////////
    // Inside of readData, ather commands were ignored in this architecture.
    if (nReadSize > 2)
    {
        tmpRxUnit->setBuf(tmp_read, nReadSize);
        tmpRxUnit->setSize(nReadSize);
	}
    else
    {
        tmpRxUnit->clearBuf();
        tmpRxUnit->setSize(0);
    }

    delete [] tmp_read;

    return nReadSize;
}

int DeviceIO_hid_over_i2c::GetCmdWaitAckTime(unsigned char cmd, int mSec)
{
	return mSec * 1000;
}

int DeviceIO_hid_over_i2c::TryWriteData(unsigned char cmd, unsigned char* data, int data_len, int trial = 1, int mSecWait = 1000)
{
    int uSecWait = 0;
    int nTry = 0;
    int nTryResult = 2;
    int size = -1;
    int ret =0;

    ReadDataAll(1000);
    Pos_init(false);
    size = CreateCmdBuffer(cmd, data, data_len);	// m_buffer allocated

    uSecWait = GetCmdWaitAckTime(cmd, mSecWait);  // 30 msec for Normal command, but some command needs more time

    // Tx and Check Ack
    nTry = 0;
    while (++nTry <= trial)
    {
        int uSecWaitWr = uSecWait < 200000 ? uSecWait : 200000;
        ret = writeData(out_buffer, size);
        if (ret > 2)
        {	// Write success
            break;
        }
        usleep(uSecWaitWr);
    }

    if(ret <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "writeData fail");
        return ret;
    }

    LOG_G2_I(CLog::getLogOwner(), TAG, "uSecWait : %d", uSecWait);

    ret =0;
    nTry = 0;
    while (++nTry <= trial)
    {
        nTryResult = TxSingleCmdWaitAck(cmd, data, data_len, uSecWait);
        if (nTryResult > 2)
        {	// ACK received
            ret = 1;
            break;
        }

    }

    return ret;
}

string DeviceIO_hid_over_i2c::TxRequestFwVer(int mSec, int format=0)
{
    int nRequestResult = TryWriteData(CMD_FW_VER, NULL, 0, CMD_F1_RETRY_MAX, mSec);
    unsigned char* buf = tmpRxUnit->getBuf();
    int size=tmpRxUnit->getSize();

    if (nRequestResult > 0)
    {
    	// parsing FW_VER PACKET
    	if (size < 5)
    	{
    		LOG_G2_E(CLog::getLogOwner(), TAG, "Try check FW Ver, but rxUnit has wrong data : count of ack packet %d", size);
    		return "";
    	}
    	else if (buf[4] != CMD_FW_VER && buf[4] != 0x51)
    	{
    		LOG_G2_E(CLog::getLogOwner(), TAG, "Try check FW Ver, but rxUnit has different command : 0x%x", buf[4]);
    		return "";
    	}
    	else
    	{
    		// set FW Version & print out to scrn
            if(strstr((char *)&buf[7],"Not Supprot Command(0xF1") != NULL )
            {
                LOG_G2_I(CLog::getLogOwner(), TAG, "Force Set FW version 0.0.0");
                buf[10] = buf[11] = buf[12] = buf[13] = 0;
            }
            else if(strstr((char *)&buf[7],"USB bootloader - Not Supported Command(F1") != NULL )
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "Stayed in USB bootloader");
                LOG_G2_I(CLog::getLogOwner(), TAG, "Force Set FW version 0.0.0");
                buf[10] = buf[11] = buf[12] = buf[13] = 0;
            }
    		LOG_G2_I(CLog::getLogOwner(), TAG, "FW VERSION %d.%d.%d", buf[10], buf[11], (buf[12] * 256 + buf[13]));

            char temp[17];
            string strVersion = "";
            int cnt = 0;
            if(format == 0)
                cnt = snprintf(temp, 16, "%02d.%02d.%05d", buf[10], buf[11], (buf[12] * 256 + buf[13]));
            else
                cnt = snprintf(temp, 16, "%02X.%02X.%04X", buf[10], buf[11], (buf[12] * 256 + buf[13]));
    		if (cnt >= 0 && cnt < 100)
    		{
    			strVersion = string(temp, cnt);
    		}
    		else
    		{
    			LOG_G2_E(CLog::getLogOwner(), TAG, "Cannot change FW Version packet to string");
    		}

    		return strVersion;
    	}

    }

    return "";
}

void DeviceIO_hid_over_i2c::CUToBin(unsigned char* src, unsigned char* dest)
{
    int i=0, j=0, k=0;

    for(i=0; j <(0xAFF-1); i=i+4)
    {
        switch(j)
        {
            case 0:
                dest[j++]= src[i];
                dest[j++]= src[i+1];
                break;
            case 18:
                for(k=0;k < 8;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }
                i+=28;

                LOG_G2_D( CLog::getLogOwner(), TAG, "18 j = %d", j);
                break;
            case 50:
            case 70:
                dest[j++]= src[i];
                dest[j++]= src[i+1];
                break;
            case 598:
                dest[j++]= src[i];
                i+=4;
                for(k=0;k < 6;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }
                dest[j++]= src[i];
                i+=4;
                for(k=0;k < 2;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }
                dest[j++]= src[i];
                i+=8;

                LOG_G2_D( CLog::getLogOwner(), TAG, "598 j = %d", j);
                break;
            case 679:
                for(k=0;k < 2;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }

                for(k=0;k < 6;k++)
                {
                    dest[j++]= src[i];
                    i+=4;
                }

                for(k=0;k < 5;k++)
                {
                    dest[j++]= src[i];
                    i+=4;
                }

                for(k=0;k < 4;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }

                for(k=0;k < 5;k++)
                {
                    dest[j++]= src[i];
                    i+=4;
                }

                for(k=0;k < 6;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }

                for(k=0;k < 6;k++)
                {
                    dest[j++]= src[i];
                    i+=4;
                }

                for(k=0;k < 2;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }

                for(k=0;k < 2;k++)
                {
                    dest[j++]= src[i];
                    i+=4;
                }
                i+=4;

                LOG_G2_D( CLog::getLogOwner(), TAG, "679 j = %d", j);
                break;
            case 774:
            case 782:
            case 840:
            case 904:
                for(k=0;k < 2;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }
                i-=4;
                break;
            case 978:
                i+=4;
                dest[j++]= src[i];
                dest[j++]= src[i+1];
                i+=4;

                for(k=0;k < 3;k++)
                {
                    dest[j++]= src[i];
                    i+=4;
                }
                for(k=0;k < 1;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }

                i-=4;
                LOG_G2_D( CLog::getLogOwner(), TAG, "980 j = %d", j);
                break;
            case 1096:
                for(k=0;k < 2;k++)
                {
                    dest[j++]= src[i];
                    dest[j++]= src[i+1];
                    i+=4;
                }

                i+=4;
                LOG_G2_D( CLog::getLogOwner(), TAG, "1107 j = %d", j);
                break;

            default:
                dest[j++]= src[i];
                break;
        }
    }
}

int DeviceIO_hid_over_i2c::TxRequestCuUpdate(unsigned char* file_buf)
{
    unsigned char send_buffer[256]={0,};
    unsigned char dump_buffer[CU_FILE_SIZE+100]={0,};
    unsigned short send_length = 0x40;
    int pos = CU_START_POS;
    int cuend_pos = (CU_START_POS + CU_FILE_SIZE);
    unsigned char buf[64];
    int buf_size = 0;
    int cu_page = 0x320;
    unsigned int base_address = 0x08004000;
    int nRequestResult = TxRequestHW_Reset(true, CMD_F1_RETRY_MAX);
    int retry = 0;

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestHW_Reset error");
        return nRequestResult;
    }
    usleep(150000);

    buf_size = GotoBoot_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if(nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "GotoBoot_data error");
        return nRequestResult;
    }
    usleep(300000);

    buf_size = FWDownReady_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_CUDownReady");
        return nRequestResult;
    }

    while(retry < CMD_F1_RETRY_MAX)
    {
        buf_size = Cu_Erase_data(buf);
        nRequestResult = TryWriteData(CMD_CU_ERASE, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Cu_Erase_data error");
            if(retry == CMD_F1_RETRY_MAX-1)
                return nRequestResult;
        }
        else
        {
            break;
        }
        retry++;
    }

    while(pos < cuend_pos)
    {
        LOG_G2_D( CLog::getLogOwner(), TAG, "CU send_pos : %X, size : %X", pos, cuend_pos);
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        nRequestResult = CU_Write_CMD(send_buffer, send_length, CMD_CU_WRITE, cu_page);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "CU Write error");
            return nRequestResult;
        }

        pos+=send_length;
        cu_page++;
    }

    nRequestResult = Dump(dump_buffer, base_address, CU_FILE_SIZE, 5000);

    if(nRequestResult < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CU Dump error");
        return nRequestResult;
    }

    nRequestResult = dumpTofile_compare(dump_buffer, file_buf+CU_START_POS, CU_FILE_SIZE);
    if(nRequestResult == 1)
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "CU Write Success");
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CU Write Fail");
    }

    return nRequestResult;
}

int DeviceIO_hid_over_i2c::TxReqOnlyCuUpdate(unsigned char* file_buf)
{
    unsigned char send_buffer[256]={0,};
    unsigned char dump_buffer[CU_FILE_SIZE+100]={0,};
    unsigned char dest_buffer[CU_FILE_SIZE+100]={0,};
    unsigned short send_length = 0x40;
    int pos = 0;
    int cuend_pos = CU_FILE_SIZE;
    unsigned char buf[64];
    int buf_size = 0;
    int cu_page = 0x320;
    unsigned int base_address = 0x08004000;
    int nRequestResult = TxRequestHW_Reset(true, CMD_F1_RETRY_MAX);
    int retry = 0;

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestHW_Reset error");
        return nRequestResult;
    }
    usleep(150000);

    CUToBin(file_buf, dest_buffer);

    if(Targetbootversion(dump_buffer) < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Version Dump Fail !!!");
        return -1;
    }

    CheckTargetType(dump_buffer);

    if((target_type.compare("I2C") == 0) && (hid_Type != TYPE_I2C))
    {
        hid_Type = TYPE_I2C;
        HID_OUTPUT_MAX_LEN = HID_OUTPUT_MAX_LEN_I2C;
        HID_OUT_REPORT_ID = HID_OUT_REPORT_ID_I2C;
        LOG_G2_D(CLog::getLogOwner(), TAG, "type change: I2C");
    }
    else if((target_type.compare("USB") == 0) && (hid_Type != TYPE_USB))
    {
        hid_Type = TYPE_USB;
        HID_OUTPUT_MAX_LEN = HID_OUTPUT_MAX_LEN_USB;
        HID_OUT_REPORT_ID = HID_OUT_REPORT_ID_USB;
        LOG_G2_D(CLog::getLogOwner(), TAG, "type change: USB");
    }

    while(retry < CMD_F1_RETRY_MAX)
    {
        buf_size = Cu_Erase_data(buf);
        nRequestResult = TryWriteData(CMD_CU_ERASE, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Cu_Erase_data error");
            if(retry == CMD_F1_RETRY_MAX-1)
                return nRequestResult;
        }
        else
        {
            break;
        }
        retry++;
    }

    while(pos < cuend_pos)
    {
        LOG_G2_D( CLog::getLogOwner(), TAG, "CU send_pos : %X, size : %X", pos, cuend_pos);
        memcpy(send_buffer, dest_buffer+pos, sizeof(send_buffer));

        nRequestResult = CU_Write_CMD(send_buffer, send_length, CMD_CU_WRITE,cu_page);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "CU Write error");
            return nRequestResult;
        }

        pos+=send_length;
        cu_page++;
    }

    nRequestResult = Dump(dump_buffer, base_address, CU_FILE_SIZE, 5000);

    if(nRequestResult < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CU Dump error");
        return nRequestResult;
    }

    nRequestResult = dumpTofile_compare(dump_buffer, dest_buffer, CU_FILE_SIZE);
    if(nRequestResult == 1)
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "CU Write Success");
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CU Write Fail");
    }

    return nRequestResult;
}

int DeviceIO_hid_over_i2c::TxReqOnlyFwUpdate(unsigned char* file_buf, int size)
{
    unsigned char buf[64];
    int buf_size = 0;
    int nRequestResult = 0;
    int pos = 0;
    int fw_size = GetFW_size(file_buf, size);
    int fw_end_pos = fw_size;
    int send_length = 0x100;
    unsigned char send_buffer[256]={0,};
    unsigned int fw_checksum = 0;
    unsigned char* read_buf;
    int padsize=0;
    int remain=0;
    int align=0;
    unsigned char dump_buffer[BOOT_VERSION_SIZE]={0,};
    int i;

    if(Targetbootversion(dump_buffer) < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Version Dump Fail !!!");
        return -1;
    }

    CheckTargetType(dump_buffer);
    CheckFWFileType(file_buf);

    if(CompareTargetToFile() != 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Target and File Type Err !!!");
        return -1;
    }

    if (fw_size == 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "fw_write_size ERR");
        return nRequestResult;
    }

    padsize = fw_end_pos - size;
    remain = 256-padsize;
    align = size - remain;

    buf_size = GotoBoot_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if(nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "GotoBoot_data error");
        return nRequestResult;
    }
    usleep(300000);

    buf_size = FWDownReady_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FWDownReady");
        return nRequestResult;
    }

    buf_size = FlashErase_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashErase");
        return nRequestResult;
    }

    while(pos < fw_end_pos)
    {
        LOG_G2_D( CLog::getLogOwner(), TAG, "FW send_pos : %X, size : %X, real:%X", pos, fw_end_pos, size);

        if(pos < align)
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        else
        {
            for(i=0; i < 256; i++)
            {
                if(i < remain)
                send_buffer[i] = file_buf[pos+i];
                else
                send_buffer[i] = 0xFF;
            }
        }

        for(i=0; i < 256;i=i+8)
        {
            fw_checksum += (send_buffer[i]+send_buffer[i+1]+send_buffer[i+2]+send_buffer[i+3]+send_buffer[i+4]+send_buffer[i+5]+send_buffer[i+6]+send_buffer[i+7]);
        }

        nRequestResult = FW_Write_CMD(send_buffer, send_length, CMD_MAIN_CMD);

        if (nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "FW Write error");
            return nRequestResult;
        }

        pos+=send_length;
    }

    LOG_G2_E(CLog::getLogOwner(), TAG, "TxReqOnlyFwUpdate fw_size = 0x%x, padsize=%d, remain=%d, align=0x%x", fw_size, padsize, remain, align);

    buf_size = FlashCheckSum_data(buf,fw_size,fw_checksum);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashCheckSum send error");
        return nRequestResult;
    }

    read_buf = tmpRxUnit->getBuf();
    nRequestResult = FlashCheckSum_Check(read_buf, fw_checksum);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CheckSum Error");
        return nRequestResult;
    }

    buf_size = FlashFinish_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashFinish send error");
        return nRequestResult;
    }

    read_buf = tmpRxUnit->getBuf();
    nRequestResult = FlashFinish_Check(read_buf);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashFinish error");
        return nRequestResult;
    }
    else
    {
        LOG_G2(CLog::getLogOwner(), TAG, "FW update Success");
    }

    return nRequestResult;
}

int DeviceIO_hid_over_i2c::TxRequestFwUpdate(unsigned char* file_buf)
{
    unsigned char buf[64];
    int buf_size = 0;
    int nRequestResult = 0;
    int pos = FW_START_POS;
    int fw_size = Fw_write_size(file_buf);
    int fw_end_pos = (FW_START_POS + fw_size);
    int send_length = 0x100;
    unsigned char send_buffer[256]={0,};
    unsigned int fw_checksum = 0;
    unsigned char* read_buf;

    if (fw_size == 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "fw_write_size ERR");
        return nRequestResult;
    }

    buf_size = FWDownReady_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FWDownReady Faile");
        return nRequestResult;
    }

    buf_size = FlashErase_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashErase Fail");
        return nRequestResult;
    }

    while(pos < fw_end_pos)
    {
        LOG_G2_D( CLog::getLogOwner(), TAG, "FW send_pos : %X, size : %X", pos, fw_end_pos);
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        nRequestResult = FW_Write_CMD(send_buffer, send_length, CMD_MAIN_CMD);

        if (nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "FW Write error");
            return nRequestResult;
        }

        pos+=send_length;
    }

    for(int i=FW_START_POS; i<fw_end_pos; ++i)
        fw_checksum += file_buf[i];

    buf_size = FlashCheckSum_data(buf,fw_size,fw_checksum);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashCheckSum send error");
        return nRequestResult;
    }        

    read_buf = tmpRxUnit->getBuf();
    nRequestResult = FlashCheckSum_Check(read_buf, fw_checksum);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CheckSum Error");
        return nRequestResult;
    }

    buf_size = FlashFinish_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashFinish send error");
        return nRequestResult;
    }

    read_buf = tmpRxUnit->getBuf();
    nRequestResult = FlashFinish_Check(read_buf);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "CMD_FlashFinish error");
        return nRequestResult;
    }
    else
    {
        LOG_G2(CLog::getLogOwner(), TAG, "FW update Success");
    }

    return nRequestResult;
}

int DeviceIO_hid_over_i2c::GetFW_size(unsigned char* file_buf, int size)
{
    int write_size=size;

    if(write_size%0x100 != 0)
    {
        int blank_data = (256-(write_size%256));
        write_size += blank_data;
    }

    return write_size;
}

int DeviceIO_hid_over_i2c::Fw_write_size(unsigned char* file_buf)
{
    int i=0;
    int write_size=0;

    for(i=0; i<4; i++)
    {
        if(file_buf[0x1fff0+i] != 0xa5)
        {
            return write_size;
        }
    }

    write_size = (file_buf[0x1fff0 + 11] <<24);
    write_size+= (file_buf[0x1fff0 + 10] <<16);
    write_size+= (file_buf[0x1fff0 + 9] <<8);
    write_size+= file_buf[0x1fff0 + 8];

    if(write_size%0x100 != 0)
    {
        int blank_data = (256-(write_size%256));
        write_size += blank_data;
    }

    return write_size;
}

int DeviceIO_hid_over_i2c::TxRequestHW_Reset(bool chkack, int trial)
{
    int nRequestResult = TryWriteData(CMD_HW_RESET, NULL, 0, trial, 2000);

    LOG_G2_D(CLog::getLogOwner(), TAG, "nRequestResult : %d", nRequestResult);

    if(chkack)
    {
        if (nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestHW_Reset Error");
        }
    }

    return nRequestResult;
}

int DeviceIO_hid_over_i2c::TxRequestSystem_Reset()
{
    unsigned char buf[64]={0,};
    int buf_size = 0;

    buf_size = system_reset_data(buf);
    int nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 3000);

    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestSystem_Reset Error");
    }

    return nRequestResult;
}


int DeviceIO_hid_over_i2c::TxRequestBootUpdate(unsigned char* file_buf, bool bBoot_force_update)
{
    LOG_G2_D(CLog::getLogOwner(), TAG, "TxRequestBootUpdate START");

    unsigned char buf[64];
    int bootstart_address = 0x08000000;
    int buf_size = 0;
    int nRequestResult = 0;
    unsigned char dump_buffer[BOOT_FILE_SIZE+10]={0,};
    unsigned char send_buffer[256]={0,};
    int pos = 0;
    int boot_end_pos = BOOT_FILE_SIZE;
    int send_length = 0x100;
    string fw_ver = "";

    fw_ver = TxRequestFwVer(1000);
    if(strstr(fw_ver.c_str(), "00.00.0000") != NULL)
    {
        //LOG_G2_E(CLog::getLogOwner(), TAG, "Can't update bootloader.. Try it again After flashing !!!");
        return 2;
    }

    if(Targetbootversion(dump_buffer) < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Version Dump Fail !!!");
        return -1;
    }

    nRequestResult = dumpTofile_compare(dump_buffer, file_buf+BOOT_VER_POS, 0x20);

    if(nRequestResult != 1)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Fail to Version Dump compare with file");
        return nRequestResult;
    }

    CheckTargetType(dump_buffer);
    CheckBootFileType(file_buf);

    if(CompareTargetToFile() != 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Target and File Type Err !!!");
        return -1;
    }

    if(bBoot_force_update == false)
    {
        if(nRequestResult == 1)
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Same Bootloader");
            return nRequestResult;
        }
        else
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Different Bootloader, Bootloader Update Needed");
        }
    }

    nRequestResult = TxRequestHW_Reset(true, CMD_F1_RETRY_MAX);
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestHW_Reset error");
        return nRequestResult;
    }
    usleep(150000);

    buf_size = boot_erase_data(buf);
    nRequestResult = TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);
    if (nRequestResult <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "boot_erase_data error");
        return nRequestResult;
    }

    while(pos < boot_end_pos)
    {
        LOG_G2_D( CLog::getLogOwner(), TAG, "Boot send_pos : %X, end_pos : %X", pos, boot_end_pos);
        memcpy(send_buffer, file_buf+pos, sizeof(send_buffer));
        nRequestResult = Boot_Write_CMD(send_buffer, send_length, CMD_MAIN_CMD);

        if(nRequestResult <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "Boot Write error");
            return nRequestResult;
        }

        pos+=send_length;
    }

    nRequestResult = Dump(dump_buffer, bootstart_address, BOOT_FILE_SIZE, 5000);

    if(nRequestResult < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Boot Dump error");
        return nRequestResult;
    }

    nRequestResult = dumpTofile_compare(dump_buffer, file_buf, BOOT_FILE_SIZE);

    if(nRequestResult == 1)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "Bootloader Update Success");
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Bootloader Update Fail");
        LOG_G2_E(CLog::getLogOwner(), TAG, "Do not shut down !! Please retry again.");
    }

    return nRequestResult;

}

int DeviceIO_hid_over_i2c::boot_erase_data(unsigned char* buf)
{
    int size = 0;
    unsigned int eraseSize = 0x4000;

    buf[size++] = G2_SUB_0x13_FLASH_ERASE;
    buf[size++] = G2_SUB_0x13_FLASH_BOOT;
    buf[size++]=(eraseSize>>24)&0xFF;
    buf[size++]=(eraseSize>>16)&0xFF;
    buf[size++]=(eraseSize>>8)&0xFF;
    buf[size++]=(eraseSize)&0xFF;

    return size;
}

int DeviceIO_hid_over_i2c::system_reset_data(unsigned char* buf)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_SYSTEMRESET;

    return size;
}

int DeviceIO_hid_over_i2c::dumpTofile_compare(unsigned char* dump_buffer, unsigned char* file_buf, int compare_size)
{
    int i=0;

    for(i=0; i<compare_size; i++)
    {
        if(dump_buffer[i] != file_buf[i])
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "dump[%X]=%02X, buf[%X]=%02X", i, dump_buffer[i], i, file_buf[i]);
            return 0;
        }
    }

    return 1;
}

int DeviceIO_hid_over_i2c::GotoBoot_data(unsigned char* buf)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_GOTOBOOT;
    return size;
}

int DeviceIO_hid_over_i2c::Cu_Erase_data(unsigned char* buf)
{
    int size = 0;

    buf[size++]=0x03;
    buf[size++]=0x20;
    buf[size++]=0x00;
    buf[size++]=0x2c;

    return size;
}

int DeviceIO_hid_over_i2c::FWDownReady_data(unsigned char* buf)
{
    int size = 0;
    unsigned int base_address = 0x08008000;

    buf[size++] = G2_SUB_0x13_FWDOWNREADY;
    buf[size++]=(base_address>>24)&0xFF;
    buf[size++]=(base_address>>16)&0xFF;
    buf[size++]=(base_address>>8)&0xFF;
    buf[size++]=(base_address)&0xFF;

    return size;
}

int DeviceIO_hid_over_i2c::FlashErase_data(unsigned char* buf)
{
    int size = 0;
    unsigned int base_address = 0x08008000;
    unsigned int fw_erase_size = 0x00018000;

    buf[size++] = G2_SUB_0x13_FWERASE;
    buf[size++]=(base_address>>24)&0xFF;
    buf[size++]=(base_address>>16)&0xFF;
    buf[size++]=(base_address>>8)&0xFF;
    buf[size++]=(base_address)&0xFF;
    buf[size++]=(fw_erase_size>>24)&0xFF;
    buf[size++]=(fw_erase_size>>16)&0xFF;
    buf[size++]=(fw_erase_size>>8)&0xFF;
    buf[size++]=(fw_erase_size)&0xFF;

    return size;
}


int DeviceIO_hid_over_i2c::FlashCheckSum_data(unsigned char* buf, int file_size, unsigned int fw_checksum)
{
    int size = 0;
    unsigned int base_address = 0x08008000;

    buf[size++] = G2_SUB_0x13_FLASHCHKSUM;
    buf[size++]=(base_address>>24)&0xFF;
    buf[size++]=(base_address>>16)&0xFF;
    buf[size++]=(base_address>>8)&0xFF;
    buf[size++]=(base_address)&0xFF;
    buf[size++]=(file_size>>24)&0xFF;
    buf[size++]=(file_size>>16)&0xFF;
    buf[size++]=(file_size>>8)&0xFF;
    buf[size++]=(file_size)&0xFF;
    buf[size++]=(fw_checksum>>24)&0xFF;
    buf[size++]=(fw_checksum>>16)&0xFF;
    buf[size++]=(fw_checksum>>8)&0xFF;
    buf[size++]=(fw_checksum)&0xFF;

    return size;
}

int DeviceIO_hid_over_i2c::FlashCheckSum_Check(unsigned char* buf, unsigned int fw_checksum)
{
    int ret = 0;

    if(buf[7] == G2_SUB_0x13_FLASHCHKSUM)
    {
        if((buf[9] == ((fw_checksum>>24)&0xFF)) &&  (buf[10] == ((fw_checksum>>16)&0xFF)) &&\
            (buf[11] == ((fw_checksum>>8)&0xFF) ) &&  (buf[12] == ((fw_checksum)&0xFF))  )
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "CHECKSUM  OK");
            ret = 1;
        }
        else
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "CHECKSUM  ERROR");
        }
    }

    return ret;
}

int DeviceIO_hid_over_i2c::FlashFinish_data(unsigned char* buf)
{
    int size = 0;

    buf[size++] = G2_SUB_0x13_FLASHFINISH;
    buf[size++] = 0x00;

    return size;
}

int DeviceIO_hid_over_i2c::FlashFinish_Check(unsigned char* read_buf)
{
    LOG_G2_D(CLog::getLogOwner(), TAG, "read_buf[7] : %x read_buf[8] %x",read_buf[7],read_buf[8]);

    if((read_buf[7] == G2_SUB_0x13_FLASHFINISH) && (read_buf[8] == 0x00) )
    {
        return 1;
    }

    return 0;
}

int DeviceIO_hid_over_i2c::CU_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd, int cu_page)
{
    int uSecWait = GetCmdWaitAckTime(0xb4, 1000);  // 30 msec for Normal command, but some command needs more time
    unsigned char buf[512+9];
    unsigned char i2c_buf[HID_OUTPUT_MAX_LEN_USB];
    unsigned char ret = 1;
    unsigned int send_len=0, sended_len=0;
    unsigned int i=0,pos=0, len=0;
    int retry_ack = 5;

    memset(i2c_buf, 0x00, sizeof(i2c_buf));

    len = send_length+2;
    buf[0] = TOKEN_STX1;
    buf[1] = TOKEN_STX2;
    buf[2] = send_cmd;
    buf[3] = (len>>8)  & 0xFF;
    buf[4] = len & 0xFF;

    buf[5] = (cu_page>>8)  & 0xFF;
    buf[6] = cu_page & 0xFF;

    for(i=0; i<len-2; i++) {
        buf[7+i] = send_buffer[i];
    }
    buf[5+len] = TOKEN_ETX1;
    buf[6+len] = TOKEN_ETX2;

    len += 7;

    sended_len = 0;
    while(sended_len < len)
    {
        if(len < (unsigned int)(HID_OUTPUT_MAX_LEN - 2))
        {
             send_len = len;
        }
        else
        {
            if((len - sended_len) > (unsigned int)(HID_OUTPUT_MAX_LEN - 2))
            {
                send_len = (HID_OUTPUT_MAX_LEN - 2);
            }
            else
            {
                send_len = (len - sended_len);
            }
        }

        i2c_buf[0] = HID_OUT_REPORT_ID;
        i2c_buf[1] = index++;
        if(index == 0xAA) index++;

        for(i= 0; i < send_len; i++){
            i2c_buf[i+2] = buf[pos++];
        }
        for(i=send_len; i < (HID_OUTPUT_MAX_LEN);i++){
            i2c_buf[i+2] = 0;
        }
        sended_len += send_len;

        ret = writeData( i2c_buf, HID_OUTPUT_MAX_LEN);

        if(ret <= 0)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "CU Write error2");
            return ret;
        }
    }

    while(retry_ack >= 0)
    {
        retry_ack--;
        do{
            waitRxData(m_fd, uSecWait);
            ret = readData(buf, HID_INPUT_MAX_LEN, 0xb4, 0 );
        }while(ret == 0);

        if(ret > 2)
        {
            break;
        }
        usleep(1000 * 500);
    }

    return ret;
}


int DeviceIO_hid_over_i2c::FW_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd)
{
    int uSecWait = GetCmdWaitAckTime(0xb4, 1000);
    unsigned char buf[512+9];
    unsigned char i2c_buf[HID_OUTPUT_MAX_LEN_USB];
    unsigned char ret = 1;
    unsigned long i=0,pos=0,sended_len, send_len, len;
    int retry_ack = 5;

    memset(i2c_buf, 0x00, sizeof(i2c_buf));

    len = send_length+1;

    buf[0] = TOKEN_STX1;
    buf[1] = TOKEN_STX2;
    buf[2] = send_cmd;
    buf[3] = (len>>8)  & 0xFF;
    buf[4] = len & 0xFF;
    buf[5] = G2_SUB_0x13_FWWRITE;

    for(i=0; i<len-1; i++) {
        buf[6+i] = send_buffer[i];
    }
    buf[5+len] = TOKEN_ETX1;
    buf[6+len] = TOKEN_ETX2;

    len += 7;

    sended_len = 0;
    while(sended_len < len)
    {
        if(len < (unsigned int)(HID_OUTPUT_MAX_LEN - 2))
        {
             send_len = len;
        }
        else
        {
            if((len - sended_len) > (unsigned int)(HID_OUTPUT_MAX_LEN - 2))
            {
                send_len = (HID_OUTPUT_MAX_LEN - 2);
            }
            else
            {
                send_len = (len - sended_len);
            }
        }

        i2c_buf[0] = HID_OUT_REPORT_ID;
        i2c_buf[1] = index++;

        if(index == 0xAA) index++;

        for(i= 0; i < send_len; i++){
            i2c_buf[i+2] = buf[pos++];
        }
        for(i=send_len; i < (HID_OUTPUT_MAX_LEN);i++){
            i2c_buf[i+2] = 0;
        }
        sended_len += send_len;

        ret = writeData( i2c_buf, HID_OUTPUT_MAX_LEN);

        if(ret <= 0)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "FW Write error2");
            return ret;
        }
    }

    while(retry_ack >= 0)
    {
        retry_ack--;
        do{
            waitRxData(m_fd, uSecWait); //30ms
            ret = readData(buf, HID_INPUT_MAX_LEN, 0x13, 0x83 );
        }while(ret ==0);

        if(ret > 2)
        {
            break;
        }
        usleep(1000 * 500);
    }

    return ret;
}

int DeviceIO_hid_over_i2c::Boot_Write_CMD(unsigned char* send_buffer, unsigned short send_length, unsigned char send_cmd)
{
    int uSecWait = GetCmdWaitAckTime(0xb4, 1000);
    unsigned char buf[512+9];
    unsigned char i2c_buf[HID_OUTPUT_MAX_LEN+10];
    unsigned char ret = 1;
    unsigned long i=0,pos=0,sended_len, send_len, len;
    int retry_ack = 5;

    memset(i2c_buf, 0x00, sizeof(i2c_buf));

    len = send_length+2;

    buf[0] = TOKEN_STX1;
    buf[1] = TOKEN_STX2;
    buf[2] = send_cmd;
    buf[3] = (len>>8)  & 0xFF;
    buf[4] = len & 0xFF;
    buf[5] = G2_SUB_0x13_FLASH_WRITE;
    buf[6] = 0x00;

    for(i=0; i<len-2; i++) {
        buf[7+i] = send_buffer[i];
    }
    buf[5+len] = TOKEN_ETX1;
    buf[6+len] = TOKEN_ETX2;

    len += 7;

    sended_len = 0;
    while(sended_len < len)
    {
        if(len < (unsigned int)(HID_OUTPUT_MAX_LEN - 2))
        {
             send_len = len;
        }
        else
        {
            if((len - sended_len) > (unsigned int)(HID_OUTPUT_MAX_LEN - 2))
            {
                send_len = (HID_OUTPUT_MAX_LEN - 2);
            }
            else
            {
                send_len = (len - sended_len);
            }
        }

        i2c_buf[0] = HID_OUT_REPORT_ID;
        i2c_buf[1] = 0x00;

        if(index == 0xAA) index++;

        for(i= 0; i < send_len; i++){
            i2c_buf[i+2] = buf[pos++];
        }
        for(i=send_len; i < (HID_OUTPUT_MAX_LEN);i++){
            i2c_buf[i+2] = 0;
        }
        sended_len += send_len;

        ret = writeData( i2c_buf, HID_OUTPUT_MAX_LEN);

        if(ret <= 0)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "Boot Write error2");
            return ret;
        }
    }

    while(retry_ack >= 0)
    {
        retry_ack--;
        do{
            waitRxData(m_fd, uSecWait); //30ms
            ret = readData(buf, HID_INPUT_MAX_LEN, 0x13, 0x06 );
        }while(ret == 0);

        if(ret > 2)
        {
            break;
        }
        usleep(1000 * 500);
    }

    return ret;
}


int DeviceIO_hid_over_i2c::Dump(unsigned char* out_buf, int address, unsigned int size, int ntime)
{
    int nRet = -1;
    unsigned int dump_size = 0x100;
    struct timespec start;
    struct timespec end;
    long long time_diff = 0;
    long long ntimeout = (long long)ntime * ONE_MILLION;

    LOG_G2_D(CLog::getLogOwner(), TAG, "Dump START ");
    Pos_init(false);
    clock_gettime(CLOCK_MONOTONIC,&start);

    while(read_pos < size)
    {
        if(size - read_pos < dump_size)
        {
            dump_size = (size - read_pos);
        }

        nRet = FlashDump(address+read_pos, dump_size);

        if( nRet < 0)
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "Dump Tx cmd fail : %d",nRet);
            return -1;
        }

        do
        {
            do{
                waitRxData(m_fd, ntime*1000); //30ms
                nRet = readData(in_buffer, HID_INPUT_MAX_LEN, CMD_MAIN_CMD, G2_SUB_0x13_FLASH_READ);
            }while(nRet == 0);

            if(nRet < 2)
            {
                LOG_G2_E(CLog::getLogOwner(), TAG, "Ackfail : %d",nRet);
                Pos_init(false);
                return -1; //error
            }

            if(nRet < DUMP_PACKET_EXCEPT_BODY)
            {
                LOG_G2_E(CLog::getLogOwner(), TAG, "Dump error : %d",buf_pos);
                Pos_init(false);
                return -1; //error
            }

            if(m_packetlength < (unsigned int)(nRet-ONE_PACKET_EXCEPT_BODY))
            {
                memcpy(out_buf+read_pos, in_buffer+DUMP_PACKET_EXCEPT_BODY, (m_packetlength - ONE_EXCEPT_PACKET));
                read_pos += (m_packetlength - ONE_EXCEPT_PACKET);
            }
            else if(buf_pos == (unsigned int)(nRet-ONE_PACKET_EXCEPT_BODY))
            {
                memcpy(out_buf+read_pos, in_buffer+DUMP_PACKET_EXCEPT_BODY, (nRet - DUMP_PACKET_EXCEPT_BODY));
                read_pos += (nRet - DUMP_PACKET_EXCEPT_BODY);
            }
            else if(buf_pos < m_packetlength)
            {
                memcpy(out_buf+read_pos, in_buffer+REPORT_ID_INDEX_SIZE, nRet -REPORT_ID_INDEX_SIZE );
                read_pos += (nRet -REPORT_ID_INDEX_SIZE);
            }
            else
            {
                memcpy(out_buf+read_pos, in_buffer+REPORT_ID_INDEX_SIZE, (m_packetlength - (buf_pos-(nRet-REPORT_ID_INDEX_SIZE)) ) );
                read_pos += (m_packetlength - (buf_pos-(nRet-REPORT_ID_INDEX_SIZE)) );
                LOG_G2_I(CLog::getLogOwner(), TAG, "(dump_size - buf_pos) : %x",(m_packetlength - (buf_pos-(nRet-REPORT_ID_INDEX_SIZE)) ));
                LOG_G2_I(CLog::getLogOwner(), TAG, "read_pos4 : %x, buf_pos: %x",read_pos, buf_pos);
            }
        }
        while(buf_pos < m_packetlength && buf_pos != 0);
        buf_pos = 0;

        clock_gettime(CLOCK_MONOTONIC,&end);

        time_diff = (NSECTOSEC*(end.tv_sec-start.tv_sec)) + (end.tv_nsec - start.tv_nsec);
        if(time_diff > ntimeout) {
            LOG_G2_I(CLog::getLogOwner(), TAG, "timed out before getting an event : %Ld %Ld",time_diff, ntimeout);
            Pos_init(false);
            return -2;
        }
        LOG_G2_I(CLog::getLogOwner(), TAG, "read_pos : %x, size: %x",read_pos, size);
    }

    Pos_init(false);
    return 0;
}

int DeviceIO_hid_over_i2c::FlashDump(int address, int size)
{
    int ret = 0;
    int idx = 0;
    unsigned char Buf[18] ;

    Buf[idx++] = 0x0A;
    Buf[idx++] = index;
    Buf[idx++] =  TOKEN_STX1 ;
    Buf[idx++] =  TOKEN_STX2 ;
    Buf[idx++] =  (unsigned char)CMD_MAIN_CMD ;
    Buf[idx++] =  (unsigned char)0x00 ;
    Buf[idx++] =  (unsigned char)0x09 ;
    Buf[idx++] =  (unsigned char)G2_SUB_0x13_FLASH_READ ;

    Buf[idx++]=(address>>24)&0xFF;
    Buf[idx++]=(address>>16)&0xFF;
    Buf[idx++]=(address>>8)&0xFF;
    Buf[idx++]=(address)&0xFF;

    Buf[idx++]=(size>>24)&0xFF;
    Buf[idx++]=(size>>16)&0xFF;
    Buf[idx++]=(size>>8)&0xFF;
    Buf[idx++]=(size)&0xFF;

    Buf[idx++] =  (unsigned char)0x03 ;
    Buf[idx++] =  (unsigned char)0xb3 ;

    ret = write(m_fd, Buf, idx);

    return ret;
}

void DeviceIO_hid_over_i2c::Pos_init(bool init_idx)
{
    buf_pos = 0;
    read_pos = 0;
    m_packetlength = 0;
}

int DeviceIO_hid_over_i2c::ReadDataAll(int duration)
{
    int nReadSize = -1;
    struct timespec start;
    struct timespec end;
    long long time_diff = 0;

    long long ntimeout = (long long)duration * ONE_MILLION;

    Pos_init(false);
    clock_gettime(CLOCK_MONOTONIC,&start);

    while(1)
    {
        waitRxData(m_fd, 40*1000);
        nReadSize = read(m_fd, in_buffer, READ_SIZE);

        if(nReadSize > 2)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "Rx DATA ALL");
        }
        else
        {
            Pos_init(false);
            LOG_G2_I(CLog::getLogOwner(), TAG, "Rx DATA ALL");
            return 0;
        }

        clock_gettime(CLOCK_MONOTONIC,&end);

        time_diff = (NSECTOSEC*(end.tv_sec-start.tv_sec)) + (end.tv_nsec - start.tv_nsec);
        if(time_diff > ntimeout)
        {
            LOG_G2_I(CLog::getLogOwner(), TAG, "timed out before getting an event : %Ld %Ld",time_diff, ntimeout);
            return -1;
        }
    }

    return -2;
}

int DeviceIO_hid_over_i2c::Targetbootversion(unsigned char* dump_buffer)
{
    int nRequestResult = -1;
    int bootver_address = 0x08000400;

    nRequestResult = Dump(dump_buffer, bootver_address, 0x20, 5000);

    if(nRequestResult < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Version Dump Fail");
        return nRequestResult; //boot_ver same
    }

    return nRequestResult;
}

void DeviceIO_hid_over_i2c::CheckTargetType(unsigned char* dump_buffer)
{
    char targ_boot[9]={0,};

    memcpy((char*)targ_boot, (char*)(dump_buffer+8), 8);

    if((targ_boot[0] == '0') && (targ_boot[1] == '0'))
    {
        target_type = "USB";
    }
    else if((targ_boot[0] == '0') && (targ_boot[1] == '1'))
    {
        target_type = "I2C";
    }

}

void DeviceIO_hid_over_i2c::CheckBootFileType(unsigned char* file_buf)
{
    char curr_file[9]={0,};

    memcpy((char*)curr_file, (char*)(file_buf+BOOT_VER_POS+8), 8);

    if((curr_file[0] == '0') && (curr_file[1] == '0'))
    {
        file_type = "USB";
    }
    else if((curr_file[0] == '0') && (curr_file[1] == '1'))
    {
        file_type = "I2C";
    }
}

void DeviceIO_hid_over_i2c::CheckFWFileType(unsigned char* file_buf)
{
    unsigned char VID[4]={0,};
    unsigned char PID[4]={0,};
    unsigned char Temp_Compare[4]={0,};

    memcpy((char*)VID, (char*)(file_buf+FW_VID_POS), 4);
    memcpy((char*)PID, (char*)(file_buf+FW_PID_POS), 4);

    for (int i = 1; i < 4; i++)
    {
        memcpy((char*)Temp_Compare, (char*)(file_buf+FW_VID_POS)+(i*4), 4);

        LOG_G2_D(CLog::getLogOwner(), TAG, "VID : %x %x %x %x",VID[0],VID[1],VID[2],VID[3]);
        LOG_G2_D(CLog::getLogOwner(), TAG, "Temp_Compare : %x %x %x %x",Temp_Compare[0],Temp_Compare[1],Temp_Compare[2],Temp_Compare[3]);
        if (VID[0] != Temp_Compare[0] || VID[1] != Temp_Compare[1] || VID[2] != Temp_Compare[2] || VID[3] != Temp_Compare[3])
        {
            file_type = "USB";
            return;
        }
    }

    for (int i = 1; i < 4; i++)
    {
        memcpy((char*)Temp_Compare, (char*)(file_buf+FW_PID_POS)+(i*4), 4);

        LOG_G2_D(CLog::getLogOwner(), TAG, "PID : %x %x %x %x",PID[0],PID[1],PID[2],PID[3]);
        LOG_G2_D(CLog::getLogOwner(), TAG, "Temp_Compare : %x %x %x %x",Temp_Compare[0],Temp_Compare[1],Temp_Compare[2],Temp_Compare[3]);
        if (PID[0] != Temp_Compare[0] || PID[1] != Temp_Compare[1] || PID[2] != Temp_Compare[2] || PID[3] != Temp_Compare[3])
        {
            file_type = "USB";
            return;
        }
    }

    file_type = "I2C";
}



int DeviceIO_hid_over_i2c::CompareTargetToFile()
{
    if(( file_type.compare(target_type) != 0) && (target_type.compare("") != 0 ))
    {
        LOG_G2(CLog::getLogOwner(), TAG,"Target Boot[%s] and F/W File[%s] are Different Interface.",target_type.c_str(),file_type.c_str());
        return TYPE_ERR;
    }

    LOG_G2_D(CLog::getLogOwner(), TAG, "file_type : %s",file_type.c_str());
    LOG_G2_D(CLog::getLogOwner(), TAG, "hid_Type : %d",hid_Type);

    if((file_type.compare("I2C") == 0) && (hid_Type != TYPE_I2C))
    {
        hid_Type = TYPE_I2C;
        HID_OUTPUT_MAX_LEN = HID_OUTPUT_MAX_LEN_I2C;
        HID_OUT_REPORT_ID = HID_OUT_REPORT_ID_I2C;
        LOG_G2_D(CLog::getLogOwner(), TAG, "type change: I2C");
    }
    else if((file_type.compare("USB") == 0) && (hid_Type != TYPE_USB))
    {
        hid_Type = TYPE_USB;
        HID_OUTPUT_MAX_LEN = HID_OUTPUT_MAX_LEN_USB;
        HID_OUT_REPORT_ID = HID_OUT_REPORT_ID_USB;
        LOG_G2_D(CLog::getLogOwner(), TAG, "type change: USB");
    }

    return 0;

}

