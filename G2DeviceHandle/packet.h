#ifndef PACKET_H
#define PACKET_H

#include <string.h>

#define HID_OUTPUT_MAX_LEN_USB  (64)
#define HID_OUT_REPORT_ID_USB   (0x80)
#define HID_OUTPUT_MAX_LEN_I2C  (59)
#define HID_OUT_REPORT_ID_I2C   (0x0A)
#define HID_INPUT_MAX_LEN   (64)
#define ONE_MILLION 1000000
#define READ_SIZE  64
#define ONE_PACKET_MAXBODY  52
#define ONE_PACKET_EXCEPT_BODY  7
#define ONE_EXCEPT_PACKET 9
#define DUMP_PACKET_EXCEPT_BODY 16
#define NSECTOSEC 1000000000
#define REPORT_ID_INDEX_SIZE 2

#define BUS_PCI			0x01
#define BUS_ISAPNP		0x02
#define BUS_USB			0x03
#define BUS_HIL			0x04
#define BUS_BLUETOOTH		0x05
#define BUS_VIRTUAL		0x06
#define BUS_ISA			0x10
#define BUS_I8042		0x11
#define BUS_XTKBD		0x12
#define BUS_RS232		0x13
#define BUS_GAMEPORT		0x14
#define BUS_PARPORT		0x15
#define BUS_AMIGA		0x16
#define BUS_ADB			0x17
#define BUS_I2C			0x18
#define BUS_HOST		0x19
#define BUS_GSC			0x1A
#define BUS_ATARI		0x1B
#define TYPE_I2C    0
#define TYPE_USB   1
#define TYPE_ERR -3

class rxUnit
{
    public:
    	rxUnit():m_size(0),m_buf(0){ m_size = 0; m_buf = new unsigned char[HID_INPUT_MAX_LEN];}
    	~rxUnit() { if (m_buf != 0x0) delete m_buf; }

    	unsigned char* setBuf(unsigned char* buf, int size) { m_buf = new unsigned char[size]; memcpy(m_buf, buf, size); return m_buf; }
    	void clearBuf() { memset(m_buf, 0x00, HID_INPUT_MAX_LEN);}
    	int setSize(int size) { m_size = size; return m_size; }

    	int getSize() { return m_size; }
    	unsigned char* getBuf() { return m_buf; }

    private:
        rxUnit(rxUnit&);
        rxUnit & operator = (rxUnit&);
    	int m_size;
    	unsigned char* m_buf;
};


#endif // PACKET_H