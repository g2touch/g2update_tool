#ifndef PACKET_H
#define PACKET_H

#include <string.h>

#define HID_I2C_OUTPUT_MAX_LEN (64-1)

class rxUnit
{
    public:
    	rxUnit()  { m_size = 0; m_buf = new unsigned char[HID_I2C_OUTPUT_MAX_LEN];}
    	~rxUnit() { if (m_buf != 0x0) delete m_buf; }

    	unsigned char* setBuf(unsigned char* buf, int size) { m_buf = new unsigned char[size]; memcpy(m_buf, buf, size); return m_buf; }
    	void clearBuf() { memset(m_buf, 0x00, sizeof(m_buf));}
    	int setSize(int size) { m_size = size; return m_size; }

    	int getSize() { return m_size; }
    	unsigned char* getBuf() { return m_buf; }

    private:
    	int m_size;
    	unsigned char* m_buf;
};


#endif // PACKET_H