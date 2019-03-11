#ifndef PACKET_H
#define PACKET_H

#include <string.h>

#if defined(USE_HID_USB)
#define HID_OUTPUT_MAX_LEN  (64)
#define HID_OUT_REPORT_ID   (0x80)
#else
#define HID_OUTPUT_MAX_LEN  (59)
#define HID_OUT_REPORT_ID   (0x0A)
#endif
#define HID_INPUT_MAX_LEN   (64)

#define ONE_MILLION 1000000
#define READ_SIZE  64
#define ONE_PACKET_MAXBODY  52
#define ONE_PACKET_EXCEPT_BODY  7
#define ONE_EXCEPT_PACKET 9

#define DUMP_PACKET_EXCEPT_BODY 16

#define NSECTOSEC 1000000000
#define REPORT_ID_INDEX_SIZE 2

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