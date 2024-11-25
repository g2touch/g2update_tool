#ifndef GENDEVICEHANDLE_H
#define GENDEVICEHANDLE_H

#define G2TOUCH_VID		(0x2A94)

#include <string>
#include "arghandler.h"
#include "deviceio_hid_over_i2c.h"

using namespace std;
using namespace ARG;

/* size */
#define _64K            0x10000
#define _128K           0x20000
#define _256K           0x40000
#define _512K           0x80000

#define _4K             0x1000
#define _8K             0x2000
#define _12K            0x3000
#define _16K            0x4000


namespace G2
{
    namespace DeviceIO
    {
        class CDeviceHandler : public DeviceIO_hid_over_i2c
        {
            public:
                CDeviceHandler(CArgHandler *argHandle);
                ~CDeviceHandler();

                bool openDevice();
                bool IsDeviceOpened();
                bool CheckFirmwareVersion(int v_format);
                bool G2Update(unsigned char* file_buf, int filesize);
                unsigned short getPID();

                bool m_bBootUpdateforce;
                bool m_bVerHex;

            private:
                string m_devPath;       // have device path resolved
                short m_VID;
                short m_PID;

                bool detectByHidRawName(string devName);
                bool detectByDeviceNode(std::string deviceName);
                bool getHidInfo(const char* deviceName);
                string m_hidrawNode;

                bool m_bFWVerReceived;

                string m_sFwVersion;
        };
    }
}
#endif	// GENDEVICEHANDLE_H
