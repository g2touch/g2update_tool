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
                bool reopenDevice(string hidrawStr);
                bool IsDeviceOpened();
                bool CheckFirmwareVersion(int v_format);
                bool G2Update(unsigned char* file_buf);
                bool findHidrawNum();
                bool findInterface(string hidRawN);
                int getHidrawCount();
                bool CheckAndCreate(string folder);

                bool m_bBootUpdateforce;
                bool m_bVerHex;
                string log_path;

            private:
                string m_devPath;		// have device path resolved
                string m_devOpenPath;
                short m_VID;
                short m_PID;

                bool detectByHidRawName(string devName);
                bool detectByDeviceNode(std::string deviceName);
                bool getHidInfo(const char* deviceName);
                void SaveHistory(int nFinalResultCode);
                string GetDateTimeString();
                string m_hidrawNode;
                bool m_bFWVerReceived;
                string m_sFwVersion;
                bool m_bFinishFWDonload;
        };
    }
}
#endif	// GENDEVICEHANDLE_H
