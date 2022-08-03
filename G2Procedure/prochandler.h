#ifndef PROCHANDLER_H
#define PROCHANDLER_H

#include <string>
#include "gendevicehandle.h"

using namespace std;
using namespace ARG;
using namespace G2::DeviceIO;

namespace G2
{
    namespace PROC
    {
        class CProcHandler
        {
            public:
                CProcHandler();
                ~CProcHandler();

                bool LoadBinary(CArgHandler *devHandler);
                int DoUpdate(CDeviceHandler *devHandler);
                bool ChkFwVer(CDeviceHandler *devHandler);
                bool CheckBinary(unsigned char* m_bufBinary);

            private:
                string m_fnameLoaded;
                int m_bufSize;
                unsigned char m_bufBinary[0x20000+100];
        };
    }
}
#endif
