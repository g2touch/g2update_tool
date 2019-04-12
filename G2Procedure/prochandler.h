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
                enum G2UpdatePart
                {
                    UPDATE_ALL = 0,
                    UPDATE_CU = 1,
                    UPDATE_FW = 2,
                    UPDATE_CUFW = 3,
                };
                                
                CProcHandler();
                ~CProcHandler();

                bool LoadBinary(CArgHandler *devHandler);
                int DoUpdate(CDeviceHandler *devHandler);
                bool ChkFwVer(CDeviceHandler *devHandler);
                bool CheckBinary(unsigned char* m_bufBinary);
                bool CheckFWBinary(unsigned char* m_bufBinary);

            private:
                string m_fnameLoaded;
                int m_bufSize;
                short m_cnfirmFile;
                unsigned char m_bufBinary[0x20000+100];
        };
    }
}
#endif
