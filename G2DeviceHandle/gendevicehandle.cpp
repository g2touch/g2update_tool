
#include "gendevicehandle.h"
#include "logmanager.h"
#include "shellcommand.h"
#include "arghandler.h"

/* Linux */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

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
#include <fstream>

#ifndef HIDIOCSFEATURE
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

#define TAG "DeviceHandler"

using namespace std;
using namespace ARG;
using namespace G2;
using namespace G2::DeviceIO;

/* G2 device node name */
#define G2_TOUCH_USB_HIDRAW "/dev/g2touch_touch_usb_hidraw"
#define G2_TOUCH_I2C_HIDRAW "/dev/g2touch_touch_i2c_hidraw"

CDeviceHandler::CDeviceHandler(CArgHandler *argHandle) :
	DeviceIO_hid_over_i2c(argHandle),
        m_bBootUpdateforce(false),
        m_bVerHex(false),
        log_path(""),
        Hidraw_Num(0),
        m_selUpdate(0),
	    m_confirmUpdateFile(0),
        m_devPath(""),
        m_devOpenPath(""),
        m_VID(0),
        m_PID(0),
        m_hidrawNode(""),
	m_bFWVerReceived(false),
	m_sFwVersion(""),
	m_bFinishFWDonload(false)
{
	m_devPath = argHandle->GetInterfaceResolved();
}

CDeviceHandler::~CDeviceHandler()
{

}

bool CDeviceHandler::findInterface(string hidrawN)
{
    string deviceID;

    string hidrawStr = "hidraw";
    string hidrawNum = hidrawN;
    if (hidrawNum.substr(0, hidrawStr.size()) == hidrawStr)
    {
    	string cmd_ls_sys_class_hidraw_hidrawN = "ls -l /sys/class/hidraw/";
    	cmd_ls_sys_class_hidraw_hidrawN.append(hidrawNum);
    	std::string ls_sys_class_hidraw_hidrawN = ShellCommand::exec(cmd_ls_sys_class_hidraw_hidrawN.c_str());

    	string keywordStr1 = "-> ../../";
    	string keywordStr2 = "/hidraw/hidraw";
    	if (ls_sys_class_hidraw_hidrawN.find(keywordStr1) != string::npos &&
    		ls_sys_class_hidraw_hidrawN.find(keywordStr2) != string::npos)
    	{
    		string tmpDeviceID = ls_sys_class_hidraw_hidrawN;
    		tmpDeviceID = tmpDeviceID.substr(tmpDeviceID.find(keywordStr1) + keywordStr1.size(), tmpDeviceID.size());
    		tmpDeviceID = tmpDeviceID.substr(0, tmpDeviceID.find(keywordStr2));
    		tmpDeviceID = tmpDeviceID.substr(0, tmpDeviceID.find_last_of("/"));

    		deviceID = "/sys/";
    		deviceID.append(tmpDeviceID);
    	}
    }
    else
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "Cannot find hidraw ", m_devOpenPath.c_str());
        return false;
    }

    m_devOpenPath = deviceID;
    LOG_G2_D(CLog::getLogOwner(), TAG, "set deviceID : \"%s\"", m_devOpenPath.c_str());

    return true;
}

bool CDeviceHandler::openDevice()
{
    if(detectByDeviceNode(m_devPath))
    {

    }
    else if (detectByHidRawName(m_devPath) == false)
    {
    	LOG_G2_D(CLog::getLogOwner(), TAG, "cannot detect device with Resolved Path : %s", m_devPath.c_str());
    	return false;
    }

    int fd = DeviceIO_hid_over_i2c::openDevice(m_hidrawNode);

    if (fd < 0)
    {
    	LOG_G2_E(CLog::getLogOwner(), TAG, "cannot open device (raw name: %s)", m_hidrawNode.c_str());
    	return false;
    }

    return true;
}

bool CDeviceHandler::findHidrawNum()
{
    string deviceID;
    string tempHidrawNum;
    string hidrawN = "hidraw";
    int Num;
    bool foundDev = false;

    vector<string> Max_Hidraw = getHidrawNum();

    while (!Max_Hidraw.empty())
    {
        hidrawN = Max_Hidraw.back();
        foundDev=reopenDevice(hidrawN);
        if(foundDev == false)
        {
            Max_Hidraw.pop_back();
            LOG_G2_D(CLog::getLogOwner(), TAG, "%s Open fail",hidrawN.c_str());
            continue;
        }
        else
        {
            if(m_PID != 1)
            m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);
            if(m_sFwVersion != "" || (m_PID == 1))
            {
                LOG_G2_D(CLog::getLogOwner(), TAG, "Found hidraw");
                break;
            }
            Max_Hidraw.pop_back();
        }
    }

    if(!Max_Hidraw.empty())
    {
        Max_Hidraw.clear();
    }

    return true;
}

vector<string> CDeviceHandler::getHidrawNum()
{
    std::string find_hidraw = " hidraw";
    std::string cmd_ls_device_count_hidraw = "ls -l /sys/class/hidraw/ |grep 2A94:";
    std::string result = ShellCommand::exec(cmd_ls_device_count_hidraw.c_str());

    vector<string> v;

    string str = "";

    while(result.find(find_hidraw) != string::npos)
    {
        int pos = result.find(find_hidraw);
        str = result.substr(pos+1, 7);
        result = result.substr(pos+8, result.size());
        v.push_back(str);
        str.clear();
    }

    return v;
}



bool CDeviceHandler::reopenDevice(string hidrawStr)
{
    if(findInterface(hidrawStr) == false)
    {
    	LOG_G2_D(CLog::getLogOwner(), TAG, "cannot find hid device : %s", hidrawStr.c_str());
    	return false;
    }

    if (detectByHidRawName(m_devOpenPath) == false) /* detect "/dev/hidraw*" */
    {
    	LOG_G2_D(CLog::getLogOwner(), TAG, "cannot detect device with Resolved Path : %s", m_devOpenPath.c_str());
    	return false;
    }

    int fd = DeviceIO_hid_over_i2c::openDevice(m_hidrawNode);

    if (fd < 0)
    {
    	LOG_G2_E(CLog::getLogOwner(), TAG, "cannot open device (raw name: %s)", m_hidrawNode.c_str());
    	return false;
    }

    return true;
}

bool CDeviceHandler::detectByDeviceNode(std::string deviceName)
{
    if( deviceName.substr(0, deviceName.size()) == G2_TOUCH_USB_HIDRAW )
    {
        m_hidrawNode = deviceName;

        LOG_G2_I(CLog::getLogOwner(), TAG, "Available G2 touch\n");
        return true;
    }
    else if( deviceName.substr(0, deviceName.size()) == G2_TOUCH_I2C_HIDRAW )
    {
        m_hidrawNode = deviceName;

        LOG_G2_I(CLog::getLogOwner(), TAG, "Available G2 touch\n");
        return true;
    }

    return false;
}

bool CDeviceHandler::detectByHidRawName(string deviceName)
{
    std::string cmd_ls_device_name_hidraw = "ls ";
    cmd_ls_device_name_hidraw.append(deviceName);
    cmd_ls_device_name_hidraw.append("/*:*:*.*/hidraw/");
    LOG_G2_D(CLog::getLogOwner(), TAG, "cmd : %s", cmd_ls_device_name_hidraw.c_str());
    std::string hidrawNum = ShellCommand::exec(cmd_ls_device_name_hidraw.c_str());
    if (!hidrawNum.empty() && hidrawNum[hidrawNum.length() - 1] == '\n')
    {
    	hidrawNum.erase(hidrawNum.length() - 1);
    }
    LOG_G2_D(CLog::getLogOwner(), TAG, "output (hidrawNum) : %s", hidrawNum.c_str());

    if (hidrawNum.empty())
    {
    	LOG_G2_I(CLog::getLogOwner(), TAG, "No (available) g2touch panel : hidrawNum is empty\n");
    	return false;
    }

    m_hidrawNode = "/dev/";
    m_hidrawNode.append(hidrawNum);
    bool bFoundDevice = false;
    if (m_hidrawNode.find("/dev/hidraw") == 0)
    {
    	if (getHidInfo(m_hidrawNode.c_str()))
    	{
    		if (m_VID == G2TOUCH_VID)
    		{
    			bFoundDevice = true;
    		}
    		else
    		{
    			bFoundDevice = false;
    		}
    	}
    	else
    	{
    		bFoundDevice = false;
    	}
    }
    else
    {
    	bFoundDevice = false;
    }

    if (bFoundDevice == false)
    {
    	LOG_G2_I(CLog::getLogOwner(), TAG, "Not (available) g2touch panel\n");
    	return false;
    }

    return true;
}

bool CDeviceHandler::getHidInfo(const char* deviceName)
{
	int fd;
	int res = 0;
	char buf[256];
	struct hidraw_report_descriptor rpt_desc;
	struct hidraw_devinfo info;

	LOG_G2_D(CLog::getLogOwner(), TAG, "getHidInfo: %s", deviceName);

	fd = open(deviceName, O_RDWR | O_NONBLOCK);

	if (fd < 0)
	{
		LOG_G2_E(CLog::getLogOwner(), TAG, "Unable to open device, errno=%d (%s)", errno, strerror(errno));
		return false;
	}

	memset(&rpt_desc, 0x0, sizeof(rpt_desc));
	memset(&info, 0x0, sizeof(info));
	memset(buf, 0x0, sizeof(buf));

	res = ioctl(fd, HIDIOCGRAWNAME(256), buf);
	if (res < 0)
	{
		LOG_G2_E(CLog::getLogOwner(), TAG, "HIDIOCGRAWNAME, errno=%d (%s)", errno, strerror(errno));
	}
	else
	{
		LOG_G2_D(CLog::getLogOwner(), TAG, "Raw Name: %s", buf);

		std::string rawName(buf);
	}

	res = ioctl(fd, HIDIOCGRAWINFO, &info);
	if (res < 0)
	{
		LOG_G2_E(CLog::getLogOwner(), TAG, "HIDIOCGRAWINFO, errno=%d (%s)", errno, strerror(errno));
	}
	else
	{
		LOG_G2_D(CLog::getLogOwner(), TAG, "Raw Info:");
		LOG_G2_D(CLog::getLogOwner(), TAG, "\tbustype: %d", info.bustype);
		LOG_G2_D(CLog::getLogOwner(), TAG, "\tvendor: 0x%04hx", info.vendor);
		LOG_G2_D(CLog::getLogOwner(), TAG, "\tproduct: 0x%04hx", info.product);

        m_VID = info.vendor;
        m_PID = info.product;

        if(info.bustype == BUS_I2C)
        {
            hid_Type = TYPE_I2C;
            HID_OUTPUT_MAX_LEN = HID_OUTPUT_MAX_LEN_I2C;
            HID_OUT_REPORT_ID = HID_OUT_REPORT_ID_I2C;
            LOG_G2_D(CLog::getLogOwner(), TAG, "bustype: BUS_I2C");
        }
        else if(info.bustype == BUS_USB)
        {
            hid_Type = TYPE_USB;
            HID_OUTPUT_MAX_LEN = HID_OUTPUT_MAX_LEN_USB;
            HID_OUT_REPORT_ID = HID_OUT_REPORT_ID_USB;
            LOG_G2_D(CLog::getLogOwner(), TAG, "bustype: BUS_USB");
        }
        else
        {
            hid_Type = TYPE_ERR;
            LOG_G2_D(CLog::getLogOwner(), TAG, "bustype: ?");
            return false;
        }
	}

	close(fd);
	return true;
}

bool CDeviceHandler::IsDeviceOpened()
{
	return DeviceIO_hid_over_i2c::isDeviceOpened();
}

bool CDeviceHandler::CheckFirmwareVersion(int v_format)
{
    m_bFWVerReceived = false;

    m_sFwVersion = TxRequestFwVer(1000, v_format);
    m_bFWVerReceived = (m_sFwVersion != "");

    if(v_format == 0 && m_bFWVerReceived != 0)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "G2TOUCH Touch Firmware Version : %s", m_sFwVersion.c_str());
    }
    else if(v_format == 1 && m_bFWVerReceived != 0)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "(HEX)G2TOUCH Touch Firmware Version : %s", m_sFwVersion.c_str());
    }
    else
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Check FW version Failed");
    }

    return m_bFWVerReceived;
}

bool CDeviceHandler::G2Update(unsigned char* file_buf)
{
    int iRet = 0;
    int nTry = 0;
    int nBootUpdate_finish = false;
    int nCUUpdate_finish = false;
    int nFWUpdate_finish = false;

    TxRequestHW_Reset(false, 1);
    ReadDataAll(2000);
    nBootUpdate_finish = TxRequestBootUpdate(file_buf, m_bBootUpdateforce);

    if(nBootUpdate_finish <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestBootUpdate Error");
        return iRet;
    }

    nTry = 0;
    while(nTry++ < 3)
    {
        nCUUpdate_finish = TxRequestCuUpdate(file_buf);

        if(nCUUpdate_finish <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestCuUpdate Error");
            if(hid_Type == TYPE_USB)
            {
                usleep(1000000);
                findHidrawNum();
                nCUUpdate_finish = TxRequestCuUpdate(file_buf);
            }
            if(nCUUpdate_finish <= 0)
                return iRet;
        }

        nFWUpdate_finish = TxRequestFwUpdate(file_buf);

        if(nFWUpdate_finish > 0)
        {
            break;
        }
        else if(nFWUpdate_finish <= 0)
        {
            LOG_G2_E(CLog::getLogOwner(), TAG, "nFWUpdate_finish Error");
        }
    }

    usleep(1500000);

    if(nBootUpdate_finish == 2)
    {
        findHidrawNum();
        nBootUpdate_finish = TxRequestBootUpdate(file_buf, m_bBootUpdateforce);
        TxRequestSystem_Reset();
        usleep(2000000);
    }

    if(findHidrawNum() == false)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "FW Update Fail : Device Cannot find");
        nFWUpdate_finish = 0;
    }

    m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);

    if(m_sFwVersion == "")
    {
        if(hid_Type == TYPE_USB)
        {
            usleep(3000000);
            findHidrawNum();
            m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);
            TxRequestSystem_Reset();
            if(m_sFwVersion == "")
            {
                LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Version Get Fail");
            }
            else
            {
                if(m_bVerHex)
                {
                    LOG_G2(CLog::getLogOwner(), TAG, "(HEX)Updated FW Ver : %s",m_sFwVersion.c_str());
                }
                else
                {
                    LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Ver : %s",m_sFwVersion.c_str());
                }
            }
        }
        else
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Version Get Fail");
        }
    }
    else
    {
        if(m_bVerHex)
        {
            LOG_G2(CLog::getLogOwner(), TAG, "(HEX)Updated FW Ver : %s",m_sFwVersion.c_str());
        }
        else
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Ver : %s",m_sFwVersion.c_str());
        }
    }

    if((nBootUpdate_finish == 1)  && (nCUUpdate_finish == 1) && (nFWUpdate_finish == 1))
    {
        SaveHistory(true);
        return true;
    }

    SaveHistory(false);
    return false;
}

bool CDeviceHandler::G2UpdateFW(unsigned char* file_buf, int size)
{
    int iRet = 0;
    int nFWUpdate_finish = false;

    nFWUpdate_finish = TxReqOnlyFwUpdate(file_buf, size);

    if(nFWUpdate_finish <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestFWUpdate Error");
        return iRet;
    }

    usleep(1500000);
    if(findHidrawNum() == false)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "FW Update Fail : Device Cannot find");
        nFWUpdate_finish = 0;
    }

    m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);

    if(m_sFwVersion == "")
    {
        LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Version Get Fail");
    }
    else
    {
        if(m_bVerHex)
        {
            LOG_G2(CLog::getLogOwner(), TAG, "(HEX)Updated FW Ver : %s",m_sFwVersion.c_str());
        }
        else
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Ver : %s",m_sFwVersion.c_str());
        }
    }

    if(nFWUpdate_finish == 1)
    {
        return true;
    }

    return false;
}

bool CDeviceHandler::G2UpdateCU(unsigned char* file_buf)
{
    int iRet = 0;
    int nCUUpdate_finish = false;

    nCUUpdate_finish = TxReqOnlyCuUpdate(file_buf);

    if(nCUUpdate_finish <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestCuUpdate Error");
        return iRet;
    }
    TxRequestSystem_Reset();
    usleep(2000000);

    if(findHidrawNum() == false)
    {
        LOG_G2(CLog::getLogOwner(), TAG, "FW Update Fail : Device Cannot find");
    }

    m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);

    if(m_sFwVersion == "")
    {
        LOG_G2(CLog::getLogOwner(), TAG, "CU update fail");
    }
    else
    {
        if(m_bVerHex)
        {
            LOG_G2(CLog::getLogOwner(), TAG, "(HEX)Updated FW Ver : %s",m_sFwVersion.c_str());
        }
        else
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Ver : %s",m_sFwVersion.c_str());
        }
    }

    if(nCUUpdate_finish == 1)
    {
        return true;
    }

    return false;
}

void CDeviceHandler::SaveHistory(int nFinalResultCode)
{
    string datetime = GetDateTimeString();
    string historyfname = log_path + "/history.txt";
    fstream fs;

    fs.open(historyfname.c_str(), std::fstream::out | std::fstream::app | std::fstream::ate);

    fs << " " << datetime.substr(0, 2) << "/" << datetime.substr(2, 2) << " "
       << datetime.substr(5, 2) << ":" << datetime.substr(7, 2) << ":" << datetime.substr(9, 2);
    fs << "   " << ((nFinalResultCode == true) ? "PASS" : "FAIL") << endl;

    fs.close();
}

string CDeviceHandler::GetDateTimeString()
{
    time_t rawtime;
    struct tm *timeinfo;
    char buf[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buf, 80, "%m%d_%H%M%S", timeinfo);

    return string(buf);
}

bool CDeviceHandler::CheckAndCreate(string folder)
{
    struct stat sb;
    bool bFound = false;
    bool isdir = stat(folder.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
    size_t loc = 0;
    string sub;

    if (folder.length() <= 0) return false;
    while (isdir == false)
    {
        if (loc >= folder.length()) break;

        loc = folder.find("/", loc);
        if (loc == string::npos)
        {
            loc = folder.length();
        }
        if (loc == 0)
        {
            ++loc;
            continue;
        }

        sub = folder.substr(0, loc++);
        bFound = stat(sub.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
        if (bFound)
        {
            continue;
        }
        else
        {
            mkdir(sub.c_str(), S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH);
            bFound = stat(sub.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode);
        }
        if (loc >= folder.length()) isdir = bFound;
    }

    return isdir;
}
