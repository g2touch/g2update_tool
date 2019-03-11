
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

    /* convert hidraw* to device path */
    string hidrawStr = "hidraw";
    string hidrawNum = hidrawN;
    if (hidrawNum.substr(0, hidrawStr.size()) == hidrawStr) // if startswith "hidraw"
    {
    	/* find ls_sys_class_hidraw_hidrawN */
    	string cmd_ls_sys_class_hidraw_hidrawN = "ls -l /sys/class/hidraw/"; // " ls -l /sys/class/hidraw/hidraw* "
    	cmd_ls_sys_class_hidraw_hidrawN.append(hidrawNum);
    	std::string ls_sys_class_hidraw_hidrawN = ShellCommand::exec(cmd_ls_sys_class_hidraw_hidrawN.c_str());

    	/* parse to deviceID */
    	string keywordStr1 = "-> ../../";
    	string keywordStr2 = "/hidraw/hidraw";
    	if (ls_sys_class_hidraw_hidrawN.find(keywordStr1) != string::npos &&
    		ls_sys_class_hidraw_hidrawN.find(keywordStr2) != string::npos)
    	{
    		string tmpDeviceID = ls_sys_class_hidraw_hidrawN;
    		tmpDeviceID = tmpDeviceID.substr(tmpDeviceID.find(keywordStr1) + keywordStr1.size(), tmpDeviceID.size());
    		tmpDeviceID = tmpDeviceID.substr(0, tmpDeviceID.find(keywordStr2));
    		tmpDeviceID = tmpDeviceID.substr(0, tmpDeviceID.find_last_of("/")); // remove device (0000:0000:0000.0000)

    		deviceID = "/sys/";
    		deviceID.append(tmpDeviceID);
    	}
    }
    else
    {
         return false;
    }

    m_devOpenPath = deviceID;
    LOG_G2_D(CLog::getLogOwner(), TAG, "set deviceID : \"%s\"", m_devOpenPath.c_str());

    return true;
}

bool CDeviceHandler::openDevice()
{
    // find which device
    if(detectByDeviceNode(m_devPath)) /* detect "/dev/G2_*" */
    {

    }
    else if (detectByHidRawName(m_devPath) == false) /* detect "/dev/hidraw*" */
    {
    	LOG_G2_D(CLog::getLogOwner(), TAG, "cannot detect device with Resolved Path : %s", m_devPath.c_str());
    	return false;
    }

    // actual open file
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
    string Num;
    int i=0;
    int Max_Hidraw = 0;
    bool foundDev = false;

    Max_Hidraw = getHidrawCount();

    for(i=0;i< Max_Hidraw;i++)
    {
        hidrawN = "hidraw";
        /* parse to deviceID */
        Num = to_string(i);
        hidrawN.append(Num.c_str());

        // open with hidraw num
        foundDev=reopenDevice(hidrawN);
        if(foundDev == false) continue;
        //check debug hidraw
        if(m_PID != 1)
        m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);
        if(m_sFwVersion != "" || (m_PID == 1))
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "Found hidraw%d", i);
            break;
        }
    }

    return true;
}

int CDeviceHandler::getHidrawCount()
{
    /* find hidrawNum */
    int nHidrawCount = 0;
    std::string cmd_ls_device_count_hidraw = "ls /sys/class/hidraw/ | wc -l";
    std::string hidrawCount = ShellCommand::exec(cmd_ls_device_count_hidraw.c_str());
    nHidrawCount = atoi(hidrawCount.c_str());

    return nHidrawCount;
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

    // actual open file
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
    /* find hidrawNum */
    std::string cmd_ls_device_name_hidraw = "ls "; // " ls /sys/device/hidraw/ "
    cmd_ls_device_name_hidraw.append(deviceName);
    cmd_ls_device_name_hidraw.append("/*:*:*.*/hidraw/"); // append device (0000:0000:0000.0000)
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

    /* using hidrawNode */ // "/dev/hidrawX"
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

// input param : "/dev/hidrawX"
bool CDeviceHandler::getHidInfo(const char* deviceName)
{
	int fd;
	int res = 0;
	char buf[256];
	struct hidraw_report_descriptor rpt_desc;
	struct hidraw_devinfo info;

	LOG_G2_D(CLog::getLogOwner(), TAG, "getHidInfo: %s", deviceName);

	/* Open the Device with non-blocking reads. In real life,
	don't use a hard coded path; use libudev instead. */
	fd = open(deviceName, O_RDWR | O_NONBLOCK);

	if (fd < 0)
	{
		LOG_G2_E(CLog::getLogOwner(), TAG, "Unable to open device, errno=%d (%s)", errno, strerror(errno));
		return false;
	}

	memset(&rpt_desc, 0x0, sizeof(rpt_desc));
	memset(&info, 0x0, sizeof(info));
	memset(buf, 0x0, sizeof(buf));

	/* Get Raw Name */
	res = ioctl(fd, HIDIOCGRAWNAME(256), buf);
	if (res < 0)
	{
		LOG_G2_E(CLog::getLogOwner(), TAG, "HIDIOCGRAWNAME, errno=%d (%s)", errno, strerror(errno));
	}
	else
	{
		LOG_G2_D(CLog::getLogOwner(), TAG, "Raw Name: %s", buf);

		/* save raw name */
		std::string rawName(buf);
	}


	/* Get Raw Info */
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

		/* save vid/pid */
		m_VID = info.vendor;
		m_PID = info.product;
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
    m_bFWVerReceived = false;	// init

    // Try 5 times
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
            #if defined(USE_HID_USB)
            usleep(1000000);
            findHidrawNum();
            nCUUpdate_finish = TxRequestCuUpdate(file_buf);
            if(nCUUpdate_finish <= 0)
            #endif
            return iRet;
        }

        nFWUpdate_finish = TxRequestFwUpdate(file_buf);

        if(nFWUpdate_finish > 0)
        {
            //success
            break;
        }
        else if(nFWUpdate_finish <= 0)
        {
            //TryWriteData(CMD_MAIN_CMD, buf, buf_size, CMD_F1_RETRY_MAX, 1000);
            LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestFWUpdate Error");
        }
    }

#if defined(USE_HID_USB)
    if(nFWUpdate_finish == 1)
    {
        findHidrawNum();
        usleep(1000000);
        TxRequestSystem_Reset();
    }
#endif

    if(nBootUpdate_finish == 2) //stay in Bootloader When boot updated
    {
        usleep(1500000);
        nBootUpdate_finish = TxRequestBootUpdate(file_buf, m_bBootUpdateforce);
        TxRequestSystem_Reset();
    }

    usleep(800000);

    m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);

    if(m_sFwVersion == "")
    {
        #if defined(USE_HID_USB)
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
        #else
        LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Version Get Fail");
        #endif
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

/////////////////////////////////////////////////////////////////////////
// history log
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
        //if (folder.compare(sub) == 0) isdir = bFound;
        if (loc >= folder.length()) isdir = bFound;
    }

    return isdir;
}

