
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
        m_devPath(""),
        m_VID(0),
        m_PID(0),
        m_hidrawNode(""),
	m_bFWVerReceived(false),
	m_sFwVersion("")
{
	m_devPath = argHandle->GetInterfaceResolved();
}

CDeviceHandler::~CDeviceHandler()
{

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

bool CDeviceHandler::G2Update(unsigned char* file_buf, int filesize)
{
    //int iRet = 1;
    int nBootUpdate_finish = 0;
    int nCUUpdate_finish = 0;
    int nFWUpdate_finish = 0;
    bool bRequestPartition = false;
    bool Partition_inTarget = false;
    int bPrechecking = 0;
    int nRet = 0x00;
    int nPrechecking = 0;
    int trynum = 0;
    
    //HW Reset
    bPrechecking = TxRequestHW_Reset();
    if (bPrechecking <= 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestHW_Reset error");
        return false;
    }
    usleep(150000);

    nPrechecking = FindBootVerandPartitionTable(file_buf, filesize);
    if(nPrechecking == 0xff)        
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "Cannot find BootVer");
        return false;
    }
    else if(nPrechecking == 0x00)
    {
        bRequestPartition = false;
    }
    else
    {
        bRequestPartition = true;
    }

    bPrechecking = FindFWFeature(file_buf, filesize);
    if(bPrechecking < 0)        
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "Not matched MCU with file old FW");
        //return false;
    }

    if(bRequestPartition == true)
    {
        Partition_inTarget = RequestPartitionInfo(bRequestPartition);
        if(Partition_inTarget == false)        
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "Not matched Target with file");
            //return false;
        }
    }
    
    //check prepare condition
    nPrechecking = Precheckforupdate(file_buf, m_bBootUpdateforce, bRequestPartition);
    if(nPrechecking < 0)
    {
        LOG_G2_E(CLog::getLogOwner(), TAG, "Update Condition Error");
        return false;
    }    

    LOG_G2_D(CLog::getLogOwner(), TAG, "Precheckforupdate Finish");

    //boot update
    for(trynum = 0; trynum < 3; trynum++)
    {
        nBootUpdate_finish = TxRequestBootUpdate(file_buf, m_bBootUpdateforce, bRequestPartition);
        if(nBootUpdate_finish <= 0)
        {
    		m_bBootUpdateforce = true;
    		if(trynum == 2)
    		{
    			LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestBootUpdate Error");
    			return false;
    		}
            else
            {
                continue;
            }
    	}
        else
        {
            break;
        }
    
    }
	usleep(500000);

    for(trynum = 0; trynum < 3; trynum++)
    {    
        nRet = HWReset();
        if(nRet <= 0)        
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "Fail to HWReset");
            continue;
        }   
        
        nRet = GoToBoot();
        if(nRet <= 0)        
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "Fail to GoToBoot");
            continue;
        }
        
        if(trynum == 2)        
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "Fail to GoToBoot & HWReset");        
            return false;
        }

        //success
        break;
    }

    //get new bootloader partition after Bootwrite
    if((bRequestPartition == true) && (nBootUpdate_finish == BOOTWRITEFINISH))
    {
        LOG_G2_D(CLog::getLogOwner(), TAG, "Get new Partition table");
    
        bRequestPartition = RequestPartitionInfo(bRequestPartition);
        if(bRequestPartition == false)        
        {
            LOG_G2_D(CLog::getLogOwner(), TAG, "Not matched MCU with file");
            return false;
        }
    }


    for(trynum = 0; trynum < 3; trynum++)
    {

        nCUUpdate_finish = TxRequestCuUpdate(file_buf, bRequestPartition);
        if(nCUUpdate_finish <= 0)
        {
    		if(trynum == 2)
    		{
                LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestCuUpdate Error");
                return false;
    		}
            else
            {
                continue;
            }
        }

        nFWUpdate_finish = TxRequestFwUpdate(file_buf, bRequestPartition);

        if(nFWUpdate_finish <= 0)
        {
    		if(trynum == 2)
    		{
                LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestFWUpdate Error");
                return false;
    		}
            else
            {
                continue;
            }
        }

        break;
    }

    LOG_G2_D(CLog::getLogOwner(), TAG, "nBootUpdate_finish %d", nBootUpdate_finish);
    if(nBootUpdate_finish == BOOTRUNNING) //stay in Bootloader When boot updated
    {
        //boot update
        for(trynum = 0; trynum < 3; trynum++)
        {
            nBootUpdate_finish = TxRequestBootUpdate(file_buf, m_bBootUpdateforce, bRequestPartition);
            if(nBootUpdate_finish <= 0)
            {
        		m_bBootUpdateforce = true;
        		if(trynum == 2)
        		{
        			LOG_G2_E(CLog::getLogOwner(), TAG, "TxRequestBootUpdate Error");
        			return false;
        		}
                else
                {
                    continue;
                }
        	}
            else
            {
                break;
            }
        }    
        usleep(1500000);
       
        TxRequestSystem_Reset();
    }

    usleep(800000);

    m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);

    if(m_sFwVersion == "")
    {
        usleep(500000);    
        m_sFwVersion = TxRequestFwVer(1000, m_bVerHex);
        if(m_sFwVersion == "")
        {
            LOG_G2(CLog::getLogOwner(), TAG, "Updated FW Version Get Fail");
        }
    }

    if(!(m_sFwVersion == ""))
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
        return true;
    }

    return false;
}

unsigned short CDeviceHandler::getPID()
{
    return m_PID;
}


