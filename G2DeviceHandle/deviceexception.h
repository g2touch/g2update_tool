#ifndef G2DEVICEEXCEPTION_H_
#define G2DEVICEEXCEPTION_H_
#ifdef USE_EXCEPTION
#include <exception>
#include <string>

class G2DeviceException : public std::exception
{
    public:
    	G2DeviceException(const std::string & errMsg, const unsigned int errCode) :
    		std::exception(), m_msg(errMsg), m_errCode(errCode) {}

    	G2DeviceException(const std::string & errMsg) :
    		std::exception(), m_msg(errMsg), m_errCode(0) {}

    	virtual ~G2DeviceException() throw() {}

    	virtual unsigned int getErrCode() const
    	{
    		return m_errCode;
    	}

    	const char* what() const throw()
    	{
    		return m_msg.c_str();
    	}

    private:
    	G2DeviceException() : m_errCode(0)
    	{}

    	std::string m_msg;
    	unsigned int m_errCode;
};
#endif
#endif
