#ifndef SPEXCEPTION
#define SPEXCEPTION
#ifdef USE_EXCEPTION
#include <exception>
#include <string>
#include "proceduredllexportdefine.h"

namespace G2
{
    namespace PROC
    {
        class G2PROCEDURE_EXPORT SPException : public std::exception
        {
            public:
                enum SpCode
                {
                    SP_G2CORE_EXCEPTION = 0,
                    SP_SELF_EXCEPTION,
                };

                SPException(const std::string& errMsg) :
                    std::exception(),
                    m_msg(errMsg),
                    m_spCode(SP_G2CORE_EXCEPTION) {}

                SPException(const std::string& errMsg, SpCode spCode) :
                    std::exception(),
                    m_msg(errMsg),
                    m_spCode(spCode) {}

                virtual ~SPException() throw(){}

                const char* what() const throw()
                {
                    return m_msg.c_str();
                }

                SpCode spcode()
                {
                    return this->m_spCode;
                }

            protected:
                SPException(){}
                std::string m_msg;
                SpCode m_spCode;
        };

    } // Procedure
} // G2
#endif

#endif // SPEXCEPTION

