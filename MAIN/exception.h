#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
#include <string>
#include "basedllexportdefine.h"

namespace G2
{
    class Base_EXPORT Exception : public std::exception
    {
        public:
            Exception(const std::string& errMsg) :
                std::exception(),
                m_msg(errMsg) {}

            virtual ~Exception() throw(){}

            const char* what() const throw()
            {
                return m_msg.c_str();
            }

        protected:
            Exception():
                m_msg(0){}
            std::string m_msg;
    };

} // G2

#endif // EXCEPTION_H
