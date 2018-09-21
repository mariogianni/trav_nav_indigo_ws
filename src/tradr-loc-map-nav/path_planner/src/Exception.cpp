#include "Exception.h"

Exception::Exception(const std::string& description) :
description_(description)
{
}

Exception::~Exception() throw ()  
{

}

const char* Exception::what() const throw () 
{
    return description_.c_str();
}

