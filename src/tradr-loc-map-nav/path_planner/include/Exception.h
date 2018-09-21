#ifndef PP_EXCEPTION_H
#define PP_EXCEPTION_H

#include <exception>
#include <string>
#include <sstream>

class Exception: public std::exception
{
public:

    Exception(const std::string& description);
    virtual ~Exception() throw();

    // override of std::exception::what()
    virtual const char* what() const throw();

private:

    std::string description_;
};


#define DEFINE_EXCEPTION(DERIVED, BASE)					\
									\
	class DERIVED : public BASE					\
	{								\
		public:							\
			DERIVED(const std::string& msg) : BASE(msg){}	\
			virtual ~DERIVED() throw(){}			\
	};								\


#define STR(x) #x


#define THROW_EXCEPTION(EXCEPTION, MSG)										\
{														\
	std::stringstream ss;											\
	ss << STR(EXCEPTION) << "(" << MSG << ") launched in function \'" << __PRETTY_FUNCTION__ << "\' at line " << __LINE__ ; \
	throw EXCEPTION(ss.str());											\
}



#endif
