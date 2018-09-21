#ifndef LOGGER_FILE_H
#define LOGGER_FILE_H

#include <iostream>
#include <fstream>

///	\class LoggerFile
///	\author Luigi Freda
///	\brief A class implementing a logger which writes on a file (it is capable of intercepting std::endl)
///	\note
/// 	\todo 
///	\date
///	\warning
class LoggerFile
{
public:

    LoggerFile(const std::string &filename) : _filename(filename)
    {
        if (!filename.empty())
        {
            _ofile.open(filename.c_str(), std::fstream::out);
            if (!_ofile.is_open())
            {
                std::cout << "LoggerFile: ERROR: unable to open file" << filename << std::endl; 
            }
        }
        else
        {
            std::cout << "LoggerFile: ERROR: filename empty";
        }
    }

    ~LoggerFile()
    {
        if (_ofile.is_open())
        {
            _ofile.close();
        }
    }

    template <typename T>
    LoggerFile &operator<<(const T &a)
    {
        _ofile << a;
        return *this;
    }

    LoggerFile &operator<<(std::ostream& (*pf) (std::ostream&))
    {
        // here we intercept std::endl
        _ofile << pf;
        //_bFirst = false;
        return *this;
    }

    /// Writes the block of data pointed by s, with a size of n characters, into the output buffer
    void Write(const char* s, std::streamsize n)
    {
        _ofile.write(s, n);
    }

    void Clear()
    {
        _ofile.clear();
    }

protected:
    std::fstream _ofile;
    std::string _filename;
};


#endif // LOGGER_FILE_H

