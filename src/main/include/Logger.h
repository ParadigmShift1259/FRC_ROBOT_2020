/*
    Logging Class
    2/15/20

    Nicholas Seidl
    
*/


#ifndef SRC_Logger_H_
#define SRC_Logger_H_

#include "Const.h"
#include <stdio.h>
#include <frc\base.h>
#include <frc\timer.h>

#include <vector>
#include <string>

enum ELogLevel
{
      eDebug
    , eInfo
    , eWarn
    , eError
};

//                                          1234567
constexpr char* c_doubleFmt = "%.3f%s";  // 3.141,2.718
constexpr char* c_intFmt = "%6d%s";      //     42,    17
constexpr int c_charsPerDouble = 6;
constexpr int c_charsPerInt = 7;

using namespace std;
using namespace frc;

class Logger
{
    FILE *m_fd;
    Timer *m_timer;
    bool m_console_echo;

   public:
    Logger(const char *path, bool console_echo);
    ~Logger();

    void logMsg(ELogLevel level, const char* func, const int line, const char* msg);
 
    void logData(const char* func, const int line, const vector<double*>& data)
    {
        logDataImpl<double>(func, line, m_dataFmtFloatingPoint, c_charsPerDouble, data);
    }

    void logData(const char* func, const int line, const vector<int*>& data)
    {
        logDataImpl<int>(func, line, m_dataFmtInteger, c_charsPerInt, data);
    }

protected:
    template <typename T>
    void logDataImpl(const char* func, const int line, const string& fmt, size_t charsPerDataElem, const vector<T*>& data)
    {
        const size_t sz = data.size();
        m_formattedData.resize(charsPerDataElem * sz + 1);
        char* out = const_cast<char*>(m_formattedData.c_str());

        for (size_t i = 0; i < sz; i++)
        {
            out += sprintf(out, fmt.c_str(), *data[i], i == sz - 1 ? "" : ",");
        }
        logMsg(eInfo, func, line, m_formattedData.c_str());
    }

    string m_dataFmtFloatingPoint;
    string m_dataFmtInteger;
    string m_formattedData;
};

#endif /* SRC_Logger_H_ */

/*

#include "stdafx.h"
#include "Logger.h"

int main()
{
       // https://gcc.gnu.org/onlinedocs/gcc/Function-Names.html
       // GCC supports __FUNCTION__ = sub
       // and          __PRETTY_FUNCTION__ = void a::sub(int)
       // Apparently __func__ is part of the C99 standard

       Logger log;
       log.LogMsg(eInfo, __FUNCTION__, __LINE__, "A freeform log statement");
       log.LogMsg(eWarn, __FUNCTION__, __LINE__, "Warning message");
       log.LogMsg(eError, __FUNCTION__, __LINE__, "Error message");
       log.LogMsg(eDebug, __FUNCTION__, __LINE__, "Debug message");

       double d1 = 3.1415;
       double d2 = 2.71828;
       double d3 = 42.0;
       double d4 = 0.12345;
       double d5 = 0.456735687;
       double d6 = 0.33333333333333333;
       double d7 = 2.34561345;
       vector<double*> data7double = { &d1, &d2, &d3, &d4, &d5, &d6, &d7 };
       log.LogMsg(eInfo, __FUNCTION__, __LINE__, "Pi,e,The answer,Random,Typing,One third,Junk");
       log.LogData(__FUNCTION__, __LINE__, data7double);

       d1 = 0.12345;
       d2 = 0.456735687;
       d3 = 0.33521356;
       vector<double*> data3double = { &d1, &d2, &d3 };
       log.LogMsg(eInfo, __FUNCTION__, __LINE__, "Fric,Frac,The ohter one");
       log.LogData(__FUNCTION__, __LINE__, data3double);

       d1 = 1.1;
       d2 = 2.2;
       d3 = 3.3;
       d4 = 4.4;
       log.LogMsg(eInfo, __FUNCTION__, __LINE__, "Thing1,Thing2,The other one,Yet another");
       vector<double*> data4double = { &d1, &d2, &d3, &d4 };
       log.LogData(__FUNCTION__, __LINE__, data4double);

       int i1 = 3;
       int i2 = 2;
       int i3 = 42;
       int i4 = 17;
       int i5 = 39;
       vector<int*> data5int = { &i1, &i2, &i3, &i4, &i5 };
       log.LogMsg(eInfo, __FUNCTION__, __LINE__, "Int1,Int2,Int3,Int4,Int5");
       log.LogData(__FUNCTION__, __LINE__, data5int);
 
       int i1b = 9;
       int i2b = 2000;
       int i3b = 42000;
       log.LogMsg(eInfo, __FUNCTION__, __LINE__, "Count,RPM,Encoder");
       vector<int*> data3int = { &i1b, &i2b, &i3b };
       log.LogData(__FUNCTION__, __LINE__, data3int);

       return 0;
}

*/