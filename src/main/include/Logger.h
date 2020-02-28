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


using namespace std;
using namespace frc;

class Logger
{
    FILE *m_fd;
    Timer m_timer;
    bool m_console_echo;
    string m_path;

  public:
    Logger(const char *path, bool console_echo);
    ~Logger();

    void openLog(const char *path);
    void closeLog();

    void logMsg(ELogLevel level, const char* func, const int line, const char* msg);
    void logData(const char* func, const int line, const vector<double*>& data);
    void logData(const char* func, const int line, const vector<int*>& data);
    void logData(const char* func, const int line, const vector<int*>& dataInt, const vector<double*>& dataDouble);

  protected:
    void formatData(const vector<double*>& data);
    void formatData(const vector<int*>& data);
    string m_formattedData;
};

#endif /* SRC_Logger_H_ */
