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

using namespace std;
using namespace frc;

class Logger
{
    private:
    FILE *m_fd;
    Timer *m_timer;
    bool m_console_echo;

    public:
    Logger(const char *path, bool console_echo);
    ~Logger();
    void logMsg(int level, const char *msg);
};

#endif /* SRC_Logger_H_ */