#include "Logger.h"

Logger::Logger(const char *path, bool console_echo)
    : m_dataFmtFloatingPoint(c_doubleFmt)
    , m_dataFmtInteger(c_intFmt)
{
    m_fd = fopen(path, "a");
    m_timer = new Timer();
    m_console_echo = console_echo;
    m_formattedData.reserve(500);
    printf("Level,Function,Line,Message\n");
    // if(m_fd == nullptr)
    // TBD how to handle failure in constructor???
}

Logger::~Logger(){
    fclose(m_fd);
}

void Logger::logMsg(ELogLevel level, const char* func, const int line, const char* msg)
{
    float timestamp = m_timer->GetFPGATimestamp();

    fprintf(m_fd, "%9.4f,l%u,%s,%d,%s\n", timestamp, level, func, line, msg);
    if (m_console_echo)
        printf("%9.4f,l%u,%s,%d,%s\n", timestamp, level, func, line, msg);

    //fputs(msg, m_fd);
    //fputs("\n", m_fd);
}

