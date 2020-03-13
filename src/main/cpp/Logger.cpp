/*
    Logging Class
    2/15/20
    Nicholas Seidl
    
*/

#include "Logger.h"

Logger::Logger(const char *path, bool console_echo)
    : m_fd(nullptr)
    , m_path(path)
    , m_console_echo(console_echo)
{
    m_formattedIntData.reserve(500);
    m_formattedDoubleData.reserve(500);
}

void Logger::openLog()
{
    printf("Opening log %s\n", m_path.c_str());
    if (m_fd != nullptr)
    {
        printf("Log was open, closing\n");
        closeLog();
    }

    m_fd = fopen(m_path.c_str(), "a");
    // CSV header for free form text logging
    if (m_fd != nullptr)
        fprintf(m_fd, "Timestamp,Level,Function,Line,Message\n");

    if (m_console_echo)
        printf("Timestamp,Level,Function,Line,Message\n");
}

Logger::~Logger()
{
    closeLog();
}

void Logger::closeLog()
{
    if (m_fd != nullptr)
    {
        printf("Closing log\n");
        fclose(m_fd);
        m_fd = nullptr;
    }
    else
    {
        printf("No log was open\n");
    }
    
}

void Logger::logMsg(ELogLevel level, const char* func, const int line, const char* msg, const char* msg2 /* = nullptr */)
{
    if (m_fd == nullptr)
    {
        openLog();
    }

    if (m_fd != nullptr)
    {
        float timestamp = m_timer.GetFPGATimestamp();
        if (msg2 == nullptr)
        {
            fprintf(m_fd, "%.6f,%u,%s,%d,%s\n", timestamp, level, func, line, msg);
        }
        else
        {
            fprintf(m_fd, "%.6f,%u,%s,%d,%s,%s\n", timestamp, level, func, line, msg, msg2);
        }
        
        if (m_console_echo)
        {
            if (msg2 == nullptr)
            {
                printf("%.6f,%u,%s,%d,%s\n", timestamp, level, func, line, msg);
            }
            else
            {
                printf("%.6f,%u,%s,%d,%s,%s\n", timestamp, level, func, line, msg, msg2);
            }
        }
    }
}

void Logger::logData(const char* func, const int line, const vector<double*>& data)
{
    formatData(data);
    logMsg(eInfo, func, line, m_formattedDoubleData.c_str());
}

void Logger::logData(const char* func, const int line, const vector<int*>& data)
{
    formatData(data);
    logMsg(eInfo, func, line, m_formattedIntData.c_str());
}

void Logger::logData(const char* func, const int line, const vector<int*>& dataInt, const vector<double*>& dataDouble)
{
    formatData(dataInt);
    formatData(dataDouble);
    logMsg(eInfo, func, line, m_formattedIntData.c_str(), m_formattedDoubleData.c_str());
}

void Logger::formatData(const vector<double*>& data)
{
    constexpr int c_charsPerDouble = 6;
    const size_t sz = data.size();
    m_formattedDoubleData.resize(c_charsPerDouble * sz + 1);
    char* out = const_cast<char*>(m_formattedDoubleData.c_str());

    for (size_t i = 0; i < sz; i++)
    {
        out += sprintf(out, "%.3f%s", *data[i], i == sz - 1 ? "" : ",");
    }
}

void Logger::formatData(const vector<int*>& data)
{
    constexpr int c_charsPerInt = 7;
    const size_t sz = data.size();
    m_formattedIntData.resize(c_charsPerInt * sz + 1);
    char* out = const_cast<char*>(m_formattedIntData.c_str());

    for (size_t i = 0; i < sz; i++)
    {
        out += sprintf(out, "%6d%s", *data[i], i == sz - 1 ? "" : ",");
    }
}