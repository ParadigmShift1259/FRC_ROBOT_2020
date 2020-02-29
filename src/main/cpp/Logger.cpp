/*
    Logging Class
    2/15/20

    Nicholas Seidl
    
*/

#include "Logger.h"

Logger::Logger(const char *path, bool console_echo)
    : m_fd(nullptr)
    , m_path(path)
{
    m_console_echo = console_echo;
    m_formattedData.reserve(500);
}

void Logger::openLog(const char *path)
{
    printf("Opening log %s\n", path);
    if (m_fd != nullptr)
    {
        printf("Log was open, closing\n");
        closeLog();
    }

    m_fd = fopen(m_path.c_str(), "a");
    printf("Log file descriptor 0x%p\n", (void*)m_fd);
    // CSV header for free form text logging
    if (m_fd != nullptr)
        fprintf(m_fd, "Level,Function,Line,Message\n");

    printf("Level,Function,Line,Message\n");
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

void Logger::logMsg(ELogLevel level, const char* func, const int line, const char* msg)
{
    if (m_fd == nullptr)
    {
        openLog(m_path.c_str());
    }

    if (m_fd != nullptr)
    {
        float timestamp = m_timer.GetFPGATimestamp();

        fprintf(m_fd, "%9.4f,l%u,%s,%d,%s\n", timestamp, level, func, line, msg);
        if (m_console_echo)
            printf("%9.4f,l%u,%s,%d,%s\n", timestamp, level, func, line, msg);
    }
}

void Logger::logData(const char* func, const int line, const vector<double*>& data)
{
    formatData(data);
    logMsg(eInfo, func, line, m_formattedData.c_str());
}

void Logger::logData(const char* func, const int line, const vector<int*>& data)
{
    formatData(data);
    logMsg(eInfo, func, line, m_formattedData.c_str());
}

void Logger::logData(const char* func, const int line, const vector<int*>& dataInt, const vector<double*>& dataDouble)
{
    formatData(dataInt);
    string temp = m_formattedData;
    formatData(dataDouble);
    logMsg(eInfo, func, line, (temp + "," + m_formattedData).c_str());
}

void Logger::formatData(const vector<double*>& data)
{
    constexpr int c_charsPerDouble = 6;
    const size_t sz = data.size();
    m_formattedData.resize(c_charsPerDouble * sz + 1);
    char* out = const_cast<char*>(m_formattedData.c_str());

    for (size_t i = 0; i < sz; i++)
    {
        out += sprintf(out, "%.3f%s", *data[i], i == sz - 1 ? "" : ",");
    }
}

void Logger::formatData(const vector<int*>& data)
{
    constexpr int c_charsPerInt = 7;
    const size_t sz = data.size();
    m_formattedData.resize(c_charsPerInt * sz + 1);
    char* out = const_cast<char*>(m_formattedData.c_str());

    for (size_t i = 0; i < sz; i++)
    {
        out += sprintf(out, "%6d%s", *data[i], i == sz - 1 ? "" : ",");
    }
}
