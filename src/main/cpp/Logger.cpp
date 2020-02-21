#include "Logger.h"

Logger::Logger(const char *path, bool console_echo){
    m_fd = fopen(path, "a");
    m_timer = new Timer();
    m_console_echo = console_echo;
    // if(m_fd == nullptr)
    // TBD how to handle failure in constructor???
}

Logger::~Logger(){
    fclose(m_fd);
}

void Logger::logMsg(int level, const char *msg){
    char *log_str;
    float timestamp = m_timer->GetFPGATimestamp();

    fprintf(m_fd, "%9.4f l%u: %s\n", timestamp, level, msg);
    if (m_console_echo)
        printf("%9.4f l%u: %s\n", timestamp, level, msg);

    //fputs(msg, m_fd);
    //fputs("\n", m_fd);
}

