#pragma once

/*
 *
 * TelnetLogger library by steve@meisners.net
 * "Forked" from the ParticleWebLog library;
*/
/* ParticleWebLog library by kevinh@geeksville.com
 */

// This will load the definition for common Particle variable types
#include "Particle.h"

#define RECALL_BUFFER_LEN 2044

/// LogHandler that send logs to Papertrail (https://papertrailapp.com/). Before using this class it's best to
/// familiarize yourself with Particle's logging facility https://docs.particle.io/reference/firmware/photon/#logging.
/// You can use this as any other LogHandler - Initialize this class as a global, then call Log.info() and friends.
class TelnetLogger : public LogHandler {
    String m_name;
    bool m_enable;
    TCPClient& m_telnet;
    LogLevel m_LogLevel;
    int m_publishing; // a counting semaphore to prevent recursion

    //
    // Support for event log buffer recall
    //
    int m_RecallPosition = 0;
    // Allocate 4 extra bytes to allow for a safety buffer
    char RecallBuffer[RECALL_BUFFER_LEN+4];

public:
/// Initialize the log handler.
/// \param telnet Instantiation of telnet client
/// \param name Name of the events we publish
/// \param level Default log level.
/// \param filters Category filters.
///
/// Each log entry uses RFC 5424 with the following format:
/// "<22>1 %ISO8601_TIME% %system% %app% - - - [%category%] %log_level%: %text".
    explicit TelnetLogger(TCPClient& telnet, String name = "log",
                            LogLevel level = LOG_LEVEL_INFO, const LogCategoryFilters &filters = {
    });
    virtual ~TelnetLogger();
    bool RecallBufferAdd(const char* message);
    void RecallBufferPrint();
    void enable();
    void disable();
    TelnetLogger *setLevel(LogLevel newLevel);
    LogLevel getLevel() {return m_LogLevel;}

private:
    void log(const char *category, String message);

protected:
    virtual void logMessage(const char *msg, LogLevel level, const char *category, const LogAttributes &attr) override;
};
