#include "TelnetLogger.h"


TelnetLogger::TelnetLogger(TCPClient& telnet, String name, LogLevel level,
                               const LogCategoryFilters &filters) : LogHandler(level, filters), m_name(name), m_telnet(telnet)
{
    LogManager::instance()->addHandler(this);
    m_LogLevel = level;
    memset (RecallBuffer, 0, RECALL_BUFFER_LEN);
    m_RecallPosition = 0;
    m_publishing = 0;
    m_enable = false;
}

TelnetLogger::~TelnetLogger() {
    LogManager::instance()->removeHandler(this);
}


void TelnetLogger::RecallBufferPrint()
{
    if (m_telnet.connected())
    {
        if (m_RecallPosition == 0)
        {
            m_telnet.println (F("Recall Buffer Empty!"));
        }
        else
        {
        //    m_telnet.printf("%s: %s\n", m_name.c_str(), message.c_str());
            m_telnet.println(F("=================================================="));
            m_telnet.println(F("===     Begin Event Log Buffer Dump            ==="));
            m_telnet.println(F("=================================================="));
            m_telnet.print(&RecallBuffer[m_RecallPosition+2]);
            m_telnet.print(RecallBuffer);
            m_telnet.println(F("=================================================="));
            m_telnet.println(F("===     End of Event Log Buffer Dump           ==="));
            m_telnet.println(F("=================================================="));
        }
    }
}

bool TelnetLogger::RecallBufferAdd(const char* message)
{
  if (strlen(message) > RECALL_BUFFER_LEN)
  {
    if (m_telnet.connected())
        m_telnet.println("*** RECALL BUFFER TOO SMALL **");
    Serial.println("*** RECALL BUFFER TOO SMALL **");
    return false;
  }
  // +2 for '\r' and '\n'
  else if (strlen(message) + 2 + m_RecallPosition < RECALL_BUFFER_LEN)
  {
    // No wrapping
    strcpy (&RecallBuffer[m_RecallPosition], message);
    m_RecallPosition += strlen(message) + 2;
  }
  else
  {
    int max_chars;

    if (m_RecallPosition + strlen(message) + 2 <= RECALL_BUFFER_LEN)
      max_chars = strlen(message);
    else if (m_RecallPosition + strlen(message) + 1 <= RECALL_BUFFER_LEN)
      // If we align with the end of the recall buffer, we just need space for a '\0'
      max_chars = strlen(message) - 1;
    else
      max_chars = RECALL_BUFFER_LEN - m_RecallPosition;

    memcpy (&RecallBuffer[m_RecallPosition], message, max_chars);
    memcpy (RecallBuffer, &message[max_chars], strlen(message) - max_chars);

    m_RecallPosition = strlen(message) - max_chars + 2;

    // We allocated 1 extra byte, so this is OK it's not zero-based
    RecallBuffer[RECALL_BUFFER_LEN] = '\0';
  }
  // Add necessary extra characters to make it print pretty
  RecallBuffer[m_RecallPosition - 2] = '\r';
  RecallBuffer[m_RecallPosition - 1] = '\n';
  RecallBuffer[m_RecallPosition] = '\0';

  return true;
}

/// Send the log message to Papertrail.
void TelnetLogger::log(const char *category, String message) {
    //String time = Time.format(Time.now(), TIME_FORMAT_ISO8601_FULL);
    //String packet = String::format("<22>1 %s %s %s - - - %s", time.c_str(), m_system.c_str(), m_app.c_str(), message.c_str());

    // Just in case someone calls Log.foo from inside publish we don't want to recurse
    if(!m_publishing && strcmp(category, "app") == 0) {
        m_publishing++;
#ifdef PARTICLE_BENCH
        Particle.publish(m_name, message, PRIVATE);
#endif
        if (m_enable)
            m_telnet.printf("%s: %s\n", m_name.c_str(), message.c_str());
        m_publishing--;
    }

    RecallBufferAdd(message.c_str());
}

TelnetLogger *TelnetLogger::setLevel(LogLevel level) {
    TelnetLogger *newLogHandler;
    LogManager::instance()->removeHandler(this);
    m_LogLevel = level;
    newLogHandler = new TelnetLogger(m_telnet, m_name, level);
    LogManager::instance()->addHandler(newLogHandler);
    return newLogHandler;
}

void TelnetLogger::enable() {
    m_enable = true;
}

void TelnetLogger::disable() {
    m_enable = false;
}

void TelnetLogger::logMessage(const char *msg, LogLevel level, const char *category, const LogAttributes &attr) {
    String s;

    // Level
    s.concat(levelName(level));
    s.concat(": ");

    // Message
    if (msg) {
        s.concat(msg);
    }

    // Additional attributes
    if (attr.has_code || attr.has_details) {
        s.concat(" [");
        // Code
        if (attr.has_code) {
            s.concat(String::format("code = %p", (intptr_t)attr.code));
        }
        // Details
        if (attr.has_details) {
            if (attr.has_code) {
                s.concat(", ");
            }
            s.concat("details = ");
            s.concat(attr.details);
        }
        s.concat(']');
    }

    log(category, s);
}
