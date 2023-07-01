// Example usage for TelnetLogger library by steve@meisners.net.

// Note: Connect via the USB serial console to watch for the IP
// address of the Particle board. Then telnet into it and poke
// around at the code.
//
// Of course, set your wifi SSID and PSK

#define WIFI_SSID "ssid"
#define WIFI_PSK  "psk"

#include <TelnetLogger.h>

TCPServer telnetServer = TCPServer(23);
TCPClient telnetClient;
// Pointer to object to install our logger
TelnetLogger *telnetLogger;

void setup() {
  Serial.begin(115200);
  waitFor(Serial.isConnected, 30000);
  WiFi.on();
  WiFi.setCredentials(WIFI_SSID, WIFI_PSK, WPA2, WLAN_CIPHER_AES);
  WiFi.connect();
  waitFor(WiFi.ready, 15000);
  Serial.printlnf("IP address: %s", WiFi.localIP().toString().c_str());
  Serial.printlnf("Telnet to the address above");

#ifdef PARTICLE_BENCH
    Particle.connect();
#endif

  telnetLogger = new TelnetLogger(telnetClient, "app", LOG_LEVEL_INFO);

  // Start the telnet server and allow connections
  telnetServer.begin();

  Log.info("Hi I'm a log message");
}

void regenerateLogHandler(LogLevel level)
{
    auto logManager = LogManager::instance();
    logManager->removeHandler(telnetLogger);
    telnetLogger = new TelnetLogger(telnetClient, "telnet", level);
    logManager->addHandler(telnetLogger);
    telnetLogger->enable();
}

void telnetProcessInput(char *txt, TCPClient telnetClient)
{
  char ch = txt[0];
  switch (ch)
  {
    case 'R':
    case 'r':
      Log.warn("Restart requested from telnet client");
      telnetClient.write ("Restarting ESP32...");
      telnetLogger->disable();
      telnetClient.stop();
      delay(100);
      System.reset();
      break;
    case 'Q':
    case 'q':
      Log.info ("Closing telnet connection");
      telnetClient.write ("Goodbye...");
      telnetLogger->disable();
      telnetClient.stop();
      delay(100);
      break;
    case 'T':
    case 't':
      regenerateLogHandler(LOG_LEVEL_TRACE);
      Log.trace("Changed telnet log level to: LOG_LEVEL_TRACE");
      break;
    case 'I':
    case 'i':
      regenerateLogHandler(LOG_LEVEL_INFO);
      Log.trace("Changed telnet log level to: LOG_LEVEL_INFO");
      break;
    case 'W':
    case 'w':
      regenerateLogHandler(LOG_LEVEL_WARN);
      Log.trace("Changed telnet log level to: LOG_LEVEL_WARN");
      break;
    case 'E':
    case 'e':
      regenerateLogHandler(LOG_LEVEL_ERROR);
      Log.trace("Changed telnet log level to: LOG_LEVEL_ERROR");
      break;
    case 'D':
    case 'd':
      telnetClient.println("Dump log history: ");
      //
      // Everytime Log.Xxx() is called, entries have
      // been collected by the TelnetLogger object.
      // The RecallBufferPrint() member func will print
      // the last n entries to the telnet client.
      //
      telnetLogger->RecallBufferPrint();
      break;
    default:
      telnetClient.println("Commands:");
      telnetClient.println("    Q - Quit Telnet session");
      telnetClient.println("    R - Reboot");
      telnetClient.println("    T, I, W, E - Change to log level");
      telnetClient.println("      (Trace, Info, Warning, Error)");
      telnetClient.println("    D - Dump most recent log");
      break;
  }
}


void telnetLoop()
{
  #define TELNET_BUFF_LEN 80
  static char telnetCommand[TELNET_BUFF_LEN];
  static bool TelnetClientConnected = false;
  char ch;
  if (telnetClient.connected())
  {
    // echo all available bytes back to the client
    while (telnetClient.available())
    {
      ch = telnetClient.read();
      if (ch == '\r')
      {
        if (strlen(telnetCommand) > 0)
        {
          // Call function to process telnet command input
          telnetProcessInput(telnetCommand, telnetClient);
        }
        memset (telnetCommand, 0, TELNET_BUFF_LEN);
      }
      else if ((strlen(telnetCommand) < TELNET_BUFF_LEN) && (isprint(ch)))
      {
        if ((strlen(telnetCommand) == 0) && (ch == ' ')) break;  // Commands can't start with space
        int p = strlen(telnetCommand);
        telnetClient.write(p);
        telnetCommand[p] = ch;
        telnetCommand[p+1] = '\0';
      }
    }
  }
  else
  {
    if (TelnetClientConnected == true)
    {
      Log.info ("Telnet connection was terminated");
      telnetLogger->disable();
      TelnetClientConnected = false;
    }
    // if no client is yet connected, check for a new connection
    telnetClient = telnetServer.available();
    if (telnetClient.connected())
    {
      telnetLogger->enable();
      TelnetClientConnected = true;
      Log.info ("Incoming telnet connection!");
      // Throw away any handshake bytes received during the first .25 secs
      unsigned long start = millis();
      while ((millis() - start) < 250)
        if (telnetClient.available())
          telnetClient.read();
      memset (telnetCommand, 0, TELNET_BUFF_LEN);
    }
  }
}

void loop() {
  static unsigned long start = millis();
  static int loop = 0;
  
  telnetLoop();

  if (millis() - start > 5000)
  {
    start = millis();
    Log.warn ("Loop count %d", loop++);
  }
}