# TelnetLogger (ported from ParticleWebLog by kevinh _at_ geeksville.com)

A Particle library for logging via a telnet client and publish().

This library was created for a device that acts as a telnet server to allow
telnet connections. This allows for remote debugging and changing of parameters.

This library was created from an existing project, named ParticleWebLog by 
kevinh@geeksville.com. The original project is located at:
https://github.com/geeksville/ParticleWebLog.git

The original project was very well written (small and concise) to allow me
to add the functionality I needed.

When you use this library, and provide a telnet client (TCPClient object),
any calls to Log.Xxx() will stream the message to the usual logging facility
but also to the telnet client. This will provide realtime monitoring of activity.
Also, if you enable it, it will also publish to the Particle.io web
console. To enable this;

```#define PARTICLE_BENCH```

## Usage

```
#include <TelnetLogger.h>

TCPServer telnetServer = TCPServer(23);
TCPClient telnetClient;
// Install our logger
TelnetLogger *telnetLogger;

void setup() {
  telnetLogger = new TelnetLogger(telnetClient, "app", LOG_LEVEL_INFO);
  memset (telnetCommand, 0, TELNET_BUFF_LEN);
  // Start the telnet server and connect
  telnetServer.begin();

  Log.info("Hi I'm a log message");
}

void loop() {
}
```

See the [examples](examples) folder for more details.

## Documentation

This library merely registers a log provider that publishes each logged string
as a Telnet client printf and, if enabled at compile time, via a Particle publish event.
The events will be published using the name of your choice (defaults to "log").

The point of yet another logging library is this one will allow operatinga Particle
microcontroller completely locally...only enabling connection to PArticle if you choose.
I wanted everything to be locally on the wifi, so I created this library and telnet into
my device (a Particle Argon at the time of this writing).

Whenever a new TelnetLogger object is created, it is good to be aware that a sizable
buffer is allocated for the ring buffer (default 2K bytes). Also good to know is
that the primary object is deleted and recreated everytime the log level is changed.
This is due to not being able to figure out how to modify this value ... there's
probably a way to do this. See the example code to understand how to change the log level.

Limitations:

* A circular ring buffer is used to store the activity log. The default size of the
ring is ~2k bytes. If a message that is larger than that is written into the 
ring, it will quietly fail.
* No connection to the Particle made is established by default. Only if you compile
 with PARTICLE_BENCH defined will the library attempt to performa Particle publish.
 This assumes your code will perform the steps neccessary to establish a link to Particle
 and your device is properly registered with Particle.

## Using web logging services (Particle.io)

The library will only attempt to perform a Particle publish if the symbol "PARTICLE_BENCH"
is defined at compile. If this is true, whenever a message is logged (using Log.Xxx in your code), 
a Particle.publish() call will also be made. Of course for this to succeed, a network connection
must be established and your device must be registered with Particle. Please refer to the Particle.io
site and forums for additional details in using Particle's workbench and console.

## Contributing

The original author previously stated, "I will happily accept pull requests and
respond to issues raised in github." Therefore, it may be best to verify any problems or
code change requests against the original Github project. Anything related to the Telnet version
of this library can be directed to steve@meisners.net .

## LICENSE
Copyright 2022 steve@meisners.net (newer Telnet based version)  
Copyright 2019 kevinh@geeksville.com (initial code)

Licensed under the MIT license
