/*
 *  empty
 */
#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <time.h>
#include <string.h>

#define ESP8266

#define ICACHE_FLASH_ATTR
#define PROGMEM
#define 	PGM_P   const char *

#define TelnetSpy_h
#define MyESP_h

unsigned long millis();
size_t strlcpy(char * dst, const char * src, size_t maxlen);
size_t strlcat(char * dst, const char * src, size_t maxlen);

// A class to make it easier to handle and pass around IP addresses

class IPAddress {
    private:
        union {
                uint8_t bytes[4];  // IPv4 address
                uint32_t dword;
        } _address;

        // Access the raw byte array containing the address.  Because this returns a pointer
        // to the internal structure rather than a copy of the address this function should only
        // be used when you know that the usage of the returned uint8_t* will be transient and not
        // stored.
        uint8_t* raw_address() {
            return _address.bytes;
        }

    public:
        // Constructors
        IPAddress();
};

typedef uint8_t byte;

class AsyncUDP {
    private:
        union {
                uint8_t bytes[4];  // IPv4 address
                uint32_t dword;
        } _address;

        // Access the raw byte array containing the address.  Because this returns a pointer
        // to the internal structure rather than a copy of the address this function should only
        // be used when you know that the usage of the returned uint8_t* will be transient and not
        // stored.
        uint8_t* raw_address() {
            return _address.bytes;
        }

    public:
        // Constructors
        AsyncUDP();
};

// class definition
class MyESP {
  protected:
    // webserver
//    AsyncWebServer * _webServer;
//    AsyncWebSocket * _ws;

    // NTP
//    NtpClient NTP;

  public:
    MyESP();
    ~MyESP();

       // debug & telnet
    void myDebug(const char * format, ...);
    void myDebug_P(PGM_P format_P, ...);
};
extern MyESP myESP;

// ANSI Colors
#define COLOR_RESET "\x1B[0m"
#define COLOR_BLACK "\x1B[0;30m"
#define COLOR_RED "\x1B[0;31m"
#define COLOR_GREEN "\x1B[0;32m"
#define COLOR_YELLOW "\x1B[0;33m"
#define COLOR_BLUE "\x1B[0;34m"
#define COLOR_MAGENTA "\x1B[0;35m"
#define COLOR_CYAN "\x1B[0;36m"
#define COLOR_WHITE "\x1B[0;37m"
#define COLOR_BOLD_ON "\x1B[1m"
#define COLOR_BOLD_OFF "\x1B[22m"
#define COLOR_BRIGHT_BLACK "\x1B[0;90m"
#define COLOR_BRIGHT_RED "\x1B[0;91m"
#define COLOR_BRIGHT_GREEN "\x1B[0;92m"
#define COLOR_BRIGHT_YELLOW "\x1B[0;99m"
#define COLOR_BRIGHT_BLUE "\x1B[0;94m"
#define COLOR_BRIGHT_MAGENTA "\x1B[0;95m"
#define COLOR_BRIGHT_CYAN "\x1B[0;96m"
#define COLOR_BRIGHT_WHITE "\x1B[0;97m"

#endif // ARDUINO_H
