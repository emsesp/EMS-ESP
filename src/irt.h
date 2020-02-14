/*
 * Header file for irt.cpp
 *
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 *
 * See ChangeLog.md for history
 * See README.md for Acknowledgments
 *
 */

#pragma once

#include <Arduino.h>
#include <list> // std::list
#include <ems.h>

// clang-format on

// IRT tx_mode types
#define IRT_TXMODE_DEFAULT 1 // Default (was previously known as tx_mode 2)
#define IRT_TXMODE_IRTPLUS 2 // IRT+
#define IRT_TXMODE_HT3 3     // Junkers HT3

#define IRT_ID_NONE 0x00 // used as a dest in broadcast messages and empty device IDs

// Fixed IRT IDs
#define IRT_ID_ME 0x0B      // our device, hardcoded as the "Service Key"
#define IRT_ID_BOILER 0x08  // all UBA Boilers have 0x08
#define IRT_ID_SM 0x30      // Solar Module SM10, SM100 and ISM1
#define IRT_ID_HP 0x38      // HeatPump
#define IRT_ID_GATEWAY 0x48 // KM200 Web Gateway

// Product IDs
#define IRT_PRODUCTID_HEATRONIC 95 // Junkers Heatronic 3 device
#define IRT_PRODUCTID_SM10 73      // SM10 solar module
#define IRT_PRODUCTID_SM50 162     // SM50 solar module
#define IRT_PRODUCTID_SM100 163    // SM100 solar module
#define IRT_PRODUCTID_ISM1 101     // Junkers ISM1 solar module

#define IRT_MIN_TELEGRAM_LENGTH 6  // minimal length for a validation telegram, including CRC
#define IRT_MAX_TELEGRAM_LENGTH 64 // max length of a telegram, for Rx and Tx.
#define IRT_MAX_SINGLE_TELEGRAM 5 // Max length of a single telegram

// default values for null values
#define IRT_VALUE_INT_ON 1             // boolean true
#define IRT_VALUE_INT_OFF 0            // boolean false
#define IRT_VALUE_INT_NOTSET 0xFF      // for 8-bit unsigned ints/bytes
#define IRT_VALUE_SHORT_NOTSET -32768  // for 2-byte signed shorts
#define IRT_VALUE_USHORT_NOTSET 0x8000 // for 2-byte unsigned shorts
#define IRT_VALUE_LONG_NOTSET 0xFFFFFF // for 3-byte longs

// thermostat specific
#define IRT_THERMOSTAT_MAXHC 4     // max number of heating circuits
#define IRT_THERMOSTAT_DEFAULTHC 1 // default heating circuit is 1
#define IRT_THERMOSTAT_WRITE_YES true
#define IRT_THERMOSTAT_WRITE_NO false

// trigger settings to determine if hot tap water or the heating is active
#define IRT_BOILER_BURNPOWER_TAPWATER 100
#define IRT_BOILER_SELFLOWTEMP_HEATING 70

// define maximum setable tapwater temperature
#define IRT_BOILER_TAPWATER_TEMPERATURE_MAX 60

#define IRT_TX_TELEGRAM_QUEUE_MAX 50 // max size of Tx FIFO queue. Number of Tx records to send.

//#define IRT_SYS_LOGGING_DEFAULT IRT_SYS_LOGGING_VERBOSE // turn on for debugging
#define IRT_SYS_LOGGING_DEFAULT IRT_SYS_LOGGING_NONE

#define IRT_SYS_DEVICEMAP_LENGTH 15 // size of the 0x07 telegram data part which stores all active IRT devices

// define the model types
// which get rendered to html colors in the web interface in file custom.js in function listCustomStats()
#define IRT_MODELTYPE_BOILER 1     // success color
#define IRT_MODELTYPE_THERMOSTAT 2 // info color
#define IRT_MODELTYPE_SM 3         // warning color
#define IRT_MODELTYPE_HP 4         // success color
#define IRT_MODELTYPE_OTHER 5      // no color
#define IRT_MODELTYPE_UNKNOWN 6    // no color

#define IRT_MODELTYPE_UNKNOWN_STRING "unknown?" // model type text to use when discovering an unknown device



#define IRT_TX_SUCCESS 0x01 // IRT single byte after a Tx Write indicating a success
#define IRT_TX_ERROR 0x04   // IRT single byte after a Tx Write indicating an error


// status/counters since last power on
typedef struct {
	unsigned long		last_send_check;			// last timestamp when the send buffer where checked
	unsigned long		send_interval;				// minimum interval between checks
	uint8_t				poll_step;					// the status poll is send in several steps
	uint8_t				my_address;					// address used to identify myself
} _IRT_Sys_Status;

#define IRT_MAX_SUB_MSGS 5 // max 5 messages in a single go
#define IRT_MAX_SUB_MSG_LEN 3 // max 3 bytes in a message
// The Tx send package
typedef struct {
	uint8_t						address;													// address of sender
	uint8_t						msg_in_use;												// number of sub msg's in telegram
	uint8_t						data[IRT_MAX_SUB_MSGS][IRT_MAX_SUB_MSG_LEN];	// data of msgs
} _IRT_TxTelegram;

// The Rx receive package
typedef struct {
	uint32_t		timestamp;			// timestamp from millis()
//	uint8_t		section_count;		// msg section counter
	uint8_t		device_nr;			// device number (should be 0)
    uint8_t * telegram;     // the full data package
    uint8_t   data_length;  // length in bytes of the data
    uint8_t   length;       // full length of the complete telegram
    uint8_t   src;          // source ID
    uint8_t   dest;         // destination ID
    uint16_t  type;         // type ID as a double byte to support EMS+
    uint8_t   offset;       // offset
    uint8_t * data;         // pointer to where telegram data starts
    bool      emsplus;      // true if ems+/ems 2.0
    uint8_t   emsplus_type; // FF, F7 or F9
} _IRT_RxTelegram;

// function definitions
extern void irt_dumpBuffer(const char * prefix, uint8_t * telegram, uint8_t length);
extern void irt_parseTelegram(uint8_t * telegram, uint8_t len);

void irt_init();
void irt_stop();
void irt_start();
void irt_loop();
void irt_sendRawTelegram(char * telegram);

