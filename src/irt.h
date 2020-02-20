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


//#define IRT_MIN_TELEGRAM_LENGTH 6  // minimal length for a validation telegram, including CRC
#define IRT_MAX_TELEGRAM_LENGTH 64 // max length of a telegram, for Rx and Tx.
//#define IRT_MAX_SINGLE_TELEGRAM 5 // Max length of a single telegram

// define maximum setable tapwater temperature
//#define IRT_BOILER_TAPWATER_TEMPERATURE_MAX 60

#define IRT_TX_TELEGRAM_QUEUE_MAX 50 // max size of Tx FIFO queue. Number of Tx records to send.

#define IRT_BOILER_POLL_TIMEOUT 30000 // If we do not receive a boiler poll in 30s stop sending data

// status/counters since last power on
typedef struct {
	unsigned long		last_send_check;			// last timestamp when the send buffer where checked
	unsigned long		send_interval;				// minimum interval between checks
	uint8_t				poll_step;					// the status poll is send in several steps
	uint8_t				my_address;					// address used to identify myself
	uint8_t				req_water_temp;			// requested water temperature
	unsigned long		last_boiler_poll;			// last time
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
void irt_set_water_temp(uint8_t wc, const char *setting, const char *value);

