/**
 * irt.cpp
 *
 * Handles all the processing of the IRT messages
 *
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 */

#include "irt.h"
#include "MyESP.h"
//#include "ems_devices.h"
#include "irtuart.h"
#include <CircularBuffer.h> // https://github.com/rlogiacco/CircularBuffer

// MyESP class for logging to telnet and serial
#define myDebug(...) myESP.myDebug(__VA_ARGS__)
#define myDebug_P(...) myESP.myDebug_P(__VA_ARGS__)

char * _hextoa(uint8_t value, char * buffer);
char * _smallitoa(uint8_t value, char * buffer);
char * _smallitoa3(uint16_t value, char * buffer);

/**
 * dump a UART Tx or Rx buffer to console...
 */
void irt_dumpBuffer(const char * prefix, uint8_t * telegram, uint8_t length) {
    uint32_t    timestamp       = millis();
    static char output_str[200] = {0};
    static char buffer[16]      = {0};


    strlcpy(output_str, "(", sizeof(output_str));
    strlcat(output_str, COLOR_CYAN, sizeof(output_str));
    strlcat(output_str, _smallitoa((uint8_t)((timestamp / 3600000) % 24), buffer), sizeof(output_str));
    strlcat(output_str, ":", sizeof(output_str));
    strlcat(output_str, _smallitoa((uint8_t)((timestamp / 60000) % 60), buffer), sizeof(output_str));
    strlcat(output_str, ":", sizeof(output_str));
    strlcat(output_str, _smallitoa((uint8_t)((timestamp / 1000) % 60), buffer), sizeof(output_str));
    strlcat(output_str, ".", sizeof(output_str));
    strlcat(output_str, _smallitoa3(timestamp % 1000, buffer), sizeof(output_str));
    strlcat(output_str, COLOR_RESET, sizeof(output_str));
    strlcat(output_str, ") ", sizeof(output_str));

    strlcat(output_str, COLOR_YELLOW, sizeof(output_str));
    strlcat(output_str, prefix, sizeof(output_str));

    // show some EMS_Sys_Status entries
    strlcat(output_str, _hextoa(EMS_Sys_Status.emsRxStatus, buffer), sizeof(output_str));
    strlcat(output_str, " ", sizeof(output_str));
    strlcat(output_str, _hextoa(EMS_Sys_Status.emsTxStatus, buffer), sizeof(output_str));
    strlcat(output_str, ": ", sizeof(output_str));


    // print whole buffer, don't interpret any data
    for (int i = 0; i < (length); i++) {
        strlcat(output_str, _hextoa(telegram[i], buffer), sizeof(output_str));
        strlcat(output_str, " ", sizeof(output_str));
    }

    strlcat(output_str, COLOR_RESET, sizeof(output_str));

    myDebug(output_str);
}

/**
 * Entry point triggered by an interrupt in irtuart.cpp
 * length is the number of all the telegram bytes
 * Read commands are asynchronous as they're handled by the interrupt
 * When a telegram is processed we forcefully erase it from the stack to prevent overflow
 */
void irt_parseTelegram(uint8_t * telegram, uint8_t length) {
    if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_JABBER) {
        irt_dumpBuffer("irt_parseTelegram: ", telegram, length);
    }
#ifdef nuniet
    /*
     * Detect the EMS bus type - Buderus or Junkers - and set emsIDMask accordingly.
     *  we wait for the first valid telegram and look at the SourceID.
     *  If Bit 7 is set we have a Buderus, otherwise a Junkers
     */
    if (EMS_Sys_Status.emsTxStatus == EMS_TX_REV_DETECT) {
        if ((length >= 5) && (telegram[length - 1] == _crcCalculator(telegram, length))) {
            EMS_Sys_Status.emsTxStatus   = EMS_TX_STATUS_IDLE;
            EMS_Sys_Status.emsIDMask     = telegram[0] & 0x80;
            EMS_Sys_Status.emsPollAck[0] = EMS_ID_ME ^ EMS_Sys_Status.emsIDMask;
        } else
            return; // ignore the whole telegram Rx Telegram while in DETECT mode
    }

    /*
     * It may happen that we where interrupted (for instance by WIFI activity) and the
     * buffer isn't valid anymore, so we must not answer at all...
     */
    if (EMS_Sys_Status.emsRxStatus != EMS_RX_STATUS_IDLE) {
        if (EMS_Sys_Status.emsLogging > EMS_SYS_LOGGING_NONE) {
            myDebug_P(PSTR("** Warning, we missed the bus - Rx non-idle!"));
        }
        return;
    }

    /*
     * check if we just received a single byte
     * it could well be a Poll request from the boiler for us, which will have a value of 0x8B (0x0B | 0x80)
     * or either a return code like 0x01 or 0x04 from the last Write command
     */
    if (length == 1) {
        uint8_t         value                  = telegram[0]; // 1st byte of data package
        static uint32_t _last_emsPollFrequency = 0;

        // check first for a Poll for us
        if ((value ^ 0x80 ^ EMS_Sys_Status.emsIDMask) == EMS_ID_ME) {
            uint32_t timenow_microsecs      = micros();
            EMS_Sys_Status.emsPollFrequency = (timenow_microsecs - _last_emsPollFrequency);
            _last_emsPollFrequency          = timenow_microsecs;

            // do we have something to send thats waiting in the Tx queue?
            // if so send it if the Queue is not in a wait state
            if ((!EMS_TxQueue.isEmpty()) && (EMS_Sys_Status.emsTxStatus == EMS_TX_STATUS_IDLE)) {
                _ems_sendTelegram(); // perform the read/write command immediately
            } else {
                // nothing to send so just send a poll acknowledgement back
                if (EMS_Sys_Status.emsPollEnabled) {
                    ems_tx_pollAck();
                }
            }
        } else if (EMS_Sys_Status.emsTxStatus == EMS_TX_STATUS_WAIT) {
            // this may be a single byte 01 (success) or 04 (error) from a recent write command?
            if (value == EMS_TX_SUCCESS) {
                EMS_Sys_Status.emsTxPkgs++;
                // got a success 01. Send a validate to check the value of the last write
                ems_tx_pollAck();  // send a poll to free the EMS bus
                _createValidate(); // create a validate Tx request (if needed)
            } else if (value == EMS_TX_ERROR) {
                // last write failed (04), delete it from queue and dont bother to retry
                if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_VERBOSE) {
                    myDebug_P(PSTR("-> Error: Write command failed from host"));
                }
                ems_tx_pollAck(); // send a poll to free the EMS bus
                _removeTxQueue(); // remove from queue
            }
        }

        return; // all done here
    }

    // ignore anything that doesn't resemble a proper telegram package
    // minimal is 5 bytes, excluding CRC at the end (for EMS1.0)
    if (length <= 4) {
        // _debugPrintTelegram("Noisy data: ", &EMS_RxTelegram, COLOR_RED);
        return;
    }

    static _EMS_RxTelegram EMS_RxTelegram; // create the Rx package
    EMS_RxTelegram.telegram  = telegram;
    EMS_RxTelegram.timestamp = millis();
    EMS_RxTelegram.length    = length;

    EMS_RxTelegram.src    = telegram[0] & 0x7F; // removing 8th bit as we deal with both reads and writes here
    EMS_RxTelegram.dest   = telegram[1] & 0x7F; // remove 8th bit (don't care if read or write)
    EMS_RxTelegram.offset = telegram[3];        // offset is always 4th byte

    // determing if its normal ems or ems plus, check for marker
    if (telegram[2] >= 0xF0) {
        // its EMS plus / EMS 2.0
        EMS_RxTelegram.emsplus      = true;
        EMS_RxTelegram.emsplus_type = telegram[2]; // 0xFF, 0xF7 or 0xF9

        if (EMS_RxTelegram.emsplus_type == 0xFF) {
            EMS_RxTelegram.type = (telegram[4] << 8) + telegram[5]; // is a long in bytes 5 & 6
            EMS_RxTelegram.data = telegram + 6;

            if (length <= 7) {
                EMS_RxTelegram.data_length = 0; // special broadcast on ems+ have no data values
            } else {
                EMS_RxTelegram.data_length = length - 7; // remove 6 byte header plus CRC
            }
        } else {
            // its F9 or F7
            uint8_t shift       = (telegram[4] != 0xFF); // true (1) if byte 4 is not 0xFF, then telegram is 1 byte longer
            EMS_RxTelegram.type = (telegram[5 + shift] << 8) + telegram[6 + shift];
            EMS_RxTelegram.data = telegram + 6 + shift; // there is a special byte after the typeID which we ignore for now
            if (length <= (9 + shift)) {
                EMS_RxTelegram.data_length = 0; // special broadcast on ems+ have no data values
            } else {
                EMS_RxTelegram.data_length = length - (9 + shift);
            }
        }
    } else {
        // Normal EMS 1.0
        EMS_RxTelegram.emsplus     = false;
        EMS_RxTelegram.type        = telegram[2]; // 3rd byte
        EMS_RxTelegram.data        = telegram + 4;
        EMS_RxTelegram.data_length = length - 5; // remove 4 bytes header plus CRC
    }

    // if we are in raw logging mode then just print out the telegram as it is
    // but still continue to process it
    if ((EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_RAW)) {
        _debugPrintTelegram("", &EMS_RxTelegram, COLOR_WHITE, true);
    }

    // Assume at this point we have something that vaguely resembles a telegram in the format [src] [dest] [type] [offset] [data] [crc]
    // validate the CRC, if it's bad ignore it
    if (telegram[length - 1] != _crcCalculator(telegram, length)) {
        LA_PULSE(200);
        EMS_Sys_Status.emxCrcErr++;
        if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_VERBOSE) {
            _debugPrintTelegram("Corrupt telegram: ", &EMS_RxTelegram, COLOR_RED, true);
        }
        return;
    }

    // here we know its a valid incoming telegram of at least 6 bytes
    // we use this to see if we always have a connection to the boiler, in case of drop outs
    EMS_Sys_Status.emsRxTimestamp  = EMS_RxTelegram.timestamp; // timestamp of last read
    EMS_Sys_Status.emsBusConnected = true;

    // now lets process it and see what to do next
    _processType(&EMS_RxTelegram);
#endif
}
