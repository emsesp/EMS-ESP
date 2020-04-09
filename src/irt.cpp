/**
 * irt.cpp
 *
 * Handles all the processing of the IRT messages
 *
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 */

#include <stdio.h>
#include "irt.h"
//#include "ems_devices.h"
#include "MyESP.h"
#include "irtuart.h"
#include <CircularBuffer.h> // https://github.com/rlogiacco/CircularBuffer

#include <Ticker.h> // https://github.com/esp8266/Arduino/tree/master/libraries/Ticker

// MyESP class for logging to telnet and serial
#define myDebug(...) myESP.myDebug(__VA_ARGS__)
#define myDebug_P(...) myESP.myDebug_P(__VA_ARGS__)

const uint32_t EMS_BUS_TIMEOUT        = 15000;   // timeout in ms before recognizing the ems bus is offline (15 seconds)

extern _EMSESP_Settings EMSESP_Settings;
_EMS_Boiler EMS_Boiler;
_EMS_Thermostat  EMS_Thermostat;
// for storing all detected EMS devices
std::list<_Generic_Device> Devices;

_IRT_Sys_Status IRT_Sys_Status; // iRT Status
_EMS_Sys_Status EMS_Sys_Status; // EMS Status

CircularBuffer<_IRT_TxTelegram, IRT_TX_TELEGRAM_QUEUE_MAX> IRT_TxQueue; // FIFO queue for Tx send buffer

Ticker updateFlowTempTimer;

char * _hextoa(uint8_t value, char * buffer);
char * _smallitoa(uint8_t value, char * buffer);
char * _smallitoa3(uint16_t value, char * buffer);

/**
 * dump a UART Tx or Rx buffer to console...
 */
void irt_dumpBuffer(const char * prefix, uint8_t * telegram, uint8_t length)
{
	uint32_t    timestamp       = millis();
	static char output_str[300] = {0};
	static char buffer[16]      = {0};

	if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_NONE)
		return;

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

	strlcat(output_str, _hextoa(length, buffer), sizeof(output_str));
	strlcat(output_str, ": ", sizeof(output_str));


	// print whole buffer, don't interpret any data
	for (int i = 0; i < (length); i++) {
		strlcat(output_str, _hextoa(telegram[i], buffer), sizeof(output_str));
		strlcat(output_str, " ", sizeof(output_str));
	}
#ifdef INCLUDE_ASCII
	// Added ASCII
	strlcat(output_str, " - \"", sizeof(output_str));
	char dump_text[2];
	for (int i = 0; i < (length); i++) {
		if ((telegram[i] < ' ') || (telegram[i] == 0x25)){
			strlcat(output_str, ".", sizeof(output_str));
		} else {
			dump_text[0] = (char)telegram[i];
			dump_text[1] = 0;
			strlcat(output_str, dump_text, sizeof(output_str));
		}
	}
	strlcat(output_str, "\"", sizeof(output_str));
#endif // INCLUDE_ASCII

	strlcat(output_str, COLOR_RESET, sizeof(output_str));

	myDebug(output_str);
}

uint8_t global_status[0x100];
uint8_t global_status_inuse[0x100];
uint8_t global_has_changed = 0;
uint8_t global_status_first_run = 1;

void irt_update_status(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{

	if (global_status_first_run) {
		memset(global_status, 0, sizeof(global_status));
		memset(global_status_inuse, 0, sizeof(global_status_inuse));
		global_status_first_run = 0;
	}
	if (data[0] == 0xF0) return;

	if (data[0] & 0x80) {
		if (global_status[data[0]] != data[4]) global_has_changed = 1;
		global_status[data[0]] = data[4];
	} else {
		if (global_status[data[0]] != data[1]) global_has_changed = 1;
		global_status[data[0]] = data[1];
	}
	global_status_inuse[data[0]] = 1;

}
void irt_update_single_status(uint8_t cmd, uint8_t data)
{
	if (global_status[cmd] != data) global_has_changed = 1;
	global_status[cmd] = data;
	global_status_inuse[cmd] = 1;
}

uint8_t irt_handle_0x05(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/*  0  1  2  3 */
	/* 05 00 E2 A6  - warm water off*/
	/* 05 04 E2 A6 - warm water on*/

	if (data[1] & 0x04) {
		// warm water on
		EMS_Boiler.wWActivated = 1;
	} else {
		// warm water off
		EMS_Boiler.wWActivated = 0;
	}

	return 0;
}

uint8_t irt_handle_0x07(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* burner power 0x4d min power 0xff max power */
	/*  0  1  2  3 */
	/* 07 00 D0 6C */
	/* 07 ss D0 ss*/

	EMS_Thermostat.model_id        = EMS_MODEL_FR10;
	EMS_Thermostat.device_id       = 0x01;
	EMS_Thermostat.write_supported = EMS_THERMOSTAT_WRITE_NO;
	EMS_Thermostat.product_id      = 0x01;
	strlcpy(EMS_Thermostat.version, "1.0", sizeof(EMS_Thermostat.version));
//	uint8_t hc = 0;
//	EMS_Thermostat.hc[hc].active = true;
//	EMS_Thermostat.hc[hc].setpoint_roomTemp = (data[1] * 3);

	ems_Device_add_flags(EMS_DEVICE_UPDATE_FLAG_THERMOSTAT);
	return 0;
}

uint8_t irt_handle_0x73(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/*  0  1  2  3  */
	/* 73 52 25 43 */
	/* 73 52 25 43 */
	return 0;

}
uint8_t irt_handle_0x78(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/*  0  1  2  3  */
	/* 78 07 FF A1 */
	/* 78 07 FF A1 */

	irt_update_single_status(0x10 + (data[1] & 0x0F), data[2]);

	return 0;
}
/*
    // UBAParameterWW
    EMS_Boiler.wWActivated   = EMS_VALUE_INT_NOTSET; // Warm Water activated
    EMS_Boiler.wWSelTemp     = EMS_VALUE_INT_NOTSET; // Warm Water selected temperature
    EMS_Boiler.wWCircPump    = EMS_VALUE_INT_NOTSET; // Warm Water circulation pump available
    EMS_Boiler.wWDesiredTemp = EMS_VALUE_INT_NOTSET; // Warm Water desired temperature to prevent infection
    EMS_Boiler.wWComfort     = EMS_VALUE_INT_NOTSET;

    // UBAMonitorFast
    EMS_Boiler.selFlowTemp = EMS_VALUE_INT_NOTSET;    // Selected flow temperature
    EMS_Boiler.curFlowTemp = EMS_VALUE_USHORT_NOTSET; // Current flow temperature
    EMS_Boiler.retTemp     = EMS_VALUE_USHORT_NOTSET; // Return temperature
    EMS_Boiler.burnGas     = EMS_VALUE_INT_NOTSET;    // Gas on/off
    EMS_Boiler.fanWork     = EMS_VALUE_INT_NOTSET;    // Fan on/off
    EMS_Boiler.ignWork     = EMS_VALUE_INT_NOTSET;    // Ignition on/off
    EMS_Boiler.heatPmp     = EMS_VALUE_INT_NOTSET;    // Boiler pump on/off
    EMS_Boiler.wWHeat      = EMS_VALUE_INT_NOTSET;    // 3-way valve on WW
    EMS_Boiler.wWCirc      = EMS_VALUE_INT_NOTSET;    // Circulation on/off
   */

uint8_t irt_handle_0x82(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* 82 4A 0A 3E 00 */
	/* 82 ?? ?? ?? ss Status 0x00 -> 0x04 -> 0x84
	Burner status:
	bit 7: 0x80 - Burner on
	bit 6: 0x40 - ??
	bit 5: 0x20 - Warm Water
	bit 4: 0x10 - 3-way valve (0 - cv / 1 - ww)
	bit 3: 0x08 - ??
	bit 2: 0x04 - central heating on
	bit 1: 0x02 - ?? (seems to be 1 in maintenance mode)
	bit 0: 0x01 - Warm Water

	*/
	if (length != 5)
		return 10;
	if (data[4] & 0x80) { // burner on
		EMS_Boiler.burnGas = 1;
		EMS_Boiler.fanWork = 1;
	} else {
		EMS_Boiler.burnGas = 0;
		EMS_Boiler.fanWork = 0;
	}
	if (data[4] & 0x20) { // warm water

		EMS_Boiler.tapwaterActive = 1;
	} else {
		EMS_Boiler.tapwaterActive = 0;
	}
	if (data[4] & 0x10) { // 3-way valve (0 - cv / 1 - ww)
		EMS_Boiler.wWHeat = 1;
	} else {
		EMS_Boiler.wWHeat = 0;
	}
	if (data[4] & 0x04) { // central heating on
		EMS_Boiler.heatingActive = 1;
	} else {
		EMS_Boiler.heatingActive = 0;
	}
	ems_Device_add_flags(EMS_DEVICE_UPDATE_FLAG_BOILER);

	return 0;
}

uint8_t irt_handle_0x83(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{

	return 0;
}
uint8_t irt_handle_0x85(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* 85 C3 79 E7 74
	 * 85 C3 79 E7 ss
	 * ss - 0x04 - on ????
	 * ss - 0x74 - off ?????
	 */

	return 0;
}
uint8_t irt_handle_0x8A(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/*
	 * External thermo sensor
	 *
	 * 8A C3 79 E8 tt
	 *
	 * tt 0x9A - 15.38kOhm resistor
	 * tt 0x67 - 6.93kOhm resistor
	 */

	return 0;
}
uint8_t irt_handle_0x90(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* 90 AE 5F B0 CF
	 * 90 AE 5F B0 ss
	 * ss - 0xCF - normal operation
	 * ss - 0xFF - in maintenance mode
	 */

	/* mark the bus as in-sync */
	EMS_Sys_Status.emsRxTimestamp  = msg->timestamp; // timestamp of last read
	EMS_Sys_Status.emsBusConnected = true;
	EMS_Sys_Status.emsIDMask = 0x00;
	EMS_Boiler.device_id = EMS_ID_BOILER;
	EMS_Sys_Status.emsPollFrequency = 500000; // poll in micro secs
	EMS_Sys_Status.emsTxCapable = true;

	return 0;
}
uint8_t irt_handle_0x93(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* pump on and off
	93 ?? ?? ?? 00 - pump on
	93 ?? ?? ?? FF - pump off
	*/


	if (data[4] == 0xFF) {
		EMS_Boiler.heatPmp = 0;
	} else {
		EMS_Boiler.heatPmp = 1;
	}
	ems_Device_add_flags(EMS_DEVICE_UPDATE_FLAG_BOILER);

	return 0;
}

char irt_getDisplayCode1(uint8_t status) {

	/* n d1 d1 d1 d1 n n n */

	status = (status >> 3) & 0x0F;
	switch (status) {
		case 0x0: return '0'; break;
		case 0x1: return '1'; break;
		case 0x2: return '2'; break;
		case 0x3: return '3'; break;
		case 0x4: return '4'; break;
		case 0x5: return '5'; break;
		case 0x6: return '6'; break;
		case 0x7: return '7'; break;
		case 0x8: return '8'; break;
		case 0x9: return '9'; break;
		case 0xA: return '-'; break;
		case 0xB: return '='; break;
		case 0xC: return 'C'; break; // ??
		case 0xD: return 'D'; break; // ??
		case 0xE: return 'E'; break; // ??
		case 0xF: return 'F'; break; // ??
	}
	return '?';
}

char irt_getDisplayCode2(uint8_t status) {

	/* n n n n n d2 d2 d2 */

	status = status& 0x07;
	switch (status) {
		case 0: return 'A'; break;
		case 1: return 'C'; break;
		case 2: return '2'; break; // ??
		case 3: return 'H'; break;
		case 4: return 'L'; break;
		case 5: return '5'; break; // ??
		case 6: return 'U'; break;
		case 7: return 'y'; break; // ??
	}
	return '?';
}

uint8_t irt_handle_0xA3(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* A3 07 D3 CA 53 */
	/* A3 ?? ?? ?? ss status '0H' '-H' '=H'*/

	/*
	 * bit 7 : 0 - normal status, 1 - Fault code
	 * bit 6 : display code 1
	 * bit 5 : display code 1
	 * bit 4 : display code 1
	 * bit 3 : display code 1
	 * bit 2 : display code 2
	 * bit 1 : display code 2
	 * bit 0 : display code 2
	 */

	EMS_Boiler.serviceCodeChar[0] = irt_getDisplayCode1(data[4]);
	EMS_Boiler.serviceCodeChar[1] = irt_getDisplayCode2(data[4]);
	EMS_Boiler.serviceCodeChar[2] = '\0';

	EMS_Boiler.serviceCode = data[4];
	ems_Device_add_flags(EMS_DEVICE_UPDATE_FLAG_BOILER);

	return 0;
}

uint8_t irt_handle_0xA4(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* A4 5D 17 42 19 */
	/* A4 ?? ?? ?? ww */
	EMS_Boiler.curFlowTemp = (data[4] * 10);
	ems_Device_add_flags(EMS_DEVICE_UPDATE_FLAG_BOILER);

	IRT_Sys_Status.cur_flow_temp = data[4];
	IRT_Sys_Status.last_flow_update = millis();
	return 0;
}
uint8_t irt_handle_0xA6(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	// return temp
	/* A6 A5 F0 1E 1C */
	/* A6 ?? ?? cc ww */
	EMS_Boiler.retTemp = (data[4] * 10);
	ems_Device_add_flags(EMS_DEVICE_UPDATE_FLAG_BOILER);
	return 0;
}

uint8_t irt_handle_0xA8(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	// warm water temp
	/* A8 A5 F0 10 41 */
	/* A8 ?? ?? cc ww */
	EMS_Boiler.wWCurTmp = (data[4] * 10);
	ems_Device_add_flags(EMS_DEVICE_UPDATE_FLAG_BOILER);
	return 0;
}
#include <limits.h>
uint8_t irt_handle_0xC9(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* diff temp between in and out ??/ */
/* last bytes is 0x00, 1 2 3 4 5 (burner status ??)*/
	return 0;
}

uint8_t irt_handle_0xF0(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* extra msgs */

	irt_update_single_status(0xE0 + (data[2] & 0x0F), data[4]);

	return 0;
}

/*
 Modified version of: https://stackoverflow.com/questions/776508/best-practices-for-circular-shift-rotate-operations-in-c
 */
static inline uint8_t rotl8 (uint8_t n, unsigned int c)
{
  const unsigned int mask = (CHAR_BIT*sizeof(n) - 1);  // assumes width is a power of 2.

  // assert ( (c<=mask) &&"rotate by type width or more");
  c &= mask;
  return (n<<c) | (n>>( (-c)&mask ));
}
uint8_t irt_check_checksum(uint8_t *data, uint8_t length)
{
	/* check if if a sub-packet has the right length
	 * and the checksum is correct.
	 * checksum calculation is done by xoring the three
	 * bytes. The second byte is shifted 1 bit the the left and
	 * the third byte is shifted 2 bits to the left before
	 * xoring the bytes.
	 * Depending on the high bit of the second byte and the 2 high bits
	 * of the third byte an additional xor is applied to the output.
	 **/


	if ((length < 4) || (length > 5)) return 0;

	uint8_t check = 0;
	check = data[0];
	check ^= rotl8(data[1], 1);
	check ^= rotl8(data[2], 2);

	switch (((data[1] >> 5) & 0x04) | ((data[2] >> 6) & 0x03)) {
	case 1:
	case 4:
		check = check ^ 0x18;
		break;
	case 2:
	case 7:
		check = check ^ 0x30;
		break;
	case 3:
	case 6:
		check = check ^ 0x28;
		break;
	}

	if (check == data[3]) return 1;
	return 0;
}


uint8_t irt_handleMsg(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* messages are 4 or 5 bytes */
	if (length < 4)
		return 10;
	if ((data[0] & 0x80) && (length < 5))
		return 10;

	if (irt_check_checksum(data, length) != 1) {
		// Drop buffer on crc error
		EMS_Sys_Status.emxCrcErr++;
		irt_dumpBuffer("irt_crcErr4: ", data, length);
		return 11;
	}

	EMS_Sys_Status.emsRxPgks++;

	irt_update_status(msg, data, length);

	if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_RAW) irt_dumpBuffer("irt_raw: ", data, length);

	switch (data[0]) {
	case 0x07:
		return irt_handle_0x07(msg, data, length);
		break;
	case 0x73:
		return irt_handle_0x73(msg, data, length);
		break;
	case 0x78:
		return irt_handle_0x78(msg, data, length);
		break;
	case 0x82:
		return irt_handle_0x82(msg, data, length);
		break;
	case 0x83:
		return irt_handle_0x83(msg, data, length);
		break;
	case 0x85:
		return irt_handle_0x85(msg, data, length);
		break;
	case 0x8A:
		return irt_handle_0x8A(msg, data, length);
		break;
	case 0x90:
		return irt_handle_0x90(msg, data, length);
		break;
	case 0x93:
		return irt_handle_0x93(msg, data, length);
		break;
	case 0xA3:
		return irt_handle_0xA3(msg, data, length);
		break;
	case 0xA4:
		return irt_handle_0xA4(msg, data, length);
		break;
	case 0xA6:
		return irt_handle_0xA6(msg, data, length);
		break;
	case 0xA8:
		return irt_handle_0xA8(msg, data, length);
		break;
	case 0xC9:
		return irt_handle_0xC9(msg, data, length);
		break;
	case 0xF0:
		return irt_handle_0xF0(msg, data, length);
		break;
	}
	return 0;
}


void irt_setupNewMsg(_IRT_RxTelegram *msg)
{
	memset(msg, 0, sizeof(_IRT_RxTelegram));
	msg->timestamp = millis();
//	msg->section_count = 0;
}
/**
 * Entry point triggered by an interrupt in irtuart.cpp
 * length is the number of all the telegram bytes
 * Read commands are asynchronous as they're handled by the interrupt
 * When a telegram is processed we forcefully erase it from the stack to prevent overflow
 */
void irt_parseTelegram(uint8_t *telegram, uint8_t length)
{

	/* the last byte should be the break (0x00). If it is not 0x00 leave it */
	if ((length > 1) && (telegram[length -1]) == 0x00)
		length--;

	if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_JABBER) {
		irt_dumpBuffer("irt_rawTelegram: ", telegram, length);
	}

	if (length == 1) {
		if (telegram[0] == IRT_Sys_Status.my_address) {
			/* mark boiler poll for sending messages */
			IRT_Sys_Status.last_boiler_poll = millis();
		}
	}

	// ignore anything that doesn't resemble a proper telegram package
	if (length <= 3) {
		/* single byte is the boiler polling */
		if (length != 1) {
			EMS_Sys_Status.emxCrcErr++;
			irt_dumpBuffer("irt_crcErr1: ", telegram, length);
		}
		return;
	}

	// for now we just listen to device 01
	// the message should start with:
	// 01 01 FE
	if ((telegram[0] == IRT_Sys_Status.my_address) && (telegram[1] == IRT_Sys_Status.my_address) && (telegram[2] == (0xFF - IRT_Sys_Status.my_address))) {
		// this is a message my device
		/* mark boiler poll for sending messages */
		IRT_Sys_Status.last_boiler_poll = millis();
	} else {
		// this is a message for another device ?
//		if (telegram[0] > 0x03) {
//			EMS_Sys_Status.emxCrcErr++;
//			irt_dumpBuffer("irt_crcErr2: ", telegram, length);
//		}

		if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_JABBER) {
			irt_dumpBuffer("irt_otherTelegram: ", telegram, length);
		}
		return;
	}

	// check the message for errors and remove doubles
	uint8_t i, j, ret;
	uint8_t irt_buffer[IRT_MAX_TELEGRAM_LENGTH];
	_IRT_RxTelegram irtMsg;
	irt_setupNewMsg(&irtMsg);

	// copy the device number
	irtMsg.device_nr = telegram[1];


	i = 3;
	j = 0;
	while (((i + 1) < length) && (j < IRT_MAX_TELEGRAM_LENGTH)) {
		if (telegram[i] == (0xFF - telegram[i+1])) {
			irt_buffer[j++] = telegram[i];
			i+=2;
		} else if (telegram[i] == telegram[i+1]) {
			irt_buffer[j++] = telegram[i];
			i += 2;
		} else {
			// Drop buffer on crc error
			EMS_Sys_Status.emxCrcErr++;
			irt_dumpBuffer("irt_crcErr3: ", telegram, length);
			i++;
			return;
		}
	}
	// no data ?
	if (j < 1)
		return;

//	irt_dumpBuffer("irt_fullTelegram: ", irt_buffer, j);

	/*
	 * Any message with the high bit set is a 'cmd id' followed
	 * by 3 bytes and a response from the boiler
	 * if the bit is not set it is just followed by 3 bytes
	 * 8x tt tt tt bb
	 * 0x tt tt tt
	 */
	i = 0;
	ret = 0;
	while ((i < j) && (ret == 0)) {
		if (irt_buffer[i] & 0x80) {
			ret = irt_handleMsg(&irtMsg, &irt_buffer[i], 5);
			i = i + 5;
		} else {
			ret = irt_handleMsg(&irtMsg, &irt_buffer[i], 4);
			i = i + 4;
		}
	}

	if ((global_has_changed) && (EMS_Sys_Status.emsLogging != EMS_SYS_LOGGING_NONE)) {
		char temp[10];
		char out_text[200];
		int i;

		out_text[0] = 0;
		for (i=0; i<256; i++) {
			if (i == 0x80) {
				strlcat(out_text, "| ", sizeof(out_text));
			}
			if (global_status_inuse[i]) {
				snprintf(temp, sizeof(temp), "%02X:%02X ", i, global_status[i]);
				strlcat(out_text, temp, sizeof(out_text));
			}
		}
		strlcat(out_text, "  ", sizeof(out_text));
		strlcat(out_text, EMS_Boiler.serviceCodeChar, sizeof(out_text));
		myDebug(out_text);
		global_has_changed = 0;
//		if (global_status[0] > 0) showInfo();
	}

}


uint8_t irt_calc_checksum(uint8_t *data)
{
	/* calculate output checksum
	 * checksum calculation is done by xoring the three
	 * bytes. The second byte is shifted 1 bit the the left and
	 * the third byte is shifted 2 bits to the left before
	 * xoring the bytes.
	 * Depending on the high bit of the second byte and the 2 high bits
	 * of the third byte an additional xor is applied to the output.
	 **/


	uint8_t check = 0;
	check = data[0];
	check ^= rotl8(data[1], 1);
	check ^= rotl8(data[2], 2);

	switch (((data[1] >> 5) & 0x04) | ((data[2] >> 6) & 0x03)) {
	case 1:
	case 4:
		check = check ^ 0x18;
		break;
	case 2:
	case 7:
		check = check ^ 0x30;
		break;
	case 3:
	case 6:
		check = check ^ 0x28;
		break;
	}
	return check;
}


void irt_check_send_queue()
{
	uint16_t status;

	status = irtuart_check_tx(1);
//	if (status > 0x01FF) myDebug("Failed transmitting telegram, status0 0x%04x", status);
	if ((status & 0xFF00) == 0x0100) {
		// still busy transmitting
		return;
	}
	if ((status & 0xFF00) == 0x0300) {
		myDebug("Failed transmitting telegram, status1 %d", (status & 0xFF));
		// the transmission has failed, but we are ready to send a new packet
	}

	// check if we have something in the queue to send
	if (IRT_TxQueue.isEmpty()) {
		return;
	}

	// if we're preventing all outbound traffic, quit
	if (ems_getTxDisabled()) {
		IRT_TxQueue.shift(); // remove from queue
		return;
	}

	// get the first in the queue, which is at the head
	// we don't remove from the queue yet
	_IRT_TxTelegram IRT_TxTelegram = IRT_TxQueue.first();


	if (IRT_TxTelegram.msg_in_use < 1) { // no messages in buffer
		IRT_TxQueue.shift(); // remove from queue
		return;
	}
	if (IRT_TxTelegram.msg_in_use > IRT_MAX_SUB_MSGS) { // to many messages in buffer
		IRT_TxQueue.shift(); // remove from queue
		return;
	}

	// convert sub messages to single
	// buffer, out buffer is big enough
	// to hold all 5 msgs
	uint8_t out_buffer[IRT_MAXTXBUFFERSIZE];
	uint8_t pos;
	uint8_t i;
	pos = 0;
	for (i=0; i<IRT_TxTelegram.msg_in_use; i++) {
		// copy first 3 bytes
		out_buffer[pos++] = IRT_TxTelegram.data[i][0];
		out_buffer[pos++] = IRT_TxTelegram.data[i][1];
		out_buffer[pos++] = IRT_TxTelegram.data[i][2];
		// add checksum
		out_buffer[pos++] = irt_calc_checksum(&IRT_TxTelegram.data[i][0]);
		// add two fill bytes on status request
		if (IRT_TxTelegram.data[i][0] & 0x80) {
			// message with response
			out_buffer[pos++] = 0;
			out_buffer[pos++] = 0;
		}
	}
	if (EMS_Sys_Status.emsLogging == EMS_SYS_LOGGING_JABBER) {
		irt_dumpBuffer("irt_tx: ", out_buffer, pos);
	}
	// send msg to irq for transmittion
	status = irtuart_send_tx_buffer(IRT_TxTelegram.address, out_buffer, pos);

	IRT_TxQueue.shift(); // remove from queue

	if ((status & 0xFF00) == 0x0200) {
		myDebug("Failed transmitting telegram, status2 %d", (status & 0xFF));
	}

}

void irt_init_telegram(_IRT_TxTelegram *IRT_Tx, uint8_t address)
{
	memset(IRT_Tx, 0, sizeof(_IRT_TxTelegram));
	IRT_Tx->msg_in_use = 0;
	IRT_Tx->address = address;
}

uint8_t irt_add_sub_msg(_IRT_TxTelegram *IRT_Tx, uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t nr_of_data)
{
	uint8_t i;
	if (IRT_Tx->msg_in_use >= IRT_MAX_SUB_MSGS) return 0;

	i = IRT_Tx->msg_in_use;
	IRT_Tx->data[i][0] = cmd;
	if (nr_of_data > 0) {
		IRT_Tx->data[i][1] = data1;
	} else {
		IRT_Tx->data[i][1] = 0xA5;
	}
	if (nr_of_data > 1) {
		IRT_Tx->data[i][2] = data2;
	} else {
		IRT_Tx->data[i][2] = 0xF0;
	}
	IRT_Tx->msg_in_use++;
	return 1;
}

void irt_send_next_poll_to_boiler()
{
	_IRT_TxTelegram IRT_Tx;

	// only poll in tx_mode 5
	if (EMSESP_Settings.tx_mode != 5) return;

	IRT_Sys_Status.poll_step++;
#ifdef nuniet
	uint16_t burner_power = 0;
	uint8_t water_temp = 0x35; // default

	if (myESP.isMQTTHealthy()) {
		// set burner power
	} else {
		// fallback
	}


	if ((IRT_Sys_Status.req_water_temp > 20) && (IRT_Sys_Status.req_water_temp < 90)) {
		// boiler will start if power is bigger then 0x50
		// we have valid temp range of 20 till 90
		// this translates to a power index of 0x50 till 0xFF
		// power = 0x50 + (temp - 20) * 2,5)

		burner_power = (IRT_Sys_Status.req_water_temp - 20) * 5;
		burner_power = burner_power / 2;
		burner_power = 0x50 + burner_power;
		water_temp = IRT_Sys_Status.req_water_temp;
	}
#endif
	// prepare the next batch of status poll messages
	switch (IRT_Sys_Status.poll_step) {
	case 1:
	case 3:
	case 5:
	case 7:
	case 9:
	case 11:
		irt_init_telegram(&IRT_Tx, IRT_Sys_Status.my_address);
		irt_add_sub_msg(&IRT_Tx, 0x90, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x82, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0xA3, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0xA4, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x8A, 0, 0, 0);
		IRT_TxQueue.push(IRT_Tx);
		break;
	case 2:
		irt_init_telegram(&IRT_Tx, IRT_Sys_Status.my_address);
		irt_add_sub_msg(&IRT_Tx, 0x90, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x81, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x86, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x85, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x83, 0, 0, 0);
		IRT_TxQueue.push(IRT_Tx);
		break;
	case 4:
		irt_init_telegram(&IRT_Tx, IRT_Sys_Status.my_address);
		irt_add_sub_msg(&IRT_Tx, 0x90, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x73, 0x52, 0x25, 2);
		irt_add_sub_msg(&IRT_Tx, 0x78, 0x01, 0xFF, 2);

		// if we are at the minimum burn power, use the temp. limit of the boiler
		// this keel keep the pump running, but switches off the burner
		if ((IRT_Sys_Status.cur_set_burner_power > 0) &&
				(IRT_Sys_Status.cur_set_burner_power <= IRT_MIN_USABLE_BURN_POWER) &&
				(IRT_Sys_Status.req_water_temp >= IRT_MIN_FLOW_TEMP)) {
			irt_add_sub_msg(&IRT_Tx, 0x01, IRT_Sys_Status.req_water_temp, 0xF6, 2); // set max cv water temp
		} else {
			irt_add_sub_msg(&IRT_Tx, 0x01, EMSESP_Settings.max_flow_temp, 0xF6, 2); // set max cv water temp
		}
		IRT_TxQueue.push(IRT_Tx);
		break;
	case 6:
		irt_init_telegram(&IRT_Tx, IRT_Sys_Status.my_address);
		irt_add_sub_msg(&IRT_Tx, 0x90, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x73, 0x52, 0x25, 2);
		irt_add_sub_msg(&IRT_Tx, 0x78, 0x07, 0xFF, 2);
		irt_add_sub_msg(&IRT_Tx, 0x07, IRT_Sys_Status.cur_set_burner_power, 0xD0, 2); // set burner power
		IRT_TxQueue.push(IRT_Tx);
		break;
	case 8:
		irt_init_telegram(&IRT_Tx, IRT_Sys_Status.my_address);
		irt_add_sub_msg(&IRT_Tx, 0x90, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x73, 0x52, 0x25, 2);
		irt_add_sub_msg(&IRT_Tx, 0x78, 0x04, 0x00, 2);
		irt_add_sub_msg(&IRT_Tx, 0x04, 0x00, 0x4D, 2);
		IRT_TxQueue.push(IRT_Tx);
		break;
	case 10:
		irt_init_telegram(&IRT_Tx, IRT_Sys_Status.my_address);
		irt_add_sub_msg(&IRT_Tx, 0x90, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x73, 0x52, 0x25, 2);
		irt_add_sub_msg(&IRT_Tx, 0x78, 0x05, 0x04, 2);
		irt_add_sub_msg(&IRT_Tx, 0x05, 0x04, 0xE2, 2);
		IRT_TxQueue.push(IRT_Tx);
		break;
	case 12:
		irt_init_telegram(&IRT_Tx, IRT_Sys_Status.my_address);
		irt_add_sub_msg(&IRT_Tx, 0x90, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0x93, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0xC9, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0xF0, 0x01, 0xCD, 2);
		irt_add_sub_msg(&IRT_Tx, 0xF0, 0x01, 0xD8, 2);
		IRT_TxQueue.push(IRT_Tx);
		break;
	case 13:
		irt_init_telegram(&IRT_Tx, IRT_Sys_Status.my_address);
		irt_add_sub_msg(&IRT_Tx, 0xA6, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0xA8, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0xAA, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0xAB, 0, 0, 0);
		irt_add_sub_msg(&IRT_Tx, 0xAC, 0, 0, 0);
		IRT_TxQueue.push(IRT_Tx);
		break;

	default:
		IRT_Sys_Status.poll_step = 0;
		break;
	}
}

void irt_loop()
{
	unsigned long now_millis = millis();

	if ((now_millis - IRT_Sys_Status.last_send_check) >= IRT_Sys_Status.send_interval) {

		// only poll in tx_mode 5
		if (EMSESP_Settings.tx_mode == 5) {
			if ((now_millis - IRT_Sys_Status.last_boiler_poll) >= IRT_BOILER_POLL_TIMEOUT) {
				myDebug("No poll from boiler, check bus");
			} else {
				irt_send_next_poll_to_boiler();
			}
		}

		IRT_Sys_Status.last_send_check = now_millis;
	}
	// check send buffers
	if (EMSESP_Settings.tx_mode > 4) {
		irt_check_send_queue();
	}
}

void irt_setup_flow_temp_pid()
{
	pid_Init((EMSESP_Settings.flow_temp_P * SCALING_FACTOR) / 100,
				(EMSESP_Settings.flow_temp_I * SCALING_FACTOR) / 100,
				(EMSESP_Settings.flow_temp_D * SCALING_FACTOR) / 100,
				 &IRT_Sys_Status.flowPidData);

}

/**
 * Called every minute to update the burner power for the requested
 * flow temp.
 */
void irt_doFlowTempTicker()
{
	unsigned long now_millis = millis();

	myDebug_P(PSTR("flow ticker."));

	// we can only set burner power in active mode
	if (EMSESP_Settings.tx_mode != 5) {
		IRT_Sys_Status.cur_set_burner_power = 0;
		return;
	}

	// is there any request for flow water ?
	if (IRT_Sys_Status.req_water_temp < IRT_MIN_FLOW_TEMP) {
		IRT_Sys_Status.cur_set_burner_power = 0;
		return;
	}

	// if there is no MQTT connection, reset request
	if (!myESP.isMQTTHealthy()) {
		if (IRT_Sys_Status.req_water_temp >= IRT_MIN_FLOW_TEMP) {
			myDebug_P(PSTR("Resetting boiler flow temp. because of MQTT error."));
		}
		IRT_Sys_Status.req_water_temp = 0;
		IRT_Sys_Status.cur_set_burner_power = 0;
		return;
	}

	// make sure the current reported flow temp is valid
	if ((now_millis - IRT_Sys_Status.last_flow_update) >= IRT_BOILER_POLL_TIMEOUT) {
		IRT_Sys_Status.cur_set_burner_power = 0;
		return;
	}

	int16_t err, new_power;


	new_power = IRT_Sys_Status.cur_set_burner_power;

	err = pid_Controller(IRT_Sys_Status.req_water_temp, IRT_Sys_Status.cur_flow_temp, &IRT_Sys_Status.flowPidData);

	if (IRT_Sys_Status.cur_set_burner_power == 0) {
		// start burn cycle with a half of power
		new_power = 0x80;
		pid_Reset_Integrator(&IRT_Sys_Status.flowPidData);
	} else {
		new_power = new_power + err;
	}
	// limit power to valid range
	if (new_power < IRT_MIN_USABLE_BURN_POWER) new_power = IRT_MIN_USABLE_BURN_POWER; // any lower and the boiler stops running
	if (new_power > 0xCF) new_power = 0xCF; // max power

	myDebug_P(PSTR("Req %d C Cur %d Err %d old pwr: %d new pwr: %d (0x%02x)"), IRT_Sys_Status.req_water_temp, IRT_Sys_Status.cur_flow_temp, err, IRT_Sys_Status.cur_set_burner_power, new_power, new_power);

	IRT_Sys_Status.cur_set_burner_power = (uint8_t)new_power;

}

/**
 * Set the boiler flow temp
 */
void irt_setFlowTemp(uint8_t temperature) {

	if (temperature > EMSESP_Settings.max_flow_temp) {
		myDebug_P(PSTR("Maximum boiler flow temperature is %d C, ignoring set of %d C"), EMSESP_Settings.max_flow_temp, temperature);
		return;
	}
	if (EMSESP_Settings.tx_mode == 5) {
		myDebug_P(PSTR("Setting boiler flow temperature to %d C"), temperature);
		IRT_Sys_Status.req_water_temp = temperature;
		if (IRT_Sys_Status.req_water_temp < IRT_MIN_FLOW_TEMP) {
			// reset burner directly
			IRT_Sys_Status.cur_set_burner_power = 0;
		}
	} else {
		myDebug_P(PSTR("Cannot set boiler flow temperature to %d C, not in active mode"), temperature);
		IRT_Sys_Status.req_water_temp = 0;
	}
}
/**
 * Activate / De-activate the Warm Water 0x33
 * true = on, false = off
 */
void irt_setWarmWaterActivated(bool activated) {
    myDebug_P(PSTR("Setting boiler warm water %s"), activated ? "on" : "off");
}

void irt_setFlowPID(int8_t flow_p, int8_t flow_i, int8_t flow_d)
{
	int8_t changed;
	if (flow_p < 0) {
		myDebug_P(PSTR("Current flow PID: %d %d %d"), EMSESP_Settings.flow_temp_P, EMSESP_Settings.flow_temp_I, EMSESP_Settings.flow_temp_D);
		return;
	}
	changed = 0;
	if ((flow_p >= 0) && (flow_p <= 100)) {
		if ((uint16_t)flow_p != EMSESP_Settings.flow_temp_P) {
			changed = 1;
			EMSESP_Settings.flow_temp_P = (uint16_t)flow_p;
		}
	}
	if ((flow_i >= 0) && (flow_i <= 100)) {
		if ((uint16_t)flow_i != EMSESP_Settings.flow_temp_I) {
			changed = 1;
			EMSESP_Settings.flow_temp_I = (uint16_t)flow_i;
		}
	}
	if ((flow_d >= 0) && (flow_d <= 100)) {
		if ((uint16_t)flow_d != EMSESP_Settings.flow_temp_D) {
			changed = 1;
			EMSESP_Settings.flow_temp_D = (uint16_t)flow_d;
		}
	}
	if (changed) {
		irt_setup_flow_temp_pid();
		myDebug_P(PSTR("Changed to, flow PID: %d %d %d"), EMSESP_Settings.flow_temp_P, EMSESP_Settings.flow_temp_I, EMSESP_Settings.flow_temp_D);
	} else {
		myDebug_P(PSTR("No Change, Current flow PID: %d %d %d"), EMSESP_Settings.flow_temp_P, EMSESP_Settings.flow_temp_I, EMSESP_Settings.flow_temp_D);
	}

}
void irt_setMaxFlowTemp(int8_t temp)
{
	if (temp < 0) {
		myDebug_P(PSTR("Current max. flow Temp.: %d C."), EMSESP_Settings.max_flow_temp);
		return;
	}
	if (temp > 90) {
		myDebug_P(PSTR("Invalid temperature of %d C, ignoring."), temp);
		return;
	}
	if (temp == EMSESP_Settings.max_flow_temp) {
		myDebug_P(PSTR("New max. flow temperature is the same (%d C)."), temp);
	} else {
		myDebug_P(PSTR("Setting new max. flow temperature of %d C."), temp);
		EMSESP_Settings.max_flow_temp = (uint8_t)temp;
	}
}

void irt_set_water_temp(uint8_t wc, const char *setting, const char *value)
{
//	int sel_test = 0;
//	uint8_t ovr_len = 0;
	myDebug("Irt set water temp, wc %d\n", wc);

	if ((wc > 0) && (setting != NULL)) {
		uint8_t water_temp;
		water_temp = atoi(setting);
		IRT_Sys_Status.req_water_temp = water_temp;
		myDebug("Water temp set to %d (%02x)", water_temp, water_temp);
	}
//	if ((wc > 1) && (value != NULL)) {
//		ovr_len = atoi(value);
//	}
}
void irt_sendRawTelegram(char *telegram)
{
	_IRT_TxTelegram IRT_Tx;
	size_t i, j, len;
	char conv[4];
	uint8_t bin_conv;
	uint8_t address;
	uint8_t state;
	uint8_t cmd, data_len, data1, data2;

	len = strlen(telegram);
	if (len < 4) {
		myDebug("send raw telegram: send 1 90 80 F001");
		return;
	}

	j = 0;
	conv[0] = conv[1] = 0;
	address = 1;
	state = 0;
	cmd = 0;
	data1 = 0;
	data2 = 0;
	data_len = 0;
	for (i=0; i<=len; i++) {
		if (telegram[i] > ' ') {
			conv[j++] = telegram[i];
		}
		if (((telegram[i] <= ' ') && (j > 0)) || (j >= 2)) {
			conv[2] = 0;
			bin_conv = (uint8_t)strtol(conv, 0, 16);
			if (state == 0) {
				address = bin_conv;
				irt_init_telegram(&IRT_Tx, address);
				state = 1;
			} else {
				switch (state) {
				case 1:
					cmd = bin_conv;
					data_len = 0;
					state = 2;
					break;
				case 2:
					data1 = bin_conv;
					data_len = 1;
					state = 3;
					break;
				case 3:
					data2 = bin_conv;
					data_len = 2;
					state = 4;
					break;
				}
			}
			conv[0] = conv[1] = 0;
			j = 0;
		}
		if ((state > 1) && (telegram[i] <= ' ')) {
			irt_add_sub_msg(&IRT_Tx, cmd, data1, data2, data_len);
			state = 1;
		}
	}
	if (state >= 1)	IRT_TxQueue.push(IRT_Tx);
}


void irt_init_uart()
{
	// init of irt, setup uart
	irtuart_init();
}

void irt_init()
{

	memset(&IRT_Sys_Status, 0, sizeof(_IRT_Sys_Status));
	IRT_Sys_Status.last_send_check = 0;
	IRT_Sys_Status.send_interval = 4000; // in milliseconds
	IRT_Sys_Status.poll_step = 0;
	IRT_Sys_Status.my_address = 1;
	IRT_Sys_Status.req_water_temp = 0;

	IRT_Sys_Status.cur_flow_temp = 20; // setup a default
	IRT_Sys_Status.last_flow_update = 0;
	IRT_Sys_Status.cur_set_burner_power = 0;

	irt_setup_flow_temp_pid();
	updateFlowTempTimer.attach(60, irt_doFlowTempTicker); // update requested flow temp
}

void irt_setup()
{
	irtuart_setup();
}

void irt_start()
{
	// called if OTA update is done
	irtuart_start();
}
void irt_stop()
{
	// called before OTA update starts
	irtuart_stop();
}

/* --- stuff t make the compiler happy --- */

_EMS_SYS_LOGGING ems_getLogging() {
    return EMS_Sys_Status.emsLogging;
}

bool ems_getBusConnected() {
    if ((millis() - EMS_Sys_Status.emsRxTimestamp) > EMS_BUS_TIMEOUT) {
        EMS_Sys_Status.emsBusConnected = false;
    }
    return EMS_Sys_Status.emsBusConnected;
}
void ems_setTxDisabled(bool b) {
    EMS_Sys_Status.emsTxDisabled = b;
}

bool ems_getTxDisabled() {
    return (EMS_Sys_Status.emsTxDisabled);
}

void ems_setTxMode(uint8_t mode) {
	EMS_Sys_Status.emsTxMode = mode;

}

bool ems_getThermostatEnabled() {
    return (EMS_Thermostat.device_id != EMS_ID_NONE);
}
void ems_setLogging(_EMS_SYS_LOGGING loglevel, bool silent) {
    if (loglevel <= EMS_SYS_LOGGING_MQTT) {
        EMS_Sys_Status.emsLogging = loglevel;
        if (silent) {
            return; // don't print to telnet/serial
        }

        if (loglevel == EMS_SYS_LOGGING_NONE) {
            myDebug_P(PSTR("System Logging set to None"));
        } else if (loglevel == EMS_SYS_LOGGING_BASIC) {
            myDebug_P(PSTR("System Logging set to Basic"));
        } else if (loglevel == EMS_SYS_LOGGING_VERBOSE) {
            myDebug_P(PSTR("System Logging set to Verbose"));
        } else if (loglevel == EMS_SYS_LOGGING_THERMOSTAT) {
            myDebug_P(PSTR("System Logging set to Thermostat only"));
        } else if (loglevel == EMS_SYS_LOGGING_SOLARMODULE) {
            myDebug_P(PSTR("System Logging set to Solar Module only"));
        } else if (loglevel == EMS_SYS_LOGGING_RAW) {
            myDebug_P(PSTR("System Logging set to Raw mode"));
        } else if (loglevel == EMS_SYS_LOGGING_JABBER) {
            myDebug_P(PSTR("System Logging set to Jabber mode"));
        } else if (loglevel == EMS_SYS_LOGGING_MQTT) {
            myDebug_P(PSTR("System Logging set to MQTT"));        }
    }
}


/**
 * Set the warm water temperature 0x33
 */
void ems_setWarmWaterTemp(uint8_t temperature) {
    // check for invalid temp values
    if ((temperature < 30) || (temperature > EMS_BOILER_TAPWATER_TEMPERATURE_MAX)) {
        return;
    }

    myDebug_P(PSTR("Setting boiler warm water temperature to %d C"), temperature);
}

bool ems_getBoilerEnabled() {
    return (EMS_Boiler.device_id != EMS_ID_NONE);
}

/*
 * Add one or more flags to the current flags.
 */
void ems_Device_add_flags(unsigned int flags) {
    EMS_Sys_Status.emsRefreshedFlags |= flags;
}
/*
 * Check if the current flags include all of the specified flags.
 */
bool ems_Device_has_flags(unsigned int flags) {
    return (EMS_Sys_Status.emsRefreshedFlags & flags) == flags;
}
/*
 * Remove one or more flags from the current flags.
 */
void ems_Device_remove_flags(unsigned int flags) {
    EMS_Sys_Status.emsRefreshedFlags &= ~flags;
}
