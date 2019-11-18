/**
 * irt.cpp
 *
 * Handles all the processing of the IRT messages
 *
 * Paul Derbyshire - https://github.com/proddy/EMS-ESP
 */

#include <stdio.h>
#include "irt.h"
#include "ems_devices.h"
#include "MyESP.h"
//#include "ems_devices.h"
#include "irtuart.h"
//#include <CircularBuffer.h> // https://github.com/rlogiacco/CircularBuffer

// MyESP class for logging to telnet and serial
#define myDebug(...) myESP.myDebug(__VA_ARGS__)
#define myDebug_P(...) myESP.myDebug_P(__VA_ARGS__)

extern _EMS_Boiler EMS_Boiler;

char * _hextoa(uint8_t value, char * buffer);
char * _smallitoa(uint8_t value, char * buffer);
char * _smallitoa3(uint16_t value, char * buffer);


#define MAX_KNOWN_MSG_LEN 16
typedef struct {
	uint8_t			length;
	uint8_t			data[MAX_KNOWN_MSG_LEN];
	uint8_t			mask[MAX_KNOWN_MSG_LEN];
} ty_known_msg;


const ty_known_msg known_msg[] = {
#ifdef junk
	{	.length = 12, .data = {0x73, 0x52, 0x25, 0x43, 0x78, 0x01, 0xFF, 0xAD, 0x01, 0x51, 0xF6, 0x50}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 12, .data = {0x73, 0x52, 0x25, 0x43, 0x78, 0x04, 0x00, 0x70, 0x04, 0x00, 0x4D, 0x29}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 12, .data = {0x73, 0x52, 0x25, 0x43, 0x78, 0x05, 0x04, 0x62, 0x05, 0x04, 0xE2, 0xAE}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	/* status set from thermostat ?? */
	{	.length = 12, .data = {0x73, 0x52, 0x25, 0x43, 0x78, 0x07, 0xFF, 0xA1, 0x07, 0x00, 0xD0, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00} },
#endif // junk
	{	.length = 4, .data = {0x01, 0x51, 0xF6, 0x50}, .mask = {0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 4, .data = {0x04, 0x00, 0x4D, 0x29}, .mask = {0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 4, .data = {0x05, 0x04, 0xE2, 0xAE}, .mask = {0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 4, .data = {0x07, 0x00, 0xD0, 0x00}, .mask = {0xFF, 0x00, 0xFF, 0x00} },

	{	.length = 4, .data = {0x73, 0x52, 0x25, 0x43}, .mask = {0xFF, 0xFF, 0xFF, 0xFF} },

	{	.length = 4, .data = {0x78, 0x01, 0xFF, 0xAD}, .mask = {0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 4, .data = {0x78, 0x04, 0x00, 0x70}, .mask = {0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 4, .data = {0x78, 0x05, 0x04, 0x62}, .mask = {0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 4, .data = {0x78, 0x07, 0xFF, 0xA1}, .mask = {0xFF, 0xFF, 0xFF, 0xFF} },

	{	.length = 5, .data = {0x81, 0xC3, 0x79, 0xE3, 0x51}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },

	/* last byte is response from boiler, looks like status ? */
	{	.length = 5, .data = {0x82, 0xC3, 0x79, 0xE0, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0x82, 0xF3, 0x04, 0x6D, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0x82, 0x4A, 0x0A, 0x3E, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0x82, 0x28, 0x2B, 0x7E, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0x82, 0x6B, 0xB8, 0x86, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },

	{	.length = 5, .data = {0x83, 0xC3, 0x79, 0xE1, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0x85, 0xC3, 0x79, 0xE7, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0x86, 0xC3, 0x79, 0xE4, 0xE9}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },


	{	.length = 5, .data = {0x8A, 0xC3, 0x79, 0xE8, 0xFE}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x8A, 0xA1, 0x98, 0x83, 0xFE}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x8A, 0x5B, 0x5D, 0x51, 0xFE}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x8A, 0x18, 0x71, 0x67, 0xFE}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x8A, 0x96, 0xAE, 0x35, 0xFE}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },


	{	.length = 5, .data = {0x90, 0xAE, 0x5F, 0xB0, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x90, 0x62, 0x54, 0x1D, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x90, 0xC6, 0x8F, 0x0B, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x90, 0xF9, 0x05, 0x6F, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x90, 0x32, 0xE3, 0x53, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x90, 0x90, 0xE0, 0x02, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x90, 0xE6, 0xD0, 0x2E, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x90, 0x40, 0x5A, 0x61, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0x90, 0xC3, 0x79, 0xF2, 0xCF}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },

	/* last bytes is 0x00 or 0xFF (pump on off ??)*/
	{	.length = 5, .data = {0x93, 0x73, 0x0E, 0x4D, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },


	/* last byte means ???? */
	{	.length = 5, .data = {0xA3, 0xC3, 0x79, 0xC1, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0xA3, 0x48, 0xBF, 0xFD, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0xA3, 0x1C, 0xED, 0x04, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0xA3, 0x15, 0x5B, 0xFC, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0xA3, 0x07, 0xD3, 0xCA, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },

	/* last byte reports water temp */
	{	.length = 5, .data = {0xA4, 0xC3, 0x79, 0xC6, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0xA4, 0x73, 0x72, 0x93, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0xA4, 0x7D, 0xAC, 0xDC, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0xA4, 0x5D, 0x17, 0x42, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },
	{	.length = 5, .data = {0xA4, 0xCB, 0x67, 0xAE, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },

	/* last bytes is 0x00, 1 2 3 4 5 (burner status ??)*/
	{	.length = 5, .data = {0xC9, 0xC3, 0x79, 0xAB, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0x00} },

	{	.length = 5, .data = {0xF0, 0x01, 0xCD, 0xED, 0x00}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },
	{	.length = 5, .data = {0xF0, 0x01, 0xD8, 0xB9, 0x05}, .mask = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF} },

	{ 	.length = 0, .data = {0x00}, .mask ={0x00} }
};

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

	// show some EMS_Sys_Status entries
	strlcat(output_str, _hextoa(EMS_Sys_Status.emsRxStatus, buffer), sizeof(output_str));
	strlcat(output_str, " ", sizeof(output_str));
	strlcat(output_str, _hextoa(EMS_Sys_Status.emsTxStatus, buffer), sizeof(output_str));
	strlcat(output_str, " ", sizeof(output_str));
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

/*
 * Check message agains the list of messages we know, or choice to ignore
 */
void irt_logRawMessage(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{

	if (EMS_Sys_Status.emsLogging != EMS_SYS_LOGGING_RAW) return;

	/* check against the known message list */
	uint8_t i, j, found, found2;
	i=0;
	found = 0;
	while ((found == 0) && (known_msg[i].length != 0)) {
		if (known_msg[i].length == length) {
			found2 = 1;
			for (j=0; ((j<length) && (found2)); j++) {
				if ((data[j] & known_msg[i].mask[j]) == known_msg[i].data[j]) {
					// match
				} else {
					found2 = 0;
				}
			}
			found = found2;
		}
		i++;
	}
	// message was found, do not log
	if (found) {
		irt_dumpBuffer("irt_raw: ", data, length);
	} else {
		irt_dumpBuffer("irt_new: ", data, length);
	}
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
	/*  0  1  2  3 */
	/* 07 00 D0 6C */
	/* 07 ss D0 ss*/

	EMS_Thermostat.model_id        = EMS_MODEL_FR10;
	EMS_Thermostat.device_id       = 0x01;
	EMS_Thermostat.write_supported = EMS_THERMOSTAT_WRITE_NO;
	EMS_Thermostat.product_id      = 0x01;
	strlcpy(EMS_Thermostat.version, "1.0", sizeof(EMS_Thermostat.version));
	uint8_t hc = 0;
	EMS_Thermostat.hc[hc].active = true;
	EMS_Thermostat.hc[hc].setpoint_roomTemp = (data[1] * 3);

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
	bit 1: 0x02 - ?? (seems to be 1 in maintenace mode)
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

	return 0;
}

uint8_t irt_handle_0xA4(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* A4 5D 17 42 19 */
	/* A4 ?? ?? ?? ww */
	EMS_Boiler.curFlowTemp = (data[4] * 10);
	return 0;
}

uint8_t irt_handle_0xC9(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* diff temp between in and out ??/ */

	return 0;
}

uint8_t irt_handle_0xF0(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* extra msgs */

	return 0;
}

uint8_t irt_handleMsg(_IRT_RxTelegram *msg, uint8_t *data, uint8_t length)
{
	/* messages are 4 or 5 bytes */
	if (length < 4)
		return 10;
	if ((data[0] & 0x80) && (length < 5))
		return 10;
	EMS_Sys_Status.emsRxPgks++;

	irt_update_status(msg, data, length);

//	if (data[0] != 0xF0) return 0;
	irt_logRawMessage(msg, data, length);


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
	case 0xC9:
		return irt_handle_0xC9(msg, data, length);
		break;
	case 0xF0:
		return irt_handle_0xF0(msg, data, length);
		break;
	}
	return 0;
}
#ifdef junk
uint8_t irt_parseSection(_IRT_RxTelegram *msg, uint8_t *section, uint8_t length)
{
	// parse a section of the data
//	irt_dumpBuffer("irt_sec: ", section, length);
	switch (msg->section_count) {
	case 0:
		// start of msg should be 01 01 FE
		// Poll from boiler (first 01)
		// reply from thermostat (second 01)
		// inverse reply from boiler (last fe)
		if ((length < 2) || (section[0] != 0x01) || (section[1] != 0x01)/* || (section[2] != 0xFE)*/) {
			// Invalid device or static noise ?
			irt_dumpBuffer("irt_unk_start: ", section, length);
			return 10;
		} else {
			// valid start
			msg->section_count++;
			msg->device_nr = section[1];
		}
		break;
	case 1:
		// second message is always from thermostat to boiler
		// it always start with 90
		if ((length != 5) || (section[0] != 0x90)/* || (section[4] != 0xCF)  || (section[5] != 0x30) */) {
			// invalid start msg
			irt_dumpBuffer("irt_unk_start: ", section, length);
			return 10;
		} else {
			/* mark the bus as in-sync */
			EMS_Sys_Status.emsRxTimestamp  = msg->timestamp; // timestamp of last read
			EMS_Sys_Status.emsBusConnected = true;
			EMS_Sys_Status.emsIDMask = 0x00;
			EMS_Boiler.device_id = EMS_ID_BOILER;
			EMS_Sys_Status.emsPollFrequency = 500000; // poll in micro secs
			EMS_Sys_Status.emsTxCapable = true;
			msg->section_count++;
		}
		irt_handleMsg(msg, section, length);
		break;
	default:
		// we, for now, assume the first byte is some sort of msg type ?
		irt_handleMsg(msg, section, length);

//		if (length >= 6) {
//			switch (section[0]) {
//			case 0x73:
//			case 0x82:
//			case 0x83:
//			case 0xA3:
//			case 0xA4:
//				irt_dumpBuffer("irt_real_msg: ", section, length);
//				break;
//			}
//		}
		break;
	}
	return 0;
}
#endif // junk

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
	if ((telegram[0] == 0x01) && (telegram[1] == 0x01) && (telegram[2] == 0xFE)) {
		// this is a message for device 01
	} else {
		// this is a message for another device ?
		if (telegram[0] > 0x03) {
			EMS_Sys_Status.emxCrcErr++;
			irt_dumpBuffer("irt_crcErr2: ", telegram, length);
		}

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
			// should we drop the whole packet ??
			EMS_Sys_Status.emxCrcErr++;
			irt_dumpBuffer("irt_crcErr3: ", telegram, length);
			i++;
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

#ifdef junk

	i = 1;
	j = 0;
	irt_buffer[j++] = telegram[0];
	ret = 0;
	while ((i + 1) < length) {
		if (telegram[i] == (0xFF - telegram[i+1])) {
			irt_buffer[j++] = telegram[i];
			//irt_buffer[j++] = telegram[i+1];
			if (j > 0) {
				// flush msg
				ret = irt_parseSection(&irtMsg, irt_buffer, j);
				j = 0;
				if (ret)
					break;
			}
			j = 0;
			i+=2;
		} else if (telegram[i] == telegram[i+1]) {
			irt_buffer[j++] = telegram[i];
			i += 2;
		} else {
			i++;
		}
	}
	if (j > 0) {
		// flush msg
		ret = irt_parseSection(&irtMsg, irt_buffer, j);
	}
#endif
//#ifdef include_later
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
//#endif
#ifdef nuniet

	static _IRT_RxTelegram IRT_RxTelegram; // create the Rx package
	IRT_RxTelegram.telegram  = telegram;
	IRT_RxTelegram.timestamp = millis();
	IRT_RxTelegram.length    = length;
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
