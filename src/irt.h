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
#include "pid.h"

// clang-format on


//#define IRT_MIN_TELEGRAM_LENGTH 6  // minimal length for a validation telegram, including CRC
#define IRT_MAX_TELEGRAM_LENGTH 64 // max length of a telegram, for Rx and Tx.
//#define IRT_MAX_SINGLE_TELEGRAM 5 // Max length of a single telegram

// define maximum setable tapwater temperature
//#define IRT_BOILER_TAPWATER_TEMPERATURE_MAX 60

#define IRT_TX_TELEGRAM_QUEUE_MAX 50 // max size of Tx FIFO queue. Number of Tx records to send.

#define IRT_BOILER_POLL_TIMEOUT 30000 // If we do not receive a boiler poll in 30s stop sending data
#define IRT_MIN_USABLE_BURN_POWER 0x4d // Any lower and the pump stops.
#define IRT_MIN_FLOWTEMP 10 // any request below 10 C is assumed inactive

/* iRT UART transfer status */
typedef enum {
    IRT_RX_STATUS_IDLE,
    IRT_RX_STATUS_BUSY // Rx package is being received
} _IRT_RX_STATUS;


// status/counters since last power on
typedef struct {
	_IRT_RX_STATUS		irtRxStatus;
	unsigned long		last_send_check;			// last timestamp when the send buffer where checked
	unsigned long		send_interval;				// minimum interval between checks
	uint8_t				poll_step;					// the status poll is send in several steps
	uint8_t				my_address;					// address used to identify myself
	uint8_t				req_water_temp;			// requested water temperature
	uint8_t				warm_water_mode;			// 0 - warm water prod off, 1- warm water prod. on
	uint8_t				cur_set_burner_power;	// Current burner power we have set
	unsigned long		last_boiler_poll;			// last time
	uint8_t				cur_flowtemp;				// last reported flow temp in Celsius
	unsigned long		last_flow_update;			// last time the flow temp was reported
	struct PID_DATA	flowPidData;				// PID calculation for flow temp.
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

void irt_setFlowTemp(uint8_t temperature);
void irt_setWarmWaterActivated(bool activated);
char *irt_format_flowtemp_pid_text(char *buf, size_t buf_len);
void irt_setFlowPID(float flow_p, float flow_i, float flow_d);
void irt_setMaxFlowTemp(int8_t temp);

void irt_init_uart();
void irt_init();
void irt_setup();
void irt_stop();
void irt_start();
void irt_loop();
void irt_sendRawTelegram(char * telegram);
void irt_set_water_temp(uint8_t wc, const char *setting, const char *value);

/* ----- ems stuff to make the compiler happy ---- */
// EMS bus IDs
#define EMS_BUSID_DEFAULT 0x0B // Default 0x0B (Service Key)

#define EMS_TXMODE_DEFAULT 1 // Default (was previously known as tx_mode 2 in v1.8.x)
// default values for null values
#define EMS_VALUE_BOOL_ON 0x01         // boolean true
#define EMS_VALUE_BOOL_ON2 0xFF        // boolean true, EMS sometimes uses 0xFF for TRUE
#define EMS_VALUE_BOOL_OFF 0x00        // boolean false
#define EMS_VALUE_INT_NOTSET 0xFF      // for 8-bit unsigned ints/bytes
#define EMS_VALUE_SHORT_NOTSET -32000  // was -32768 for 2-byte signed shorts
#define EMS_VALUE_USHORT_NOTSET 32000  // was 0x8000 for 2-byte unsigned shorts
#define EMS_VALUE_LONG_NOTSET 0xFFFFFF // for 3-byte longs
#define EMS_VALUE_BOOL_NOTSET 0xFE     // random number that's not 0, 1 or FF

// Fixed EMS IDs
#define EMS_ID_ME 0x0B      // our device, hardcoded as the "Service Key"
#define EMS_ID_BOILER 0x08  // all UBA Boilers have 0x08
#define EMS_ID_SM 0x30      // Solar Module SM10, SM100 and ISM1
#define EMS_ID_HP 0x38      // HeatPump
#define EMS_ID_GATEWAY 0x48 // KM200 Web Gateway

#define EMS_ID_NONE 0x00 // used as a dest in broadcast messages and empty device IDs


#define EMS_SYS_DEVICEMAP_LENGTH 15 // size of the 0x07 telegram data part which stores all active EMS devices

#define EMS_THERMOSTAT_WRITE_YES true
#define EMS_THERMOSTAT_WRITE_NO false

#define EMS_THERMOSTAT_DEFAULTHC 1 // default heating circuit is 1

// define maximum setable tapwater temperature
#define EMS_BOILER_TAPWATER_TEMPERATURE_MAX 60

#define IRT_FLOWTEMP_PID_P_DEFAULT 50
#define IRT_FLOWTEMP_PID_I_DEFAULT 10
#define IRT_FLOWTEMP_PID_D_DEFAULT 0

#define IRT_MAX_FLOWTEMP_DEFAULT 60



/* EMS logging */
typedef enum {
    EMS_SYS_LOGGING_NONE,        // no messages
    EMS_SYS_LOGGING_RAW,         // raw data mode
    EMS_SYS_LOGGING_WATCH,       // watch a specific type ID
    EMS_SYS_LOGGING_BASIC,       // only basic read/write messages
    EMS_SYS_LOGGING_THERMOSTAT,  // only telegrams sent from thermostat
    EMS_SYS_LOGGING_SOLARMODULE, // only telegrams sent from solarmodule
    EMS_SYS_LOGGING_VERBOSE,     // everything
    EMS_SYS_LOGGING_JABBER,      // lots of debug output...
    EMS_SYS_LOGGING_DEVICE,       // watch the device ID
    EMS_SYS_LOGGING_MQTT         // start logging the MQTT messages
} _EMS_SYS_LOGGING;

#define EMS_SYS_LOGGING_DEFAULT EMS_SYS_LOGGING_NONE

// Known EMS devices
typedef enum {
    EMS_MODEL_NONE, // unset
    EMS_MODEL_ALL,  // common for all devices

    // heatpump
    EMS_MODEL_HP,

    // solar module
    EMS_MODEL_SM,

    // boiler
    EMS_MODEL_UBA,

    // and the thermostats
    EMS_MODEL_ES73,
    EMS_MODEL_RC10,
    EMS_MODEL_RC20,
    EMS_MODEL_RC20F,
    EMS_MODEL_RC30,
    EMS_MODEL_RC35,
    EMS_MODEL_EASY,
    EMS_MODEL_RC300,
    EMS_MODEL_CW100,
    EMS_MODEL_1010,
    EMS_MODEL_OT,
    EMS_MODEL_FW100,
    EMS_MODEL_FR10,
    EMS_MODEL_FR100,
    EMS_MODEL_FR110,
    EMS_MODEL_FW120,

    // mixing devices
    EMS_MODEL_MM100

} _EMS_MODEL_ID;

// flags for triggering changes when EMS data is received
typedef enum : uint8_t {
    EMS_DEVICE_UPDATE_FLAG_NONE       = 0,
    EMS_DEVICE_UPDATE_FLAG_BOILER     = (1 << 0),
    EMS_DEVICE_UPDATE_FLAG_THERMOSTAT = (1 << 1),
    EMS_DEVICE_UPDATE_FLAG_MIXING     = (1 << 2),
    EMS_DEVICE_UPDATE_FLAG_SOLAR      = (1 << 3),
    EMS_DEVICE_UPDATE_FLAG_HEATPUMP   = (1 << 4)
} _EMS_DEVICE_UPDATE_FLAG;

/*
 * Telegram package defintions
 */
typedef struct {
    // settings
    uint8_t      device_id; // this is typically always 0x08
    uint8_t      device_flags;
    const char * device_desc_p;
    uint8_t      product_id;
    char         version[10];

    uint8_t brand; // 0=unknown, 1=bosch, 2=junkers, 3=buderus, 4=nefit, 5=sieger, 11=worcester

    // UBAParameterWW
    uint8_t wWActivated;     // Warm Water activated
    uint8_t wWSelTemp;       // Warm Water selected temperature
//    uint8_t wWCircPump;      // Warm Water circulation pump Available
//    uint8_t wWDesinfectTemp; // Warm Water desinfection temperature
    uint8_t wWComfort;       // Warm water comfort or ECO mode

    // UBAMonitorFast
    uint8_t  selFlowTemp;        // Selected flow temperature
    uint16_t curFlowTemp;        // Current flow temperature
    uint16_t wwStorageTemp1;     // warm water storage temp 1
    uint16_t wwStorageTemp2;     // warm water storage temp 2
    uint16_t retTemp;            // Return temperature
    uint8_t  burnGas;            // Gas on/off
    uint8_t  fanWork;            // Fan on/off
    uint8_t  ignWork;            // Ignition on/off
    uint8_t  heatPmp;            // Circulating pump on/off
    uint8_t  wWHeat;             // 3-way valve on WW
//    uint8_t  wWCirc;             // Circulation on/off
    uint8_t  selBurnPow;         // Burner max power
//    uint8_t  curBurnPow;         // Burner current power
//    uint16_t flameCurr;          // Flame current in micro amps
//    uint8_t  sysPress;           // System pressure
    char     serviceCodeChar[3]; // 2 character status/service code
    uint16_t serviceCode;        // error/service code

    // UBAMonitorSlow
    int16_t  extTemp;     // Outside temperature
    uint16_t boilTemp;    // Boiler temperature
    uint16_t exhaustTemp; // Exhaust temperature
    uint8_t  pumpMod;     // Pump modulation
    uint32_t burnStarts;  // # burner starts
    uint32_t burnWorkMin; // Total burner operating time
    uint32_t heatWorkMin; // Total heat operating time
    uint16_t switchTemp;  // Switch temperature

    // UBAMonitorWWMessage
    uint16_t wWCurTmp;  // Warm Water current temperature
//    uint32_t wWStarts;  // Warm Water # starts
//    uint32_t wWWorkM;   // Warm Water # minutes
    uint8_t  wWOneTime; // Warm Water one time function on/off
//    uint8_t  wWCurFlow; // Warm Water current flow in l/min

    // UBATotalUptimeMessage
    uint32_t UBAuptime; // Total UBA working hours

    // UBAParametersMessage
    uint8_t heating_temp; // Heating temperature setting on the boiler
//    uint8_t pump_mod_max; // Boiler circuit pump modulation max. power
//    uint8_t pump_mod_min; // Boiler circuit pump modulation min. power

    // calculated values
    uint8_t tapwaterActive; // Hot tap water is on/off
    uint8_t heatingActive;  // Central heating is on/off

} _EMS_Boiler;
// Thermostat data
typedef struct {
    uint8_t            device_id; // the device ID of the thermostat
    uint8_t            model_id;  // thermostat model
    uint8_t            product_id;
    char               version[10];
    char               datetime[25]; // HH:MM:SS DD/MM/YYYY
    bool               write_supported;
//    _EMS_Thermostat_HC hc[EMS_THERMOSTAT_MAXHC]; // array for the 4 heating circuits
} _EMS_Thermostat;


typedef struct {
    uint8_t dallas_sensors; // count of dallas sensors

    // custom params
    bool    shower_timer;      // true if we want to report back on shower times
    bool    shower_alert;      // true if we want the alert of cold water
    bool    led;               // LED on/off
    bool    listen_mode;       // stop automatic Tx on/off
    int16_t publish_time;      // frequency of MQTT publish in seconds, -1 for off
    uint8_t led_gpio;          // pin for LED
    uint8_t dallas_gpio;       // pin for attaching external dallas temperature sensors
    bool    dallas_parasite;   // on/off is using parasite
    uint8_t tx_mode;           // TX mode 1,2 or 3
    uint8_t bus_id;            // BUS ID, defaults to 0x0B for the service key
    uint8_t master_thermostat; // Product ID of master thermostat to use, 0 for automatic
    char *  known_devices;     // list of known deviceIDs for quick boot

	// PID flow settings
	uint16_t	flowtemp_P;		// P value of PID for flow temp control
	uint16_t	flowtemp_I;		// I value of PID for flow temp control
	uint16_t	flowtemp_D;		// D value of PID for flow temp control

	uint8_t	max_flowtemp;		// safety value, system will always limit to this max temp


} _EMSESP_Settings;

// status/counters since last power on
typedef struct {
//    _EMS_RX_STATUS   emsRxStatus;
//    _EMS_TX_STATUS   emsTxStatus;
    uint16_t         emsRxPgks;           // # successfull received
    uint16_t         emsTxPkgs;           // # successfull sent
    uint16_t         emxCrcErr;           // CRC errors
    bool             emsPollEnabled;      // flag enable the response to poll messages
    _EMS_SYS_LOGGING emsLogging;          // logging
    uint16_t         emsLogging_ID;       // the type or device ID to watch
    uint8_t          emsRefreshedFlags;   // fresh data, needs to be pushed out to MQTT
    bool             emsBusConnected;     // is there an active bus
    uint32_t         emsRxTimestamp;      // timestamp of last EMS message received
    uint32_t         emsPollFrequency;    // time between EMS polls
    bool             emsTxCapable;        // able to send via Tx
    bool             emsTxDisabled;       // true to prevent all Tx
    uint8_t          txRetryCount;        // # times the last Tx was re-sent
    uint8_t          emsIDMask;           // Buderus: 0x00, Junkers: 0x80
    uint8_t          emsPollAck[1];       // acknowledge buffer for Poll
    uint8_t          emsTxMode;           // Tx mode 1, 2 or 3
    uint8_t          emsbusid;            // EMS bus ID, default 0x0B for Service Key
    uint8_t          emsMasterThermostat; // product ID for the default thermostat to use
} _EMS_Sys_Status;

// for consolidating all types
typedef struct {
    uint8_t model_type; // 1=boiler, 2=thermostat, 3=sm, 4=other, 5=unknown
    uint8_t product_id;
    uint8_t device_id;
    char    version[10];
    char    model_string[50];
} _Generic_Device;


bool             ems_getBusConnected();
_EMS_SYS_LOGGING ems_getLogging();
bool             ems_getTxDisabled();
void ems_setTxDisabled(bool b);
void ems_setTxMode(uint8_t mode);
void             ems_Device_add_flags(unsigned int flags);
bool             ems_Device_has_flags(unsigned int flags);
void             ems_Device_remove_flags(unsigned int flags);

bool             ems_getThermostatEnabled();
void ems_setLogging(_EMS_SYS_LOGGING loglevel, bool silent = false);

void ems_setWarmWaterTemp(uint8_t temperature);
bool ems_getBoilerEnabled();
