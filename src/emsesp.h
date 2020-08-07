/*
 * EMS-ESP - https://github.com/proddy/EMS-ESP
 * Copyright 2019  Paul Derbyshire
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef EMSESP_EMSESP_H
#define EMSESP_EMSESP_H

#include <Arduino.h>

#include <vector>
#include <queue>
#include <string>

#include <uuid/common.h>
#include <uuid/console.h>
#include <uuid/log.h>

#ifndef EMSESP_STANDALONE
#include <uuid/telnet.h>
#endif

#include <ESP8266React.h>
#include "EMSESPStatusService.h"
#include "EMSESPDevicesService.h"
#include "EMSESPSettingsService.h"

#include "emsdevice.h"
#include "emsfactory.h"
#include "telegram.h"
#include "mqtt.h"
#include "system.h"
#include "sensors.h"
#include "console.h"
#include "shower.h"
#include "roomcontrol.h"

#define WATCH_ID_NONE 0 // no watch id set

namespace emsesp {

class Shower; // forward declaration for compiler

class EMSESP {
  public:
    static void start();
    static void loop();

    static void publish_all_values();

#ifdef EMSESP_STANDALONE
    static void run_test(uuid::console::Shell & shell, const std::string & command); // only for testing
    static void dummy_mqtt_commands(const char * message);
    static void rx_telegram(const std::vector<uint8_t> & data);
    static void uart_telegram(const std::vector<uint8_t> & rx_data);
#endif

    static bool        process_telegram(std::shared_ptr<const Telegram> telegram);
    static std::string pretty_telegram(std::shared_ptr<const Telegram> telegram);

    static void send_read_request(const uint16_t type_id, const uint8_t dest);
    static void send_write_request(const uint16_t type_id,
                                   const uint8_t  dest,
                                   const uint8_t  offset,
                                   uint8_t *      message_data,
                                   const uint8_t  message_length,
                                   const uint16_t validate_typeid);
    static void send_raw_telegram(const char * data);
    static bool device_exists(const uint8_t device_id);

    static void device_info(const uint8_t unique_id, JsonObject & root);

    static uint8_t count_devices(const uint8_t device_type);

    static uint8_t actual_master_thermostat();
    static void    actual_master_thermostat(const uint8_t device_id);
    static uint8_t check_master_device(const uint8_t device_id, const uint16_t type_id, const bool read);

    static void show_device_values(uuid::console::Shell & shell);
    static void show_sensor_values(uuid::console::Shell & shell);

    static void show_devices(uuid::console::Shell & shell);
    static void clear_all_devices();

    static void show_ems(uuid::console::Shell & shell);

    static void add_context_menus();

    static void reset_tx();

    static void incoming_telegram(uint8_t * data, const uint8_t length);

    static const std::vector<Sensors::Device> sensor_devices() {
        return sensors_.devices();
    }

    enum Watch : uint8_t { WATCH_OFF, WATCH_ON, WATCH_RAW };
    static void     watch_id(uint16_t id);
    static uint16_t watch_id() {
        return watch_id_;
    }

    static void watch(uint8_t watch) {
        watch_ = watch; // 0=off, 1=on, 2=raw
        if (watch == WATCH_OFF) {
            watch_id_ = 0; // reset watch id if watch is disabled
        }
    }
    static uint8_t watch() {
        return watch_;
    }

    enum Bus_status : uint8_t { BUS_STATUS_CONNECTED = 0, BUS_STATUS_TX_ERRORS, BUS_STATUS_OFFLINE };
    static uint8_t bus_status();

    static bool tap_water_active() {
        return tap_water_active_;
    }

    static void tap_water_active(const bool tap_water_active) {
        tap_water_active_ = tap_water_active;
    }

    static void fetch_device_values(const uint8_t device_id = 0);

    static bool add_device(const uint8_t device_id, const uint8_t product_id, std::string & version, const uint8_t brand);

    static std::vector<std::unique_ptr<EMSdevice>> emsdevices;

    // services
    static Mqtt      mqtt_;
    static System    system_;
    static Sensors   sensors_;
    static Console   console_;
    static Shower    shower_;
    static RxService rxservice_;
    static TxService txservice_;

    // web controllers
    static ESP8266React          esp8266React;
    static EMSESPSettingsService emsespSettingsService;
    static EMSESPStatusService   emsespStatusService;
    static EMSESPDevicesService  emsespDevicesService;

    static uuid::log::Logger logger() {
        return logger_;
    }

  private:
    EMSESP() = delete;

    static uuid::log::Logger logger_;

    static std::string device_tostring(const uint8_t device_id);

    static void process_UBADevices(std::shared_ptr<const Telegram> telegram);
    static void process_version(std::shared_ptr<const Telegram> telegram);

    static constexpr uint32_t EMS_FETCH_FREQUENCY = 60000; // check every minute
    static uint32_t           last_fetch_;

    struct Device_record {
        uint8_t                     product_id;
        EMSdevice::DeviceType       device_type;
        const __FlashStringHelper * name;
        uint8_t                     flags;
    };

    static std::vector<Device_record> device_library_;

    static uint8_t  actual_master_thermostat_;
    static uint16_t watch_id_;
    static uint8_t  watch_;
    static bool     tap_water_active_;

    static uint8_t unique_id_count_;
};

} // namespace emsesp

#endif
