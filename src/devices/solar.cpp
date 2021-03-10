/*
 * EMS-ESP - https://github.com/proddy/EMS-ESP
 * Copyright 2020  Paul Derbyshire
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

#include "solar.h"

namespace emsesp {

REGISTER_FACTORY(Solar, EMSdevice::DeviceType::SOLAR);

uuid::log::Logger Solar::logger_{F_(solar), uuid::log::Facility::CONSOLE};

Solar::Solar(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand)
    : EMSdevice(device_type, device_id, product_id, version, name, flags, brand) {
    LOG_DEBUG(F("Adding new Solar module with device ID 0x%02X"), device_id);

    // telegram handlers
    if (flags == EMSdevice::EMS_DEVICE_FLAG_SM10) {
        register_telegram_type(0x0097, F("SM10Monitor"), true, [&](std::shared_ptr<const Telegram> t) { process_SM10Monitor(t); });
    }

    if (flags == EMSdevice::EMS_DEVICE_FLAG_SM100) {
        if (device_id == 0x2A) {
            register_telegram_type(0x07D6, F("SM100wwTemperature"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100wwTemperature(t); });
            register_telegram_type(0x07AA, F("SM100wwStatus"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100wwStatus(t); });
            register_telegram_type(0x07AB, F("SM100wwCommand"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100wwCommand(t); });
        } else {
            register_telegram_type(0xF9, F("ParamCfg"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100ParamCfg(t); });
            register_telegram_type(0x0358, F("SM100SystemConfig"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100SystemConfig(t); });
            register_telegram_type(0x035A, F("SM100SolarCircuitConfig"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100SolarCircuitConfig(t); });
            register_telegram_type(0x0362, F("SM100Monitor"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Monitor(t); });
            register_telegram_type(0x0363, F("SM100Monitor2"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Monitor2(t); });
            register_telegram_type(0x0366, F("SM100Config"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Config(t); });
            register_telegram_type(0x0364, F("SM100Status"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100Status(t); });
            register_telegram_type(0x036A, F("SM100Status2"), false, [&](std::shared_ptr<const Telegram> t) { process_SM100Status2(t); });
            register_telegram_type(0x0380, F("SM100CollectorConfig"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100CollectorConfig(t); });
            register_telegram_type(0x038E, F("SM100Energy"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Energy(t); });
            register_telegram_type(0x0391, F("SM100Time"), true, [&](std::shared_ptr<const Telegram> t) { process_SM100Time(t); });

            register_mqtt_cmd(F("SM100TankBottomMaxTemp"), [&](const char * value, const int8_t id) { return set_SM100TankBottomMaxTemp(value, id); });
        }
    }

    if (flags == EMSdevice::EMS_DEVICE_FLAG_ISM) {
        register_telegram_type(0x0103, F("ISM1StatusMessage"), true, [&](std::shared_ptr<const Telegram> t) { process_ISM1StatusMessage(t); });
        register_telegram_type(0x0101, F("ISM1Set"), false, [&](std::shared_ptr<const Telegram> t) { process_ISM1Set(t); });
    }

    // device values...

    // special case for a device_id with 0x2A where it's not actual a solar module
    if (device_id == 0x2A) {
        register_device_value(TAG_NONE, &type_, DeviceValueType::TEXT, nullptr, F("type"), F("type"));
        strncpy(type_, "warm water circuit", sizeof(type_));
    }

    register_device_value(TAG_NONE, &id_, DeviceValueType::UINT, nullptr, F("id"), nullptr); // empty full name to prevent being shown in web or console
    id_ = product_id;

    register_device_value(TAG_NONE, &collectorTemp_, DeviceValueType::SHORT, FL_(div10), F("collectorTemp"), F("collector temperature (TS1)"), DeviceValueUOM::DEGREES);
    register_device_value(TAG_NONE, &tankBottomTemp_, DeviceValueType::SHORT, FL_(div10), F("tankBottomTemp"), F("tank bottom temperature (TS2)"), DeviceValueUOM::DEGREES);
    register_device_value(TAG_NONE, &tankBottomTemp2_, DeviceValueType::SHORT, FL_(div10), F("tank2BottomTemp"), F("second tank bottom temperature (TS5)"), DeviceValueUOM::DEGREES);
    register_device_value(TAG_NONE, &heatExchangerTemp_, DeviceValueType::SHORT, FL_(div10), F("heatExchangerTemp"), F("heat exchanger temperature (TS6)"), DeviceValueUOM::DEGREES);

    register_device_value(TAG_NONE, &tankBottomMaxTemp_, DeviceValueType::UINT, nullptr, F("tank1MaxTempCurrent"), F("maximum tank temperature"), DeviceValueUOM::DEGREES);
    register_device_value(TAG_NONE, &solarPumpModulation_, DeviceValueType::UINT, nullptr, F("solarPumpModulation"), F("pump modulation (PS1)"), DeviceValueUOM::PERCENT);
    register_device_value(TAG_NONE, &cylinderPumpModulation_, DeviceValueType::UINT, nullptr, F("cylinderPumpModulation"), F("cylinder pump modulation (PS5)"), DeviceValueUOM::PERCENT);

    register_device_value(TAG_NONE, &solarPump_, DeviceValueType::BOOL, nullptr, F("solarPump"), F("pump (PS1)"), DeviceValueUOM::PUMP);
    register_device_value(TAG_NONE, &valveStatus_, DeviceValueType::BOOL, nullptr, F("valveStatus"), F("valve status"));
    register_device_value(TAG_NONE, &tankHeated_, DeviceValueType::BOOL, nullptr, F("tankHeated"), F("tank heated"));
    register_device_value(TAG_NONE, &collectorShutdown_, DeviceValueType::BOOL, nullptr, F("collectorShutdown"), F("collector shutdown"));

    register_device_value(TAG_NONE, &pumpWorkTime_, DeviceValueType::TIME, nullptr, F("pumpWorkTime"), F("pump working time"), DeviceValueUOM::MINUTES);

    register_device_value(TAG_NONE, &energyLastHour_, DeviceValueType::ULONG, FL_(div10), F("energyLastHour"), F("energy last hour"), DeviceValueUOM::WH);
    register_device_value(TAG_NONE, &energyTotal_, DeviceValueType::ULONG, FL_(div10), F("energyTotal"), F("energy total"), DeviceValueUOM::KWH);
    register_device_value(TAG_NONE, &energyToday_, DeviceValueType::ULONG, nullptr, F("energyToday"), F("energy today"), DeviceValueUOM::WH);
}

// publish HA config
bool Solar::publish_ha_config() {
    StaticJsonDocument<EMSESP_JSON_SIZE_HA_CONFIG> doc;
    doc["name"]    = FJSON("ID");
    doc["uniq_id"] = F_(solar);

    char stat_t[Mqtt::MQTT_TOPIC_MAX_SIZE];
    snprintf_P(stat_t, sizeof(stat_t), PSTR("%s/solar_data"), Mqtt::base().c_str());
    doc["stat_t"] = stat_t;

    doc["val_tpl"] = FJSON("{{value_json.id}}");
    JsonObject dev = doc.createNestedObject("dev");
    dev["name"]    = FJSON("EMS-ESP Solar");
    dev["sw"]      = EMSESP_APP_VERSION;
    dev["mf"]      = brand_to_string();
    dev["mdl"]     = name();
    JsonArray ids  = dev.createNestedArray("ids");
    ids.add("ems-esp-solar");

    char topic[Mqtt::MQTT_TOPIC_MAX_SIZE];
    snprintf_P(topic, sizeof(topic), PSTR("homeassistant/sensor/%s/solar/config"), Mqtt::base().c_str());
    Mqtt::publish_ha(topic, doc.as<JsonObject>()); // publish the config payload with retain flag

    return true;
}

// SM10Monitor - type 0x97
void Solar::process_SM10Monitor(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(collectorTemp_, 2));       // collector temp from SM10, is *10
    has_update(telegram->read_value(tankBottomTemp_, 5));      // tank bottom temp from SM10, is *10
    has_update(telegram->read_value(solarPumpModulation_, 4)); // modulation solar pump
    has_update(telegram->read_bitvalue(solarPump_, 7, 1));
    has_update(telegram->read_value(pumpWorkTime_, 8, 3));
}

/*
 * process_SM100SystemConfig - type 0x0358 EMS+ - for MS/SM100 and MS/SM200
 * e.g. B0 0B FF 00 02 58 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF 00 FF 01 00 00
 */
void Solar::process_SM100SystemConfig(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(heatTransferSystem_, 5, 1));
    has_update(telegram->read_value(externalTank_, 9, 1));
    has_update(telegram->read_value(thermalDisinfect_, 10, 1));
    has_update(telegram->read_value(heatMetering_, 14, 1));
    has_update(telegram->read_value(solarIsEnabled_, 19, 1));
}

/*
 * process_SM100SolarCircuitConfig - type 0x035A EMS+ - for MS/SM100 and MS/SM200
 * e.g. B0 0B FF 00 02 5A 64 05 00 58 14 01 01 32 64 00 00 00 5A 0C
 */
void Solar::process_SM100SolarCircuitConfig(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(collectorMaxTemp_, 0, 1));
    has_update(telegram->read_value(tankBottomMaxTemp_, 3, 1));
    has_update(telegram->read_value(collectorMinTemp_, 4, 1));
    has_update(telegram->read_value(solarPumpMode_, 5, 1));
    has_update(telegram->read_value(solarPumpMinRPM_, 6, 1));
    has_update(telegram->read_value(solarPumpTurnoffDiff_, 7, 1));
    has_update(telegram->read_value(solarPumpTurnonDiff_, 8, 1));
    has_update(telegram->read_value(solarPumpKick_, 9, 1));
    has_update(telegram->read_value(plainWaterMode_, 10, 1));
    has_update(telegram->read_value(doubleMatchFlow_, 11, 1));
}

/* process_SM100ParamCfg - type 0xF9 EMS 1.0
 * This telegram is used to inquire the min, default, max, and current values of a value that is usually read and written with another telegram ID
 * The CS200 uses this method extensively to find out which values may be set in the SM100
 * e.g. B0 10 F9 00 FF 02 5A 03 17 00 00 00 14 00 00 00 3C 00 00 00 5A 00 00 00 59 29 - requested with 90 B0 F9 00 11 FF 02 5A 03 AF
 * byte 0 = 0xFF
 * byte 1-2 = telegram ID used to write this value
 * byte 3 = offset in telegram used to write this value
 * byte 4 = unknown
 * bytes 5..8 = minimum value
 * bytes 9..12 = default value
 * bytes 13..16 = maximum value
 * bytes 17..20 = current value
 *
 * e.g. B0 0B F9 00 00 02 5A 00 00 6E
 */
void Solar::process_SM100ParamCfg(std::shared_ptr<const Telegram> telegram) {
    uint16_t t_id;
    uint8_t  of;
    int32_t  min, def, max, cur;
    has_update(telegram->read_value(t_id, 1));
    has_update(telegram->read_value(of, 3));
    has_update(telegram->read_value(min, 5));
    has_update(telegram->read_value(def, 9));
    has_update(telegram->read_value(max, 13));
    has_update(telegram->read_value(cur, 17));

    // LOG_DEBUG(F("SM100ParamCfg param=0x%04X, offset=%d, min=%d, default=%d, max=%d, current=%d"), t_id, of, min, def, max, cur));
}

/*
 * SM100Monitor - type 0x0362 EMS+ - for MS/SM100 and MS/SM200
 * e.g. B0 0B FF 00 02 62 00 77 01 D4 80 00 80 00 80 00 80 00 80 00 80 00 80 00 80 00 00 F9 80 00 80 9E - for heat exchanger temp
 * e.g, 30 00 FF 00 02 62 01 AC
 *      30 00 FF 18 02 62 80 00
 *      30 00 FF 00 02 62 01 A1 - for tank bottom temps
 * bytes 0+1 = TS1 Temperature sensor for collector
 * bytes 2+3 = TS2 Temperature sensor 1 cylinder, bottom
 * bytes 16+17 = TS5 Temperature sensor 2 cylinder, bottom, or swimming pool
 * bytes 20+21 = TS6 Temperature sensor external heat exchanger
 */
void Solar::process_SM100Monitor(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(collectorTemp_, 0));      // is *10 - TS1: Temperature sensor for collector array 1
    has_update(telegram->read_value(tankBottomTemp_, 2));     // is *10 - TS2: Temperature sensor 1 cylinder, bottom
    has_update(telegram->read_value(tankBottomTemp2_, 16));   // is *10 - TS5: Temperature sensor 2 cylinder, bottom, or swimming pool
    has_update(telegram->read_value(heatExchangerTemp_, 20)); // is *10 - TS6: Heat exchanger temperature sensor
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// SM100Monitor2 - 0x0363
// e.g. B0 00 FF 00 02 63 80 00 80 00 00 00 80 00 80 00 80 00 00 80 00 5A
void Solar::process_SM100Monitor2(std::shared_ptr<const Telegram> telegram) {
    // not implemented yet
}

// SM100wwTemperatur - 0x07D6
// Solar Module(0x2A) -> (0x00), (0x7D6), data: 01 C1 00 00 02 5B 01 AF 01 AD 80 00 01 90
void Solar::process_SM100wwTemperature(std::shared_ptr<const Telegram> telegram) {
    // has_update(telegram->read_value(wwTemp_1_, 0));
    // has_update(telegram->read_value(wwTemp_3_, 4));
    // has_update(telegram->read_value(wwTemp_4_, 6));
    // has_update(telegram->read_value(wwTemp_5_, 8));
    // has_update(telegram->read_value(wwTemp_7_, 12));
}

// SM100wwStatus - 0x07AA
// Solar Module(0x2A) -> (0x00), (0x7AA), data: 64 00 04 00 03 00 28 01 0F
void Solar::process_SM100wwStatus(std::shared_ptr<const Telegram> telegram) {
    // has_update(telegram->read_value(wwPump_, 0));
}

// SM100wwCommand - 0x07AB
// Thermostat(0x10) -> Solar Module(0x2A), (0x7AB), data: 01 00 01
void Solar::process_SM100wwCommand(std::shared_ptr<const Telegram> telegram) {
    // not implemented yet
}

#pragma GCC diagnostic pop

// SM100Config - 0x0366
// e.g. B0 00 FF 00 02 66     01 62 00 13 40 14
void Solar::process_SM100Config(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(availabilityFlag_, 0));
    has_update(telegram->read_value(configFlag_, 1));
    has_update(telegram->read_value(userFlag_, 2));
}

/*
 * SM100Status - type 0x0364 EMS+ for pump modulations - for MS/SM100 and MS/SM200
 - PS1: Solar circuit pump for collector array 1
 - PS5: Cylinder primary pump when using an external heat exchanger
 * e.g. 30 00 FF 09 02 64 64 = 100%
 */
void Solar::process_SM100Status(std::shared_ptr<const Telegram> telegram) {
    uint8_t solarpumpmod    = solarPumpModulation_;
    uint8_t cylinderpumpmod = cylinderPumpModulation_;
    has_update(telegram->read_value(cylinderPumpModulation_, 8));
    has_update(telegram->read_value(solarPumpModulation_, 9));

    if (solarpumpmod == 0 && solarPumpModulation_ == 100) { // mask out boosts
        solarPumpModulation_ = 15;                          // set to minimum
    }

    if (cylinderpumpmod == 0 && cylinderPumpModulation_ == 100) { // mask out boosts
        cylinderPumpModulation_ = 15;                             // set to minimum
    }
    has_update(telegram->read_bitvalue(tankHeated_, 3, 1));        // issue #422
    has_update(telegram->read_bitvalue(collectorShutdown_, 3, 0)); // collector shutdown
}

/*
 * SM100Status2 - type 0x036A EMS+ for pump on/off at offset 0x0A - for SM100 and SM200
 * e.g. B0 00 FF 00 02 6A 03 03 03 03 01 03 03 03 03 03 01 03
 * byte 4 = VS2 3-way valve for cylinder 2 : test=01, on=04 and off=03
 * byte 10 = PS1 Solar circuit pump for collector array 1: test=b0001(1), on=b0100(4) and off=b0011(3)
 */
void Solar::process_SM100Status2(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_bitvalue(valveStatus_, 4, 2)); // on if bit 2 set
    has_update(telegram->read_bitvalue(solarPump_, 10, 2));  // on if bit 2 set
}

/*
 * SM100CollectorConfig - type 0x0380 EMS+  - for SM100 and SM200
 * e.g. B0 0B FF 00 02 80 50 64 00 00 29 01 00 00 01
 */
void Solar::process_SM100CollectorConfig(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(climateZone_, 0, 1));
    has_update(telegram->read_value(collector1Area_, 3, 2));
    has_update(telegram->read_value(collector1Type_, 5, 1));
}

/*
 * SM100Energy - type 0x038E EMS+ for energy readings
 * e.g. 30 00 FF 00 02 8E 00 00 00 00 00 00 06 C5 00 00 76 35
 */
void Solar::process_SM100Energy(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(energyLastHour_, 0)); // last hour / 10 in Wh
    has_update(telegram->read_value(energyToday_, 4));    // todays in Wh
    has_update(telegram->read_value(energyTotal_, 8));    // total / 10 in kWh
}

/*
 * SM100Time - type 0x0391 EMS+ for pump working time
 */
void Solar::process_SM100Time(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(pumpWorkTime_, 1, 3));
}

/*
 * Junkers ISM1 Solar Module - type 0x0103 EMS+ for energy readings
 *  e.g. B0 00 FF 00 00 03 32 00 00 00 00 13 00 D6 00 00 00 FB D0 F0
 */
void Solar::process_ISM1StatusMessage(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(collectorTemp_, 4));  // Collector Temperature
    has_update(telegram->read_value(tankBottomTemp_, 6)); // Temperature Bottom of Solar Boiler tank
    uint16_t Wh = 0xFFFF;
    has_update(telegram->read_value(Wh, 2)); // Solar Energy produced in last hour only ushort, is not * 10

    if (Wh != 0xFFFF) {
        energyLastHour_ = Wh * 10; // set to *10
    }

    has_update(telegram->read_bitvalue(solarPump_, 8, 0));         // PS1 Solar pump on (1) or off (0)
    has_update(telegram->read_value(pumpWorkTime_, 10, 3));        // force to 3 bytes
    has_update(telegram->read_bitvalue(collectorShutdown_, 9, 0)); // collector shutdown on/off
    has_update(telegram->read_bitvalue(tankHeated_, 9, 2));        // tank full
}

/*
 * Junkers ISM1 Solar Module - type 0x0101 EMS+ for setting values
 */
void Solar::process_ISM1Set(std::shared_ptr<const Telegram> telegram) {
    has_update(telegram->read_value(setpoint_maxBottomTemp_, 6));
}

// set temperature for tank
bool Solar::set_SM100TankBottomMaxTemp(const char * value, const int8_t id) {
    int temperature;
    if (!Helpers::value2number(value, temperature)) {
        return false;
    }

    // write value
    // 90 30 FF 03 02 5A 59 B3
    // note: optionally add the validate to 0x035A which will pick up the adjusted tank1MaxTempCurrent_
    write_command(0x35A, 0x03, (uint8_t)temperature);

    return true;
}

} // namespace emsesp
