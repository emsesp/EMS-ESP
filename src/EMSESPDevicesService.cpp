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

#include "EMSESPDevicesService.h"
#include "emsesp.h"
#include "mqtt.h"

namespace emsesp {

EMSESPDevicesService::EMSESPDevicesService(AsyncWebServer * server, SecurityManager * securityManager)
    : _device_dataHandler(DEVICE_DATA_SERVICE_PATH,
                          securityManager->wrapCallback(std::bind(&EMSESPDevicesService::device_data, this, _1, _2), AuthenticationPredicates::IS_AUTHENTICATED)) {
    server->on(EMSESP_DEVICES_SERVICE_PATH,
               HTTP_GET,
               securityManager->wrapRequest(std::bind(&EMSESPDevicesService::all_devices, this, _1), AuthenticationPredicates::IS_AUTHENTICATED));

    server->on(SCAN_DEVICES_SERVICE_PATH,
               HTTP_GET,
               securityManager->wrapRequest(std::bind(&EMSESPDevicesService::scan_devices, this, _1), AuthenticationPredicates::IS_AUTHENTICATED));

    _device_dataHandler.setMethod(HTTP_POST);
    _device_dataHandler.setMaxContentLength(256);
    server->addHandler(&_device_dataHandler);
}

void EMSESPDevicesService::scan_devices(AsyncWebServerRequest * request) {
    EMSESP::clear_all_devices();
    EMSESP::send_read_request(EMSdevice::EMS_TYPE_UBADevices, EMSdevice::EMS_DEVICE_ID_BOILER);
    request->send(200);
}

void EMSESPDevicesService::all_devices(AsyncWebServerRequest * request) {
    AsyncJsonResponse * response = new AsyncJsonResponse(false, MAX_EMSESP_DEVICE_SIZE);
    JsonObject          root     = response->getRoot();

    JsonArray devices = root.createNestedArray("devices");
    for (const auto & emsdevice : EMSESP::emsdevices) {
        if (emsdevice) {
            JsonObject obj   = devices.createNestedObject();
            obj["id"]        = emsdevice->unique_id();
            obj["type"]      = emsdevice->device_type_name();
            obj["brand"]     = emsdevice->brand_to_string();
            obj["name"]      = emsdevice->name();
            obj["deviceid"]  = emsdevice->device_id();
            obj["productid"] = emsdevice->product_id();
            obj["version"]   = emsdevice->version();
        }
    }

    JsonArray sensors = root.createNestedArray("sensors");
    if (!EMSESP::sensor_devices().empty()) {
        for (const auto & sensor : EMSESP::sensor_devices()) {
            JsonObject obj = sensors.createNestedObject();
            obj["id"]      = sensor.to_string();
            obj["temp"]    = sensor.temperature_c;
        }
    }

    response->setLength();
    request->send(response);
}

void EMSESPDevicesService::device_data(AsyncWebServerRequest * request, JsonVariant & json) {
    if (json.is<JsonObject>()) {
        AsyncJsonResponse * response = new AsyncJsonResponse(false, MAX_EMSESP_DEVICE_SIZE);
#ifndef EMSESP_STANDALONE
        uint8_t id = json["id"]; // get id from selected table row
        EMSESP::device_info(id, (JsonObject &)response->getRoot());
#endif
        response->setLength();
        request->send(response);
    } else {
        AsyncWebServerResponse * response = request->beginResponse(200);
        request->send(response);
    }
}

} // namespace emsesp