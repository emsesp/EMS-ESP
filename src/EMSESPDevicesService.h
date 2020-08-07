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

#ifndef EMSESPDevicesService_h
#define EMSESPDevicesService_h

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <SecurityManager.h>

// #define MAX_EMSESP_STATUS_SIZE 1024
#define MAX_EMSESP_DEVICE_SIZE 1536

#define EMSESP_DEVICES_SERVICE_PATH "/rest/allDevices"
#define SCAN_DEVICES_SERVICE_PATH "/rest/scanDevices"
#define DEVICE_DATA_SERVICE_PATH "/rest/deviceData"

namespace emsesp {

using namespace std::placeholders; // for `_1`

class EMSESPDevicesService {
  public:
    EMSESPDevicesService(AsyncWebServer * server, SecurityManager * securityManager);

  private:
    void all_devices(AsyncWebServerRequest * request);
    void scan_devices(AsyncWebServerRequest * request);
    void device_data(AsyncWebServerRequest * request, JsonVariant & json);

    AsyncCallbackJsonWebHandler _device_dataHandler;
};

} // namespace emsesp

#endif
