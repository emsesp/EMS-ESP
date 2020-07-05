#ifndef EMSESPStatusService_h
#define EMSESPStatusService_h

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <SecurityManager.h>


#include <HttpEndpoint.h>
#include <MqttPubSub.h>
#include <WebSocketTxRx.h>

#include "EMSESPSettingsService.h"
#include "version.h"

#define MAX_EMSESP_STATUS_SIZE 1024
#define EMSESP_STATUS_SERVICE_PATH "/rest/emsespStatus"

namespace emsesp {

class EMSESPStatusService {
  public:
    EMSESPStatusService(AsyncWebServer * server, SecurityManager * securityManager, AsyncMqttClient * mqttClient);

  private:
    EMSESPSettingsService * _emsespSettingsService;
    void                    emsespStatusService(AsyncWebServerRequest * request);
    AsyncMqttClient *       _mqttClient;

    void init_mqtt();
};

} // namespace emsesp

#endif