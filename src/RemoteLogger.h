#ifndef REMOTELOGGER_H
#define REMOTELOGGER_H

#include <PubSubClient.h>

#define API_TIMEOUT       5000
#define TINY_GSM_MODEM_SIM7600

#include <TinyGsmClient.h>
#include "structs.h"

class RemoteLogger : public PubSubClient{
public:
 RemoteLogger(TinyGsmClient& mqttClient, TinyGsmClient& apiClient)
              : PubSubClient(mqttClient), apiClient(&apiClient){}

  RemoteLogger& setCreds(MQTT mqtt);
  RemoteLogger& setCreds(API api);
  PubSubClient& setMqttServer();
  boolean mqttConnect(const char* id);
  boolean mqttConnected();
  boolean mqttPublish(const char* payload, boolean retained);
  boolean mqttSubscribe();
  
  void retrieveToken();
  bool apiConnect();
  bool apiConnected();
  bool post(const char* endpoint, const char* msg);
  bool authPost(const char* endpoint, const char* msg);
  bool send(String& jsonPayload, int ndPntCode);
private:
  TinyGsmClient* mqttClient;
  TinyGsmClient* apiClient;
  String token;
  MQTT mqtt;
  API api;
};

void callBack(char* topic, byte* message, unsigned int len);

#endif