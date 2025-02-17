#include "RemoteLogger.h"
#include <ArduinoJson.h>

RemoteLogger& RemoteLogger::setCreds(MQTT mqtt){
  this->mqtt = mqtt;
  return *this;
}

RemoteLogger& RemoteLogger::setCreds(API api){
  this->api = api;
  return *this;
}

PubSubClient& RemoteLogger::setMqttServer(){
  return PubSubClient::setServer(this->mqtt.broker.c_str(), this->mqtt.port);
}

boolean RemoteLogger::mqttConnect(const char* id){
  return PubSubClient::connect(id, this->mqtt.user.c_str(), this->mqtt.pass.c_str());
}

boolean RemoteLogger::mqttConnected(){
  return PubSubClient::connected();
}

boolean RemoteLogger::mqttSubscribe(){
  return PubSubClient::subscribe(this->mqtt.topic.c_str());
}

boolean RemoteLogger::mqttPublish(const char* payload, boolean retained){
  return PubSubClient::publish(this->mqtt.topic.c_str(), payload, retained);
}

bool RemoteLogger::apiConnect(){
  if(apiClient->connect(this->api.host.c_str(), this->api.port, 10) != 1){
    return false;
  } else {
    return true;
  }
}

bool RemoteLogger::apiConnected(){
  return apiClient->connected();
}

// POST request without authentication
bool RemoteLogger::post(const char* endpoint, const char* msg){
  if(!apiClient){
    Serial.println("Client not initialized!");
    return false;
  }
  apiClient->print( "POST " + String(endpoint) + " HTTP/1.1\r\n"
                    "Host: "+ this->api.host + ":" + String(this->api.port) + "\r\n"
                    "Content-Type: application/json\r\n"
                    "tenant: root\r\n"
                    "Accept-Language: en-US\r\n"
                    "Connection: keep-alive\r\n"
                    "Content-Length: " + String(strlen(msg)) + "\r\n\r\n");
  apiClient->print(msg);
  return true;
}
// POST request with authentication
bool RemoteLogger::authPost(const char* endpoint, const char* msg){
  if(!apiClient){
    Serial.println("Client not initialized!");
    return false;
  }
  apiClient->print( "POST " + String(endpoint) + " HTTP/1.1\r\n"
                    "Host: "+ this->api.host + ":" + String(this->api.port) + "\r\n"
                    "Content-Type: application/json\r\n"
                    "Authorization: Bearer " + this->token + "\r\n"
                    "Accept-Language: en-US\r\n"
                    "Connection: keep-alive\r\n"
                    "Content-Length: " + String(strlen(msg)) + "\r\n\r\n");
  apiClient->print(msg);
  return true;
}

void RemoteLogger::retrieveToken(){
  if(!apiClient->connected()){
    Serial.println("Failed to connect to server.");
    return;
  }
  char tokenAuth[128];
  snprintf(tokenAuth, sizeof(tokenAuth),
          "{\"username\":\"%s\",\"password\":\"%s\"}",
          this->api.user.c_str(), this->api.pass.c_str());
  this->post("/api/tokens", tokenAuth);

  Serial.println("Waiting for authentication response...");
  unsigned long startTime = millis();
  String response;
  while((millis() - startTime) < API_TIMEOUT){
    if(apiClient->available()){
      response = apiClient->readString();
      Serial.println(response);
      break;
    }
  }
  if(response.isEmpty()){
    Serial.println("Error: No response from server.");
    return;
  }
  int jsonStart = response.indexOf("{");
  if(jsonStart != -1){
    response = response.substring(jsonStart);
    int jsonEnd = response.lastIndexOf("}");
    if(jsonEnd != -1)
      response = response.substring(0, jsonEnd + 1);
  } else {
    response = "";
  }
  // Parse the token from the response (assumes JSON response format)
  JsonDocument jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, response);
  if (error){
    Serial.print("Failed to get token: ");
    Serial.println(error.c_str());
    return;
  }
  if(jsonDoc.containsKey("token")){
    this->token = jsonDoc["token"].as<String>();
    Serial.print("Extracted token: ");
    Serial.println(this->token);
  } else {
    Serial.println("Error: 'token' field not found.");
  }
}

bool RemoteLogger::send(String& jsonPayload, int ndPntCode){
  if(this->token == NULL){
    Serial.println("Error: Token is NULL");
    return false;
  }
  Serial.println("Sending error chunk...");
  if(ndPntCode == 0){
    authPost("/api/v1/errorloggers/addlisterrorogger", jsonPayload.c_str());
  } else if(ndPntCode == 1){
    authPost("/api/v1/dataloggers/addlistdatalogger", jsonPayload.c_str());
  }
  Serial.println("Waiting for server response...");
  unsigned long startTime = millis();
  String response;
  while((millis() - startTime) < API_TIMEOUT){
    if(apiClient->available()){
      response = apiClient->readString();
      Serial.println(response);
      break;
    }
  }
  if(response.isEmpty()){
    Serial.println("Error: No response from server");
    return false;
  }
  if(response.startsWith("HTTP/1.1 200")){
    Serial.println("Data successfully sent to API");
    return true;
  } else {
    Serial.println("Error: API response indicates failure");
  }
  return false;
}

void callBack(char* topic, byte* msg, unsigned int len){
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String tempMsg;
  
  for (int i = 0; i < len; i++) {
    Serial.print((char)msg[i]);
    tempMsg += (char)msg[i];
  }
  Serial.println();
}