#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <ModbusRTUClient.h>
#include "ConfigManager.h"
#include "RemoteLogger.h"
#include "LocalLogger.h"
#include "Display.h"
#include "Modem.h"
#include "GPS.h"

#define SIM_RXD           32
#define SIM_TXD           33
#define SIM_BAUD          115200
#define RTU_BAUD          9600
#define EEPROM_SIZE       56
#define TASK_WDT_TIMEOUT  60

Display lcd(0x27, 20, 4);
HardwareSerial SerialAT(1);
Modem modem(SerialAT, SIM_RXD, SIM_TXD, SIM_BAUD);
TinyGsmClient apiClient(modem, 1);
TinyGsmClient mqttClient(modem, 0);
RemoteLogger remote(mqttClient,apiClient);
ConfigManager config(remote, modem);
GPS gps(modem);
RTC Rtc(Wire);
/*********************************** Variable declarations ****************************************/
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

uint8_t mac[6];
char macAdr[18];
int logCount = 0;
char dateTime[20];
char logString[300];
RtcDateTime run_time, compiled;
std::vector<File> openFiles;
std::vector<ProbeData> probeData;

// Task handles
static TaskHandle_t gpsTaskHandle = NULL;
static TaskHandle_t modbusTaskHandle = NULL;
static TaskHandle_t rtcTaskHandle = NULL;
static TaskHandle_t pushTaskHandle = NULL;
static TaskHandle_t logTaskHandle = NULL;
static TaskHandle_t apiTaskHandle = NULL;

// Task delay times
const TickType_t gpsDelay = pdMS_TO_TICKS(5000);
const TickType_t modbusDelay = pdMS_TO_TICKS(10000);
const TickType_t rtcDelay = pdMS_TO_TICKS(5000);
const TickType_t logDelay = pdMS_TO_TICKS(5000);
const TickType_t pushDelay = pdMS_TO_TICKS(5000);
const TickType_t apiDelay = pdMS_TO_TICKS(5000);
/*********************************** Function declarations ****************************************/
bool sendRows(File &data, String &timeStamp, int fileNo);
bool processCsv(fs::FS &fs, const char* path, int fileNo);
bool findTimestamp(File &data, String &timeStamp, size_t &filePtr);
// Task prototypes
void readGPS(void *pvParameters);
void readModbus(void *pvParameters);
void checkRTC(void *pvParameters);
void remotePush(void *pvParameters);
void localLog(void *pvParameters);
void apiLog(void* pvParameters);
/**************************************************************************************************/
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  esp_efuse_mac_get_default(mac);
  sprintf(macAdr, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("ESP32 MAC Address: %s\r\n", macAdr);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  /************************************** Initialize DS3231 ****************************************/
  Rtc.Begin();
  compiled = RtcDateTime(__DATE__, __TIME__);
  Serial.println();
  if (!Rtc.IsDateTimeValid()){
    if(Rtc.LastError() != 0){
      Serial.println("RTC communication error");
      Serial.println(Rtc.LastError());
    } else {
      Rtc.SetDateTime(compiled);
    }
  }
  if (!Rtc.GetIsRunning()){
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }
  // Compare RTC time with compile time
  run_time = Rtc.GetDateTime();
  if (run_time < compiled){
    Serial.println("RTC is older than compile time! Updating DateTime");
    Rtc.SetDateTime(compiled);
  } else if (run_time > compiled) {
    Serial.println("RTC is newer than compile time, this is expected");
  } else if (run_time == compiled) {
    Serial.println("RTC is the same as compile time, while not expected all is still fine");
  }
  // Disable unnecessary functions of the RTC DS3231
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);
  delay(1000);
  /*************************************** Set Up microSD *******************************************/
  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
  }
  // Check SD card type
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  // Check SD card size
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  /************************************ Get Config Credentials **************************************/
  if(!config.readGprs()){
    Serial.println(config.lastError);
  }
  if(!config.readMqtt()){
    Serial.println(config.lastError);
  }
  if(!config.readApi()){
    Serial.println(config.lastError);
  }
  if(!config.readTank()){
    Serial.println(config.lastError);
  }
  delay(1000);
  /************************************** Initialize 4G Modem ***************************************/
  Serial.println("Initializing modem...");
  if(!modem.init()){
    Serial.println("Restarting modem...");
    modem.restart();
  }
  delay(5000);
  if(modem.getSimStatus() != 1){
    Serial.println("SIM not ready, checking for PIN...");
    if(modem.getSimStatus() == 2){
      Serial.println("SIM PIN required.");
      // Send the PIN to the modem
      modem.simUnlock();
      delay(1000);
      if(modem.getSimStatus() != 1){
        Serial.println("Failed to unlock SIM.");
      }
      Serial.println("SIM unlocked successfully.");
    } else {
      Serial.println("SIM not detected or unsupported status");
    }
  }
  delay(1000);
  /**************************************** Initialize GPS ******************************************/
  if(!gps.init()){
    Serial.println("Failed to initialize GPS");
    errLog("Failed to initialize GPS", Rtc);
  }
  /************************************* Connect modem to GPRS **************************************/
  if(!modem.isGprsConnected()){
    if(!modem.gprsConnect()){
      lcd.clearRow(1);
      lcd.print("GPRS failed");
    } else {
      lcd.clearRow(1);
      lcd.print("GPRS connected");
    }
  } else {
    lcd.clearRow(1);
    lcd.print("GPRS connected");
  }
  delay(5000);
  /************************************* Initialize MQTT client *************************************/
  remote.setMqttServer();
  remote.setBufferSize(1024);
  remote.setCallback(callBack);
  if(!remote.mqttConnected()){
    if(!remote.mqttConnect(macAdr)){
      lcd.clearRow(2);
      lcd.print("MQTT failed");
    } else {
      lcd.clearRow(2);
      lcd.print("MQTT connected");
    }
  } else {
    lcd.clearRow(2);
    lcd.print("MQTT connected");
  }
  /***************************** Connect to API & check for unlogged data ***************************/
  Serial.println("Connecting to host");
  remote.apiConnect();
  Serial.println("Getting API token");
  remote.retrieveToken();

  //deleteFlash<String>(0);
  //deleteFlash<size_t>(20);
  processCsv(SD, "/error.csv", 0);

  //deleteFlash<String>(20 + sizeof(size_t));
  //deleteFlash<size_t>(20 + sizeof(size_t) + 20);
  processCsv(SD, "/probe1.csv", 1);
  /********************************** Initialize Modbus RTU client **********************************/
  if(!ModbusRTUClient.begin(RTU_BAUD))
    Serial.println("Failed to start Modbus RTU Client!");
  probeData.reserve(config.probeId.size());

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(readGPS, "Read GPS", 3072, NULL, 2, &gpsTaskHandle, app_cpu);
  xTaskCreatePinnedToCore(readModbus, "Read Modbus", 5012, NULL, 2, &modbusTaskHandle, app_cpu);
  xTaskCreatePinnedToCore(apiLog, "Push to API", 5012, NULL, 3, &apiTaskHandle, pro_cpu);
  xTaskCreatePinnedToCore(localLog, "Log to SD", 3072, NULL, 2, &logTaskHandle, app_cpu);
  xTaskCreatePinnedToCore(remotePush, "Push to MQTT", 5012, NULL, 2, &pushTaskHandle, app_cpu);
  xTaskCreatePinnedToCore(checkRTC, "Check RTC", 2048, NULL, 1, &rtcTaskHandle, app_cpu);
  // Initialize the Task Watchdog Timer
  esp_task_wdt_init(TASK_WDT_TIMEOUT, true);
  vTaskDelete(NULL);
}

void loop(){}
/***************************************** GPS Task *************************************************/
void readGPS(void *pvParameters){
  while(1){
    Serial.println("Reading GPS...");
    int status = gps.update();
    if(status != 1){
      if(status == 0){
        Serial.println("No GPS response");
        errLog("No GPS response", Rtc);
      } else if(status == 2) {
        Serial.println("Invalid GPS data");
        errLog("Invalid GPS data", Rtc);
      }
    } else {
      Serial.println("GPS updated");
    }
    xTaskNotifyGive(pushTaskHandle);
    xTaskNotifyGive(logTaskHandle);
    vTaskDelay(gpsDelay);
  }
}
/*************************************** Modbus Task ************************************************/
void readModbus(void *pvParameters){
  while(1){
    Serial.println("Reading Modbus...");
    char errBuffer[512];
    bool errors = false;
    for (size_t i = 0; i < config.probeId.size(); ++i){
      float* dataPointers[] = {&probeData[i].volume, &probeData[i].ullage, 
                              &probeData[i].temperature, &probeData[i].product, 
                              &probeData[i].water};
      const char* labels[] = {"Volume", "Ullage", "Temperature", "Product", "Water"};
      Serial.printf("Probe number %d:\n", config.probeId[i]);
      for (size_t j = 0; j < 5; ++j){
        *dataPointers[j] = ModbusRTUClient.holdingRegisterRead<float>(
                          config.probeId[i], config.modbusReg[j], BIGEND);
        if (*dataPointers[j] < 0){
          Serial.printf("Failed to read %s for probe ID: %d\r\nError: ", labels[j], config.probeId[i]);
          Serial.println(ModbusRTUClient.lastError());
          snprintf(errBuffer + strlen(errBuffer), 
                  512 - strlen(errBuffer), 
                  "%s error in probe %d: %s, ", 
                  labels[j], config.probeId[i], ModbusRTUClient.lastError());
          errors = true;
        } else {
          Serial.printf("%s: %.2f, ", labels[j], *dataPointers[j]);
        }
      }
      Serial.print("\n");
      if (errors) {
        errBuffer[strlen(errBuffer) - 2] = '\0';
        errLog(errBuffer, Rtc);
        memset(errBuffer, 0, sizeof(errBuffer));
      }
    }
    xTaskNotifyGive(pushTaskHandle);
    xTaskNotifyGive(logTaskHandle);
    vTaskDelay(modbusDelay);
  }
}
/****************************************************************************************************/
void remotePush(void *pvParameters){
  while(1){
    Serial.println("Pushing to MQTT...");
    if(!modem.isGprsConnected()){
      if (!modem.gprsConnect()){
        lcd.clearRow(1);
        lcd.print("GPRS failed");
        errLog("GPRS connection failed", Rtc);
      } else {
        lcd.clearRow(1);
        lcd.print("GPRS connected");
      }
    } else if(!remote.mqttConnected()){
      if(!remote.mqttConnect(macAdr)){
        lcd.clearRow(2);
        lcd.print("MQTT failed");
      } else {
        lcd.clearRow(2);
        lcd.print("MQTT connected");
      }
    } else {
      lcd.clearRow(2);
      lcd.print("MQTT connected");
      remote.mqttSubscribe();

      Serial.println("Sending data to MQTT...");
      JsonDocument data;
      data["Device"] = macAdr;
      data["Date/Time"] = dateTime;

      JsonObject gpsData = data.createNestedObject("Gps");
      gpsData["Latitude"] = gps.location.latitude;
      gpsData["Longitude"] = gps.location.longitude;
      gpsData["Altitude"] = gps.location.altitude;
      gpsData["Speed"] = gps.location.speed;

      JsonArray measures = data.createNestedArray("Measure");
      for(size_t i = 0; i < config.probeId.size(); i++){
        JsonObject measure = measures.createNestedObject();
        measure["Id"] = config.probeId[i];
        measure["Volume"] = probeData[i].volume;
        measure["Ullage"] = probeData[i].ullage;
        measure["Temperature"] = probeData[i].temperature;
        measure["ProductLevel"] = probeData[i].product;
        measure["WaterLevel"] = probeData[i].water;
      }
      // Serialize JSON and publish
      char buffer[1024];
      size_t n = serializeJson(data, buffer);
      remote.loop();
      remote.mqttPublish(buffer, n);
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(pushDelay);
  }
}
/********************* Read and push data from CSV file number 'fileNo' to API ***********************/
void apiLog(void* pvParameters){
  while(1){
    Serial.println("Pushing to API...");
    if(!modem.isGprsConnected()){
      if (!modem.gprsConnect()){
        lcd.clearRow(2);
        lcd.print("GPRS failed");
        errLog("GPRS connection failed", Rtc);
      } else {
        lcd.clearRow(2);
        lcd.print("GPRS connected");
      }
    } else if(!remote.apiConnected()){ 
      if(!remote.apiConnect()){
        lcd.clearRow(0);
        lcd.print("API failed");
      } else {
        lcd.clearRow(0);
        lcd.print("API connected");
      }
    } else {
      lcd.print("API connected");
      if(logCount == 5){
        esp_task_wdt_reset();
        if(processCsv(SD, "/error.csv", 0) && processCsv(SD, "/probe1.csv", 1))
          logCount = 0;
      }
    }
    esp_task_wdt_reset();
    vTaskDelay(apiDelay);
  }
}
/****************************************************************************************************/
void localLog(void *pvParameters){
  while(1){
    Serial.println("Logging microSD...");
    for(size_t i = 0; i < config.probeId.size(); i++){
      char path[15];
      snprintf(path, sizeof(path), "/probe%d.csv", i + 1);
      snprintf(logString, sizeof(logString), "\n%s;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f",
              dateTime, gps.location.latitude, gps.location.longitude, gps.location.speed,
              gps.location.altitude, probeData[i].volume, probeData[i].ullage,
              probeData[i].temperature, probeData[i].product, probeData[i].water);
      if(appendFile(SD, path, logString)){
        logCount++;
      }
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    vTaskDelay(logDelay);
  }
}
/****************************************************************************************************/
void checkRTC(void *pvParameters){
  while(1){
    if(!Rtc.IsDateTimeValid()){
      if(Rtc.LastError() != 0){
        Serial.println("RTC communication error");
        Serial.println(Rtc.LastError());
      } else {
        Rtc.SetDateTime(compiled);
      }
    }
    if(!Rtc.GetIsRunning()){
      Serial.println("RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
    }
    Rtc.saveTime(dateTime);
    lcd.clearRow(3);
    lcd.print(dateTime);
    xTaskNotifyGive(pushTaskHandle);
    xTaskNotifyGive(logTaskHandle);
    vTaskDelay(rtcDelay);
  }
}
/*************************************** Support functions *******************************************/
// Function to skip rows until the last pushed timestamp is found
bool findTimestamp(File &data, String &timeStamp, size_t &filePtr){
  if(data.seek(filePtr, SeekSet)){
    while(data.available()){
      String line = data.readStringUntil('\n');
      line.trim();
      if(timeStamp == "" || line.startsWith(timeStamp)){
        return true;
      }
    }
  }
  return false;
}
// Read and process rows from the CSV file
bool sendRows(File &data, String &timeStamp, int fileNo){
  Serial.println("Sending rows");
  const int chunkSize = 5;
  const int maxRetries = 5;
  int retries = 0;
  int rowCount = 0;
  bool success = true;
  String jsonPayload;
  String rows[chunkSize];
  // Lambda to process and send rows
  auto processAndSend = [&](bool isFinalChunk){
    jsonPayload = (fileNo == 0) ? errorToJson(rows, rowCount) : dataToJson(rows, rowCount);
    Serial.println(jsonPayload);
    retries = 0;
    while(retries < maxRetries){
      bool sent = (fileNo == 0) ? remote.send(jsonPayload, 0) : remote.send(jsonPayload, 1);
      if(sent){
        timeStamp = readCsv(rows[rowCount - 1]);
        rowCount = 0;
        return true;
      } else {
        Serial.println(isFinalChunk ? "Failed to send final chunk. Retrying..." 
                                    : "Failed to send data chunk. Retrying...");
        retries++;
      }
    }
    Serial.println("Max retries reached. Aborting...");
    return false;
  };
  // Lambda to check for corrupted character
  auto corrupted = [&] (const String& line){
    for(unsigned int i = 0; i < line.length(); i++){
      if(line[i] < 32 || line[i] > 126)
        return true;
    }
    return false;
  };
  // Main loop to read and process rows
  while(data.available()){
    String line = data.readStringUntil('\n');
    line.trim();
    if(line == "" || line.length() == 0 || corrupted(line))
      continue;
    rows[rowCount++] = line;
    // Process a chunk when full
    if(rowCount == chunkSize){
      if(!processAndSend(false)){
        success = false;
        rowCount = 0;
        break;
      }
    }
  }
  // Process remaining rows if any
  if(rowCount > 0){
    if(!processAndSend(true)){
      success = false;
    }
  }
  return success;
}

bool processCsv(fs::FS &fs, const char* path, int fileNo){
  File data = openFile(fs, path);
  if (!data){
    return false;
  }
  String timeStamp = readFlash<String>(fileNo * 20 + fileNo * sizeof(size_t));
  size_t filePtr = readFlash<size_t>(fileNo * 20 + fileNo * sizeof(size_t) + 20);
  if(!findTimestamp(data, timeStamp, filePtr)){
    data.close();
    return false;
  }
  if(sendRows(data, timeStamp, fileNo)){
    saveToFlash(fileNo * 20 + fileNo * sizeof(size_t), timeStamp);
    saveToFlash(fileNo * 20 + fileNo * sizeof(size_t) + 20, data.position());
  }
  data.close();
  return true;
}