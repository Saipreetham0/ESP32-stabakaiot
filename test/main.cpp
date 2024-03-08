#include <Arduino.h>
#include "pinDefs.h"
#include "ArduinoJson.h"
//#include <FreeRTOS.h>
#include "enroll.h"
#include "fingerPrint.h"
#include "rfid.h"
#include "database.h"
#include "accessMode.h"
#include "esp_wifi.h"
#include <SPIFFS.h>         // Built-in
#include "esp_system.h"     // Built-in
#include "esp_spi_flash.h"  // Built-in
#include "esp_wifi_types.h" // Built-in
#include "webpage.h"


uint32_t cloudConnectTime = millis();

bool isEnrollTaskCreated = false;

bool isOtaFlag = false;
String eTag = "";

void setup()
{
  Serial.begin(115200);
  disableCore0WDT();
  timerInit(wdt_timeout);
  loadSettings();

  accesssModeSetup();
  Serial.println();
  Serial.println(" 15 ###  ");
  mainDebugln("In setup");

  homeInit();

  delay(50);

  String en = cloudParameters["isEnroll"].as<String>();
  mainDebugln(en);
  if (en == "1")
  {
    cloudParameters["isEnroll"] = 0;
    updateSettings();
    digitalWrite(BUZZER, HIGH);
    mainDebugln("enrol online");
    accessForTemplateTransfer();
    digitalWrite(BUZZER, LOW);
    extractEnrollData();
    delay(100);
    esp_restart();
  }
}

void loop()
{
  timerWrite(timer, 0);
  delay(100);
  blinkAttiny();  delay(100);
  if (isEnroll)
  {
    mainDebugln("Enrollment Started deleting Acesss task starting Enroll Task");
    isEnrollTaskCreated = true;
    isEnroll = false;

    timerWrite(timer, 0);
    delay(200);
    mainDebugln("going to enroll mode");
    mainDebugln("Free RAM is ::");
    mainDebugln(heap_caps_get_free_size(MALLOC_CAP_8BIT));
    xTaskCreatePinnedToCore(enrollModeTask, "ENROLL_TASK", 8000, NULL, 5, &enrollModeTaskHandle, 1);
    delay(100);
    Serial.println("Deleting Task");
    delay(500);
    vTaskDelete(NULL);
  }
  else
  {
    // Serial.println("TX...");
    accessModeLoop();
    timerWrite(timer, 0);
    delay(100);
  }
}
