#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <MHZ19_uart.h>
#include <LiquidCrystal_I2C.h>
#include <FirebaseESP32.h>
#include <FirebaseESP32Stream.h>
#include <FirebaseJson.h>
#include <ArduinoJson.h>
#include <TaskScheduler.h>
#include <Button.h>
#include <Encoder.h>




// Define pin numbers
#define Relay1 10
#define Relay2 11
#define Relay3 12
#define Relay4 13
#define Relay5 14
#define Relay6 15
#define Relay7 16
#define Relay8 17
#define ROTARY_ENCODER_CLK 18
#define ROTARY_ENCODER_DT 19
#define buttonMenuPin 20
#define buttonOkPin 21

// Other constants
#define STORAGE_BUCKET_ID "your_storage_bucket_id"
#define FIRMWARE_PATH "your_firmware_path"
#define API_KEY "your_api_key"
#define USER_EMAIL "your_email"
#define USER_PASSWORD "your_password"
#define DATABASE_URL "your_database_url"

// Initialize objects
LiquidCrystal_I2C lcd(0x27, 16, 2); // Change the LCD address and dimensions as needed
Adafruit_BME280 bme;
MHZ19_uart myMHZ19;
Button button(buttonMenuPin);
Encoder encoder(ROTARY_ENCODER_CLK, ROTARY_ENCODER_DT);

// Firebase configuration
FirebaseData fbdo;
FirebaseConfig configF;
FirebaseAuth auth;
FirebaseStream stream;
FirebaseJson json;

// Other variables
unsigned long previousMillis = 0;
const long interval = 1000;
char timestamp[10];
struct tm timeinfo;
bool DeviceStatus = false;
bool sensorsData = true;
char parentPath[100] = "/";
TaskScheduler scheduler;
int charsetPosition = 0;

/