#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h" // SparkFun heart rate algorithm
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 심박 센서
MAX30105 particleSensor;
const byte RATE_SIZE = 4; 
byte rates[RATE_SIZE]; 
byte rateSpot = 0;
float bpm = 0;
int beatAvg = 0;

// BLE
BLECharacteristic* pCharacteristic;

// FreeRTOS Task
TaskHandle_t TaskReadSensor;
TaskHandle_t TaskUpdateOLED;
TaskHandle_t TaskSendBLE;

// 서버 콜백
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    Serial.println("BLE client disconnected");
  }
};

// 함수 선언
void ReadSensorTask(void * pvParameters);
void UpdateOLEDTask(void * pvParameters);
void SendBLETask(void * pvParameters);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // MAX30105 초기화
  if(!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
    while(1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0xFF);

  // OLED 초기화
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("OLED init failed");
    while(1);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Place your finger");
  display.display();

  // BLE 초기화
  BLEDevice::init("ESP32_HeartRate");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService("180D"); // Heart Rate Service
  pCharacteristic = pService->createCharacteristic(
                      "2A37",
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE Heart Rate Service started");

  // FreeRTOS Task 생성
  xTaskCreatePinnedToCore(ReadSensorTask, "ReadSensor", 4096, NULL, 1, &TaskReadSensor, 0);
  xTaskCreatePinnedToCore(UpdateOLEDTask, "UpdateOLED", 2048, NULL, 1, &TaskUpdateOLED, 0);
  xTaskCreatePinnedToCore(SendBLETask, "SendBLE", 2048, NULL, 1, &TaskSendBLE, 1);
}

void loop() {
  delay(1000); // loop는 비워둬도 됨
}

// 센서 읽기 Task
void ReadSensorTask(void * pvParameters){
  static long lastBeat = 0;
  for(;;){
    long irValue = particleSensor.getIR();
    Serial.print("IR="); Serial.println(irValue);

    if(checkForBeat(irValue)){
      long delta = millis() - lastBeat;
      lastBeat = millis();
      bpm = 60.0 / (delta / 1000.0);
      Serial.print("BPM="); Serial.println(bpm);

      // 평균 BPM 계산
      rates[rateSpot++] = (byte)bpm;
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for(byte i = 0; i < RATE_SIZE; i++){
        beatAvg += rates[i];
      }
      beatAvg /= RATE_SIZE;
    }
  }
}

// OLED 업데이트 Task
void UpdateOLEDTask(void * pvParameters){
  for(;;){
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("BPM: "); display.println(bpm,1);
    display.print("Avg BPM: "); display.println(beatAvg);
    display.display();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// BLE notify Task
void SendBLETask(void * pvParameters){
  for(;;){
    uint8_t bpmValue = (uint8_t)beatAvg; // 평균 BPM 전송
    if(pCharacteristic){
      pCharacteristic->setValue(&bpmValue, 1);
      pCharacteristic->notify();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
