#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/semphr.h"

// ==================== OLED 설정 ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== MAX30105 센서 ====================
MAX30105 particleSensor;

// ==================== BLE UUID ====================
#define SERVICE_UUID        "180D"  // Heart Rate Service
#define CHARACTERISTIC_UUID "2A37"  // Heart Rate Measurement
#define LED_CTRL_UUID       "2A56"  // Integer Control (from Hub)

// ==================== 전역 변수 ====================
float bpm = 0;
int beatAvg = 0;
bool clientConnected = false;
int receivedValue = 0; // 허브로부터 받은 정수

SemaphoreHandle_t beatAvgMutex;
TaskHandle_t TaskReadSensor;
TaskHandle_t TaskUpdateOLED;
TaskHandle_t TaskSendBLE;

BLECharacteristic* pCharacteristic;
BLECharacteristic* pLedControlChar;

// ==================== BLE 서버 콜백 ====================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    clientConnected = true;
    Serial.println("✅ Hub connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    clientConnected = false;
    pServer->getAdvertising()->start();
    Serial.println("❌ Hub disconnected");
  }
};

// ==================== 정수 제어 콜백 ====================
class ValueControlCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      receivedValue = (uint8_t)value[0];
      Serial.printf("📩 정수 수신: %d\n", receivedValue);
    }
  }
};

// ==================== BLE 초기화 ====================
void setupBLE() {
  BLEDevice::init("ESP32_Wearable");

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // BPM Notify Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());

  // 정수 수신 Characteristic (Write)
  pLedControlChar = pService->createCharacteristic(
    LED_CTRL_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pLedControlChar->setCallbacks(new ValueControlCallback());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("🚀 BLE Service started");
}

// ==================== FreeRTOS Tasks ====================
void ReadSensorTask(void *pvParameters);
void UpdateOLEDTask(void *pvParameters);
void SendBLETask(void *pvParameters);

// ==================== setup ====================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Wearable Init");
  display.display();

  // MAX30105 초기화
  if(!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
    while(1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0xFF);

  beatAvgMutex = xSemaphoreCreateMutex();
  setupBLE();

  // FreeRTOS Task 생성
  xTaskCreatePinnedToCore(ReadSensorTask, "ReadSensor", 4096, NULL, 1, &TaskReadSensor, 0);
  xTaskCreatePinnedToCore(UpdateOLEDTask, "UpdateOLED", 2048, NULL, 1, &TaskUpdateOLED, 0);
  xTaskCreatePinnedToCore(SendBLETask, "SendBLE", 2048, NULL, 1, &TaskSendBLE, 1);
}

void loop() { 
  vTaskDelay(10 / portTICK_PERIOD_MS); 
}

// ==================== Sensor Read ====================
void ReadSensorTask(void * pvParameters) {
  static long lastBeat = 0;
  const byte RATE_SIZE = 8;
  byte rates[RATE_SIZE] = {0};  // ✅ 초기화
  byte rateSpot = 0;
  bool filled = false;          // ✅ 평균 안정화용

  for(;;) {
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      float rawBpm = 60.0 / (delta / 1000.0);

      if (rawBpm > 40 && rawBpm < 180) {
        bpm = rawBpm;
        if (xSemaphoreTake(beatAvgMutex, portMAX_DELAY) == pdTRUE) {
          rates[rateSpot++] = (byte)bpm;
          if (rateSpot >= RATE_SIZE) {
            rateSpot = 0;
            filled = true;
          }

          beatAvg = 0;
          int count = filled ? RATE_SIZE : rateSpot; // ✅ 아직 다 안 찼으면 일부만 평균
          for (int i = 0; i < count; i++) beatAvg += rates[i];
          beatAvg /= count;
          xSemaphoreGive(beatAvgMutex);
        }
      }
    }
  }
}

// ==================== OLED Update ====================
void UpdateOLEDTask(void * pvParameters) {
  int localAvg = 0;
  int localVal = 0;

  for(;;) {
    if (xSemaphoreTake(beatAvgMutex, 0) == pdTRUE) {
      localAvg = beatAvg;
      xSemaphoreGive(beatAvgMutex);
    }
    localVal = receivedValue;

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    display.printf("BPM:%3.0f\nAVG:%3d", bpm, localAvg);

    display.setTextSize(1);
    display.setCursor(0,40);
    display.printf("POSE: %d%%", localVal);
    display.setCursor(0,50);
    display.print(clientConnected ? "Hub Connected" : "Waiting...");

    display.display();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ==================== BLE Notify ====================
void SendBLETask(void * pvParameters) {
  uint8_t bpmValue = 0;
  for(;;) {
    if (clientConnected && pCharacteristic) {
      if (xSemaphoreTake(beatAvgMutex, 0) == pdTRUE) {
        bpmValue = (uint8_t)beatAvg;
        xSemaphoreGive(beatAvgMutex);
      }
      pCharacteristic->setValue(&bpmValue, 1);
      pCharacteristic->notify();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
