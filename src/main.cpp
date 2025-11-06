#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/semphr.h" // FreeRTOS Semaphore/Mutex 헤더 추가

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// ==================== 하드웨어 및 센서 설정 ====================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105 particleSensor;

const byte RATE_SIZE = 8; 
byte rates[RATE_SIZE]; 
byte rateSpot = 0;

// ==================== 공유 및 전역 변수 ====================
float bpm = 0;              // BPM 임시값 (float)
int beatAvg = 0;            // 평균 BPM (공유 변수, 뮤텍스 보호 대상)
volatile bool clientConnected = false;  // 연결 상태 플래그 (volatile 유지)

// FreeRTOS 뮤텍스 핸들: beatAvg 변수 접근 보호
SemaphoreHandle_t beatAvgMutex = NULL; 

// FreeRTOS Task Handles
TaskHandle_t TaskReadSensor;
TaskHandle_t TaskUpdateOLED;
TaskHandle_t TaskSendBLE;

// BLE
BLECharacteristic* pCharacteristic;
#define SERVICE_UUID        "180D" // Heart Rate Service
#define CHARACTERISTIC_UUID "2A37" // Heart Rate Measurement Characteristic

// ==================== BLE 서버 콜백 ====================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("BLE client connected");
    clientConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    Serial.println("BLE client disconnected");
    clientConnected = false;
    // 광고 재시작
    pServer->getAdvertising()->start();
  }
};

// ==================== Task 함수 선언 ====================
void ReadSensorTask(void * pvParameters);
void UpdateOLEDTask(void * pvParameters);
void SendBLETask(void * pvParameters);

// ==================== SETUP 함수 ====================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // I2C 핀 설정 (ESP32 D21=SDA, D22=SCL이 일반적)

  // 1. MAX30105 초기화
  if(!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found!");
    while(1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0xFF);

  // 2. OLED 초기화
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("OLED init failed");
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Place your finger");
  display.display();

  // 3. 뮤텍스 생성 (가장 먼저)
  beatAvgMutex = xSemaphoreCreateMutex();
  if (beatAvgMutex == NULL) {
    Serial.println("FATAL: Failed to create Mutex!");
    while(1);
  }

  // 4. BLE 초기화 및 서버 설정
  BLEDevice::init("ESP32_HeartRate");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE Heart Rate Service started");

  // 5. FreeRTOS Task 생성 및 코어 할당
  // Core 0: 센서/OLED (I/O 및 계산)
  xTaskCreatePinnedToCore(ReadSensorTask, "ReadSensor", 4096, NULL, 1, &TaskReadSensor, 0);
  xTaskCreatePinnedToCore(UpdateOLEDTask, "UpdateOLED", 2048, NULL, 1, &TaskUpdateOLED, 0);
  
  // Core 1: BLE (BLE 스택과 연관된 작업을 Core 1에 할당)
  xTaskCreatePinnedToCore(SendBLETask, "SendBLE", 2048, NULL, 1, &TaskSendBLE, 1);
}

// loop는 FreeRTOS Task Scheduler에게 제어권을 완전히 넘깁니다.
void loop() {
  vTaskDelay(10 / portTICK_PERIOD_MS); 
}

// ==================== Task 구현 ====================

// 센서 읽기 Task (Core 0)
void ReadSensorTask(void * pvParameters){
  static long lastBeat = 0;
  float rawBpm = 0;  // 센서로부터 계산된 임시 BPM

  for(;;){
    long irValue = particleSensor.getIR();

    // 손 안 댄 상태 등 무의미한 IR 값은 무시
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      rawBpm = 60.0 / (delta / 1000.0);

      // BPM 유효성 검사 (40~180 사이)
      if (rawBpm > 40 && rawBpm < 180) {
        // 정상값만 bpm에 반영
        bpm = rawBpm;
        Serial.print("BPM(valid)="); Serial.println(bpm);

        // 평균 계산
        if (xSemaphoreTake(beatAvgMutex, portMAX_DELAY) == pdTRUE) {
          rates[rateSpot++] = (byte)bpm;
          rateSpot %= RATE_SIZE;

          beatAvg = 0;
          for (byte i = 0; i < RATE_SIZE; i++) {
            beatAvg += rates[i];
          }
          beatAvg /= RATE_SIZE;

          xSemaphoreGive(beatAvgMutex);
        }

      } else {
        // 이상치는 bpm 갱신 안 함
        Serial.print("BPM(outlier ignored)="); Serial.println(rawBpm);
      }
    }
  }
}


// OLED 업데이트 Task (Core 0)
void UpdateOLEDTask(void * pvParameters){
  int localBeatAvg = 0;

  // ESP32 MAC 주소 얻기 (BLE용)
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT); // 블루투스 MAC
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  for(;;){
    if (xSemaphoreTake(beatAvgMutex, 0) == pdTRUE) {
      localBeatAvg = beatAvg;
      xSemaphoreGive(beatAvgMutex);
    }

    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.print("BPM: "); display.println(bpm,1);
    display.print("AVG: "); display.println(localBeatAvg);

    display.setTextSize(1);
    display.setCursor(0,50);
    display.print("Status: ");
    if(clientConnected){
      display.println("BLE Connected");
    } else {
      display.println(macStr);
    }

    display.display();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// BLE notify Task (Core 1)
void SendBLETask(void * pvParameters){
  uint8_t bpmValue = 0;
  for(;;){
    if(clientConnected && pCharacteristic){
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
