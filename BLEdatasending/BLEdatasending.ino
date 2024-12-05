#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "MAX30105.h"
#include "heartRate.h"

// MPU6050 Setup
Adafruit_MPU6050 mpu;

// MAX30105 Setup
MAX30105 particleSensor;

const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;

// BLE Setup
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int LED_PIN = 0;

// UUIDs for BLE
#define SERVICE_UUID        "0000fff0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000fff0-0000-1000-8000-00805f9b34fb"

// Threshold for movement detection
const float movement_threshold = 0.1;
float prev_ax = 0, prev_ay = 0, prev_az = 0;

// BLE Server Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    digitalWrite(LED_PIN, HIGH);
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    digitalWrite(LED_PIN, LOW);
  }
};

void setup(void) {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Initialize MAX30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0);

  // Initialize BLE
  BLEDevice::init("MPU6050_MAX30105_BLE");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("Waiting for a client connection...");
}

void loop() {
  if (!deviceConnected) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    return;
  }

  // MPU6050 Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float delta_ax = abs(a.acceleration.x - prev_ax);
  float delta_ay = abs(a.acceleration.y - prev_ay);
  float delta_az = abs(a.acceleration.z - prev_az);
  prev_ax = a.acceleration.x;
  prev_ay = a.acceleration.y;
  prev_az = a.acceleration.z;
  float total_movement = delta_ax + delta_ay + delta_az;

  // MAX30105 Data
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }
  if (irValue < 10000) {
    beatsPerMinute = 0;
    beatAvg = 0;
  }

  // Combine data
  String data = String("Accel X: ") + String(a.acceleration.x) +
                ", Y: " + String(a.acceleration.y) +
                ", Z: " + String(a.acceleration.z) +
                ", Total movement: " + String(total_movement) +
                ", IR: " + String(irValue) +
                ", BPM: " + String(beatsPerMinute) +
                ", Avg BPM: " + String(beatAvg);

  // Send BLE notification
  pCharacteristic->setValue(data.c_str());
  pCharacteristic->notify();
  Serial.println("Sent data: " + data);

  delay(500);
}
