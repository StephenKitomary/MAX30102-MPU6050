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

// Timing variables
unsigned long lastMPURead = 0;
unsigned long lastMAXRead = 0;
unsigned long lastBLEUpdate = 0;

// Sampling intervals
const unsigned long MPU_INTERVAL = 100;   // 100 ms for MPU6050
const unsigned long MAX_INTERVAL = 10;   // 10 ms for MAX30105
const unsigned long BLE_INTERVAL = 500;  // 500 ms for BLE notifications

float prev_ax = 0, prev_ay = 0, prev_az = 0;
float delta_ax = 0, delta_ay = 0, delta_az = 0; // Declare globally

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

void setup() {
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
  unsigned long currentTime = millis();

  // Read MPU6050 at regular intervals
  if (currentTime - lastMPURead >= MPU_INTERVAL) {
    lastMPURead = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate movement
    delta_ax = abs(a.acceleration.x - prev_ax);
    delta_ay = abs(a.acceleration.y - prev_ay);
    delta_az = abs(a.acceleration.z - prev_az);
    prev_ax = a.acceleration.x;
    prev_ay = a.acceleration.y;
    prev_az = a.acceleration.z;

    Serial.print("MPU Movement: ");
    Serial.println(delta_ax + delta_ay + delta_az);
  }

  // Read MAX30105 at regular intervals
  if (currentTime - lastMAXRead >= MAX_INTERVAL) {
    lastMAXRead = currentTime;

    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;
      }
    }
    if (irValue < 10000) {
      beatsPerMinute = 0;
      beatAvg = 0;
    }

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.println(beatAvg);
  }

  // Send BLE notifications at regular intervals
  if (deviceConnected && (currentTime - lastBLEUpdate >= BLE_INTERVAL)) {
    lastBLEUpdate = currentTime;

    // Combine movement and heart rate data into one payload
    String data = String("Accel X: ") + String(prev_ax) +
                  ", Y: " + String(prev_ay) +
                  ", Z: " + String(prev_az) +
                  ", Movement: " + String(delta_ax + delta_ay + delta_az) +
                  ", BPM: " + String(beatsPerMinute) +
                  ", Avg BPM: " + String(beatAvg);

    pCharacteristic->setValue(data.c_str());
    pCharacteristic->notify();

    Serial.println("Sent data: " + data);
  }
}
