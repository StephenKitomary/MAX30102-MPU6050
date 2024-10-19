#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// MPU6050 Setup
Adafruit_MPU6050 mpu;

// BLE Setup
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
int LED_PIN = 0; // Use pin 0 for the LED

// UUIDs for the BLE service and characteristic
#define SERVICE_UUID        "0000fff0-0000-1000-8000-00805f9b34fb" // Replace if necessary
#define CHARACTERISTIC_UUID "0000fff0-0000-1000-8000-00805f9b34fb" // Replace if necessary

// Threshold for movement detection
const float movement_threshold = 0.1;

// Variables to store previous acceleration values
float prev_ax = 0, prev_ay = 0, prev_az = 0;

// Callback class for BLE connection status
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    digitalWrite(LED_PIN, HIGH); // Turn on LED when connected
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    digitalWrite(LED_PIN, LOW); // Turn off LED when disconnected
  }
};

void setup(void) {
  Serial.begin(115200);

  // Initialize the LED for connection feedback
  pinMode(LED_PIN, OUTPUT);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Initialize BLE
  BLEDevice::init("MPU6050_BLE_Sensor");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the BLE service
  pService->start();

  // Start advertising the BLE service
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection...");
}

void loop() {
  // Blink the LED until the device is connected
  if (!deviceConnected) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  } else {
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate the change in acceleration (delta) for X, Y, Z axes
    float delta_ax = abs(a.acceleration.x - prev_ax);
    float delta_ay = abs(a.acceleration.y - prev_ay);
    float delta_az = abs(a.acceleration.z - prev_az);

    // Store current acceleration values for the next loop iteration
    prev_ax = a.acceleration.x;
    prev_ay = a.acceleration.y;
    prev_az = a.acceleration.z;

    // Sum of the changes in acceleration
    float total_movement = delta_ax + delta_ay + delta_az;

    // Create a string of data to send over BLE
    String data = String("Accel X: ") + String(a.acceleration.x) + 
                  ", Y: " + String(a.acceleration.y) + 
                  ", Z: " + String(a.acceleration.z) + 
                  ", Total movement: " + String(total_movement);

    // Send data over BLE if device is connected
    if (deviceConnected) {
      pCharacteristic->setValue(data.c_str());  // Send the data as a BLE notification
      pCharacteristic->notify();  // Notify the client with new data
      Serial.println("Sent data: " + data);  // Print to serial for debugging
    }
  }

  delay(500);  // Adjust delay as necessary
}
