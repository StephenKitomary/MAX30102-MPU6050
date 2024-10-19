#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// Threshold for movement detection (tuned for light sleep detection)
const float movement_threshold = 0.1;  // Adjust based on testing

// Variables to store previous acceleration values
float prev_ax = 0, prev_ay = 0, prev_az = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set a lower accelerometer range for more sensitivity
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: +-2G\n");

  // Set a lower filter bandwidth to reduce noise
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.println("Filter bandwidth set to: 5 Hz");

  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
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

  // Check if the movement exceeds the threshold for light sleep
  if (total_movement > movement_threshold) {
    Serial.println("Light sleep detected (movement above threshold).");
  } else {
    Serial.println("Deep sleep detected (minimal movement).");
  }

  // Print the values for debugging
  Serial.print("Acceleration X: "); Serial.print(a.acceleration.x);
  Serial.print(", Y: "); Serial.print(a.acceleration.y);
  Serial.print(", Z: "); Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Total movement: ");
  Serial.println(total_movement);

  Serial.println();
  delay(500);  // Adjust delay to tune sensitivity
}
