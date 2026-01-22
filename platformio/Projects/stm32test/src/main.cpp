#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <Wire.h>

// Built-in LED on LoRa-E5-mini is on PB5
#define LED_BUILTIN PB5

// IMU interrupt pin
#define IMU_INT_PIN PB4

Adafruit_ISM330DHCX imu;
bool imuFound = false;
volatile bool dataReady = false;

// Interrupt handler for IMU data ready
void imuInterruptHandler() {
  dataReady = true;
  Serial.println("Interrupt triggered");
}

void setup() {

  // Initialize LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize Serial
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ISM330DHCX Test");
  
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  Serial.println("Trying I2C address 0x6A...");
  
  
  // Try to initialize with address 0x6A (default)
  if (!imu.begin_I2C(0x6A)) {
    Serial.println("Not found at 0x6A");
    Serial.println("Trying I2C address 0x6B...");
    // Try alternate address 0x6B
    if (!imu.begin_I2C(0x6B)) {
      Serial.println("Failed to find ISM330DHCX chip at 0x6A or 0x6B");
      
      imuFound = false;
    } else {
      Serial.println("ISM330DHCX Found at 0x6B!");
      imuFound = true;
    }
  } else {
    Serial.println("ISM330DHCX Found at 0x6A!");
    imuFound = true;
  }
  
  // Configure sensor ranges if IMU found
  if (imuFound) {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
    
    // Configure interrupt pin
    pinMode(IMU_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imuInterruptHandler, RISING);
    
    // Enable data ready interrupt on INT1
    imu.configInt1(false, false, true);  // INT1 for data ready
    
    Serial.println("IMU interrupt configured on PB4");
  }
}

void loop() {
  
  if (imuFound && dataReady) {
    dataReady = false;
    
    // Read sensor data
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp);
    
    // Print accelerometer data
    Serial.print("Accel X: "); Serial.print(accel.acceleration.x);
    Serial.print(" Y:      "); Serial.print(accel.acceleration.y);
    Serial.print(" Z:      "); Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2");
    
    // Print gyroscope data
    Serial.print("Gyro X:  "); Serial.print(gyro.gyro.x);
    Serial.print(" Y:      "); Serial.print(gyro.gyro.y);
    Serial.print(" Z:      "); Serial.print(gyro.gyro.z);
    Serial.println(" rad/s");
    
    // Print temperature
    Serial.print("Temp:    "); Serial.print(temp.temperature);
    Serial.println(" C");
    
    Serial.println();
  }
  
  // Blink LED
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}