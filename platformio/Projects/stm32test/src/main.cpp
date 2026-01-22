#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// IMU interrupt pin
#define IMU_INT_PIN PB4

// GPS pins
#define GPS_RX_PIN PC0
#define GPS_TX_PIN PC1
#define GPS_SLEEP_PIN PA0  // Set HIGH to wake up GPS

// SD card SPI pins
#define SD_CS_PIN PB9
#define PIN_SPI_MISO PB14
#define PIN_SPI_MOSI PA10
#define PIN_SPI_SCK PB13

#define SD_FAT_TYPE 1

Adafruit_ISM330DHCX imu;
SdFat SD;
File logFile;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

bool imuFound = false;
bool sdFound = false;
bool logging = false;
bool liveMonitoring = false;
bool gpsEnabled = true;  // GPS enabled by default
volatile bool dataReady = false;
unsigned long logCounter = 0;
char currentLogFile[32] = "";

// GPS data
double gpsLat = 0.0;
double gpsLng = 0.0;
double gpsAlt = 0.0;
int gpsHour = 0;
int gpsMinute = 0;
int gpsSecond = 0;
int gpsMillis = 0;  // Centiseconds (0-99) * 10 = milliseconds

// Interrupt handler for IMU data ready
void imuInterruptHandler() {
  dataReady = true;
}

void showMenu() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("  STM32 IMU Data Logger Menu");
  Serial.println("========================================");
  Serial.println("1 - List files");
  if (logging) {
    Serial.println("2 - Stop logging");
  } else {
    Serial.println("2 - Start logging");
  }
  if (gpsEnabled) {
    Serial.println("3 - Disable GPS");
  } else {
    Serial.println("3 - Enable GPS");
  }
  Serial.println("4 - View file");
  Serial.println("5 - Delete file");
  Serial.println("6 - Status");
  Serial.println("7 - Download file");
  Serial.println("8 - Toggle live monitoring");
  Serial.println("? - Show this menu");
  Serial.println("========================================");
  Serial.print("> ");
}

void listFiles() {
  if (!sdFound) {
    Serial.println("SD card not available");
    return;
  }
  
  Serial.println("\nFiles on SD card:");
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root");
    Serial.print("> ");
    return;
  }
  
  File file;
  while (file = root.openNextFile()) {
    if (!file.isDirectory()) {
      char filename[64];
      file.getName(filename, sizeof(filename));
      Serial.print("  ");
      Serial.print(filename);
      Serial.print(" (");
      Serial.print(file.size());
      Serial.println(" bytes)");
    }
    file.close();
  }
  root.close();
  Serial.print("> ");
}

void startLogging() {
  if (!sdFound) {
    Serial.println("SD card not available");
    return;
  }
  
  if (logging) {
    Serial.println("Already logging!");
    return;
  }
  
  // Find next available log file name
  char filename[32];
  for (int i = 1; i < 1000; i++) {
    sprintf(filename, "dataLog%03d.TXT", i);
    if (!SD.exists(filename)) {
      break;
    }
  }
  
  logFile = SD.open(filename, FILE_WRITE);
  if (logFile) {
    logging = true;
    logCounter = 0;
    strcpy(currentLogFile, filename);  // Save filename for later use
    Serial.print("Logging started: ");
    Serial.println(filename);
    
    // Write header
    logFile.println("gps_time_ms,ax,ay,az,gx,gy,gz,t,lat,lng,alt");
    logFile.close();
  } else {
    Serial.println("Failed to create log file");
  }
  Serial.print("> ");
}

void stopLogging() {
  if (!logging) {
    Serial.println("Not logging");
    return;
  }
  
  logging = false;
  currentLogFile[0] = '\0';
  Serial.print("Logging stopped. ");
  Serial.print(logCounter);
  Serial.println(" samples written");
  Serial.print("> ");
}

void viewFile() {
  if (!sdFound) {
    Serial.println("SD card not available");
    return;
  }
  
  Serial.println("Enter filename:");
  Serial.print("> ");
  
  // Wait for filename input
  while (!Serial.available()) {
    delay(10);
  }
  
  String filename = Serial.readStringUntil('\n');
  filename.trim();
  
  File file = SD.open(filename.c_str(), FILE_READ);
  if (file) {
    Serial.println("\n--- File contents ---");
    while (file.available()) {
      Serial.write(file.read());
    }
    Serial.println("\n--- End of file ---");
    file.close();
  } else {
    Serial.print("Failed to open: ");
    Serial.println(filename);
  }
  Serial.print("> ");
}

void deleteFile() {
  if (!sdFound) {
    Serial.println("SD card not available");
    return;
  }
  
  Serial.println("Enter filename to delete:");
  Serial.print("> ");
  
  // Wait for filename input
  while (!Serial.available()) {
    delay(10);
  }
  
  String filename = Serial.readStringUntil('\n');
  filename.trim();
  
  if (SD.remove(filename.c_str())) {
    Serial.print("Deleted: ");
    Serial.println(filename);
  } else {
    Serial.print("Failed to delete: ");
    Serial.println(filename);
  }
  Serial.print("> ");
}

void showStatus() {
  Serial.println("\n--- System Status ---");
  Serial.print("IMU: ");
  Serial.println(imuFound ? "Connected" : "Not found");
  Serial.print("SD Card: ");
  Serial.println(sdFound ? "Connected" : "Not found");
  Serial.print("GPS Fix: ");
  Serial.println(gps.location.isValid() ? "Valid" : "No fix");
  Serial.print("GPS Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("GPS Chars Processed: ");
  Serial.println(gps.charsProcessed());
  Serial.print("Logging: ");
  Serial.println(logging ? "Active" : "Stopped");
  Serial.print("Live Monitoring: ");
  Serial.println(liveMonitoring ? "Active" : "Off");
  if (logging) {
    Serial.print("Samples: ");
    Serial.println(logCounter);
  }
  Serial.println("---------------------");
  Serial.print("> ");
}

void toggleLiveMonitoring() {
  liveMonitoring = !liveMonitoring;
  Serial.print("Live monitoring: ");
  Serial.println(liveMonitoring ? "ON" : "OFF");
  if (liveMonitoring) {
    Serial.println("Press '8' again to stop...");
  }
  Serial.print("> ");
}

void toggleGPS() {
  gpsEnabled = !gpsEnabled;
  Serial.print("GPS: ");
  Serial.println(gpsEnabled ? "ENABLED" : "DISABLED");
  if (!gpsEnabled) {
    Serial.println("GPS data will be zeros in logs");
  }
  Serial.print("> ");
}

void downloadFile() {
  if (!sdFound) {
    Serial.println("SD card not available");
    return;
  }
  
  Serial.println("Enter filename to download:");
  Serial.print("> ");
  
  // Wait for filename input
  while (!Serial.available()) {
    delay(10);
  }
  
  String filename = Serial.readStringUntil('\n');
  filename.trim();
  
  File file = SD.open(filename.c_str(), FILE_READ);
  if (file) {
    unsigned long fileSize = file.size();
    Serial.println("\n=== BEGIN FILE ===");
    Serial.print("Filename: ");
    Serial.println(filename);
    Serial.print("Size: ");
    Serial.print(fileSize);
    Serial.println(" bytes");
    Serial.println("==================");
    
    // Send file contents
    while (file.available()) {
      Serial.write(file.read());
    }
    
    Serial.println("\n=== END FILE ===");
    file.close();
  } else {
    Serial.print("Failed to open: ");
    Serial.println(filename);
  }
  Serial.print("> ");
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
  delay(200);

  // Initialize GPS
  Serial.println("Initializing GPS...");
  pinMode(GPS_SLEEP_PIN, OUTPUT);
  digitalWrite(GPS_SLEEP_PIN, HIGH);  // Wake up GPS
  delay(100);
  gpsSerial.begin(9600);
  delay(200);
  Serial.println("GPS initialized");

  // Configure SPI pins for SD card
  SPI.setMOSI(PIN_SPI_MOSI);
  SPI.setMISO(PIN_SPI_MISO);
  SPI.setSCLK(PIN_SPI_SCK);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  
  // Initialize SD card with retries
  Serial.println("Initializing SD card...");
  sdFound = false;
  for (int attempt = 1; attempt <= 10; attempt++) {
    Serial.print("Attempt ");
    Serial.print(attempt);
    Serial.print("/10... ");
    
    if (SD.begin(SD_CS_PIN)) {
      Serial.println("Success!");
      sdFound = true;
      break;
    } else {
      Serial.println("Failed");
      delay(500);
    }
  }
  
  if (!sdFound) {
    Serial.println("SD card initialization failed after 10 attempts!");
  } else {
    Serial.println("SD card initialized successfully");
  }
  
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
  
  // Show initial menu
  Serial.println("\nSetup complete!");
  showMenu();
}

void loop() {
  
  // Update GPS data continuously if GPS is enabled
  if (gpsEnabled) {
    while (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      gps.encode(c);
    }
    
    // Store latest GPS data when available
    if (gps.location.isValid()) {
      gpsLat = gps.location.lat();
      gpsLng = gps.location.lng();
    }
    if (gps.altitude.isValid()) {
      gpsAlt = gps.altitude.meters();
    }
    if (gps.time.isValid()) {
      gpsHour = gps.time.hour();
      gpsMinute = gps.time.minute();
      gpsSecond = gps.time.second();
      gpsMillis = gps.time.centisecond() * 10;  // Convert centiseconds to milliseconds
    }
  }
  
  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Clear any remaining newline/carriage return
    while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
      Serial.read();
    }
    
    switch (cmd) {
      case '?':
        showMenu();
        break;
      case '1':
        listFiles();
        break;
      case '2':
        if (logging) {
          stopLogging();
        } else {
          startLogging();
        }
        break;
      case '3':
        toggleGPS();
        break;
      case '4':
        viewFile();
        break;
      case '5':
        deleteFile();
        break;
      case '6':
        showStatus();
        break;
      case '7':
        downloadFile();
        break;
      case '8':
        toggleLiveMonitoring();
        break;
      default:
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        Serial.print("> ");
        break;
    }
  }
  
  // Handle IMU data
  if (imuFound && dataReady) {
    dataReady = false;
    
    // Read sensor data
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp);
    
    // Log to SD card if logging is active
    if (logging && sdFound) {
      logFile = SD.open(currentLogFile, FILE_WRITE);
      if (logFile) {
        // Write GPS timestamp with milliseconds
        char timeStr[20];
        sprintf(timeStr, "%02d:%02d:%02d.%03d", gpsHour, gpsMinute, gpsSecond, gpsMillis);
        logFile.print(timeStr);
        logFile.print(",");
        logFile.print(accel.acceleration.x, 6);
        logFile.print(",");
        logFile.print(accel.acceleration.y, 6);
        logFile.print(",");
        logFile.print(accel.acceleration.z, 6);
        logFile.print(",");
        logFile.print(gyro.gyro.x, 6);
        logFile.print(",");
        logFile.print(gyro.gyro.y, 6);
        logFile.print(",");
        logFile.print(gyro.gyro.z, 6);
        logFile.print(",");
        logFile.print(temp.temperature, 6);
        logFile.print(",");
        logFile.print(gpsLat, 8);
        logFile.print(",");
        logFile.print(gpsLng, 8);
        logFile.print(",");
        logFile.println(gpsAlt, 2);
        logFile.close();
        logCounter++;
        
        // Print progress every 1000 samples
        if (logCounter % 1000 == 0) {
          Serial.print("Logged ");
          Serial.print(logCounter);
          Serial.println(" samples");
        }
      }
      
      // Blink LED when logging
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    
    // Print to serial only if live monitoring is active
    if (liveMonitoring) {
      Serial.print("GPS: ");
      if (gps.location.isValid()) {
        Serial.print("FIX ");
      } else {
        Serial.print("NOFIX ");
      }
      Serial.print("(");
      Serial.print(gps.satellites.value());
      Serial.print(" sats) ");
      Serial.print(gpsHour);
      Serial.print(":");
      Serial.print(gpsMinute);
      Serial.print(":");
      Serial.print(gpsSecond);
      Serial.print(" | Lat: ");
      Serial.print(gpsLat, 6);
      Serial.print(" Lng: ");
      Serial.print(gpsLng, 6);
      Serial.print(" Alt: ");
      Serial.print(gpsAlt, 1);
      Serial.println("m");
      
      Serial.print("Accel X: "); Serial.print(accel.acceleration.x);
      Serial.print(" Y: "); Serial.print(accel.acceleration.y);
      Serial.print(" Z: "); Serial.print(accel.acceleration.z);
      Serial.print(" m/s^2  |  ");
      
      Serial.print("Gyro X: "); Serial.print(gyro.gyro.x);
      Serial.print(" Y: "); Serial.print(gyro.gyro.y);
      Serial.print(" Z: "); Serial.print(gyro.gyro.z);
      Serial.print(" rad/s  |  ");
      
      Serial.print("Temp: "); Serial.print(temp.temperature);
      Serial.println(" C");
    }
  }
}