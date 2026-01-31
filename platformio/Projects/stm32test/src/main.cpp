#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <time.h>
#include <sys/time.h>

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

// IMU default data rates
#define IMU_ACCELERATOR_DATA_RATE LSM6DS_RATE_104_HZ
#define IMU_ACCELERATOR_GYRO_RATE LSM6DS_RATE_104_HZ


// Live monitoring modes
enum LiveMode : uint8_t { LIVE_OFF = 0, LIVE_UNITS = 1, LIVE_CSV = 2 };


// Default values
#define SERIAL_BAUD 115200
#define GPS_BAUD 9600
#define LOG_RATE_HZ 100
#define LIVE_MONITORING_DEFAULT LIVE_CSV
#define LIVE_UNITS_RATE_HZ 1
#define LOG_FILE_PREFIX "dataLog"
#define IMU_ADDRESS 0x6A
#define GPS_TIME_SYNC_INTERVAL 3600000  // Resync every 1 hour
#define GPS_TIME_MAX_DRIFT 2  // Max allowed drift in seconds before resync

// DEBUG FLAGS
#define DEBUG_DISABLE_GPS false  // Set to true to disable GPS by default


Adafruit_ISM330DHCX imu;
SdFat SD;
File logFile;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

bool imuFound = false;
bool sdFound = false;
bool logging = false;
LiveMode liveMode = LIVE_MONITORING_DEFAULT;
bool gpsEnabled = !DEBUG_DISABLE_GPS;  // GPS enabled/disabled based on debug flag
volatile bool dataReady = false;
unsigned long logCounter = 0;
char currentLogFile[32] = "";

// GPS data
double gpsLat = 0.0;
double gpsLng = 0.0;
double gpsAlt = 0.0;
double gpsSpeed = 0.0;  // Raw doppler speed from GPS in m/s
double gpsCourse = 0.0; // Raw course from GPS in degrees
bool gpsTimeValid = false;  // Track if we have valid GPS time
unsigned long lastGpsTimeSync = 0;  // Track last sync time
int gpsHour = 0;
int gpsMinute = 0;
int gpsSecond = 0;
int gpsMillisecond = 0;
unsigned long lastSystemTimeUpdate = 0;  // Track last system time update
time_t gpsTimeAtSync = 0;  // GPS time_t value at sync
unsigned long millisAtSync = 0;  // millis() at sync

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
  Serial.println("8 - Toggle live monitoring (OFF/UNITS (1HZ)/ CSV)");
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
    logFile.println("gps_time_ms,ax,ay,az,gx,gy,gz,t,lat,lng,alt,speed_mps,course_deg");
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
  if (liveMode == LIVE_OFF) Serial.println("OFF");
  else if (liveMode == LIVE_UNITS) Serial.println("UNITS");
  else Serial.println("CSV");
  if (logging) {
    Serial.print("Samples: ");
    Serial.println(logCounter);
  }
  Serial.println("---------------------");
  Serial.print("> ");
}

void toggleLiveMonitoring() {
  if (liveMode == LIVE_OFF) {
    liveMode = LIVE_CSV;
  } else if (liveMode == LIVE_CSV) {
    liveMode = LIVE_UNITS;
  } else {
    liveMode = LIVE_OFF;
  }

  Serial.print("Live monitoring: ");
  if (liveMode == LIVE_OFF) Serial.println("OFF");
  else if (liveMode == LIVE_UNITS) Serial.println("UNITS");
  else Serial.println("CSV");
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

void updateGPSData() {
  if (!gpsEnabled) return;
  
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }
  
  if (gps.location.isValid()) {
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    gpsAlt = gps.altitude.meters();
  }
  if (gps.speed.isValid()) {
    gpsSpeed = gps.speed.mps();
  }
  if (gps.course.isValid()) {
    gpsCourse = gps.course.deg();
  }
  if (gps.time.isValid()) {
    unsigned long now_ms = millis();
    
    if (!gpsTimeValid) {
      struct tm timeinfo = {};
      timeinfo.tm_year = gps.date.year() - 1900;
      timeinfo.tm_mon = gps.date.month() - 1;
      timeinfo.tm_mday = gps.date.day();
      timeinfo.tm_hour = gps.time.hour();
      timeinfo.tm_min = gps.time.minute();
      timeinfo.tm_sec = gps.time.second();
      gpsTimeAtSync = mktime(&timeinfo);
      millisAtSync = now_ms;
      
      gpsTimeValid = true;
      lastGpsTimeSync = now_ms;
      lastSystemTimeUpdate = now_ms;
      
      Serial.println("GPS time acquired - system clock synchronized");
    }
    else if (now_ms - lastGpsTimeSync >= GPS_TIME_SYNC_INTERVAL) {
      struct tm timeinfo = {};
      timeinfo.tm_year = gps.date.year() - 1900;
      timeinfo.tm_mon = gps.date.month() - 1;
      timeinfo.tm_mday = gps.date.day();
      timeinfo.tm_hour = gps.time.hour();
      timeinfo.tm_min = gps.time.minute();
      timeinfo.tm_sec = gps.time.second();
      gpsTimeAtSync = mktime(&timeinfo);
      millisAtSync = now_ms;
      lastGpsTimeSync = now_ms;
      lastSystemTimeUpdate = now_ms;
    }
  }
}

void calculateCurrentTime(time_t& now, uint16_t& millis_val, uint8_t& hours, uint8_t& minutes, uint8_t& seconds) {
  if (gpsTimeValid) {
    unsigned long elapsed_ms = millis() - millisAtSync;
    now = gpsTimeAtSync + (elapsed_ms / 1000);
    millis_val = elapsed_ms % 1000;
  } else {
    unsigned long uptime_ms = millis();
    now = uptime_ms / 1000;
    millis_val = uptime_ms % 1000;
  }
  
  struct tm* timeinfo = localtime(&now);
  if (!timeinfo) {
    now = 0;
    timeinfo = localtime(&now);
  }
  
  hours = timeinfo->tm_hour;
  minutes = timeinfo->tm_min;
  seconds = timeinfo->tm_sec;
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  
  char cmd = Serial.read();
  while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
    Serial.read();
  }
  
  switch (cmd) {
    case '?': showMenu(); break;
    case '1': listFiles(); break;
    case '2': logging ? stopLogging() : startLogging(); break;
    case '3': toggleGPS(); break;
    case '4': viewFile(); break;
    case '5': deleteFile(); break;
    case '6': showStatus(); break;
    case '7': downloadFile(); break;
    case '8': toggleLiveMonitoring(); break;
    default: 
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      Serial.print("> ");
      break;
  }
}

void logToSD(time_t now, uint16_t millis_val, uint8_t hours, uint8_t minutes, uint8_t seconds, 
             float ax, float ay, float az, float gx, float gy, float gz, float temp) {
  if (!logging || !sdFound) return;
  
  logFile = SD.open(currentLogFile, FILE_WRITE);
  if (logFile) {
    char timeStr[20];
    sprintf(timeStr, "%02d:%02d:%02d.%03d", hours, minutes, seconds, millis_val);
    logFile.print(timeStr);
    logFile.print(",");
    logFile.print(ax, 6); logFile.print(",");
    logFile.print(ay, 6); logFile.print(",");
    logFile.print(az, 6); logFile.print(",");
    logFile.print(gx, 6); logFile.print(",");
    logFile.print(gy, 6); logFile.print(",");
    logFile.print(gz, 6); logFile.print(",");
    logFile.print(temp, 6); logFile.print(",");
    logFile.print(gpsLat, 8); logFile.print(",");
    logFile.print(gpsLng, 8); logFile.print(",");
    logFile.print(gpsAlt, 2); logFile.print(",");
    logFile.print(gpsSpeed, 3); logFile.print(",");
    logFile.println(gpsCourse, 2);
    logFile.close();
    logCounter++;
    
    if (logCounter % 1000 == 0) {
      Serial.print("Logged ");
      Serial.print(logCounter);
      Serial.println(" samples");
    }
    
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void printLiveMonitoring(uint8_t hours, uint8_t minutes, uint8_t seconds, uint16_t millis_val,
                         float ax, float ay, float az, float gx, float gy, float gz, float temp) {
  if (liveMode == LIVE_UNITS) {
    static unsigned long lastLiveOutput = 0;
    unsigned long currentTime = millis();
    unsigned long liveInterval = 1000 / LIVE_UNITS_RATE_HZ;
    
    if (currentTime - lastLiveOutput >= liveInterval) {
      lastLiveOutput = currentTime;
      
      Serial.print("GPS: ");
      Serial.print(gps.location.isValid() ? "FIX " : "NOFIX ");
      Serial.print("("); Serial.print(gps.satellites.value()); Serial.print(" sats) ");
      Serial.print(hours); Serial.print(":"); Serial.print(minutes); Serial.print(":"); Serial.print(seconds);
      Serial.print(" | Lat: "); Serial.print(gpsLat, 6);
      Serial.print(" Lng: "); Serial.print(gpsLng, 6);
      Serial.print(" Alt: "); Serial.print(gpsAlt, 1); Serial.print(" m ");
      Serial.print("Speed: "); Serial.print(gpsSpeed, 1); Serial.print(" m/s ");
      Serial.print("Course: "); Serial.print(gpsCourse, 1); Serial.print(" deg | ");
      Serial.print("Accel X: "); Serial.print(ax);
      Serial.print(" Y: "); Serial.print(ay);
      Serial.print(" Z: "); Serial.print(az);
      Serial.print(" m/s^2 | ");
      Serial.print("Gyro X: "); Serial.print(gx);
      Serial.print(" Y: "); Serial.print(gy);
      Serial.print(" Z: "); Serial.print(gz);
      Serial.print(" rad/s | ");
      Serial.print("Temp: "); Serial.print(temp);
      Serial.println(" C");
    }
  } else if (liveMode == LIVE_CSV) {
    char timeStr[20];
    sprintf(timeStr, "%02d:%02d:%02d.%03d", hours, minutes, seconds, millis_val);
    Serial.print(timeStr); Serial.print(",");
    Serial.print(ax, 6); Serial.print(",");
    Serial.print(ay, 6); Serial.print(",");
    Serial.print(az, 6); Serial.print(",");
    Serial.print(gx, 6); Serial.print(",");
    Serial.print(gy, 6); Serial.print(",");
    Serial.print(gz, 6); Serial.print(",");
    Serial.print(temp, 6); Serial.print(",");
    Serial.print(gpsLat, 8); Serial.print(",");
    Serial.print(gpsLng, 8); Serial.print(",");
    Serial.print(gpsAlt, 2); Serial.print(",");
    Serial.print(gpsSpeed, 3); Serial.print(",");
    Serial.println(gpsCourse, 2);
  }
}

void processIMUData(time_t now, uint16_t millis_val, uint8_t hours, uint8_t minutes, uint8_t seconds) {
  if (!imuFound || !dataReady) return;
  
  dataReady = false;
  
  if (logging && gpsEnabled && !gpsTimeValid) {
    static unsigned long lastWaitMsg = 0;
    if (millis() - lastWaitMsg > 5000) {
      Serial.println("Waiting for GPS time fix...");
      lastWaitMsg = millis();
    }
    return;
  }
  
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);
  
  logToSD(now, millis_val, hours, minutes, seconds, 
          accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
          gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, temp.temperature);
  
  printLiveMonitoring(hours, minutes, seconds, millis_val,
                      accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                      gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, temp.temperature);
}

void setup() {

  // Initialize LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize Serial
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  
  Serial.println("ISM330DHCX Test");
  
  // Initialize I2C
  Wire.begin();
  delay(200);

  // Initialize GPS (only if not disabled by debug flag)
  if (!DEBUG_DISABLE_GPS) {
    Serial.println("Initializing GPS...");
    pinMode(GPS_SLEEP_PIN, OUTPUT);
    digitalWrite(GPS_SLEEP_PIN, HIGH);  // Wake up GPS
    delay(100);
    gpsSerial.begin(GPS_BAUD);
    delay(200);
    Serial.println("GPS initialized");
  } else {
    Serial.println("GPS disabled by DEBUG_DISABLE_GPS flag");
    pinMode(GPS_SLEEP_PIN, OUTPUT);
    digitalWrite(GPS_SLEEP_PIN, LOW);  // Keep GPS in sleep mode to save power
  }

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
  
  Serial.print("Trying I2C address 0x");
  Serial.print(IMU_ADDRESS, HEX);
  Serial.println("...");
  
  
  // Try to initialize IMU at default address
  if (!imu.begin_I2C(IMU_ADDRESS)) {
    Serial.print("Not found at 0x");
    Serial.println(IMU_ADDRESS, HEX);
  } else {
    Serial.print("ISM330DHCX Found at 0x");
    Serial.println(IMU_ADDRESS, HEX);
    imuFound = true;
  }
  
  // Configure sensor ranges if IMU found
  if (imuFound) {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(IMU_ACCELERATOR_DATA_RATE);
    imu.setGyroDataRate(IMU_ACCELERATOR_GYRO_RATE);
    
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
  updateGPSData();
  
  time_t now;
  uint16_t millis_val;
  uint8_t hours, minutes, seconds;
  calculateCurrentTime(now, millis_val, hours, minutes, seconds);
  
  handleSerialCommands();
  processIMUData(now, millis_val, hours, minutes, seconds);
}