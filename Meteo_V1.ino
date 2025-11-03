/*
================================================================================
  WELCOME TO THE METEOR FLIGHT COMPUTER 
================================================================================
Hello and welcome! This code runs the METEOR Flight Computer — a self-contained
high-altitude balloon avionics system. It automatically reads data from all
onboard sensors, logs that data to an SD card, and sends APRS telemetry packets
for live tracking during flight.

-------------------------------------------------------------------------------
 WHAT THIS PROGRAM DOES
-------------------------------------------------------------------------------
• Reads GPS location, altitude, and time from the u-blox GPS
• Measures pressure and internal temperature using the BMP085
• Records acceleration, rotation, and orientation using the LSM6DS3 IMU
• Reads magnetic field data from the LIS3MDL magnetometer
• Monitors battery voltage and ambient temperature (DS18B20)
• Logs all data to the SD card as a CSV file with timestamps
• Transmits APRS telemetry packets through the DRA818V radio

-------------------------------------------------------------------------------
 HOW TO USE IT
-------------------------------------------------------------------------------
1. **Power On**
   - Plug in the battery or USB power to the flight computer.
   - Wait ~5–10 seconds for sensors and the radio to initialize.
   - The LED indicators and serial monitor (if connected) will show startup logs.

2. **Set Your Callsign and Frequency**
   - Open this file in the Arduino IDE.
   - Scroll down to the section labeled "APRS / RADIO CONFIG".
   - Change the following lines as needed:
        char CallSign[7] = "K5FPS";     // Your callsign
        int8_t CallNumber = 11;         // SSID (-11, -12, etc.)
        char Frequency[9] = "144.3900"; // APRS frequency in MHz
   - Optionally edit `StatusMessage` or `comment` for your custom text.

3. **Upload the Code**
   - Board: Arduino M0 / SAMD21 (as used in LightAPRS)
   - Connect via USB and click “Upload” in Arduino IDE.

4. **Insert an SD Card**
   - Use a FAT32-formatted microSD card (8GB or smaller recommended).
   - The flight computer will automatically create a new CSV log file on startup.
   - Files are named by date/time (example: FLIGHT_2025-11-03_10-15-00.csv).

5. **Flight Operation**
   - When powered, the METEOR board continuously:
       → Reads all sensors
       → Transmits APRS position & telemetry packets
       → Saves data locally to SD
   - The APRS beacon interval and battery sleep settings can be adjusted:
        BeaconWait = 10;   // Seconds between APRS beacons
        BattWait = 60;     // Sleep delay when voltage is low
        BattMin = 2.8;     // Minimum battery voltage before sleep

6. **After Flight**
   - Power off the computer and remove the SD card.
   - Open the `.csv` file in Excel, Numbers, or Google Sheets to view data:
       Timestamp, GPS, altitude, pressure, temperature, etc.

-------------------------------------------------------------------------------
 TROUBLESHOOTING
-------------------------------------------------------------------------------
• **BMP FAIL / GPS not detected** → Power cycle and wait 5 seconds after boot.
• **No SD log file created** → Ensure SD card is inserted and formatted FAT32.
• **No APRS packets heard** → Check antenna, frequency, and callsign settings.
• **Low battery performance** → Ensure >3.3V at VIN or recharge Li-ion pack.
• **RTC not recording accurate time stamps** → This can occur due to lack of connection to reliable time source, or miscalibration. To fix run the RTC Calibration program below. Input current time and date.
• **If other problems occur** → Please feel free to contact distributor.
-------------------------------------------------------------------------------
  QUICK NOTES
-------------------------------------------------------------------------------
• The program automatically adjusts APRS path at high altitudes.
• The RTC (DS3231) timestamps every log entry.
• You can monitor output using the Arduino Serial Monitor at 115200 baud.
• Designed for the LightAPRS or custom SAMD21-based flight computer boards.

-------------------------------------------------------------------------------
  CUSTOMIZATION SUMMARY
-------------------------------------------------------------------------------
🔧 Edit these variables to customize behavior:
    - `CallSign`, `CallNumber`, `Symbol`, `Frequency`
    - `BeaconWait` (time between APRS packets)
    - `BattMin` and `BattWait` (battery thresholds)
    - `StatusMessage` and `comment` (display text in APRS)
    - `pathSize`, `Wide1`, `Wide2` (radio path hops)

================================================================================

 ========= RTC Time Calibration Code =========
 #include <RTClib.h>
 RTC_DS3231 rtc;

 void setup() {
   Serial.begin(115200);
     Serial.println("Couldn't find RTC");
     while (1);
   }

   // Replace this with the *exact* current local time when uploading
   
   rtc.adjust(DateTime(  , , , , , #00));  // YYYY, MM, DD, HH, MM, SS

   Serial.println("RTC adjusted.");
}

void loop() {}
// ============================================================================
*/
// --- LIBRARIES ---
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <MemoryFree.h>

#include <ZeroAPRS.h>                       // APRS / DRA818V radio
#include <SparkFun_Ublox_Arduino_Library.h> // u-blox GPS
#include <Adafruit_BMP085.h>                // Barometer
#include <Adafruit_LIS3MDL.h>               // Magnetometer
#include <Adafruit_Sensor.h>                // Unified sensor framework
#include <Adafruit_LSM6DS3.h>               // IMU (accelerometer + gyro)
#include <RTClib.h>                         // DS3231 real-time clock
#include <OneWire.h>                        // DS18B20 temp communication
#include <DallasTemperature.h>
#include <SPI.h>
#include <SdFat.h>                          // SD card logging

// --- SD card interface ---
SdFat SD;
File logFile;

// =================== PIN DEFINITIONS ===================
// Assign labels to pins so the code is readable and wiring is easy to follow.
#define SD_CS         A2     // SD card chip select
#define TEMP_A3_PIN   A1     // TMP36 analog temp sensor
#define BattPin       A5     // Battery voltage read

#define GpsPwr        7      // GPS power control
#define PttPin        3      // Radio transmit (PTT)
#define PwDwPin       A3     // Radio power-down
#define PowerHL       A4     // Radio power select (high/low)

// --- Simple macros to toggle power lines ---
#define GpsON       digitalWrite(GpsPwr, LOW)
#define GpsOFF      digitalWrite(GpsPwr, HIGH)
#define PttON       digitalWrite(PttPin, HIGH)
#define PttOFF      digitalWrite(PttPin, LOW)
#define RadioON     digitalWrite(PwDwPin, HIGH)
#define RadioOFF    digitalWrite(PwDwPin, LOW)
#define RfHiPwr     digitalWrite(PowerHL, HIGH)
#define RfLowPwr    digitalWrite(PowerHL, LOW)

// =================== RADIO / APRS CONFIG ===================
// These values define the station identity and how it transmits.

char    CallSign[7] = "K5FPS";          // Callsign for APRS packets, CHANGE THIS TO EDIT CALL SIGN
int8_t  CallNumber  = 11;               // SSID (-11), CHANGE THIS TO EDIT CALL NUMBER

char    Symbol      = '/O';             // APRS map symbol
bool    alternateSymbolTable = false;   // Use standard APRS symbols
char    Frequency[9] = "144.3900";      // APRS frequency in MHz

// Basic APRS messages
char    comment[50]        = "METEOR APRS v1.1 (Adafruit IMU)";
char    StatusMessage[50]  = "METEOR Flight Computer online";

// Radio & timing parameters
uint16_t  BeaconWait = 10;     // Seconds between APRS packets
uint16_t  BattWait   = 60;     // Sleep time if battery low
float     BattMin    = 2.8;    // Minimum voltage before sleep
float     DraHighVolt= 5.0;    // Switch to high power above this voltage

// Digipeater path settings
uint8_t   Wide1 = 1;
uint8_t   Wide2 = 1;
uint8_t   pathSize = 2;
bool      autoPathSizeHighAlt = true; // Reduce hops at high altitude

// System state
bool      aliveStatus = true;
bool      radioSetup  = false;
static char telemetry_buff[120]; // Stores outgoing APRS message
uint16_t  TxCount = 1;           // Number of transmissions

// =================== GPS SETTINGS ===================
// Handles u-blox GPS configuration and recovery
int16_t   GpsResetTime = 1800;
bool      ublox_high_alt_mode_enabled = false;
int16_t   GpsInvalidTime = 0;
bool      gpsSetup = false;

// =================== SENSOR OBJECTS ===================
// Creates the actual hardware connections
SFE_UBLOX_GPS myGPS;
Adafruit_BMP085 bmp;
Adafruit_LIS3MDL lis3mdl;
RTC_DS3231 rtc;
Adafruit_LSM6DS3 imu;

// DS18B20 1-Wire setup
#define DS18B20_PIN  A1
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

// Track whether each sensor is working
bool bmp_ok=false, mag_ok=false, rtc_ok=false, imu_ok=false, ds_ok=false;

// =================== SD CONFIG ===================
bool sd_ok = false;
char currentLogFile[40] = "flightlog.csv";

// =================== SETUP ===================
void setup() {
  // --- Basic startup configuration ---
  analogReference(AR_INTERNAL1V65); // Use 1.65V ADC reference for consistent readings

  // Set pin modes
  pinMode(PttPin, OUTPUT);
  pinMode(GpsPwr, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PwDwPin, OUTPUT);
  pinMode(PowerHL, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(TEMP_A3_PIN, INPUT);
  analogReadResolution(10);

  // Ensure all hardware starts off
  GpsOFF; PttOFF; RadioOFF; RfLowPwr;

  delay(3000); // Let voltage stabilize

  // Power on GPS and shared I2C bus sensors
  GpsON; delay(1500);
  Wire.begin(); SPI.begin();

  // --- Initialize barometer (retry once if needed) ---
  bmp_ok = bmp.begin();
  if (!bmp_ok) { delay(500); bmp_ok = bmp.begin(); }
  SerialUSB.println(bmp_ok ? F("BMP OK") : F("BMP FAIL"));

  // Reset I2C bus if stuck
  for (int i = 0; i < 9; i++) {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, HIGH); delay(2);
    digitalWrite(SCL, LOW);  delay(2);
  }
  pinMode(SDA, INPUT_PULLUP); pinMode(SCL, INPUT_PULLUP);
  Wire.begin();

  // Start serial console
  SerialUSB.begin(115200);
  delay(1500);
  SerialUSB.println(F("\n=== METEOR Boot Sequence ==="));

  // --- Initialize APRS radio ---
  Serial1.begin(9600);
  APRS_init();
  APRS_setCallsign(CallSign, CallNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", Wide1);
  APRS_setPath2("WIDE2", Wide2);
  APRS_setPathSize(pathSize);
  APRS_useAlternateSymbolTable(alternateSymbolTable);
  APRS_setSymbol(Symbol);
  APRS_setGain(2);
  delay(2000);
  configDra818(Frequency);

  // --- Initialize sensors ---
  if (lis3mdl.begin_I2C()) {
    mag_ok = true;
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  }

  rtc_ok = rtc.begin();
  if (rtc_ok && rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  if (imu.begin_I2C(0x6A) || imu.begin_I2C(0x6B)) {
    imu_ok = true;
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  }

  ds18b20.begin();
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  ds_ok = (t > -100 && t < 150);

  // --- SD card setup ---
  SerialUSB.println(F("[SD] Initializing..."));
  sd_ok = SD.begin(SD_CS);
  if (!sd_ok) { delay(500); sd_ok = SD.begin(SD_CS); }
  if (sd_ok) ensureCSVHeader();

  SerialUSB.println(F("=== Setup Complete ==="));
}

// =================== MAIN LOOP ===================
// Runs continuously after setup().
void loop() {
  if (readBatt() > BattMin) {

    // Send a status beacon every 10 seconds
    static uint32_t lastStatus = 0;
    uint32_t now = millis();
    if (now - lastStatus >= 10000UL) {
      sendStatus();
      lastStatus = now;
    }

    // Ensure GPS is running
    if (!gpsSetup) gpsStart();
    if (!ublox_high_alt_mode_enabled) setupUBloxDynamicModel();

    // Only proceed if GPS has data
    if (myGPS.getPVT()) {
      gpsDebug();

      if (myGPS.getFixType() != 0 && myGPS.getSIV() > 3) {
        GpsInvalidTime = 0;
        updatePosition();
        updateTelemetry();

        // Adjust APRS path at high altitude
        if (autoPathSizeHighAlt && (myGPS.getAltitude() / 1000.f) > 3000)
          APRS_setPathSize(1);
        else
          APRS_setPathSize(pathSize);

        sendStatus();
        sendLocation();
        freeMem();
        sleepSeconds(BeaconWait);
      } else {
        GpsInvalidTime++;
        if (GpsInvalidTime > GpsResetTime) {
          GpsOFF; ublox_high_alt_mode_enabled = false; delay(1000); GpsON;
          GpsInvalidTime = 0;
        }
        updateTelemetry();
        sleepSeconds(2);
      }
    } else {
      updateTelemetry();
      sleepSeconds(2);
    }
  } else {
    sleepSeconds(BattWait); // Battery low — wait longer between loops
  }
}

// =================== GPS / RADIO HELPERS ===================
// These handle startup and communication with GPS and radio.

void gpsStart() {
  bool gpsBegin = false;
  while (!gpsBegin) {
    GpsON; delay(1000);
    Wire.begin();
    gpsBegin = myGPS.begin();
    if (!gpsBegin) delay(2000);
  }
  myGPS.setUART1Output(0);
  myGPS.setUART2Output(0);
  myGPS.setI2COutput(COM_TYPE_UBX);
  myGPS.saveConfiguration();
  gpsSetup = true;
}

void sleepSeconds(int sec) {
  PttOFF; RadioOFF; SerialUSB.flush();
  for (int i = 0; i < sec; i++) delay(1000);
}

byte configDra818(char *freq) {
  RadioON; char ack[3]; delay(2000);
  char cmd[50];
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial1.println(cmd);
  ack[2] = 0;
  while (ack[2] != 0x0A) {
    if (Serial1.available() > 0) { ack[0] = ack[1]; ack[1] = ack[2]; ack[2] = Serial1.read(); }
  }
  delay(2000); RadioOFF;
  return (ack[0] == 0x30);
}

// =================== POSITION / TELEMETRY ===================
// Converts GPS + sensor data into readable formats for radio + SD logs.
void updatePosition() {
  char latStr[10], lonStr[10];
  double d_lat = myGPS.getLatitude() / 10000000.f;
  double d_lon = myGPS.getLongitude() / 10000000.f;
  int temp = 0;
  double dm_lat = (modf(fabs(d_lat), &d_lat)) * 60.0 + (fabs(d_lat) * 100);
  double dm_lon = (modf(fabs(d_lon), &d_lon)) * 60.0 + (fabs(d_lon) * 100);

  dtostrf(dm_lat, 7, 2, latStr);
  dtostrf(dm_lon, 8, 2, lonStr);
  latStr[7] = (myGPS.getLatitude() >= 0.0) ? 'N' : 'S';
  lonStr[8] = (myGPS.getLongitude() >= 0.0) ? 'E' : 'W';

  APRS_setLat(latStr);
  APRS_setLon(lonStr);
  APRS_setTimeStamp(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());
}

void updateTelemetry() {
  float internalT = bmp_ok ? bmp.readTemperature() : NAN;
  float bmpP = bmp_ok ? bmp.readPressure() / 100.0 : NAN;
  float battV = readBatt();
  float extT = ds_ok ? ds18b20.getTempCByIndex(0) : NAN;

  float alt_m = myGPS.getAltitude() / 1000.0f;
  float alt_ft = alt_m * 3.28084f;

  // Build the message text for APRS
  snprintf(telemetry_buff, sizeof(telemetry_buff),
    "%03dTxC %.2fC %.2fhPa %.2fV %.0fft METEOR Flight Computer",
    TxCount, internalT, bmpP, battV, alt_ft);

  // --- SD Logging ---
  if (sd_ok) {
    char ts[24]; bool usedGps = false;
    getTimestamp(ts, sizeof(ts), usedGps);
    char line[300];
    snprintf(line, sizeof(line),
      "%s,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f",
      ts, myGPS.getLatitude() / 10000000.f, myGPS.getLongitude() / 10000000.f,
      alt_m, bmpP, battV, internalT, extT);
    logToSD(line);
  }
}

void sendLocation() {
  if (readBatt() > DraHighVolt) RfHiPwr; else RfLowPwr;
  RadioON; delay(2000); PttON; delay(1000);
  APRS_sendLoc(telemetry_buff);
  delay(10); PttOFF; RadioOFF;
  TxCount++;
}

void sendStatus() {
  if (readBatt() > DraHighVolt) RfHiPwr; else RfLowPwr;
  RadioON; delay(2000); PttON; delay(1000);
  APRS_sendStatus(StatusMessage);
  delay(10); PttOFF; RadioOFF;
  TxCount++;
}

// =================== SUPPORT FUNCTIONS ===================
void gpsDebug() {
  SerialUSB.print(F("FixType:")); SerialUSB.print(myGPS.getFixType());
  SerialUSB.print(F(" Sats:")); SerialUSB.print(myGPS.getSIV());
  SerialUSB.print(F(" Lat:")); SerialUSB.print(myGPS.getLatitude() / 10000000.f, 7);
  SerialUSB.print(F(" Lon:")); SerialUSB.print(myGPS.getLongitude() / 10000000.f, 7);
  SerialUSB.print(F(" Alt:")); SerialUSB.print(myGPS.getAltitude() / 1000.0, 1);
  if (bmp_ok) {
    SerialUSB.print(F(" BMP T: ")); SerialUSB.print(bmp.readTemperature());
    SerialUSB.print(F(" P: ")); SerialUSB.print(bmp.readPressure() / 100.0);
  }
  SerialUSB.println();
}

void setupUBloxDynamicModel() {
  if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g))
    ublox_high_alt_mode_enabled = true;
}

float readBatt() {
  const float R1 = 560000.0f, R2 = 100000.0f;
  float value  = analogRead(BattPin);
  value  = (value * 1.65f) / 1024.0f;
  return value / (R2 / (R1 + R2));
}

float readAnalogTempC() {
  const int samples = 8;
  float sum = 0;
  for (int i = 0; i < samples; i++) { sum += analogRead(TEMP_A3_PIN); delay(2); }
  float avg = sum / samples;
  float voltage = (avg * 1.65f) / 1023.0f;
  return (voltage - 0.5f) * 100.0f;
}

// =================== SD LOGGING ===================
void ensureCSVHeader() {
  if (!sd_ok) return;

  // Create new log file named by RTC timestamp
  char filename[40];
  if (rtc_ok) {
    DateTime now = rtc.now();
    snprintf(filename, sizeof(filename),
      "FLIGHT_%04d-%02d-%02d_%02d-%02d-%02d.csv",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second());
  } else {
    snprintf(filename, sizeof(filename), "flightlog_unknown.csv");
  }
  strcpy(currentLogFile, filename);

  if (SD.exists(currentLogFile)) SD.remove(currentLogFile);

  File f = SD.open(currentLogFile, FILE_WRITE);
  if (f) {
    f.println("timestamp,latitude,longitude,altitude_m,pressure_hPa,voltage_V,internal_temp_C,external_temp_C");
    f.flush();
    f.close();
  }
}

void logToSD(const char* line) {
  if (!sd_ok) return;
  File f = SD.open(currentLogFile, FILE_WRITE);
  if (f) {
    f.println(line);
    f.flush();
    f.close();
  }
}

bool getTimestamp(char* out, size_t len, bool &usedGps) {
  if (rtc_ok) {
    DateTime now = rtc.now();
    snprintf(out, len, "%04d-%02d-%02d %02d:%02d:%02d",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second());
    usedGps = false;
    return true;
  } else {
    snprintf(out, len, "0000-00-00 00:00:00");
    usedGps = false;
    return false;
  }
}

void freeMem() {
  SerialUSB.print(F("Free RAM: "));
  SerialUSB.print(freeMemory());
  SerialUSB.println(F(" bytes"));
}
