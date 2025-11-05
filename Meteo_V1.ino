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
#include <Arduino.h>          // Core Arduino functions
#include <Wire.h>             // I2C communication
#include <math.h>             // Math utilities
#include <stdio.h>            // C-style string formatting
#include <avr/dtostrf.h>      // Float-to-string conversion for telemetry
#include <MemoryFree.h>       // For free-RAM reporting (debugging)


// ===== SENSOR & HARDWARE LIBRARIES =====
#include <ZeroAPRS.h>                       // APRS / DRA818V radio driver
#include <SparkFun_Ublox_Arduino_Library.h> // GPS (u-blox)
#include <Adafruit_BMP085.h>                // Pressure/Temperature sensor
#include <Adafruit_LIS3MDL.h>               // 3-axis Magnetometer
#include <Adafruit_Sensor.h>                // Common Adafruit sensor interface
#include <Adafruit_LSM6DS3.h>               // 6-axis IMU (Accel + Gyro)
#include <RTClib.h>                         // DS3231 Real-Time Clock
#include <OneWire.h>                        // 1-Wire bus for DS18B20
#include <DallasTemperature.h>              // DS18B20 temperature sensor
#include <SPI.h>                            // SPI bus for SD and radio
#include <SdFat.h>                          // Fast SD-card library


// ===== SD-CARD OBJECTS =====
SdFat SD;                 // Main SD-card interface
File logFile;             // Active log file handle


// ===== HARDWARE PIN DEFINITIONS =====
// Note: Some pin names differ from silkscreen labels on your PCB.
// These constants make the code portable and clear.
#define SD_CS         A2     // SD card chip-select pin
#define TEMP_A3_PIN   A1     // Analog temperature sensor (TMP36 input)
#define BattPin       A5     // Battery voltage divider input

#define GpsPwr        7      // GPS power control (active LOW)
#define PttPin        3      // Push-to-talk control for radio
#define PwDwPin       A3     // RF power-down pin
#define PowerHL       A4     // RF high/low-power select

// ===== DIGITAL CONTROL MACROS =====
// Simplify repetitive digitalWrite operations.
#define GpsON       digitalWrite(GpsPwr, LOW)     // Turn GPS power ON
#define GpsOFF      digitalWrite(GpsPwr, HIGH)    // Turn GPS power OFF
#define PttON       digitalWrite(PttPin, HIGH)    // Transmit ON
#define PttOFF      digitalWrite(PttPin, LOW)     // Transmit OFF
#define RadioON     digitalWrite(PwDwPin, HIGH)   // Enable radio
#define RadioOFF    digitalWrite(PwDwPin, LOW)    // Disable radio
#define RfHiPwr     digitalWrite(PowerHL, HIGH)   // High-power RF mode
#define RfLowPwr    digitalWrite(PowerHL, LOW)    // Low-power RF mode

// #define DEVMODE  // Uncomment to enable extra debug serial prints


// ===== APRS / GLOBAL CONFIGURATION =====
// Callsign, frequency, and beacon settings for your APRS transmitter.
char    CallSign[7] = "K5FPS";      // Your amateur callsign
int8_t  CallNumber  = 11;           // SSID / device number
char    Symbol      = '/O';         // APRS map icon (balloon symbol)
bool    alternateSymbolTable = false;
char    Frequency[9] = "144.3900";  // Standard APRS frequency (MHz)

char    comment[50]        = "METEOR APRS v1.1 (Adafruit IMU)";
char    StatusMessage[50]  = "METEOR Flight Computer online";

// Timing and power thresholds
uint16_t  BeaconWait   = 10;     // Seconds between telemetry packets
uint16_t  BattWait     = 60;     // Delay when battery low
float     BattMin      = 2.8;    // Minimum voltage before sleeping
float     DraHighVolt  = 5.0;    // Threshold for high RF power

// APRS path configuration (for digipeater hops)
uint8_t   Wide1 = 1;
uint8_t   Wide2 = 1;
uint8_t   pathSize = 2;
bool      autoPathSizeHighAlt = true; // Reduce path at high altitudes

// Runtime state variables
bool      aliveStatus = true;
bool      radioSetup  = false;
static char telemetry_buff[120];  // Buffer for APRS text packets
uint16_t  TxCount = 1;            // Incremented after each transmission


// ===== GPS CONFIGURATION =====
int16_t   GpsResetTime = 1800;    // Reset GPS if no fix after N seconds
bool      ublox_high_alt_mode_enabled = false;
int16_t   GpsInvalidTime = 0;     // Counts seconds of invalid fix
bool      gpsSetup = false;       // Tracks if GPS has initialized


// ===== SENSOR OBJECTS =====
// Instantiate all sensor interfaces and drivers.
SFE_UBLOX_GPS myGPS;          // GPS (u-blox)
Adafruit_BMP085 bmp;          // Barometer
Adafruit_LIS3MDL lis3mdl;     // Magnetometer
RTC_DS3231 rtc;               // Real-Time Clock
Adafruit_LSM6DS3 imu;         // IMU (accelerometer + gyroscope)


// ===== EXTERNAL TEMPERATURE (DS18B20) SETUP =====
#define DS18B20_PIN  A1       // 1-Wire bus pin (shared with TMP36 analog line)
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);


// ===== SENSOR STATUS FLAGS =====
// Used during startup self-test to track which sensors initialize successfully.
bool bmp_ok = false, mag_ok = false, rtc_ok = false,
     imu_ok = false, ds_ok = false;


// ===== SD-CARD GLOBALS =====
bool sd_ok = false;                     // Flag for SD initialization
char currentLogFile[40] = "flightlog.csv";  // Default file name placeholder


// ===== DEBUG TIMERS =====
// Used for periodic debug output in DEVMODE.
static uint32_t lastDevPrint  = 0;
static uint32_t lastMagPrint  = 0;


// ===== FUNCTION DECLARATIONS =====
// Forward declarations let us define functions later in the file
// while still allowing calls earlier (e.g., from setup()).
void gpsStart();
void sleepSeconds(int sec);
byte configDra818(char *freq);
void updatePosition();
void updateTelemetry();
void sendLocation();
void sendStatus();
void gpsDebug();
void setupUBloxDynamicModel();
float readBatt();
float readAnalogTempC();
void freeMem();
void logToSD(const char* line);
void ensureCSVHeader();
void logBootBanner(bool gpsTime);
bool getTimestamp(char* out, size_t len, bool &usedGps);
// ============================================================================
//  SYSTEM SETUP
//  Initializes hardware, sensors, communication buses, and SD logging
// ============================================================================

void setup() {

  // --- Begin: Safe Boot and Power Stabilization ---
  // The system boots into a safe, low-power state with outputs off.
  // Using the 1.65 V internal ADC reference for consistent analog readings.
  analogReference(AR_INTERNAL1V65);

  // Configure I/O pins for each subsystem
  pinMode(PttPin, OUTPUT);     // Radio push-to-talk
  pinMode(GpsPwr, OUTPUT);     // GPS power control
  pinMode(BattPin, INPUT);     // Battery voltage monitor
  pinMode(PwDwPin, OUTPUT);    // RF module power-down
  pinMode(PowerHL, OUTPUT);    // RF power-level select
  pinMode(SD_CS, OUTPUT);      // SD card chip select
  digitalWrite(SD_CS, HIGH);   // Keep SD deselected until ready
  pinMode(TEMP_A3_PIN, INPUT); // Analog temperature input
  analogReadResolution(10);    // 10-bit ADC resolution

  // Make sure all external devices start off
  GpsOFF;
  PttOFF;
  RadioOFF;
  RfLowPwr;

  // Give USB & 3.3 V rails a moment to stabilize
  delay(3000);

  // --- Enable shared I²C rail powering GPS and BMP sensors ---
  GpsON;
  delay(1500);   // Allow peripherals to boot up
  Wire.begin();  // Start I²C bus
  SPI.begin();   // Start SPI bus

  // --- Try initializing the onboard barometer twice ---
  bmp_ok = bmp.begin();
  if (!bmp_ok) {
    SerialUSB.println(F("BMP not ready, retrying..."));
    delay(500);
    bmp_ok = bmp.begin();
  }
  SerialUSB.println(bmp_ok ? F("BMP OK (LightAPRS onboard)") : F("BMP FAIL"));

  // --- I²C bus recovery ---
  // Sometimes SDA/SCL can be stuck LOW if a device hangs during startup.
  // Pulsing SCL 9 times releases any stuck slaves.
  for (int i = 0; i < 9; i++) {
    pinMode(SCL, OUTPUT);
    digitalWrite(SCL, HIGH); delay(2);
    digitalWrite(SCL, LOW);  delay(2);
  }

  // Re-enable I²C pull-ups after recovery
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  Wire.begin();
  SPI.begin();  // Re-start SPI just in case

  // --- Serial (USB) setup for debug output ---
  SerialUSB.begin(115200);
  delay(1500);
  SerialUSB.println(F("\n=== METEOR Boot Sequence ==="));
  SerialUSB.println(F("Power & Bus stabilized; initializing subsystems..."));

  // ======================================================================
  //  RADIO & APRS INITIALIZATION
  // ======================================================================
  Serial1.begin(9600);     // DRA818V UART interface
  APRS_init();             // Initialize APRS protocol stack
  APRS_setCallsign(CallSign, CallNumber);
  APRS_setDestination("APLIGA", 0);   // Default destination
  APRS_setPath1("WIDE1", Wide1);      // Digipeater path config
  APRS_setPath2("WIDE2", Wide2);
  APRS_setPathSize(pathSize);
  APRS_useAlternateSymbolTable(alternateSymbolTable);
  APRS_setSymbol(Symbol);
  APRS_setGain(2);         // Modulation gain level

  delay(2000);
  SerialUSB.println(F("[RADIO] Initializing DRA818V..."));
  configDra818(Frequency); // Send AT command to configure RF frequency

  // ======================================================================
  //  SENSOR INITIALIZATION & SELF-TEST
  // ======================================================================

  // --- Barometer (BMP085) ---
  bmp_ok = bmp.begin();
  SerialUSB.println(bmp_ok ? F("BMP085 OK") : F("BMP085 FAIL"));

  // --- Magnetometer (LIS3MDL) ---
  if (lis3mdl.begin_I2C()) {
    mag_ok = true;
    SerialUSB.println(F("LIS3MDL OK"));
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  } else {
    SerialUSB.println(F("LIS3MDL FAIL"));
  }

  // --- Real-Time Clock (DS3231) ---
  rtc_ok = rtc.begin();
  if (rtc_ok) {
    SerialUSB.println(F("DS3231 OK"));
    if (rtc.lostPower()) {
      // If RTC lost power, set to compile time as baseline
      SerialUSB.println(F("RTC lost power, setting to compile time"));
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  } else {
    SerialUSB.println(F("DS3231 FAIL"));
  }

  // --- Inertial Measurement Unit (LSM6DS3) ---
  // Some boards use address 0x6A, others 0x6B; try both.
  if (imu.begin_I2C(0x6A) || imu.begin_I2C(0x6B)) {
    imu_ok = true;
    SerialUSB.println(F("LSM6DS3 OK"));
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
  } else {
    imu_ok = false;
    SerialUSB.println(F("LSM6DS3 FAIL"));
  }

  // --- External Temperature (DS18B20) ---
  ds18b20.begin();
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  if (t > -100 && t < 150) {   // crude sanity check for valid reading
    ds_ok = true;
    SerialUSB.println(F("DS18B20 OK"));
  } else {
    SerialUSB.println(F("DS18B20 not detected"));
  }

  // ======================================================================
  //  SD CARD INITIALIZATION
  // ======================================================================
  SerialUSB.println(F("[SD] Initializing SD card..."));
  delay(1000);
  sd_ok = SD.begin(SD_CS);
  if (!sd_ok) {
    SerialUSB.println(F("[SD] First init failed, retrying..."));
    delay(500);
    sd_ok = SD.begin(SD_CS);
  }

  // Create a new log file and write CSV header if successful
  if (sd_ok) {
    SerialUSB.println(F("SD: OK"));
    ensureCSVHeader();
    SerialUSB.println(F("[SD] ensureCSVHeader() executed"));
  } else {
    SerialUSB.println(F("SD: No card or init failed"));
  }

  // ======================================================================
  //  FINAL STATUS REPORT
  // ======================================================================
  SerialUSB.print(F("APRS CallSign: "));
  SerialUSB.print(CallSign);
  SerialUSB.print(F("-"));
  SerialUSB.println(CallNumber);

  SerialUSB.println(F("=== Setup Complete ==="));
}
// ============================================================================
//  MAIN LOOP
//  Handles continuous flight operation: telemetry, logging, and APRS beacons
// ============================================================================

void loop() {

  // --- Power safety check ---
  // If the battery voltage is too low, the system enters a low-power wait mode.
  if (readBatt() > BattMin) {

    // ------------------------------------------------------------------------
    // PERIODIC STATUS BROADCAST (Every 10 seconds)
    // ------------------------------------------------------------------------
    static uint32_t lastStatus = 0;
    uint32_t now = millis();
    if (now - lastStatus >= 10000UL) {   // 10-second interval
      sendStatus();                      // Send short APRS "status" packet
      lastStatus = now;
    }

    // ------------------------------------------------------------------------
    // GPS HANDLING
    // ------------------------------------------------------------------------
    // Ensure GPS is started and configured for high-altitude flight.
    if (!gpsSetup) { gpsStart(); }
    if (!ublox_high_alt_mode_enabled) { setupUBloxDynamicModel(); }

    // --- If GPS has a valid Position-Velocity-Time (PVT) message available ---
    if (myGPS.getPVT()) {
      gpsDebug(); // Print debug info (satellites, fix type, etc.)

      // --- Check for a valid GPS fix (>= 3 satellites) ---
      if ((myGPS.getFixType() != 0) && (myGPS.getSIV() > 3)) {
        GpsInvalidTime = 0;

        // Update position and telemetry data
        updatePosition();    // Parse and encode latitude/longitude
        updateTelemetry();   // Collect sensor data and log it

        // --- Adaptive APRS path ---
        // Reduce digipeater path size at very high altitudes (> 3000 ft).
        if (autoPathSizeHighAlt && ((myGPS.getAltitude() * 3.2808399) / 1000.f) > 3000) {
          APRS_setPathSize(1);  // Use single hop (WIDE2 only)
        } else {
          APRS_setPathSize(pathSize);  // Default ground path
        }

        // --- Log boot event only once per power cycle ---
        static bool bootLogged = false;
        if (!bootLogged) { logBootBanner(true); bootLogged = true; }

        // Transmit and log telemetry
        sendStatus();       // APRS status beacon
        sendLocation();     // APRS position report
        freeMem();          // Debug: report free RAM
        SerialUSB.flush();  // Ensure all serial prints are sent
        sleepSeconds(BeaconWait);  // Wait until next beacon interval

      } else {
        // --------------------------------------------------------------------
        // GPS FIX LOST OR INVALID
        // --------------------------------------------------------------------
        GpsInvalidTime++;

        // If GPS has been invalid too long, restart it
        if (GpsInvalidTime > GpsResetTime) {
          GpsOFF;
          ublox_high_alt_mode_enabled = false;
          delay(1000);
          GpsON;
          GpsInvalidTime = 0;
        }

        // Log and store data even without a valid fix (uses RTC time)
        updateTelemetry();
        sleepSeconds(2);
      }

    } else {
      // ----------------------------------------------------------------------
      // GPS NOT READY (no PVT data yet)
      // ----------------------------------------------------------------------
      SerialUSB.println(F("GPS PVT not ready"));
      updateTelemetry(); // Log anyway
      sleepSeconds(2);
    }

    // ------------------------------------------------------------------------
    // OPTIONAL DEVELOPMENT DEBUG OUTPUT
    // (Executed only if DEVMODE is enabled)
    // ------------------------------------------------------------------------
    static uint32_t lastDevPrint = 0, lastMagPrint = 0;
    if (now - lastDevPrint > 1000) {  // Every second
      lastDevPrint = now;

      // --- External temperature sensor (DS18B20) ---
      if (ds_ok) {
        ds18b20.requestTemperatures();
        float tc = ds18b20.getTempCByIndex(0);
        SerialUSB.print(F("DS18B20: "));
        SerialUSB.print(tc, 2);
        SerialUSB.println(F(" C"));
      }

      // --- Analog TMP36 sensor (backup internal temperature) ---
      float a3C = readAnalogTempC();
      SerialUSB.print(F("A1 Analog Temp (TMP36): "));
      SerialUSB.print(a3C, 1);
      SerialUSB.println(F(" C"));

      // --- IMU accelerometer output ---
      if (imu_ok) {
        sensors_event_t accel, gyro, temp;
        imu.getEvent(&accel, &gyro, &temp);
        SerialUSB.print(F("IMU Acc[g] "));
        SerialUSB.print(accel.acceleration.x / 9.80665f, 2); SerialUSB.print(" ");
        SerialUSB.print(accel.acceleration.y / 9.80665f, 2); SerialUSB.print(" ");
        SerialUSB.println(accel.acceleration.z / 9.80665f, 2);
      }

      // --- Magnetometer vector output ---
      if (mag_ok && (now - lastMagPrint > 2000)) {
        lastMagPrint = now;
        sensors_event_t ev;
        lis3mdl.getEvent(&ev);
        SerialUSB.print(F("MAG uT "));
        SerialUSB.print(ev.magnetic.x, 1); SerialUSB.print(" ");
        SerialUSB.print(ev.magnetic.y, 1); SerialUSB.print(" ");
        SerialUSB.println(ev.magnetic.z, 1);
      }
    }

  } else {
    // ------------------------------------------------------------------------
    // LOW BATTERY MODE
    // ------------------------------------------------------------------------
    // If below BattMin voltage, sleep for a longer period to conserve energy.
    sleepSeconds(BattWait);
  }
}
// ============================================================================
//  GPS AND RADIO HELPER FUNCTIONS
//  Handle GPS startup, radio configuration, and APRS packet management
// ============================================================================


// -----------------------------------------------------------------------------
// gpsStart()
// Initializes and powers up the u-blox GPS module. Keeps trying until success.
// -----------------------------------------------------------------------------
void gpsStart() {
  bool gpsBegin = false;

  while (!gpsBegin) {
    GpsON;               // Power up GPS (active LOW)
    delay(1000);
    Wire.begin();        // Start I²C communication
    gpsBegin = myGPS.begin();  // Attempt to connect to u-blox device

    if (gpsBegin) break; // Exit loop if successful

    SerialUSB.println(F("Ublox GPS not detected. Retrying..."));
    delay(2000);
  }

  // Disable UART outputs; use I²C-only communication to avoid interference
  myGPS.setUART1Output(0);
  myGPS.setUART2Output(0);
  myGPS.setI2COutput(COM_TYPE_UBX);
  myGPS.saveConfiguration();

  gpsSetup = true;
}


// -----------------------------------------------------------------------------
// sleepSeconds()
// Simple blocking delay function used for pacing transmissions or power saving.
// Turns off transmit pins and radio during wait.
// -----------------------------------------------------------------------------
void sleepSeconds(int sec) {
  PttOFF;           // Disable push-to-talk (TX)
  RadioOFF;         // Power down radio
  SerialUSB.flush(); // Ensure all data sent before sleeping
  for (int i = 0; i < sec; i++) delay(1000); // Sleep for 'sec' seconds
}


// -----------------------------------------------------------------------------
// configDra818()
// Sends AT command to configure the DRA818V radio module with the APRS frequency.
// -----------------------------------------------------------------------------
byte configDra818(char *freq) {
  RadioON;                // Enable radio before sending config
  char ack[3];
  delay(2000);

  char cmd[50];
  // Example: AT+DMOSETGROUP=0,144.3900,144.3900,0000,4,0000
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial1.println(cmd);
  SerialUSB.println("RF Config");

  // Wait for newline in radio's acknowledgment
  ack[2] = 0;
  while (ack[2] != 0x0A) {
    if (Serial1.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial1.read();
    }
  }

  delay(2000);
  RadioOFF; // Turn off radio after config command

  // Validate response: ASCII '0' (0x30) indicates success
  if (ack[0] == 0x30) {
    SerialUSB.print(F("Frequency updated: "));
    SerialUSB.print(freq);
    SerialUSB.println(F("MHz"));
  } else {
    SerialUSB.println(F("Frequency update error!!!"));
  }

  return (ack[0] == 0x30) ? 1 : 0;
}


// ============================================================================
//  POSITION AND TELEMETRY FORMATTING
//  Converts GPS data into APRS-friendly formats and builds telemetry strings
// ============================================================================


// -----------------------------------------------------------------------------
// updatePosition()
// Converts GPS latitude and longitude from decimal degrees into APRS "ddmm.mmN"
// and "dddmm.mmE" format. Also updates APRS timestamp.
// -----------------------------------------------------------------------------
void updatePosition() {
  // --- Convert latitude ---
  char latStr[10];
  int temp = 0;
  double d_lat = myGPS.getLatitude() / 10000000.f;  // degrees
  double dm_lat = 0.0;

  // Separate degrees from fractional part and convert to ddmm.mm
  if (d_lat < 0.0) {
    temp = -(int)d_lat;
    dm_lat = temp * 100.0 - (d_lat + temp) * 60.0;
  } else {
    temp = (int)d_lat;
    dm_lat = temp * 100 + (d_lat - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, latStr);
  if (dm_lat < 1000) latStr[0] = '0';
  latStr[7] = (d_lat >= 0.0) ? 'N' : 'S';
  APRS_setLat(latStr);

  // --- Convert longitude ---
  char lonStr[10];
  double d_lon = myGPS.getLongitude() / 10000000.f;
  double dm_lon = 0.0;

  if (d_lon < 0.0) {
    temp = -(int)d_lon;
    dm_lon = temp * 100.0 - (d_lon + temp) * 60.0;
  } else {
    temp = (int)d_lon;
    dm_lon = temp * 100 + (d_lon - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, lonStr);
  if (dm_lon < 10000) lonStr[0] = '0';
  if (dm_lon < 1000)  lonStr[1] = '0';
  lonStr[8] = (d_lon >= 0.0) ? 'E' : 'W';

  APRS_setLon(lonStr);
  APRS_setTimeStamp(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());
}


// -----------------------------------------------------------------------------
// updateTelemetry()
// Gathers all sensor data, constructs APRS comment string, and logs CSV line.
// -----------------------------------------------------------------------------
void updateTelemetry() {

  // --- Core environmental data ---
  float internalT = bmp_ok ? bmp.readTemperature() : NAN;
  float bmpP = bmp_ok ? bmp.readPressure() / 100.0 : NAN; // hPa
  float battV = readBatt();

  // --- Satellite info ---
  int sats = myGPS.getSIV();

  // --- External analog temp sensor (TMP36) ---
  float externalT = readAnalogTempC();

  // --- DS18B20 external temperature (if connected) ---
  float extT = NAN;
  if (ds_ok) {
    ds18b20.requestTemperatures();
    extT = ds18b20.getTempCByIndex(0);
  }

  // --- Magnetometer readings ---
  sensors_event_t magEv;
  float mx = NAN, myv = NAN, mz = NAN;
  if (mag_ok) {
    lis3mdl.getEvent(&magEv);
    mx = magEv.magnetic.x;
    myv = magEv.magnetic.y;
    mz = magEv.magnetic.z;
  }

  // --- GPS position, altitude, course, speed ---
  float flat = myGPS.getLatitude() / 10000000.f;
  float flon = myGPS.getLongitude() / 10000000.f;
  float alt_m = myGPS.getAltitude() / 1000.0f;
  float crs = myGPS.getHeading() / 100000.0f;
  float spd_mph = myGPS.getGroundSpeed() * 0.00223694f;

  // --- Convert altitude to feet for APRS comment ---
  float alt_ft = alt_m * 3.28084f;

  // --- Mark BMP sensor out of range above 9000 m ---
  if (alt_m >= 9000.0f) {
    bmpP = NAN;
    SerialUSB.println(F("[BMP] Sensor out of range above 9000 m"));
  }

  // --- Format APRS comment string (transmitted text) ---
  if (alt_m >= 9000.0f)
    snprintf(telemetry_buff, sizeof(telemetry_buff),
             "%03dTxC %.2fC SENSOR_OUT_OF_RANGE %.2fV %.0fft METEOR Flight Computer",
             TxCount, internalT, battV, alt_ft);
  else
    snprintf(telemetry_buff, sizeof(telemetry_buff),
             "%03dTxC %.2fC %.2fhPa %.2fV %.0fft METEOR Flight Computer",
             TxCount, internalT, bmpP, battV, alt_ft);

  SerialUSB.println(telemetry_buff);

  // --- CSV LOGGING SECTION ---
  if (sd_ok) {
    char ts[24];
    bool usedGps = false;
    getTimestamp(ts, sizeof(ts), usedGps);

    char line[400];

    // If barometer out of range, write "SENSOR_OUT_OF_RANGE" text instead of pressure value
    if (alt_m >= 9000.0f) {
      snprintf(line, sizeof(line),
        "%s,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,SENSOR_OUT_OF_RANGE,%.2f,%d,%.2f,%.2f,%.2f,%.2f",
        ts,
        flat, flon, alt_m, crs, spd_mph,
        internalT, battV, sats,
        mx, myv, mz, externalT
      );
    } else {
      snprintf(line, sizeof(line),
        "%s,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f",
        ts,
        flat, flon, alt_m, crs, spd_mph,
        internalT, bmpP, battV, sats,
        mx, myv, mz, externalT
      );
    }

    logToSD(line); // Append telemetry line to the log file
  }
}
// ============================================================================
//  RADIO TRANSMISSION & DEBUG UTILITIES
//  Handles APRS position/status beacons, GPS debug output, and sensor reads
// ============================================================================


// -----------------------------------------------------------------------------
// sendLocation()
// Transmits a full APRS position/telemetry packet via the DRA818V radio.
// Handles push-to-talk (PTT) sequencing and RF power control.
// -----------------------------------------------------------------------------
void sendLocation() {
  SerialUSB.println(F("Location sending with comment..."));

  // Choose RF power mode depending on battery voltage
  if (readBatt() > DraHighVolt) RfHiPwr;
  else RfLowPwr;

  // Turn on radio and key transmitter
  RadioON; delay(2000);
  PttON;   delay(1000);

  // Send APRS packet containing position and comment buffer
  APRS_sendLoc(telemetry_buff);

  // Release PTT and power down radio
  delay(10);
  PttOFF;
  RadioOFF;
  delay(1000);

  SerialUSB.print(F("Location sent - "));
  SerialUSB.println(TxCount);
  TxCount++;     // Increment transmission counter
}


// -----------------------------------------------------------------------------
// sendStatus()
// Sends a short APRS status packet (non-position message).
// -----------------------------------------------------------------------------
void sendStatus() {
  SerialUSB.println(F("Status sending..."));

  if (readBatt() > DraHighVolt) RfHiPwr;
  else RfLowPwr;

  RadioON; delay(2000);
  PttON;   delay(1000);

  APRS_sendStatus(StatusMessage);  // Transmit predefined status string

  delay(10);
  PttOFF;
  RadioOFF;
  delay(1000);

  SerialUSB.print(F("Status sent - "));
  SerialUSB.println(TxCount);
  TxCount++;
}


// -----------------------------------------------------------------------------
// gpsDebug()
// Prints current GPS data (fix type, satellites, position, speed, etc.)
// Useful for development via USB serial output.
// -----------------------------------------------------------------------------
void gpsDebug() {
  byte fixType = myGPS.getFixType();
  SerialUSB.print(F("FixType:"));
  SerialUSB.print(fixType);

  int SIV = myGPS.getSIV();   // Satellites in view
  SerialUSB.print(F(" Sats:"));
  SerialUSB.print(SIV);

  // Latitude / Longitude in decimal degrees
  float flat = myGPS.getLatitude() / 10000000.f;
  float flon = myGPS.getLongitude() / 10000000.f;
  SerialUSB.print(F(" Lat:"));
  SerialUSB.print(flat, 7);
  SerialUSB.print(F(" Lon:"));
  SerialUSB.print(flon, 7);

  // Altitude (m) and Ground Speed (knots)
  float altitude = myGPS.getAltitude() / 1000.0;
  float speed = myGPS.getGroundSpeed();
  SerialUSB.print(F(" Alt(m):"));
  SerialUSB.print(altitude, 1);
  SerialUSB.print(F(" Spd(kn):"));
  SerialUSB.print(speed * 0.00194384f, 2);

  // Timestamp (UTC)
  SerialUSB.print(" Time: ");
  SerialUSB.print(myGPS.getYear());   SerialUSB.print("-");
  SerialUSB.print(myGPS.getMonth());  SerialUSB.print("-");
  SerialUSB.print(myGPS.getDay());    SerialUSB.print(" ");
  SerialUSB.print(myGPS.getHour());   SerialUSB.print(":");
  SerialUSB.print(myGPS.getMinute()); SerialUSB.print(":");
  SerialUSB.print(myGPS.getSecond());

  // Optional: print barometric temperature/pressure
  if (bmp_ok) {
    SerialUSB.print(" BMP T: ");
    SerialUSB.print(bmp.readTemperature());
    SerialUSB.print(" C  P: ");
    SerialUSB.print(bmp.readPressure() / 100.0);
    SerialUSB.print(" hPa");
  }

  SerialUSB.println();
}


// -----------------------------------------------------------------------------
// setupUBloxDynamicModel()
// Configures the u-blox GPS for high-altitude (balloon) operation.
// The "Airborne <4g" mode supports up to ~50 km altitude and 100 m/s velocity.
// -----------------------------------------------------------------------------
void setupUBloxDynamicModel() {
  if (!myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g)) {
    SerialUSB.println(F("*** Warning: setDynamicModel failed ***"));
  } else {
    ublox_high_alt_mode_enabled = true;
    SerialUSB.print(F("u-blox dynamic model set: "));
    SerialUSB.println(myGPS.getDynamicModel());
  }
}


// ============================================================================
//  SENSOR & ANALOG UTILITY FUNCTIONS
// ============================================================================


// -----------------------------------------------------------------------------
// readBatt()
// Measures battery voltage through a resistor divider.
// Divider: R1 = 560 kΩ (top) and R2 = 100 kΩ (bottom).
// Converts 10-bit ADC reading (1.65 V ref) to actual voltage.
// -----------------------------------------------------------------------------
float readBatt() {
  const float R1 = 560000.0f, R2 = 100000.0f;
  float value = analogRead(BattPin);

  // Convert ADC reading to volts (1.65 V internal ref)
  value = (value * 1.65f) / 1024.0f;

  // Compensate for voltage divider ratio
  value = value / (R2 / (R1 + R2));

  return value;
}


// -----------------------------------------------------------------------------
// readAnalogTempC()
// Reads analog TMP36 temperature sensor (in °C) on TEMP_A3_PIN.
// Uses 1.65 V ADC reference. Formula: (Vout − 0.5) × 100 = °C
// -----------------------------------------------------------------------------
float readAnalogTempC() {
  const int samples = 8;
  float sum = 0;

  // Average multiple samples for noise reduction
  for (int i = 0; i < samples; i++) {
    sum += analogRead(TEMP_A3_PIN);
    delay(2);
  }

  float avg = sum / samples;
  float voltage = (avg * 1.65f) / 1023.0f;
  return (voltage - 0.5f) * 100.0f;
}
// ============================================================================
//  SD CARD LOGGING & MEMORY UTILITIES
//  Handles flight log creation, timestamping, and memory diagnostics
// ============================================================================


// -----------------------------------------------------------------------------
// ensureCSVHeader()
// Creates a new CSV log file with a unique timestamped filename and header row.
// Called once during setup() after SD initialization.
// -----------------------------------------------------------------------------
void ensureCSVHeader() {
  if (!sd_ok) {
    SerialUSB.println(F("[SD] ensureCSVHeader: sd_ok==false"));
    return;
  }

  // --- Generate timestamped filename using RTC ---
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

  // Store filename globally for later writes
  strcpy(currentLogFile, filename);

  SerialUSB.print(F("[SD] Creating new log file: "));
  SerialUSB.println(currentLogFile);

  // --- Ensure a clean file (delete if exists) ---
  if (SD.exists(currentLogFile)) {
    SD.remove(currentLogFile);
  }

  // --- Create and write the header row ---
  File f = SD.open(currentLogFile, FILE_WRITE);
  if (f) {
    f.println(
      "timestamp (YYYY-MM-DD HH:MM:SS),"
      "latitude (decimal_deg),"
      "longitude (decimal_deg),"
      "altitude_meters,"
      "course_degrees,"
      "speed_mph,"
      "internal_temperature_C,"
      "barometer_pressure_hPa,"
      "battery_voltage_V,"
      "gps_satellites_visible,"
      "magnetometer_X_uT,"
      "magnetometer_Y_uT,"
      "magnetometer_Z_uT,"
      "external_temperature_C (TMP36)"
    );
    f.flush();
    f.close();
    SerialUSB.println(F("[SD] New log file created successfully"));
  } else {
    SerialUSB.println(F("[SD] Error creating new log file"));
  }
}


// -----------------------------------------------------------------------------
// logToSD()
// Appends a single line of telemetry data to the current flight log CSV file.
// -----------------------------------------------------------------------------
void logToSD(const char* line) {
  if (!sd_ok) return;

  SerialUSB.print(F("[SD] Writing line: "));
  SerialUSB.println(line);

  File f = SD.open(currentLogFile, FILE_WRITE);
  if (f) {
    f.println(line);
    f.flush();    // Force write to SD to prevent data loss on power failure
    f.close();
    SerialUSB.println(F("[SD] Log write successful"));
  } else {
    SerialUSB.println(F("[SD] Error opening flightlog.csv"));
  }
}


// -----------------------------------------------------------------------------
// logBootBanner()
// Writes a boot-time banner line to the CSV log indicating startup time and
// whether the timestamp source was GPS, RTC, or unknown.
// -----------------------------------------------------------------------------
void logBootBanner(bool gpsTime) {
  if (!sd_ok) return;

  char ts[24];
  bool used = false;
  getTimestamp(ts, sizeof(ts), used);

  File f = SD.open(currentLogFile, FILE_WRITE);
  if (f) {
    f.print("# BOOT ");
    f.print(ts);
    f.print(" (");
    f.print(gpsTime ? "GPS" : (rtc_ok ? "RTC" : "UNK"));
    f.println(")");
    f.flush();
    f.close();
  }
}


// -----------------------------------------------------------------------------
// getTimestamp()
// Returns a formatted timestamp string (YYYY-MM-DD HH:MM:SS).
// Always prefers RTC; GPS time can be integrated later if desired.
// -----------------------------------------------------------------------------
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


// -----------------------------------------------------------------------------
// freeMem()
// Reports remaining free SRAM bytes for diagnostic purposes.
// Uses MemoryFree.h library. Printed over USB serial.
// -----------------------------------------------------------------------------
void freeMem() {
  SerialUSB.print(F("Free RAM: "));
  SerialUSB.print(freeMemory(), DEC);
  SerialUSB.println(F(" byte"));
}
