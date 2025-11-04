METEOR is an APRS-based flight computer that runs off the LightAPRS2.0 from QRP Labs. Since the LightAPRS2.0 is open-source, we’ve added a custom PCB to connect it to a suite of sensors, primarily using I2C connections. The components on this computer are outlined below:

<img width="708" height="573" alt="Screenshot 2025-11-04 at 8 42 12 AM" src="https://github.com/user-attachments/assets/071fa9a1-c0bf-4582-8c1a-6219028fa128" /> 
<img width="708" height="573" alt="Screenshot 2025-11-04 at 8 43 15 AM" src="https://github.com/user-attachments/assets/25dfedbc-22be-4ec9-a0e2-aed7843c7906" />

The LightAPRS2.0 already includes the following sensors:
GPS - The GPS outputs latitude, longitude, and information for altitude and groundspeed calculations. 
BMP180 - This is the on-board pressure, temperature, and altitude sensor. It reads the board temperature (internal computer temperature) and atmospheric pressure directly to the SD card. However, the pressure sensor is limited and can only read down to 300 hPa, which is inadequate for high-altitude flights. Therefore, the code has been modified so that once the computer detects an altitude of 9,000m, it outputs “out of sensor range.” 

Additional components added include:
Sensors:
TMP36 - This is an analog temperature sensor used to measure external temperature. You can see the head of this sensor protruding from the box near the SD card slot. 
Adafruit DS3231 Precision RTC - This is a real-time clock. It outputs accurate time at your location and records directly onto the SD card. 
Adafruit Triple-axis Magnetometer LIS3MDL - This is a 3-axis magnetometer with x, y, and z value readouts to the SD card. 
MS5611 Pressure Sensor - This additional pressure sensor was added because the BMP180 in the LightAPRS2.0 is only functional up to 300 hPa, or the atmospheric pressure at approximately 9,000 meters, which is insufficient for typical high-altitude flights. This pressure sensor is currently not functional; if you can get it to read out anything, you win a prize! If you can get it to read out accurate pressure values, two prizes! 
Adafruit LSM6DS3TR-C 6-DoF Accelerometer + Gyro IMU - This is an accelerometer/ gyroscope that measures acceleration in 3D space, plus spin and twist. This reads out the x, y, and z values onto the SD card.

Functional Components:
Adafruit Micro SD SPI or SDIO Card Reader - This is the SD card reader. You can find it on the front of the computer under the external temperature sensor. During flight, this should be taped over so that the SD card doesn’t fall out.
3.7V 3000mAh LiPo Battery - The entire system is powered by a rechargeable 3.7V LiPo battery that rests under the PCB. 
Printed Circuit Board - A custom PCB houses the connections between the main board, batteries, and sensors. 
Switch and Indicator Light - METEOR has an external on/off switch and a green LED indicator to show when the computer is powered on. 
