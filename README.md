# Explore Labs 3D Accelerometer + 3D Magnetometer LSM303D Compass Breakout Board
## Overview

<img src="http://www.explorelabs.com/media/catalog/product//l/3/l3gd20h-03.jpg" width="400" alt="Explore Labs 3D Gyroscope L3GD20H Breakout Board - Top" />
<img src="http://www.explorelabs.com/media/catalog/product//l/3/l3gd20h-02.jpg" width="400" alt="Explore Labs 3D Gyroscope L3GD20H Breakout Board - Bottom" />

[Explore Labs 3D Gyroscope L3GD20H Breakout Board](http://www.explorelabs.com/explore-labs-3d-gyroscope-l3gd20h-breakout-board-5v-ready-with-voltage-regulator "Explore Labs 3D Gyroscope L3GD20H Breakout Board") The LSM303D is a system-in-package featuring a 3D digital linear acceleration sensor and a 3D digital magnetic sensor that is ideal for making a tilt-compensated compass.

The LSM303D includes an I2C serial bus interface that supports standard and fast mode (100 kHz and 400 kHz) and SPI serial standard interface. The system can be configured to generate an interrupt signal for free-fall, motion detection and magnetic field detection. Thresholds and timing of interrupt generators are programmable by the end user. Magnetic and accelerometer blocks can be enabled or put into power-down mode separately.

The carrier board includes a low-dropout linear voltage regulator that provides the 3.3 V required by the LSM303D, which allows the sensor to be powered from 2.5 V to 5.5 V. The regulator output is available on the 3Vo pin which can supply approx. 150 mA to external devices.

The breakout board also includes a circuit that shifts the two I2C lines or all four SPI lines (clock, data-in/out and chip select) to the same logic voltage level as the supplied VIN, making it simple to interface the board with 5V systems; and the board’s 0.1″ pin spacing makes it easy to use with standard solderless breadboards and 0.1″ perfboards. The board ships fully populated with its SMD components, including the LSM303D, as shown in the product picture.

## Features of Explore Labs 3D Accelerometer + 3D Magnetometer LSM303D Compass Breakout Board:
* Wide supply voltage - 2.5V to 5V
* Level shifted IO lines for easy interfacing to high voltage boards like Arduino Uno, Mega, Leonardo, etc.
* All pins of the LSM303D sensor are available like I2C and SPI communication interfaces as well as INT1 and INT2 Interrupt pins
* Layout takes care of keeping the area below the sensor free from any active current carrying trace or a power or ground plane
* Capacitors with low ESR characteristics are used for decoupling on both VDD and VDDIO power inputs
* Open Source Hardware
* Designed, Assembled and Made in India

## Features of LSM303D Sensor IC:
* 3 magnetic field channels and 3 acceleration channels
* &plusmn;2/&plusmn;4/&plusmn;8/&plusmn;12 gauss magnetic full scale
* &plusmn;2/&plusmn;4/&plusmn;6/&plusmn;8/&plusmn;16 g linear acceleration full scale
* 16-bit data output
* SPI / I2C serial interfaces
* Power-down mode / low-power mode
* Extended operating temperature range (from -40 °C to 85 °C)
* Programmable interrupt generators for free-fall, motion detection and magnetic field detection
* Embedded temperature sensor
* Embedded FIFO

## Specifications:
* Input Voltage: 2.5V to 5V
* 5V Ready IOs: Level shifted IO lines for easy interfacing to incompatible voltage boards
* Digital Output: Inter Integrated Circuit (I2C/IIC) or Serial Peripheral Interface (SPI)
* Regulator Pin - Provides 3.3V Out at 150mA (when 3.6V < VIN < 5V) else provides voltage ~equal to VIN
* Dimensions: 20.4 mm x 20 mm
* Weight: 3 grams

## Applications:
* Tilt-compensated compasses
* Map rotation
* Position detection
* Motion-activated functions
* Free-fall detection
* Click/double-click recognition
* Pedometers
* Intelligent power saving for handheld devices
* Display orientation
* Gaming and virtual reality input devices
* Impact recognition and logging
* Vibration monitoring and compensation

## Output Format:
Values are expressed in 16-bits as two’s complement for each axis. As shown in the example codes below, knowing the sensitivites of the device, one can easily find out the acceleration value in g (acceleration due to gravity) and magnetic field intensity in gauss. These readings may be used separately or together to create a tilt-compensated compass or other complex orientation calculations for Inertial Measurement Units (IMU).

## Pinout Table for Explore Labs 3D Accelerometer + 3D Magnetometer LSM303D Compass Breakout
|Pin|5V Safe?   |Function|
|---|-----------|--------|
|VIN|Yes|**Input Voltage** Pin: 2.5V to 5.0V (depending on Master device being used as mentioned in later connection sections and tables below)|
|3Vo|Not Applicable|This is the output pin of the onboard 3.3V Voltage Regulator. It provides a clean and regulated 3.3V output to the sensor IC. It can also be used by the user to provide power to external devices or sensors. Maximum current that can be sourced easily would be approximately 125mA to 150mA after leaving some room for the on board sensor IC|
|GND|Not Applicable|Ground Connection. Connect this pin to the master device's ground pin or to any externally powered devices from onboard regulator|
|SDI/SDA|Yes. Bidirectional MOSFET with 4K7 pull-ups|This pin has dual functions - **Serial Data Input** (for SPI) or **Serial Data** (for I2C). In case of SPI, it can also be called as Master Out Slave In (MOSI)|
|SCK/SCL|Yes. Bidirectional MOSFET with 4K7 pull-ups|This pin is the **Serial Clock** input pin. It is common for both SPI and I2C interfaces|
|SDO/ADR|Yes. Only a 4K7 pull-up resistor needed as this is an output pin|This pin has dual functions - **Serial Data Output** (for SPI) or **Address Select** (for I2C). Also called as SA0. In case of SPI, it can also be called as Master In Slave Out (MOSI). In case of I2C, this pin may be pulled LOW or grounded for selecting an alternate address to avoid address conflicts with similar I2C devices. The change will have to be updated in code as well. Leave unconnected if no change in address required for I2C|
|CS|Yes. Bidirectional MOSFET with 4K7 pull-ups|**Chip Select** or Slave Select pin. By default, it is pulled HIGH using an onboard 4K7 pull-up resistor. This sets the default mode of communication for the sensor IC as I2C. For SPI, this pin acts as the chip select pin to communicate with an SPI master device. Leave unconnected when using I2C|
|INT1|No|Interrupt output pin. The LSM303D interrupt signal can be configured in a very flexible way allowing to recognize independent magnetic field detections of the X, Y and Z-axis. Or, independently recognize inertial events like an interrupt is generated when orientation moves from an unknown zone to a known zone. Single click and double clicks on any individual axis can easily be detected using any interrupt pin. FIFO Programmable threshold level, FIFO_empty or FIFO_Full events can be enabled to generate dedicated interrupts on the INT 1 or INT 2 pin. Also, An interrupt can be generated when the magnetic field value exceeds the user-defined threshold. The threshold and the timing of the two interrupt pins (INT 1 and INT 2) can be completely programmed by the user through the I2C/SPI interfaces. A detailed explanation is available in the ST LSM303D datasheet|
|INT2|No|Second Interrupt output pin. This pin acts as an additional interrupt pin for the LSM303D. It has the same functionality as of the INT1 pin. For more details, see INT1|
|GND|Not Applicable|Second Ground connection. Optionally, connect this pin to the master device's ground pin or to any externally powered devices from onboard regulator|

<img src="http://www.explorelabs.com/media/files/EX-10021-Schematic-A4-PNG-Explore-Labs-3D-Accelerometer-3D-Magnetometer-LSM303D-Compass-Breakout-Board.png" width="800" alt="Schematic A4 PNG - Explore Labs 3D Accelerometer + 3D Magnetometer LSM303D Compass Breakout Board" />

## Connections

The carrier board supports both I2C as well as SPI method of communication with a host microcontroller. The communication mode is selected by pulling the CS pin HIGH or LOW. By default, the CS pin is pulled HIGH through a 4.7 kΩ resistor on the board. This makes I2C communication the default method to talk to this carrier board.

**I2C Communication**: By default, I2C communication is implemented. The Explore Labs 3D Accelerometer + 3D Magnetometer LSM303D Compass Breakout Board can be configured and its angular velocity readings can be queried through the I2C bus. Level shifters on the I2C clock (SCL) and data (SDA) lines enable I2C communication with microcontrollers operating at the same voltage as VIN (2.5V to 5V). A detailed explanation of the I2C interface on the LSM303D can be found in its [datasheet][1] and more detailed information about I2C Protocol in general can be found in [NXP’s I2C-bus specification manual][2].

**I2C Address**: In I2C mode, the device’s 7-bit slave address has its least significant bit (LSb) determined by the voltage on the SDO/ADR pin. The carrier board pulls SDO/ADR HIGH through a 4.7 kΩ resistor, making the LSb 1 and setting the slave address to 0011101b (0x1D). This is the default device address in I2C mode. If SDO/ADR pin is connected to ground, LSb value is ‘0’ which changes the device address to 0011110b (0x1E). This solution permits the connection and addressing of two different accelerometers to the same I2C lines.

**Arduino Uno with I2C Interface**
* Sensor VIN - Arduino Uno 5V
* Sensor GND - Arduino Uno GND
* Sensor SDI/SDA - Arduino Uno Analog Input Pin A4 or SDA
* Sensor SCK/SCL - Arduino Uno Analog Input Pin A5 or SCL

**Raspberry Pi (all models) with I2C Interface**
* Sensor VIN - Raspberry Pi 3.3V on Pin 1 or 17
* Sensor GND - Raspberry Pi GND
* Sensor SDI/SDA - Raspberry Pi SDA1 (I2C) on Pin 3 or GPIO2
* Sensor SCK/SCL - Raspberry Pi SCL1 (I2C) on Pin 5 or GPIO3

**BeagleBone Black with I2C Interface**
* Sensor VIN - BeagleBone Black 3.3V
* Sensor GND - BeagleBone Black GND
* Sensor SDI/SDA - BeagleBone Black I2C2_SDA (Pin 20) on Header P9
* Sensor SCK/SCL - BeagleBone Black I2C2_SCL (Pin 19) on Header P9

## Connecting Explore Labs 3D Accelerometer + 3D Magnetometer LSM303D Compass Breakout Board using I2C (Inter Integrated Circuit) Interface
|L3GD20H|Arduino Uno|Arduino Mega|Arduino Leonardo|Arduino Due|Genuino Zero|Genuino 101|Arduino MKR1000|Raspberry Pi|BeagleBone Black|
|---|---|---|---|---|---|---|---|---|---|
|**VIN**|5V|5V|5V|3.3V|3.3V|VCC (3.3V)|3.3V|3.3V|3.3V|
|**GND**|GND|GND|GND|GND|GND|GND|GND|GND|GND|
|**SDI/SDA**|A4 or SDA|Pin 20 (SDA)|Pin 2 (SDA)|Pin 20 (SDA)|SDA|SDA|Pin 12 (SDA)|Pin 3 (SDA1) or GPIO2|Pin 20 (I2C2_SDA)|
|**SCK/SCL**|A5 or SCL|Pin 21 (SCL)|Pin 3 (SCL)|Pin 21 (SCL)|SCL|SCL|Pin 11 (SCL)|Pin 5 (SCL1) or GPIO3|Pin 19 (I2C2_SCL)|

## SPI Communication:
To use the LSM303D in SPI mode, CS pin must be driven low (connected to GND). A minimum of four logic connections are used viz., SDI, SCK, SDO and CS. These should be connected to an SPI bus operating at the same logic level as VIN. The SPI interface operates in 4-wire mode by default, with SDI and SDO on separate pins. In the default 4-wire mode, the sensor transmits data to the SPI master on a dedicated data out (SDO) line. If the SPI interface is configured to use 3-wire mode instead, the SDI line doubles as SDO and is driven by the LSM303D when it transmits data to the master. A detailed explanation of the SPI interface on the LSM303D can be found in its [datasheet][1].

**Arduino Uno with Serial Peripheral Interface (SPI)**
* Sensor VIN - Arduino Uno 5V
* Sensor GND - Arduino Uno GND
* Sensor SDI/SDA - Arduino Uno Digital Pin 11 or MOSI (Master Out Slave In)
* Sensor SCK/SCL - Arduino Uno Digital Pin 13 or SCK (Serial Clock)
* Sensor SDO/ADR - Arduino Uno Digital Pin 12 or MISO (Master In Slave Out)
* Sensor CS - Arduino Uno Digital Pin 10 or SS (Slave Select or Chip Select)

**Raspberry Pi (all models) with Serial Peripheral Interface (SPI)**
* Sensor VIN - Raspberry Pi 3.3V on Pin 1 or 17
* Sensor GND - Raspberry Pi GND
* Sensor SDI/SDA - Raspberry Pi `MOSI` (SPI) on Pin 19 or GPIO10
* Sensor SCK/SCL - Raspberry Pi `SCLK` (SPI) on Pin 23 or GPIO11
* Sensor SDO/ADR - Raspberry Pi `MISO` (SPI) on Pin 21 or GPIO9
* Sensor CS - Raspberry Pi `CE0_N` (SPI) on Pin 24 or GPIO8

**BeagleBone Black with Serial Peripheral Interface (SPI)**
* Sensor VIN - BeagleBone Black 3.3V
* Sensor GND - BeagleBone Black GND
* Sensor SDI/SDA - BeagleBone Black `SPI0_D0` (Pin 21) on Header P9
* Sensor SCK/SCL - BeagleBone Black `SPI0_SCLK` (Pin 22) on Header P9
* Sensor SDO/ADR - BeagleBone Black `SPI0_D1` (Pin 18) on Header P9
* Sensor CS - BeagleBone Black `SPI0_CS0` (Pin 17) on Header P9

## Connecting Explore Labs 3D Accelerometer + 3D Magnetometer LSM303D Compass Breakout Board using Serial Peripheral Interface (SPI)
|L3GD20H|Arduino Uno|Arduino Mega|Arduino Leonardo|Arduino Due|Genuino Zero|Genuino 101|Arduino MKR1000|Raspberry Pi|BeagleBone Black|
|---|---|---|---|---|---|---|---|---|---|
|**VIN**|5V|5V|5V|3.3V|3.3V|VCC (3.3V)|3.3V|3.3V|3.3V|
|**GND**|GND|GND|GND|GND|GND|GND|GND|GND|GND|
|**SDI/SDA**|Pin 11 or MOSI|Pin 51|Pin 4 (ICSP)|Pin 4 (ICSP)|Pin 4 (ICSP)|Pin 11 (MOSI)|Pin 8 (MOSI)|Pin 19 (`MOSI`) or GPIO10|Pin 21 (`SPI0_D0`)|
|**SCK/SCL**|Pin 13 or SCK|Pin 52|Pin 3 (ICSP)|Pin 3 (ICSP)|Pin 3 (ICSP)|Pin 13|Pin 9|Pin 23 (`SCLK`) or GPIO11|Pin 22 (`SPI0_SCLK`)|
|**SDO/ADR**|Pin 12 or MISO|Pin 50|Pin 1 (ICSP)|Pin 1 (ICSP)|Pin 1 (ICSP)|Pin 12 (MISO)|Pin 10 (MISO)|Pin 21 (`MISO`) or GPIO9|Pin 18 (`SPI0_D1`)|
|**CS**|Pin 10 or SS|Pin 53|Any IO|Any IO|Any IO|Pin 10|Any IO|Pin 24 (`CE0_N`) or GPIO8|Pin 17 (`SPI0_CS0`)|

## Resources:
[1]: www.st.com/resource/en/datasheet/lsm303d.pdf
[2]: http://www.nxp.com/documents/user_manual/UM10204.pdf
[Datasheet ST LSM303D: e-compass: 3D accelerometer and 3D magnetometer module](www.st.com/resource/en/datasheet/lsm303d.pdf "Datasheet ST LSM303D: e-compass: 3D accelerometer and 3D magnetometer module")

## Code:
Sample code is provided for interfacing the Explore Labs 3D Accelerometer + 3D Magnetometer LSM303D Compass Breakout Board to an Arduino Uno. After choosing the communication method (I2C or SPI), following code examples can be used to test the sensor.

**Using SPI**:
```c
/*
 * Interface Code for Explore Labs 3D Accelerometer + 3D Magnetometer
 * LSM303D Compass Breakout Board and Arduino Uno. This code assumes
 * selected communication method is Serial Peripheral Interface (SPI)
 * and hardware connections are made as follows:
 * 
 * Sensor VIN - Arduino Uno 5V or 3.3V
 * Sensor GND - Arduino Uno GND
 * Sensor SDI/SDA - Arduino Uno Digital Pin 11 or MOSI (Master Out Slave In)
 * Sensor SCK/SCL - Arduino Uno Digital Pin 13 or SCK (Serial Clock)
 * Sensor SDO/ADR - Arduino Uno Digital Pin 12 or MISO (Master In Slave Out)
 * Sensor CS - Arduino Uno Digital Pin 10 or SS (Slave Select or Chip Select)
 */
 
#include <SPI.h>          // Include Arduino SPI library

#define OUT_X_L_M   0x08  // Register addresses from sensor datasheet.
#define OUT_X_H_M   0x09  // Only the registers that are used
#define OUT_Y_L_M   0x0A  // in this program are defined here.
#define OUT_Y_H_M   0x0B
#define OUT_Z_L_M   0x0C
#define OUT_Z_H_M   0x0D
#define CTRL1       0x20
#define CTRL5       0x24
#define CTRL7       0x26
#define OUT_X_L_A   0x28
#define OUT_X_H_A   0x29
#define OUT_Y_L_A   0x2A
#define OUT_Y_H_A   0x2B
#define OUT_Z_L_A   0x2C
#define OUT_Z_H_A   0x2D

int8_t readData   = 0x80;
int8_t writeData  = 0x00;

int16_t ax, ay, az;       // 16-bit variables to hold raw data from sensor
int16_t mx, my, mz;
float heading;

const int CS = 10;        // Chip Select pin for SPI

void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(CS, OUTPUT);
  writeReg(CTRL1, 0x37);  // Initialize the sensor by setting
  writeReg(CTRL5, 0x04);  // control registers
  writeReg(CTRL7, 0x00);  
  delay(100);
}

void loop() {  
  mx = (int16_t) readReg(OUT_X_H_M) <<8 | readReg(OUT_X_L_M); // typecast as
  my = (int16_t) readReg(OUT_Y_H_M) <<8 | readReg(OUT_Y_L_M); // 16-bit
  mz = (int16_t) readReg(OUT_Z_H_M) <<8 | readReg(OUT_Z_L_M);
  ax = (int16_t) readReg(OUT_X_H_A) <<8 | readReg(OUT_X_L_A);
  ay = (int16_t) readReg(OUT_Y_H_A) <<8 | readReg(OUT_Y_L_A);
  az = (int16_t) readReg(OUT_Z_H_A) <<8 | readReg(OUT_Z_L_A);
  
  Serial.print("Magnetic Field (Gauss):\t");
  
  //Sensitivity from Page 10 of datasheet - use 0.08 mili-gauss/LSB for default +-2gauss.
  Serial.print(mx*0.00008F, DEC); Serial.print("\t");
  Serial.print(my*0.00008F, DEC); Serial.print("\t");
  Serial.print(mz*0.00008F, DEC); Serial.print("\t");
  Serial.println();

  Serial.print("Acceleration due to gravity (g):\t");
  
  //Sensitivity from Page 10 of datasheet - use 0.061 mili-g/LSB for default +-2g.
  Serial.print("X-Axis_A: ");
  Serial.print(ax*0.000061F, DEC); Serial.print("\t");
  Serial.print(ay*0.000061F, DEC); Serial.print("\t");
  Serial.print(az*0.000061F, DEC); Serial.print("\t");
  Serial.println();

  heading = atan2(my, mx);// Calculation for heading. North is zero degrees.
  if(heading < 0)
    heading += 2 * M_PI;
  Serial.println(heading * 180/M_PI);
  
  delay(100);
}

int8_t readReg(int8_t address) {
  int8_t buffer = 0;
  digitalWrite(CS, LOW); 
  SPI.transfer(readData | address);
  buffer = SPI.transfer(writeData);
  digitalWrite(CS, HIGH);
  return(buffer);  
}

void writeReg(int8_t address, int8_t val) {
  digitalWrite(CS, LOW);
  SPI.transfer(writeData | address);
  SPI.transfer(val);
  digitalWrite(CS, HIGH);
}
```

**Using I2C**:
```c
/*
 * Interface Code for Explore Labs 3D Accelerometer + 3D Magnetometer
 * LSM303D Compass Breakout Board and Arduino Uno. This code assumes
 * selected communication method is Inter Integrated Circuit (I2C)
 * Interface and hardware connections are made as follows:
 * 
 * Sensor VIN - Arduino Uno 5V or 3.3V
 * Sensor GND - Arduino Uno GND
 * Sensor SDI/SDA - Arduino Uno Analog Input Pin A4
 * Sensor SCK/SCL - Arduino Uno Analog Input Pin A5 
 */
 
#include <Wire.h>         // Include Arduino I2C library

#define OUT_X_L_M   0x08  // Register addresses from sensor datasheet.
#define OUT_X_H_M   0x09  // Only the registers that are used
#define OUT_Y_L_M   0x0A  // in this program are defined here.
#define OUT_Y_H_M   0x0B
#define OUT_Z_L_M   0x0C
#define OUT_Z_H_M   0x0D
#define CTRL1       0x20
#define CTRL5       0x24
#define CTRL7       0x26
#define OUT_X_L_A   0x28
#define OUT_X_H_A   0x29
#define OUT_Y_L_A   0x2A
#define OUT_Y_H_A   0x2B
#define OUT_Z_L_A   0x2C
#define OUT_Z_H_A   0x2D

int8_t readData   = 0x01;
int8_t writeData  = 0x00;
int8_t address    = 0x1D; // Device address of LSM303D with SDO/ADR/SA0 pulled HIGH.
//int8_t address  = 0x1E; // Device address of LSM303D with SDO/ADR/SA0 connected to GND.

int16_t ax, ay, az;       // 16-bit variables to hold raw data from sensor
int16_t mx, my, mz;
float heading;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  writeReg(CTRL1, 0x37);  // Initialize the sensor by setting
  writeReg(CTRL5, 0x08);  // control registers
  writeReg(CTRL7, 0x00);  
  delay(100);
}

void loop() {  
  mx = (int16_t) readReg(OUT_X_H_M) <<8 | readReg(OUT_X_L_M); // typecast as
  my = (int16_t) readReg(OUT_Y_H_M) <<8 | readReg(OUT_Y_L_M); // 16-bit
  mz = (int16_t) readReg(OUT_Z_H_M) <<8 | readReg(OUT_Z_L_M);
  ax = (int16_t) readReg(OUT_X_H_A) <<8 | readReg(OUT_X_L_A);
  ay = (int16_t) readReg(OUT_Y_H_A) <<8 | readReg(OUT_Y_L_A);
  az = (int16_t) readReg(OUT_Z_H_A) <<8 | readReg(OUT_Z_L_A);
  
  Serial.print("Magnetic Field (Gauss):\t");
  
  //Sensitivity from Page 10 of datasheet - use 0.08 mili-gauss/LSB for default +-2gauss.
  Serial.print(mx*0.00008F, DEC); Serial.print("\t");
  Serial.print(my*0.00008F, DEC); Serial.print("\t");
  Serial.print(mz*0.00008F, DEC); Serial.print("\t");
  Serial.println();

  Serial.print("Acceleration due to gravity (g):\t");
  
  //Sensitivity from Page 10 of datasheet - use 0.061 mili-g/LSB for default +-2g.
  Serial.print("X-Axis_A: ");
  Serial.print(ax*0.000061F, DEC); Serial.print("\t");
  Serial.print(ay*0.000061F, DEC); Serial.print("\t");
  Serial.print(az*0.000061F, DEC); Serial.print("\t");
  Serial.println();

  heading = atan2(my, mx);// Calculation for heading. North is zero degrees.
  if(heading < 0)
    heading += 2 * M_PI;
  Serial.println(heading * 180/M_PI);
  
  delay(100);
}

int8_t readReg(int8_t reg){
  int8_t buffer = 0;
  Wire.beginTransmission((address | writeData) >>1);
  Wire.write(reg);
  Wire.endTransmission(0);
  Wire.requestFrom((address | readData) >>1 , 1);
  while( Wire.available() == 0);
  buffer = Wire.read();
  Wire.endTransmission();
  return(buffer);  
}

void writeReg(int8_t reg, int8_t val) {
  Wire.beginTransmission((address | writeData) >>1);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
```
