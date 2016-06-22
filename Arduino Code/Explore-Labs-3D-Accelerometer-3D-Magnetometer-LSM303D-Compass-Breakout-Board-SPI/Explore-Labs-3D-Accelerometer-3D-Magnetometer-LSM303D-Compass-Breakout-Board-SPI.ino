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

int16_t ax, ay, az;   //16-bit variables to hold raw data from sensor
int16_t mx, my, mz;
float heading;

const int CS = 10;    // Chip Select pin for SPI

void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(CS, OUTPUT);
  writeReg(CTRL1, 0x37); // Initialize the sensor by setting
  writeReg(CTRL5, 0x04); // control registers
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

  heading = atan2(my, mx);  // Calculation for heading. North is zero degrees.
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
