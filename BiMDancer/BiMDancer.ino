#include <SoftwareSerial.h>
#include <L3G4200D.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>

/**************************************************
*                                                 *
*  BiMDancer provides processing for a suite of   *
*  sensors embedded on a dancer's costume, that   *
*  will read data from five sensors over an i2C   *
*  network, and transmit that data wirelessly     *
*  over an XBee network.                          *
*                                                 *
*  (c) 2016 Rich Dionne                           *
*                                                 *
**************************************************/

SoftwareSerial xbee(9,10);

Adafruit_ADS1115 leftArm(0x48);
Adafruit_ADS1115 rightArm(0x49);
Adafruit_ADS1115 leftLeg(0x4A);
Adafruit_ADS1115 rightLeg(0x4B);
L3G4200D gyro;

int16_t leftArmX, leftArmY, leftArmZ, rightArmX, rightArmY, rightArmZ,
     leftLegX, leftLegY, leftLegZ, rightLegX, rightLegY, rightLegZ,
     gyroX, gyroY, gyroZ;
     
char startChar, endChar, delimChar;

void setup() {
  Wire.begin();
  gyro.initialize(2000);
  startChar = '#';    // Indicates start of Serial message
  endChar = '!';      // Indicates end of Serial message
  delimChar = ',';    // Serial message data delimeter
}

void loop() {
  /***** Read X, Y, Z accelerometer values on left arm  *****/
  leftArmX = leftArm.readADC_SingleEnded(1);
  leftArmY = leftArm.readADC_SingleEnded(2);
  leftArmZ = leftArm.readADC_SingleEnded(3);
  
  /***** Read X, Y, Z accelerometer values on right arm *****/
  rightArmX = rightArm.readADC_SingleEnded(1);
  rightArmY = rightArm.readADC_SingleEnded(2);
  rightArmZ = rightArm.readADC_SingleEnded(3);

  /***** Read X, Y, Z accelerometer values on left leg  *****/
  leftLegX = leftLeg.readADC_SingleEnded(1);
  leftLegY = leftLeg.readADC_SingleEnded(2);
  leftLegZ = leftLeg.readADC_SingleEnded(3);
  
  /***** Read X, Y, Z accelerometer values on right leg *****/
  rightLegX = rightLeg.readADC_SingleEnded(1);
  rightLegY = rightLeg.readADC_SingleEnded(2);
  rightLegZ = rightLeg.readADC_SingleEnded(3);

  /***** Read the X, Y, Z gyroscope values              *****/
  gyroX = gyro.getX();
  gyroY = gyro.getY();
  gyroZ = gyro.getZ();
  
  scaleData();

  sendSerial();
}

void scaleData() {
  /* We'll need to do some tests to figure out the
     appropriate scaling factor for each sensor.
     Ideally, we will get this down to a single
     byte each. */
}

void sendSerial() {
  byte outgoingBytes[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  outgoingBytes[0] = leftArmX << 8;
  outgoingBytes[1] = leftArmX & B11111111;
  outgoingBytes[2] = leftArmY << 8;
  outgoingBytes[3] = leftArmY & B11111111;
  outgoingBytes[4] = leftArmZ << 8;
  outgoingBytes[5] = leftArmZ & B11111111;
  outgoingBytes[6] = rightArmX << 8;
  outgoingBytes[7] = rightArmX & B11111111;
  outgoingBytes[8] = rightArmY << 8;
  outgoingBytes[9] = rightArmY & B11111111;
  outgoingBytes[10] = rightArmZ << 8;
  outgoingBytes[11] = rightArmZ & B11111111;
  outgoingBytes[12] = leftLegX << 8;
  outgoingBytes[13] = leftLegX & B11111111;
  outgoingBytes[14] = leftLegY << 8;
  outgoingBytes[15] = leftLegY & B11111111;
  outgoingBytes[16] = leftLegZ << 8;
  outgoingBytes[17] = leftLegZ & B11111111;
  outgoingBytes[18] = rightLegX << 8;
  outgoingBytes[19] = rightLegX & B11111111;
  outgoingBytes[20] = rightLegY << 8;
  outgoingBytes[21] = rightLegY & B11111111;
  outgoingBytes[22] = rightLegZ << 8;
  outgoingBytes[23] = rightLegZ & B11111111;
  outgoingBytes[24] = gyroX << 8;
  outgoingBytes[25] = gyroX & B11111111;
  outgoingBytes[26] = gyroY << 8;
  outgoingBytes[27] = gyroY & B11111111;
  outgoingBytes[28] = gyroZ << 8;
  outgoingBytes[29] = gyroZ & B11111111;

  xbee.print("#");
  for(int i = 0; i < 30; i++) {
    xbee.write(outgoingBytes[i]);
    xbee.print(",");
  }
  xbee.print("!");
}
