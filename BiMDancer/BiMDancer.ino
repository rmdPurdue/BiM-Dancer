#include <Printers.h>
#include <XBee.h>
//#include <SoftwareSerial.h>
#include <L3G4200D.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include "BiMDancer.h"

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

//SoftwareSerial xbee(9,10);

Adafruit_ADS1115 leftArm(0x48);
Adafruit_ADS1115 rightArm(0x49);
Adafruit_ADS1115 leftLeg(0x4A);
Adafruit_ADS1115 rightLeg(0x4B);
L3G4200D gyro;
XBee xbee = XBee();
uint8_t payload[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40a71640);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();


int16_t leftArmX, leftArmY, leftArmZ, rightArmX, rightArmY, rightArmZ,
     leftLegX, leftLegY, leftLegZ, rightLegX, rightLegY, rightLegZ,
     gyroX, gyroY, gyroZ;


void setup() {
  Serial1.begin(57600);
  Serial.begin(9600);
  xbee.setSerial(Serial1);
  Wire.begin();
//  gyro.initialize(2000);
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
//  gyroX = gyro.getX();
//  gyroY = gyro.getY();
//  gyroZ = gyro.getZ();

  payload[0] = leftArmX >> 8;
  payload[1] = leftArmX & B11111111;
  payload[2] = leftArmY >> 8;
  payload[3] = leftArmY & B11111111;
  payload[4] = leftArmZ >> 8;
  payload[5] = leftArmZ & B11111111;
  payload[6] = rightArmX >> 8;
  payload[7] = rightArmX & B11111111;
  payload[8] = rightArmY >> 8;
  payload[9] = rightArmY & B11111111;
  payload[10] = rightArmZ >> 8;
  payload[11] = rightArmZ & B11111111;
  payload[12] = leftLegX >> 8;
  payload[13] = leftLegX & B11111111;
  payload[14] = leftLegY >> 8;
  payload[15] = leftLegY & B11111111;
  payload[16] = leftLegZ >> 8;
  payload[17] = leftLegZ & B11111111;
  payload[28] = rightLegX >> 8;
  payload[19] = rightLegX & B11111111;
  payload[20] = rightLegY >> 8;
  payload[21] = rightLegY & B11111111;
  payload[22] = rightLegZ >> 8;
  payload[23] = rightLegZ & B11111111;
  payload[24] = gyroX >> 8;
  payload[25] = gyroX & B11111111;
  payload[26] = gyroY >> 8;
  payload[27] = gyroY & B11111111;
  payload[28] = gyroZ >> 8;
  payload[29] = gyroZ & B11111111;
  Serial.print(leftArmX);
  Serial.print(" ");
  Serial.print(leftArmY);
  Serial.print(" ");
  Serial.print(leftArmZ);
  Serial.print(" ");
  Serial.print(rightArmX);
  Serial.print(" ");
  Serial.print(rightArmY);
  Serial.print(" ");
  Serial.print(rightArmZ);
  Serial.print(" ");
  Serial.print(leftLegX);
  Serial.print(" ");
  Serial.print(leftLegY);
  Serial.print(" ");
  Serial.print(leftLegZ);
  Serial.print(" ");
  Serial.print(rightLegX);
  Serial.print(" ");
  Serial.print(rightLegY);
  Serial.print(" ");
  Serial.print(rightLegZ);
  Serial.print(" ");
  Serial.print(gyroX);
  Serial.print(" ");
  Serial.print(gyroY);
  Serial.print(" ");
  Serial.print(gyroZ);
  
  Serial.println();
  xbee.send(zbTx);
  if(xbee.readPacket(500)) {
    if(xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if(txStatus.getDeliveryStatus() == SUCCESS) {
        Serial.println("Success");
      } else {
        Serial.println("Failure");
      }
    }
  }else if (xbee.getResponse().isError()) {
    Serial.println("Error Reading Packet");
  } else {
    Serial.println("Local XBee slow to respond");
  }
  
  delay(500);
}

void scaleData() {
  /* We'll need to do some tests to figure out the
     appropriate scaling factor for each sensor.
     Ideally, we will get this down to a single
     byte each. */
}
