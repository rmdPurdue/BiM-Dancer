#include <OSCMessage.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <L3G4200D.h>
#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include "Network.h"

/**************************************************
*                                                 *
*  BaMDancer provides processing for a suite of   *
*  wearable sensors. The software initializes     *
*  an i2C bus, and connects to ADS1115 ADC        *
*  devices on the bus. These devices can connect  *
*  to a variety of analog sensors.                *
*                                                 *
*  Once connected to the bus, the controller      *
*  collects data, populates an OSC bundle, and    *
*  transmits the bundle over a WiFi connection.   *
*                                                 *
*  (c) 2016 Rich Dionne                           *
*                                                 *
**************************************************/

const float NO_BEND = 12000;
const float FULL_BEND = 6000;
const float NO_BEND2 = 20600;
const float FULL_BEND2 = 12400;
const float NO_PRESSURE = 375;
const float FULL_PRESSURE = 2400;
const float GYRO_MIN = -15000;
const float GYRO_MAX = 15000;
const int powerLED = 0;
const int wifiLED = 2;

WiFiUDP Udp;

Adafruit_ADS1115 ADC1(0x48);
Adafruit_ADS1115 ADC2(0x49);
Adafruit_ADS1115 ADC3(0x4A);
Adafruit_ADS1115 ADC4(0x4B);
L3G4200D gyro;

int16_t ADC1_0, ADC1_1, ADC1_2, ADC1_3, ADC2_0, ADC2_1,
     ADC2_2, ADC2_3, ADC3_0, ADC3_1, ADC3_2, ADC3_3,
     ADC4_0, ADC4_1, ADC4_2, ADC4_3, gyroX, gyroY, gyroZ;

boolean adc1Present, adc2Present, adc3Present, adc4Present, gyroPresent;

void setup() {
  adc1Present = false;
  adc2Present = false;
  adc3Present = false;
  adc4Present = false;
  gyroPresent = false;

  pinMode(powerLED, OUTPUT);
  pinMode(wifiLED, OUTPUT);

  digitalWrite(powerLED, LOW);
  
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  boolean ledState = LOW;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(wifiLED, ledState);
    ledState = !ledState;
  }

  Serial.println("");
  Serial.println("Wifi connected.");
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(wifiLED, LOW);

  if(Udp.begin(outPort) == 1) {
    Serial.println("UDP Connection successful.");
  } else {
    Serial.println("UDP Connection failed.");
  }

  Wire.begin();
  delay(5000);
  for(byte i = 1; i < 120; i++) {
    Wire.beginTransmission(i);
    if(Wire.endTransmission() == 0) {
      delay(1);
      switch(i) {
        case 72:
        adc1Present = true;
        break;
        case 73:
        adc2Present = true;
        break;
        case 74:
        adc3Present = true;
        break;
        case 75:
        adc4Present = true;
        break;
        case 104:
        gyroPresent = true;
        break;
      }
    }
  }
  if(adc1Present) {
    Serial.println("ADC1 Detected.");
    ADC1.setGain(GAIN_ONE);
  }
  if(adc2Present) {
    Serial.println("ADC2 Detected.");
    ADC2.setGain(GAIN_ONE);
  }
  if(adc3Present) {
    Serial.println("ADC3 Detected.");
    ADC3.setGain(GAIN_ONE);
  }
  if(adc4Present) {
    Serial.println("ADC4 Detected.");
    ADC4.setGain(GAIN_ONE);
  }
  if(gyroPresent) {
//    Serial.println("Gyro Detected.");
    gyro.initialize(250);
  }  
}

void loop() {
  /***** Read ADC 1 values *****/
  if(adc1Present) {
    ADC1_0 = ADC1.readADC_SingleEnded(0);
    ADC1_1 = ADC1.readADC_SingleEnded(1);
    ADC1_2 = ADC1.readADC_SingleEnded(2);
    ADC1_3 = ADC1.readADC_SingleEnded(3);
  }

  /***** Read ADC 2 values *****/
  if(adc2Present) {
    ADC2_0 = ADC2.readADC_SingleEnded(0);
    ADC2_1 = ADC2.readADC_SingleEnded(1);
    ADC2_2 = ADC2.readADC_SingleEnded(2);
    ADC2_3 = ADC2.readADC_SingleEnded(3);
  }
  
  /***** Read ADC 3 values *****/
  if(adc3Present) {
    ADC3_0 = ADC3.readADC_SingleEnded(0);
    ADC3_1 = ADC3.readADC_SingleEnded(1);
    ADC3_2 = ADC3.readADC_SingleEnded(2);
    ADC3_3 = ADC3.readADC_SingleEnded(3);
  }

  /***** Read ADC 4 values *****/
  if(adc4Present) {
    ADC4_0 = ADC4.readADC_SingleEnded(0);
    ADC4_1 = ADC4.readADC_SingleEnded(1);
    ADC4_2 = ADC4.readADC_SingleEnded(2);
    ADC4_3 = ADC4.readADC_SingleEnded(3);
  }

  /***** Read the X, Y, Z gyroscope values  *****/
  if(gyroPresent) {
    gyroX = gyro.getX();
    gyroY = gyro.getY();
    gyroZ = gyro.getZ();
  }

  sendMessageViaOSC();
  delay(100);
}

// collapse this to a single message with multiple arguments

void sendMessageViaOSC() {
  OSCMessage message(url);
  message.add(scaleData(300, 0, 1024));
  message.add(scaleData(512, 0, 1024));
  message.add(scaleData(1024, 0, 1024));
  message.add(scaleData(ADC2_0, 0 , 1024));
  message.add(scaleData(ADC2_1, 0 , 1024));
  message.add(scaleData(ADC2_2, 0 , 1024));
  message.add(scaleData(ADC2_3, 0 , 1024));
  message.add(scaleData(ADC3_0, 0 , 1024));
  message.add(scaleData(ADC3_1, 0 , 1024));
  message.add(scaleData(ADC3_2, 0 , 1024));
  message.add(scaleData(ADC3_3, 0 , 1024));
  message.add(scaleData(ADC4_0, 0 , 1024));
  message.add(scaleData(ADC4_1, 0 , 1024));
  message.add(scaleData(ADC4_2, 0 , 1024));
  message.add(scaleData(ADC4_3, 0 , 1024));
  message.add(scaleData(gyroX, 0 , 1024));
  message.add(scaleData(gyroY, 0 , 1024));
  message.add(scaleData(gyroZ, 0 , 1024));
  Udp.beginPacket(outIP, outPort);
  message.send(Udp);
  Udp.endPacket();
  message.empty();
}

int scaleData(float val, float low, float hi) {
  // Scale each sensor reading to a single byte
  int byte = map(val,low,hi,0,255);
  if(byte < 0) byte = 0;
  if(byte > 255) byte = 255;
  return byte;
}

void blinkLED(int pin, int number) {
  for(int i = 0 ; i < number; i++) {
    digitalWrite(pin, LOW);
    delay(500);
    digitalWrite(pin, HIGH);
    delay(500);
  }
}

