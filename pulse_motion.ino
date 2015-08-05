/*
>> Pulse Sensor Amped 1.2 <<
This code is for Pulse Sensor Amped by Joel Murphy and Yury Gitman
    www.pulsesensor.com 
    >>> Pulse Sensor purple wire goes to Analog Pin 0 <<<
Pulse Sensor sample aquisition and processing happens in the background via Timer 2 interrupt. 2mS sample rate.
PWM on pins 3 and 11 will not work when using this code, because we are using Timer 2!
The following variables are automatically updated:
Signal :    int that holds the analog signal data straight from the sensor. updated every 2mS.
IBI  :      int that holds the time interval between beats. 2mS resolution.
BPM  :      int that holds the heart rate value, derived every beat, from averaging previous 10 IBI values.
QS  :       boolean that is made true whenever Pulse is found and BPM is updated. User must reset.
Pulse :     boolean that is true when a heartbeat is sensed then false in time with pin13 LED going out.

This code is designed with output serial data to Processing sketch "PulseSensorAmped_Processing-xx"
The Processing sketch is a simple data visualizer. 
All the work to find the heartbeat and determine the heartrate happens in the code below.
Pin 13 LED will blink with heartbeat.
If you want to use pin 13 for something else, adjust the interrupt handler
It will also fade an LED on pin fadePin with every beat. Put an LED and series resistor from fadePin to GND.
Check here for detailed code walkthrough:
http://pulsesensor.myshopify.com/pages/pulse-sensor-amped-arduino-v1dot1

Code Version 1.2 by Joel Murphy & Yury Gitman  Spring 2013
This update fixes the firstBeat and secondBeat flag usage so that realistic BPM is reported.
000
*/

#include <Wire.h> //I2C LIB

//  VARIABLES
int ADXAddress = 0x53;      // ADXL345 I2C address
int L3GAddress = 0x69;      // L3G4200D I2C address
int HMCAddress = 0x1E;      // HMC5883L I2C address

byte vL, vH;                // 存放低位、高位值
int xAcc,  yAcc,  zAcc;     // 存放加速度值
int xGyro, yGyro, zGyro;    // 存放角速度值
int xMag,  yMag,  zMag;     // 存放地磁场值

int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin

// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, must be seeded!
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
int loopflag = 0;

void setup() {
  pinMode(blinkPin, OUTPUT);        // pin that will blink to your heartbeat!
  pinMode(fadePin, OUTPUT);         // pin that will fade to your heartbeat!
  Serial.begin(115200);             // we agree to talk fast!
  Wire.begin();
  delay(100);
  Serial.println("starting up....");
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS
  // UN-COMMENT THE NEXT LINE IF YOU ARE POWERING The Pulse Sensor AT LOW VOLTAGE,
  // AND APPLY THAT VOLTAGE TO THE A-REF PIN
  //analogReference(EXTERNAL);

  // configure ADXL345
  writeRegister(ADXAddress, 0x2D, 0b00001000);

  // 配置L3G4200D(2000 deg/sec)
  writeRegister(L3GAddress, 0x20, 0b00001111);   // set up sleep mode, enable x, y, z axis
  writeRegister(L3GAddress, 0x21, 0b00000000);   // 选择高通滤波模式和高通截止频率
  writeRegister(L3GAddress, 0x22, 0b00000000);   // 设置中断模式
  writeRegister(L3GAddress, 0x23, 0b00110000);   // 设置量程(2000dps)、自检状态、SPI模式
  writeRegister(L3GAddress, 0x24, 0b00000000);   // FIFO & 高通滤波

  // 配置HMC5883L
  writeRegister(HMCAddress, 0x02, 0x00);         // 连续测量

  delay(2000);
  Serial.println("setup end....");
}



void loop() {
  sendDataToProcessing('S', Signal);     // send Processing the raw Pulse Sensor data
  if (QS == true) {                      // Quantified Self flag is true when arduino finds a heartbeat
    fadeRate = 255;                  // Set 'fadeRate' Variable to 255 to fade LED with pulse
    sendDataToProcessing('B', BPM);  // send heart rate with a 'B' prefix
    sendDataToProcessing('Q', IBI);  // send time between beats with a 'Q' prefix
    QS = false;                      // reset the Quantified Self flag for next time
  }

  ledFadeToBeat();
  if (loopflag == 50)
  {
    getAccValues();
    Serial.print("xA");
    Serial.println(xAcc);
    Serial.print("yA");
    Serial.println(yAcc);
    Serial.print("zA");
    Serial.println(zAcc);
    
    getGyroValues();
    Serial.print("xG");
    Serial.println(xGyro);
    Serial.print("yG");
    Serial.println(yGyro);
    Serial.print("zG");
    Serial.println(zGyro);

    getMagValues();
    Serial.print("xM");
    Serial.println(xMag);
    Serial.print("yM");
    Serial.println(yMag);
    Serial.print("zM");
    Serial.println(zMag);
  }
  if (loopflag != 50)
  {
    delay(20);                             //  take a break
    loopflag++;
  }
  else
  {
    loopflag = 0;
  }
}


void ledFadeToBeat() {
  fadeRate -= 15;                         //  set LED fade value
  fadeRate = constrain(fadeRate, 0, 255); //  keep LED fade value from going into negative numbers!
  analogWrite(fadePin, fadeRate);         //  fade LED
}


void sendDataToProcessing(char symbol, int data ) {
  Serial.print(symbol);                // symbol prefix tells Processing what type of data is coming
  Serial.println(data);                // the data to send culminating in a carriage return
}

void getAccValues() {       // 加速度值读取

  vL = readRegister(ADXAddress, 0x32);
  vH = readRegister(ADXAddress, 0x33);
  xAcc = (vH << 8) | vL;

  vL = readRegister(ADXAddress, 0x34);
  vH = readRegister(ADXAddress, 0x35);
  yAcc = (vH << 8) | vL;

  vL = readRegister(ADXAddress, 0x36);
  vH = readRegister(ADXAddress, 0x37);
  zAcc = (vH << 8) | vL;
}

void getGyroValues() {       // 角速度值读取

  vL = readRegister(L3GAddress, 0x28);
  vH = readRegister(L3GAddress, 0x29);
  xGyro = (vH << 8) | vL;

  vL = readRegister(L3GAddress, 0x2A);
  vH = readRegister(L3GAddress, 0x2B);
  yGyro = (vH << 8) | vL;

  vL = readRegister(L3GAddress, 0x2C);
  vH = readRegister(L3GAddress, 0x2D);
  zGyro = (vH << 8) | vL;
}

void getMagValues() {       // 磁场值读取

  vH = readRegister(HMCAddress, 0x03);
  vL = readRegister(HMCAddress, 0x04);
  xMag = (vH << 8) | vL;

  vH = readRegister(HMCAddress, 0x05);
  vL = readRegister(HMCAddress, 0x06);
  yMag = (vH << 8) | vL;

  vH = readRegister(HMCAddress, 0x07);
  vL = readRegister(HMCAddress, 0x08);
  zMag = (vH << 8) | vL;
}

int readRegister(int deviceAddress, byte address) {
  // 读寄存器
  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1);
  while (!Wire.available()) {}
  v = Wire.read();
  return v;
}

void writeRegister(int deviceAddress, byte address, byte val) {
  // 写寄存器
  Wire.beginTransmission(deviceAddress);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}





