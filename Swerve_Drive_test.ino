#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>
#include <math.h>

#include <SPI.h>
#include "hidjoystickrptparser.h"

#include <HardwareSerial.h>
#include <ODriveArduino.h>

#include <Sabertooth.h>

// Printing with stream operator helper functions odrive
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

//ODrive communication
HardwareSerial& odrive_serial2 = Serial2;
HardwareSerial& odrive_serial3 = Serial3;

// ODrive object
ODriveArduino odrive1(odrive_serial2);
ODriveArduino odrive2(odrive_serial3);

Sabertooth ST1(128, Serial);
Sabertooth ST2(128, Serial1);

//Joystick derived
float thetal = 0;
float vl = 0;

float thetar = 0;
float vr = 0;

float VAL = 0;
float VBL = 0;
float VCL = 0;
float VDL = 0;

float VAR = 0;
float VBR = 0;
float VCR = 0;
float VDR = 0;

float VL1 = 0;
float VL2 = 0;
float VL3 = 0;
float VL4 = 0;

float VR1 = 0;
float VR2 = 0;
float VR3 = 0;
float VR4 = 0;

float W1 = 0;
float W2 = 0;
float W3 = 0;
float W4 = 0;

float thetaRA = 0;
float thetaRB = 0;
float thetaRC = 0;
float thetaRD = 0;

float thetaLA = 0;
float thetaLB = 0;
float thetaLC = 0;
float thetaLD = 0;

//Logitech 310
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);
JoystickEvents JoyEvents;
JoystickReportParser Joy(&JoyEvents);

float Buttons = JoyEvents.bu;
float lxa =  JoyEvents.lx;        //  map( JoyEvents.lx, 0, 127,0, 1023);      //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f);
float lya = JoyEvents.ly;           //map(JoyEvents.ly, 0, 127, 0, 1023);                   // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);
float rxa = JoyEvents.rx;          //         map(JoyEvents.rx, 0, 127, 0, 1023); // map(JoyEvents.Z2, 0, 0xFF, 0.f, 255.f);
float rya = JoyEvents.ry;          // map(JoyEvents.ry, 0, 127, 0, 1023); // map(JoyEvents.Rz, 0, 0xFF, 0.f, 255.f);
//Group initialize
float blue = Joy.blue;
float green = Joy.green;
float red = Joy.red;
float yellow = Joy.yellow;
float L1 = Joy.lb;
float R1 = Joy.rb;
float gpad = JoyEvents.ht;
float L2 = Joy.lt;
float R2 = Joy.rt;
float back = Joy.bk;
float start = Joy.st;
float leftjoy = Joy.jl;
float rightjoy = Joy.jr;
int result = 0;

//Encoder counter
volatile int pulse1 = 0;
volatile int pulse2 = 0;
volatile int pulse3 = 0;
volatile int pulse4 = 0;

void setup() {
  // put your setup code here, to run once:

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  while (Usb.Init() == -1)
    Serial.println("OSC did not start.");

  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t > (PSTR("SetReportParser"), 1);

  pinMode(4, OUTPUT);
  digitalWrite(4, 1);

  pinMode(12, OUTPUT);

  // ODrive uses 115200 baud
  odrive_serial2.begin(115200);
  odrive_serial3.begin(115200);

  Serial.begin(115200);
  Serial1.begin(19200);
  //  Serial2.begin(9600);
  Sabertooth::autobaud(Serial); // Autobaud is for the whole serial line -- you don't need to do
  Sabertooth::autobaud(Serial1); // Autobaud is for the whole serial line -- you don't need to do

  //base encoders
  pinMode(2, INPUT_PULLUP);        //Motor1
  pinMode(5, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), motor1, RISING);

  pinMode(3, INPUT_PULLUP);        //Motor2
  pinMode(6, INPUT_PULLUP);

  attachInterrupt( digitalPinToInterrupt(3), motor2, RISING);

  pinMode(20, INPUT_PULLUP);        //Motor3
  pinMode(7, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(20), motor3, RISING);

  pinMode(21, INPUT_PULLUP);        //Motor4
  pinMode(8, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(21), motor4, RISING);

}

void loop() {
  // put your main code here, to run repeatedly:
  lxa = lxa - 128;
  lya = lya - 127;
  rxa = rxa - 128;
  rya = rya - 127;

  Usb.Task();

  int maplx =  map(JoyEvents.lx, 0, 255, -100, 100);     //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f); JoyEvents.lx;
  int maply =  map(JoyEvents.ly, 0, 255, 100, -100);      // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);  JoyEvents.ly;

  vl = sqrt((pow(maplx, 2)) + (pow(maply, 2)));
  thetal = atan2(maply, maplx);

  int maprx =  map(JoyEvents.rx, 0, 255, -100, 100);     //map(JoyEvents.Y, 0, 0xFF, 0.f, 255.f); JoyEvents.lx;
  int mapry =  map(JoyEvents.ry, 0, 255, 100, -100);      // map(JoyEvents.Z1, 0, 0xFF, 0.f, 255.f);  JoyEvents.ly;

  vr = sqrt((pow(maprx, 2)) + (pow(mapry, 2)));
  thetal = atan2(maply, maprx);

  if (((-25 <= maplx) && (maplx <= 25)) && ((-25 <= maply) && (maply <= 25)))
  {
    vl = 0;
    thetal = 0;
  }

  if (((-25 <= maprx) && (maprx <= 25)) && ((-25 <= mapry) && (mapry <= 25))) {
    vr = 0;
    thetar = 0;

    //Serial.print(" *********************************** ");
  }
  Serial.println(" \t maplx : ");
  Serial.print(maplx);
  Serial.print(" \t maply : ");
  Serial.print(maply);

  Serial.print(" \t maprx : ");
  Serial.print(maprx);
  Serial.print(" \t mapry : ");
  Serial.print(mapry);

  thetal = atan2(maply, maplx) * PI/180;
  Serial.print(" \t thetal : ");
  Serial.print(thetal);

  thetar = atan2(mapry, maprx) * PI/180;
  Serial.print(" \t thetar : ");
  Serial.print(thetar);

  VAL = maplx - (thetal * 312.5);
  VBL = maplx + (thetal * 312.5);
  VCL = maply - (thetal * 262.5);
  VDL = maply + (thetal * 262.5);

  VL1 = sqrt((pow(VBL, 2)) + (pow(VCL, 2)));
  VL2 = sqrt((pow(VBL, 2)) + (pow(VDL, 2)));
  VL3 = sqrt((pow(VAL, 2)) + (pow(VDL, 2)));
  VL4 = sqrt((pow(VAL, 2)) + (pow(VCL, 2)));

  int VLA =  map(VL1, -100, 2, -1000, 10);
  int VLB =  map(VL2, -100, 2, -1000, 10);
  int VLC =  map(VL3, -100, 2, -1000, 10);
  int VLD =  map(VL4, -100, 2, -1000, 10);

//  Serial.println("\t VL1 : ");
//  Serial.print(VAL);
//  Serial.print("\t VL2 : ");
//  Serial.print(VBL);
//  Serial.print("\t VL3 : ");
//  Serial.print(VCL);
//  Serial.print("\t VL4 : ");
//  Serial.print(VDL);

  VAR = maprx - (thetar * 312.5);
  VBR = maprx + (thetar * 312.5);
  VCR = mapry - (thetar * 262.5);
  VDR = mapry + (thetar * 262.5);

  VR1 = sqrt((pow(VBR, 2)) + (pow(VCR, 2)));
  VR2 = sqrt((pow(VBR, 2)) + (pow(VDR, 2)));
  VR3 = sqrt((pow(VAR, 2)) + (pow(VDR, 2)));
  VR4 = sqrt((pow(VAR, 2)) + (pow(VCR, 2)));

  int VRA =  map(VR1, -100, 2, -1000, 10);
  int VRB =  map(VR2, -100, 2, -1000, 10);
  int VRC =  map(VR3, -100, 2, -1000, 10);
  int VRD =  map(VR4, -100, 2, -1000, 10);

//  Serial.print("\t VL1 : ");
//  Serial.print(VAR);
//  Serial.print("\t VL2 : ");
//  Serial.print(VBR);
//  Serial.print("\t VL3 : ");
//  Serial.print(VCR);
//  Serial.print("\t VL4 : ");
//  Serial.print(VDR);

  int final_V1 = VLA + VRA;
  int final_V2 = VLB + VRB;
  int final_V3 = VLC + VRC;
  int final_V4 = VLD + VRD;

  odrive1.SetVelocity(0, final_V1);
  odrive1.SetVelocity(1, final_V2);
  odrive2.SetVelocity(0, final_V3);
  odrive2.SetVelocity(1, final_V4);

  /////////////////////////////////////////////

  thetaLA = (atan2(VBL, VCL)) * (180 / PI);
  thetaLB = (atan2(VBL, VDL)) * (180 / PI);
  thetaLC = (atan2(VAL, VDL)) * (180 / PI);
  thetaLD = (atan2(VAL, VCL)) * (180 / PI);

  thetaRA = (atan2(VBR, VCR)) * (180 / PI);
  thetaRB = (atan2(VBR, VDR)) * (180 / PI);
  thetaRC = (atan2(VAR, VDR)) * (180 / PI);
  thetaRD = (atan2(VAR, VCR)) * (180 / PI);

  W1 =  thetaLA + thetaRA;
  W2 =  thetaLB + thetaRB;
  W3 =  thetaLC + thetaRC;
  W4 =  thetaLD + thetaRD;

  int ppr = 432 / 360;


//  Serial.print("\t W1 : ");
//  Serial.print(W1);
//  Serial.print("\t W2 : ");
//  Serial.print(W2);
//  Serial.print("\t W3 : ");
//  Serial.print(W3);
//  Serial.print("\t W4 : ");
//  Serial.print(W4);

  //if ( pulse1 >= ppr*W1 || pulse2 >= ppr*W2 || pulse3 >= ppr*W3 || pulse4 >= ppr*W4)
  //{
  //    ST1.motor(1, 0);
  //    ST2.motor(2, 0);
  //    ST2.motor(1, 0);
  //    ST2.motor(2, 0);
  //}
  //else {
  //    ST1.motor(1, W1);
  //    ST2.motor(2, W2);
  //    ST2.motor(1, W3);
  //    ST2.motor(2, W4);
  //}
  if ( pulse1 >= ppr * W1) {
    ST1.motor(1, 0);
  }
  else {
    ST1.motor(1, W1);
  }
  if ( pulse2 >= ppr * W2) {
    ST1.motor(2, 0);
  }
  else {
    ST1.motor(2, W2);
  }
  if ( pulse3 >= ppr * W3) {
    ST2.motor(1, 0);
  }
  else {
    ST2.motor(1, W3);
  }
  if ( pulse4 >= ppr * W4) {
    ST2.motor(2, 0);
  }
  else {
    ST2.motor(2, W4);
  }
}

//rotary encoders

void motor1()
{
  if (digitalRead(2))
  {
    if (digitalRead(5))
      --pulse1;
    else
      ++pulse1;
  }
}

void motor2()
{
  if (digitalRead(3))
  {
    if (digitalRead(6))
      ++pulse2;
    else
      --pulse2;
  }
}

void motor3()
{
  if (digitalRead(20))
  {
    if (digitalRead(7))
      ++pulse3;
    else
      --pulse3;
  }
}

void motor4()
{
  if (digitalRead(21))
  {
    if (digitalRead(8))
      ++pulse4;
    else
      --pulse4;
  }
}
