#include <Arduino.h>
#include <analogWrite.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AutoPID.h>

String incomingbyte;
//Encoder and encoder switch variables

Encoder knob(34, 35);
const int encoder_SW = 14;

//OLED display

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   19
#define MAXCS  16 // CS pin Thermocouple 1
#define MAXCLK  5
#define MAXCS2  17 // CS pin Thermocouple 2

// initialize the Thermocouple 1
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS2, MAXDO);

// initialize the Thermocouple 2
Adafruit_MAX31855 thermocouple2(MAXCLK, MAXCS, MAXDO);

// L9958XP driver pins
int DIR = 27;
int PWM = 33;
//int DI = 15; // Disable pin connected to GND
int EN = 32;

//Temperaure variables
double c;
double c2;
double old_c2;
double nan_c2;

double sp = 38;

int FET = 12;
int FET2 = 13;

int TRIG = 2;
bool state = LOW;

// Initialise PID
double Input, Output;
const double Kp = 100;//100
const double Ki = 2;//2
const double Kd = 1;//1
double Setpoint = 60;

AutoPID myPID(&Input, &Setpoint, &Output, 0, 255, Kp, Ki, Kd);

// Initialise PID2
double Input2, Output2;
const double Kp2 = 10;
const double Ki2 = 1;
const double Kd2 = 0.5;
double Setpoint2 = 55;

AutoPID myPID2(&Input2, &Setpoint2, &Output2, 0, 255, Kp2, Ki2, Kd2);

// setting PWM properties
const int freq = 5000;
const int Channel = 0;
const int resolution = 8;

double positionknob = 60;

void setup()
{ 
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  //pinMode(DI, OUTPUT);
  pinMode(EN, OUTPUT);
  
  pinMode(TRIG, INPUT);

  digitalWrite(EN, HIGH);
  //digitalWrite(DI, LOW);

  pinMode(FET, OUTPUT);
  pinMode(FET2, OUTPUT);

  digitalWrite(FET, LOW);
  digitalWrite(FET2, LOW);
  
  pinMode(encoder_SW, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.setTimeout(200);
  
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

  if (!thermocouple2.begin()) 
  {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(1000); 
  display.clearDisplay();
  
  delay(2000);

  //if temperature is more than 2 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setBangBang(2);
  //myPID.setBangBang(1);
  //set PID update interval to 200ms
  myPID.setTimeStep(200);

    //if temperature is more than 2 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  myPID2.setBangBang(1);
  //myPID.setBangBang(1);
  //set PID update interval to 200ms
  myPID2.setTimeStep(200);
}

void loop()
{
//  double newpos;
//  newpos = positionknob + knob.read()*0.125;
//  
//  if (newpos != positionknob)
//  {
//    Setpoint = newpos;
//  }
//  
  Setpoint2 = 55+knob.read()*0.125;
  
    // send data only when you receive data:
  if (Serial.available() > 0) 
  {
    // read the incoming byte:
    incomingbyte = Serial.readString();
    if (incomingbyte.toDouble()>=18 && incomingbyte.toDouble()<=70){
      Setpoint = incomingbyte.toDouble();
    }
    //Serial.println(incomingbyte);
  }

  c2 = thermocouple2.readCelsius();
  c = thermocouple.readCelsius();
  
  if (isnan(c2)||isnan(c))//(isnan(c2)||isnan(c))
    {
        return; 
    }
  else
  {
   Input = c2;
   Input2 = c;
   
   myPID.run();
   myPID2.run();
   
   analogWrite(FET, Output);
   analogWrite(FET2, Output2);
   
   OLED_display();

//   Serial.print(c2);
//   Serial.print(" ");
//   Serial.println(c);
   
//   Serial.print(Setpoint);
//   Serial.print(" ");
//   Serial.print(c2);
//   Serial.print(" ");
 //   Serial.print(Output);
  //  Serial.print(" ");
  //  Serial.println(Output2);
  }

}

void OLED_display()
{   
    display.clearDisplay();
    display.setTextColor(WHITE);        // Draw white text
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print(F("TControl"));
    display.setTextSize(2);
    display.println();
//    display.println();
   // display.print(F("T1:"));
//    display.println();
   // display.setTextSize(2);
   // display.print(c);
   // display.setTextSize(2);
   // display.println  (F(" C"));
//    display.println();
    display.print(F("Tc:"));
    display.print(c2);
    display.println(F(" C"));
    display.print(F("Tp:"));
    display.print(c);
    display.println(F(" C"));
    display.setTextSize(1);
    display.print(F("DT(c):"));
    display.print(Setpoint);
    display.println(F(" C"));
    display.print(F("DT(p):"));
    display.print(Setpoint2);
    display.println(F(" C"));
    display.setTextSize(1);
    //if(state == 0) {display.print(F("NOT HEATING"));}
    //if(state == 1) {display.print(F("HEATING"));}
    //display.print(digitalRead(encoder_SW));
    display.display(); 
}
