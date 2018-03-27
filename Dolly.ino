#include <SoftwareSerial.h>
#include <Wire.h>
#include <TimerOne.h>

// include the library code:
#include <LiquidCrystal_I2C.h>


//Motor Shield
#include "AccelStepper.h"
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//ClickEncoder
#include "ClickEncoder.h"

//I2C ADRESSE FIND WITH I2C_SCANNER : http://playground.arduino.cc/Main/I2cScanner#.UxJJG_0xJFI

LiquidCrystal_I2C lcd(0x3F,16,2);

SoftwareSerial BTserial(8, 9); // RX | TX

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myStepper1 = AFMS.getStepper(200, 1);
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);

char* MenueLine[] = {" Speed"," Duration"," Direction"};

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
int motorRelease = 0;

void forwardstep1() {
  motorRelease = motorRelease + 1;
  myStepper1->onestep(FORWARD, SINGLE);
}
void backwardstep1() {  
  motorRelease = motorRelease + 1;
  myStepper1->onestep(BACKWARD, SINGLE);
}

void betweenstep() {
  if(motorRelease == 3) {
    //myStepper1->release();
    motorRelease = 0;  
    Serial.println("BETWEEN STEP");
  }
}

// Now we'll wrap the steppers in an AccelStepper object
AccelStepper stepper1(forwardstep1, backwardstep1, betweenstep);
 
char c = ' ';
boolean motorRun = false;

//Rotary Selector
#define ENCODER_PIN_CLK 11
#define ENCODER_PIN_DT 12
#define ENCODER_PIN_SW 13

ClickEncoder *encoder;
int16_t last, value;

void timerIsr() {
  encoder->service();
}

boolean ClickEncoderHeld = false;

void setup()
{
  //Config input pin for Rotary Encoder
  //pinMode (PinCLK,INPUT);
  //pinMode (PinDT,INPUT);
  //pinMode (PinSW,INPUT_PULLUP);
  
  //Init Shield Motor
  AFMS.begin();

  //Init Motor 1
  stepper1.setMaxSpeed(0.5);
  stepper1.setAcceleration(100);
  stepper1.moveTo(100);

  //New Encoder
  encoder = new ClickEncoder(ENCODER_PIN_DT, ENCODER_PIN_CLK, ENCODER_PIN_SW, 4);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  last = -1;

  
  lcd.init(); 
  lcd.cursor_on();
  lcd.blink_on();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Dolly booting !");

  //INIT SERIAL
  Serial.begin(9600);
  Serial.println("Arduino is ready");
  Serial.println("Remember to select Both NL & CR in the serial monitor");

  // AT-09 default serial speed for AT mode is 9600
  BTserial.begin(9600);

  sendCommand("AT");
  sendCommand("AT+ROLE0");
  sendCommand("AT+UUID0xFFE0");
  sendCommand("AT+CHAR0xFFE1");
  sendCommand("AT+NAMEDolly");

  //DISPLAY INIT
  lcd.setCursor(0,0);
  lcd.print("Dolly is alive !");
  delay(1000);  
  lcd.clear();
  lcd.cursor_off();
  lcd.blink_off();
}

void loop()
{
  value += encoder->getValue();

  if (value < 1) {
    value = 1;
  }
  if (value > 250) {
    value = 250;
  }

  myMotor->setSpeed(value);
  stepper1.setMaxSpeed(0.2);

  if (value != last) {
    last = value;
    Serial.print("Encoder Value: ");
    Serial.println(value);

    lcd.setCursor(0, 0);
    lcd.print("         ");
    lcd.setCursor(0, 0);
    lcd.print(value);

  }

  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) {
    //Serial.print("Button: ");
    #define VERBOSECASE(label) case label: Serial.println(#label); break;
    switch (b) {
      //VERBOSECASE(ClickEncoder::Pressed);
      //VERBOSECASE(ClickEncoder::Held)
      //VERBOSECASE(ClickEncoder::Released)
      //VERBOSECASE(ClickEncoder::Clicked)
      //VERBOSECASE(ClickEncoder::DoubleClicked)
      case ClickEncoder::Held:
          if(!ClickEncoderHeld) {
            ClickEncoderHeld = true;
            Serial.println("ClickEncoder::Held");
            motorRun = !motorRun;
            if (motorRun) {
              Serial.println("Motor::RUN");
            } else {
              Serial.println("Motor::STOP");
              myStepper1->release();
            }
          }
          break;
      case ClickEncoder::Released:
          Serial.println("ClickEncoder::Released");
          ClickEncoderHeld = false;
          break;
      case ClickEncoder::Clicked:
          Serial.println("ClickEncoder::Clicked");
          motorRun = !motorRun;
          myStepper1->release();
          break;
      case ClickEncoder::DoubleClicked:
          Serial.println("ClickEncoder::DoubleClicked");
          encoder->setAccelerationEnabled(!encoder->getAccelerationEnabled());
          Serial.print("  Acceleration is ");
          Serial.println((encoder->getAccelerationEnabled()) ? "enabled" : "disabled");
          displayAccelerationStatus();
        break;
    }
  }   
  
  // Change direction at the limits
  if (stepper1.distanceToGo() == 0) {
    stepper1.moveTo(-stepper1.currentPosition());
  }

  if (motorRun) {
    //Serial.println("MOTOR:STEP");
    //stepper1.run();
    myMotor->run(FORWARD);
  } else {
    myMotor->run(RELEASE);
  }
  
  // Keep reading from AT-09 and send to Arduino Serial Monitor
  if (BTserial.available())
  {
    c = BTserial.read();
    Serial.write(c);
    //lcd.setCursor(0,0);
    //lcd.print(c);
  }

 //readTextFromBT();

 // Keep reading from Arduino Serial Monitor and send to HC-05
 if (Serial.available())
  {
    c = Serial.read();
    BTserial.write(c);
  }

/*
  if (!(digitalRead(PinSW))) {      // Reset la position si on appui sur le potentiomètre
     encoderPos = 0;
     lcd.setCursor(0,1);
     lcd.print("Reset position");
     sendTextToBT("Reset Position");
     Serial.println("Reset position");
   }
   
   n = digitalRead(PinCLK);
   
   if ((PinCLKLast == LOW) && (n == HIGH)) {
     
     if (digitalRead(PinDT) == LOW) {
      lcd.setCursor(0,1);
      lcd.print("< pos      ");
       Serial.print("<->ah, position ");
       sendTextToBT("<->ah, position ");
       encoderPos--;
       if ( encoderPos < 0 ) {
         encoderPos = nbPas;
       }
     } else {
       lcd.setCursor(0,1);
       lcd.print("> pos      ");
       Serial.print("<->h, position ");
       sendTextToBT("<->h, position ");
       encoderPos++;
       if ( encoderPos > ( nbPas - 1 ) ) {
         encoderPos = 0;
       }
     }
     lcd.setCursor(7,1);
     lcd.print(encoderPos);
     Serial.print (encoderPos);
     char str[10];
     sprintf(str, "%d", encoderPos);
     sendTextToBT(str);
   } 
   PinCLKLast = n;
*/

}

char * readTextFromBT(){
  //wait some time
  delay(100);
  
  char reply[16];
  int i = 0;
  while (BTserial.available()) {
    reply[i] = BTserial.read();
    i += 1;
  }
  
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.print(reply);
  
  //end the string
  reply[i] = '\0';
  Serial.println("Read Text from BT:");
  Serial.print(reply);
}

void sendTextToBT(const char * text){
  Serial.print("text send :");
  Serial.println(text);
  BTserial.print("#");
  BTserial.print(text);
  BTserial.println("~");
}

void displayAccelerationStatus() {
  lcd.setCursor(0, 1);  
  lcd.print("Acceleration ");
  lcd.print(encoder->getAccelerationEnabled() ? "on " : "off");
}


void sendCommand(const char * command){
  Serial.print("Command send :");
  Serial.println(command);
  BTserial.println(command);
  //wait some time
  delay(100);
  
  char reply[100];
  int i = 0;
  while (BTserial.available()) {
    reply[i] = BTserial.read();
    i += 1;
  }
  //end the string
  reply[i] = '\0';
  Serial.print(reply);
  Serial.println("Reply end");
}

