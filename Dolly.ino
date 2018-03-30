#include <SoftwareSerial.h>
#include <Wire.h>
#include <TimerOne.h>

//Include menu definition
#include "Menu.h"

enum EncoderMove {ENCODER_TOP, ENCODER_BOTTOM, ENCODER_NONE};
/*
static const char *MAIN_MENU_ITEMS[] = {
    "Start",
    "Mode",
    "Configuration"};

static const char *MODE_MENU_ITEMS[] = {
    "Duration",
    "Distance",
    "Manual"};

static const char *CONFIGURATION_MENU_ITEMS[] = {
    "Duration",
    "Distance",
    "Offset time"};

   const MenuType menutype;
  const struct MenuItem **p;
  const char* menuPrompt;
  const void *callbackFnct;
*/
// Custom character for LCD.
byte cursor[8] = {
  0b10000,
  0b11000,
  0b11100,
  0b11110,
  0b11100,
  0b11000,
  0b10000,
  0b00000
};

byte arrowDown[8] = {
  0b00000,
  0b00000,
  0b00100,
  0b00100,
  0b00100,
  0b11111,
  0b01110,
  0b00100
};

byte arrowUp[8] = {
  0b00100,
  0b01110,
  0b11111,
  0b00100,
  0b00100,
  0b00100,
  0b00000,
  0b00000
};

static MenuItem FAKE_ITEM_1 = {
    MENU_TYPE_ITEM,
    NULL,
    0,
    "Fake-1",
    NULL};

static MenuItem FAKE_ITEM_2 = {
    MENU_TYPE_ITEM,
    NULL,
    0,
    "Fake-2",
    NULL};

static MenuItem FAKE_MENU_ITEMS[] = {FAKE_ITEM_1, FAKE_ITEM_2};


static MenuItem FAKE_MENU = {
    MENU_TYPE_MENU,
    FAKE_MENU_ITEMS,
    2,
    "Fake",
    NULL};
    
static MenuItem START_MENU = {
    MENU_TYPE_MENU,
    NULL,
    0,
    "Start",
    NULL};
    
static MenuItem MODE_MENU = {
    MENU_TYPE_MENU,
    NULL,
    0,
    "Mode",
    NULL};

static MenuItem CONFIGURATION_MENU = {
    MENU_TYPE_MENU,
    NULL,
    0,
    "Configuration",
    NULL};

static MenuItem MAIN_MENU_ITEMS[] = {
    START_MENU, MODE_MENU, FAKE_MENU, CONFIGURATION_MENU
};

static MenuItem MAIN_MENU = {
    MENU_TYPE_MENU,
    MAIN_MENU_ITEMS,
    4,
    "Main",
    NULL};


// include the library code:
#include <LiquidCrystal_I2C.h>

//Motor Shield
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//ClickEncoder
#include "ClickEncoder.h"

//I2C ADRESSE FIND WITH I2C_SCANNER : http://playground.arduino.cc/Main/I2cScanner#.UxJJG_0xJFI

LiquidCrystal_I2C lcd(0x3F, 16, 2);

SoftwareSerial BTserial(8, 9); // RX | TX

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *dcMotor = AFMS.getMotor(3);

char *MenueLine[] = {" Speed", " Duration", " Direction"};

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
int motorRelease = 0;

char c = ' ';
boolean motorRun = false;

//Rotary Selector
#define ENCODER_PIN_CLK 11
#define ENCODER_PIN_DT 12
#define ENCODER_PIN_SW 13

ClickEncoder *encoder;
int16_t last, value;

void timerIsr()
{
  encoder->service();
}

boolean ClickEncoderHeld = false;

MenuLevel menuLevel[3];

void setup()
{
  menuLevel[0].menu = MAIN_MENU;
  
  //Init Shield Motor
  AFMS.begin();

  //New Encoder
  encoder = new ClickEncoder(ENCODER_PIN_DT, ENCODER_PIN_CLK, ENCODER_PIN_SW, 4);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  last = -1;

  //LCD
  lcd.init();
  lcd.cursor_on();
  lcd.blink_on();
  lcd.backlight();
  
  // Create the custom character.
  lcd.createChar(0, cursor);
  lcd.createChar(1, arrowDown);
  lcd.createChar(2, arrowUp);
  
  lcd.setCursor(0, 0);
  lcd.print("Dolly booting !");
  //lcd.print(MAIN_MENU.menuPrompt);
  

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
  lcd.setCursor(0, 0);
  lcd.print("Dolly is alive !");
  delay(1000);
  lcd.clear();
  lcd.cursor_off();
  lcd.blink_off();
}

void loop()
{
  displayMenu();

/*
  if(!motoRun) {
    //displayMenu();
  } else {
    //displayRun();
  }
  */

/*
  value += encoder->getValue();

  if (value < 1)
  {
    value = 1;
  }
  if (value > 250)
  {
    value = 250;
  }

  dcMotor->setSpeed(value);

  if (value != last)
  {
    last = value;
    Serial.print("Encoder Value: ");
    Serial.println(value);

    lcd.setCursor(0, 0);
    lcd.print("         ");
    lcd.setCursor(0, 0);
    lcd.print(value);
  }

  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open)
  {
#define VERBOSECASE(label)  \
  case label:               \
    Serial.println(#label); \
    break;
    switch (b)
    {
    //VERBOSECASE(ClickEncoder::Pressed);
    //VERBOSECASE(ClickEncoder::Held)
    //VERBOSECASE(ClickEncoder::Released)
    //VERBOSECASE(ClickEncoder::Clicked)
    //VERBOSECASE(ClickEncoder::DoubleClicked)
    case ClickEncoder::Held:
      if (!ClickEncoderHeld)
      {
        ClickEncoderHeld = true;
        Serial.println("ClickEncoder::Held");
        motorRun = !motorRun;
        if (motorRun)
        {
          Serial.println("Motor::RUN");
        }
        else
        {
          Serial.println("Motor::STOP");
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
*/

  /*
  if (motorRun)
  {
    dcMotor->run(FORWARD);
  }
  else
  {
    dcMotor->run(RELEASE);
  }
  */

  // Keep reading from AT-09 and send to Arduino Serial Monitor
  /*
  if (BTserial.available())
  {
    c = BTserial.read();
    Serial.write(c);
    //lcd.setCursor(0,0);
    //lcd.print(c);
  }
  */

  // Keep reading from Arduino Serial Monitor and send to HC-05
  /*
 if (Serial.available())
  {
    c = Serial.read();
    BTserial.write(c);
  }
  */
}

//Display Menu
//int currentMenuPos = 1;
//int firstItemDisplayed = 1;

int currentMenuLevel = 0;

void menuCheckButton() {
  
  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open)
  {
    switch (b)
    {
    //VERBOSECASE(ClickEncoder::Pressed);
    //VERBOSECASE(ClickEncoder::Held)
    //VERBOSECASE(ClickEncoder::Released)
    //VERBOSECASE(ClickEncoder::Clicked)
    //VERBOSECASE(ClickEncoder::DoubleClicked)
    /*
    case ClickEncoder::Held:
      if (!ClickEncoderHeld)
      {
        ClickEncoderHeld = true;
        Serial.println("ClickEncoder::Held");
        motorRun = !motorRun;
        if (motorRun)
        {
          Serial.println("Motor::RUN");
        }
        else
        {
          Serial.println("Motor::STOP");
        }
      }
      break;
    */
    case ClickEncoder::DoubleClicked:
      if(currentMenuLevel > 0) {
        currentMenuLevel = currentMenuLevel - 1;
        lcd.clear();
      }
      break;
    case ClickEncoder::Clicked:
      MenuItem currentMenu = menuLevel[currentMenuLevel].menu;
      int currentMenuPos = menuLevel[currentMenuLevel].currentMenuPos;
      
      if(MENU_TYPE_MENU == (currentMenu.menuItems)[currentMenuPos].menutype) {
         currentMenuLevel = currentMenuLevel + 1;
         //menuLevel[currentMenuLevel] = 
         menuLevel[currentMenuLevel].menu = ((currentMenu.menuItems)[currentMenuPos-1]);
         menuLevel[currentMenuLevel].currentMenuPos = 1;
         menuLevel[currentMenuLevel].firstItemDisplayed = 1;
         lcd.clear();
      }

      break;
    }
  }
}

void displayMenu()
{
  
  EncoderMove movement = ENCODER_NONE;
  int valueMenu = encoder->getValue();
  
  if(valueMenu > 0) {
    movement = ENCODER_TOP;
  } else if (valueMenu <0) {
    movement = ENCODER_BOTTOM;
  }

  //Get current information
  MenuItem currentMenu = menuLevel[currentMenuLevel].menu;
  int currentMenuPos = menuLevel[currentMenuLevel].currentMenuPos;
  int nextMenuPos = currentMenuPos;
  int firstItemDisplayed = menuLevel[currentMenuLevel].firstItemDisplayed;
  
  int menuItemsLenght = currentMenu.menuItemsLenght;
  
  lcd.setCursor(13, 1);
  lcd.print(currentMenu.menuItemsLenght);

  if (ENCODER_BOTTOM == movement ) {
    nextMenuPos = nextMenuPos + 1;

    if(nextMenuPos > menuItemsLenght) {
      nextMenuPos = menuItemsLenght;
    }
  } else if (ENCODER_TOP == movement) {
    nextMenuPos = nextMenuPos - 1;

    if(nextMenuPos < 1) {
      nextMenuPos = 1;
    }
  }

  if(currentMenuPos != nextMenuPos) {
    currentMenuPos = nextMenuPos;
    lcd.clear();
  }

  //Compute first item displayed
  if (ENCODER_BOTTOM == movement ) {
    if (1 == currentMenuPos) {
      firstItemDisplayed = 1;
    } else {
      firstItemDisplayed = currentMenuPos - 1;
    }
  } else if (ENCODER_TOP == movement) {
    if (menuItemsLenght == currentMenuPos) {
      firstItemDisplayed = currentMenuPos - 1;
    } else {
      firstItemDisplayed = currentMenuPos;
    }
  }

  //have to subscract 1 from firstItemDisplayed, First LCD index is 0
  lcd.setCursor(1, 0);
  lcd.print((currentMenu.menuItems)[firstItemDisplayed-1].menuPrompt);
  lcd.setCursor(1, 1);
  lcd.print((currentMenu.menuItems)[firstItemDisplayed].menuPrompt);

  //Display arrow down if necessary
  if(firstItemDisplayed != menuItemsLenght-1){
    lcd.setCursor(15, 1);
    lcd.write(byte(1)); //Bottom
  }

  //Display arrow up if necessary
  if(menuItemsLenght>2 && firstItemDisplayed > 1){
    lcd.setCursor(15, 0);
    lcd.write(byte(2)); //TOP
  }
  
  //Display current item selected
  lcd.setCursor(0, currentMenuPos-firstItemDisplayed);
  lcd.write(byte(0));

  //Save current pos and displayItem in current menu.
  menuLevel[currentMenuLevel].currentMenuPos = currentMenuPos;
  menuLevel[currentMenuLevel].firstItemDisplayed = firstItemDisplayed;

  menuCheckButton();
}

/*

#define VERBOSECASE(label)  \
  case label:               \
    Serial.println(#label); \
    break;
 
 */


char *readTextFromBT()
{
  //wait some time
  delay(100);

  char reply[16];
  int i = 0;
  while (BTserial.available())
  {
    reply[i] = BTserial.read();
    i += 1;
  }

  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.print(reply);

  //end the string
  reply[i] = '\0';
  Serial.println("Read Text from BT:");
  Serial.print(reply);
}

void sendTextToBT(const char *text)
{
  Serial.print("text send :");
  Serial.println(text);
  BTserial.print("#");
  BTserial.print(text);
  BTserial.println("~");
}

void displayAccelerationStatus()
{
  lcd.setCursor(0, 1);
  lcd.print("Acceleration ");
  lcd.print(encoder->getAccelerationEnabled() ? "on " : "off");
}

void sendCommand(const char *command)
{
  Serial.print("Command send :");
  Serial.println(command);
  BTserial.println(command);
  //wait some time
  delay(100);

  char reply[100];
  int i = 0;
  while (BTserial.available())
  {
    reply[i] = BTserial.read();
    i += 1;
  }
  //end the string
  reply[i] = '\0';
  Serial.print(reply);
  Serial.println("Reply end");
}
