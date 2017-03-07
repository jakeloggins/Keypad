
// NOTE: This example code also uses the Adafruit_PWMServoDriver library, which was not included in this fork. Look at the keypad setup section to get a sense of how it works.

// Sketch basically does 4 things:
// Save the keys as you enter.
// Once you've entered a series of numbers on the keypad, press * to "open" or # to "close" the servo connected to that servo driver pin.
// Press 'A' to clear all the key presses
// After 10 seconds of no key presses, the saved keys are cleared.


// -- keypad setup
  // input expander
  #include <Keypad.h>

  String msg;

  // This is just like all the other arduino playground examples ...

  const byte ROWS = 4; //four rows
  const byte COLS = 4; //three columns
  char keys[ROWS][COLS] = {
    {'1','2','3', 'A'},
    {'4','5','6', 'B'},
    {'7','8','9', 'C'},
    {'*','0','#', 'D'}
  };


  // ... but these pin references now magically refer to the MCP23017 rather than GPIO.

  byte rowPins[ROWS] = {0, 1, 2, 3}; //connect to the row pinouts of the keypad to the mcp2017 pins (GPA 0,1,2,3 ~ pins 21, 22, 23, 24)
  byte colPins[COLS] = {4, 5, 6, 7}; //connect to the column pinouts of the keypad to the mcp2017 pins (GPA 4,5,6,7 ~ pins 25, 26, 27, 28)




  Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

  String savedKeys;
  int selectedServo = 0;


  void saveKey(char justPressed) {
    savedKeys += (char)justPressed;
  }

  void clearKeys() {
    savedKeys = "";
  }

  void storedKeys() {
    selectedServo = savedKeys.toInt();
  }


// -- Servo setup

  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>

  // called this way, it uses the default address 0x40
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
  // you can also call it with a different address you want
  //Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);  

  #define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
  #define SERVOMAX  450 // this is the 'maximum' pulse length count (out of 4096)


  uint16_t pulselen;
  bool servoFlag = false;

  int selectedPos;
  int selectedOpen = 175; // should be calibrated first
  int selectedClose = 5; // should be calibrated first
  static uint32_t lastMove = 0;
  static uint32_t moveLimit = 250;

  int selectedFanSpeed = 90;


  void servoRun() {

    if (millis() - lastMove > moveLimit) {
      lastMove = millis();
     
      yield();
      // convert to PWM using map function
      pulselen = map(selectedPos, 0, 180, SERVOMIN, SERVOMAX);
      Serial.printf("selected servo .. %d \n", selectedServo);
      Serial.printf("selected pos .. %d \n", selectedPos);
      Serial.printf("pulse .. %d \n", pulselen);
      // set PWM
      pwm.setPWM(selectedServo, 0, pulselen);

    }
    
  }


void servoOpen() {
  selectedPos = selectedOpen;
  storedKeys();
  servoRun();
  clearKeys();

}

void servoClose() {
  selectedPos = selectedClose;
  storedKeys();
  servoRun();
  clearKeys();
}


// waiting loops
  static uint32_t tick = 0;
  static uint32_t tickLimit = 30000;

  static uint32_t keypress = 0;
  static uint32_t keypressLimit = 10000;



void setup() {

  Serial.begin(115200);
  Serial.println("Booting");
  Serial.println(ESP.getResetInfo());

  pwm.begin();
  pwm.setPWMFreq(50);  
}


void loop() {

  if (millis() - keypress > keypressLimit) {
    Serial.println("key press timeout");
    clearKeys();
    keypress = millis();
  }
  
  if (keypad.getKeys()) {
    // reset keypress time so timeout doesn't clear
    keypress = millis();

      for (int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
      {
          if ( keypad.key[i].stateChanged )   // Only find keys that have changed state.
          {
              switch (keypad.key[i].kstate) {  // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
                  case PRESSED:
                  msg = " PRESSED.";


                  switch (keypad.key[i].kchar) {
                    case 'A':
                      Serial.println("clear all saved keys ...");
                      Serial.println(savedKeys);
                      clearKeys();
                      Serial.println(savedKeys);
                      break;

                    case '*':
                      Serial.println("opening ...");
                      Serial.println(savedKeys);
                      servoOpen();
                      Serial.println("cleared ...");
                      Serial.println(savedKeys);  
                      break;

                    case '#':
                      Serial.println("closing ...");
                      Serial.println(savedKeys);
                      servoClose();
                      Serial.println("cleared ...");
                      Serial.println(savedKeys);     
                      break;


                    default:
                      Serial.println("saved key");
                      Serial.println(keypad.key[i].kchar);
                      saveKey(keypad.key[i].kchar);
                      Serial.println(savedKeys);
                      break;

                  }
                  break;
                  
                  case HOLD:
                  msg = " HOLD.";
                  break;
                  
                  case RELEASED:
                  msg = " RELEASED.";
                  break;
                  
                  case IDLE:
                  msg = " IDLE.";
              }
              //Serial.print("Key ");
              //Serial.print(keypad.key[i].kchar);
              //Serial.println(msg);
          }
      }
  }
  

  // Do things every tickLimit seconds
  
  if ( millis() - tick > tickLimit) {
    

    tick = millis();
    /*    
    // i2c scan
    byte error, address;
    int nDevices;
   
    Serial.println("Scanning...");
   
    nDevices = 0;
    for(address = 1; address < 127; address++ ) 
    {
   
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
   
      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address<16) 
          Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
   
        nDevices++;
      }
      else if (error==4) 
      {
        Serial.print("Unknow error at address 0x");
        if (address<16) 
          Serial.print("0");
        Serial.println(address,HEX);
      }    
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");
    */


  }
  

  yield();

}