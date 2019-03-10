/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <JC_Button.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  300 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  470 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 1;

const int analogInPin = A2;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

int sequCounter = 0;
int servoSequId[100];
int servoSequVal[100];
const unsigned long LONG_PRESS(1000);

static bool adjustSelectState;
static bool programPlayState;

const byte BUTTON_PIN(2), BUTTON2_PIN(3); // connect a button switch from this pin to ground
Button setServo(BUTTON_PIN); // define the button
Button program(BUTTON2_PIN); // define the button

void setup() {
  Serial.begin(9600);
  Serial.println("RoboCat Initialized!");
  setServo.begin();              // initialize the button object
  program.begin();              // initialize the button object

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  pinMode(12, OUTPUT); // programming indicator led

  // reset joints to vertical
  pwm.setPWM(1, 0, 363);
  pwm.setPWM(2, 0, 359);
  pwm.setPWM(3, 0, 335);
  pwm.setPWM(4, 0, 362);
  pwm.setPWM(5, 0, 310);
  pwm.setPWM(6, 0, 340);
  pwm.setPWM(7, 0, 350);
  pwm.setPWM(8, 0, 322);

  delay(10);
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);

  setServo.read();               // read the button
  program.read();               // read the button

  if (setServo.wasReleased())   
  {
    adjustSelectState = !adjustSelectState;
      Serial.println("setServo setServo setServo setServo setServo setServo setServo setServo setServo");
      Serial.println(sequCounter);
      Serial.println(servonum);
      Serial.println(outputValue);
      Serial.println(programPlayState);
      Serial.println("----------------------------1");
  }

  if(adjustSelectState){
    sensorValue = analogRead(analogInPin);
    outputValue = map(sensorValue, 0, 1023, 1, 8);
    servonum = outputValue;
    Serial.println(servonum);
  }else{
    // read the analog in value:
    sensorValue = analogRead(analogInPin);
    // map it to the range of the analog out:
    outputValue = map(sensorValue, 0, 1023, 175, 540);
    pwm.setPWM(servonum, 0, outputValue); // set servo position
  }

    if (program.releasedFor(LONG_PRESS)){
      Serial.println("long press long press long press long press long press long press long press ");
      programPlayState = !programPlayState;
    } else if (program.wasReleased()){
      digitalWrite(12, HIGH);   
      delay(1000);              
      digitalWrite(12, LOW);   
      delay(1000);

      
      if(sequCounter > 99){
        sequCounter = 1;
      }

      servoSequId[sequCounter] = servonum;
      servoSequVal[sequCounter] = outputValue;
      

      Serial.println("program program program program program program program program program program ");
      Serial.println(servonum);
      Serial.println(outputValue);
      Serial.println(servoSequId[sequCounter]);
      Serial.println(servoSequVal[sequCounter]);
      Serial.println("----------------------------2");

      sequCounter = sequCounter + 1; // increment counter after reporting recorded value
    }  

    if(programPlayState == 1){
      digitalWrite(12, HIGH);   
      delay(500); 
      servonum = 1; // reset the servo counter

      // play the program for each programmed step
      int walkLeg[]={5,3,7,1,5,3,7,1};
  //pwm.setPWM(1, 0, 363);
  //pwm.setPWM(2, 0, 359);
  //pwm.setPWM(3, 0, 335);
  //pwm.setPWM(4, 0, 362);
  //pwm.setPWM(5, 0, 310);
  //pwm.setPWM(6, 0, 340);
  //pwm.setPWM(7, 0, 350);
  //pwm.setPWM(8, 0, 322);
      int walkMove[]={288,305,385,390,311,360,369,373};
      int walkStepCount = 8;
      for (int i=0; i < walkStepCount; i++){
        pwm.setPWM(walkLeg[i], 0, walkMove[i]);
        //pwm.setPWM(servoSequId[i], 0, servoSequVal[i]);
        delay(300); 
       
        Serial.println(i);
        Serial.println(servoSequId[i]);
        Serial.println(servoSequVal[i]);
        Serial.println("----------------------------3");
        delay(10);
     }
    }else{
      digitalWrite(12, LOW);   
      delay(500); 
    }

}
