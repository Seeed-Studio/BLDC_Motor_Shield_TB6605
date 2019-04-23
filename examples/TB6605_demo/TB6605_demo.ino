/*  BurstRead_demo.ino
/*
 *  
 * Copyright (c) 2019 Seeed Technology Co., Ltd.
 * Website    : www.seeed.cc
 * Create Time: March 2019
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
  User can do following thing by pressing key on board.
    1. Start,stop or brake motor
    2. Change the direction of motor rotation (CW or CCW)

  User can understand current status by reading info display on LCD.
    1. Target speed (tuned by potentiometer)
    2. Current speed detected by HP
    3. Motor status: start, stop or brake
    4. The direction of motor rotation (CW/CCW)
    5. Kp value
*/

#include <rgb_lcd.h>
#include <TimerOne.h>


rgb_lcd lcd;

// digital pins
#define PORT_HP 2                      // digital pin 2 used to check hall pulse from MCD
#define PORT_PWMIN 3                   // digital pin 3 used to output PWM-in signal to MCD as speed control
#define PORT_DIR_CTR 4                 // digital pin 4 used to write CW/CCW switch state from MCD board
#define PORT_DIR 5                     // digital pin 5 used to read CW/CCW switch state from MCD board
#define PORT_START 7                   // digital pin 7 used to read Start switch state from MCD board
#define PORT_BRAKE 8                   // digital pin 8 used to read Brake switch state from MCD board

// analog pins
#define PORT_VR 1                      // analog pin 1 used to connect the potentiometer as target speed input

#define READY 0                        // motor stop state
#define ACTIVE 1                       // motor rotating state

byte mode;                             // motor work state, READY or ACTIVE
volatile long counterVal[2] = {0, 0};  // hall pulse number value to calculate actual rotation speed
byte curCnt = 0;                       // counter to read hall pulse number
bool rotateDir;                        // state of CW/CCW switch
bool rotateStart;                      // state of Start switch
bool rotateBrake;                      // state of Brake switch
int rotateSpeedSetHp;                  // hall pulse number in 150ms against target rotation speed
int rotateSpeedPwm;                    // PWM output to PWM-in pin as a speed command
volatile int hp_freq;                  // hall pulse number counted in 150ms
volatile bool flagReadVR = false;      // flag to read potentiometer value
int oldVRval = 0;                      // potentiometer value read at last time
volatile bool flagReadHP = false;      // flag to read hall pulse number
byte readHPCnt = 0;                    // the timer counter to read hall pulse number

int err1 = 0;                          // error calculated at last time used for PID
int Kp = 4, Ki = 2;                    // pid control parameter, user can modify it for actual used motor

const int hp2Speed = 100;              // rotation speed(per min) = hp_150ms*(1000ms/150ms)*60s/4 = hp_150ms * 100, paired pole=8/2=4
const int hpMax = 4000 / hp2Speed;     // max rotation speed: 4000RPM, hpMax(150ms): 40
const int vrMax = 245;                 // max VR: 252, 5V -> 1023, 1.16V -> vrMax

void counter();
int get_key(int input);
void timer1_Isr();
void PID_Control(int detectedHp);

/**************************************************************************************
   setup()
   the function is called when a sketch starts. Use it to initialize
   variables, pin modes, start using libraries, etc.
   The function will only run once, after each powerup or reset of the Arduino board.
**************************************************************************************/
void setup() {
  // initialize port for motor control
  pinMode(PORT_START, INPUT);
  pinMode(PORT_BRAKE, INPUT);
  pinMode(PORT_HP, INPUT);
  pinMode(PORT_DIR, INPUT);
  pinMode(PORT_DIR_CTR,OUTPUT);
  
  // fast PWM for D3(PWMIN) using timer2
  TCCR2B = TCCR2B & 0xF8 | 0x01;        // clock prescale by 1, PWM frequency: 32KHz

  // read potentiometer value to set target rotation speed
  int value = analogRead(PORT_VR);
  rotateSpeedSetHp = map(value, 0, vrMax, 0, hpMax); // convert the value to hall pulse number in 150ms (between 0 and hpMax)
  oldVRval = value;
  int rotateSpeedSet = rotateSpeedSetHp * hp2Speed;
  rotateSpeedPwm = 0; 

  // read current state of motor control switches on MCD board
  rotateDir = digitalRead(PORT_DIR);
  rotateStart = digitalRead(PORT_START);
  rotateBrake = digitalRead(PORT_BRAKE);
  mode = READY;

  lcd.begin(16,2);
  lcd.setRGB(255,255,255);

  lcd.setCursor(0, 0);
  lcd.print("T ");
  lcd.print(rotateSpeedSet);
  lcd.setCursor(6,0);
  lcd.print("RPM");
  lcd.setCursor(6,1);
  lcd.print("RPM");
  lcd.setCursor(11, 0);
  if (rotateDir == HIGH)
    lcd.print("CCW");
  else
    lcd.print("CW ");

  lcd.setCursor(0, 1);
  lcd.print("C 0   ");
  lcd.setCursor(11, 1);
  if (rotateStart == HIGH)
    lcd.print("P ");
  else
    lcd.print("S ");
  if (rotateBrake == HIGH)
    lcd.print("  ");
  else
    lcd.print("B ");
  lcd.print(Kp);

  Timer1.initialize(25000); // initialize timer1 with 25ms period
  Timer1.attachInterrupt( timer1_Isr ); // enable timer1 interrupt and associate interrupt service routine
  Timer1.start();

  
  Serial.begin(9600);     // for debug
  Serial.println("== Input Kp:(1-9):");  // Input for Kp
}

/**************************************************************************************
  loop()
  the function loops consecutively,Use it to actively control the Arduino board.
  this loop() constantly checks change of motor control switches, read potentiometer
  value as target rotation speed and get hall pulse frequecncy as actual rotation speed.
  Then achieve target speed with PI control method.
**************************************************************************************/
void loop()
{
  static bool flagFirstMotor = false;  // flag of start motor rotating
  bool switchState;
 
  // check if the state of Brake switch is changed
  switchState = digitalRead(PORT_BRAKE);
  if (switchState != rotateBrake)
  {
    rotateBrake = switchState;
    if (mode == READY)  // motor is not rotating
    {
      lcd.setCursor(13, 1);
      if (rotateBrake == HIGH)
        lcd.print(" ");
      else
        lcd.print("B");
    }
    else
    { // mode = ACTIVE, motor is rotating
      if (rotateBrake == LOW)
      { //brake motor
        analogWrite(PORT_PWMIN, 255); // output High to stop output to motor
        detachInterrupt(digitalPinToInterrupt(PORT_HP)); // disable interrupt to counter HP

        lcd.setCursor(2, 1);
        lcd.print("0   ");
        lcd.setCursor(13, 1);
        lcd.print("B");
      }
      else
      { //restart
        counterVal[0] = 0;
        counterVal[1] = 0;
        curCnt = 0;
        readHPCnt = 0;
        flagReadHP = false;
        rotateSpeedPwm = 0;
        err1 = 0;
        flagFirstMotor = true;
        attachInterrupt(digitalPinToInterrupt(PORT_HP), counter, RISING); // enable interrupt by rising edge of HP signal to count HP

        lcd.setCursor(2, 1);
        lcd.print("0   ");
        lcd.setCursor(13, 1);
        lcd.print(" ");
      }
    }
  }

  // check if the state of Start switch is changed
  switchState = digitalRead(PORT_START);
  if (switchState != rotateStart)
  {
    rotateStart = switchState;
    if (mode == READY)
    {
      if (rotateStart == LOW)
      {
        mode = ACTIVE;
        if (rotateBrake == HIGH)
        {
          //start motor
          counterVal[0] = 0;
          counterVal[1] = 0;
          curCnt = 0;
          readHPCnt = 0;
          flagReadHP = false;
          rotateSpeedPwm = 0;
          err1 = 0;
          flagFirstMotor = true;
          attachInterrupt(digitalPinToInterrupt(PORT_HP), counter, RISING);  // enable interrupt by rising edge of HP signal to count HP
        }

        lcd.setCursor(11, 1);
        lcd.print("S");
      }
    }
    else
    { //active
      if (switchState == HIGH)
      {
        //stop motor
        mode = READY;
        analogWrite(PORT_PWMIN, 255);    // output High to stop output to motor
        detachInterrupt(digitalPinToInterrupt(PORT_HP)); // disable interrupt to count HP

        lcd.setCursor(2, 1);
        lcd.print("0   ");
        lcd.setCursor(11, 1);
        lcd.print("P");

        Serial.println("== Input Kp:(1-9):");
      }
    }
  }

  // check if the state of CW/CCW switch is changed
  switchState = digitalRead(PORT_DIR);
  if (switchState != rotateDir)
  {
    rotateDir = switchState;
    lcd.setCursor(11, 0);
    if (rotateDir == HIGH) {
      lcd.print("CCW");
      for(int i = 200; i < 256; i++) {
        analogWrite(PORT_PWMIN, i);
        delay(10);
      }
      digitalWrite(PORT_DIR_CTR,HIGH);
      
    }
    else {
      lcd.print("CW ");
      for(int i = 200; i < 256; i++) {
        analogWrite(PORT_PWMIN, i);
        delay(10);
      }
      digitalWrite(PORT_DIR_CTR,LOW);
     
    }
      
  }

  // check if target speed is changed
  if (flagReadVR == true)
  {
    // read potentiometer value to set target rotation speed
    int value = analogRead(PORT_VR);
    if(value > vrMax) 
    {
      value = vrMax;     // If the speed exceeds 4000, the motor will be damaged
    }

    if ( (value > oldVRval + 5) || (value < oldVRval - 5) )
    {
      rotateSpeedSetHp = map(value, 0, vrMax, 0, hpMax); // convert the value to hall pulse number in 150ms (between 0 and hpMax)
      int rotateSpeedSet = rotateSpeedSetHp * hp2Speed; // convert to rotation speed for LCD display
      
      if(rotateSpeedSet < 1000) {
        lcd.setCursor(2, 0);
        lcd.print(rotateSpeedSet);
        lcd.print(" ");
      } 
      else {
        lcd.setCursor(2, 0);
        lcd.print(rotateSpeedSet);
      }
      
      
      
      oldVRval = value;

      Serial.print("target speed: ");
      Serial.println(rotateSpeedSet);
    }
    flagReadVR = false;
  }

  // check current motor rotation speed
  if (mode == ACTIVE && rotateBrake == HIGH)
  {
    if (flagReadHP == true)
    {
      flagReadHP = false;
      int speed_val = hp_freq * hp2Speed;

      Serial.print("Hall pulse in 150ms: speed/100  ------- ");
      Serial.println(hp_freq);

      
      if (flagFirstMotor == true )   // ignore speed detected at first time which is not valid value.
        flagFirstMotor = false;
      else
      {
        if(speed_val < 1000) {
          lcd.setCursor(2, 1);
          lcd.print(speed_val);
          lcd.print(" ");
        }
        else {
          lcd.setCursor(2, 1);
          lcd.print(speed_val);
        }
        
        PID_Control(hp_freq);     // adjust PWM output to achive target speed based on current speed
      }
    }
  }

  // check if Kp value is changed by input in serial monitor
  if (mode != ACTIVE)
  {
    if ( Serial.available() )     // check if there is input in serial monitor
    {
      int x = Serial.parseInt(); // look for the next valid integer in the incoming serial stream
      if ( x < 10 && x > 0)
      {
        Kp = x;                   // apply new Kp for PID control
        Serial.print("Kp: ");
        Serial.println(x, DEC);
        lcd.setCursor(15, 1);
        lcd.print(Kp);
      }
      else
        Serial.println("Invalid value!");
    }
  }

  
}

/**************************************************************************************
  counter()
  Interrupt service routine of external interrupt triggered by rising edge of HP signal
**************************************************************************************/
void counter()
{
  counterVal[curCnt & 0x01] += 1;
}

/**************************************************************************************
  timer1_Isr()
  Timer1 interrupt servie routine
  Read potentiometer value and check hall pulse number at specified timing
**************************************************************************************/
void timer1_Isr()
{
  static byte readVRCnt = 0;   // the timer counter to read potentiometer
  
  if (readVRCnt == 4)         // period: 25ms*5 = 125ms
  {
    flagReadVR = true;        // set flag to tell loop() to read potentiometer value
    readVRCnt = 0;
  }
  else
  {
    readVRCnt++;
  }

  if (mode == ACTIVE && rotateBrake == HIGH)
  {
    if (readHPCnt == 5)        // period: 25ms*6 = 150ms
    {
      flagReadHP = true;       // set flag to tell loop() to read HP
      curCnt++;
      readHPCnt = 0;

      if ((curCnt & 0x01) == 0)
      {
        hp_freq = counterVal[1];
        counterVal[1] = 0;     // clear for new data
      }
      else
      {
        hp_freq = counterVal[0];
        counterVal[0] = 0;     // clear for new data
      }
    }
    else
    {
      readHPCnt++;
    }
  }
  
}

/**************************************************************************************
  PID_Control()
  Incremental PID control based on current speed detected by Hall sensor
  int detectedHp: hall pulse number in 150ms
**************************************************************************************/
const int pidOver1_U = 4;  // upper limit value
const int pidOver1_L = -4; // lower limit value
const int pidOver2_U = 2;  // upper limit value in case error is small
const int pidOver2_L = -2; // lower limit value in case error is small
int Kp_s = 4;  // Kp value during starting motor, user can modify it for actual used motor
void PID_Control( int detectedHp )
{
  int err;
  int bias;
  int pidOver_U;
  int pidOver_L;

  err = rotateSpeedSetHp - detectedHp;     // error = set value - actual value

  Serial.print("hp e: ");
  Serial.println(err);

  if (detectedHp == 0) {
    bias = Kp_s * (err - err1);            // do only when starting motor rotation and speed is zero
  }
  else
  {
    bias =  Kp * (err - err1) + Ki * err;  // do always when motor is rotating and speed is not zero
    Serial.print("bias: ");
    Serial.println(bias);

    if (err < 2 && err > -2)
    {
      pidOver_U = pidOver2_U;
      pidOver_L = pidOver2_L;
    }
    else
    {
      pidOver_U = pidOver1_U;
      pidOver_L = pidOver1_L;
    }

    if (bias > pidOver_U)
      bias = pidOver_U;
    else if (bias < pidOver_L)
      bias = pidOver_L;
  }

  Serial.print("set bias: ");
  Serial.println(bias);

  rotateSpeedPwm += bias;
  err1 = err;

  Serial.print("set PWM value: ");
  Serial.println(rotateSpeedPwm, DEC);

  if ( rotateSpeedPwm > 255)           // PWM value between 0 and 255
    rotateSpeedPwm = 255;
  if (rotateSpeedPwm < 1)
    rotateSpeedPwm = 1;

  int pwmIn = 0xFF - rotateSpeedPwm;   // low active
  Serial.println(pwmIn, DEC);
  analogWrite(PORT_PWMIN, pwmIn);      // change PWM duty to change rotation speed
}
