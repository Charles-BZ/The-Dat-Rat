#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7
float sensorMins[8] = {828, 712, 734, 689, 666, 732, 758, 757.2};
float sensorMax[8] = {1672, 1747, 1766, 1074.6, 1187.6, 1768, 1661.6, 1742.8};
float sensorWeights1[8] = {-15, -14, -12, -8, 8, 12, 14, 15};
float sensorWeights2[8] = {-16, -8, -4, -2, 2, 4, 8, 16};
float error = 0;
float pastError = 0;

float Kp = 0.04;
float Kd = 0.3;
//works for turn: 0.09, 0.2
//turn: 0.1, 0.3, 70 (8/4)
//rly good turn: 0.05, 0.3, 70 (15/14)
//only turns if straight: 0.04, 0.35, 80 (15/14)

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;

int leftSpd = 30;
int rightSpd = 30;

float leftOutput = 0;
float rightOutput = 0;

int turnSpeed = 100;

bool donut = false;
bool hasGlazed = false;

bool hasTurned1 = false;
bool Turn1 = false;

bool hasOffset = false;
bool Offset = false;

bool hasTurned2 = false;
bool Turn2 = false;

bool straight2 = false;
bool s2 = false;

bool donut2 = false;

int weightingSystem = 1;

float sensorFusion(float sensorWeights[]) {
  float temp_error = 0;
  float temp_vals[8];
  for (unsigned char i = 0; i < 8; i++)
  {
    temp_vals[i] = sensorValues[i];
    temp_vals[i] -= sensorMins[i];
    temp_vals[i] *= (1000 / sensorMax[i]);
    temp_vals[i] *= sensorWeights[i];
    temp_error += temp_vals[i];
  }
  return temp_error / 8;
}

void  ChangeBaseSpeeds(int initialLeftSpd,int finalLeftSpd,int initialRightSpd,int finalRightSpd) {
/*  
 *   This function changes the car speed gradually (in about 30 ms) from initial
 *   speed to final speed. This non-instantaneous speed change reduces the load 
 *   on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int diffLeft  = finalLeftSpd-initialLeftSpd;
  int diffRight = finalRightSpd-initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft  = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  
  int pwmLeftVal = initialLeftSpd;        // initialize left wheel speed 
  int pwmRightVal = initialRightSpd;      // initialize right wheel speed 
  int deltaLeft = (diffLeft)/numSteps;    // left in(de)crement
  int deltaRight = (diffRight)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(30);   
  } // end for int k
  analogWrite(left_pwm_pin,finalLeftSpd);  
  analogWrite(right_pwm_pin,finalRightSpd);  
} // end void  ChangeWheelSpeeds

void turn() {
  ChangeBaseSpeeds(leftOutput, 0, rightOutput, 0);
  digitalWrite(right_dir_pin,HIGH);
  ChangeBaseSpeeds(0, turnSpeed, 0, turnSpeed);
  resetEncoderCount_left();
  resetEncoderCount_right();
  while(getEncoderCount_left() < 300 && getEncoderCount_right() < 300) {}
  digitalWrite(right_dir_pin,LOW);
  ChangeBaseSpeeds(turnSpeed, 100, turnSpeed, 100);
  leftSpd = 80;
  rightSpd = 80;
  hasGlazed = true;
  resetEncoderCount_left();
  resetEncoderCount_right();
  Kp = 0.015;
  Kd = 0.3;
}

///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();
  

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  
}

void loop() {

  ECE3_read_IR(sensorValues);

  pastError = error;
  if(weightingSystem == 1) {
    error = sensorFusion(sensorWeights1);
  }
  else {
    error = sensorFusion(sensorWeights2);
  }
   
  float derivative = error - pastError;

  if (hasTurned1 == false){
    //allow rat to align from different starting positions
    if(getEncoderCount_left() > 200 && getEncoderCount_right() > 200) {
      Kp = 0.065;
      
      leftSpd = 70;
      rightSpd = 70;
      Turn1 = true;
      hasTurned1 = true;
    }
  }
  if (Turn1 == true){
      //time for rat to complete first turn
      if(getEncoderCount_left() > 1750 && getEncoderCount_right() > 1750) {
          ChangeBaseSpeeds(leftSpd, 200, rightSpd, 200);
          Kp = 0.025;
          Kd = 0.3;
          leftSpd = 200;
          rightSpd = 200;
          Turn1 = false;  
          resetEncoderCount_left();
          resetEncoderCount_right();
          hasOffset = true;
          weightingSystem = 1;
          //Kp = 0.025;
          //Kd = 0.3;
      }
  }
   
  if(hasOffset) {
      //first straightaway
      if(getEncoderCount_left() > 1190 && getEncoderCount_right() > 1190) {
          weightingSystem = 1;
          Kp = 0.020;
          Kd = 0.15;
          ChangeBaseSpeeds(leftSpd, 20, rightSpd, 20);
          leftSpd = 20;
          rightSpd = 20;  
          resetEncoderCount_left();
          resetEncoderCount_right();
          hasOffset = false;
          Offset = true;
      }
  }

  if(Offset) {
    //slow down, align properly
     if(getEncoderCount_left() > 300 && getEncoderCount_right() > 300) {
          Kp = 0.03;
          Kd = 0.3;
          leftSpd = 60;
          rightSpd = 60;
          resetEncoderCount_left();
          resetEncoderCount_right();
          Offset = false;
          donut2 = true;
     }   
  }

  if(donut2 && getEncoderCount_left() > 800 && getEncoderCount_right() > 800) {
    turn();
    donut2 = false;
  }
  
  /*
  if(donut2 && sensorValues[0] < 850 && sensorValues[1] < 750 && sensorValues[2] < 750 && sensorValues[3] < 650 && sensorValues[4] < 680 
  && sensorValues[5] < 750 && sensorValues[6] < 800 && sensorValues[7] < 770) {
    if(true){
      if(hasGlazed) {
        ChangeBaseSpeeds(leftOutput, 0, rightOutput, 0);
        exit(0);
      }
      turn();
      donut = false;
    }
    else
      donut = true;
  }
  */
  //else {
  //  donut = false;
  //}
  
  //turn function resets encoder counts
  
  if (hasTurned2 == false && Offset == false && hasGlazed == true){
    //rat gets past second offset on way back at 80 speed
    if(getEncoderCount_left() > 1400 && getEncoderCount_right() > 1400) { 
      ChangeBaseSpeeds(leftOutput, 200, rightOutput, 200);
      Kp = 0.025;
      Kd = 0.3;
      leftSpd = 200;
      rightSpd = 200;
      straight2 = true;
      hasTurned2 = true;
      resetEncoderCount_left();
      resetEncoderCount_right();
    }
  }
  
  if (straight2 == true) {
    //rat hits second straight at 200
    if(getEncoderCount_left() > 800 && getEncoderCount_right() > 800) { 
      //Serial.println("Finished Second Straight");
      Kp = 0.065;
      Kd = 0.3;
      ChangeBaseSpeeds(leftOutput, 70, rightOutput, 70);
      leftSpd = 70;
      rightSpd = 70;
      Turn2 = true;
      straight2 = false;
      resetEncoderCount_left();
      resetEncoderCount_right();
      //Serial.println("read");
    }
  }
  
  if(Turn2 == true) {
    //Serial.println("turn2");
    if (getEncoderCount_left() > 1950 && getEncoderCount_right() > 1950) {
        //Serial.println("end");
        Turn2 = false;
        ChangeBaseSpeeds(leftOutput, 0, rightOutput, 0);
        leftSpd = 0;
        rightSpd = 0;
        exit(0);
    }
  }
  
  float output = Kp * error + Kd * derivative;
  leftOutput = leftSpd - output;
  rightOutput = rightSpd + output;

  analogWrite(left_pwm_pin,leftOutput);
  analogWrite(right_pwm_pin,rightOutput);
  }
