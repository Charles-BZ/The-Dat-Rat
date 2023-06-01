// Base code.
// 
// *  NOTE: this code will do only three things:
// *    --rotate one wheel, and 
// *    --blink the right front mainboard LED.
// *    
// *  You will need to add more code to
// *  make the car do anything useful. 
// 

#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7
float sensorMins[8] = {828, 712, 734, 689, 666, 732, 758, 757.2};
float sensorMax[8] = {1672, 1747, 1766, 1074.6, 1187.6, 1768, 1661.6, 1742.8};
//float sensorWeights[8] = {-15, -14, -12, -8, 8, 12, 14, 15};
float sensorWeights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
float error = 0;
float pastError = 0;

float Kp = 0.09;
float Kd = 0.3;
//works for turn: 0.09, 0.2

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int LED_RF = 41;

int leftSpd = 80;
int rightSpd = 80;

float leftOutput = 0;
float rightOutput = 0;

int turnSpeed = 100;

bool donut = false;
bool hasGlazed = false;


float sensorFusion() {
  float temp_error = 0;
  float temp_vals[8];
  for (unsigned char i = 0; i < 8; i++)
  {
    temp_vals[i] = sensorValues[i];
    temp_vals[i] -= sensorMins[i];
    temp_vals[i] *= (1000 / sensorMax[i]);
    temp_vals[i] *= sensorWeights[i];
    temp_error += temp_vals[i];
    //Serial.println(temp_error);
  }
  return temp_error / 4;
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
  ChangeBaseSpeeds(turnSpeed, leftSpd, turnSpeed, rightSpd);
  hasGlazed = true;
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
  // put your main code here, to run repeatedly: 
  

  ECE3_read_IR(sensorValues);

  pastError = error;
  error = sensorFusion();
  float derivative = error - pastError;

  if(sensorValues[0] < 850 && sensorValues[1] < 770 && sensorValues[2] < 780 && sensorValues[3] < 720 && sensorValues[4] < 710
  && sensorValues[5] < 780 && sensorValues[6] < 800 && sensorValues[7] < 820) {
    if(donut){
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
  else {
    donut = false;
  }
  
  float output = Kp * error + Kd * derivative;
  leftOutput = leftSpd - output;
  rightOutput = rightSpd + output;

  analogWrite(left_pwm_pin,leftOutput);
  analogWrite(right_pwm_pin,rightOutput);
  
/*
  for (unsigned char i = 0; i < 8; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
*/

  delay(0);
  }
