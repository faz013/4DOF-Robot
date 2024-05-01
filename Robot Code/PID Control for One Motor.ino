#include <util/atomic.h> // For the position read

//Define Encoder pins, pwm pin, and motor driver pins
#define ENCA 18 // interrupt pin
#define ENCB 19
#define PWM 8 // can be any pwm pin on board 
#define IN2 35 // can be any digital pin
#define IN1 37

volatile int posi = 0; // specify posi as volatile, int because counts are discrete
long prevT = 0; //variable for time count
float eprev = 0; // previous loop's error 
float eintegral = 0;//integral error 

void setup() {
  //baud rate 
  Serial.begin(230400);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  //pwm and direction pins are outputs that go to the motor
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

}
// This loop has PID control code in it, but any other control algorithm can be used 
void loop() {

  // PID constants
  float kp = 10;
  float kd = 0.5;
  float ki = 0;

  // find time difference between each loop 
  long currT = micros(); // micros gives time of simulation in microseconds 
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  //set a target position for the motor. It will be in counts, so conversion is needed to degrees for output. 
  //This target is an example of how to make it time-dependent 
  float target = 180*sin(currT/1e6);


  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // current position error
  float e = target - pos;

  // derivative error 
  float dedt = (e-eprev)/(deltaT);

  // integral error
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // set motor power as absolute of control signal 
  float pwr = fabs(u);

  // limit pwr as 255 to not overdrive motors, if this response is too fast then limit can be lowered. 
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction based on sign of control signal 
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // call a function to drive the motor at a set direction and torque
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;
 
  //print target and position to see the response 
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");  
  Serial.println();
}

//function to set motor (could also be define at top of code)
// set inputs and their classifications 
void setMotor(int dir, float pwmVal, int pwm, int in1, int in2){
  //set the pwm pin to the appropriate level
  analogWrite(pwm,pwmVal);
  //set direction
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}