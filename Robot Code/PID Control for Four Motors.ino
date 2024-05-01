 //Code to run PID control on four motors 
 
 #include <util/atomic.h>
 #include <math.h> // include in case any more mathematical calculations need doing 


// With more motors, a class can be used to compute the control signal to make things neater
// The class doesn't have to be used, an extension of the One motor code would also work.
class SimplePID{
  private:
    float kp[4], kd[4], ki[4], umax[4]; // Parameters for each motor
    float eprev[4], eintegral[4]; //Error storage for each motor

   public:
  
  // Construct basic/default parameters that then get changed 
  SimplePID() {
    for(int i = 0; i < 4; i++) {
      kp[i] = 1;
      kd[i] = 0;
      ki[i] = 0;
      umax[i] = 150;// maximum torque to a given motor (to stop them going too fast)
      eprev[i] = 0.0;
      eintegral[i] = 0.0;
    }
  }

 // A function to set the parameters for a specific motor, this could be changed for a new control method
  void setParams(int motor, float kpIn, float kdIn, float kiIn){
    kp[motor] = kpIn; kd[motor] = kdIn; ki[motor] = kiIn;
  }


// A function to compute the control signal for a specific motor
  void evalu(int mot, int value, int target, float deltaT, float &pwr, int &dir , float umax, float G[4]){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev[mot])/(deltaT);
  
    // integral
    eintegral[mot] = eintegral[mot] + e*deltaT;
  
    // control signal
    float u = kp[mot]*e + kd[mot]*dedt + ki[mot]*eintegral[mot] + G[mot];
  
    // motor power and it's limit
    pwr = (float) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
     dir = -1;
    }
    // store previous error
    eprev[mot] = e;
  }
  
};

//necessary for serial control method
byte incomingByte = 0;
// Define the number of motors to allow the code to be versatile 
#define NMOTORS 4

// Define pins as the user wishes 
// Currently in this code first is motor 2, second is motor 3, third is motor 1, fourth is motor 4 
const int enca[] = {3,2,18,20};//Encoder A
const int encb[] = {5,4,19,21};//Encoder B 
const int pwm[] = {9,7,8,10};//PWM
const int in1[] = {41,31,37,43};//Motor Direction 1 
const int in2[] = {39,33,35,45};//Motor Direction 2

//In this code calibration functions are written for motors 2 and 3 
const int MotorLimit[] = {52,51};
const int MotorRange[] = {1500,1000};

//initial target positions are zero (for calibration)
float target[NMOTORS] = {0,0,0,0};

// link lengths for inverse kinematics 
const int L[] = {150,140,130,70};
//End effector desired angle 
const float phi = 0;
//radius and centre of circle 
const int r = 60;
const int centre[] = {230,0};


// I've set two maximum motor power/speed settings, since motor 2 struggles a lot more to lift it's own weight
// The but when pushed by it's own weight it goes very fast.
int umax_up[NMOTORS] = {255,155,100,100};
int umax_down[NMOTORS] = {100,100,100,100};

// Globals
long prevT = 0;
volatile int posi[NMOTORS];

// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(115200);//115200 baud rate is optimal

//set the pinmodes
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
  }
  
  //set the limit switches to "Input Pullup"
  pinMode(MotorLimit[0], INPUT_PULLUP);
  pinMode(MotorLimit[1], INPUT_PULLUP);
  
  //set parameters for each motor
  pid[0].setParams(0,10,0.2,0);
  pid[1].setParams(1,10,0.2,0);
  pid[2].setParams(2,8,0.2,0);
  pid[3].setParams(3,8,0.2,0);

  //Attach all 4 interrupts 
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]),readEncoder<2>,RISING);  
  attachInterrupt(digitalPinToInterrupt(enca[3]),readEncoder<3>,RISING);  
  
  //calibrate the motors buy calling these two functions 
  //Motor2Calibrate();
  //Motor1Calibrate();

  //If the calibration is done manually, or the calibraiton is undefined, just set their positions to zero 
  posi[0] = 0;
  posi[1] = 0;
  posi[2] = 0;
  posi[3] = 0;
}

void loop() {

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Coordinates of a circle in the x-y plane 
  float x = r*cos(1.5*prevT/1e6) + centre[0];
  float y = r*sin(1.5*prevT/1e6) + centre[1];
  
  //Setting a constant Z value for the circle to be drawn in 
  float z = 260;

  //Inverse Kinematics Calculations (NOTE:C++ can't do indicies so you have to multiply variables by themselves)
  double q1 = atan2(y,x); //Motor 1 target in radians 
  float eq1 = (x/cos(q1)) - L[3]*cos(phi);
  float eq2 = (z - L[0] - (L[3]*sin(phi)));
  int L1 = L[1]*L[1]; int L2 = L[2]*L[2];
  int L21 = (L[2])*(L[1]);
  float c3 = 0.5*((eq1*eq1) + (eq2*eq2) - L1 - L2) / (L21);
  double s3 = sqrt(1 - (c3*c3));
  double q3 = atan2(s3,c3); //Motor 3 target in radians 
  float k1 = L[1] + L[2]*cos(q3);
  float k2 = L[1]*sin(q3);
  double q2 = atan2(eq2,eq1) - atan2(k2,k1);//Motor 2 target in radians 
  double q4 = phi - q2 - q3;//Motor 4 target in radians 

  // Set Targets for the motors to draw the circle
  target[0] = 1*(q2+1.5708)*(2096/6.283);//Motor 2, needs 90 degree offset due to the IK
  target[1] = -1*q3*(1440/6.283);//Motor 3, needs to be negative due to motor coord system
  target[2] = q1*(1440/6.283);//Motor 1 
  target[3] = -q4*(1440/6.293);//Motor 4, also negative

 // You can build in an if statement to return the arm to upright after a certain amount of time 
 // if ((prevT/1e6)>2*6.283){
 //   target[0]=0; target[1]=0; target[2]=0; target[3]=0;
 // }

  // Read the position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }

  //Gravity Compensation
  float G11 = -2*9.81*cos(pos[0]/333.6);
  float G21 = 0.5*9.81*cos(pos[0]/333.6 + pos[1]/229.2);
  float G31 = 0.01*9.81*cos(pos[0]/333.6 + pos[1]/229.2 + pos[3]/229.2);
  float G[4] = {G11,G21,0,G31};


  // Set all the motors, using the class defined earlier 
  for(int k = 0; k < NMOTORS; k++){
    float pwr;//power
    int dir;//direction

    // evaluate the control signal
    //Currently using max torque for the motors based on "umax_up" 
    pid[k].evalu(k,pos[k],target[k],deltaT,pwr,dir,umax_up[k],G);
    
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }

  //Print all the targets and positions to output the responses
  for(int k = 0; k < 4; k++){
    Serial.print(target[k]);
    Serial.print(" ");
    Serial.print(pos[k]);
    Serial.print(" ");
  }
  Serial.println();

  //Instead of the above prints and targets, targets can be manually changed while the code runs using the "Serial Control" function
  //To use, call it (and not serial.print above) and type into the serial monitor (see below for instruction)
  //SerialControl();
}

//Calibration functions, make sure microswitches are installed on the correct side for calibration
void Motor1Calibrate(){
while (digitalRead(MotorLimit[0]) == LOW) {
 setMotor(1,60,pwm[0],in1[0],in2[0]);
}
 delay(10);
 posi[0] = MotorRange[0]/2; 
 setMotor(0,0,pwm[0],in1[0],in2[0]);
}

void Motor2Calibrate(){
while (digitalRead(MotorLimit[1]) == LOW) {
 setMotor(1,75,pwm[1],in1[1],in2[1]);
}
 delay(10);
 posi[1] = MotorRange[1]/2; 
 setMotor(0,0,pwm[1],in1[1],in2[1]);
}

//Set motor function, same as always 
void setMotor(int dir, float pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
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

template <int j> 
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}


//To use serial control to change target 1, type in e.g. a200 and the target will become 200. Or for motor 2 type in
//b100 and the target will change to 100. This code can be adapted to change something other than targets if needed.
void SerialControl(){
  
  while (Serial.available() > 0)
{
  incomingByte = Serial.read();

 switch (incomingByte) {

case 'a':    
for (int L=0; L < 1; L++) {
Serial.print('\n');
Serial.println("Motor 1 Selected");
target[0] = Serial.parseInt();
Serial.print("Motor 1 Target Pos");
Serial.print(target[0]);
}
       break;
       
    case 'b':    
for (int L=0; L < 1; L++) {
Serial.print('\n');
Serial.println("Motor 2 Selected");
target[1] = Serial.parseInt();
Serial.print(" Motor 2 Target Pos ");
Serial.print(target[1]);
}
      break;

case 'c':    
for (int L=0; L < 1; L++) {
Serial.print('\n');
Serial.println("Motor 3 Selected");
target[2] = Serial.parseInt();
Serial.print("Motor 3 Target Pos");
Serial.print(target[2]);
}
       break;

case 'd':    
for (int L=0; L < 1; L++) {
Serial.print('\n');
Serial.println("Motor 4 Selected");
target[3] = Serial.parseInt();
Serial.print("Motor 4 Target Pos");
Serial.print(target[3]);
}
       break;
}
}
}