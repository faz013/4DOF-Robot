#include <util/atomic.h> 

// Define the pin numbers for the encoder pins A and B 
#define ENCA 2 // this one needs to be on an interupt (2,3,18,19,20,21)
#define ENCB 4

volatile float posi = 0; // specify posi as volatile so it can change quickly

void setup() {
  //set baud rate 
  Serial.begin(9600);

  //set the encoder pins as inputs 
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);

  //attach interrupt function to encoder pin a, activates when digital signal rises 
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  // Read the position in an atomic block to avoid a potential misread of position
  // Atomic block library is needed for this 
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  //Print the position so it can be view on serial plotter, or exported to excel/python
  Serial.println(pos);
}

//write function to read the encoder, this is called in the attach interrupt function
void readEncoder(){
  //read digital input to pin B 
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
