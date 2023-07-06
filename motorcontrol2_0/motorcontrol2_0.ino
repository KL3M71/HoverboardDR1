#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

#define CHANNEL A0
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 9000; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/

double vReal[samples];
double vImag[samples];

bool digital = false; // digitalpin on microphone to detect when microphon is pickuping sound
// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
/* if we need a second motor we need to un comment said variables this applies to the other commnents in the setup() and
   anywhere else these variables are used 
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;
*/

/*
    is a flag for telling the motor which direction to turn
   I just made a random convention for determining directions 
   - 0 no turn 
   - 1 clockwise
   - -1 counter clockwise 
*/
int turnDirection = 0;
int pos = 0;

void directionControl(int direction);

void setup(){
  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");

  pinMode(2,INPUT);

	// Set all the motor control pins to outputs 
	pinMode(enA, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
  /*
  pinMode(enB, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	*/
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
  /*
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
  */
}

void loop(){
  /*SAMPLING*/
  microseconds = micros();
  digital = digitalRead(2);

  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  Serial.println(x, 6); //Print out what frequency is the most dominant.


  delay(50); /* Repeat after delay */
  //Serial.println(digital); // used to get a glance at when the microphone is registering


  /* frequency detection for turning the hoverboard
     digital is also used to help reduce the influence of noise in frequency detecting
  */
  switch(pos){
    //neutral
    case 0:
        if(x > 700 && x < 800 && digital){
        Serial.println("this is the low frequency");
        Serial.println("neutral");
        turnDirection = 1;
        directionControl(turnDirection);
        pos = 1;
      }
      else if(x > 1200 && x < 1300 && digital){
        Serial.println("this is the high frequency");
        Serial.println("neutral");
        turnDirection = -1;
        directionControl(turnDirection);
        pos = 2;
      }
      break;
    //forward
    case 1:
        if(x > 700 && x < 800 && digital){
          Serial.println("this is the low frequency");
          Serial.println("forward");
      }

      else if(x > 1200 && x < 1300 && digital){
        
        Serial.println("this is the high frequency");
        Serial.println("forward");
        turnDirection = -1;
        directionControl(turnDirection);
        pos = 0;
      }
      break;
    //backwards
    case 2:
        if(x > 700 && x < 800 && digital){
          Serial.println("this is the low frequency");
          Serial.println("backwards");
          turnDirection = 1;
          directionControl(turnDirection);
          pos = 0;
      }

      else if(x > 1200 && x < 1300 && digital){
        Serial.println("this is the high frequency");
        Serial.println("backwards");
      }
      break;
    }

  

  turnDirection = 0;
}

void directionControl(int direction){

  // Set motors to maximum speed
	// For PWM maximum possible values are 0 to 255
	analogWrite(enA, 255);
  switch(direction){
    case -1:
      	digitalWrite(in1, HIGH);
	      digitalWrite(in2, LOW);
      break;
    case 1:
        digitalWrite(in1, LOW);
	      digitalWrite(in2, HIGH);
      break; 
  } 
  delay(750);

  // turn off all motors

  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);

}