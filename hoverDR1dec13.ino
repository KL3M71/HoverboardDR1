#include "arduinoFFT.h"
#include <Servo.h>

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

#define CHANNEL A0
#define LINEARACTUATORANGLE 120
#define LINEARACTUATORDEFAULT 70
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 4500; //Hz, must be less than 10000 due to ADC

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
int lowDig = 0;

// timelapse variable
unsigned long int startTime = 0;
unsigned long int endTime = 0;
unsigned long int elapsedTime = 0;
unsigned long int totalTime = 0;

// a boolean variable that tells you if there has been a command sent or note used for the emergency stop timer
bool command = false;

Servo linearActuator;

void directionControl(int direction);
unsigned long int emergencyStopTimer(unsigned long int elapsedTime ,unsigned long int totalTime, bool command );

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
  linearActuator.attach(10); // pin 10 is a pwm pin for the pwm wire on the servo motor

}

void loop(){
  /*SAMPLING*/
  startTime = millis();
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
  //Serial.println(x, 6); //Print out what frequency is the most dominant.
  Serial.println(digital);

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
        lowDig = 0;
        command = true;
      }
      else if(x > 1200 && x < 1300 && digital){
        Serial.println("this is the high frequency");
        Serial.println("neutral");
        turnDirection = -1;
        directionControl(turnDirection);
        pos = 2;
        lowDig = 0;
        command = true;
      }
      else if( x > 1500 && x < 1700){
        quickstop();
      }
      break;
    //forward
    case 1:
        if(x > 700 && x < 800 && digital){
          Serial.println("this is the low frequency");
          Serial.println("forward");
          lowDig = 0;
          command = true;
      }

      else if(x > 1200 && x < 1300 && digital){
        
        Serial.println("this is the high frequency");
        Serial.println("forward");
        turnDirection = -1;
        directionControl(turnDirection);
        pos = 0;
        lowDig = 0;
        command = true;
      }
      else if( x > 1500 && x < 1700){
        quickstop();
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
          lowDig = 0;
          command = true;
      }

      else if(x > 1200 && x < 1300 && digital){
        Serial.println("this is the high frequency");
        Serial.println("backwards");
        lowDig = 0;
        command = true;
      }
      else if( x > 1500 && x < 1700){
        quickstop();
      }

      break;
    }


    if(digital == false){
      lowDig++;
      Serial.println(lowDig);
      if(lowDig >= 5){
        if(pos == 1){
          Serial.println("forward");
          turnDirection = -1;
          directionControl(turnDirection);
          pos = 0;
          lowDig = 0;

        }
        if(pos == 2){
          Serial.println("backwards");
            turnDirection = 1;
            directionControl(turnDirection);
            pos = 0;
            lowDig =0;
        }
        else
        lowDig = 0;
      }
    }

  turnDirection = 0;
  endTime = millis();
  elapsedTime = endTime - startTime;
  totalTime = emergencyStopTimer(elapsedTime, totalTime,command);

}

void quickstop(){
  Serial.println("stop frequency");
  linearActuator.write(LINEARACTUATORANGLE);
  delay(500);        
  linearActuator.write(LINEARACTUATORDEFAULT);
  delay(5000);
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
  delay(300);

  // turn off all motors

  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);

}

unsigned long int emergencyStopTimer(unsigned long int elapsedTime, unsigned long int totalTime, bool command){
  totalTime += elapsedTime; 
  // Total time is in milliseconds so 10000 10 seconds
  if ( totalTime >= 5000 && command == false){
    linearActuator.write(LINEARACTUATORANGLE); // the servo moves 90 degrees or in this case moves up to push the button this can be fine tuned with experimentation
    delay(500);
    linearActuator.write(LINEARACTUATORDEFAULT);
    delay(100000);
    totalTime = 0 ; // resets the total time elapsed when no command it sent
  } 
  else if ( command == true){
    command = false; // so when you go through an iteration again you have to check for if a comman is executed
    return 0;
  }
  return totalTime;
}