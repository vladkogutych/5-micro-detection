#include "arduinoFFT.h"
#include <Servo.h>

// analog inputs for all microphones
#define MICROPHONE1 A0
#define MICROPHONE2 A1
#define MICROPHONE3 A2
#define MICROPHONE4 A3
#define MICROPHONE5 A6
// pin for servo
#define SERVOPIN 5
// altering ADC prescale
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) //set bit to 0
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  //set bit to 1
// object of the servo class
Servo directionServo;
// object for FFT
arduinoFFT FFT = arduinoFFT();
// servo enable
bool servoEnable = 1;
// angle of the sound source
float angle = 90;
// number of microphones
const int microphone_count = 5;

// ----------------1 Initialize arrays and variables-----------------

// constants for FFT
const uint16_t samples = 16;                  // number of samples, must be a power of 2 to be more efficient during FFT
const double sampling_frequency = 10000;      // Hz, limited by ADC conversion time (~15us per reading with prescaling)
int sampling_period_us = round(1000000.0/sampling_frequency); // sampling period in microseconds, used in sampling loop

// data arrays for FFT, each microphone gets a real and an imaginary part
double Real1[samples];
double Real2[samples];
double Real3[samples];
double Real4[samples];
double Real5[samples];
double Imag1[samples];
double Imag2[samples];
double Imag3[samples];
double Imag4[samples];
double Imag5[samples];
// array that contains the total average of all magnitudes for each microphone
float averages[microphone_count];

// method to calculate the average of a given array with a given size
float calculate_average(double arr[], int arrSize) {
  // initialize return value
  float sum = 0;
  // loop over array
  for(int i=0;i<arrSize;i++) {
    // add up values
    sum += arr[i];
  }
  // return sum/size
  return sum/float(arrSize);
}

// method to find the highest value in a given array and return the index
int findMaximumIndex(float arr[],int arrSize) {
  int index = 0;
  float currentMax = 0;
  for(int i=0;i<arrSize;i++){
    if(arr[i]>currentMax){
      currentMax = arr[i];
      index = i;
    }
  }
  return index;
}

// method to determine the angle based on counters for each microphone
float calculateAngle(float arr[], int arrSize) {
  //------------------------------TODO: implement function--------------------
  /**
  Serial.print(counts[0]);
  Serial.print(",");
  Serial.print(counts[1]);
  Serial.print(",");
  Serial.print(counts[2]);
  Serial.print(",");
  Serial.print(counts[3]);
  Serial.print(",");
  Serial.println(counts[4]);
  **/
  float finalAngle = 0;
  int maxIndex = findMaximumIndex(arr,arrSize);
  finalAngle = maxIndex*45;
  return finalAngle;
}

// method to determine the angle of the sound source
void determineAngle() {
  long startTime = micros();  // time measurement start
  // initialize timestamp for discrete sampling
  long microseconds = micros();
  // take samples
  for(int i=0;i<samples;i++) {
    // read analog values
    Real1[i] = analogRead(MICROPHONE1);
    Real2[i] = analogRead(MICROPHONE2);
    Real3[i] = analogRead(MICROPHONE3);
    Real4[i] = analogRead(MICROPHONE4);
    Real5[i] = analogRead(MICROPHONE5);
    // set imaginary values to 0
    Imag1[i] = 0;
    Imag2[i] = 0;
    Imag3[i] = 0;
    Imag4[i] = 0;
    Imag5[i] = 0;
    // wait until sampling period is over
    while((micros()-microseconds)<sampling_period_us) {
      // empty loop, just wait
    }
    // add sampling period to first timestamp
    microseconds += sampling_period_us;
  }
  // --------FFT calculations for all microhpones-----------
  // window data to avoid issues at sampling edges
  FFT.Windowing(Real1, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Windowing(Real2, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Windowing(Real3, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Windowing(Real4, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Windowing(Real5, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  // compute FFT
  FFT.Compute(Real1, Imag1, samples, FFT_FORWARD);
  FFT.Compute(Real2, Imag2, samples, FFT_FORWARD);
  FFT.Compute(Real3, Imag3, samples, FFT_FORWARD);
  FFT.Compute(Real4, Imag4, samples, FFT_FORWARD);
  FFT.Compute(Real5, Imag5, samples, FFT_FORWARD);
  // calculate absolute magnitudes
  FFT.ComplexToMagnitude(Real1, Imag1, samples);
  FFT.ComplexToMagnitude(Real2, Imag2, samples);
  FFT.ComplexToMagnitude(Real3, Imag3, samples);
  FFT.ComplexToMagnitude(Real4, Imag4, samples);
  FFT.ComplexToMagnitude(Real5, Imag5, samples);
  // calculate average magnitudes
  averages[0]=calculate_average(Real1,samples);
  averages[1]=calculate_average(Real2,samples);
  averages[2]=calculate_average(Real3,samples);
  averages[3]=calculate_average(Real4,samples);
  averages[4]=calculate_average(Real5,samples);
  // determine angle
  angle = calculateAngle(averages,microphone_count);
  long totalTime = micros();            // time measurement
  // Prints for debugging
  Serial.println("angle: "+String(angle));
  // Print for time measurementS
  Serial.println("Total Time: "+String(totalTime-startTime));
}


void setup() {
  // -----------------------2 Initialize IO-pins---------------------
  Serial.begin(115200);
  // set up analog inputs
  pinMode(MICROPHONE1,INPUT);
  pinMode(MICROPHONE2,INPUT);
  pinMode(MICROPHONE3,INPUT);
  pinMode(MICROPHONE4,INPUT);
  pinMode(MICROPHONE5,INPUT);
  // ADPS2 ADPS1 ADPS0
  //   1     0     0    -> Prescale 16 ->  15 us (1 reading)   75 (5 readings)
  sbi(ADCSRA, ADPS2); // set bit to 1
  cbi(ADCSRA, ADPS1); // set bit to 0
  cbi(ADCSRA, ADPS0); // set bit to 0
  
  // ------------------------3 servoEnable?-------------------------
  if(servoEnable) {
    // --------------------3.1 Initialize Servo---------------------
    directionServo.attach(SERVOPIN);
    directionServo.write(90);
  }
}

void loop() {
  Serial.println("--------------------");
  determineAngle();
  // do something with angle
  if(servoEnable) {
    directionServo.write(angle);
  }
  // wait for delay
  delay(1000);
}
