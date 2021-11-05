

//This version of Valves puzzle uses servos
     
// INCLUDES
// See: https://github.com/NachtRaveVL/PCA9685-Arduino
#include "PCA9685.h"
 
// CONSTANTS
// The input pins to which the potentiometers is connected. NOTE - Must use analog (Ax) input pins!
const byte potPins[] = {A0, A1, A2, A3, A6};
// The total number of valves
const byte numPots = 5;
// The level to which each valve must be set to solve the puzzle
const int solution[numPots] = {5, 2, 9, 6, 1};
// This digital pin will be driven LOW to release a lock when puzzle is solved
const byte lockPin = A7;

//SDA -> A4
//SCL -> A5


 
// GLOBALS
// This array will record the current reading of each input valve
int currentReadings[numPots] = {};
// Create a controller for the servos
PCA9685 pwmController;
// This will help evaluate the PWM signal for a given servo angle
PCA9685_ServoEval pwmServo;
// Array of output values to send to PCA9685 for each servo
int channelOutput[numPots];
// Track state of overall puzzle
enum PuzzleState {Initialising, Running, Solved};
PuzzleState puzzleState = Initialising;
 
/**
 * Returns true if the puzzle has been solved, false otherwise
 */
bool checkIfPuzzleSolved(){
  for(int i=0; i<numPots; i++) {
    if(currentReadings[i] != solution[i]) {
      return false;
    }
  }
  return true;
}
 
/**
 * Called when the puzzle is solved
 */
void onSolve() {
  Serial.println("Puzzle has just been solved!");
  // Release the lock
  digitalWrite(lockPin, LOW);
  puzzleState = Solved;
}
 
/**
 * Initialisation
 */
void setup(){
 
  // Initialise serial communications channel with the PC
  Serial.begin(115200);
 
  // Set the linear pot pins as input
  for(int i=0; i<numPots; i++){
    // Set the pin for the pot
    pinMode(potPins[i], INPUT);
  }
 
  // Set the lock pin as output and secure the lock
  pinMode(lockPin, OUTPUT);
  digitalWrite(lockPin, HIGH);
 
  // Initialise I2C interface used for the PCA9685 PWM controller
  Wire.begin();
  // Supported baud rates are 100kHz, 400kHz, and 1000kHz
  Wire.setClock(400000);
  // Initialise PCA9685
  pwmController.resetDevices();
  pwmController.init();
  // 50Hz provides 20ms standard servo phase length
  pwmController.setPWMFrequency(50);  
 
  // Set the puzzle into the running state
  puzzleState = Running;
}
 
/**
 *  Read the input from the potentiometers and store in the currentReadings array
 */
void getInput() {
  // Read the value from the pots
  for(int i=0; i<numPots; i++){
    // Get the "raw" input, which is a value from 0-1023
    int rawValue = analogRead(potPins[i]);
    // Scale the input to a value from 0-10
    int scaledValue = map2(rawValue, 0, 1023, 0, 10);
    // To ensure we don't get any dodgy values, constrain the output range too
    scaledValue = constrain(scaledValue, 0, 10);
    // Store the scaled value in the currentReadings array
    currentReadings[i] = scaledValue;
  }
}
 
// The Arduino map() function produces incorrect distribution for integers. See https://github.com/arduino/Arduino/issues/2466
long map2(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min+1) / (in_max - in_min+1) + out_min;
}
 
/**
 * Main program loop runs indefinitely
 */
void loop(){
 
  // Switch action based on the current state of the puzzle
  switch(puzzleState) {
  case Initialising:
    puzzleState = Running;
    break;
 
  case Running:
    getInput();
    // Now, send the output values to the PWM servo controller
    for(int i=0; i<numPots; i++) {
    // Convert input reading (which is from 0-10) to angle from -90 to +90
    int angle = map(currentReadings[i], 0, 10, -85, 85);
    // Then, calculate PWM value corresponding to that angle and assign it to the appropriate output channel
    pwmController.setChannelPWM(i, pwmServo.pwmForAngle(angle));
    }
    if(checkIfPuzzleSolved()){ 
    onSolve();
    }
    break;
 
  case Solved:
    break;
  }
}
