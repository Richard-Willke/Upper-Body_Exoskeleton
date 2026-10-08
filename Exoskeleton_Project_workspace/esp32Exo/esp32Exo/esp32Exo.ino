/*
 * This code is used to read the controlelr outputs coming from the python script and apply it onto the servomotors after applying linear mapping.
 * The ESP32, which is used here reads the sensor values of 4 individual force sensors and one digital input from a switch and applies a PWM signal as input to two separate servo motors 
 */

#include <ESP32Servo.h>
#include <string.h>

#define SERVO_PIN  12
#define SERVO_PIN2 13
#include <stdlib.h>

Servo elbow;
Servo wrist;

const int analogInPin1 = 25; // analog pin A1, GPIO 25, force sensor 1
const int analogInPin2 = 34; // analog pin A2, GPIO 34, force sensor 2
const int analogInPin3 = 39; // analog pin A3, GPIO 39, force sensor 3
const int analogInPin4 = 36; // analog pin A4, GPIO 36, force sensor 4

const int switchPin = 14; // analog pin A14, GPIO 14 , digital switch

int sensorValue1 = 0; // values of the force sensors 
int sensorValue2 = 0;
int sensorValue3 = 0; 
int sensorValue4 = 0;

//upper and lower bound of the servomotor PWM signal (in micro seconds)
double posServo1 = 2500;//3040; //0 degrees 
double posServo2 = 500; //180 degrees 

double alpha_offset1 = 0.0;
double alpha_offset2 = 0.0;

bool motorAttach1 = false;
bool motorAttach2 = false;

double angle1 = 0.0;
double angle2 = 0.0;

bool mode = true; // mode of the system (continuous assistance or activity triggered)

//setting up the serial port and attaching the motors 
void setup()
{
  Serial.begin(9600);//(115200);//(9600);
  Serial.setTimeout(10);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SERVO_PIN2, OUTPUT);
  
  elbow.attach(SERVO_PIN); // PWM pin 1
  wrist.attach(SERVO_PIN2); // PWM pin 2
  delay(1000);
  
}

bool servoAttached = false;

/*
void motor_working(double mode) //function used to attach / detach the servo according to input from python
{
  if (mode > 0.0 && servoAttached == false){
    elbow.attach(SERVO_PIN); // PWM pin
    wrist.attach(SERVO_PIN2); // PWM pin
    servoAttached = true;
  }
  else if (mode <= 0.0 && servoAttached == true){
    elbow.attach(SERVO_PIN); // PWM pin
    wrist.attach(SERVO_PIN2); // PWM pin
    servoAttached = false;
  }
}
*/

//linear mapping function to map the desired angles to PWM values 
double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//function, that inhibits the motor from turning more than the valid ROM, returns the limiting values in case it would surpass either
double safety(double x, double offset, double out_max, double out_min)
{
  if ((x+offset) < out_min){
    return out_min;
  }
  else if ((x+offset) > out_max){
    return out_max;
  }
  else{
    return x;
  }
}

// controll function. repeatedly reads sensor values and serial port input and applies output to servomotor. then sends sensory values through serial port to the python part 
void loop()
{
  mode = digitalRead(switchPin); // read the physical switch

  sensorValue1 = analogRead(analogInPin1); // read the force sensors 
  sensorValue2 = analogRead(analogInPin2);
  sensorValue3 = analogRead(analogInPin3); 
  sensorValue4 = analogRead(analogInPin4);

  String x = Serial.readStringUntil('\n');
  char* token;
  char* string;
  string = strdup(x.c_str()); //reading string from serial port -> desired angular values
  double data[2];
 
  // Split the string at the comma and convert to double -> to be able to send the desired angles into the mapf function and set the motor inputs
  int commaIndex = x.indexOf(',');
  if (commaIndex != -1) {
    data[0] = x.substring(0, commaIndex).toDouble();
    data[1] = x.substring(commaIndex + 1).toDouble();
  }

  //send sensory data through serial port
  Serial.print("<");
  Serial.print(sensorValue1);
  Serial.print(",");
  Serial.print(sensorValue2);
  Serial.print(",");
  Serial.print(sensorValue3);
  Serial.print(",");
  Serial.print(sensorValue4);
  Serial.print(",");
  Serial.print(mode);
  Serial.println(">");

  // map the angular values to PWM signals 
  angle1 = safety(mapf(data[0], 0.0,3.14159, posServo1, posServo2), 0.0, posServo1, posServo2); //0.0 will be replaced by alpha_offset once the IMU is connected to the system (future work)
  angle2 = safety(mapf(data[1], 0.0,3.14159, posServo1, posServo2), 0.0, posServo1, posServo2);
  elbow.writeMicroseconds(angle1); //send the desired angle as input to the servomotor
  wrist.writeMicroseconds(angle2);

  delay(50);
}
