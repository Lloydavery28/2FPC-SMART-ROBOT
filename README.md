# 2FPC-SMART-ROBOT
Fish feeding robot

CODES:

BLUETOOTH MODULE AND TEMPERATURE SENSOR (SENDER)


#include <DallasTemperature.h>

#include <OneWire.h>

 

int greenLedPin = 2;

int yellowLedPin = 3;

int redLedPin = 4;

 

int temp_sensor = 5;

 

float temperature = 0;

int lowerLimit = 15;

int higherLimit = 35;

 

OneWire oneWirePin(temp_sensor);

DallasTemperature sensors(&oneWirePin);

 

#include <Servo.h>

#define button 8

Servo myServo;

int state = 20;

int buttonState = 0;

void setup() {

  pinMode(button, INPUT);

  myServo.attach(9);

  Serial.begin(9600); // Default communication rate of the Bluetooth module

  Serial.begin(9600);

  

  //Setup the LEDS to act as outputs

  pinMode(redLedPin,OUTPUT);

  pinMode(greenLedPin,OUTPUT);

  pinMode(yellowLedPin,OUTPUT);

  

  sensors.begin();

}

void loop() {

 if(Serial.available() > 0){ // Checks whether data is comming from the serial port

    state = Serial.read(); // Reads the data from the serial port

 }

 

 

 

 

 

 

 // Reading the button

 buttonState = digitalRead(button);

 if (buttonState == HIGH) {

   Serial.write('1'); // Sends '1' to the master to turn on LED

 }

 else {

   Serial.write('0');

 }  

 {

  Serial.print("Requesting Temperatures from sensors: ");

  sensors.requestTemperatures(); 

  Serial.println("DONE");

  

  temperature = sensors.getTempCByIndex(0);

 

  digitalWrite(redLedPin, LOW);

  digitalWrite(greenLedPin, LOW);

  digitalWrite(yellowLedPin, LOW);

  

  Serial.print("Temperature is ");

  Serial.print(temperature);

 

  //Setup the LEDS to act as outputs

  if(temperature <= lowerLimit){

    Serial.println(", Yellow LED is Activated");

    digitalWrite(yellowLedPin, HIGH);

  }

  else if(temperature > lowerLimit && temperature < higherLimit){

    Serial.println(", Green LED is Activated");

    digitalWrite(greenLedPin, HIGH);

  }

  else if(temperature >= higherLimit){

    Serial.println(", Red LED is Activated");

    digitalWrite(redLedPin, HIGH);

  }

  delay(500);

 }

}

 


 

CODES:

BLUETOOTH AND WATER VALVE (RECEIVER)

#define ledPin 7

 

int state = 0;

int buttonPinState = 0;

 

void setup() {

  pinMode(buttonPin, INPUT);

  pinMode(ledPin, OUTPUT);

  digitalWrite(ledPin, LOW);

  Serial.begin(9600); // Default communication rate (9600 replace with 38400 if your bluetooth) of the Bluetooth module

}

void loop() {

  if (Serial.available() > 0) { // Checks whether data is comming from the serial port

    state = Serial.read(); // Reads the data from the serial port

  }

 

  // data receiving part

  if (state == '1')

  {

    digitalWrite(ledPin, HIGH); // led turn on

  }

  else if (state == '0')

  {

    digitalWrite(ledPin, LOW); // led turn of

  }

 

  // Data sending Part

  buttonPinState = digitalRead(buttonPin);

  if (buttonPinState == HIGH) {

    Serial.write('1'); // Sends '1' to the master to turn on LED

  }

  else {

    Serial.write('0');

  }

}

 

CODES:

(SERVO MOTOR, DC MOTOR, ULTRASONIC SENSOR)

 

#include <AFMotor.h> //import your motor shield library

#define trigPin 12 // define the pins of your sensor

#define echoPin 13 

#include <Servo.h>

AF_DCMotor motor1(1,MOTOR12_64KHZ); // set up motors.

AF_DCMotor motor2(2, MOTOR12_8KHZ);

 

void setup() {

  Serial.begin(9600); // begin serial communitication  

  Serial.println("Motor test!");

   pinMode(trigPin, OUTPUT);// set the trig pin to output (Send sound waves)

  pinMode(echoPin, INPUT);// set the echo pin to input (recieve sound waves)

  motor1.setSpeed(250); //set the speed of the motors, between 0-255

motor2.setSpeed (250);  

servo.attach(10);

servo.write(0);

delay(2000);

}

 

void loop() {

 

   long duration, distance; // start the scan

  digitalWrite(trigPin, LOW);  

  delayMicroseconds(3); // delays are required for a succesful sensor operation.

  digitalWrite(trigPin, HIGH);

 

  delayMicroseconds(10); //this delay is required as well!

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

 

 

 

 

 

 

 

 

 

 

 

 distance = (duration/2) / 29.1;// convert the distance to centimeters.

  if (distance > 30)/*if there's an obstacle 25 centimers, ahead, do the following: */ {   

   Serial.println ("No obstacle detected. going forward");

   delay (5);

    motor1.run(FORWARD);  // Turn as long as there's an obstacle ahead.

    motor2.run (BACKWARD);

 

}

  else {

 

Serial.println ("Close Obstacle detected!" );

Serial.println ("Obstacle Details:");

Serial.print ("Distance From Robot is " );

Serial.print ( distance);

Serial.print ( " CM!");// print out the distance in centimeters.

Serial.println (" The obstacle is declared a threat due to close distance. ");

Serial.println (" Turning !");

 

   motor1.run(FORWARD); //if there's no obstacle ahead, Go Forward! 

    motor2.run(FORWARD);  

}  

  

  

 

   servo.write(90);

    delay(5000);

    servo.write(0);

    delay(2000);

  

  

 

}

 

