#include <Adafruit_MotorShield.h>

/// CHANGE THESE VARIABLES ///
int MOTOR_DELAY = 30; // [ms]
float Z_MAX = 10; // [cm] maximum scanline height
float dz = 0.2; // [cm]
float dtheta = 36; // must be a multiple of 3.6 degrees

//////////////////////////////

// HIGH LEVEL CONTROLS //
bool scan = false;
bool button_state = LOW;
const int button_pin = 2; 

// MOTOR RELATED //
int steps_per_cm = 100;
int dz_step = dz*steps_per_cm; // number of motor steps between scan planes
int dtheta_step = (int) dtheta/3.6; // number of motor steps

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *z_stepper = AFMS.getStepper(100, 1);
Adafruit_StepperMotor *theta_stepper = AFMS.getStepper(100, 2);

// SCAN RELATED //
const int trig_pin = 5;
const int echo_pin = 6;
const float delimiter = 1234;
double d;

double measure_distance(){
  // Clears the trigPin
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  int duration = pulseIn(echo_pin, HIGH);
  // Calculating the distance
  double distance = duration * 0.034 / 2;
  return distance;
}

void move_z(int dz_step){
  z_stepper->step(dz_step,FORWARD, SINGLE); 
}
void move_theta(int dtheta_step){
  theta_stepper->step(dtheta_step,FORWARD, SINGLE);
}
void reset_z(){
  for( int i =0; i<Z_MAX*steps_per_cm; i++){
    z_stepper->onestep(BACKWARD, SINGLE);
    delay(MOTOR_DELAY);
  }
}

void setup() {

  Serial.begin(9600);           // set up Serial library at 9600 bps
   Serial.println("Stepper test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");



  // ultrasound
  pinMode(trig_pin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echo_pin, INPUT); // Sets the echoPin as an Input
  pinMode(button_pin, INPUT); 

  z_stepper->setSpeed(100);
  theta_stepper->setSpeed(100);
}

void loop() {

  button_state = digitalRead(button_pin);

  if(button_state == HIGH){
    scan = true;
  }
  
  // wait for trigger
  if (scan){
    Serial.println("Start scan");
    Serial.println(Z_MAX);
    Serial.println(dz);
    Serial.println(dtheta);
    Serial.println(delimiter);
    
    for(float z = 0; z<Z_MAX; z+=dz){
      for(int theta = 0; theta<360; theta+=dtheta){
        d = measure_distance();
        Serial.println(d);
        move_theta(dtheta_step);       
      }
      Serial.println(delimiter);
      move_z(dz_step);
    }
    Serial.println("Scan Complete");
    reset_z();
    scan = false; 
  }

  button_state = LOW;
  delay(1000);
}
