#include <Adafruit_MotorShield.h>

/// CHANGE THESE VARIABLES ///
int MOTOR_DELAY = 20; // [ms]
float Z_MAX = 10; // [cm] maximum scanline height
float dz = 1; // [cm]

// Common Multiples: 3.6, 7.2, 18.0, 36.0, 72.0 (only 18 and greater seems to work)
float dtheta = 39.6; // must be more than 36 degrees <- not true


//////////////////////////////

// HIGH LEVEL CONTROLS //
bool scan = false;
bool button_state = LOW;
const int button_pin = 2; 

// MOTOR RELATED //
int steps_per_cm = 100;
int dz_step = dz*steps_per_cm; // number of motor steps between scan planes
int dtheta_step = (int) dtheta/3.6; // number of motor steps
int num_steps = 3960/dtheta; //steps for 360 degree turn

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *z_stepper = AFMS.getStepper(100, 1);
Adafruit_StepperMotor *theta_stepper = AFMS.getStepper(100, 2);

// SCAN RELATED //
const int trig_pin = 5;
const int echo_pin = 6;
const float delimiter = 1234;
const int tx_per_scan = 5; 
double d;

double measure_distance(){
  delay(50);
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
  for (int i=0; i<dz_step; i++){
    //z_stepper: raise sensor
    z_stepper->onestep(FORWARD,  SINGLE);
    delay(MOTOR_DELAY);
  }
  
}
void move_theta(int dtheta_step){
  for (int i=0; i<dtheta_step; i++){
    //theta_stepper: rotate table
    theta_stepper->onestep(FORWARD,  SINGLE);
    delay(MOTOR_DELAY);
  }
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
  Serial.println("Press green button to start scanning.");

  // ultrasound
  pinMode(trig_pin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echo_pin, INPUT); // Sets the echoPin as an Input
  pinMode(button_pin, INPUT); 

  z_stepper->setSpeed(255);
  theta_stepper->setSpeed(255);
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
    Serial.println(tx_per_scan);
    
    for(float z = 0; z<Z_MAX; z+=dz){
      for(int i = 0; i < num_steps; i+=1){// theta = 0; theta<360; theta+=dtheta){
        for(int scan = 0; scan<tx_per_scan; scan++){
          d = measure_distance();
          Serial.println(d);
        }
        move_theta(dtheta_step); 
        delay(500);
      }
      Serial.println(delimiter);
      if(z != Z_MAX - 1){
        move_z(dz_step);
        AFMS.begin();
      }
 
    }
    Serial.println("Scan Complete");
    reset_z();
    scan = false; 
  }

  button_state = LOW;
  delay(1000);
}
