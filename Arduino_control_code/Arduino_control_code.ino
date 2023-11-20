#include <Adafruit_MotorShield.h>

/// CHANGE THESE VARIABLES ///
int MOTOR_DELAY = 20; // [ms]
float Z_MAX = 10; // [cm] maximum scanline height
float dz = 0.5; // [cm]

// Common Multiples: 3.6, 7.2, 18.0, 36.0, 72.0 (only 18 and greater seems to work)
float dtheta = 39.6; // must be more than 36 degrees <- not true


//////////////////////////////

// HIGH LEVEL CONTROLS //
int scan_mode = 2; // 2 - 2D scanning; 3 - 3D scanning
bool scan = false;
bool s2D = LOW; 
const int pin2D = A0; // green button
bool s3D = LOW;
const int pin3D = A1; // red button

// MOTOR RELATED //
int steps_per_cm = 100;
int dz_step = dz*steps_per_cm; // number of motor steps between scan planes
int dtheta_step = (int) dtheta/3.6; // number of motor steps
int num_steps = 3960/dtheta; //steps for 360 degree turn

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *z_stepper = AFMS.getStepper(100, 1);
Adafruit_StepperMotor *theta_stepper = AFMS.getStepper(100, 2);

// SCAN RELATED //
const int n_scanners = 5;
const int tx_Pins[n_scanners] = {13,7,5,3,1}; // 5 is the center sensor
const int rx_Pins[n_scanners] = {8 ,6,4,2,0}; // 4 is center sensor

const float delimiter = 1234;
const int tx_per_scan = 5; 
double d;

// scan_number: the scanner to be used for this tx/rx. 
// (Only for 2D scanning)
double measure_distance(int sensor_index = 2){
  if (sensor_index>4){
    return 0.0;
  }
  // default to the center scanner
  int trig_pin = tx_Pins[sensor_index];
  int echo_pin = rx_Pins[sensor_index];

  // perform single scan as determined by the scan number
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
    // while (1);
  }
  Serial.println("Motor Shield found.");
  Serial.println("Press button to start scanning. Green=2D, Red=3D");

  // sensor pins
  for( int i = 0; i<5; i++){
    pinMode(tx_Pins[i], OUTPUT); // set all trig pins
    pinMode(rx_Pins[i], INPUT); // set all echo pins
  }

  pinMode(pin2D, INPUT);
  pinMode(pin3D, INPUT); 

  z_stepper->setSpeed(255);
  theta_stepper->setSpeed(255);
}

void loop() {

  // s2D = digitalRead(pin2D);
  // s3D = digitalRead(pin3D);
  s2D=HIGH;

  if(s2D == HIGH){
    scan = true;
    scan_mode = 2;
  }
  if(s3D == HIGH){
    scan = true;
    scan_mode = 3;
  }
  
  // wait for trigger
  if (scan){
    Serial.println("Start scan");
    Serial.println(scan_mode);
    Serial.println(Z_MAX);
    Serial.println(dz);
    Serial.println(dtheta);
    Serial.println(delimiter);
    Serial.println(tx_per_scan);
    
    if (scan_mode==2){
      for (float z=0; z<Z_MAX; z+=dz){
        for(int sensor = 0; sensor<n_scanners; sensor++){
          d = measure_distance(sensor);
          Serial.println(d);
        }
        if(z!=Z_MAX-1){
          move_z(dz_step);
        }
      }
    }

    else if (scan_mode==3){
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
    }
    
    Serial.println("Scan Complete");
    reset_z();
    scan = false; 
    Serial.println("Press button to start scanning. Green=2D, Red=3D");
  }

  s2D = LOW;
  s3D = LOW;
  delay(1000);
}
