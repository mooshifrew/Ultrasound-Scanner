// defines pins numbers
const int txPins[3] = {8,9,10};
const int rxPin = 7;

// defines variables
float distances[3] = {0,0,0};

// Mux
const int aPin = 5;
const int bPin = 4;

void setup() {
  for (int i =0; i<3; i++){
    pinMode(txPins[i], OUTPUT);
  }
  pinMode(aPin, OUTPUT);
  pinMode(bPin, OUTPUT);
  pinMode(rxPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  txRx(distances);

  Serial.print("Distances: [");
  for (int i = 0; i<3; i++){
    Serial.print(distances[i]);
    Serial.print(",");
  }
  Serial.println("]");
  delay(1000);

}

void txRx(float distances[]){
  for (int i =0; i<3; i++){
    // Serial.print("Scanning tx ");
    // Serial.println(i);
    //set multiplexer
    int result = i/2;
    // Serial.print("A Pin: ");
    // Serial.print(result);
    if (result = 0){ digitalWrite(aPin, LOW);    }
    else {
      digitalWrite(aPin, HIGH);
    }
    result = i%2;
    // Serial.print("B Pin: ");
    // Serial.println(result);
    if (result=0){
      digitalWrite(bPin, HIGH);
    }else{
      digitalWrite(bPin,LOW);
    }
    
    digitalWrite(aPin, i/2);
    digitalWrite(bPin, i%2);
    int tx = txPins[i];

    digitalWrite(tx, LOW);
    delayMicroseconds(2);

    digitalWrite(tx, HIGH);
    delayMicroseconds(10);
    digitalWrite(tx, LOW);

    int duration = pulseIn(rxPin, HIGH);
    float distance = duration*0.034/2;
    // Serial.println(distance);
    distances[i] = distance;  
  }  
}
