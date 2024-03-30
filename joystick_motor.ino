//controller 
const int buttonPin = 13; // Pin connected to the button
const int buttonPin2 = 4; // digitial pin connected to the stop button 
//motors
int motor1speedpin = 5; //enable A pin
int motor2speedpin = 6;// enable B pin 

//input pins from h bridge
int motor1pin1 = 12;
int motor1pin2 = 11;
int motor2pin1 = 10;
int motor2pin2 = 9;

int motorSpeed =100;
int stopSpeed =0;

void setup() {
  Serial.begin(9600);

  pinMode(motor1speedpin,OUTPUT); // enable a
  //pinMode(motor2speedpin, OUTPUT); // enable b

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  //pinMode(motor2pin1, OUTPUT);
  //pinMode(motor2pin2, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP); // Internal pull-up resistor enabled
  pinMode(buttonPin2, INPUT_PULLUP); // Internal pull-up resistor enabled

}

void loop() {
  
  int buttonState = digitalRead(buttonPin);
  int buttonState2 =digitalRead(buttonPin2);
    
  if (buttonState == LOW) {
    Serial.println("Button pressed");
    //analogWrite(motor1speedpin, motorSpeed);
    //analogWrite(motor2speedpin, motorSpeed);

    // Control the left motor
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    Serial.println("Motor On");

    //right motor 
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);

  } else {
    Serial.println("Button not pressed");
  }

  if (buttonState2 ==LOW) {
  Serial.println("Stop button pressed ");

  //analogWrite(motor1speedpin, stopSpeed);
  //analogWrite(motor2speedpin, stopSpeed);

  //left motor
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
         
  //right motor 
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  }

  delay(100); // Delay for stability
}
