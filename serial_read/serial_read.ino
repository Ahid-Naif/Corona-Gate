// include needed libraries
#include <Servo.h>

// define where electronics elements are connected with the Arduino
int red = 2;
int green = 3;
int blue = 4;
int buzzer = 7;

Servo myservo; // create servo object to control a servo

void setup() {
  // initialize Serial connection with the Raspberry Pi at 9600 buad rate
  Serial.begin(9600);

  // define elements as OUTPUT elements
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  myservo.attach(10); // attaches the servo on pin 10 to the servo object
}
void loop() {
  // if there is a command/message coming from the raspberry pi
  if (Serial.available() > 0) {
    // read command/message recived from the raspberry pi
    String data = Serial.readStringUntil('\n');

    // if command is red
    if(data == "red")
    {       
      // turn red light on 
      digitalWrite(red, HIGH);
      digitalWrite(green, LOW);
      digitalWrite(blue, LOW);
    }
    // if command is green
    else if(data == "green")
    {
      // turn green light on
      digitalWrite(red, LOW);
      digitalWrite(green, HIGH);
      digitalWrite(blue, LOW);
    }
    // if command is white
    else if(data == "white")
    {
      // turn white light on
       digitalWrite(red, HIGH);
      digitalWrite(green, HIGH);
      digitalWrite(blue, HIGH);
    }
    // if command is buzzerON
    else if(data == "buzzerON")
    {
      // turn buzzer on
      digitalWrite(buzzer, HIGH);
    }
     // if command is buzzerOFF
    else if(data == "buzzerOFF")
    {
      // turn buzzer off
      digitalWrite(buzzer, LOW);
    }
    // if command is buzzerON_red
    else if(data == "buzzerON_red")
    {
      // turn buzzer on
      digitalWrite(buzzer, HIGH);

      // turn red light
      digitalWrite(red, HIGH);
      digitalWrite(green, LOW);
      digitalWrite(blue, LOW);
    }
    // if command is openGate
    else if(data == "openGate")
    {
      // move servo motor to 180 degrees position
      myservo.write(180);
      delay(2000);
    }
    // if command is closeGate
    else if(data == "closeGate")
    {
      // move servo motor to 100 degrees position
      myservo.write(100);
      delay(2000);
    }
  }
}
