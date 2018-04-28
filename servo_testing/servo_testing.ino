#include <Servo.h>
#define PIN 3

Servo servo;
int i = 0;

void setup() {
  servo.attach(PIN);
}

void loop() {
  for(i = 0; i < 179; i++){
    servo.write(i);
    delay(2);
  }
  delay(1000);
  
  for(i = 179; i > 0; i--){
    servo.write(i);
    delay(2);
  } 
  delay(1000);
}
