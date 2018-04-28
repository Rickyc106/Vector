// MD03A_Motor_basic
// Test MD03a / Pololu motor

//variables
#define M1DIR 7 // Motor 1 Direction Pin -- Low: CW with Shaft facing person. High: CCW
#define M1PWM 9 // PWM motor pin -- Controls speed (0 -> 255)
#define D2 4 // Toggles Motor Channel Output -- Default LOW. Set High to enable motor output.

void setup()
{
  Serial.begin(9600);
  Serial.println("Dual MC33926 Motor Shield");
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  
  pinMode(D2, OUTPUT);
  digitalWrite(D2,HIGH);
}

void loop()
{
  motorForward(50); // (25%=64; 50%=127; 100%=255)
}

void motorForward(int PWM_val) 
{
  // Speed
  analogWrite(M1PWM, PWM_val);
  // Direction HIGH and LOW opposite directions
  digitalWrite(M1DIR, LOW);
}



