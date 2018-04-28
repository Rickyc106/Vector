#include <SPI.h>
#include <Servo.h>

#define led_Pin 3
#define servo_Pin A2
#define joystick_X_Pin A0
#define joystick_Y_Pin A1
#define pot_pin A3
#define joystick_Switch_Pin 4

#define CE_pin 9
#define CSN_pin 10
#define MOSI_pin 11
#define MISO_pin 12
#define SCK_pin 13
#define IRQ_pin 0 // IRQ pinout of 0 will read from pin 2 on Arduino Uno

#define MOTOR_PIN 6   // CHOOSE PWM PIN

byte data_in[5], data_2, Switch_Input, Analog_Input_X, Analog_Input_Y, Analog_Input_X2, Analog_Input_Y2, throttle;
uint8_t bytes = 2;

Servo mySwervo;
byte servo_Previous;
int sum = 0;
double pitch, roll;
double pitch_previous, roll_previous;
double arm_reference_state, base_reference_state;

int timer, read;
int state = 1;
int debounce = 200;
int previous = 0;

double k_p, k_i, k_d;
double pitch_error, roll_error;
double pitch_sum, roll_sum;
double arm_output, base_output;
double arm_base_ratio, base_arm_ratio;

int ON = 180;   // <-- CHANGE THESE LATER
int OFF = 0;

Servo shaft_servos[4];

/******
 * Servo 1 -> Arm Forwards
 * Servo 2 -> Arm Backwards
 * Servo 3 -> Base Forwards
 * Servo 4 -> Base Backwards
 ******/

void setup() {
  Serial.begin(115200);
  Serial.println("Booting Up");
  delay(100);
  NRF_Init();
  NRF_set_RX_Payload(0, bytes);
  NRF_alter_bit(0, 0, 1); // register address#, bit#, state 0 or 1  ::  0,0,1 RX Mode
  NRF_alter_bit(0, 1, 1); // 0,1,1 PowerUp
  NRF_alter_bit(0, 4, 1); // Turns off Mask RT interrupt
  NRF_alter_bit(0, 5, 1); // Turns off Mask TX interrupt

  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(B11100010); // Flush RX FIFO
  digitalWrite(CSN_pin, HIGH);
  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(B11100001); // Flush TX FIFO
  digitalWrite(CSN_pin , HIGH);

  NRF_clear_interrupts(7);
  delay(100);
  attachInterrupt(0, NRF_receive, FALLING);
}

void loop() {
//  Serial.print("Pitch: ");
//  Serial.print(pitch);
//  Serial.print(" Roll: ");
//  Serial.println(roll);

  PID_sub_routine(pitch, roll);
  Serial.print("arm: ");
  Serial.print(arm_output);
  Serial.print(" base: ");
  Serial.println(base_output);
}

void NRF_Init() {
  pinMode(CE_pin, OUTPUT);
  pinMode(CSN_pin, OUTPUT);
  pinMode(MOSI_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  pinMode(SCK_pin, OUTPUT);

  pinMode(led_Pin, OUTPUT);
  pinMode(joystick_Switch_Pin, INPUT_PULLUP);

  mySwervo.attach(servo_Pin);

  Serial.println("NRF Pins Initialized");
  SPI.setBitOrder(MSBFIRST); // Most Significant Bit First
  SPI.setDataMode(SPI_MODE0); // Data capture on rising edge of Serial Clock
  //SPI.setClockDivider(SPI_CLOCK_DIV4); // Runs the data in at 16 MHz / 4 (4 MHz)
  digitalWrite(CE_pin, HIGH); // sets to RX mode
  digitalWrite(CSN_pin, HIGH); // Chip Select is active-low. CSN_pin set to high sets SPI to idle
  SPI.begin();
  Serial.println("Radio ready");
}

void NRF_set_RX_Payload(byte pipe, byte bytes) {
  byte address = pipe + 32 + 16 + 1; // write register starts at 32. Add 16 and 1 to get to RX_PW_P# specified on datasheet
  digitalWrite(CSN_pin, LOW); // CSN_pin set to low sets SPI to active
  data_in[0] = SPI.transfer(address);
  data_in[1] = SPI.transfer(bytes);
  digitalWrite(CSN_pin, HIGH); //CSN_pin set to high sets SPI to idle
  Serial.print("RX Payload configured for RX_PW_P");
  Serial.print(pipe);
  Serial.print(" with payload size of ");
  Serial.print(bytes);
  Serial.println(" bytes");
}

void NRF_alter_bit(byte address, byte specific_bit, byte state) {
  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(address);
  data_in[1] = SPI.transfer(B00000000);
  digitalWrite(CSN_pin, HIGH);

  if (state == 1)
    bitSet(data_in[1], specific_bit);
  else
    bitClear(data_in[1], specific_bit);

  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(32 + address);
  data_in[1] = SPI.transfer(data_in[1]);
  digitalWrite(CSN_pin, HIGH);
}

void NRF_transmit(byte code, byte val) {
  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(B11100001); // Flush TX
  digitalWrite(CSN_pin, HIGH);

  digitalWrite(CSN_pin, LOW);
  //data_in[0] = SPI.transfer(B00010000); // Transmit address. Used for PTX device only.
  data_in[0] = SPI.transfer(B10100000); // Call to write TX payload
  data_in[1] = SPI.transfer(code); // Transmit code
  data_in[2] = SPI.transfer(val); // Transmit byte_data
  digitalWrite(CSN_pin, HIGH);

  digitalWrite(CE_pin, LOW);
  delay(1);
  NRF_alter_bit(0, 0, 0); // Change to TX mode
  delay(1);
  digitalWrite(CE_pin, HIGH);
  delay(1);
  NRF_alter_bit(0, 0, 1); // Change back to RX mode;
}

void NRF_receive() {
  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(B01100001); // Call to read RX payload
  data_in[1] = SPI.transfer(B00000000); // data_in[1] is the code
  data_in[2] = SPI.transfer(B00000000); // data_in[2] is the byte_data
  data_in[3] = SPI.transfer(B00000000);
  digitalWrite(CSN_pin, HIGH);

  if (data_in[1] == 0) {
    delay(10);
    Switch_Input = data_in[2];

    digitalWrite(led_Pin, Switch_Input);

    //Serial.print(Switch_Input);
  }
  else if (data_in[1] == 1) {
    delay(10);
    Analog_Input_X = data_in[2];
    Analog_Input_Y = data_in[3];

    Analog_Input_X2 = map(Analog_Input_X, 0, 255, 0, 180);
    Analog_Input_Y2 = map(Analog_Input_Y, 0, 255, 0, 180);
    
    mySwervo.write(Analog_Input_X2);
    //analogWrite(servo_Pin, Analog_Input_X);
  }
  else if (data_in[1] == 2) {
    delay(10);
    data_2 = data_in[2];
    NRF_transmit(3, data_2);
  }
  else if (data_in[1] == 3) {
    delay(10);
    data_2 = data_in[2];
  }
  else if (data_in[1] == 4) {
    delay(10);
    pitch = data_in[2];
    pitch = map(pitch, 0, 255, -180, 180);
  }
  else if (data_in[1] == 5) {
    delay(10);
    roll = data_in[2];
    roll = map(roll, 0, 255, -180, 180);
  }

  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(B11100010); // Flush RX
  digitalWrite(CSN_pin, HIGH);

  NRF_alter_bit(7, 6, 1); // Clear RX Interrupt
}

void NRF_ping() {
  byte ping;
  ping = random(255);
  Serial.print("Pinging with\t");
  Serial.print(ping);

  NRF_transmit(2, ping);

  delay(15);
  if (data_2 == ping)
    Serial.println(" PING SUCCESSFUL! ");
  else
    Serial.println(" PING FAILED! ");
  Serial.println(data_2);
  data_2 = 0;
}

void NRF_clear_interrupts(byte address) {
  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(address);
  data_in[1] = SPI.transfer(B00000000);
  digitalWrite(CSN_pin, HIGH);

  if (bitRead(data_in[1], 4))
    NRF_alter_bit(7, 4, 1); // Clear RT interrupt

  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(address);
  data_in[1] = SPI.transfer(B00000000);
  digitalWrite(CSN_pin, HIGH);

  if (bitRead(data_in[1], 5))
    NRF_alter_bit(7, 5, 1); // Clear TX interrupt

  digitalWrite(CSN_pin, LOW);
  data_in[0] = SPI.transfer(address);
  data_in[1] = SPI.transfer(B00000000);
  digitalWrite(CSN_pin, HIGH);

  if (bitRead(data_in[1], 6))
    NRF_alter_bit(7, 6, 1); // Clear RX interrupt
}

void PID_sub_routine(double pitch, double roll){
  k_p = 1.0;
  k_i = 0.0;
  k_d = 0.0;

  pitch_error = pitch - arm_reference_state;
  roll_error = roll - base_reference_state;

  pitch_sum += pitch_error;
  roll_sum += roll_sum;

  if (pitch_sum > 50) pitch_sum = 50;
  else if (pitch_sum < -50) pitch_sum = -50;

  if (roll_sum > 50) roll_sum = 50;
  else if (roll_sum < -50) roll_sum = -50;

  arm_output = (k_p * pitch_error) + (k_i * pitch_sum) + (k_d * (pitch_error - pitch_previous));
  base_output = (k_p * roll_error) + (k_i * roll_sum) + (k_d * (roll_error - roll_previous));

  // Specify duty cycle for PWM synchro control
  if (base_output != 0) arm_base_ratio = arm_output / base_output;
  if (arm_output != 0) base_arm_ratio = base_output / arm_output;

  // Notes -- Arduino PWM Frequency (500 Hz) -> Period (2 ms)

  if (abs(arm_output) >= abs(base_output) && arm_output > 0) {
    analogWrite(MOTOR_PIN, arm_output);
    toggle(1, 2, 1);
    PWM_synchro_control(3, 4);
  }
  else if (abs(arm_output) > abs(base_output) && arm_output < 0) {
    analogWrite(MOTOR_PIN, arm_output);
    toggle(1, 2, 2);
    PWM_synchro_control(3, 4);
  }
  else if (abs(base_output) > abs(arm_output) && base_output > 0) {
    analogWrite(MOTOR_PIN, base_output);
    toggle(3, 4, 1);
    PWM_synchro_control(1, 2);
  }
  else if (abs(base_output) > abs(arm_output) && base_output < 0) {
    analogWrite(MOTOR_PIN, base_output);
    toggle(3, 4, 2);
    PWM_synchro_control(1, 2);
  }

  pitch_previous = pitch;
  roll_previous = roll;
}

/*****
 * Condition Mapping:
 * 1 -> Forward, no reverse
 * 2 -> Reverse, no forward
 * 3 -> Both off
 *****/

void toggle(int s1, int s2, int condition) {
  if (condition == 1) {
    shaft_servos[s1].write(ON);
    shaft_servos[s2].write(OFF);
  }
  else if (condition == 2) {
    shaft_servos[s1].write(OFF);
    shaft_servos[s2].write(ON);
  }
  else if (condition == 3) {
    shaft_servos[s1].write(OFF);
    shaft_servos[s2].write(OFF);
  }
}

void PWM_synchro_control(int s1, int s2) {
  if (s1 == 3 && base_output > 0){
    toggle(s1, s2, 1);
    delayMicroseconds(base_arm_ratio * 2000);
    toggle(s1, s2, 3);
  }
  else if (s1 == 3 && base_output < 0){
    toggle(s1, s2, 2);
    delayMicroseconds(base_arm_ratio * 2000);
    toggle(s1, s2, 3);
  }
  else if (s1 == 1 && arm_output > 0){
    toggle(s1, s2, 1);
    delayMicroseconds(arm_base_ratio * 2000);
    toggle(s1, s2, 3);
  }
  else if (s1 == 1 && arm_output < 0){
    toggle(s1, s2, 2);
    delayMicroseconds(arm_base_ratio * 2000);
    toggle(s1, s2, 3);
  }
}

