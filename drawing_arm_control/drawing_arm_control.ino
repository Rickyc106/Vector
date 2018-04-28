#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <SPI.h>

#define N (2)

const int MPU_9250_address = 0x68;
const double RAD_TO_DEGREES = 180 / PI;
const double R_measure = 5;

int i,j;
int16_t Temp;
int16_t ax,ay,az,gx,gy,gz;
float a_x,a_y,a_z,g_x,g_y,g_z;
float Current_Temperature;

double previous,sample_Rate;

double var_x = 0, var_y = 0, var_z = 0,var_p = 0,var_r = 0;
double dt = 0.012;
double pitch, roll, yaw;
double ax_mean, ay_mean, az_mean;
double gyr_x_mean = 0,gyr_y_mean = 0,gyr_z_mean = 0;

double x_p[2][1], x_r[2][1], x_y[2][1], P[2][2], K[2][1];
double Q_angle = 0.01, Q_bias = 0.01; 
double y_p, y_r, y_y, S;

double comp_pitch, comp_roll;
double alpha = 0.97;

double gyro_pitch, gyro_roll, gyro_yaw;

double rate;

double pitch_p_gain, pitch_i_gain, pitch_d_gain;
double roll_p_gain, roll_i_gain, roll_d_gain;
double pitch_reference_state, roll_reference_state;
double max_angle;
double val;
double throttle = 0;
double pitch_mapped, roll_mapped;
double pitch_stick, roll_stick;
int counter;
int pot_pin;
//int esc_pin[4];
double esc_input[4];
//Servo esc[4];
Servo esc_1, esc_2, esc_3, esc_4;
#define esc_pin 4

double eps = 0.02;
double pitch_error, roll_error;
double pitch_sum, roll_sum;
double pitch_output, roll_output;
double pitch_previous, roll_previous;

bool test = true;

//----------------------------------------------------------------------------------------------------------------


#define led_Pin 3
#define servo_Pin A2
#define joystick_X_Pin A0
#define joystick_Y_Pin A1
#define joystick_Switch_Pin 4

#define CE_pin 9
#define CSN_pin 10
#define MOSI_pin 11
#define MISO_pin 12
#define SCK_pin 13
#define IRQ_pin 0 // IRQ pinout of 0 will read from pin 2 on Arduino Uno

byte data_in[5], data_2, Switch_Input, roll_input, pitch_input, throttle_input;
uint8_t bytes = 2;
int state = 1;

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_9250_address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);
  Serial.println("Booting up!");

  delay(5);

  calibrate_Data();
  
  delay(5);
  
  //--------------------------------------------------------------------------------------------------------------

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
  previous = millis();
  Wire.beginTransmission(MPU_9250_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_9250_address, 20, true);
  
  ax = Wire.read()<<8 | Wire.read();
  ay = Wire.read()<<8 | Wire.read();
  az = Wire.read()<<8 | Wire.read();
  Temp = Wire.read()<<8 | Wire.read();
  gx = Wire.read()<<8 | Wire.read();
  gy = Wire.read()<<8 | Wire.read();
  gz = Wire.read()<<8 | Wire.read();

  a_x = float (ax) / 16384;
  a_y = float (ay) / 16384;
  a_z = float (az) / 16384;

  g_x = float (gx) / 131;
  g_y = float (gy) / 131;
  g_z = float (gz) / 131;

  pitch = atan2(a_y,a_z) * RAD_TO_DEGREES;
  roll = atan2(a_x,a_z) * RAD_TO_DEGREES;
  
  //yaw = (g_z - gyr_z_mean) * (dt / RAD_TO_DEGREES);                      // Yaw angle in RADIANS

  rate = (g_x - gyr_x_mean) - x_p[1][0];                                 // First Update State Matrix Estimations
  x_p[0][0] += dt * (rate);
  //x_p[0][0] -= x_r[0][0] * sin(yaw);
  gyro_pitch = x_p[0][0];

  rate = (g_y - gyr_y_mean) - x_r[1][0];
  x_r[0][0] += dt * (rate);
  //x_r[0][0] += x_p[0][0] * sin(yaw);
  gyro_roll = x_r[0][0];
  
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * (P[1][1]);                              // Error Covariance Matrix
  P[1][0] -= dt * (P[1][1]);
  P[1][1] += dt * (Q_bias);

  y_p = pitch - x_p[0][0];                                // Difference between measured values and estimation
  y_r = roll - x_r[0][0];
  //y_y = yaw - x_y[0][0];

  S = P[0][0] + R_measure;
  K[0][0] = P[0][0] / S;                                  // Kalman gain for Angles     (same for both pitch and roll angles)
  K[1][0] = P[1][0] / S;                                  // Kalman gain for gyro Bias

  x_p[0][0] += K[0][0] * y_p;                             // Update both pitch and roll vectors 
  x_r[0][0] += K[0][0] * y_r;
  //x_y[0][0] += K[0][0] * y_y;

  x_p[1][0] += K[1][0] * y_p;
  x_r[1][0] += K[1][0] * y_r;
  //x_y[1][0] += K[1][0] * y_y;

  P[0][0] -= K[0][0] * P[0][0];                           // Update Error Covariance Matrix
  P[0][1] -= K[0][0] * P[0][1];
  P[1][0] -= K[1][0] * P[0][0];
  P[1][1] -= K[1][0] * P[0][1];

  comp_pitch = alpha * (comp_pitch + (g_x - gyr_x_mean) * dt) + ((1 - alpha) * pitch);
  comp_roll = alpha * (comp_roll + (g_y - gyr_y_mean) * dt) + ((1 - alpha) * roll);

  //if (abs(x_y[0][0] - yaw) < 0.01) x_y[0][0] = 0;
  

  //Serial.print(x_p[0][0]);
  //Serial.print("\t");
  //Serial.print(pitch);
  //Serial.print("\t");
  //Serial.print(roll);
  //Serial.print("\t");
  //Serial.print(yaw);
  //Serial.print(x_r[0][0]);
  //Serial.print("\t");
  //Serial.print(yaw);
  //Serial.print(x_y[0][0]);
  //Serial.print("\t");
  //Serial.print(roll);
  //Serial.print("\t");
  //Serial.print(gyro_pitch);
  //Serial.print(gyro_roll);
  //Serial.print("\t");
  //Serial.print(comp_roll);
  //Serial.println("\t");
  
  //Serial.print("\t");
  //Serial.println(throttle);

  //throttle = map(val, 0, 1023, 1000, 2000);
  

  //esc.writeMicroseconds(throttle);

  Serial.print(comp_pitch);
  Serial.print("\t");
  Serial.println(comp_roll);

  NRF_control(comp_pitch, comp_roll);

  Current_Temperature = float (Temp)/340.00 + 36.53;
  String reading = String(Current_Temperature,3);
  
  sample_Rate = millis() - previous;
  dt = sample_Rate / 1000;

  //Serial.println(dt);
}


void calibrate_Data(){
  double pitch_mean = 0, roll_mean = 0;
  double x_bias = 0, y_bias = 0, z_bias = 0;
  double ax_bias = 0, ay_bias = 0, az_bias = 0;
  
  for (i = 0; i < 100; i++){
    Wire.beginTransmission(MPU_9250_address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_9250_address,20,true);
    
    ax = Wire.read()<<8 | Wire.read();
    ay = Wire.read()<<8 | Wire.read();
    az = Wire.read()<<8 | Wire.read();
    Temp = Wire.read()<<8 | Wire.read();
    gx = Wire.read()<<8 | Wire.read();
    gy = Wire.read()<<8 | Wire.read();
    gz = Wire.read()<<8 | Wire.read();

    a_x = float (ax) / 16384;
    a_y = float (ay) / 16384;
    a_z = float (az) / 16384;

    g_x = float (gx) / 131;
    g_y = float (gy) / 131;
    g_z = float (gz) / 131;

    ax_bias += a_x;
    ay_bias += a_y;
    az_bias += a_z;
    
    x_bias = x_bias + g_x;
    y_bias = y_bias + g_y;
    z_bias = z_bias + g_z;
  }

  //pitch_mean = pitch_sum / 100;
  //roll_mean = roll_sum / 100;

  ax_mean = ax_bias / 100;
  ay_mean = ay_bias / 100;
  az_mean = (az_bias / 100) - 1.0;

  gyr_x_mean = x_bias / 100;
  gyr_y_mean = y_bias / 100;
  gyr_z_mean = z_bias / 100;
}

void NRF_Init() {
  pinMode(CE_pin, OUTPUT);
  pinMode(CSN_pin, OUTPUT);
  pinMode(MOSI_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  pinMode(SCK_pin, OUTPUT);

  pinMode(led_Pin, OUTPUT);
  pinMode(joystick_Switch_Pin, INPUT_PULLUP);

  //Serial.println("NRF Pins Initialized");
  SPI.setBitOrder(MSBFIRST); // Most Significant Bit First
  SPI.setDataMode(SPI_MODE0); // Data capture on rising edge of Serial Clock
  // SPI.setClockDivider(SPI_CLOCK_DIV4); // Runs the data in at 16 MHz / 4 (4 MHz)
  digitalWrite(CE_pin, HIGH); // sets to RX mode
  digitalWrite(CSN_pin, HIGH); // Chip Select is active-low. CSN_pin set to high sets SPI to idle
  SPI.begin();
  //Serial.println("Radio ready");
}

void NRF_set_RX_Payload(byte pipe, byte bytes) {
  byte address = pipe + 32 + 16 + 1; // write register starts at 32. Add 16 and 1 to get to RX_PW_P# specified on datasheet
  digitalWrite(CSN_pin, LOW); // CSN_pin set to low sets SPI to active
  data_in[0] = SPI.transfer(address);
  data_in[1] = SPI.transfer(bytes);
  digitalWrite(CSN_pin, HIGH); //CSN_pin set to high sets SPI to idle
  //Serial.print("RX Payload configured for RX_PW_P");
  //Serial.print(pipe);
  //Serial.print(" with payload size of ");
  //Serial.print(bytes);
  //Serial.println(" bytes");
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

  //data_in[2] = SPI.transfer(data_in[2]);
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
  data_in[4] = SPI.transfer(B00000000);
  digitalWrite(CSN_pin, HIGH);

  if (data_in[1] == 0) {
    delay(10);
    Switch_Input = data_in[2];

    digitalWrite(led_Pin, Switch_Input);

    //Serial.print(Switch_Input);
  }
  else if (data_in[1] == 1) {
    delay(10);
    roll_input = data_in[2];
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
    throttle_input = data_in[2];
  }
  else if (data_in[1] == 5) {
    delay(10);
    pitch_input = data_in[2];
  }
  else if (data_in[1] == 6) {
    delay(10);
    pitch_p_gain += 0.1;
  }
  else if (data_in[1] == 7) {
    delay(10);
    pitch_i_gain += 0.1;
  }
  else if (data_in[1] == 8) {
    delay(10);
    pitch_d_gain += 0.1;
  }

  throttle = map(throttle_input, 0, 255, 1000, 2000);
                                                                                                                                                                                                                                                                                                               
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

void NRF_control(double pitch, double roll) {
  pitch = map(pitch, -180, 180, 0, 255);
  roll = map(roll, -180, 180, 0, 255);
  
  NRF_transmit(4, pitch);
  NRF_transmit(5, roll);
}

