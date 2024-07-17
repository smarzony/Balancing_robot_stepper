#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <AccelStepper.h>

#define DOUBLE_SIZEOF 4
#define UINT8_T_SIZEOF 2

#define ADDR_P 0
#define ADDR_I (ADDR_P + DOUBLE_SIZEOF)
#define ADDR_D (ADDR_I + DOUBLE_SIZEOF)
#define ADDR_ANGLE (ADDR_D + DOUBLE_SIZEOF)
#define ADDR_VELOCITY_LIMIT_MIN (ADDR_ANGLE + DOUBLE_SIZEOF)
#define ADDR_VELOCITY_LIMIT_MAX (ADDR_VELOCITY_LIMIT_MIN + DOUBLE_SIZEOF)
#define ADDR_ANGLE_BALANCE_SPAN (ADDR_VELOCITY_LIMIT_MAX + DOUBLE_SIZEOF)

#define GYRO_INTERVAL 5
#define SERIAL_INTERVAL 50

#define PIN_ENABLE 6

#define PID_BALANCING_SAMPLE_TIME_MS 50 // originally 50 ms

// KEYBOARD CONTROL
uint16_t adc_key_val[5] = { 50, 200, 400, 600, 800 };
int8_t NUM_KEYS = 5;
int8_t adc_key_in;
int8_t key = -1;
int8_t oldkey = -1;

bool enable_balancing = false;
bool disable_serial = true;
bool motor_test_run = false;

// Motors
AccelStepper motor1(AccelStepper::DRIVER, 2, 4);
AccelStepper motor2(AccelStepper::DRIVER, 3, 5);

// PID
double Setpoint_angle, Input_angle, Output_motor_speed, Kp_balancing, Ki_balancing, Kd_balancing;
double Velocity_limit_min, Velocity_limit_max, Angle_balance_span;

PID balancePID(&Input_angle, &Output_motor_speed, &Setpoint_angle, Kp_balancing, Ki_balancing, Kd_balancing, DIRECT);

// GYRO & KALMAN
MPU6050 mpu;

// Konfiguracja filtru Kalmana dla osi X i Y (kat, odchylka, pomiar)
KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

// Obliczone wartosci Pitch i Roll tylko z akcelerometru
float accPitch = 0;
float accRoll = 0;

// Obliczone wartosci Pitch i Roll z uwzglednieniem filtru Kalmana i zyroskopu
float kalPitch = 0;
float kalRoll = 0;

unsigned long now, gyro_timer, serial_timer;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  EEPROM.get(ADDR_P, Kp_balancing);
  EEPROM.get(ADDR_I, Ki_balancing);
  EEPROM.get(ADDR_D, Kd_balancing);
  EEPROM.get(ADDR_ANGLE, Setpoint_angle);
  EEPROM.get(ADDR_VELOCITY_LIMIT_MIN, Velocity_limit_min);
  EEPROM.get(ADDR_VELOCITY_LIMIT_MAX, Velocity_limit_max);
  EEPROM.get(ADDR_ANGLE_BALANCE_SPAN, Angle_balance_span);
  // Angle_balance_span = 20;

  pinMode(PIN_ENABLE, OUTPUT);

  balancePID.SetTunings(Kp_balancing, Ki_balancing, Kd_balancing);
  balancePID.SetOutputLimits(-Velocity_limit_min, Velocity_limit_max);

  // GYRO
  // Inicjalizacja MPU6050
  Serial.println("Inicjalizacja MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Nie znaleziono MPU6050!");
    delay(500);
  }

  // Kalibracja zyroskopu
  mpu.calibrateGyro();

    // Konfiguracja silników
  motor1.setMaxSpeed(2000);
  motor1.setAcceleration(2000);
  motor2.setMaxSpeed(2000);
  motor2.setAcceleration(2000);

  balancePID.SetMode(AUTOMATIC);                           //PID is set to automatic mode
  balancePID.SetSampleTime(PID_BALANCING_SAMPLE_TIME_MS);  //Set PID sampling frequency is 100ms
  balancePID.SetOutputLimits(-Velocity_limit_min, Velocity_limit_max);

  delay(200);
  digitalWrite(PIN_ENABLE, HIGH);

}

void loop() {
  now = millis();
  keyboard_read();
  commandEngine();
  if (now - gyro_timer > GYRO_INTERVAL) {
    gyro_timer = now;
    read_gyro_kalman();
  }

  if (enable_balancing) {
	digitalWrite(PIN_ENABLE, LOW);
    // manual_go = false;
    Input_angle = kalPitch;
    // Input_angle = Setpoint_angle-5;
    // motor_right_setpoint_speed = Output_motor_speed;
    // motor_left_setpoint_speed = Output_motor_speed;
    balancePID.Compute();
    // Motors movement
    motor1.setSpeed(-Output_motor_speed);
    motor2.setSpeed(Output_motor_speed);
    motor1.runSpeed();
    motor2.runSpeed();
    if (kalPitch < Setpoint_angle - Angle_balance_span || kalPitch > Setpoint_angle + Angle_balance_span) {
		enable_balancing = false;
		digitalWrite(PIN_ENABLE, HIGH);

		motor1.setSpeed(0);
		motor2.setSpeed(0);
		motor1.runSpeed();
		motor2.runSpeed();   
		Serial.println("Angle span exceeded!");
    }
  } else {
		digitalWrite(PIN_ENABLE, HIGH);

		motor1.setSpeed(0);
		motor2.setSpeed(0);
		motor1.runSpeed();
		motor2.runSpeed();     

  }

  if ((now - serial_timer > SERIAL_INTERVAL) and !disable_serial) {
    serial_timer = now;
    serial_data();
  }
}

void serial_data() {
  Serial.print("E:");
  Serial.print(kalPitch - Setpoint_angle);
  Serial.print(",");
  Serial.print("Out_spd:");
  Serial.print(Output_motor_speed);
  Serial.print(",");
  Serial.println();
}