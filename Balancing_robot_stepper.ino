#include <Wire.h>
#include <MPU6050_light.h>
#include <KalmanFilter.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <FastAccelStepper.h>

#define DOUBLE_SIZEOF 4
#define UINT8_T_SIZEOF 2

#define ADDR_P 0
#define ADDR_I (ADDR_P + DOUBLE_SIZEOF)
#define ADDR_D (ADDR_I + DOUBLE_SIZEOF)
#define ADDR_ANGLE (ADDR_D + DOUBLE_SIZEOF)
#define ADDR_VELOCITY_LIMIT_MIN (ADDR_ANGLE + DOUBLE_SIZEOF)
#define ADDR_VELOCITY_LIMIT_MAX (ADDR_VELOCITY_LIMIT_MIN + DOUBLE_SIZEOF)
#define ADDR_ANGLE_BALANCE_SPAN (ADDR_VELOCITY_LIMIT_MAX + DOUBLE_SIZEOF)

#define GYRO_INTERVAL 2 // Zmniejsz z 5 na 2 ms
#define SERIAL_INTERVAL 50

#define PIN_ENABLE 6

#define PID_BALANCING_SAMPLE_TIME_MS 20 // Zmniejsz ze 100 na 20 ms

#define dirPinStepperL    4
#define enablePinStepperL PIN_ENABLE
#define stepPinStepperL   9

#define dirPinStepperR    5
#define enablePinStepperR PIN_ENABLE
#define stepPinStepperR   10

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
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperL = NULL;
FastAccelStepper *stepperR = NULL;

// PID
double Setpoint_angle, Input_angle, Output_motor_speed, Kp_balancing, Ki_balancing, Kd_balancing;
double Velocity_limit_min, Velocity_limit_max, Angle_balance_span;

PID balancePID(&Input_angle, &Output_motor_speed, &Setpoint_angle, Kp_balancing, Ki_balancing, Kd_balancing, DIRECT);

// GYRO & KALMAN
MPU6050 mpu(Wire);

float kalPitch = 0;

unsigned long now, gyro_timer, serial_timer;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (EEPROM.get(ADDR_P, Kp_balancing) != DOUBLE_SIZEOF) {
    Serial.println("Błąd odczytu EEPROM dla Kp_balancing");
  }
  if (EEPROM.get(ADDR_I, Ki_balancing) != DOUBLE_SIZEOF) {
    Serial.println("Błąd odczytu EEPROM dla Ki_balancing");
  }
  if (EEPROM.get(ADDR_D, Kd_balancing) != DOUBLE_SIZEOF) {
    Serial.println("Błąd odczytu EEPROM dla Kd_balancing");
  }
  if (EEPROM.get(ADDR_ANGLE, Setpoint_angle) != DOUBLE_SIZEOF) {
    Serial.println("Błąd odczytu EEPROM dla Setpoint_angle");
  }
  if (EEPROM.get(ADDR_VELOCITY_LIMIT_MIN, Velocity_limit_min) != DOUBLE_SIZEOF) {
    Serial.println("Błąd odczytu EEPROM dla Velocity_limit_min");
  }
  if (EEPROM.get(ADDR_VELOCITY_LIMIT_MAX, Velocity_limit_max) != DOUBLE_SIZEOF) {
    Serial.println("Błąd odczytu EEPROM dla Velocity_limit_max");
  }
  if (EEPROM.get(ADDR_ANGLE_BALANCE_SPAN, Angle_balance_span) != DOUBLE_SIZEOF) {
    Serial.println("Błąd odczytu EEPROM dla Angle_balance_span");
  }

  pinMode(PIN_ENABLE, OUTPUT);

  balancePID.SetTunings(Kp_balancing, Ki_balancing, Kd_balancing);
  balancePID.SetOutputLimits(-Velocity_limit_min, Velocity_limit_max);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done!");

  // FastAccelStepper setup
   engine.init();
   stepperL = engine.stepperConnectToPin(stepPinStepperL);
   stepperR = engine.stepperConnectToPin(stepPinStepperR);
   if (stepperL) {
      stepperL->setDirectionPin(dirPinStepperL);
      stepperL->setEnablePin(enablePinStepperL);
      stepperL->setAutoEnable(false);

      stepperL->setSpeedInHz(2000); // Zwiększ z 1000 na 2000
      stepperL->setAcceleration(200000); // Zwiększ ze 100000 na 200000
   }

      if (stepperR) {
      stepperR->setDirectionPin(dirPinStepperR);
      stepperR->setEnablePin(enablePinStepperR);
      stepperR->setAutoEnable(false);

      stepperR->setSpeedInHz(2000);
      stepperR->setAcceleration(200000);
   }

  balancePID.SetMode(AUTOMATIC);
  balancePID.SetSampleTime(PID_BALANCING_SAMPLE_TIME_MS);
  balancePID.SetOutputLimits(-Velocity_limit_min, Velocity_limit_max);

  delay(200);
  digitalWrite(PIN_ENABLE, HIGH);
}

void loop() {
  now = millis();
  //keyboard_read();
  commandEngine();

  if (now - gyro_timer > GYRO_INTERVAL) {
    mpu.update();
    gyro_timer = now;
    kalPitch = mpu.getAngleY();
  }

  if (enable_balancing) {
    digitalWrite(PIN_ENABLE, LOW);
    Input_angle = kalPitch;

    balancePID.Compute();

    // Przesuwanie silników tylko gdy konieczne
    if (stepperL && stepperR) {
      stepperL->setSpeedInHz(uint32_t(abs(Output_motor_speed)));
      stepperR->setSpeedInHz(uint32_t(abs(Output_motor_speed)));

      if (Output_motor_speed < 0) {
        stepperL->runForward();
        stepperR->runBackward();
      } else {
        stepperL->runBackward();
        stepperR->runForward();
      }
    }

    if (kalPitch < Setpoint_angle - Angle_balance_span || kalPitch > Setpoint_angle + Angle_balance_span) {
      disableBalancing();
    }
  } else {
    stopMotors();
  }

  if ((now - serial_timer > SERIAL_INTERVAL) && !disable_serial) {
    serial_timer = now;
    serial_data();
  }

  delay(1);
}

void stopMotors() {
  if (stepperL && stepperR) {
    stepperL->forceStop();
    stepperR->forceStop();
  }
  digitalWrite(PIN_ENABLE, HIGH);
}

void disableBalancing() {
  enable_balancing = false;
  stopMotors();
  Serial.println("Angle span exceeded!");
  digitalWrite(PIN_ENABLE, HIGH);
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