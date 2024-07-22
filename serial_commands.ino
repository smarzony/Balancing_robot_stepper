void commandEngine() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "PRINT") {
      functionPrint();
    } else if (command == "BAL") {
      functionBalanceEnable();
    } else if (command == "SERIAL") {
      functionSerialEnable();
    } else if (command == "ANG_SET") {
      functionAngleCurrent();
    } else if (command.startsWith("P ")) {
      functionP(command.substring(2).toFloat());
    } else if (command.startsWith("I ")) {
      functionI(command.substring(2).toFloat());
    } else if (command.startsWith("D ")) {
      functionD(command.substring(2).toFloat());
    // } else if (command.startsWith("ANGLE ")) {
    //   functionAngle(command.substring(6).toFloat());
    } else if (command.startsWith("SPAN ")) {
      functionAngleSpan(command.substring(5).toFloat());
    } else if (command.startsWith("V_LIM ")) {
      handleVLim(command);
    } else {
      Serial.println("Nieznana komenda!");
    }
  }
}

void handleVLim(const String &command) {
  int firstSpace = command.indexOf(' ', 6);
  if (firstSpace != -1) {
    int val1 = command.substring(6, firstSpace).toInt();
    int val2 = command.substring(firstSpace + 1).toInt();
    functionVLim(val1, val2);
  } else {
    Serial.println("Nieprawidłowe argumenty!");
  }
}

void functionP(double arg) {
  Kp_balancing = arg;
  EEPROM.put(ADDR_P, arg);
  balancePID.SetTunings(Kp_balancing, Ki_balancing, Kd_balancing);
  Serial.print("P set to: ");
  Serial.println(arg);
}

void functionI(double arg) {
  Ki_balancing = arg;
  EEPROM.put(ADDR_I, arg);
  balancePID.SetTunings(Kp_balancing, Ki_balancing, Kd_balancing);
  Serial.print("I set to: ");
  Serial.println(arg);
}

void functionD(double arg) {
  Kd_balancing = arg;
  EEPROM.put(ADDR_D, arg);
  balancePID.SetTunings(Kp_balancing, Ki_balancing, Kd_balancing);
  Serial.print("D set to: ");
  Serial.println(arg);
}

// void functionAngle(double arg) {
//   kalPitch = arg;
//   EEPROM.put(ADDR_ANGLE, arg);
//   Serial.print("ANGLE set to: ");
//   Serial.println(arg);
// }

void functionAngleSpan(double arg) {
  Angle_balance_span = arg;
  EEPROM.put(ADDR_ANGLE_BALANCE_SPAN, arg);
  Serial.print("ANGLE SPAN set to: ");
  Serial.println(arg);
}

void functionAngleCurrent() {
  Setpoint_angle = kalPitch;
  EEPROM.put(ADDR_ANGLE, Setpoint_angle);
  Serial.print("ANGLE set to: ");
  Serial.println(Setpoint_angle);
}

void functionSerialEnable() {
  disable_serial = !disable_serial;
  Serial.print("disable_serial set to: ");
  Serial.println(disable_serial);
}

void functionBalanceEnable() {
  enable_balancing = !enable_balancing;
  Serial.print("enable_balancing set to: ");
  Serial.println(enable_balancing);
}

void functionPrint() {
  Serial.print("PID value: ");
  Serial.print(balancePID.GetKp());
  Serial.print(" ");
  Serial.print(balancePID.GetKi());
  Serial.print(" ");
  Serial.println(balancePID.GetKd());
  Serial.print("ANGLE SETPOINT: ");
  Serial.println(Setpoint_angle);
  Serial.print("ANGLE SPAN: ");
  Serial.println(Angle_balance_span);
  Serial.print("Limits value: -");
  Serial.print(Velocity_limit_min);
  Serial.print(" ");
  Serial.println(Velocity_limit_max);
}

void functionVLim(double arg1, double arg2) {
  arg1 = abs(arg1);
  arg2 = abs(arg2);
  EEPROM.put(ADDR_VELOCITY_LIMIT_MIN, arg1);
  EEPROM.put(ADDR_VELOCITY_LIMIT_MAX, arg2);
  Velocity_limit_min = arg1;
  Velocity_limit_max = arg2;
  balancePID.SetOutputLimits(-Velocity_limit_min, Velocity_limit_max);

  Serial.print("Wartość parametru V_LIMIT_MIN ustawiona na: ");
  Serial.println(Velocity_limit_min);
  Serial.print("Wartość parametru V_LIMIT_MAX ustawiona na: ");
  Serial.println(Velocity_limit_max);
}