#define MAX_ARGUMENTS 3

void commandEngine() {
  String unknown = "Nieznana komenda!";
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Odczytaj dane z portu szeregowego do momentu napotkania znaku nowej linii

    // Serial.print("Otrzymano komendę: ");
    // Serial.println(command);

    if (command.startsWith("PRINT")) 
    {
      if (command.length() == 5 || command.length() == 6) {
        functionPrint();
      } else {
        Serial.println(unknown);
      }
    } 
    else if (command.startsWith("BAL")) 
    {
      if (command.length() == 3 || command.length() == 4) {
        functionBalanceEnable();
      } 
      else {
        Serial.println(unknown);
      }
    } 
    else if (command.startsWith("SERIAL")) 
    {
      if (command.length() == 6 || command.length() == 7) {
        functionSerialEnable();
      } 
      else {
        Serial.println(unknown);
      }
    } 
    else if (command.startsWith("ANG_SET")) 
    {
      if (command.length() == 7 || command.length() == 8) {
        functionAngleCurrent();
      } 
      else {
        Serial.println(unknown);
      }
    } 
    else if (command.startsWith("P ")) 
    {
      float arg = command.substring(2).toFloat();
      functionP(arg);
    } 
    else if (command.startsWith("I ")) 
    {
      float arg = command.substring(2).toFloat();
      functionI(arg);
    }
    else if (command.startsWith("D ")) 
    {
      float arg = command.substring(2).toFloat();
      functionD(arg);
    }
    else if (command.startsWith("ANGLE ")) 
    {
      float arg = command.substring(6).toFloat();
      functionAngle(arg);
    }
    else if (command.startsWith("SPAN ")) 
    {
      String arg1 = getValue(command, ' ', 1);
      if (arg1 != "") {
        int val1 = arg1.toInt();
        functionAngleSpan(val1);
      } 
      else {
        Serial.println("Nieprawidłowy argument!");
      }
    }

    // else if (command.startsWith("GO ")) 
    // {
    //   String arg1 = getValue(command, ' ', 1);
    //   if (arg1 != "") {
    //     int val1 = arg1.toInt();
    //     functionManualGo(val1);
    //   } 
    //   else {
    //     Serial.println("Nieprawidłowy argument!");
    //   }
    // }
    // else if (command.startsWith("STOP")) 
    // {
    //   functionSTOP();
    // }
    else if (command.startsWith("V_LIM ")) {
      String arg1 = getValue(command, ' ', 1);
      String arg2 = getValue(command, ' ', 2);
      if (arg1 != "" && arg2 != "") {
        int val1 = arg1.toInt();
        int val2 = arg2.toInt();
        functionVLim(val1, val2);
      } 
      else {
        Serial.println("Nieprawidłowe argumenty!");
      }
    }
    else 
    {
      Serial.println(unknown);
    }
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
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
  Serial.print("I set to: ");
  balancePID.SetTunings(Kp_balancing, Ki_balancing, Kd_balancing);
  Serial.println(arg);
}

void functionD(double arg) {
  Kd_balancing = arg;
  EEPROM.put(ADDR_D, arg);
  Serial.print("D set to: ");
  balancePID.SetTunings(Kp_balancing, Ki_balancing, Kd_balancing);
  Serial.println(arg);
}

void functionAngle(float arg) {
  Setpoint_angle = arg;
  // balancePID.SetSetpoint(arg);
  EEPROM.put(ADDR_ANGLE, arg);
  // Dodaj kod obsługujący zapis parametru ANGLE
  Serial.print("ANGLE set to: ");
  Serial.println(arg);
}

// void functionManualGo(float arg) {
//   enable_balancing = false;
//   manual_go = true;
//   motor_right_setpoint_speed = arg;
//   motor_left_setpoint_speed = arg;

//   Serial.print("Manual go: ");
//   Serial.println(arg);
// }

// void functionSTOP() {
//   enable_balancing = false;
//   manual_go = true;
//   motor_right_setpoint_speed = 0;
//   motor_left_setpoint_speed = 0;
//   Serial.println("STOP");
// }

void functionAngleSpan(float arg) {
  Angle_balance_span = arg;
  EEPROM.put(ADDR_ANGLE_BALANCE_SPAN, arg);
  Serial.print("ANGLE SPAN set to: ");
  Serial.println(arg);
}

void functionAngleCurrent() {
  EEPROM.put(ADDR_ANGLE, kalPitch);
  Setpoint_angle = kalPitch;
  // balancePID.SetSetpoint(kalPitch);
  // Dodaj kod obsługujący zapis parametru ANGLE
  Serial.print("ANGLE set to: ");
  Serial.println(kalPitch);
  // Serial.println("Do hardware reset!");
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
  // Wyświetl wszystkie parametry
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
  

  // Dodaj inne parametry, jeśli są potrzebne
}

void functionVLim(int16_t arg1, int16_t arg2) {
  arg1 = abs(arg1);
  arg2 = abs(arg2);
  EEPROM.put(ADDR_VELOCITY_LIMIT_MIN, arg1);
  EEPROM.put(ADDR_VELOCITY_LIMIT_MAX, arg2);
  Velocity_limit_min = arg1;
  Velocity_limit_max = arg2;
  

  // Dodaj kod obsługujący zapis parametrów V_LIM
  balancePID.SetOutputLimits(-arg1, arg2);
  Serial.print("Wartość parametru V_LIMIT_MIN ustawiona na: ");
  Serial.println(arg1);
  Serial.print("Wartość parametru V_LIMIT_MAX ustawiona na: ");
  Serial.println(arg2);
}

