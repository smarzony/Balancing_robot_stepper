void keyboard_read() {
  adc_key_in = analogRead(7);
  key = get_key(adc_key_in);  //Call the button judging function.

  if (key != oldkey) {  // Get the button pressed
    delay(50);
    adc_key_in = analogRead(7);
    key = get_key(adc_key_in);
    if (key != oldkey) {
      oldkey = key;
      if (key >= 0) {
        digitalWrite(13, HIGH);
        switch (key) {  // Send messages accordingly.
          case 0:
            disable_serial = !disable_serial;
            break;
          case 1:
            enable_balancing = !enable_balancing;
            break;
          case 2:
            functionAngleCurrent();
            break;
          case 3:
            break;
          case 4:
            break;
        }
      }
    }
  }
}

int get_key(unsigned int input) {
  int k;
  for (k = 0; k < NUM_KEYS; k++) {
    if (input < adc_key_val[k]) {  // Get the button pressed
      return k;
    }
  }
  if (k >= NUM_KEYS) k = -1;  // No button is pressed.
  return k;
}