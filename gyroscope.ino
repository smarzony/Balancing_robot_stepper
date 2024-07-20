void read_gyro_kalman() {
  // Vector acc = mpu.readNormalizeAccel();
  // Vector gyr = mpu.readNormalizeGyro();

  // // Kalukacja Pitch &amp; Roll z akcelerometru
  // accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis * acc.YAxis + acc.ZAxis * acc.ZAxis)) * 180.0) / M_PI;
  // accRoll = (atan2(acc.YAxis, acc.ZAxis) * 180.0) / M_PI;

  // // Kalman - dane z akcelerometru i zyroskopu
  // kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  // kalRoll = kalmanX.update(accRoll, gyr.XAxis);
}
