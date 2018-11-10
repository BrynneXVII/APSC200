//configureSensor sets the tolerances/sensitivties of the sensors, other options can be uncommented to switch them
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

//displaySensorDetails gets the sensor details from the ui
void displaySensorDetails(void)
{  
  lsm.getSensor(&accelSetup, &magSetup, &gyroSetup, &tempSetup);
  #if DEBUG
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accelSetup.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accelSetup.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accelSetup.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accelSetup.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accelSetup.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accelSetup.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(magSetup.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(magSetup.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(magSetup.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(magSetup.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(magSetup.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(magSetup.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyroSetup.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyroSetup.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyroSetup.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyroSetup.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyroSetup.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyroSetup.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(tempSetup.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(tempSetup.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(tempSetup.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(tempSetup.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(tempSetup.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(tempSetup.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  #endif
}

//getHeading gets a snapshot of all of the sensors on the module and then calculates the angle from that
void getHeading(){
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
 
  heading = (float) (atan2(mag.magnetic.y, mag.magnetic.x));//Uses the strength of the magnetic field in the x and y directions to determine the heading 
  //Serial.print("Heading: "); Serial.print(heading, 5); Serial.println(" * from N?");
  while(heading > PI || heading < -PI){
    if(heading > PI){
      heading = heading - 2*PI;
    }
    else{
      heading = heading + 2*PI;
    }
  }
  theta = heading - baseline;
  while(theta > PI || theta < -PI){
    if(theta > PI){
      theta = theta - 2*PI;
    }
    else{
      theta = theta + 2*PI;
    }
  }
  /*
  //Calculates the angle from the gyroscope
  currentTime = millis();
  float timeDiff = (float) (currentTime-oldTime)/1000;
  gyroZ = (float) gyro.gyro.z + 12.2; //offset for the resting gyro error
  theta = (float) theta + gyroZ*timeDiff; 
  while(true){  
    if (theta > 180){ 
      theta = theta - 2*180;
    }else if(theta < -180){
      theta = theta + 2*180;
    }else{
      break;
    }
  }
  oldTime = currentTime;
  */
  #if DEBUG
  Serial.print("Temp: "); Serial.println(temp.temperature);
  #endif
  ambientTemp = temp.temperature;
}
