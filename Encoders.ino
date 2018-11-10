//Left_Encoder_Ticks is a interrupt service routine that updates the encoder
void Left_Encoder_Ticks(){ // runs in the background updating left encoder value. Never needs to be called
  leftEncoder = (leftInput >= 0) ? leftEncoder+1 : leftEncoder-1;
  lCount_abs++;
}

//Right_Encoder_Ticks is a interrupt service routine that updates the encoder
void Right_Encoder_Ticks(){ // runs in the background updating right encoder value. Never needs to be called
  rightEncoder = (rightInput >= 0) ? rightEncoder+1 : rightEncoder-1;
  rCount_abs++;
}

//PositionCalc is a function that uses the system properties and dynamic equations to estimate the position of the robot (dead reckoning)
//this estimation uses the encoders and the angle of the robot 
void PositionCalc(){
  leftRads = (leftEncoder - oldLeftEncoder)*2*PI/192;
  rightRads = (rightEncoder - oldRightEncoder)*2*PI/192;
  xPosition = xPosition + cos(theta)*(0.065/2)*0.5*(leftRads + rightRads);
  yPosition = yPosition + sin(theta)*(0.065/2)*0.5*(leftRads + rightRads);
  oldLeftEncoder = leftEncoder;
  oldRightEncoder = rightEncoder;
}



