// 2018 APSC 200 Robot Code

/////////////////////////////// Set up the libraries ///////////////////////////////////////////////
// IMPORTANT: "#define NO_PORTD_PINCHANGES" must be before "#include <SoftSerialFix.h>"
#define NO_PORTD_PINCHANGES // to indicate that port d will not be used for pin change interrupts
#include <PinChangeInt.h> //Needed with the above line to have more interrupts that dont interfere with the Xbee
#include <SoftSerialFix.h> //Communication: Needed to create a software serial port for the Xbee
#include "Wire.h"
#include "math.h"
#include <I2Cdev.h> //Sensing/Communication: Needed to have communication to the sensor module
#include <Adafruit_Sensor.h> //Sensing: Needed to get the sensor data from the accel, gyro, compass unit
#include <Adafruit_LSM9DS0.h> //Sensing: Needed to process the specific sensor's (LSM9DS0) raw data into units



/////////////////////////////// Program Execution Options ///////////////////////////////////////////////

// Localization
#define DEBUG 1

/////////////////////////////// Program Parameters ///////////////////////////////////////////////


/////////////////////////////// Define all needed pins ///////////////////////////////////////////////
#define MOTOR_R 0 // right motor (A)
#define MOTOR_L 1 // left motor (B)
#define both 2 // a tag for both motors
#define Encoder_Pin_Right 2   //Encoder pins
#define Encoder_Pin_Left 3   //Encoder pins
#define DIRB 7 // Direction control for motor B
#define DIRA 8 // Direction control for motor A
#define PWMA 9  // PWM control (speed) for motor A
#define PWMB 10 // PWM control (speed) for motor B
#define irPin 11    // Infrared reception pin (for localization)
#define usPin A0    // Ultrasonic reception pin (for localization)
#define batPin A1   // battery level indicator pin. Would be hooked up to votlage divider from 9v barrel jack, but currently not implemented


/////////////////////////////// Sensor Variables ///////////////////////////////////////////////
sensor_t accelSetup, magSetup, gyroSetup, tempSetup; //Variables used to setup the sensor module
sensors_event_t accel, mag, gyro, temp; // Variables to store the data of the sensors every time it is retrieved
float heading, baseline = 0; // Variables to store the calculated heading and the baseline variable (Baseline may be unnecessary)

/////////////////////////////// Encoder Variables ///////////////////////////////////////////////
int lCount_abs, rCount_abs; // Variables used in the encoder ISRs, the absolute value  ones may be unneeded
int oldLeftEncoder = 0, oldRightEncoder = 0; // Stores the encoder value from the loop prior to estimate x, y position
int leftEncoder = 0, rightEncoder = 0; // Stores the encoder values for the current loop

/////////////////////////////// Position Variables ///////////////////////////////////////////////
float leftRads = 0, rightRads = 0; // Stores the left and right radians of the wheels (from encoder values)
float xPosition = 0, yPosition = 0; // Stores the robot's current x and y position estimate from the encoders
float theta = 0; // Stores the current angle of the robot, from the gyro
float gyroZ; //stores the Z component of the gyroscope so it can be manipulated into an angle
unsigned long oldTime = 0, currentTime = 0; // Variables to timestamp the loops to calculate position

/////////////////////////////// Other Variables /////////////////////////////////////////////////
char bot; // A variable to store what the Xbee has received
char response_to_rpi_remote = 'K'; //From the 2017 code, a confirmation that the xbee received a signal
int leftTemp[3], rightTemp[3];
int leftInput, rightInput; //A variable to convert the wheel speeds from char (accepted), to int
char leftArray[5], rightArray[5]; //Am array to store all wheel info from the Xbee to be interpreted later

int startLoop, endLoop; //Loop timing variables to know how long the loop takes
float loopTime;

/////////////////////////////// Agent Tag Data - CHANGE FOR EACH ROBOT ///////////////////////////////////////////////
char agentTag = 'S';
char agentTagLower = 's';

////////////////////////////////////////////////////////// Object Declarations //////////////////////////////////////////////////////////
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(); //An object for the sensor module, to be accessed to get the data
SoftSerialFix XBee(4,5); //The software created serial port to communicate through the Xbee

//////////////////////////////// Setup ////////////////////////////////////////////////////////////


void setup(){
  #if DEBUG 
  Serial.begin(9600);
  #endif
  
  botSetup();
  botCheck();
  
  #if DEBUG
  Serial.println();
  Serial.println();
  Serial.println(F("___________________________________________________________________________________________________________________________"));
  Serial.println(F("Robot setup complete, beginning main loop"));
  Serial.println();
  Serial.println();
  #endif
}

//////////////////////////////// Main Loop /////////////////////////////////////////////////////////
void loop() {
  botLoop();
}

//////////////////////////////// Functions /////////////////////////////////////////////////////////

void botSetup(){
  #if DEBUG
  Serial.println(F("botSetup started"));
  #endif
  #ifndef ESP8266         // from sensor module example code, don't know if we need this 
    while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
  XBee.begin(9600);      //Set the baud rate for the software serial port
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
    while(1);
  }
  #if DEBUG
  displaySensorDetails(); //Shows the details about the sensor module
  #endif
  configureSensor(); //Configures the sensitivity of the sensor module
  setupArdumoto(); //Sets up the ardumoto shield for the robot's motors
  pinMode(Encoder_Pin_Left, INPUT_PULLUP); // Set the mode for the encoder pins
  pinMode(Encoder_Pin_Right, INPUT_PULLUP);
  digitalWrite(Encoder_Pin_Right, HIGH); //Initialize the state of the encoder pins
  digitalWrite(Encoder_Pin_Left, HIGH);
  attachInterrupt(digitalPinToInterrupt(Encoder_Pin_Left), Left_Encoder_Ticks, RISING); //assign the interrupt service routines to the pins
  attachInterrupt(digitalPinToInterrupt(Encoder_Pin_Right), Right_Encoder_Ticks, RISING); //This is done on the Uno's interrupts pins so this syntax is valid, else use the PCI syntax 
  #if DEBUG
  Serial.println(F("botSetup completed"));
  #endif
}

//botCheck enters this check after it has completed all of its setup, it waits here until MATLAB checks that its ready
void botCheck(){
  #if DEBUG
  Serial.println(F("botCheck started"));
  #endif
   while(true){
    delay(20);
    bot = (XBee.available()) ? char(XBee.read()) : '0';
    if (bot == agentTag){
       XBee.write(agentTag);
       char initialPositionArray[11];
       int byteCounter = 0;
       int intArray[9];

       while(byteCounter < 11){
          if (XBee.available()){
            initialPositionArray[byteCounter] = char(XBee.read()); //Get all of the left wheel data (check char, direction, speed)
            delay(20);
            byteCounter ++;
          }
        }

        //get all of the initial position data from MATLAB, the unique x and y as well as the baseline to adjust the frame of reference, 
        //convert them from the characters to the appropriate integers by subtracting 48 ('1' = 49 so (int) '1'-48 = 1)
        intArray[0] = (int) initialPositionArray[0] - 48;
        intArray[1] = (int) initialPositionArray[2] - 48;
        intArray[2] = (int) initialPositionArray[3] - 48;
        intArray[3] = (int) initialPositionArray[4] - 48;
        intArray[4] = (int) initialPositionArray[6] - 48;
        intArray[5] = (int) initialPositionArray[7] - 48;
        intArray[6] = (int) initialPositionArray[8] - 48;
        intArray[7] = (int) initialPositionArray[9] - 48;
        intArray[8] = (int) initialPositionArray[10] - 48;
        
        XBee.write('K');

        //Take each digit and multiply it by the appropriate base ten to turn it into positions 
        xPosition = (float) intArray[0] + intArray[1]*0.1 + intArray[2]*0.01;
        yPosition = (float) intArray[3] + intArray[4]*0.1 + intArray[5]*0.01;
        baseline = intArray[6]*100 + intArray[7]*10 + intArray[8];

        //Convert the baseline input from 0 to 359 to radians and from -pi to pi 
        baseline = (float) baseline*PI/180; 
        Serial.println(baseline);
        while(baseline > PI){
          baseline = baseline - 2*PI;
        }
        Serial.println(baseline);
        //gets the heading of the robot and also theta which is then adjusted by the baseline
        getHeading();
        break;
    }
  }
  #if DEBUG
  Serial.print(F("Inital X ")); Serial.println(xPosition); 
  Serial.print(F("Initial Y ")); Serial.println(yPosition); 
  Serial.println(F("botCheck completed"));
  #endif
}

//The main loop of the robot, could be moved fully to the loop function if desired, on every iteration, x, y, theta are all
//updated by the encoders and the gyro then the Xbee is checked to see if it has any intruction from MATLAB, then the appropriate
//action is performed
void botLoop()
{
  //update the angle of the robot
  getHeading();
  PositionCalc(); //update the position of the robot
  #if DEBUG
  Serial.print(F("Angle in Degrees ")); Serial.println(theta); 
  Serial.print(F("Heading in Degrees ")); Serial.println(heading); 
  #endif   
  delay(20);

  //Get the Xbee data (if there is data)
  bot = (XBee.available()) ? char(XBee.read()) : '0'; 
  if (bot == agentTag){ //If it is the agent's tag, respond then get the wheel inputs
    XBee.write(agentTag);
    getWheelInputs();
  }
  else if (bot == agentTagLower){ //If it is the agent's lower tag, respond then send x, y, theta to MATLAB
    XBee.write(agentTagLower);
    sendSensors();
  }
  else if (bot == '1'){ //If a 1 is received, take the stored wheel inputs and run them for the specified duration 
    sendInputs(leftInput, rightInput); 
    //delay(MOVEMENT_DURATION); //can adjust to determine how long the wheels will run for
    stopMotors(both);
  }
  else if (bot == 'C'){ //If a C is sent, wheel calibration is going on so MOVEMENT_DURATION cannot be used
    while(true){
        bot = (XBee.available()) ? char(XBee.read()) : '0';
        if (bot == agentTag){
          XBee.write(agentTag);
          getWheelInputs();
          sendInputs(leftInput, rightInput); 
        }
        else if (bot == agentTagLower){
          stopMotors(both);
          break;
        }
     }
  }
  else if (bot == 'R'){ // If an R is sent to the Xbee, reset the Arduino (restart the program)
    asm volatile ("  jmp 0");  
  }
}


//getWheelInputs waits for the Xbee then receives 10bytes. First byte is a B and the sixth byte is an A (checks), could be removed????
void getWheelInputs() {
  int byteCounter = 0;
  while(byteCounter < 5){
    Serial.println("Here 3");
    if (XBee.available()){
      leftArray[byteCounter] = char(XBee.read()); //Get all of the left wheel data (check char, direction, speed)
      byteCounter++;
      if (leftArray[0] != 'B'){
        byteCounter = 0;
        delay(10);
      }
    }
  }
  
  leftTemp[0] = (int) leftArray[2] - 48;
  leftTemp[1] = (int) leftArray[3] - 48;
  leftTemp[2] = (int) leftArray[4] - 48;
    
  XBee.write('K');
  
  leftInput = leftTemp[0]*100 + leftTemp[1]*10 + leftTemp[2];  
  #if DEBUG
  Serial.println(leftArray[0]);
  Serial.println(leftArray[1]);
  Serial.println(leftInput);
  #endif

  //unnescessary????
  while (XBee.available() && XBee.read()); 
  
  byteCounter = 0;
  while(byteCounter < 5){
    Serial.println("Here 3.5");
    if (XBee.available()){
      rightArray[byteCounter] = char(XBee.read()); //Get all of the right wheel data (check char, direction, speed)
      byteCounter++;
      if (rightArray[0] != 'A'){
        byteCounter = 0;
        delay(10);
      }
    }
  }
  
  rightTemp[0] = (int) rightArray[2] - 48;
  rightTemp[1] = (int) rightArray[3] - 48;
  rightTemp[2] = (int) rightArray[4] - 48;
    
  XBee.write('K');
  
  rightInput = rightTemp[0]*100 + rightTemp[1]*10 + rightTemp[2];

  #if DEBUG
  Serial.println(rightArray[0]);
  Serial.println(rightArray[1]);
  Serial.println(rightInput);
  #endif

  //unnescessary????
  while (XBee.available() && XBee.read());
  
  if (leftArray[1] == 'N'){
    leftInput = leftInput*(-1);
  }
  if (rightArray[1] == 'N'){
    rightInput = rightInput*(-1);
  }
}

//sendInputs sends the left and right inputs to their appropriate wheels
void sendInputs(int leftWheelInput, int rightWheelInput){
  driveArdumoto(MOTOR_L, leftWheelInput);
  driveArdumoto(MOTOR_R, rightWheelInput);
  #if DEBUG
  Serial.println(F("Motors running"));
  #endif
}

//sendSensors sends the lower case agent tag to the Xbee to let the receiver know where to store the variables, then it sends x, y, and theta to MATLAB
void sendSensors(){  
  //Convert the information needing to be sent into byte arrays (sends much easier)
  byte * thetaBytes = (byte *) &heading; //&theta;
  byte * xBytes = (byte *) &xPosition;
  byte * yBytes = (byte *) &yPosition;
  XBee.write(xBytes,4);
  XBee.write(yBytes,4);
  XBee.write(thetaBytes,4);

  #if DEBUG
  Serial.print(F("left: "));Serial.println(leftEncoder);
  Serial.print(F("right: "));Serial.println(rightEncoder);
  Serial.print(F("x: "));Serial.println(xPosition);
  Serial.print(F("y: "));Serial.println(yPosition);
 // Serial.print("gyro: ");Serial.println(gyroZ);
  Serial.print(F("theta: "));Serial.println(theta);
  #endif
}

//sendAllData sends the x, y, and theta positions as well as the localization variables
void sendAllData(){
  //Convert the information needing to be sent into byte arrays (sends much easier)
  byte * thetaBytes = (byte *) &heading; //&theta;
  byte * xBytes = (byte *) &xPosition;
  byte * yBytes = (byte *) &yPosition;
  
  XBee.write(xBytes,4);
  XBee.write(yBytes,4);
  XBee.write(thetaBytes,4);
}



