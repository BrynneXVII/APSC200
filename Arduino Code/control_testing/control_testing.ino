/// PREVIOUS CODE
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

//////////////////////////////// Current Code ////////////////////////////////////////////////////////////






// Vehicle paramters
#define r 0.065     // wheel radius
#define R 0.15    // width between two wheels

// Control parameters
#define ts 0.01   //time step


// Vehicle parameters
int omega_max = 12;
int omega_zero = 100;
int omega_r,omega_l;


// Control parameters




void setup()
{
  Serial.begin(9600);
  
  botSetup();  

  
  Serial.println();
  Serial.println();
  Serial.println(F("___________________________________________________________________________________________________________________________"));
  Serial.println(F("Robot setup complete, beginning main loop"));
  Serial.println();
  Serial.println();

}

void loop()
{
  float x_d = 1;
  float y_d = 0;
  getMotorInputs(1,0,1,0,5);
  sendInputs(omega_l,omega_r);
  delay(5000);
  sendInputs(0,0);

  PositionCalc();


while(1);
}

void getMotorInputs(float x_2, float x_1, float y_2, float y_1, float timeStep)
{
  float v_x = (x_2 - x_1)/timeStep;   // Desired velocity in x
  float v_y = (y_2 - y_1)/timeStep;    // Desired velocity in y
  float v = sqrt(pow(v_x,2) + pow(v_y,2));
  
  float omega = atan2(v_y,v_x);

  omega_r = constrainMotorInput((v+R*omega)/r);
  omega_l = constrainMotorInput((v-R*omega)/r);
}


int constrainMotorInput(float omega_cntrl)
{
  if(omega_cntrl>omega_max) omega_cntrl = omega_max;
  return ceil(omega_cntrl/omega_max*(255-omega_zero) + sign(omega_cntrl)*omega_zero);   // 
}

int sign(float x)
{
  if(x>0) return 1;
  else if(x<0) return -1;
  else return 0;
}




/////////////////////// PREIVOUS FUNCTIONS /////////////////////
void botSetup(){
  
  Serial.println(F("botSetup started"));
  
  XBee.begin(9600);      //Set the baud rate for the software serial port
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
    while(1);
  }
  
  displaySensorDetails(); //Shows the details about the sensor module
  
  configureSensor(); //Configures the sensitivity of the sensor module
  setupArdumoto(); //Sets up the ardumoto shield for the robot's motors
  pinMode(Encoder_Pin_Left, INPUT_PULLUP); // Set the mode for the encoder pins
  pinMode(Encoder_Pin_Right, INPUT_PULLUP);
  digitalWrite(Encoder_Pin_Right, HIGH); //Initialize the state of the encoder pins
  digitalWrite(Encoder_Pin_Left, HIGH);
  attachInterrupt(digitalPinToInterrupt(Encoder_Pin_Left), Left_Encoder_Ticks, RISING); //assign the interrupt service routines to the pins
  attachInterrupt(digitalPinToInterrupt(Encoder_Pin_Right), Right_Encoder_Ticks, RISING); //This is done on the Uno's interrupts pins so this syntax is valid, else use the PCI syntax 
  
  Serial.println(F("botSetup completed"));
  
}


//sendInputs sends the left and right inputs to their appropriate wheels
void sendInputs(int leftWheelInput, int rightWheelInput){
  driveArdumoto(MOTOR_L, leftWheelInput);
  driveArdumoto(MOTOR_R, rightWheelInput);
  Serial.println(F("Motors running"));
}



