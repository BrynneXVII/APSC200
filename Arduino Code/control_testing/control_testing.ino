#include <MatrixMath.h>

// Vehicle paramters
#define r 0.065     // wheel radius
#define R 0.15    // width between two wheels

// Control parameters
#define ts 0.01   //time step


// Vehicle parameters
int omega_max;
int omega_zero;


// Control parameters






void setup()
{
  Serial.begin(9600);


}

void loop()
{
  

  //double q[n] = {0,0,0}   // Current state
  
  //double A[][M] = {
  //  {1,2},
  //  {3,4}
  //};

  //Matrix.Print((mtx_type*)A, N, N, "A");

  

while(1);
}

void getMotorInputs(float x_d, float x, float y_d, float y)
{
  float v_x = (x_d - x)/ts;   // Desired velocity in x
  float v_y = (y_d - y)/ts;    // Desired velocity in y
  float v = sqrt(v_x^2 + v_y^2);

  float omega = atan((y_d - y)/(x_d - x));

  int omega_r = constrainMotorInput((v+R*omega)/r);
  int omega_l = constrainMotorInput(((v-R*omega)/r);
}

int constrainMotorInput(float omega_cntrl)
{
  return ceil(omega_cntrl*omega_max/(255-omega_zero) + sign(omega_cntrl)*omega_zero);
}












