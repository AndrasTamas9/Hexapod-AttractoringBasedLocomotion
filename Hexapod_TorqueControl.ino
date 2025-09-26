#include <stdio.h>  
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <BioloidController.h>  

#define NR 6
#define PI 3.14159265359
#define SERVONR 18

//                 ID>           1       2         3         4         5         6        7        8        9         10        11        12        13       14        15        16        17        18
#define ID_to_LEG (int[]){18, COXA[3], COXA[0], FEMUR[3], FEMUR[0], TIBIA[3], TIBIA[0], COXA[5], COXA[2], FEMUR[5], FEMUR[2], TIBIA[5], TIBIA[2], COXA[4], COXA[1], FEMUR[4], FEMUR[1], TIBIA[4], TIBIA[1] }
//           NR>coxa+femur    1   2  3  4   5  6  1   2   3  4  5   6
#define LEG_to_ID (int[]){18, 2, 14, 8, 1, 13, 7, 4, 16, 10, 3, 15, 9, 6, 18, 12, 5, 17, 11}



BioloidController bioloid = BioloidController(1000000);

// PROTOTIPUSOK:
void setupHexapod();
void print_next_pose(int length, int *vector1, int *vector2);
void print_current_pose();
void print_current_speed();
void print_current_torq();
void print_y();
void print_s();
float y(float x, float b);
void y_calc();
float s(int i);
void M_calc();
void COXA_FEMUR_values();
void set_delay_time(int time);
void set_max_torq(int tc, int tf, int tt);
void set_speed(int sc, int sf);
void set_speed_loop();
void set_joint_mode();
void set_wheel_mode();
void MINMAX_values(int dir);
void error_correction(int i);
void pose_check(int i);
void make_next_pose();
int rad_to_servo(float rads);
void voltage_test();
void random_start_param();
void python_connection();
void serial_break();
void setMovingSpeed();
void setZeroSpeed();

// ################################################### PARAMETERS ################################################### PARAMETERS ################################################### 

//moving thresholds both dirrections in radian
float threshold_coxa = PI/12;
float threshold_femur = PI/10;
int error_treshold = 100;
int error_num[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int speed_coxa = 0; 
int speed_femur = 0;
int torq_coxa = 1023;
int torq_femur = 1023;             
int torq_tibia = 1023;

// ###############################################

// ### Min and max values for servos, from Pheonix ###
//                  id>   2    14   8     1    13   7
const int COXA_min[6] = {225, 230, 225, 225, 230, 225};
const int COXA_max[6] = {790, 790, 790, 790, 790, 790};
//                  id>   4    16   10    3    15   9
const int FEMUR_min[6] = {160, 160, 160, 160, 160, 160};
const int FEMUR_max[6] = {860, 860, 860, 860, 860, 860};

//
const int TIBIA[6] = {250, 250, 250, 774, 774, 774};
int COXA[6] = {450, 512, 573, 573, 512, 450};
int FEMUR[6] = {670, 670, 670, 350, 350, 350};

// ###############################################

const float tau_x[2] = {0.05, 0.025}; 
const float tau_b = 0.5 ;
const float a = 2.0;

float dt = 0.036;
float w_k = -0.4;
float w_y = 2.00;
float w_s = 0.1;
float gamma = 0.01; // 0.01
float k = 1.0;  //1.0

double previousTime = 0;
float delta_w = 0.1;

double U[24] = {2.0, -2.0, 2.0, -2.0, 2.0, -2.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, 4.0,  0.0, 4.0,  0.0, 4.0,  0.0, 1.0, 4.0,  1.0, 4.0,  1.0, 4.0};


float M_raw[18];
int M[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// ################################################### GLOBALVARIABLES ################################################### GLOBALVARIABLES ################################################### 
int COXA_MIN[6];
int COXA_MAX[6];
int FEMUR_MIN[6];
int FEMUR_MAX[6];

unsigned long t, begin_time, setup_time;
int print_freq = 0;
int current_pose[18];
int temp_current_pose[18] = {450, 512, 573, 573, 512, 450, 670, 670, 670, 350, 350, 350, 250, 250, 250, 774, 774, 774};
int previous_pose[18] = {450, 512, 573, 573, 512, 450, 670, 670, 670, 350, 350, 350, 250, 250, 250, 774, 774, 774};
int previous_target_pose[18] = {450, 512, 573, 573, 512, 450, 670, 670, 670, 350, 350, 350, 250, 250, 250, 774, 774, 774};
int current_speed[12];
int current_torq[12];
float y_coxfem[12];
float s_coxfem[12];

float random_par = 0;
int python_index = 1;
String szam = "";
String python_data = "";
float connect_flag = 0;
int counter = 0;

double K1[24];
double K2[24];
double K3[24];
double K4[24];



void setup() {
  Serial.begin(115200); 
  
  begin_time = millis();
  
  MINMAX_values(1); //1 forward, -1 backward
  
  set_joint_mode();
  set_speed(speed_coxa, speed_femur); 
  set_max_torq(torq_coxa, torq_femur, torq_tibia); 
  bioloid.setup(SERVONR);
  python_connection();

  //startpose with macro from COXA,FEMUR,TIBIA
  for ( int i = 1; i < SERVONR + 1 ; i++){
    bioloid.setNextPose(i, (int)ID_to_LEG[i]);
  }
  bioloid.readPose();
  bioloid.interpolateSetup(1000);
  while(bioloid.interpolating > 0){
    bioloid.interpolateStep();
    delay(3);
  }
  delay(100);
  
  setZeroSpeed();
  delay(100);

  y_calc();
  delay(10);
  s_calc();
  delay(1000);
  
  //WHEEL MODE:
  set_wheel_mode();
  set_speed(0, 0);
  setZeroSpeed();
  delay(10);
  
  
  setup_time = millis();  

}

void loop() {
  t = micros();
  
  serial_break();
  
  RK4(dt);
  y_calc();
  COXA_FEMUR_values();
  
 
  for(int i = 0; i < 18; i++){
    current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
    error_correction(i);
    pose_check(i);
    temp_current_pose[i] = current_pose[i];
  }
  
  serial_break();
 
  s_calc();
  M_calc();
  
  setMovingSpeed();
  
  serial_break();
  
  Serial.print(counter);
  Serial.print(" ");
  counter = counter + 1;
  Serial.print(dt, 3);
  Serial.print(" ");
  
  //measure_current_torq();
  
  //print_temp_current_pose();
  //print_next_pose(NR, COXA, FEMUR);
  print_current_pose();
  serial_break();
  print_next_pose(NR, COXA, FEMUR);
  //print_s();
  //print_M();
  //print_current_torq();
  //check_current_torq();
  //print_current_torq();
  Serial.println();
  //Serial.println();
  //print_current_pose();
  //print_current_speed();
  serial_break();

  dt = (micros()-t)/1000000.0000;
}




// PRINT FUNCTIONS ###################################################################################

//It outputs the following joint positions through the serial port
void print_next_pose(int length, int *vector1, int *vector2){
  //Serial.print((millis()- begin_time) / 1000.0 );
  //Serial.print(" ");
  for (int i = 0; i < length; i++){
    Serial.print(vector1[i]);
    Serial.print(" ");
  }
  for (int i = 0; i < length; i++){
    Serial.print(vector2[i]);
    Serial.print(" ");
  }
  //Serial.println();
}

//It outputs the current joint positions through the serial port
void print_current_pose(){
  Serial.print((millis()- setup_time) / 1000.0 );
  Serial.print(" ");
  for(int i = 0; i < 18; i++){
    Serial.print(current_pose[i]);
    Serial.print(" ");
  }
  //Serial.println(" ");
  //Serial.flush();  
}

//It outputs the currently temporarily saved joint positions through the serial port
void print_temp_current_pose(){
  Serial.print((millis()- setup_time) / 1000.0 );
  Serial.print(" ");
  for(int i = 0; i < 12; i++){
    Serial.print(temp_current_pose[i]);
    Serial.print(" ");
  }
  //Serial.println(" ");
  //Serial.flush();  
}

//It measures and outputs the servos' speed through the serial port
void print_current_speed(){
  for(int i=0; i < 12; i++){
    current_speed[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_SPEED_L, 2);
    if (current_speed[i] < 0){
      current_speed[i] = 0;
    }
    Serial.print(current_speed[i]);
    Serial.print(" ");
    }
    //Serial.println();
}

//It outputs the current joint torques through the serial port
void print_current_torq(){
    for(int i=0; i < 12; i++){
    Serial.print(current_torq[i]);
    Serial.print(" ");
    }
    //Serial.println();
}

//It outputs the y (activity) values through the serial port
void print_y()
{
  for(int i=0; i < 12; i++){
    Serial.print(y_coxfem[i], 3);
    Serial.print(" ");
  }
  Serial.println();
}

//It outputs the calculated M (torque) values through the serial port
void print_M()
{
  for(int i=0; i < 12; i++){
    Serial.print(M[i]);
    Serial.print(" ");
  }
}

void print_s()
{
  for(int i=0; i < 12; i++){
    Serial.print(s_coxfem[i]);
    Serial.print(" ");
  }
}

/*
void measure_current_torq()
{
  for(int i=0; i < 12; i++){
    current_torq[i] = ax12GetRegister(LEG_to_ID[i+1], AX_GOAL_SPEED_L, 2);
    }
}

void check_current_torq()
{
  for(int i=0; i < 12; i++)
  {
    if(current_torq[i] != M[i])
    {
      current_torq[i] = ax12GetRegister(LEG_to_ID[i+1], AX_GOAL_SPEED_L, 2);
    }
    //current_torq[i] = ax12GetRegister(LEG_to_ID[i+1], AX_GOAL_SPEED_L, 2);
  }
}
*/
//#####################################################################################################

// THEORY FUNCTIONS ###################################################################################

//This is an activation sigmoid function
float y(float x, float b){
    return  1.0 / (1.0 + exp(a * (b - x)));
}

//This calculates the neurons' activity
void y_calc()
{
  for(int i = 0; i < 12; i++)
  {
    y_coxfem[i] = y(U[i], U[i+12]);
  }
}

//This calculates the feedback from the joints
float s(int i){
  
  float temp_s;
  
    if (i < 6)
    {
      temp_s = (float)(current_pose[i] - COXA_MIN[i]) / (float)(COXA_MAX[i]-COXA_MIN[i]);
      
      if(temp_s < 0)
      {
        temp_s = 0;
      }
      if(temp_s > 1)
      {
        temp_s = 1;
      }
      return temp_s;
    }
    else
    {
      temp_s = (float)(current_pose[i] - FEMUR_MIN[i-6]) / (float)(FEMUR_MAX[i-6] - FEMUR_MIN[i-6]);
      
      if(temp_s < 0)
      {
        temp_s = 0;
      }
      if(temp_s > 1)
      {
        temp_s = 1;
      }
      return temp_s;
    }
}

void s_calc()
{
  for(int i = 0; i < 12; i++)
  {
    s_coxfem[i] = s(i);
  }
}

//This calculates the torques based on the theory
void M_calc()
{
  for(int i = 0; i < 18; i++)
  {
    if(i<6)
    {
      M_raw[i] = (-1) * gamma * (current_pose[i] - previous_pose[i]) / dt + k * (COXA[i] - current_pose[i]);
    }
    if((i >= 6) && (i < 12))
    {
      M_raw[i] = (-1) * gamma * (current_pose[i] - previous_pose[i]) / dt + k * (FEMUR[i-6] - current_pose[i]);
    }
    if((i >= 12) && (i < 18))
    {
      M_raw[i] = (-1) * gamma * (current_pose[i] - previous_pose[i]) / dt + k * (TIBIA[i-12] - current_pose[i]);
    }
    
    if((M_raw[i] < 0) && (M_raw[i] >= -100))
    {
      M[i] = (-M_raw[i] / 100.0 * 1023 + 1024);
    }
    
    if((M_raw[i] < 0) && (M_raw[i] < -100))
    {
      M[i] = 2047;
    }
    
    if((M_raw[i] >= 0) && (M_raw[i] <= 100))
    {
      M[i] = (M_raw[i] / 100.0 * 1023);
    }
    
    if((M_raw[i] >= 0) && (M_raw[i] >= 100))
    {
      M[i] = 1023;
    }
    
    previous_pose[i] = current_pose[i];
  }
}
//#####################################################################################################

// RUNGE-KUTTA FUNCTIONS ##############################################################################
void funcK1()
{
  for(int i = 0; i < 12; i++)
  {
    float temp0 = -U[i];
    float temp1 = 0;
    float temp2 = 0;

    if(i < 6)
    {
      for(int j = 0; j < 6; j++)
      {
        if(i != j)
        {
          temp1 += y_coxfem[j]; //y(U[j], U[j+12]);
        }
      }
      temp1 *= w_k;
      temp2 = w_y * y_coxfem[i] + w_s * s_coxfem[i]; //y(U[i], U[i+12]); //+ w_s * s(0, i);

      K1[i] = (temp0 + temp1 + temp2) / tau_x[0];
    }

    if(i >= 6)
    {
      temp1 = w_k * y_coxfem[i-6]; //y(U[i-6], U[i+6]);
      temp2 = w_y * y_coxfem[i] + w_s * s_coxfem[i]; //y(U[i], U[i+12]); //+ w_s * s(1, i) ;

      K1[i] = (temp0 + temp1 + temp2) / tau_x[1];
    }

    K1[i+12] =  (y_coxfem[i] - 0.5)/ tau_b; //(y(U[i], U[i+12]) - 0.5) / tau_b;
  }
}

void funcK2(float dt)
{
  double y_precalc[12];

  for(int i = 0; i < 12; i++)
  {
    y_precalc[i] = y((U[i] + dt*K1[i]/2.0), (U[i+12] + dt*K1[i+12]/2.0));
  }
  
  for(int i = 0; i < 12; i++)
  {
    float temp0 = -(U[i] + dt*K1[i]/2.0);
    float temp1 = 0;
    float temp2 = 0;

    if(i < 6)
    {
      for(int j = 0; j < 6; j++)
      {
        if(i != j)
        {
          temp1 += y_precalc[j];//y((U[j] + dt*K1[j]/2.0), (U[j+12] + dt*K1[j+12]/2.0));
        }
      }
      temp1 *= w_k;
      temp2 = w_y * y_precalc[i] + w_s * s_coxfem[i]; //y((U[i] + dt*K1[i]/2.0), (U[i+12] + dt*K1[i+12]/2.0)); //+ w_s * s(0, i);

      K2[i] = (temp0 + temp1 + temp2) / tau_x[0];
    }

    if(i >= 6)
    {
      temp1 = w_k * y_precalc[i-6]; //y((U[i-6] + dt*K1[i-6]/2.0), (U[i+6] + dt*K1[i+6]/2.0));
      temp2 = w_y * y_precalc[i] + w_s * s_coxfem[i]; //y((U[i] + dt*K1[i]/2.0), (U[i+12] + dt*K1[i+12]/2.0)); //+ w_s * s(1, i) ;

      K2[i] = (temp0 + temp1 + temp2) / tau_x[1];
    }

    K2[i+12] = (y_precalc[i] - 0.5) /tau_b; //(y((U[i] + dt*K1[i]/2.0), (U[i+12] + dt*K1[i+12]/2.0)) - 0.5) / tau_b;
  }
}

void funcK3(float dt)
{
  double y_precalc[12];

  for(int i = 0; i < 12; i++)
  {
    y_precalc[i] = y((U[i] + dt*K2[i]/2.0), (U[i+12] + dt*K2[i+12]/2.0));
  }
  
  
  for(int i = 0; i < 12; i++)
  {
    float temp0 = -(U[i] + dt*K2[i]/2.0);
    float temp1 = 0;
    float temp2 = 0;

    if(i < 6)
    {
      for(int j = 0; j < 6; j++)
      {
        if(i != j)
        {
          temp1 += y_precalc[j]; //y((U[j] + dt*K2[j]/2.0), (U[j+12] + dt*K2[j+12]/2.0));
        }
      }
      temp1 *= w_k;
      temp2 = w_y * y_precalc[i] + w_s * s_coxfem[i];//y((U[i] + dt*K2[i]/2.0), (U[i+12] + dt*K2[i+12]/2.0)); //+ w_s * s(0, i);

      K3[i] = (temp0 + temp1 + temp2) / tau_x[0];
    }

    if(i >= 6)
    {
      temp1 = w_k * y_precalc[i-6];//y((U[i-6] + dt*K2[i-6]/2.0), (U[i+6] + dt*K2[i+6]/2.0));
      temp2 = w_y * y_precalc[i] + w_s * s_coxfem[i];//y((U[i] + dt*K2[i]/2.0), (U[i+12] + dt*K2[i+12]/2.0)); //+ w_s * s(1, i) ;

      K3[i] = (temp0 + temp1 + temp2) / tau_x[1];
    }

    K3[i+12] = (y_precalc[i] - 0.5) / tau_b;
  }
}

void funcK4(float dt)
{
  double y_precalc[12];

  for(int i = 0; i < 12; i++)
  {
    y_precalc[i] = y((U[i] + dt*K3[i]), (U[i+12] + dt*K3[i+12]));
  }

  for(int i = 0; i < 12; i++)
  {
    float temp0 = -(U[i] + dt*K3[i]);
    float temp1 = 0;
    float temp2 = 0;

    if(i < 6)
    {
      for(int j = 0; j < 6; j++)
      {
        if(i != j)
        {
          temp1 += y_precalc[j];
        }
      }
      temp1 *= w_k;
      temp2 = w_y * y_precalc[i] + w_s * s_coxfem[i];//y((U[i] + dt*K3[i]), (U[i+12] + dt*K3[i+12])); //+ w_s * s(0, i);

      K4[i] = (temp0 + temp1 + temp2) / tau_x[0];
    }

    if(i >= 6)
    {
      temp1 = w_k * y_precalc[i-6];//y((U[i-6] + dt*K3[i-6]), (U[i+6] + dt*K3[i+6]));
      temp2 = w_y * y_precalc[i] + w_s * s_coxfem[i];//y((U[i] + dt*K3[i]), (U[i+12] + dt*K3[i+12])); //+ w_s * s(1, i) ;

      K4[i] = (temp0 + temp1 + temp2) / tau_x[1];
    }

    K4[i+12] = (y_precalc[i] - 0.5) / tau_b;
  }
}

void RK4(float dt)
{
  funcK1();
  funcK2(dt);
  funcK3(dt);
  funcK4(dt);
  for(int i = 0; i < 24; i++)
  {
    U[i] = U[i] + dt * (K1[i] + 2.0*K2[i] + 2.0*K3[i] + K4[i])/6.0; 
  }
}
//#####################################################################################################

// OTHER - ROBOT CONTROL FUNCTIONS ####################################################################

//This calculates the displacement range limits of the coxa and femur joints (COXA_MIN/MAX, FEMUR_MIN/MAX)... It should be called once in the setup
void MINMAX_values(int dir){
    threshold_coxa = abs(rad_to_servo(threshold_coxa));
    threshold_femur = abs(rad_to_servo(threshold_femur));
    for (int i = 0; i < NR; i++){
        if (COXA[i] - threshold_coxa > COXA_min[i])
            COXA_MIN[i] = COXA[i] - threshold_coxa;
        else
            COXA_MIN[i] = COXA_min[i];
    
        if (COXA[i] + threshold_coxa < COXA_max[i])
            COXA_MAX[i] = COXA[i] + threshold_coxa;
        else
            COXA_MAX[i] = COXA_max[i];
    
        if (FEMUR[i] + threshold_femur < FEMUR_max[i])
            FEMUR_MAX[i] = FEMUR[i] + threshold_femur;
        else
            FEMUR_MAX[i] = FEMUR_max[i];
        
        if (FEMUR[i] - threshold_femur > FEMUR_min[i])
            FEMUR_MIN[i] = FEMUR[i] - threshold_femur;
        else
            FEMUR_MIN[i] = FEMUR_min[i];
    }
    
    //Mirroring the robot’s two sides
        
    int minmax_temp;
    for (int i = 0; i < NR - 3; i++){
        minmax_temp = FEMUR_MIN[i];
        FEMUR_MIN[i] = FEMUR_MAX[i];
        FEMUR_MAX[i] = minmax_temp;
    }
    if (dir == 1){
        for (int i = 3; i < NR ; i++){
        minmax_temp = COXA_MIN[i];
        COXA_MIN[i] = COXA_MAX[i];
        COXA_MAX[i] = minmax_temp;
        }
    }
    if (dir == -1){
        for (int i = 0; i < NR -3 ; i++){
        minmax_temp = COXA_MIN[i];
        COXA_MIN[i] = COXA_MAX[i];
        COXA_MAX[i] = minmax_temp;
        }
    }
}

//This calculates the next value of the coxa and femur joints based on the neurons' activity --> the joints should be moved to these positions in the next step
void COXA_FEMUR_values(){
    for (int i = 0; i < 12; i++)
    {
      if(i < 6)
      {
        previous_target_pose[i] = COXA[i];
        COXA[i] = COXA_MIN[i] + y_coxfem[i] * (COXA_MAX[i]-COXA_MIN[i]);
      }
      else
      {
        previous_target_pose[i] = FEMUR[i-6];
        FEMUR[i-6] = FEMUR_MIN[i-6] + y_coxfem[i] * (FEMUR_MAX[i-6]-FEMUR_MIN[i-6]);
      }
    }
}

//In joint mode, it sets the target position of the joints
void make_next_pose(){
  //bioloid konyvtarban uj fuggveny, hogy gyorsabb legyen
    bioloid.setNextPose2(5, (int)ID_to_LEG[5]);
    bioloid.setNextPose2(6, (int)ID_to_LEG[6]);
    bioloid.setNextPose2(11, (int)ID_to_LEG[11]);
    bioloid.setNextPose2(12, (int)ID_to_LEG[12]);
    bioloid.setNextPose2(17, (int)ID_to_LEG[17]);
    bioloid.setNextPose2(18, (int)ID_to_LEG[18]);
}

//This calculates which joint encoder value corresponds to the angle given in radians
int rad_to_servo(float rads){
  float val = (rads*100)/51 * 100;
  return (int) val;
}

//It measures and outputs the battery voltage value
void voltage_test(){
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.print ("System Voltage: ");
  Serial.print (voltage);
  Serial.println (" volts.");
  if (voltage < 10.0)
    while(1);
  Serial.flush();
}

//It generates random initial values for x and b
void random_start_param(){
  for ( int i = 0; i < 24 ; i++){
    U[i] = random(-100, 100) / 20.0;
  }
  for ( int i = 0; i < 24 ; i++){
    Serial.print(U[i]);
    Serial.print(' ');
    if(i == 11)
    {
      Serial.println();
    }
  }
  Serial.println();
}

//#####################################################################################################

// SET - ROBOT CONTROL FUNCTIONS ####################################################################
//This sets the Return Delay Time for each servo (EEPROM memory, it only needs to be done once and the servo will remember it even after being powered off)
void set_delay_time(int time){
 for ( int i = 1; i < SERVONR + 1 ; i++){
    ax12SetRegister(i, AX_RETURN_DELAY_TIME, time);
  }
 delay(1000);
 
 for ( int i = 1; i < SERVONR + 1 ; i++){
    Serial.println(ax12GetRegister(i, AX_RETURN_DELAY_TIME, 1));
  }
}

//This sets the maximum allowable torque value for the servos... it only needs to be done once in the setup
void set_max_torq(int tc, int tf, int tt){
  for ( int i = 1; i < NR + 1 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_TORQUE_LIMIT_L, tc);
    delay(10);
  }
  for ( int i = 7; i < NR + 7 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_TORQUE_LIMIT_L, tf);
    delay(10);
  }
  for ( int i = 13; i < NR + 13 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_TORQUE_LIMIT_L, tt);
    delay(10);
  }
 /*
 for ( int i = 1; i < SERVONR + 1 ; i++){
    Serial.println(ax12GetRegister(i, AX_TORQUE_LIMIT_L, 2));
  }
  */
}

//It sets the value of the servos' speed (torque)... sending the value to each servo individually
void set_speed(int sc, int sf){
 for ( int i = 1; i < NR + 1 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_GOAL_SPEED_L, sc);
    delay(10);
  }
  for ( int i = 7; i < NR + 7 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_GOAL_SPEED_L, sf);
    delay(10);
  }
  for ( int i = 13; i < NR + 13 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_GOAL_SPEED_L, sf);
    delay(10);
  }
 /*
 for ( int i = 1; i < SERVONR + 1 ; i++){
    Serial.println(ax12GetRegister(i, AX_GOAL_SPEED_L, 2));
  }*/
}

//It sets the M value of the servos' speed (torque)... sending the value to each servo individually... used in the loop
void set_speed_loop(){
 for ( int i = 1; i < NR + 1 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_GOAL_SPEED_L, M[i-1]);
  }
  for ( int i = 7; i < NR + 7 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_GOAL_SPEED_L, M[i-1]);
  }
  for ( int i = 13; i < NR + 13 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_GOAL_SPEED_L, M[i-1]);
  }
}

//It assembles the message bit sequence and sets the servos' torque/speed M value synchronously/all at once
void setMovingSpeed()
{
    int temp;
    int length = 4 + (SERVONR * 3);   // 3 = id + pos(2byte)
    int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_SPEED_L;
    setTXall();
    ax12write(0xFF);
    ax12write(0xFF);
    ax12write(0xFE);
    ax12write(length);
    ax12write(AX_SYNC_WRITE);
    ax12write(AX_GOAL_SPEED_L);
    ax12write(2);
    for(int i = 1; i < SERVONR + 1; i++)
    {
        temp = M[i-1];
        checksum += (temp&0xff) + (temp>>8) + ((int)LEG_to_ID[i]); 
        ax12write((int)LEG_to_ID[i]);
        ax12write(temp&0xff);
        ax12write(temp>>8);
    } 
    ax12write(0xff - (checksum % 256));
    setRX(0);
}


void setZeroSpeed()
{
    int temp;
    int length = 4 + (SERVONR * 3);   // 3 = id + pos(2byte)
    int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_SPEED_L;
    setTXall();
    ax12write(0xFF);
    ax12write(0xFF);
    ax12write(0xFE);
    ax12write(length);
    ax12write(AX_SYNC_WRITE);
    ax12write(AX_GOAL_SPEED_L);
    ax12write(2);
    for(int i = 1; i < SERVONR + 1; i++)
    {
        temp = 0;
        checksum += (temp&0xff) + (temp>>8) + ((int)LEG_to_ID[i]); 
        ax12write((int)LEG_to_ID[i]);
        ax12write(temp&0xff);
        ax12write(temp>>8);
    } 
    ax12write(0xff - (checksum % 256));
    setRX(0);
}

//It sets the servos to JOINT mode
void set_joint_mode()
{
  for ( int i = 1; i < NR + 1 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_CCW_ANGLE_LIMIT_L, 1023);
    delay(10);
  }
  for ( int i = 7; i < NR + 7 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_CCW_ANGLE_LIMIT_L, 1023);
    delay(10);
  }
  for ( int i = 13; i < NR + 13 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_CCW_ANGLE_LIMIT_L, 1023);
    delay(10);
  }
}

//It sets the servos to WHEEL mode
void set_wheel_mode()
{
  for ( int i = 1; i < NR + 1 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_CCW_ANGLE_LIMIT_L, 0);
    delay(10);
  }
  for ( int i = 7; i < NR + 7 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_CCW_ANGLE_LIMIT_L, 0);
    delay(10);
  }
  for ( int i = 13; i < NR + 13 ; i++){
    ax12SetRegister2(LEG_to_ID[i], AX_CCW_ANGLE_LIMIT_L, 0);
    delay(10);
  }
}
//#####################################################################################################

// ERROR CORRECTION - ROBOT CONTROL FUNCTIONS ####################################################################

//It corrects the measurement errors occurring in the current position measurement
void error_correction(int i){
  int error = 0;
  
  error = abs(previous_target_pose[i] - current_pose[i]); /// (float)(previous_target_pose[i])));
  
  if(current_pose[i] < 0)
  {
    //current_pose[i] = previous_target_pose[i];
    current_pose[i] = temp_current_pose[i];
  }
  else if(error > error_treshold)
  {
    current_pose[i] = ax12GetRegister(LEG_to_ID[i+1], AX_PRESENT_POSITION_L, 2);
  }
  
  
  /*
  if((error < error_treshold) && (error_num[i] == 1))
  {
    error_num[i] = 0;
  }
  
  if((error > error_treshold) && (error_num[i] == 0))
  {
    error_num[i] = 1;
    current_pose[i] = previous_target_pose[i];
  }
  */
  
  
  /*
  if(current_pose[i] < 0)
  {
    current_pose[i] = temp_current_pose[i];
  } 
  */  
}

//It checks that the servomotors are moving within the proper, safe range
void pose_check(int i)
{
  if((current_pose[i] <= 160) && (current_pose[i] > 0))
  {
    set_speed(0, 0);
    Serial.print("POS: ");
    Serial.print(current_pose[i]);
    Serial.print(" ;ID: ");
    Serial.print(LEG_to_ID[i+1]);
    Serial.println(" - LOW CURRENT POSE ERROR!");
    while(1);
  }
  
  if((current_pose[i] >= 860) && (current_pose[i] > 0))
  {
    set_speed(0, 0);
    Serial.print("POS: ");
    Serial.print(current_pose[i]);
    Serial.print(" ;ID: ");
    Serial.print(LEG_to_ID[i+1]);
    Serial.println(" - HIGH CURRENT POSE ERROR!");
    while(1);
  }
}

//#####################################################################################################

// PYTHON COMMUNICATION FUNCTIONS ####################################################################

//This function implements the connection between the Raspberry (Python) and the ArbotixM and receives the parameters coming from Python
void python_connection()
{
  while(connect_flag == 0)
  {
    if (Serial.available() > 0) {
    python_data = Serial.readStringUntil('\n');
    //Serial.print("You sent me1: ");
    //Serial.println(python_data);
    if (python_data == "connect")
    {
      Serial.println("CONNECTED!");
      connect_flag = 1;
    }
  }
  }
  
  while(connect_flag == 1)
  {
    //DisablingServos(false);
    if (Serial.available() > 0) {
    python_data = Serial.readStringUntil('\n');
    //Serial.print("You sent me1: ");
    //Serial.println(data);
    }
    if(python_data[0] == 'a')
     {
      szam = ""; 
      Serial.println("#GET!");
      connect_flag = 2;
        while(python_data[python_index] != 'b')
        {
          szam += python_data[python_index];
          python_index = python_index + 1;
        }
        w_y = szam.toFloat();
      }
      if(python_data[python_index] == 'b')
      {
        szam = "";
        while(python_data[python_index] != 'c')
        {
          if(python_data[python_index] != 'b')
          {
            szam += python_data[python_index];
          }
          python_index = python_index + 1;
        }
        w_s = szam.toFloat();
      }
      if(python_data[python_index] == 'c')
      {
        szam = "";
        while(python_data[python_index] != 'd')
        {
          if(python_data[python_index] != 'c')
          {
           szam += python_data[python_index]; 
          }
          python_index = python_index + 1;
        }
        w_k = szam.toFloat();
      }
      if(python_data[python_index] == 'd')
      {
        szam = "";
        while(python_data[python_index] != 'z')
        {
          if(python_data[python_index] != 'd')
          {
           szam += python_data[python_index]; 
          }
          python_index = python_index + 1;
        }
        random_par = szam.toFloat();
      }
  }
  Serial.print("#w_y = ");
  Serial.println(w_y);
  Serial.print("#w_s = ");
  Serial.println(w_s);
  Serial.print("#w_k = ");
  Serial.println(w_k);
  Serial.print("#random_par = ");
  Serial.println(random_par);
  Serial.print("#tau_x: ");
  Serial.print(tau_x[0], 3);
  Serial.print(" ");
  Serial.println(tau_x[1], 3);
  Serial.print("#tau_b: ");
  Serial.println(tau_b);
  Serial.print("#k: ");
  Serial.println(k);
  Serial.print("#gamma: ");
  Serial.println(gamma);
  delay(100);
  if (random_par == 1){
    random_start_param();
  }
}

//After stopping the Raspberry’s Python program, this function receives the stop command sent by Python and stops the robot
void serial_break()
{
  if (Serial.available() > 0) {
    char command = Serial.read();
    //Serial.print("command: ");
    //Serial.println(command);
    if (command == 'S') {
      // Received a stop command from Python
      // Perform any necessary cleanup
      // Exit loop or reset as needed
      set_speed(0, 0);
      Serial.println("#STOP!");
      while(1);
     }
  }
}
