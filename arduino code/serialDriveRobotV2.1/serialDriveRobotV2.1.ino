
/*
 *************************************************************************************
basic serial/serial1 driven robot code
**************************************************************************************
 *  using SPI, here is an example hardware setup:
  LSM9DS0 --------- Arduino
          CSG -------------- 17
          CSXM ------------- 9
          SDOG ------------- 12
          SDOXM ------------ 12 (tied to SDOG)
          SCL -------------- 13
          SDA -------------- 11
          VDD -------------- 3.3V
          GND -------------- GND
 */
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>

#define LSM9DS0_CSG  17  // CSG connected to Arduino pin 9
#define LSM9DS0_CSXM 9 // CSXM connected to Arduino pin 10
LSM9DS0 dof(MODE_SPI, LSM9DS0_CSG, LSM9DS0_CSXM);

// Do you want to print calculated values or raw ADC ticks read
// from the sensor? Comment out ONE of the two #defines below
// to pick:
#define PRINT_CALCULATED
//#define PRINT_RAW

#define PRINT_SPEED 50 // keep this the same as the looptime (defined below)

//Motor pin assignments
//swapped for layout improvements
int RMOTOR_F = 6;
int RMOTOR_B = 5;
int LMOTOR_B = 11;
int LMOTOR_F = 10;

//Motor directions
char RDIR = 'f';
char LDIR = 'f';

//Wheel sensor pins
int LSENSOR = 7;
int RSENSOR = 2;

//Wheel conversion
float TICKS_PER_REV = 384; //8 state changes per rev, 48:1 gearbox
float  MM_PER_REV = 202; //65 mm wheel * pi = circumference
float MM_PER_TICK = MM_PER_REV / TICKS_PER_REV; //~0.5mm / tick

//robot Axle length
float AXLE_LEN = 162; //mm mid-width of wheel to mid-width of wheel

//Loop update rate in ms
unsigned long LOOPTIME = 50;  //50ms = 20 Hz
unsigned long start_time;
unsigned long curr_time;

volatile long left_ticks;
volatile long right_ticks;
long left_ticks_old;
long right_ticks_old;

struct wheelState {
  float arc_left;
  float arc_right;
  float arc_left_dot;
  float arc_right_dot;
} wheels;

struct odometryState {
  float mag_x;  //magnetometer strength in x direction
  float mag_y; //magnetometer stremgth in y direction
  float theta_dot; //gyro angular velocity around z
  float d2xdt2;  // accelerometer in x direction
  float d2ydt2; //accelerometer in y direction
} odometry;

struct robotState {
  float x;
  float y;
  float theta;
  float theta_mag;
} robot;

//controller inputs:
int v_left;
int v_right;
//Serial1 delimeters:
char myChar = 'A';
char left_bracket[1] = "[";
char right_bracket[1] = "]";


void setup() {
  pinMode(LSENSOR, INPUT_PULLUP); //interrupt pins, want pullup mode enabled
  pinMode(RSENSOR, INPUT_PULLUP);
  Serial1.begin(57600);  //XBee/UART1/pins 0 and 1
  Serial1.setTimeout(1000);
  Serial.begin(9600);   //USB

  uint16_t status = dof.begin();
  // Or call it with declarations for sensor scales and data rates:
  //uint16_t status = dof.begin(dof.G_SCALE_2000DPS,
  //                            dof.A_SCALE_6G, dof.M_SCALE_2GS);
  // begin() returns a 16-bit value which includes both the gyro
  // and accelerometers WHO_AM_I response. You can check this to
  // make sure communication was successful.
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();

  pinMode(LMOTOR_B, OUTPUT);   // set LMOTOR_B as output
  pinMode(LMOTOR_F, OUTPUT);   // set LMOTOR_F as output
  pinMode(RMOTOR_B, OUTPUT);   // set RMOTOR_B as output
  pinMode(RMOTOR_F, OUTPUT);   // set RMOTOR_F as output
  pinMode(LSENSOR, INPUT_PULLUP);
  pinMode(RSENSOR, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LSENSOR), left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RSENSOR), right, CHANGE);
  left_ticks = 0;
  right_ticks = 0;
  left_ticks_old = 0;
  right_ticks_old = 0;

  // initialize the wheels states
  wheels.arc_left = 0;
  wheels.arc_right = 0;
//  wheels.arc_left_dot = 0;
//  wheels.arc_right_dot = 0;

  //initialize odometry
  updateOdometry();

  //initialize the robot state to zero with heading determined by magnetometer
  robot.theta_mag = calcHeading(odometry.mag_x,odometry.mag_y);
  robot.theta = robot.theta_mag;
  robot.x = 0.0;
  robot.y = 0.0;

  //intialize the contoller inputs:
  v_left = 0;
  v_right = 0;

  //set loop timer
  start_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  //check loop timer
  curr_time = millis();
  //check for controller command
  if (Serial1.available() > 0 ) {
//    if(Serial1.findUntil(left_bracket,right_bracket)){
      v_left = Serial1.parseInt();
      v_right = Serial1.parseInt();
      Serial.print("v_left: ");
      Serial.print(v_left);
      Serial.print("; v_right: ");
      Serial.println(v_right);
//    }

    }
    //get rid of any other spurious data but print it first
    while (Serial1.available() > 0) {
      char junk = Serial1.read();
      //Serial.print(junk);
    }  

  if (curr_time - start_time >= LOOPTIME) {
    updateState();
    updateOdometry();
    updateRobot();
    
    start_time = curr_time;
//    Serial.println();
//    Serial.print("Left wheel distance = ");
//    Serial.print(wheels.arc_left);
//    Serial.print("; right wheel distance = ");
//    Serial.print(wheels.arc_right);
//    Serial.print("; mag_x = ");
//    Serial.print(odometry.mag_x);
//    Serial.print("; mag_y = ");
//    Serial.println(odometry.mag_y);
//    Serial.print("angular velocity = ");
//    Serial.print(odometry.theta_dot);
//    Serial.print("; accel(x) = ");
//    Serial.print(odometry.d2xdt2);
//    Serial.print("; accel(y) = ");
//    Serial.print(odometry.d2ydt2);
//    Serial.println();
//    Serial.print("robot.x= ");
//    Serial.print(robot.x);
//    Serial.print("; robot.y = ");
//    Serial.print(robot.y);
//    Serial.print("; heading(mag) = ");
//    Serial.print(robot.theta_mag);
//    Serial.print("; heading(calc) = ");
//    Serial.println(robot.theta);

    //odometry string to the controller
    //TODO: update to just provide robot.x robot.y, robot.heading, robot_heading_mag
    Serial1.print("[ ");
    Serial1.print(robot.x,4);
    Serial1.print(" ");
    Serial1.print(robot.y,6);
    Serial1.print(" ");
    Serial1.print(robot.theta);
    Serial1.print(" ");
    Serial1.print(robot.theta_mag);
//    Serial1.print(" ");
//    Serial1.print(wheels.arc_left_dot);
//    Serial1.print(" ");
//    Serial1.print(wheels.arc_right_dot);
    Serial1.print(" ");
    Serial1.print(odometry.mag_x,4);
    Serial1.print(" ");
    Serial1.print(odometry.mag_y,4);
    Serial1.print(" ");
    Serial1.print(odometry.theta_dot,4); //leave this in for troubleshooting on controller side
//    Serial1.print(" ");
//    Serial1.print(odometry.d2xdt2);
//    Serial1.print(" ");
//    Serial1.print(odometry.d2ydt2);
    Serial1.print(" ]\n");
    Serial1.flush();

    drive(v_left, v_right);
  }
}

void updateState() {
  float delta_theta;
  float delta_x;
  float delta_y;
  float arc_c;
  float rad;
  float l;
  //copy current sensor readings  not sure I need to disable interrupts
  noInterrupts();
  long left_ticks_copy = left_ticks;
  long right_ticks_copy = right_ticks;
  interrupts();
  long delta_left_ticks = left_ticks_copy - left_ticks_old;
  long delta_right_ticks = right_ticks_copy - right_ticks_old;

  left_ticks_old = left_ticks_copy;
  right_ticks_old = right_ticks_copy;

  wheels.arc_left = delta_left_ticks * MM_PER_TICK;//distance traveled by left wheel
  wheels.arc_right = delta_right_ticks * MM_PER_TICK; //distance traveled by right wheel
//  wheels.arc_left_dot = wheels.arc_left / (LOOPTIME * 1000);
//  wheels.arc_right_dot = wheels.arc_right / (LOOPTIME * 1000);
}

void updateOdometry() {
  dof.readMag();
  odometry.mag_x = dof.calcMag(dof.mx);
  odometry.mag_y = dof.calcMag(dof.my);
  dof.readGyro();
  odometry.theta_dot = dof.calcGyro(dof.gz);
  dof.readAccel();
  odometry.d2xdt2 = dof.calcAccel(dof.ax);
  odometry.d2ydt2 = dof.calcAccel(dof.ay);
}

void updateRobot() {
  //IMU is oriented towards negative -x axis.  Motion is corrected so that x motion is foward
  //Orientation is relative to magnetic north with CW positive angle
  float radius;
  float newTheta;
  float denom = wheels.arc_right - wheels.arc_left;
  float num = wheels.arc_left + wheels.arc_right;
  float dTheta = denom/AXLE_LEN; //calculated change in heading
  float s = sin(dTheta/2);
  float c = cos(dTheta/2);
  if (denom == 0) {
    robot.x = wheels.arc_left;
    robot.y = 0;
  } else {
    //changed to just displacements, delta x, between callbacks
      radius = AXLE_LEN*num/denom;
      robot.x = 2*radius*s*c;
      robot.y = 2*radius*s*s; 

  }
  robot.theta_mag = calcHeading(odometry.mag_x,odometry.mag_y); //new heading from compass
  newTheta = robot.theta - dTheta*180/PI;
  if (newTheta > 360) {
    newTheta = newTheta - 360;
  }
  if (newTheta < 0) {
    newTheta = newTheta + 360;
  }
  robot.theta = dTheta; //update the calculated heading from odometry

}

void drive(int leftCommand, int rightCommand) {
  if (leftCommand > 255) {
    leftCommand = 255;
  }
  if (leftCommand < -255) {
    leftCommand = -255;
  }
  if (rightCommand > 255) {
    rightCommand = 255;
  }
  if (rightCommand < -255) {
    rightCommand = -255;
  }

  if (leftCommand  < 0) {
    LDIR = 'b';
    analogWrite(LMOTOR_F, 0);
    analogWrite(LMOTOR_B, abs(leftCommand));
  } else {
    LDIR = 'f';
    analogWrite(LMOTOR_F, leftCommand);
    analogWrite(LMOTOR_B, 0);
  }
  if (rightCommand  < 0) {
    RDIR = 'b';
    analogWrite(RMOTOR_F, 0);
    analogWrite(RMOTOR_B, abs(rightCommand));
  } else {
    RDIR = 'f';
    analogWrite(RMOTOR_F, rightCommand);
    analogWrite(RMOTOR_B, 0);
  }
}

float calcHeading(float hx, float hy)
{  float angle = atan(hy / hx) * 180 / PI;
  if (hx > 0)
  {
    return 180 + angle;
  }
  else 
  {
    if (angle < 0){
      angle = angle +360;
    }
    return  angle;
  }
}


void left() {
  if (LDIR == 'f') {
    left_ticks ++;
  } else {
    left_ticks --;
  }
}

void right() {
  if (RDIR == 'f') {
    right_ticks ++;
  } else {
    right_ticks --;
  }
}
