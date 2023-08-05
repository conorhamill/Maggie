#ifndef functions_h
#define functions_h

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

class Gripper
{
  private:
    uint8_t gripper_pin;
    Servo gripper_servo;
  public:
    void begin(uint8_t gripper_pin)
    {
      gripper_servo.attach(gripper_pin);  
    }
    void open()
    {
      gripper_servo.write(100);
      delay(500);
    }
    void close()
    {
      gripper_servo.write(40);
      delay(500);
    }    
};

class Arm
{
  private:
  
    Servo servo[2]; //Servo[0]=TopArm, Servo[1]=MiddleArm

    // servo Angles
    double angle_current[2] = {90,90};
    double angle_next[2] = {90, 90};
    
    // make sure that the servo angle position, correctly translates to the trigonometric formulas of the arm position (see docs)
    // set the servo to 90 degress, then measure the real angle of the arm.
    // the calibration value is (90 - {position of arm vs reference at servo 90}), which then helps all values translate to sevo position.
    const double calibrate_TopArm=90-45; // was 90-35.7
    const double calibrate_MiddleArm=90-43; // was 90-41.4
    
    //Coordintates for current position
    double yCurrent;
    double zCurrent;

  public: 
    void begin(uint8_t servo_top_pin, uint8_t servo_middle_pin)
    {
      servo[0].attach(servo_top_pin, 500, 2500); //pin, min, max
      servo[1].attach(servo_middle_pin, 500, 2500); //pin, min, max
      angle_current[0]=90;
      angle_current[1]=90;  
    }
    void get_angles_from_yz(double y, double z) 
    {
      //refer to trigonometry illustration for variable description
      double H, s1, s2, aB, aA, aQ, servo1angle, servo2angle, y2, z2, y3, z3;
    
      //arm length in cm
      int L = 13;
    
      H= sqrt (pow(y,2) + pow(z,2));
      s1=H/2;
      s2=sqrt (pow(L,2) - pow(s1,2));
      aB=atan(s2/s1);
      y2=y/2;
      z2=z/2;
      aA=atan(y2/z2);
      servo1angle=aA+aB;
      servo1angle= (servo1angle/ (2 * M_PI)) * 360;
    
      //matrix multiplication - counterclockwise rotation
      y3 = -L*sin(aA+aB);
      z3 = -L* cos(aA+aB);
      servo2angle=atan((y-y3)/(z-z3));  
      servo2angle= (servo2angle / (2 * M_PI)) * 360;
      
      //tangent calculation changes when servo2 exceeds 90 degrees, correction below
      if ((z-z3)>0) 
      {
        servo2angle=servo2angle-180;
      }
    
      //Absolute Top Arm Angle
      //Top Arm moves 0 to +90
      angle_next[0] = servo1angle;
    
      //Absolute Middle Arm Angle
      //Middle Arm moves 0 to +90
      angle_next[1] = -servo2angle;
    
      //Convert to SERVO Angle
      //in this case, a 90 servo position is equal to 71 degrees for Top arm
      //90 servo position is equal to 65 Middle Arm
      angle_next[0] = angle_next[0] + calibrate_TopArm; //servo[0].write(angle_next[0]);
      angle_next[1] = angle_next[1] + calibrate_MiddleArm; //servo[1].write(angle_next[1]);
    }

    void servo_steps(int servo_num, double angle_target, double incr_step = 10, int step_delay = 50) 
    {
      /*
       * Using LD-20MG Servos with an Arduino Mega 2560, they only work smoothly on maximum 25 degree instructions at a time.
       * I haven't debugged the hardware, but this functions solves the issue.      
       * This function helps you send commands to servos to move in a set of degrees at at a time. 
       */
    
      int set_angle;
      int angle_start = angle_current[servo_num];
    
      if (angle_start > angle_target) 
      {
        //start from angle_start, and then move the servo by incr_steps into the angle_target
        //stepping down to target
        for (set_angle = angle_start; set_angle >= angle_target; set_angle -= incr_step) 
        {
          servo[servo_num].write(set_angle);
          delay(step_delay);
        }
      }
      else 
      {
        //stepping up to target
        for (set_angle = angle_start; set_angle <= angle_target; set_angle += incr_step) 
        {
          servo[servo_num].write(set_angle);
          delay(step_delay);
        }
      }
      // make sure the servo arrives at the target
      servo[servo_num].write(angle_target);
      //update the current angle
      angle_current[servo_num] = angle_target;
    }

    void twoarm_step_coordinate(double toparm_target, double middlearm_target) 
    {
      double incr_steps0=1;
      double incr_steps1= 1;
      int inner_step_delay0 = 0;
      int inner_step_delay1 = 0;
      int outer_step_delay = 30;
      double i, j;
      int e0 = 0;
      int e1 = 0;
    
      //identify which of the arms has a greater delta in terms of degress to move
      double delta0 = abs(angle_current[0] - toparm_target);
      double delta1 = abs(angle_current[1] - middlearm_target);
    
      //coordinate the speed of the two access through the incremental steps, so they can move smoothly in the x/y plane
      //(this avoids one arm finishing its movements first, and generating huge variagtionsin the real x/y position of the endpoint)
      if (delta0!=0 && delta1!=0) 
      {
        if (delta0 >= delta1) 
        {
          incr_steps0 = (delta0 / delta1)*incr_steps1;
          //slow down motion as steps increase (just in case big jump in steps)
          inner_step_delay0=(delta0/delta1)*0.5;
          //reduce the outer step
          outer_step_delay=outer_step_delay-inner_step_delay0;
        }
        else 
        {
          incr_steps1 = (delta1 / delta0)*incr_steps0;
          //slow down motion as steps increase (just in case big jump in steps)
          inner_step_delay1=(delta1/delta0)*0.5;
          //reduce the outer step
          outer_step_delay=outer_step_delay-inner_step_delay1;
        }
      }
      //set to zero if negative value on outer delay
      if (outer_step_delay<0) 
      {
        outer_step_delay=0;
      }
      
      //identify the direction of steps
      if (angle_current[0] > toparm_target) 
      {
        i = -incr_steps0;
      }
      else
      {
        i = incr_steps0;
      }
      if (angle_current[1] > middlearm_target) 
      {
        j = -incr_steps1;
      } 
      else 
      {
        j = incr_steps1;
      }
      
      // user the servo step functions, doing inter-twined steps until the gaps are reached.
      // we send a delay of 0 to the servo step function, given we'll control the delay in this outer loop.
      while (1) 
      {
        // top arm moves
        if (abs(angle_current[0] - toparm_target) > incr_steps0) 
        {
          servo_steps(0, angle_current[0] + i, incr_steps0, inner_step_delay0);
        } 
        else 
        {
          servo_steps(0, toparm_target, incr_steps0, inner_step_delay0);
          e0 = 1;
        }
        // middle arm moves
        if (abs(angle_current[1] - middlearm_target) > incr_steps1) 
        {
          servo_steps(1, angle_current[1] + j, incr_steps1, inner_step_delay1);
        } 
        else 
        {
          servo_steps(1, middlearm_target, incr_steps1, inner_step_delay1);
          e1 = 1;
        }
        delay(outer_step_delay);
        if ((e0 + e1) >= 2) break;
      }
    }
    void home()
    {
      //arm moves to Start position
      get_angles_from_yz(0,-16);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);
      //sets new co-ordinates
      yCurrent = 0;
      zCurrent = -16;
    
      delay(500);
    }
    void move(double yMove, double zMove)
    {   
      // move arms in Y direction first
      get_angles_from_yz(yMove, zCurrent);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);
      // move arms in Z direction
      get_angles_from_yz(yMove, zMove);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);

      //sets new co-ordinates
      double yCurrent = yMove;
      double zCurrent = zMove;
      
      delay(500);
    }
    void max_travel(){
      Arm::move(-9,-16.5);
      Arm::move(-9,-20);
      Arm::move(7,-21);
      Arm::move(10,-13);
      Arm::home();
    }
};

class Button
{
  private:
    uint8_t btn;
    uint16_t state;
  public:
    void begin(uint8_t button) 
    {
      btn = button;
      state = 0;
      pinMode(btn, INPUT);
    }
    bool debounce() 
    {
      state = (state<<1) | digitalRead(btn) | 0xfe00;
      return (state == 0xff00);
    }
};

#endif
