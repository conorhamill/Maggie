#include "functions.h"

Button limit_switches;
Gripper grip;
Arm main_arm;

//create stepper variables
uint8_t step_pin = 3;
uint8_t direction_pin = 4;

double x_axis_range;
double xCurrent;
double x_steps_from_datum;
  

void setup() 
{  
  //setup stepper motor
  pinMode(step_pin, OUTPUT);
  pinMode(direction_pin, OUTPUT);
  digitalWrite(direction_pin, HIGH);
 
  limit_switches.begin(2);  
  grip.begin(10);
  main_arm.begin(8,9);

  main_arm.home();
  x_axis_home();
}

void loop() 
{

  //first lift and push
  x_axis_move(40);  main_arm.move(0,-20);  grip.close();  main_arm.home();
  
  x_axis_move(32);  main_arm.move(0,-20);  x_axis_move(37);  grip.open();  main_arm.home();

  //second lift and push
  x_axis_move(40);  main_arm.move(0,-20);  grip.close();  main_arm.home();
  
  x_axis_move(48);  main_arm.move(0,-20);  x_axis_move(43);  grip.open();  main_arm.home();

}

void x_axis_home()
{
  //goes to extremes of x-axis and measures the step between
  //converts steps to mm and sets the x_axis travel range. Adds offset off each end. 
  //goes to 0 position.
  while (limit_switches.debounce()== false)
  {
    digitalWrite(step_pin,HIGH);
    delayMicroseconds(600);
    digitalWrite(step_pin,LOW);
    delayMicroseconds(600); 
  }
    digitalWrite(direction_pin,LOW); //change direction of motor
    int x_axis_total_steps = 0; //begin count of steps inbetween limit switches
    while (limit_switches.debounce()== false)
    {
      digitalWrite(step_pin,HIGH);
      delayMicroseconds(600);
      digitalWrite(step_pin,LOW);
      delayMicroseconds(600);
      x_axis_total_steps++;
    }
    digitalWrite(direction_pin,HIGH); //change direction of motor a final time
    //goes to datum x position. 5% of total distance away from limit switch 
    for (int i=0; i< (x_axis_total_steps*0.05); i++)
    { 
      digitalWrite(step_pin,HIGH);
      delayMicroseconds(600);
      digitalWrite(step_pin,LOW);
      delayMicroseconds(600); 
    }
    xCurrent = 0; 
    //x travel range is 90% of the distance between limit switches. 
    //x_axis_range is in cm. converted to cm per step. 20cm per 1000 steps. 0.02 cm/step 
    x_axis_range = x_axis_total_steps*0.9*0.02;
    delay(500);
}

void x_axis_move(double x_position)
{
  //x_position should be in cm
  //checks if the x_position is within the x_axis travel range
  //Calculates the delta and direction in relation to current position
  //moves the motor to that position
  if (0 < x_position <= x_axis_range)
  {
    double xDelta = x_position - xCurrent;
    if (xDelta > 0)
    {
      digitalWrite(direction_pin, HIGH);
      for (int i=0; i< (xDelta/0.02); i++)
      { 
        digitalWrite(step_pin,HIGH);
        delayMicroseconds(600);
        digitalWrite(step_pin,LOW);
        delayMicroseconds(600);
      }
    }
    if (xDelta < 0)
    {
      digitalWrite(direction_pin, LOW);
      for (int i=0; i< abs(xDelta/0.02); i++)
      { 
        digitalWrite(step_pin,HIGH);
        delayMicroseconds(600);
        digitalWrite(step_pin,LOW);
        delayMicroseconds(600);
      }
    }
    xCurrent = x_position;
    delay(500);
  }   
}

void return_home()
{
  digitalWrite(direction_pin, LOW);
  for (int i=0; i< (xCurrent/0.02); i++)
  { 
    digitalWrite(step_pin,HIGH);
    delayMicroseconds(600);
    digitalWrite(step_pin,LOW);
    delayMicroseconds(600); 
  }
  xCurrent = 0;
  delay(500);
}
