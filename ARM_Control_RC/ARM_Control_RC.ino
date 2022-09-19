#include <string.h>
#include <Servo.h>
#include <AccelStepper.h>

#define IBUS_BUFFSIZE 32    
#define IBUS_MAXCHANNELS 10 // I am using only 10 channels because my TX (FlySky i6) supports max 10 channels

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 3
#define dirPin1 4
#define stepPin1 5
#define dirPin2 6
#define stepPin2 7 
#define motorInterfaceType 1
#define channel_default 1500


static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint16_t rcValue[IBUS_MAXCHANNELS];
static boolean rxFrameDone;

int ch_width_1 = 1500;
int ch_width_2 = 1500;
int ch_width_3 = 1500;
int ch_width_4 = 1500;

int rot = 0;
int left = 0;
int right = 0;
int rot_data = 0;
int left_data = 0;
int right_data = 0;
bool data_present = false;
bool data_arrived = false;


// Create a new instance of the AccelStepper class:
AccelStepper left_motor = AccelStepper(motorInterfaceType, stepPin, dirPin);
AccelStepper center_motor = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper right_motor = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

void move(int motor,int steps);
void readRx();

void setup() {
  // Set the maximum speed and acceleration:
  left_motor.setMaxSpeed(400);
  left_motor.setAcceleration(100);
  center_motor.setMaxSpeed(400);
  center_motor.setAcceleration(100);
  right_motor.setMaxSpeed(400);
  right_motor.setAcceleration(100);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("Program Started");
}

void loop() 
{
  if(Serial.available()>0 )readRx();
  if(data_arrived)checkdata();
  if(data_present)
  {
   // Serial.println("db2");
    if(rot_data !=0)
    {
    move(2,rot_data);
    rot += rot_data;
    }
    if(left_data != 0 )
    {
   move(1,left_data);
    left += left_data;
    }
    if(right_data != 0)
    {
    move(3,right_data);
    right += right_data;
    }
    
    Serial.print("Rot: ");
    Serial.print(rot);
    Serial.print(" Left: ");
    Serial.print(left);
    Serial.print(" Right: ");
    Serial.println(right);
    data_present = false;
  }
}

void readRx()
{
rxFrameDone = false;
                   
if (Serial.available()>0)
{
  uint8_t val = Serial.read();
  // Look for 0x2040 as start of packet
  if (ibusIndex == 0 && val != 0x20)
  {
    ibusIndex = 0;
    return;
  }
  if (ibusIndex == 1 && val != 0x40)
  {
    ibusIndex = 0;
    return;
  }

  if (ibusIndex == IBUS_BUFFSIZE)
  {
    ibusIndex = 0;
    int high=3;
    int low=2;
    for(int i=0;i<IBUS_MAXCHANNELS; i++)
    {
      rcValue[i] = (ibus[high] << 8) + ibus[low];
      high += 2;
      low += 2;
    }
    ch_width_1 = map(rcValue[0], 1000, 2000, 1000, 2000);
   // ch1.writeMicroseconds(ch_width_1);

  //  Serial.print(ch_width_1);
  //  Serial.print("     ");

    ch_width_2 = map(rcValue[1], 1000, 2000, 1000, 2000);
   // ch2.writeMicroseconds(ch_width_2);

  //  Serial.print(ch_width_2);
  //  Serial.print("     ");

    ch_width_3 = map(rcValue[2], 1000, 2000, 1000, 2000);
  //  ch3.writeMicroseconds(ch_width_3);

  //  Serial.print(ch_width_3);
  //  Serial.print("     ");

    ch_width_4 = map(rcValue[3], 1000, 2000, 1000, 2000);
   // ch4.writeMicroseconds(ch_width_4);
   
  //  Serial.print(ch_width_4);
  //  Serial.println("     ");
                            
    rxFrameDone = true;
    data_arrived = true;

    rot_data = 0;
    left_data = 0;
    right_data = 0;
  int total = (abs(ch_width_1 - channel_default) 
                  + abs(ch_width_2 - channel_default)
                  + abs(ch_width_3 - channel_default)
                  + abs(ch_width_4 - channel_default));

  bool condition = total>100;
  //Serial.println(condition);
   

  if(condition)
  {
    if((ch_width_4 - channel_default) > 100)
      rot_data += 5; //(ch_width_4 - channel_default)/10;
    if((ch_width_3 - channel_default) > 100)
      right_data += 5; //(ch_width_3 - channel_default)/10;
    if((ch_width_2 - channel_default) > 100)
      left_data += 5; //(ch_width_2 - channel_default)/10;

    if((channel_default - ch_width_4) > 100)
      rot_data -= 5; //(ch_width_4 - channel_default)/10;
    if((channel_default - ch_width_3) > 100)
      right_data -= 5; //(ch_width_3 - channel_default)/10;
    if((channel_default - ch_width_2) > 100)
      left_data -= 5; //(ch_width_2 - channel_default)/10;

    data_present = true;
    data_arrived = false;
  }
    return;
  }
  else
  {
    ibus[ibusIndex] = val;
    ibusIndex++;
  }
  
}
}


void move(int motor,int steps)
{
  // Set the target position:
  if(motor ==1)
  {
    left_motor.move(steps);
    left_motor.setSpeed(800);
    while(left_motor.distanceToGo() != 0)
    // Run to target position with set speed and acceleration/deceleration:
     left_motor.runSpeedToPosition();
  }
  if(motor ==2)
  {
    center_motor.move(steps);
    center_motor.setSpeed(800);
    while(center_motor.distanceToGo() != 0)
    // Run to target position with set speed and acceleration/deceleration:
     center_motor.runSpeedToPosition();
  }
  if(motor ==3)
  {
    right_motor.move(steps);
    right_motor.setSpeed(800);
    while(right_motor.distanceToGo() != 0)
    // Run to target position with set speed and acceleration/deceleration:
     right_motor.runSpeedToPosition();
  }
}

void checkdata()
{
 //Serial.println("db2");
  // rot_data = 0;
  // left_data = 0;
  // right_data = 0;
  // int total = (abs(ch_width_1 - channel_default) 
  //                 + abs(ch_width_2 - channel_default)
  //                 + abs(ch_width_3 - channel_default)
  //                 + abs(ch_width_4 - channel_default));

 // bool condition = total>500;
  //Serial.println(condition);
   

/*
  if(condition)
  {
    if(abs(ch_width_4 - channel_default) > 100)
      rot_data = (ch_width_4 - channel_default)/10;
    if(abs(ch_width_4 - channel_default) > 100)
      right_data = (ch_width_3 - channel_default)/10;
    if(abs(ch_width_4 - channel_default) > 100)
      left_data = (ch_width_2 - channel_default)/10;
    data_present = true;
    data_arrived = false;
  }*/
}
