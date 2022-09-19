//used for stepper motr 
#define STEPPER

//incase of arduino uno board
#define UNO
//for debugging purpose

#define DEBUG

//running with inverse kinematics
#define MATHMODEL

//value of pi
#define MATH_PI 3.14159265359f

#define RAD2DEG 180.0 / MATH_PI
#define DEG2RAD MATH_PI / 180

//variables of the chess board and arm in mm
#define CHESSX 270
#define CHESSY 270
//box size of chess board
#define BOXLENGTH 33.5625

//specific for robotic arm
#define  uStepper_roboticArm


#ifdef uStepper_roboticArm
  //gear ratio with stepper motor
  #define GEARRATIO 5.1f
  #define LOWERARMLEN 182.0f//183.0f
  #define UPPERARMLEN 188.0f//188.0f
  #define XOFFSET 47.0f // offset in the L1 vector direction
  #define ZOFFSET 73.0f//73.0f
  #define AXOFFSET 40.0f  // Actuator offset in L1 vector direction
  #define AZOFFSET -55.0f // Actuator offset in Z-direction

  // Pre-calculated offsets in motor angle at home position
  #define ROTOFFSET 0.0f
  #define SHOULDEROFFSET (DEG2RAD * 158.65f)
  #define ELBOWOFFSET (DEG2RAD * 51.5f)
  #define RAD2DEGGEARED RAD2DEG *GEARRATIO
  #define DEG2RADGEARED DEG2RAD / GEARRATIO
#endif


//Initialization of the Servos
#ifdef STEPPER
  #include "AccelStepper.h" // Include the Stepper library:
  #define StepperNumber 3 //defines numbers of stepper used in the application
  #define motorInterfaceType 1 //Motor interface type must be set to 1 when using a driver:
  byte StepPins[3] = {3,5,7,}; //step pins connection to the Arduino
  byte DirectionPins[3] = {2,4,6}; //direction pins connection to the Arduino
  AccelStepper stepper[StepperNumber];
  
#endif


//initialization of the Bluetooth ; we have used HC-05
#ifdef BLUETOOTH
  #include <NeoSWSerial.h>
  NeoSWSerial bt(2,4); //Arduino : RX,TX
  String dataIn = "";
#endif

//Initialization of the WIFI module we have used the ESP-8266
#ifdef WIFI
  #include <NeoSWSerial.h>
  SoftwareSerial wifi(2,4); //Arduino : RX,TX
  String dataIn = "";
#endif

float yValue = 0.5;

void setup() 
{
  //Assignes the pin to the Motors
  #ifdef STEPPER
  for(int i = 0; i < StepperNumber; i++)
  {
    stepper[i] = AccelStepper(motorInterfaceType, StepPins[i], DirectionPins[i]); //objects creation of the stepper
    stepper[i].setMaxSpeed(200);
    stepper[i].setAcceleration(100);
  }
  #endif

  //Initialization of Bluetooth Software serial
  #ifdef BLUETOOTH
  bt.begin(9600);
  bt.setTimeout(1);
  delay(20);
  #endif

  //Initialization of the WIFI Software serial
  #ifdef WIFI
  wifi.begin(9600);
  wifi.setTimeout(1);
  delay(20);
  #endif

  #ifdef DEBUG
  Serial.begin(9600);
  delay(100);
  Serial.println("\nProgram Started");

  //angleToxyz(0,0,0);

  //offsets for positionCalculator: x :111.71 y :0.00 z :158.56
  //x :75.08 y :0.00 z :187.14
  //x :80.06 y :0.00 z :178.69

  //for(int i=10;i<200;i++)
  float x = 180;
  // float z = 150;
  // if(x<150.0)
  //   x = 0.05*x+x;
  // else
  //   x = x+10;
  // // if(z>50.0)
  // //   z -= 0.05*z;
  
   positionCalculator(80.06+x,0,90);
  
  #endif
}

void loop() 
{
//check whether bluetooth data is available
 #ifdef BLUETOOTH
 if(bt.available()  > 0)checkData(); //if serial data available call checkdata function
 delay(10);
 #endif

//checkData of wifi
 #ifdef WIFI
 if(wifi.available() > 0)checkData();
 #endif
 //float radians = asin(0.707);
 //float degrees = (radians*180)/PI; //converts the value to the degrees
#ifdef DEBUG
//for(int i= 0; i <180; i++)
//{
 // Serial.print("value : ");
  //Serial.println(i);
  //positionCalculator(0,130,138);
  //angleToxyz(0,0,0);
//}
#endif

}

void movement(byte number, int degree)
{
  #ifdef SERVO
  Serial.print("Required pos : ");
  Serial.println(degree);
  Serial.print("Last position: ");
  Serial.println(last_position[number]);
  if (last_position[number] > degree)
  {
    Serial.print("Decreasing POS");
    for(int i = last_position[number]; i>=degree; i--)
    {
      servos[number].write(i);
      delay(20);
      Serial.println(i);
    }
  }
  if (last_position[number] < degree)
  {
    Serial.print("Increasing POS");
    for(int i = last_position[number];i<degree;i++)
    {
      servos[number].write(i);
      delay(20);
      Serial.println(i);
    }
  }
  last_position[number] = degree;
  delay(10);
  Serial.println("****************************");
  #endif
  #ifdef STEPPER
  int value = float(degree*5)/9;
  Serial.println(value);
  stepper[number].move(value);//(degree*200)/360
  while(stepper[number].distanceToGo() != 0)
  // Run to target position with set speed and acceleration/deceleration:
  stepper[number].run();
  Serial.println("Done");
  #endif
}
void checkData()
{
  //works for data in format of ":servo_num=position;"
  #ifdef BLUETOOTH
  bt.find(':');
  dataIn = bt.readStringUntil(';');
  #endif
  #ifdef WIFI
  wifi.find(':');
  dataIn = wifi.readStringUntil(';');
  #endif
  #ifdef WIFI || BLUETOOTH
  number = dataIn.substring(0,dataIn.indexOf('=')).toInt();
  pos = dataIn.substring(dataIn.indexOf('=')+1,dataIn.length()).toInt();
  #endif
  #ifdef SERVO
  if (number<ServoNumber && pos<=180);
  movement(number,pos);
  #endif
}


/*
calcultes the inverse of the tan,sin and cos in radians
*/
/*
float asin(float c)
{
//calcultes the inverse of the sin
float out;

out= ((c+(c*c*c)/6+(3*c*c*c*c*c)/40+(5*c*c*c*c*c*c*c)/112 +

(35*c*c*c*c*c*c*c*c*c)/1152 +(c*c*c*c*c*c*c*c*c*c*c*0.022) +

(c*c*c*c*c*c*c*c*c*c*c*c*c*.0173)+(c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*.0139) +

(c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*0.0115)+(c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*c*0.01)

));


if(c>=.96 && c<.97){out=1.287+(3.82*(c-.96)); }
if(c>=.97 && c<.98){out=(1.325+4.5*(c-.97)); } // arcsin

if(c>=.98 && c<.99){out=(1.37+6*(c-.98)); }

if(c>=.99 && c<=1){out=(1.43+14*(c-.99)); }

return out;
}

float acos(float c){
  float out;
  out=asin(sqrt(1-c*c));
  return out;
}

float atan(float c)
{
  float out;
  out=asin(c/(sqrt(1+c*c)));
  return out;
}
*/

void positionCalculator(float x, float y,float z)
{
  #ifdef uStepper_roboticArm
  float rot,left,right;

  rot = atan2(y, x);

  x -= cos(rot) * AXOFFSET;
  y -= sin(rot) * AXOFFSET;
  z -= AZOFFSET;

  float L1 = sqrt(
      (x * x) +
      (y * y)); // not offset in the x direction but in the L1 vector direction

  float L2 = sqrt(((L1 - XOFFSET) * (L1 - XOFFSET)) +
                  ((z - ZOFFSET) *
                   (z - ZOFFSET))); // again, it would be nice to make it fit
                                    // with the documentation. e.g. XOFFSET is
                                    // not an offset in the x direction...

  float a = (z - ZOFFSET) / L2;
  float b =
      ((L2 * L2) + (LOWERARMLEN * LOWERARMLEN) - (UPPERARMLEN * UPPERARMLEN)) /
      (2 * L2 * LOWERARMLEN);
  float c =
      ((LOWERARMLEN * LOWERARMLEN) + (UPPERARMLEN * UPPERARMLEN) - (L2 * L2)) /
      (2 * LOWERARMLEN * UPPERARMLEN);

  right = (atan2(a, sqrt(1 - (a * a))) + atan2(sqrt(1 - (b * b)), b));
  left = atan2(sqrt(1 - (c * c)), c);

  rot = (rot * RAD2DEGGEARED) - ROTOFFSET; // This is Theta1

  left -= ELBOWOFFSET;

  right -= SHOULDEROFFSET;

  left = ((left + right) *
           RAD2DEGGEARED); // Remeber to map elbow angle down to motor angle
  right *= RAD2DEGGEARED;

  #ifdef DEBUG
  Serial.print("rot : ");
  Serial.print(round(rot));
  Serial.print(" ; ");
  Serial.print("left : ");
  Serial.print(round(left));
  Serial.print(" ; ");
  Serial.print("right : ");
  Serial.print(round(right));
  Serial.println(" ; ");
  #endif
  #endif

  #ifdef STEPPER
    movement(1,round(rot));
    movement(2,round(-right));//-right
    movement(0,round(left));//left
    
    
  #endif
  #ifdef DOF3
  float theta = asin((y+DELTAY)/sqrt((y+DELTAY)*(y+DELTAY) + (x*x))); //in radians
  theta = (theta*180)/PI;

  float alpha = acos(sqrt((y+DELTAY)*(y+DELTAY) + (x*x))/(2*ARML));
  alpha = ((alpha*180)/PI);

  float beta = 180-(2*alpha);//180-(2*alpha)

  #ifdef SERVO
  movement(0,round(theta));
  movement(2,round(beta-180)); //servos direction offset
  movement(1,round(alpha));
  #endif
  #ifdef DEBUG
  Serial.print("Theta : ");
  Serial.print(round(theta));
  Serial.print(" ; ");
  Serial.print("alpha : ");
  Serial.print(round(alpha));
  Serial.print(" ; ");
  Serial.print("beta : ");
  Serial.print(round(beta));
  Serial.println(" ; ");
  #endif
  #endif

  #ifdef DOF4
  float h = sqrt((y+DELTAY)*(y+DELTAY) + (x*x));

  float theta = asin((y+DELTAY)/h); //in radians
  theta = (theta*180)/PI;

  float alpha = acos(((l2*l2) - (l1*l1) - (h*h) - (l3*l3) ) / (-2*l1*sqrt(h*h + l3*l3))) + atan(l3/h);
  alpha = (alpha*180)/PI;

  float beta = acos(((y+DELTAY)*(y+DELTAY) + (x*x) + (l3*l3) - (l2*l2) - (l1*l1)) / (-2*l1*l2));
  beta = (beta*180)/PI;

  float gamma = (270-alpha-beta);

  #ifdef SERVO
  movement(3,round(gamma));
  movement(0,round(theta+10));
  movement(2,round(180-beta+16));//direction of servos ; here 0 degree is changed
  movement(1,round(alpha+4)); //offset because of servos
  #endif
  #endif
}


void angleToxyz(float rot, float left, float right) {
  // From the documentation the elbow angle theta3 i s the manipulated through
  // the secondary gear The primary gear i s manipulat ing the shoulder angle
  // theta2
  // REMEMBER TO ADD OFFSET TO ACTUATOR!
  float x,y,z;
  rot = (DEG2RADGEARED * rot) + ROTOFFSET;

  right *= DEG2RADGEARED;
  left *= DEG2RADGEARED;

  left = (left - right) + ELBOWOFFSET; // Remember to map secondary gear angle to elbow angle !
  right += SHOULDEROFFSET;

  z = ZOFFSET + sin(right) * LOWERARMLEN -
      cos(left - (1.570796326795f - right)) * UPPERARMLEN +
      AZOFFSET; // o f f s e t in the Z di r e c ton f o r the a c tua tor i s
                // added here

  float k1 = sin(left - (1.570796326795f - right)) * UPPERARMLEN +
             cos(right) * LOWERARMLEN + XOFFSET +
             AXOFFSET; // o f f s e t in the L1 di r e c ton f o r the a c tua
                       // tor i s added here

  x = cos(rot) * k1;
  y = sin(rot) * k1;

  Serial.print("x :");
  Serial.print(x);
  Serial.print(" y :");
  Serial.print(y);
  Serial.print(" z :");
  Serial.println(z);
}
