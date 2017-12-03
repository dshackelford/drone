#include <Servo.h>
#include <IMU.h>
#include <Drone.h>
#include <Control.h>
//#include <math.h>


Drone myDrone(1, 2, 3, 4);
IMU imu;
Control pidL(1, 0, 0, 2000, 500); //kp ki kd max min
Control pidR(1, 0, 0, 2000, 500);

double ref;
double throttle;

int rightLED = 4;
int leftLED = 5;
int MPULED = 6;
bool MPUEngaged = false;
bool motorsEngaged = false;
int motorSwitchPin = 9;

int talkToMotorsPin = 8;
int buttonState = 0;

int BluePin = 10;
int GreenPin = 11;
int RedPin = 12;


void setup()
{
  Serial.begin(115200);
  Serial.println("Arduino Sketch Start!");
  pinMode(talkToMotorsPin, INPUT);

  pinMode(RedPin, OUTPUT);
  pinMode(GreenPin, OUTPUT);
  pinMode(BluePin, OUTPUT);
  //MPUEngaged = true;

  if (imu.initialSetup()) //waits for 15s for start up if successful on dmp initialization.
  {
    attachInterrupt(digitalPinToInterrupt(2), interruptFunc, RISING); //necessary for IMU to communicate.
    imu.getYPR();
    if (isnan(imu.ypr[0]))
    {
      Serial.print("Not a number");

    }

    MPUEngaged = true;
    goWhite();
  }
  else
  {
    //failed to start MPU
  }
}

void loop()
{
  if (MPUEngaged && motorsEngaged)
  {
    goGreen();
  }
  else if (MPUEngaged && !motorsEngaged)
  {
    goWhite();
  }
  else
  {
    goRed;
  }

  if (MPUEngaged)
  {
    imu.getYPR();
    Serial.print(imu.ypr[0]);
    Serial.print("\t");
    Serial.print(imu.ypr[1]);
    Serial.print("\t");
    Serial.print(imu.ypr[2]);
    Serial.print("\n");
  }

  if(digitalRead(motorSwitchPin))
  {
    motorsEngaged = true;
  }
  else
  {
    motorsEngaged = false;
  }
  /*
    if (digitalRead(talkToMotorsPin == HIGH))
    {
      digitalWrite(motorsEngagedLED, HIGH);
    }
    else
    {
      digitalWrite(motorsEngagedLED, LOW);
    }
  */
  //
  //  if(digitalRead(talkToMotorsPin) == HIGH && buttonState == 0)
  //  {
  //    buttonState = 1; //go high
  //    delay(10);
  //  }
  //  else if(digitalRead(talkToMotorsPin) == HIGH && buttonState == 1)
  //  {
  //    buttonState = 0; //go low
  //    delay(10);
  //  }
  //
  //  if(buttonState ==1)
  //  {
  //    digitalWrite(motorsEngagedLED,HIGH);
  //  }

  ref = 0;
  throttle = 1000;
  //
  //  double outL = pidL.getOutput(imu.ypr[2],1000,ref); //current, throttle, ref
  //  double outR = pidR.getOutput(-imu.ypr[2],1000,-ref);
  //  Serial.print("\t");
  //  Serial.print(outL);
  //  Serial.print("\t");
  //  Serial.print(outR);
  //  Serial.print("\n");
}

void interruptFunc()
{
  imu.dmpDataReady();
}



//COLORS - RGB
void goRed()
{
  setColor(255, 0, 0);
}

void goGreen()
{
  setColor(0, 255, 0);
}
void goBlue()
{
  setColor(0, 0, 255);
}

void goWhite()
{
  setColor(255, 255, 255);
}

void goYellow()
{
  setColor(255, 255, 0);
}

void setColor(int red, int green, int blue)
{
  analogWrite(RedPin, red);
  analogWrite(GreenPin, green);
  analogWrite(BluePin, blue);
}

