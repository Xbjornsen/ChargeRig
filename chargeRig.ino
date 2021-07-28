#include <AFMotor.h>
#include <Servo.h>
#include <StopWatch.h>

#define XLimit 13
#define ZLimit 10
#define YLimit 2

#define DLimit A2
#define ChargeContact A1
#define VOLT_PIN_BAT A0

Servo myServo;
StopWatch clock;
StopWatch ProgramRuntime;
StopWatch BatteryChargeTime;
StopWatch BatteryDischargeTime;

AF_DCMotor XMotor(1); //declare which motors are attached to which outlets on the motor shield
AF_DCMotor YMotor(2);
AF_DCMotor ZMotor(3);

float healthyContact = 0;      // number of times a healthy contact was made. based on circuit being closed.
float unhealthyContact = 0;    // number of times an unhealthy contact was made.
float numberOfDrops = 0;       // number of times the drone was lowered.
float ContactEfficiency = 0.0; // Contact efficiency
float batteryChargeTimeElapsed = 0.0;
float batteryDischargeTimeElapsed = 0.0;
int numberOfCycles = 0; // number of cycles for the rig

bool discharging; // boolean for charging action
bool discharged;

int Xtime; //set time of movement
int Ytime;
int Ztime;
int Dtime;
int SafeZ;

// Arrays
int Xcoordinates[30];
int Ycoordinates[30];

int UnhealthyContactCoorsX[30];
int UnhealthyContactCoorsY[30];

//Battery
const float voltageMaximum = 4.10;
const float voltageMinimum = 3.6;

// global
int numberOfProgramCycles; // number of program cylces

// booleans
bool XAxisInit = false;
bool ZAxisInit = false;
bool YAxisInit = false;
bool rigInit = false;
bool rigCycle = false;

//consts
const int motorSpeed = 255;    // DC motor speed for Z axis
const int motorSpeedRig = 200; // Dc motor speed for X axis
const int servoMotorSpeed = 55;

void setup()
{
  pinMode(XLimit, INPUT);       //set pin 2 as limit switch input Z Limit
  pinMode(ZLimit, INPUT);       //set pin 3 as limit switch input X Right Limit
  pinMode(YLimit, INPUT);       //set pin 13 as limit switch input Y Back Limit
  pinMode(DLimit, INPUT);       // set A3 as limit switch, pull down resistor external
  pinMode(VOLT_PIN_BAT, INPUT); // set pin A0 as input - voltage across charge indicator LED

  Serial.begin(115200);
  Serial.println("Program to Initialise, Cycle");
  Serial.println("Begin Initalisation");

  // set motor speed for the axis motors
  ZMotor.setSpeed(motorSpeed);
  XMotor.setSpeed(motorSpeed);
  YMotor.setSpeed(motorSpeed);

  //servo
  myServo.attach(9);
  myServo.write(0);
}

void loop()
{
  ProgramRuntime.start();
  // Check if rig is initialised
  if (rigInit == false)
  {
    initialiseRig();
    Serial.print("Rig has initialised to: ");
    Serial.println(rigInit);
    rigCycle = true;
  }
  RigCoordinates(); 
  // Cycle while rig cycle is true
  while (numberOfProgramCycles < 5)
  {
    CycleRig(Xcoordinates[numberOfCycles], Ycoordinates[numberOfCycles]);
    numberOfProgramCycles++;
    if (numberOfProgramCycles == 3)
    {
      BatteryStatus();
    }
    Serial.print("Program cycle: ");
    Serial.println(numberOfProgramCycles);
  }
  rigCycle = false;

  ProgramRuntime.elapsed();
  Serial.print("Program run time in mili seconds: ");
  Serial.println(ProgramRuntime.elapsed());
  // program ends
  if ((rigInit == true) && (rigCycle == false))
  {
    Serial.println("###################################");
    Serial.print("Rig cylce was completed in seconds: ");
    Serial.println(ProgramRuntime.elapsed() / 1000);
    Serial.print("In minutes: ");
    Serial.println(ProgramRuntime.elapsed() / 60000);
    Serial.print("Number of successful contacts made: ");
    Serial.println(healthyContact);
    Serial.print("Number of Cycles where X and Y was changed: ");
    Serial.println(numberOfCycles);
    Serial.print("Number of drops on the rail: ");
    Serial.println(numberOfDrops);
    ProgramRuntime.stop();
    while (1 == 1)
    {
    };
  }
}

// initialise array with coordinates
void RigCoordinates()
{
  for (int i = 0; i < 30; i++)
  {
    Xcoordinates[i] = random(250, 500);
    Ycoordinates[i] = random(250, 500);
  }
}

/*#################### Initialize Rig Section ######################*/
void initialiseRig()
{
  if ((ZAxisInit == true) && (XAxisInit == true) && (YAxisInit == true))
  {
    rigInit = true;
    Serial.println("Rig has initialised to true");
  }
  if (ZAxisInit == false)
  {
    Serial.print("X axis has initialised to: ");
    Serial.println(ZAxisInit);
    initialiseZAxis();
    ZAxisInit = true;
    Serial.print("Z axis is now: ");
    Serial.println(ZAxisInit);
  }
  if (XAxisInit == false)
  {
    Serial.print("X axis has initialised to: ");
    Serial.println(XAxisInit);
    initialiseXAxis();
    Serial.print("Z axis is now: ");
    Serial.println(XAxisInit);
  }
  if (YAxisInit == false)
  {
    Serial.print("Y axis has initialised to: ");
    Serial.println(YAxisInit);
    initialiseYAxis();
    Serial.print("Y axis is now: ");
    Serial.println(YAxisInit);
  }
}

// initializes Z axis using Dlimit and Zlimit switches
void initialiseZAxis()
{
  int ZUpperLimit = digitalRead(ZLimit);
  while (digitalRead(ZLimit) == HIGH)
  {
    ZUpperLimit = digitalRead(ZLimit);
    Serial.println("Raising Drone");
    ZMotor.run(BACKWARD);
    Serial.print(" Z axis: ");
    Serial.println(ZUpperLimit);
  }
  Serial.println("Drone reached height limit");
  ZMotor.run(RELEASE);
  // lower drone onto the charging rail
  int DLowerLimit = digitalRead(DLimit);
  while (digitalRead(DLimit) == HIGH)
  {
    DLowerLimit = digitalRead(DLimit);
    clock.start();
    ZMotor.run(FORWARD);
    Serial.print("Reached lower limit: ");
    Serial.println(DLowerLimit);
  }
  clock.stop();
  Ztime = clock.elapsed();
  SafeZ = Ztime / 2;
  ZMotor.run(RELEASE);
  // raise up 1/4 way
  ZMotor.run(BACKWARD);
  delay(SafeZ);
  ZMotor.run(RELEASE);
  Serial.println(" Z axis should be now just off the rail");
}

// initializes X axis using Zlimit switch
void initialiseXAxis()
{
  int XRightLimit = digitalRead(XLimit);
  if (XAxisInit == false)
  {
    while (digitalRead(XLimit) == HIGH)
    {
      XRightLimit = digitalRead(XLimit);
      Serial.println("Initialising X axis: ");
      XMotor.run(FORWARD);
      Serial.print(" X axis: ");
      Serial.println(XRightLimit);
    }
    XMotor.run(RELEASE);
    Serial.println("At limit switch");
    XMotor.run(BACKWARD);
    Serial.println("X axis to centre");
    delay(2000);
    XMotor.run(RELEASE);
    XAxisInit = true;
    Serial.println("X Initialisation Complete");
  }
}

void initialiseYAxis()
{
  int YBackLimit = digitalRead(YLimit);
  if (YAxisInit == false)
  {
    while (digitalRead(YLimit) != LOW)
    {
      YBackLimit = digitalRead(YLimit);
      Serial.println("Setting y limit");
      YMotor.run(FORWARD);
      Serial.print("Y axis: ");
      Serial.println(YBackLimit);
    }
    Serial.println("At Y limit switch");
    YMotor.run(RELEASE);
    Serial.println("Y axis to centre");
    YMotor.run(BACKWARD);
    Serial.println("Centering");
    delay(2000);
    YMotor.run(RELEASE);
    Serial.println("Y Initialisation Complete");
    YAxisInit = true;
  }
}

  /*#################### Rig Cylce Section ######################*/
  void CycleRig(int xTime, int yTime)
  {
    Serial.println("Cycling rig for random amounts of time for x and y axis");

    for (int XandYCycle = 0; XandYCycle < 5; XandYCycle++)
    {
      // run y axis for random amount of time
      YMotor.setSpeed(motorSpeedRig);
      YMotor.run(FORWARD);
      delay(yTime);
      YMotor.run(RELEASE);
      Serial.print("y axis moved by: ");
      Serial.println(yTime);

      // run x axis for random amount of time
      XMotor.setSpeed(motorSpeedRig);
      XMotor.run(FORWARD);
      delay(xTime);
      XMotor.run(RELEASE);
      Serial.print("X axis moved by: ");
      Serial.println(xTime);

      DroneDropCycle();
      // return X and Y to Zero
      ReturnToZero(xTime, yTime);

      Serial.print("Number of X&Y axis move cycles ");
      Serial.println(XandYCycle);
      numberOfCycles++;
    }
  }

  // cycle for the drone drops. This is called once after every move
  // of X and Y
  void DroneDropCycle()
  {
    for (int drops = 0; drops < 10; drops++)
    {
      DroneDrop();
      numberOfDrops++; // count global amount of drops
      DroneRaise();
    }
  }

  // lower drone onto the charging rail
  void DroneDrop()
  {
    float rasiedVoltage = voltageCheck(VOLT_PIN_BAT);
    int DLowerLimit = digitalRead(DLimit);
    while (digitalRead(DLimit) == HIGH)
    {
      ZMotor.run(FORWARD);
    }
    ZMotor.run(RELEASE);
    CycleCount(rasiedVoltage);
  }

  // raise Drone from the rail.
  void DroneRaise()
  {
    ZMotor.run(BACKWARD);
    delay(SafeZ);
    ZMotor.run(RELEASE);
  }

  // Counts the number of cylces where there was good contact.
  void CycleCount(float raisedVoltage)
  {
    float ContactVoltage = voltageCheck(VOLT_PIN_BAT);
    delay(1000);
    Serial.print("Rasied voltage: ");
    Serial.println(raisedVoltage);
    Serial.print("Contact volatge: ");
    Serial.println(ContactVoltage);
    if (ContactVoltage > raisedVoltage)
    {
      healthyContact++;
    }
    else if (ContactVoltage == raisedVoltage)
    {
      unhealthyContact++;
      UnhealthyContactCoorsX[numberOfCycles] = Xcoordinates[numberOfCycles];
      UnhealthyContactCoorsY[numberOfCycles] = Ycoordinates[numberOfCycles];
    }
    Serial.print("Number of drops: ");
    Serial.println(numberOfDrops);
    Serial.print("Cycle Count: ");
    Serial.println(numberOfCycles);
    Serial.print("Healthy contact Count: ");
    Serial.println(healthyContact);
    Serial.print("Unhealthy contact count");
    Serial.println(unhealthyContact);
    ContactEfficiency = ((healthyContact / numberOfDrops) * 100);
    Serial.print("Contact Efficiency %: ");
    Serial.println(ContactEfficiency);
  }

  // returns the rig back to a starting point for the next cylce.
  void ReturnToZero(int xtime, int ytime)
  {
    XMotor.run(BACKWARD); //Return everything to where it started
    delay(xtime);
    XMotor.run(RELEASE);
    YMotor.run(BACKWARD);
    delay(ytime);
    YMotor.run(RELEASE);
    Serial.println("Everything returned to zero");
  }

  /*#################### Voltage Section ######################*/

  // calculates the voltage average from analogue pin
  float voltageCheck(int valueIn)
  {
    int value;
    float volt;
    float voltAverage = 0.0;
    value = analogRead(valueIn);
    volt = value * 5.0 / 1023.0;

    // looping 10 times
    for (int i = 0; i < 10; i++)
    {
      voltAverage = voltAverage + volt;
    }
    voltAverage = voltAverage / 10; // calculates the agerage voltage
    float voltageAdjustment = 0.12;
    voltAverage = voltAverage - voltageAdjustment;
    return voltAverage;
  }

  // function to determine if the battery should be discharged or charged
  void BatteryStatus()
  {
    float batVoltage = voltageCheck(VOLT_PIN_BAT);
    Serial.print("Battery voltage at: ");
    Serial.println(batVoltage);
    if ((batVoltage >= voltageMaximum) && (discharged == false))
    {
      Serial.print("Battery voltage equals voltage maximum, discharge sequence initiated: ");
      Serial.println(batVoltage);
      dischargeInitialization();
      discharged == true;
    }

    if (discharged == true)
    {
      BatteryChargeTime.start();
      DroneDrop();
      float checkCharge = voltageCheck(VOLT_PIN_BAT);
      while (checkCharge < voltageMaximum)
      {
        float checkCharge = voltageCheck(VOLT_PIN_BAT);
        Serial.print("Charging battery, voltage reading: ");
        Serial.println(checkCharge);
      }
      BatteryChargeTime.stop();
      batteryChargeTimeElapsed = BatteryChargeTime.MINUTES;
      Serial.print("Time taken to charge the battery: ");
      Serial.println(batteryChargeTimeElapsed);
      DroneRaise();
      Serial.println("Battery charged will continue with rig cycle ");
      discharged = false;
    }
  }

  /*#################### Discharge Section ######################*/

  // Discharge init and run. Will stop discharge bank when the voltage has reached cutoff
  void dischargeInitialization()
  {
    DischargeBankInit();
    DischargeBankControl();
    DischargeBankShutdown();
  }

  // Initialise motor function
  void DischargeBankInit()
  {
    Serial.println("Initialising discharge motor");
    myServo.write(10); // don't change this
    delay(5000);       // don't change this. Mandatory wait time
  }

  // Function to discharge the battery
  void DischargeBankControl()
  {
    float voltAverage = voltageCheck(VOLT_PIN_BAT);
    do
    {
      Serial.println("Discharging");
      myServo.write(servoMotorSpeed);
      float voltAverage = voltageCheck(VOLT_PIN_BAT);
    } while (voltAverage > voltageMinimum);
  }

  // shut down discharge bank
  void DischargeBankShutdown()
  {
    myServo.write(0);
    Serial.println("Shut down load bank, discharged set to true");
  }
