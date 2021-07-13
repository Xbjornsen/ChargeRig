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

AF_DCMotor XMotor(1); //declare which motors are attached to which outlets on the motor shield
AF_DCMotor YMotor(2);
AF_DCMotor ZMotor(3);

float numberOfContacts = 0; // number of times a contact was made. based on circuit being closed.
float numberOfDrops = 0;    // number of times the drone was lowered.
float ContactEfficiency = 0.0;
int numberOfCycles = 0;

const int val = 50; // sets the motor speed. don't change for now.

bool discharging; // boolean for charging action
bool discharged;

int Xtime; //set time of movement
int Ytime;
int Ztime;
int Dtime;
int SafeZ; 

// global
int numberOfProgramCycles;

// booleans
bool XAxisInit = false;
bool ZAxisInit = false;
bool YAxisInit = false;
bool rigInit = false;
bool rigCycle = false;

//consts
const int motorSpeed = 255;

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
}

void loop()
{
  ProgramRuntime.start();
  // check if rig is initialised
  if (rigInit == false)
  {
    initialiseRig();
    Serial.print("Rig has initialised to: ");
    Serial.println(rigInit);
    rigCycle = true;
  }


  // cylce while rig cycle is true
  while (numberOfProgramCycles <= 2)
  {
    CycleRig();
    numberOfProgramCycles++;
    initialiseRig();
    int batvoltage = voltageCheck(VOLT_PIN_BAT);
    if(batvoltage >= 4.20){
      Serial.print("Battery voltage is  greater than 4.20 discharge sequence initiated: ");
      Serial.println(batvoltage);
      dischargeInitialization();
    }
  }
  rigCycle = false; 
  
  ProgramRuntime.elapsed();
  Serial.print("Program run time in mili seconds: ");
  Serial.println(ProgramRuntime.elapsed());
  // program ends
  if((rigInit == true) && (rigCycle == false)){
    Serial.println("###################################");
    Serial.print("Rig cylce was completed in seconds: ");
    Serial.println(ProgramRuntime.elapsed()/1000);
    Serial.print("In minutes: ");
    Serial.println(ProgramRuntime.elapsed()/60000);
    Serial.print("Number of success contacts made: ");
    Serial.println(numberOfContacts);
    Serial.print("Number of Cylces where X and Y was changed: ");
    Serial.println(numberOfCycles);
    Serial.print("Number of crops on the rail: ");
    Serial.println(numberOfDrops);
    ProgramRuntime.stop();
    while (1 ==1 ) {};
  }
}

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
  Serial.println(" z axis should be now just off the rail");
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

void CycleRig()
{
  Serial.println("Cycling rig for random amounts of time for x and y axis");
  for (numberOfCycles; numberOfCycles < 5; numberOfCycles++)
  {
    DroneDropCycle();
    // drop drone 
    DroneDrop();
    // return X and Y to Zero
    returnToZero(Xtime, Ytime);
    // run y axis for random amount of time
    YMotor.run(FORWARD);
    Ytime = random(250, 500);
    delay(Ytime);
    YMotor.run(RELEASE);
    Serial.print("y axis moved by: ");
    Serial.println(Ytime);

    // run x axis for random amount of time
    XMotor.run(FORWARD);
    Xtime = random(250, 500);
    delay(Xtime);
    XMotor.run(RELEASE);
    Serial.print("X axis moved by: ");
    Serial.println(Xtime);

    Serial.print("Number of X&Y axis move cycles");
    Serial.println(numberOfCycles);
    DroneRaise();
  }
  Serial.println(voltageCheck(VOLT_PIN_BAT));
}

// cycle for the drone drops. This is called once after every move
// of X and Y 
void DroneDropCycle()
{
  int drops;
  //drop drone 10 times then move x and y
  for (drops = 0; drops < 10; drops++)
  {
    DroneDrop();
    numberOfDrops++; // count global amount of drops
    DroneRaise();
  }
}

// lower drone onto the charging rail
void DroneDrop()
{
  int DLowerLimit = digitalRead(DLimit);
  while (digitalRead(DLimit) == HIGH)
  {
    ZMotor.run(FORWARD);
  }
  ZMotor.run(RELEASE);
  CycleCount();
}

// raise Drone from the rail. 
void DroneRaise()
{
  ZMotor.run(BACKWARD);
  delay(SafeZ);
  ZMotor.run(RELEASE);
}

// calculates the voltage average from analogue pin
float voltageCheck(int valueIn)
{
  int value;
  float volt;
  float voltAverage = 0.0;
  value = analogRead( valueIn ); 
  volt = value * 5.0 / 1023.0;

  for (int i = 0; i < 10; i++)
  {
    voltAverage = voltAverage + volt; // concatinates
  }
  voltAverage = voltAverage / 10; // calculates the agerage voltage
  float voltageAdjustment = 0.10;
  voltAverage = voltAverage - voltageAdjustment;
  Serial.print("Volt average in check voltage function after adjustment: ");
  Serial.println(voltAverage);
  return voltAverage;
}

// Counts the number of cylces where there was good contact.
void CycleCount()
{
  int ContactClosed = digitalRead(ChargeContact);
  if (digitalRead(ChargeContact) == HIGH)
  {
    numberOfContacts++;
  }
  Serial.print("Cycle Count: ");
  Serial.println(numberOfCycles);
  Serial.print("Contact Count: ");
  Serial.println(numberOfContacts);
  ContactEfficiency = ((numberOfContacts / numberOfDrops) * 100);
  Serial.print("Contact Efficiency %: ");
  Serial.println(ContactEfficiency);
}

// returns the rig back to a starting point for the next cylce.
void returnToZero(int xtime, int ytime)
{
  XMotor.run(BACKWARD); //Return everything to where it started
  delay(xtime);
  XMotor.run(RELEASE);

  YMotor.run(BACKWARD);
  delay(ytime);
  YMotor.run(RELEASE);
  Serial.println("Everything returned to zero");
}


// Discharge init and run. will stop motor when the voltage has reached cutoff 
void dischargeInitialization()
{
    motorInit();
    motorBeginDischarge();
    motorShutdown();
}

void BatteryCheck()
{
    int voltAverage = voltageCheck(VOLT_PIN_BAT);
    Serial.println(voltAverage);
    // for loop to check if the average voltage is above the cut off voltage
    if (voltAverage > 3.98)
    {                       
        discharging = true; // set discharging to true
        Serial.print("discharging set to true: ");
        Serial.println(discharging);
        delay(500);
    }
    else
    {
        discharging = false; //sets discharging to false
        Serial.print("discharging set to false: ");
        Serial.println(discharging);
        delay(500);
    }
}

// initialise motor function
void motorInit()
{
    myServo.write(10); // don't change this
    delay(5000);       // don't change this. Mandatory wait time
}

// begin motor and set speed.
void motorBeginDischarge()
{
    int motorSpeed = val;
    BatteryCheck();
    delay(500);
    while (discharging == true)
    {
        Serial.println("Discharging");
        myServo.write(motorSpeed);
        BatteryCheck();
    }
    Serial.print("Discharging: ");
    Serial.print(discharging);
    Serial.println(" Battery voltage cuttoff reached, shutting down load bank");
}

// shut down motor
void motorShutdown()
{
    myServo.write(0);
    discharged = true; // sets discharged bool to true
    delay(5000);
}
