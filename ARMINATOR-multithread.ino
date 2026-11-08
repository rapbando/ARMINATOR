
/*
                                             "ARMINATOR"

           A 4 axis system designed to control an industrial excavator arm by moving 4 hydraulic levers.

   CNC shield V3 and 4 screw stepper motors, controlled by XBOX360 USB joypad connected via USB host to ARDUINO MEGA ADK.
*/

#include <XBOXUSB.h>
#include <ThreadHandler.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.

#ifdef dobogusinclude
#endif

#define EN        8   // stepper motor enable , active low

//Direction pin
#define X_DIR     5   // X axis stepper motor direction control
#define Y_DIR     6   // y axis stepper motor direction control
#define Z_DIR     7   // z axis stepper motor direction control
#define A_DIR     13  // a axis stepper motor direction control

//Step pin
#define X_STP     2   // x axis stepper control
#define Y_STP     3   // y axis stepper control
#define Z_STP     4   // z axis stepper control
#define A_STP     12  // a axis stepper control

bool L2State ;           // current state of the L2 button
int lastL2State = 0;     // previous state of the L2 button

bool R2State ;           // current state of the R2 button
int lastR2State = 0;     // previous state of the R2 button

int RHY_P_Value ;            // current value of the RHY+ hat
int RHY_P_State ;           // current state of the RHY+ hat
int lastRHY_P_State = 0;    // previous state of the RHY+ hat

int RHY_N_Value ;            // current value of the RHY- hat
int RHY_N_State ;           // current state of the RHY- hat
int lastRHY_N_State = 0;    // previous state of the RHY- hat

int LHY_P_Value ;            // current value of the LHY+ hat
int LHY_P_State ;           // current state of the LHY+ hat
int lastLHY_P_State = 0;    // previous state of the LHY+ hat

int LHY_N_Value ;            // current value of the LHY- hat
int LHY_N_State ;           // current state of the LHY- hat
int lastLHY_N_State = 0;    // previous state of the LHY- hat

int LHX_P_Value ;            // current value of the LHX+ hat
int LHX_P_State ;           // current state of the LHX+ hat
int lastLHX_P_State = 0;    // previous state of the LHX+ hat

int LHX_N_Value ;            // current value of the LHX- hat
int LHX_N_State ;           // current state of the LHX- hat
int lastLHX_N_State = 0;    // previous state of the LHX- hat



//Configures the interrupt timer ticks in us (1000 gives 1ms).
//This tick time will be the lowest time resolution for thread periods and offsets.
//Setting the value to 0 leaves the Interrupt timers defautl tick, this is usefull since
//this setting will not be in conflict with other libraries and analogWrite.
SET_THREAD_HANDLER_TICK(455);

//This macro configures which timer should be used for generating
//the interrupts driving the ThreadHandler.
//The user can implement there own InterruptTimer by inhereting
//from the ThreadHandler::InterruptTimerInterface. This is useful
//for adding support for new boards that this library does not support yet.
//Currently the library supports SAMD21 and all version supported by the
//TimerOne library. To see how to do this look in the "platformSpecificClasses.h".
THREAD_HANDLER(InterruptTimer::getInstance());

USB Usb;
XBOXUSB Xbox(&Usb);

/*
  // CNC shield V3 Function : step . function: to control the direction of the stepper motor , the number of steps .
  //Parameters : dir direction control , dirPin corresponding stepper motor DIR pin ,
  //stepperPin corresponding stepper motor ” step ” pin , Step number of step of no return value.

*/
class StepperMotorHandler
{
public:
  StepperMotorHandler(byte dirPin, byte stepperPin)
  {
    this->dirPin = dirPin;
    this->stepperPin = stepperPin;

    pinMode(this->dirPin, OUTPUT);
    pinMode(this->stepperPin, OUTPUT);

    position = 0;
    targetPos = 0;
    moving = false;

    steppingThread = createThreadWithCodeBlocks(2, 455 * 3, 0, [&]()
        {
          ThreadInterruptBlocker blocker;
          moving = position != targetPos;

          if (moving == false)
          {
            return;
          }
          bool dir = targetPos >= position;
          if (dir)
          {
            position += 1;
            digitalWrite (this->dirPin, true);
          }
          else
          {
            position -= 1;
            digitalWrite (this->dirPin, false);
          }
          Thread::delayNextCodeBlock(455);
        });

    steppingThread->addCodeBlock([&](){
          if (moving == false)
          {
            return;
          }
          digitalWrite (this->stepperPin, HIGH);
          Thread::delayNextCodeBlock(455);
        });

    steppingThread->addCodeBlock([&](){
          if (moving == false)
          {
            return;
          }
          digitalWrite (this->stepperPin, LOW);
        });
  }

  void fwd(int count)
  {
    ThreadInterruptBlocker blocker;
    targetPos += count;
  }

  void back(int count)
  {
    ThreadInterruptBlocker blocker;
    targetPos -= count;
  }

  void setPosition(int pos)
  {
    ThreadInterruptBlocker blocker;
    targetPos = pos;
  }

  bool isMoving()
  {
    return moving;
  }

  int getPosition()
  {
    ThreadInterruptBlocker blocker;
    return position;
  }

private:
  int position;
  int targetPos;
  bool moving;
  byte dirPin;
  byte stepperPin;
  CodeBlocksThread* steppingThread;
};

StepperMotorHandler* xAxis;
StepperMotorHandler* yAxis;
StepperMotorHandler* zAxis;
StepperMotorHandler* aAxis;

void setup()
{
  xAxis = new StepperMotorHandler(X_DIR, X_STP);
  yAxis = new StepperMotorHandler(Y_DIR, Y_STP);
  zAxis = new StepperMotorHandler(Z_DIR, Z_STP);
  aAxis = new StepperMotorHandler(A_DIR, A_STP);

  // Pins setup
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));

   //start executing threads
  ThreadHandler::getInstance()->enableThreadExecution();
}

void loop() {

  Usb.Task();
  if (Xbox.Xbox360Connected)


    // disable operation by clicking the XBOX main button
    if (Xbox.getButtonClick(XBOX)) {
      Xbox.setLedMode(SLOWBLINK);
      digitalWrite(EN, HIGH);
      Serial.println(F("Movement disabled"));
    }

  // enable operation by clicking the START button
  if  (Xbox.getButtonClick(START)) {
    Xbox.setLedMode(ROTATING);
    digitalWrite(EN, LOW);
    Serial.println(F("Movement enabled"));
  }

  // read the analog hat input
  LHX_P_Value = (Xbox.getAnalogHat(LeftHatX)); // read the analog hat input value from serial (range is -32000 to 32000)

  if (LHX_P_Value > 10000) {
    LHX_P_State = true;
  }
  else  {
    LHX_P_State = false;
  }

  // compare the buttonState to its previous state
  if (LHX_P_State != lastLHX_P_State) {

    // if the current state is TRUE then the button went from off to on:
    if (LHX_P_State == true)  {

      xAxis->fwd(850); // set N of steps to move forward,  200 steps are a shaft turn.
      Serial.print("\r\nStepping X FWD");
    }
    // on hat release go back to reset the original position
    else {

      xAxis->back(850); // set N of steps to move backward,  200 steps are a shaft turn.
      Serial.print("\r\nStepping X BACK");
    }
  }
  lastLHX_P_State = LHX_P_State;

  // read the analog hat input
  LHX_N_Value = (Xbox.getAnalogHat(LeftHatX)); // read the analog hat input value from serial (range is -32000 to 32000)

  if (LHX_N_Value < -10000) {
    LHX_N_State = true;
  }
  else  {
    LHX_N_State = false;
  }

  // compare the buttonState to its previous state
  if (LHX_N_State != lastLHX_N_State) {

    // if the current state is TRUE then the button went from off to on:
    if (LHX_N_State == true)  {

      xAxis->back(850); // set N of steps to move backward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping X BACK");
    }
    // on hat release go back to reset the original position
    else {

      xAxis->fwd(850); // set N of steps to move forward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping X FWD");
    }
  }
  lastLHX_N_State = LHX_N_State;


  // read the pushbutton input from serial
  L2State = (Xbox.getButtonPress(L2));

  // compare the buttonState to its previous state
  if (L2State != lastL2State) {

    // if the current state is TRUE then the button went from off to on:
    if (L2State  == true)  {

      aAxis->fwd(950); // set N of steps to move forward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping A FWD");
    }
    // on button release go back to reset the original position

    else  {

      aAxis->back(950); // set N of steps to move backward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping A BACK");
    }
  }
  // save the current state as the last state, for next time through the loop
  lastL2State = L2State;

  // read the pushbutton input from serial
  R2State = (Xbox.getButtonPress(R2));

  // compare the buttonState to its previous state
  if (R2State != lastR2State) {

    // if the current state is TRUE then the button went from off to on:
    if (R2State == true)  {

      aAxis->back(950); // set N of steps to move backward, 200 steps are is a shaft turn.
      Serial.print("\r\nStepping A BACK");
    }
    // on button release go back to reset the original position
    else  {

      aAxis->fwd(950); // set N of steps to move forward, 200 steps are is a shaft turn.
      Serial.print("\r\nStepping A FWD");
    }
  }
  // save the current state as the last state, for next time through the loop
  lastR2State = R2State;


  // read the analog hat input
  RHY_P_Value = (Xbox.getAnalogHat(RightHatY));

  if (RHY_P_Value > 10000) {
    RHY_P_State = true;
  }
  else  {
    RHY_P_State = false;
  }

  // compare the buttonState to its previous state
  if (RHY_P_State != lastRHY_P_State) {

    // if the current state is TRUE then the button went from off to on:
    if (RHY_P_State == true)  {

      yAxis->back(950); // set N of steps to move backward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping Y BACK");
    }
    // on hat release go back to reset the original position
    else {

      yAxis->fwd(950); // set N of steps to move forward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping Y FWD");
    }
  }
  lastRHY_P_State = RHY_P_State;

  RHY_N_Value = (Xbox.getAnalogHat(RightHatY)); // read the analog hat input value from serial (range is -32000 to 32000)

  if (RHY_N_Value < -10000) {
    RHY_N_State = true;
  }
  else  {
    RHY_N_State = false;
  }

  // compare the buttonState to its previous state
  if (RHY_N_State != lastRHY_N_State) {

    // if the current state is TRUE then the button went from off to on:
    if (RHY_N_State == true)  {

      yAxis->fwd(950); // set N of steps to move forward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping Y FWD");
    }
    // on hat release go back to reset the original position
    else {

      yAxis->back(950); // set N of steps to move backward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping Y BACK");
    }
  }
  lastRHY_N_State = RHY_N_State;


  // read the analog hat input
  LHY_P_Value = (Xbox.getAnalogHat(LeftHatY)); // read the analog hat input value from serial (range is -32000 to 32000)

  if (LHY_P_Value > 10000) {
    LHY_P_State = true;
  }
  else  {
    LHY_P_State = false;
  }

  // compare the buttonState to its previous state
  if (LHY_P_State != lastLHY_P_State) {

    // if the current state is TRUE then the button went from off to on:
    if (LHY_P_State == true)  {

      zAxis->fwd(750); // set N of steps to move forward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping Z FWD");
    }
    // on hat release go back to reset the original position
    else {

      zAxis->back(750); // set N of steps to move backward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping Z BACK");
    }
  }
  lastLHY_P_State = LHY_P_State;

  // read the analog hat input
  LHY_N_Value = (Xbox.getAnalogHat(LeftHatY)); // read the analog hat input value from serial (range is -32000 to 32000)

  if (LHY_N_Value < -10000) {
    LHY_N_State = true;
  }
  else  {
    LHY_N_State = false;
  }

  // compare the buttonState to its previous state
  if (LHY_N_State != lastLHY_N_State) {

    // if the current state is TRUE then the button went from off to on:
    if (LHY_N_State == true)  {

      zAxis->back(1200); // set N of steps to move backward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping Z BACK");
    }
    // on hat release go back to reset the original position
    else {

      zAxis->fwd(1200); // set N of steps to move forward, 200 steps are a shaft turn.
      Serial.print("\r\nStepping Z FWD");
    }
  }
  lastLHY_N_State = LHY_N_State;
}
