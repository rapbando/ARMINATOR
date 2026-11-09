/*
                                             "ARMINATOR"

           A 4 axis system designed to control an industrial excavator arm by moving 4 hydraulic levers.

   CNC shield V3 and 4 screw stepper motors, controlled by XBOX360 USB joypad connected via USB host to ARDUINO MEGA ADK.

   Safety behaviour:
   - START enables movement, but only when all sticks and triggers are released.
   - XBOX button returns all levers to neutral, then turns the motors off.
   - If the controller disconnects, all levers return to neutral, then the motors turn off.
   - Lever positions follow the controller inputs directly, so the step count can't drift.

   Control is proportional: each lever is pushed in proportion to how far its stick or
   trigger is moved, up to the maximum travel set below.
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

// Proportional control: lever travel scales with how far a stick or trigger is pushed.
// Analog hat range is about -32768 to 32767, triggers are 0 to 255.
#define STICK_DEADZONE    8000   // stick values inside +/- this are neutral
#define STICK_FULL        31000  // stick values beyond +/- this give full lever travel
#define TRIGGER_DEADZONE  30     // trigger values below this are neutral
#define TRIGGER_FULL      245    // trigger values above this give full lever travel
#define TARGET_HYSTERESIS 8      // ignore target changes smaller than this (steps) to filter stick noise
#define STATUS_PRINT_MS   200    // how often to print lever targets while they change

// Maximum lever travel in steps from neutral (200 steps are a shaft turn).
#define X_STEPS       850   // left stick X:  right = +, left = -
#define A_STEPS       950   // L2 = +, R2 = -
#define Y_STEPS       950   // right stick Y: down = +, up = -
#define Z_STEPS_UP    750   // left stick Y up   = +
#define Z_STEPS_DOWN  1200  // left stick Y down = -

enum ArmState
{
  DISABLED,   // motors off, levers at neutral, inputs ignored
  ENABLED,    // motors on, levers follow the controller
  PARKING     // motors on, levers returning to neutral, inputs ignored
};

ArmState armState = DISABLED;

// Forward declaration so the Arduino IDE's auto-generated function prototypes compile.
class StepperMotorHandler;



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

  void setPosition(int pos)
  {
    ThreadInterruptBlocker blocker;
    targetPos = pos;
  }

  int getTarget()
  {
    ThreadInterruptBlocker blocker;
    return targetPos;
  }

  // True until the motor has reached its target and the last step pulse has finished.
  bool isMoving()
  {
    ThreadInterruptBlocker blocker;
    return moving || position != targetPos;
  }

  int getPosition()
  {
    ThreadInterruptBlocker blocker;
    return position;
  }

private:
  volatile int position;
  volatile int targetPos;
  volatile bool moving;
  byte dirPin;
  byte stepperPin;
  CodeBlocksThread* steppingThread;
};

StepperMotorHandler* xAxis;
StepperMotorHandler* yAxis;
StepperMotorHandler* zAxis;
StepperMotorHandler* aAxis;

// Scales an input magnitude to 0..maxSteps: 0 up to the deadzone, maxSteps at or
// beyond full, linear in between.
int scaleToSteps(long magnitude, long deadzone, long full, int maxSteps)
{
  if (magnitude <= deadzone) return 0;
  if (magnitude >= full) return maxSteps;
  return (int)((magnitude - deadzone) * maxSteps / (full - deadzone));
}

// Maps an analog hat value to a lever position: proportional up to posSteps
// when pushed positive, up to negSteps when pushed negative, 0 in the deadzone.
// posSteps/negSteps carry the sign, so an axis can be inverted.
int hatToSteps(int value, int posSteps, int negSteps)
{
  long v = value;
  if (v > 0) return (int)((long)scaleToSteps(v, STICK_DEADZONE, STICK_FULL, abs(posSteps)) * (posSteps < 0 ? -1 : 1));
  if (v < 0) return (int)((long)scaleToSteps(-v, STICK_DEADZONE, STICK_FULL, abs(negSteps)) * (negSteps < 0 ? -1 : 1));
  return 0;
}

int triggerToSteps(uint8_t value, int maxSteps)
{
  return scaleToSteps(value, TRIGGER_DEADZONE, TRIGGER_FULL, maxSteps);
}

int xInput() { return hatToSteps(Xbox.getAnalogHat(LeftHatX), X_STEPS, -X_STEPS); }
int yInput() { return hatToSteps(Xbox.getAnalogHat(RightHatY), -Y_STEPS, Y_STEPS); }
int zInput() { return hatToSteps(Xbox.getAnalogHat(LeftHatY), Z_STEPS_UP, -Z_STEPS_DOWN); }
int aInput()
{
  // L2 pushes A forward, R2 pushes it back; pressed together they cancel out.
  return triggerToSteps(Xbox.getButtonPress(L2), A_STEPS) -
         triggerToSteps(Xbox.getButtonPress(R2), A_STEPS);
}

bool targetsChanged = false;        // a lever target changed since the last status print
unsigned long lastStatusPrint = 0;

bool inputsNeutral()
{
  return xInput() == 0 && yInput() == 0 && zInput() == 0 && aInput() == 0;
}

// Sets a lever's target. Small changes are ignored to filter stick noise, but
// neutral (0) and full travel are always applied exactly.
void moveAxis(StepperMotorHandler* axis, int target, int maxPos, int maxNeg)
{
  int current = axis->getTarget();
  if (target == current) return;
  if (target == 0 || target == maxPos || target == maxNeg ||
      abs(target - current) >= TARGET_HYSTERESIS)
  {
    axis->setPosition(target);
    targetsChanged = true;
  }
}

void moveAllAxes(int x, int y, int z, int a)
{
  moveAxis(xAxis, x, X_STEPS, -X_STEPS);
  moveAxis(yAxis, y, Y_STEPS, -Y_STEPS);
  moveAxis(zAxis, z, Z_STEPS_UP, -Z_STEPS_DOWN);
  moveAxis(aAxis, a, A_STEPS, -A_STEPS);
}

// Prints all lever targets at most every STATUS_PRINT_MS, and only if one changed,
// so proportional control doesn't flood the serial port.
void printStatus()
{
  if (!targetsChanged || millis() - lastStatusPrint < STATUS_PRINT_MS) return;
  lastStatusPrint = millis();
  targetsChanged = false;
  Serial.print(F("\r\nTargets X "));
  Serial.print(xAxis->getTarget());
  Serial.print(F(" Y "));
  Serial.print(yAxis->getTarget());
  Serial.print(F(" Z "));
  Serial.print(zAxis->getTarget());
  Serial.print(F(" A "));
  Serial.print(aAxis->getTarget());
}

bool allAxesAtNeutral()
{
  return !xAxis->isMoving() && xAxis->getPosition() == 0 &&
         !yAxis->isMoving() && yAxis->getPosition() == 0 &&
         !zAxis->isMoving() && zAxis->getPosition() == 0 &&
         !aAxis->isMoving() && aAxis->getPosition() == 0;
}

// Return all levers to neutral. The motors stay on until they get there,
// then loop() turns them off.
void startParking()
{
  moveAllAxes(0, 0, 0, 0);
  armState = PARKING;
}

void setup()
{
  // Turn the motors off before anything else.
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  xAxis = new StepperMotorHandler(X_DIR, X_STP);
  yAxis = new StepperMotorHandler(Y_DIR, Y_STP);
  zAxis = new StepperMotorHandler(Z_DIR, Z_STP);
  aAxis = new StepperMotorHandler(A_DIR, A_STP);

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
  bool connected = Xbox.Xbox360Connected;

  if (!connected)
  {
    // Failsafe: controller lost while enabled.
    if (armState == ENABLED)
    {
      Serial.print(F("\r\nController disconnected - returning levers to neutral"));
      startParking();
    }
  }
  else
  {
    // disable operation by clicking the XBOX main button
    if (Xbox.getButtonClick(XBOX) && armState == ENABLED)
    {
      Serial.print(F("\r\nMovement disabled - returning levers to neutral"));
      startParking();
    }

    // enable operation by clicking the START button
    if (Xbox.getButtonClick(START) && armState == DISABLED)
    {
      if (inputsNeutral())
      {
        digitalWrite(EN, LOW);
        Xbox.setLedMode(ROTATING);
        armState = ENABLED;
        Serial.print(F("\r\nMovement enabled"));
      }
      else
      {
        Serial.print(F("\r\nRelease all sticks and triggers before pressing START"));
      }
    }
  }

  if (armState == ENABLED)
  {
    // Each lever's position follows its input directly.
    moveAllAxes(xInput(), yInput(), zInput(), aInput());
  }
  else if (armState == PARKING && allAxesAtNeutral())
  {
    digitalWrite(EN, HIGH);
    armState = DISABLED;
    if (connected)
    {
      Xbox.setLedMode(SLOWBLINK);
    }
    Serial.print(F("\r\nLevers at neutral - motors off"));
  }

  printStatus();
}
