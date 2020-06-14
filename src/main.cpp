
//#include <Servo.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//#include <LiquidCrystal_PCF8574.h>

#include <I2Cdev.h>
#include "Battery.h"
#include "Wheelmotor.h"
#include "CutterMotor.h"
#include "Controller.h"
#include "Error.h"
#include "Definition.h"

#ifdef DEBUG_ENABLED
  #include "SetupDebug.h"
#endif

// Global variables
  unsigned long lastUpdate;           //Move to global variables
  int interval = 1000;                //Move to global variables
  struct MOWERDATA {          // move to global variable
    String activity;          //Current activity, I.E mowing, looking for signal, charging etc.
    int SoC;                  //Current battery level
    String SoCString;         //Current battery level in string format 
    float batMin;             //Minimum defined level of battery. Depleted level.
    float batMax;             //Maximum defined level of battery. SoC when to consider charging complete
  } mowerData;                //Object name. Fetch or edit data by calling for instance mowerData.SoC = batteryLevel;


int state;
long time_at_turning = millis();
int extraTurnAngle = 0;

long olastemp = millis();
// Set up all the defaults (check the Definition.h file for all default values)
DEFINITION Defaults;

// Please select which type of cutter motor you have
CUTTERMOTOR CutterMotor(CUTTER_MOTOR_TYPE, CUTTER_PWM_PIN, CUTTER_CURRENT_PIN);

// Wheelmotors
WHEELMOTOR rightMotor(WHEEL_MOTOR_A_PWM_PIN, WHEEL_MOTOR_A_DIRECTION_PIN, WHEEL_MOTOR_A_CURRENT_PIN, WHEELMOTOR_SMOOTHNESS);
WHEELMOTOR leftMotor(WHEEL_MOTOR_B_PWM_PIN, WHEEL_MOTOR_B_DIRECTION_PIN, WHEEL_MOTOR_B_CURRENT_PIN, WHEELMOTOR_SMOOTHNESS);

// Battery
BATTERY Battery(BATTERY_TYPE, BAT_PIN, DOCK_PIN);

// Controller (pass adresses to the motors and sensors for the controller to operate on)
//CONTROLLER Mower(&leftMotor, &rightMotor, &CutterMotor, &Sensor, &Compass);
CONTROLLER Mower(&leftMotor, &rightMotor, &CutterMotor);

#ifdef DEBUG_ENABLED
//SETUPDEBUG SetupAndDebug(&Mower, &leftMotor, &rightMotor, &CutterMotor, &Sensor, &Compass, &Battery);
SETUPDEBUG SetupAndDebug(&Mower, &leftMotor, &rightMotor, &CutterMotor, &Battery);
#endif

// Error handler
//ERROR Error(&Display, LED_PIN, &Mower);

void doInterruptThings() {
}

void setupInterrupt() {

  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  //cli();//stop interrupts

  //TCCR0A = 0;// set entire TCCR0A register to 0
  //TCCR0B = 0;// same for TCCR0B
  //TCNT0 = 0;//initialize counter value to 0
  //          // set compare match register for 2khz increments
  //OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  //            // turn on CTC mode
  //TCCR0A |= (1 << WGM01);
  //// Set CS01 and CS00 bits for 64 prescaler
  //TCCR0B |= (1 << CS01) | (1 << CS00);
  //// enable timer compare interrupt
  //TIMSK0 |= (1 << OCIE0A);

  //sei();//allow interrupts
}
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect)
{
   doInterruptThings();
}


// ****************** SETUP ******************************************
void setup() {
  // Slow communication on the serial port for all terminal messages.
  // Slowed down for better stability. 
  Serial.begin(9600);

  // Configure all the pins for input or output
  Defaults.definePinsInputOutput();

  // Turn off the cutter motor as fast as possible
  CutterMotor.initialize();

  // Set default levels (defined in Definition.h) for your mower
  Defaults.setDefaultLevels(&Battery, &leftMotor, &rightMotor, &CutterMotor);

  setupInterrupt();

  // Reset the battery voltage reading
  Serial.println("Reset batt reading");

  Battery.resetVoltage();

  // Run the updateBWF function every time there is a pulse on digital pin2
//  attachInterrupt(0, updateBWF, RISING);
  
  #ifdef DEBUG_ENABLED
  Serial.println(F("----------------"));
  Serial.println(F("Send D to enter setup and debug mode"));
  #endif
  delay(5000);
  #ifdef DEBUG_ENABLED
  state = SetupAndDebug.tryEnterSetupDebugMode(0);
  #endif
  
  if (state != SETUP_DEBUG) {
    if (Battery.isBeingCharged()) {
      state = CHARGING;
      Mower.stopCutter();
    }
    else {
      state = MOWING;
    }
  }
// state = IDLE;
} // setup.

// TODO: This should probably be in Controller
void randomTurn(bool goBack) {
  if(goBack) {
    Mower.runBackward(FULLSPEED);
    delay(2000);
  }

  int angle = random(90, 160);
  if (random(0, 100) % 2 == 0) {
    Mower.turnRight(angle);
  } else {
    Mower.turnLeft(angle);
  }
  time_at_turning = millis();
  Mower.runForward(MOWING_SPEED);
}


// ***************** SAFETY CHECKS ***********************************
void checkIfFlipped() {
#if defined __MS9150__ || defined __MS5883L__ || __ADXL345__
Serial.println("FlipSensor Defined");
  if (Mower.hasFlipped()) {
    Serial.print("Mower has flipped ");
    Mower.stopCutter();
    Mower.stop();
    Error.flag(ERROR_TILT);
  }
#endif
}

void checkIfLifted() {
#if defined __Lift_Sensor__
  Serial.println("LiftSensor Defined");
  if (Mower.isLifted()) {
    Serial.println("Mower is lifted");
    Mower.stopCutter();
    Mower.stop();
    Mower.runBackward(FULLSPEED);
    delay(2000);
    if(Mower.isLifted())
      Error.flag(ERROR_LIFT);
    Mower.randomTurn(false);
  }
#endif
}


// ***************** MOWING ******************************************
void doMowing() {
  if (Battery.mustCharge()) {
    state = IDLE;
    return;
  }

  // Make regular turns to avoid getting stuck on things
  if ((millis() - time_at_turning) > TURN_INTERVAL) {
    randomTurn(true);
    return;
  }

  // Avoid obstacles
  Mower.turnIfObstacle();

  // When mowing, the cutter should be on and we should be going forward
  Mower.startCutter();
  Mower.runForwardOverTime(SLOWSPEED, MOWING_SPEED, ACCELERATION_DURATION);

  // Adjust the speed of the mower to the grass thickness
  Mower.compensateSpeedToCutterLoad();
}

void doWait()
{
  char buf[30];
  delay(500);
}

//void awareDelay(int ms) {
//  unsigned long exitAt = millis() + ms;
//  int sensor = Sensor.getCurrentSensor();
//  while (millis() < exitAt) {
//    Sensor.select(sensor % NUMBER_OF_SENSORS);
//    Sensor.sensorOutside[Sensor.getCurrentSensor()] = Sensor.isOutOfBounds();
//  }
//}
// ***************** MAIN LOOP ***************************************
void loop() {

  static long lastDisplayUpdate = 0;
  static int previousState;

  long looptime = millis();

  if ((millis() - lastUpdate) > interval) {   //Place this if statement to void loop
    mowerData.activity = state;
    //connectedLiam();
  }

  if((state = SetupAndDebug.tryEnterSetupDebugMode(state)) == SETUP_DEBUG)
    return;

  Battery.updateVoltage();


  // int startingSensor = Sensor.getCurrentSensor();
  // Check state of all sensors
  // for(int i = startingSensor; i < startingSensor + NUMBER_OF_SENSORS; i++) {

  //   Sensor.select(i % NUMBER_OF_SENSORS);
  //   Sensor.sensorOutside[Sensor.getCurrentSensor()] = Sensor.isOutOfBounds();
  // }

  // Safety checks
  checkIfFlipped();
  checkIfLifted();

  switch(state) {
    case MOWING:
      doMowing();
      break;
    /*
    case LAUNCHING:
      doLaunching();
      break;
    case DOCKING:
      doDocking();
      break;
    */
    /*
    case LOOKING_FOR_BWF:
      doLookForBWF();
      break;
    */
    /*
    case CHARGING:
      doCharging();
      break;
    */
      case IDLE:
      doWait();
      break;
  }

  if(millis()-lastDisplayUpdate > 5000) {
    // olastemp = millis();
    //Mower.stop();
    //Display.update();
    //Mower.runForwardOverTime(SLOWSPEED, MOWING_SPEED, ACCELERATION_DURATION);
    // Serial.print("\nprintTime : ");
    // Serial.println(millis() -olastemp);
    lastDisplayUpdate = millis();
  }
}