#include <AngleStepper.h>
#include <RFPowerMonitor.h>

// CONSTANTS

/* the number of measurements to make at each azimuth/elevation set */
int NUM_MEASUREMENTS = 5;

int azAngle = 0;


/* the current state of the running */
enum class State : uint8_t {
  NOT_STARTED = 0,    // <-- script has not gotten a start command from the controller
  RUNNING,            // <-- have started a characterization run
  PAUSED              // <-- paused a characterization run
};

State state = State::NOT_STARTED;

// Objects needed

AngleStepper azimuthStepper(AngleStepper::EIGTH_STEP, 3, 2);
AngleStepper elevationStepper(AngleStepper::EIGTH_STEP, 7, 6);
RFPowerMonitor powerMonitor(RFPowerMonitor::F_880, 5, A0);


/* serial communication constants needed */
enum class CommandType : uint8_t {
  START = 0,
  STOP,
  PAUSE,
  ZERO,
  RESET,
  MOVE,
  CONFIGURE
};
CommandType cmdType;  // global command type - set when getting a command from serial

enum class Axis : uint8_t {
  BOTH = 0,
  AZIMUTH,
  ELEVATION,
};

byte sbuf[32];  // buffer to fill when reading from serial
uint8_t si = 0; // index of the buffer we are currently on




void setup() {

  Serial.begin(115200);

  // need to make sure that the monitor is enabled
  powerMonitor.enable();
  powerMonitor.setAzimuth(azimuthStepper.getCurrentAngle());
  powerMonitor.setElevation(azimuthStepper.getCurrentAngle());

  //powerMonitor.setup();

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

}

void loop() {

  // check the serial connection to see if there is a new command
  // if so, execute the command
  if (getCommand()) {
    handleCommand();
  }

  // DEBUG: for testing
  testMotors();

  switch (state) {
    
    case State::NOT_STARTED:
      // nothing to do here...
      // TODO: there should be some sort of initialize function
      // that probably needs to be called before we go to the runing state
      break;
      
    case State::RUNNING:
      // TODO: this is where the real work goes
      // once a start command comes in, code should start and move to the running state

      // for actual runs
      //runCharacterization();

      // DEBUG: for testing
      //testMotors();
      
      break;

    case State::PAUSED:
      // TODO: this should be triggered if a pause command is received
      // continuously reads from serial waiting for a restart command
      break;

    default:
      // for now, nothing to do for the unknown state case
      break;
  }
  

  
}


/**
 * helper function to be able to quickly run the loop to test the motors
 */
void testMotors() {
  // simply move each motors one step at a time
  // TODO: this is where might want a run command...
  azimuthStepper.moveToNext();
  elevationStepper.moveBy(9);
  delay(500);
}


/**
 * helper function for actually running the characterization
 */
void runCharacterization() {
  if (powerMonitor.getMeasurementCount() >= NUM_MEASUREMENTS) {
    powerMonitor.resetMeasurementCount();

    // move to the next position for a measurement and update the monitor's state
    azimuthStepper.moveToNext();
    powerMonitor.setAzimuth(azimuthStepper.getCurrentAngle());
    powerMonitor.setElevation(0);
    //Serial.println();
    //Serial.println(azimuthStepper.getCurrentAngle());
  }

  // continually call this to make sure the measurements are made and sent over serial
  powerMonitor.run();
}



/**
 * checks to see if there is a command from the serial port.
 * 
 * TODO: figure out if want to execute the command here
 * or just write to some global command variable that can then
 * get handled in the main loop...
 * 
 * returns true if a command was received
 */
bool getCommand() {

  // check to see if we have serial data
  if (Serial.available() > 0) {

    // DEBUG
    digitalWrite(13, HIGH);

    // the first byte is the command type, so just read that
    char b = Serial.read();
    //Serial.write(b);
    cmdType = static_cast<CommandType>(b);

    // the number of bytes to read now will depend on the command type
    switch (cmdType) {

      /* commands that only have a type */
      case CommandType::STOP:
      case CommandType::PAUSE:
        break;

      /* commands that have a length of 1 byte */
      case CommandType::START:
      case CommandType::ZERO:
      case CommandType::RESET:
        Serial.readBytes(sbuf, 1);
        //Serial.write(sbuf, 1);
        break;

      /* move command has 2 bytes of additional data */
      case CommandType::MOVE:
        Serial.readBytes(sbuf, 2);
        //Serial.write(sbuf, 2);
        break;

      /* configure command has 6 additional bytes of data */
      case CommandType::CONFIGURE:
        Serial.readBytes(sbuf, 6);
        //Serial.write(sbuf, 6);
        break;
        
    }

    // DEBUG
    digitalWrite(13, LOW);
    
    return true;

    // TODO: catch the errors that might occur if for some reason
    // we time out before reading the entire command
  }

  return false;
}



/**
 * actually do what is required of the command
 */
void handleCommand() {

  switch (cmdType) {
  
    case CommandType::START:
      // set the state to running
      state = State::RUNNING;

      // for debugging, also light up the LED
      digitalWrite(13, HIGH);

      // TODO: read the desired axis and handle that
      
      break;
  
    case CommandType::STOP:
      state = State::NOT_STARTED;

      // for debugging, turn off the LED
      digitalWrite(13, LOW);
      break;
  
    case CommandType::PAUSE:
      state = State::PAUSED;

      // for debugging, turn off the LED
      digitalWrite(13, LOW);
      break;
  
    case CommandType::ZERO:
  
      break;
  
    case CommandType::RESET:
  
      break;
  
    case CommandType::MOVE:
  
      break;
  
    case CommandType::CONFIGURE:
  
      break;
    default:
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      break;
  }
  
}










