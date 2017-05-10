/* file that contains the overall state machine for the controller */

// includes
#include <AngleStepper.h>
#include <RFPowerMonitor.h>



// Variables

/* the current state of the running */
enum class State : uint8_t {
  NOT_STARTED = 0,    // <-- script has not gotten a start command from the controller
  RUNNING,            // <-- have started a characterization run
  PAUSED              // <-- paused a characterization run
};
State state = State::NOT_STARTED;

/* characterization stuff */
AngleStepper azimuthStepper(AngleStepper::EIGTH_STEP, 3, 2);
AngleStepper elevationStepper(AngleStepper::EIGTH_STEP, 7, 6);
RFPowerMonitor powerMonitor(RFPowerMonitor::F_880, 5, A0);



void setup() {

  Serial.begin(115200);

  // need to make sure that the monitor is enabled
  powerMonitor.enable();
  powerMonitor.setAzimuth(azimuthStepper.getCurrentAngle());
  powerMonitor.setElevation(azimuthStepper.getCurrentAngle());

  //powerMonitor.setup();

  // to speed things up, we will up the step sizes used
  azimuthStepper.setNextStepSize(16);
  elevationStepper.setNextStepSize(32);

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
  //testMotors();
  runCharacterization();

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
















