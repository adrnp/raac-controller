/* file that contains the overall state machine for the controller */

// includes
#include <AngleStepper.h>
#include <RFPowerMonitor.h>
#include <AutoCharacterization.h>

// Variables

/* the current state of the running */
enum class State : uint8_t {
  STOPPED = 0,    // <-- script has not gotten a start command from the controller
  RUNNING,            // <-- have started a characterization run
  PAUSED              // <-- paused a characterization run
};
State state = State::STOPPED;

/* characterization stuff */
AngleStepper azimuthStepper(AngleStepper::StepMode::EIGTH_STEP, 3, 2, 200, 1);
AngleStepper elevationStepper(AngleStepper::StepMode::EIGTH_STEP, 7, 6, 200, 4);
RFPowerMonitor powerMonitor(RFPowerMonitor::Frequency::F_880_MHz, 5, A0);

AutoCharacterization autoChar(AutoCharacterization::Type::FULL, AutoCharacterization::Mode::PHI_THETA, &powerMonitor, &azimuthStepper, &elevationStepper);

// needed for continually sending data to the display
unsigned long pauseTime = 1000;  // ms time between measurements being sent (1Hz)
unsigned long lastTime = millis();

void setup() {

  Serial.begin(115200);

  // need to make sure that the monitor is enabled
  powerMonitor.enable();
  
  //powerMonitor.setup();

  // to speed things up, we will up the step sizes used
  azimuthStepper.setNextStepSize(16);
  elevationStepper.setNextStepSize(32);

  // take into account real world
  // note that the physical initial state is actually 90 degrees, so need to account for that
  elevationStepper.setCurrentAngle(90);


  // to speed things up with the characterization
  autoChar.setAzimuthSweep(0, 360000000);
  autoChar.setElevationSweep(0, 90000000);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // send a message that signals this is ready to go
  autoChar.sendStatus(AutoCharacterization::Status::PAUSED);

  // also send the current position to update the visual in matlab
  sendPosition(1, azimuthStepper.getCurrentMicroAngle());
  sendPosition(2, elevationStepper.getCurrentMicroAngle());

}

void loop() {

  // check the serial connection to see if there is a new command
  // if so, execute the command
  if (getCommand()) {
    handleCommand();
  }

  // DEBUG: for testing
  //testMotors();
  //runCharacterization();

  switch (state) {
    
    case State::STOPPED:
      // nothing to do here...
      // TODO: there should be some sort of initialize function
      // that probably needs to be called before we go to the runing state

      // want to always send position and measurement to the display - allows testing things
      // we will send at 10Hz
      if (millis() - lastTime > pauseTime) {
        autoChar.sendMeasurement(powerMonitor.makeMeasurement());
        lastTime = millis();
      }
      
      break;
      
    case State::RUNNING:

      // run the charactertization -> this should be called once per loop
      // this handles moving the motors, making measurements and send data
      autoChar.run();

      // once completed, reset everything
      if (autoChar.isCompleted()) {
        state = State::STOPPED;

        // reset everything to 0 (this reset the motors and all the characterization states)
        autoChar.reset();
        //azimuthStepper.reset();
        //elevationStepper.reset();
      }
      
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
  elevationStepper.moveBy((float) 9.0);
  delay(500);
}
















