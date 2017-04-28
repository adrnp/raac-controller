#include <AngleStepper.h>
#include <RFPowerMonitor.h>

// CONSTANTS

/* the number of measurements to make at each azimuth/elevation set */
int NUM_MEASUREMENTS = 5;

int azAngle = 0;


/* the current state of the running */
enum State : uint8_t {
  NOT_STARTED = 0,    // <-- script has not gotten a start command from the controller
  RUNNING,            // <-- have started a characterization run
  PAUSED              // <-- paused a characterization run
};

State state = State::NOT_STARTED;

// Objects needed

AngleStepper azimuthStepper(AngleStepper::EIGTH_STEP, 3, 2);
RFPowerMonitor powerMonitor(RFPowerMonitor::F_880, 5, A0);

void setup() {

  Serial.begin(115200);

  // need to make sure that the monitor is enabled
  powerMonitor.enable();

  powerMonitor.setup();

}

void loop() {

  if (powerMonitor.getMeasurementCount() >= NUM_MEASUREMENTS) {
    powerMonitor.resetMeasurementCount();

    // move to the next position for a measurement and update the monitor's state
    azimuthStepper.moveToNext();
    powerMonitor.setAzimuth(azAngle);
    powerMonitor.setElevation(azAngle++);
    //Serial.println();
    //Serial.println(azimuthStepper.getCurrentAngle());
  }

  // continually call this to make sure the measurements are made and sent over serial
  powerMonitor.run();
}
