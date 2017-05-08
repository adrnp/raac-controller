/* file that will contain all of the characterization related functions */

// CONSTANTS
const int DEFAULT_NUM_MEASUREMENTS = 5;

enum class Characterization : uint8_t {
  FULL = 0,
  AZIMUTH,
  ELEVATION
};


// Run Configuration Information
int minAzAngle = 0;
int maxAzAngle = 360;

int minElAngle = 0;
int maxElAngle = 90;

int numMeasurements = DEFAULT_NUM_MEASUREMENTS;

Characterization characterizationType = Characterization::FULL;


// Run State Information
float currentAzAngle = 0;
float currentElAngle = 0;

bool azimuthCompleted = false;
bool elevationCompleted = false;


void configureCharacterizationRun() {
  // TODO: fill this in to be able to configure everything for the characterization
  // e.g. motor speed, step size, etc
}



/**
 * helper function to set azimuth angle.
 * depending on the current state.
 */
void setAzimuth() {

  // if still have azimuth to sweep out, move to the next step
  if (azimuthStepper.getCurrentAngle() < maxAzAngle) {
    azimuthStepper.moveToNext();
    
  } else {  // once we've swept through the entire azimuth range, reset
    azimuthStepper.moveTo(minAzAngle);
    azimuthCompleted = true;
  }
}


/**
 * helper function to set the elevation angle.
 * depending on the current state.
 */
void setElevation() {

  // only move the elevation when the azimuth sweep has completed
  if (!azimuthCompleted) {
    return;
  }

  // move the elevation motor - means we need another azimuth sweep, so change the flag
  if (elevationStepper.getCurrentAngle() < maxElAngle) {
    elevationStepper.moveToNext();
    azimuthCompleted = false;
  } else {  // reset the elevation motor and flag completion
    elevationStepper.moveTo(minElAngle);
    elevationCompleted = true;
  }
}




/**
 * helper function to run a characterization.
 * 
 */
void runCharacterization() {

  // move motors, only if have made enough measurements
  // TODO: move to class
  if (powerMonitor.getMeasurementCount() >= numMeasurements) {
    bool completed = false;

    // motor movement is based on which characterization is being run
    switch(characterizationType) {

      /* sweeping azimuth and elevation */
      case Characterization::FULL:
        completed = updateFullCharacterization();
        break;

      /* sweeping only azimuth */
      case Characterization::AZIMUTH:
        completed = updateAzimuthCharacterization();
        break;

      /* sweeping only elevation */
      case Characterization::ELEVATION:
        completed = updateElevationCharacterization();
        break;
    }

    // end the run if the characterization is completed
    if (completed) {
      state = State::NOT_STARTED;
      return;
    }
  }

  // run the measurement making scheme
  // TODO: move to class
  powerMonitor.run();
}


/**
 * sweeps through all elevation and azimuth.
 * the sweep structure is elevation on outer loop and azimuth on inner loop.
 */
bool updateFullCharacterization() {
  setAzimuth();
  setElevation();

  // update the power monitor information for angles
  // TODO: move to class
  powerMonitor.setAzimuth(azimuthStepper.getCurrentAngle());
  powerMonitor.setElevation(elevationStepper.getCurrentAngle());

  // return completion state
  return (elevationCompleted && azimuthCompleted);
}

bool updateAzimuthCharacterization() {
  setAzimuth();

  // update the power monitor information for angles
  // TODO: move to class
  powerMonitor.setAzimuth(azimuthStepper.getCurrentAngle());

  // return completion state
  return azimuthCompleted;
}


bool updateElevationCharacterization() {
  setElevation();

  // update the power monitor information for angles
  // TODO: move to class
  powerMonitor.setElevation(elevationStepper.getCurrentAngle());

  // return completion state
  return elevationCompleted;
}

