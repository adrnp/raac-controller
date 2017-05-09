/* file that will contain all of the characterization related functions */

// CONSTANTS
const int DEFAULT_NUM_MEASUREMENTS = 5;

enum class Characterization : uint8_t {
  FULL = 0,
  AZIMUTH,
  ELEVATION
};

/*
struct __attribute__((__packed__)) MeasurementMessage {
  unsigned long timestamp;
  float signalStrength;
  float azimuth;
  float elevation;
};
*/

// Run Configuration Information
int minAzAngle = 0;
int maxAzAngle = 359;  // DEBUG - for testing, not doing a full 360

int minElAngle = 0;
int maxElAngle = 90;

int numMeasurements = 1; //DEFAULT_NUM_MEASUREMENTS;

Characterization characterizationType = Characterization::FULL;

/*
int measurementRate = 10; // [Hz]
float timeout = 1.0/measurementRate * 1000.0; // for loop rate
*/

// Run State Information
float currentAzAngle = 0;
float currentElAngle = 0;

bool azimuthCompleted = false;
bool elevationCompleted = false;

/*
int measurementCount = 0;
long lastMeasurementTime = 0;
*/

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
  if (azimuthStepper.getAngleSwept() < maxAzAngle) {
    azimuthStepper.moveToNext();
    
  } else {  // once we've swept through the entire azimuth range, reset
    azimuthStepper.moveTo(0);
    azimuthStepper.resetAngleSwept();
    azimuthCompleted = true;

    Serial.println();
    Serial.println("azimuth completed");
    Serial.println();
  }

  Serial.println();
  Serial.print("azimuth: ");
  Serial.print(azimuthStepper.getCurrentAngle());
  Serial.print(", ");
  Serial.print(azimuthStepper.getAngleSwept());
  Serial.println();
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

    Serial.println();
    Serial.println("elevation completed");
    Serial.println();
  }

  Serial.println();
  Serial.print("elevation: ");
  Serial.print(elevationStepper.getCurrentAngle());
  Serial.print(", ");
  Serial.print(elevationStepper.getCurrentStep());
  Serial.println();
}




/**
 * helper function to run a characterization.
 * 
 */
void runCharacterization() {

  // want to do this at a specific frequency
  /*
  if (millis() - lastMeasurementTime < timeout) {
    return;
  }
  */

  // move motors, only if have made enough measurements
  // TODO: move to class
  if (powerMonitor.getMeasurementCount() >= numMeasurements) {
    powerMonitor.resetMeasurementCount();  // important to reset measurement counter!!!
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

  // run the measurement scheme
  // TODO: move to class
  powerMonitor.run();

  /*
  // make a single measurement
  float measurement = powerMonitor.makeMeasurement();
  measurementCount++;
  lastMeasurementTime = millis();

  // send the data to the computer
  sendData(measurement, azimuthStepper.getCurrentAngle(), elevationStepper.getCurrentAngle());
  */
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


/**
 * send the data to the computer
 */
/*
void sendData(float signalStrength, float azimuth, float elevation) {

  // pack the message
  MeasurementMessage msg;
  msg.timestamp = lastMeasurementTime;
  msg.signalStrength = signalStrength;
  msg.azimuth = azimuth;
  msg.elevation = elevation;


  // send the data over the serial port
  // sync bytes first
  Serial.write(0xA0);
  Serial.write(0xB1);

  // the actual data
  Serial.write((uint8_t*) &msg, sizeof(msg));

  return;
}
*/


