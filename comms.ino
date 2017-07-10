/* serial communication constants needed */
enum class CommandType : uint8_t {
  START = 0,
  STOP,
  PAUSE,
  ZERO,
  RESET,
  MOVE,
  CONFIGURE,
  MOVE_TO,
  SET_PHASE
};
CommandType cmdType;  // global command type - set when getting a command from serial

enum class Axis : uint8_t {
  BOTH = 0,
  AZIMUTH,
  ELEVATION,
};

byte sbuf[32];  // buffer to fill when reading from serial
uint8_t si = 0; // index of the buffer we are currently on


struct __attribute__((__packed__)) PositionMessage {
  unsigned long timestamp;
  uint8_t axis;
  int32_t angle;
};

void sendPosition(uint8_t axis, int32_t position) {

  PositionMessage msg;
  msg.timestamp = millis();
  msg.axis = axis;
  msg.angle = position;

  Serial.write(0xA0);
  Serial.write(0xB1);
  Serial.write((uint8_t) 2);
  Serial.write((uint8_t*) &msg, sizeof(msg));
  
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
      case CommandType::ZERO:
      case CommandType::RESET:
        Serial.readBytes(sbuf, 1);
        //Serial.write(sbuf, 1);
        break;

      /* start command has 2 bytes of additional data */
      case CommandType::START:
        Serial.readBytes(sbuf, 2);
        //Serial.write(sbuf, 2);
        break;

      /* move command has 3 bytes of additional data */
      case CommandType::MOVE:
        Serial.readBytes(sbuf, 3);
        //Serial.write(sbuf, 3);
        break;

      /* move to command has 5 bytes of additional data */
      case CommandType::MOVE_TO:
        Serial.readBytes(sbuf, 5);
        break;

      /* configure command has 14 additional bytes of data */
      case CommandType::CONFIGURE:
        Serial.readBytes(sbuf, 14);
        //Serial.write(sbuf, 14);
        break;

      case CommandType::SET_PHASE:
        Serial.readBytes(sbuf, 3);
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


int32_t buftoint32(int si) {
  return (((int32_t) sbuf[si+3]) << 24) | (((int32_t) sbuf[si+2]) << 16) | (((int32_t) sbuf[si+1]) << 8) | ((int32_t) sbuf[si]);
}


/**
 * actually do what is required of the command
 */
void handleCommand() {

  switch (cmdType) {
  
    case CommandType::START:
    {
      // DEBUG
      Serial1.println("start cmd received");
      
      // set the state to running
      state = State::RUNNING;

      // for debugging, also light up the LED
      digitalWrite(13, HIGH);

      // read axis and number of measurements to make
      Axis axis = static_cast<Axis> (sbuf[0]);
      uint8_t numMeasurements = sbuf[1];

      // set the configuration values
      autoChar.setNumMeasurements(numMeasurements);

      // set the type for the auto char based on the axis parameter
      switch (axis) {
        case Axis::BOTH:
          autoChar.setType(AutoCharacterization::Type::FULL);
          break;

        case Axis::AZIMUTH:
          autoChar.setType(AutoCharacterization::Type::AZIMUTH);
          break;

        case Axis::ELEVATION:
          autoChar.setType(AutoCharacterization::Type::ELEVATION);
          break;
      }

      // set the auto charactertization to the start position
      autoChar.setToStart();      
      break;
    }
    case CommandType::STOP:
      // DEBUG
      Serial1.println("stop cmd received");
      
      state = State::STOPPED;

      // for debugging, turn off the LED
      digitalWrite(13, LOW);
      break;
  
    case CommandType::PAUSE:
      // DEBUG
      Serial1.println("pause cmd received");
      
      state = State::PAUSED;

      // for debugging, turn off the LED
      digitalWrite(13, LOW);
      break;
  
    case CommandType::ZERO:
  
      break;
  
    case CommandType::RESET:
  
      break;
  
    case CommandType::MOVE:
    {
      // DEBUG
      Serial1.println("move cmd received");
      
      Axis axis = static_cast<Axis> (sbuf[0]);
      uint8_t dir = sbuf[1]; // 0 = cw, 1 = ccw
      uint8_t numSteps = sbuf[2];

      int dirSigned = dir;
      if (dirSigned == 0) {
        dirSigned = -1;
      }
      int stepsToMove = dirSigned * (int) numSteps;

      switch (axis) {
        case Axis::BOTH:
          azimuthStepper.move(stepsToMove);
          elevationStepper.move(stepsToMove);
          break;

        case Axis::AZIMUTH:
          azimuthStepper.move(stepsToMove);
          sendPosition(static_cast<uint8_t> (Axis::AZIMUTH), azimuthStepper.getCurrentMicroAngle());
          break;

        case Axis::ELEVATION:
          elevationStepper.move(stepsToMove);
          sendPosition(static_cast<uint8_t> (Axis::ELEVATION), elevationStepper.getCurrentMicroAngle());
          break;          
      }
      
      break;
    }
    case CommandType::MOVE_TO:
    {
      // DEBUG
      Serial1.println("move to cmd received");
      
      // get the axis for the command
      Axis axis = static_cast<Axis> (sbuf[0]);

      // the angle to command in microdegrees
      int32_t desiredAngle = buftoint32(1);

      // command the desired angle
      switch (axis) {
        case Axis::BOTH:
          break;

        case Axis::AZIMUTH:
          azimuthStepper.moveTo(desiredAngle);
          break;

        case Axis::ELEVATION:
          elevationStepper.moveTo(desiredAngle);
          break;
      }
      
      break;
    }
    case CommandType::CONFIGURE:
    {
      // DEBUG
      Serial1.println("configure cmd received");
      
      // extract the configuration values
      Axis axis = static_cast<Axis> (sbuf[0]);
      uint8_t stepSize = sbuf[1];  // TODO: actually use this parameter!!! - for now ignoring it

      // all angles here are in microdegrees
      int32_t stepIncrement = buftoint32(2);
      int32_t startAngle = buftoint32(6); //(((int32_t) sbuf[7]) << 24) | (((int32_t) sbuf[6]) << 16) | (((int32_t) sbuf[5]) << 8) | ((int32_t) sbuf[4]);
      int32_t endAngle =  buftoint32(10); //(((int32_t)sbuf[11]) << 24) | (((int32_t) sbuf[10]) << 16) | (((int32_t) sbuf[9]) << 8) | ((int32_t) sbuf[8]);

      // set the configuation value based on which axis was commanded
      switch (axis) {
        case Axis::BOTH:
          azimuthStepper.setNextStepSize(stepIncrement);
          elevationStepper.setNextStepSize(stepIncrement);

          autoChar.setAzimuthStepSize(stepIncrement);
          autoChar.setElevationStepSize(stepIncrement);
          autoChar.setAzimuthSweep(startAngle, endAngle);
          autoChar.setElevationSweep(startAngle, endAngle);
          break;

        case Axis::AZIMUTH:
          autoChar.setAzimuthStepSize(stepIncrement);
          autoChar.setAzimuthSweep(startAngle, endAngle);
          break;

        case Axis::ELEVATION:
          autoChar.setElevationStepSize(stepIncrement);
          autoChar.setElevationSweep(startAngle, endAngle);
          break;
      }
    
      break;
    }

    case CommandType::SET_PHASE:
    {
      // DEBUG
      Serial1.println("set phase cmd received");
      
      // need to calculate the fletcher checksum to append that when sending to PABSt
      uint8_t setPhaseMsgId = 0;
      uint8_t chkA = setPhaseMsgId;
      uint8_t chkB = chkA;
      for (uint8_t i = 0; i < 3; i++) {
        chkA += sbuf[i];
        chkB += chkA;
      }

      // DEBUG
      Serial1.print(sbuf[0]);
      Serial1.print(" ");
      Serial1.print(sbuf[1]);
      Serial1.print(" ");
      Serial1.print(sbuf[2]);
      Serial1.print(" ");
      Serial1.print(chkA, HEX);
      Serial1.print(" ");
      Serial1.println(chkB, HEX);
      
      // send over the set phase message + checksum
      Serial3.write(setPhaseMsgId);
      Serial3.write(sbuf, 3);
      Serial3.write(chkA);
      Serial3.write(chkB);
      
      break;
    }
    default:
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      break;
  }
  
}





