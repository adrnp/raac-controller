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

      // set the auto charactertization to the start position
      autoChar.setToStart();

      // for debugging, also light up the LED
      digitalWrite(13, HIGH);

      // TODO: read the desired axis and handle that
      
      break;
  
    case CommandType::STOP:
      state = State::STOPPED;

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

