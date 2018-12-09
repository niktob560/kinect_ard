//<============================================================================[START-STOP funcs]============================================================================>


/*
   Function startTurnACW
   Desc     Start turning anti-clockwise
   Input    none
   Output   none
*/
void startTurnManACW(void)
{
  digitalWrite(TURN_MAN_ACW_PIN, HIGH);
}


/*
   Function stopTurnManACW
   Desc     Stop turning manipulator anti-clockwise
   Input    none
   Output   none
*/
void stopTurnManACW(void)
{
  digitalWrite(TURN_MAN_ACW_PIN, LOW);
}


/*
   Function startTurnManCW
   Desc     Start turning manipulator clockwise
   Input    none
   Output   none
*/
void startTurnManCW(void)
{
  digitalWrite(TURN_MAN_CW_PIN, HIGH);
}


/*
   Function stopTurnManCW
   Desc     Stop turning manipulator clockwise
   Input    none
   Output   none
*/
void stopTurnManCW(void)
{
  digitalWrite(TURN_MAN_CW_PIN, LOW);
}


/*
   Function startExtendMan
   Desc     Start extending manipulator
   Input    none
   Output   none
*/
void startExtendMan(void)
{
  digitalWrite(EXT_MAN_PIN, HIGH);
}


/*
   Function stopExtendMan
   Desc     Stop extending manipulator
   Input    none
   Output   none
*/
void stopExtendMan(void)
{
  digitalWrite(EXT_MAN_PIN, LOW);
}


/*
   Function staRtetractMan
   Desc     Start retracting manipulator
   Input    none
   Output   none
*/
void startRetractMan(void)
{
  digitalWrite(RETRACT_MAN_PIN, HIGH);
}


/*
   Function stopRetractMan
   Desc     Stop retracting manipulator
   Input    none
   Output   none
*/
void stopRetractMan(void)
{
  digitalWrite(RETRACT_MAN_PIN, LOW);
}


/*
   Function startElevateMan
   Desc     Start elevating manipulator (from bottom to top)
   Input    none
   Output   none
*/
void startElevateMan(void)
{
  digitalWrite(ELEVATE_MAN_PIN, HIGH);
}


/*
   Function stopElevateMan
   Desc     StopElevateManipulator (from bottom to top)
   Input    none
   Output   none
*/
void stopElevateMan(void)
{
  digitalWrite(ELEVATE_MAN_PIN, LOW);
}


/*
   Function startDescendMan
   Desc     Start descending manipulator (from top to bottom)
   Input    none
   Output   none
*/
void startDescendMan(void)
{
  digitalWrite(DESCEND_MAN_PIN, HIGH);
}


/*
   Function stopDescendMan
   Desc     Stop descending manipulator (from top to bottom)
   Input    none
   Output   none
*/
void stopDescendMan(void)
{
  digitalWrite(DESCEND_MAN_PIN, LOW);
}


/*
   Function startExtendGrabber
   Desc     Start extending grabber on manipulator
   Input    none
   Output   none
*/
void startExtendGrabber(void)
{
  digitalWrite(GRABBER_EXTEND_PIN, HIGH);
}


/*
   Function stopExtendGrabber
   Desc     Stop extending grabber on manipulator
   Input    none
   Output   none
*/
void stopExtendGrabber(void)
{
  digitalWrite(GRABBER_EXTEND_PIN, LOW);
}



/*
   Function startRetractGrabber
   Desc     Start retracting grabber on manipulator
   Input    none
   Output   none
*/
void startRetractGrabber(void)
{
  digitalWrite(GRABBER_RETRACT_PIN, HIGH);
}


/*
   Function stopRetractGrabber
   Desc     Stop Retracting grabber on manipulator
   Input    none
   Output   none
*/
void stopRetractGrabber(void)
{
  digitalWrite(GRABBER_RETRACT_PIN, LOW);
}


//<============================================================================[Motor funcs]============================================================================>

//TODO: refactor


/*
   Function setLeftMotorDir
   Desc     Set direction of left motor
   Input    bool _dir: what direction set to
   Output   none
*/
void setLeftMotorDir(bool _dir)
{
  digitalWrite(LEFT_MOTOR_DIR_PIN, _dir == FORWARD);
}


/*
   Function startLeftMotor
   Desc     Start left motor with speed
   Input    unsigned int _spd: speed of motor
   Output   none
*/
void startLeftMotor(unsigned int _spd)
{
  analogWrite(LEFT_MOTOR_SPD_PIN, _spd);
}


/*
   Function enableLeftMotor
   Desc     Enable left motor
   Input    bool _en: state (is enabled)
   Output   none
*/
void enableLeftMotor(bool _en)
{
  digitalWrite(LEFT_MOTOR_EN_PIN, _en);
}


/*
   Function enableRightMotor
   Desc     Enable right motor
   Input    bool _en: state (is enabled)
   Output   none
*/
void enableRightMotor(bool _en)
{
  digitalWrite(RIGHT_MOTOR_EN_PIN, _en);
}


/*
   Function setRightMotorDir
   Desc     Set direction of right motor
   Input    bool _dir: what direction set to
   Output   none
*/
void setRightMotorDir(bool _dir)
{
  digitalWrite(RIGHT_MOTOR_DIR_PIN, _dir != FORWARD);
}


/*
   Function startRightMotor
   Desc     Start right motor with speed
   Input    unsigned int _spd: speed of motor
   Output   none
*/
void startRightMotor(unsigned int _spd)
{
  analogWrite(RIGHT_MOTOR_SPD_PIN, _spd);
}


/*
   Function turnLeftMotor_us
   Desc     Turn left motor for some time
   Input    unsigned long _time: what time will turn in micros
   Output   none
*/
void turnLeftMotor_us(unsigned long _time)
{
  while (_time >= DELAY_MICROS * 2)
  {
    digitalWrite(LEFT_MOTOR_SPD_PIN, HIGH);
    delayMicroseconds(DELAY_MICROS);
    digitalWrite(LEFT_MOTOR_SPD_PIN, LOW);
    delayMicroseconds(DELAY_MICROS);
    _time -= DELAY_MICROS * 2;
  }
}


/*
   Function turnLeftMotor_imps
   Desc     Turn left motor for some imps
   Input    unsigned long _imps: what num of imps generate to
   Output   none
*/
void turnLeftMotor_imps(unsigned long _imps)
{
  while (_imps)
  {
    digitalWrite(LEFT_MOTOR_SPD_PIN, HIGH);
    delayMicroseconds(DELAY_MICROS);
    digitalWrite(LEFT_MOTOR_SPD_PIN, LOW);
    delayMicroseconds(DELAY_MICROS);
    _imps--;
  }
}


/*
   Function turnLeftMotor_ms
   Desc     Turn left motor for some time
   Input    unsigned int _time: what time will turn in millis
   Output   none
*/
inline void turnLeftMotor_ms(unsigned int _time)
{
  turnLeftMotor_us(MICROS_IN_MILLIS * _time);
}


/*
   Function turnLeftMotor_s
   Desc     Turn left motor for some time
   Input    unsigned long _time: what time will turn in seconds
   Output   none
*/
inline void turnLeftMotor_s(unsigned int _time)
{
  turnLeftMotor_us(MICROS_IN_S * _time);
}


/*
   Function turnRightMotor_us
   Desc     Turn right motor for some time
   Input    unsigned long _time: what time will turn in micros
   Output   none
*/
void turnRightMotor_us(unsigned long _time)
{
  while (_time >= DELAY_MICROS * 2)
  {
    digitalWrite(RIGHT_MOTOR_SPD_PIN, 1);
    delayMicroseconds(DELAY_MICROS);
    digitalWrite(RIGHT_MOTOR_SPD_PIN, 0);
    delayMicroseconds(DELAY_MICROS);
    _time -= DELAY_MICROS * 2;
  }
}


/*
   Function turnRightMotor_imps
   Desc     Turn right motor for some imps
   Input    unsigned long _imps: what num of imps generate to
   Output   none
*/
void turnRightMotor_imps(unsigned long _imps)
{
  while (_imps)
  {
    digitalWrite(LEFT_MOTOR_SPD_PIN, HIGH);
    delayMicroseconds(DELAY_MICROS);
    digitalWrite(LEFT_MOTOR_SPD_PIN, LOW);
    delayMicroseconds(DELAY_MICROS);
    _imps--;
  }
}


/*
   Function turnRightMotor_ms
   Desc     Turn right motor for some time
   Input    unsigned int _time: what time will turn in millis
   Output   none
*/
inline void turnRightMotor_ms(unsigned int _time)
{
  turnRightMotor_us(MICROS_IN_MILLIS * _time);
}


/*
   Function turnRightMotor_s
   Desc     Turn right motor for some time
   Input    unsigned int _time: what time will turn in seconds
   Output   none
*/
inline void turnRightMotor_s(unsigned int _time)
{
  turnRightMotor_us(MICROS_IN_S * _time);
}


/*
   Function turnBothMotors_us
   Desc     Turn both motors for some time
   Input    unsigned long _time: what time will turning in micros
   Output   none
*/
void turnBothMotors_us(unsigned long _time)
{
  while (_time >= DELAY_MICROS * 2)
  {
    digitalWrite(RIGHT_MOTOR_SPD_PIN, 1);
    digitalWrite(LEFT_MOTOR_SPD_PIN, 1);
    delayMicroseconds(DELAY_MICROS);
    digitalWrite(RIGHT_MOTOR_SPD_PIN, 0);
    digitalWrite(LEFT_MOTOR_SPD_PIN, 0);
    delayMicroseconds(DELAY_MICROS);
    _time -= DELAY_MICROS * 2;
  }
}


/*
   Function turnBothMotors_imps
   Desc     Turn both motors for some ticks
   Input    unsigned long _imps: what num of impulses will generate
   Output   none
*/
void turnBothMotors_imps(unsigned long _imps)
{
  while (_imps)
  {
    digitalWrite(RIGHT_MOTOR_SPD_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_SPD_PIN, HIGH);
    delayMicroseconds(DELAY_MICROS);
    digitalWrite(RIGHT_MOTOR_SPD_PIN, LOW);
    digitalWrite(LEFT_MOTOR_SPD_PIN, LOW);
    delayMicroseconds(DELAY_MICROS);
    _imps--;
  }
}


/*
   Function turnBothMotors_ms
   Desc     Turn both motors for some time
   Input    unsigned long _time: what time will turning in millis
   Output   none
*/
inline void turnBothMotors_ms(unsigned int _time)
{
  turnBothMotors_us(MICROS_IN_MILLIS * _time);
}


/*
   Function turnBothMotors_s
   Desc     Turn both motors for some time
   Input    unsigned long _time: what time will turning in seconds
   Output   none
*/
inline void turnBothMotors_s(unsigned int _time)
{
  turnBothMotors_us(MICROS_IN_S * _time);
}


/*
   Function driveForward
   Desc     Drive robot to forward for some imps
   Input    unsigned long _imps: what imps generate to
   Output   none
*/
void driveForward(unsigned long _imps)//TODO: override for degrees and mm's
{
  setRightMotorDir(FORWARD);
  setLeftMotorDir(FORWARD);
  turnBothMotors_imps(_imps);
}


/*
   Function driveBackward
   Desc     Drive robot to backward for some imps
   Input    unsigned long _imps: what imps generate to
   Output   none
*/
void driveBackward(unsigned long _imps)//TODO: override for degrees and mm's
{
  setRightMotorDir(BACKWARD);
  setLeftMotorDir(BACKWARD);
  turnBothMotors_imps(_imps);
}


/*
   Function driveRight
   Desc     Drive robot to right for some imps
   Input    unsigned long _imps: what imps generate to
   Output   none
*/
void driveRight(unsigned long _imps)//TODO: override for degrees and mm's
{
  setRightMotorDir(BACKWARD);
  setLeftMotorDir(FORWARD);
  turnBothMotors_imps(_imps);
}


/*
   Function driveLeft
   Desc     Drive robot to left for some imps
   Input    unsigned long _imps: what imps generate to
   Output   none
*/
void driveLeft(unsigned long _imps)//TODO: override for degrees and mm's
{
  setRightMotorDir(FORWARD);
  setLeftMotorDir(BACKWARD);
  turnBothMotors_imps(_imps);
}




//<============================================================================[Manipulator funcs]============================================================================>

//TODO: add grabber funcs


/*
   Function manExtendTo
   Desc     extend manipulator to N pulses
   Input    long _p: target pulses
   Output   none
*/
void manExtendTo(long _p)
{
  int _startImps = extEnc.read();
  Serial.println(_startImps);
  if (_p < 0)
  {
    startRetractMan();
    Serial.println("retracting...");
    while (extEnc.read() > _p + _startImps) //TODO: check ranges
    {
      Serial.println(extEnc.read());
    }
    stopRetractMan();
  }
  else
  {
    startExtendMan();

    Serial.println("extending...");
    while (extEnc.read() < _p + _startImps
           && digitalRead(EXTEND_LIMIT_PIN))
    {
      Serial.println(extEnc.read());
    }
    stopExtendMan();
    if (!digitalRead(EXTEND_LIMIT_PIN))
    {
      extEnc.write(0);
    }
  }
}


/*
   Function manExtendTo_mm
   Desc     Extend manipulator in mm
   Input    int _mm: what millimeters extend to
   Output   none
*/
inline void manExtendTo_mm(int _mm)
{
  manExtendTo(_mm * EXT_IMPS_PER_MM);
}


/*
   Function manExtendToZero
   Desc     Set extend of manipulator to zero
   Input    none
   Output   none
*/
void manExtendToZero(void)
{
  startExtendMan();
  while (digitalRead(EXTEND_LIMIT_PIN))
  {
  }
  stopExtendMan();
  extEnc.write(0);
}


/*
   Function manElevateToZero
   Desc     Send manipulator Z axis to zero(to limiter)
   Input    none
   Output   none
*/
void manElevateToZero(void)
{
  startDescendMan();
  while (digitalRead(ELEVATOR_LIMIT_PIN))
  {
    //Serial.println(elevatorEnc.read());
  }
  stopDescendMan();
}


/*
   Function manElevateTo
   Desc     Send manipulator Z axis to N
   Input    unsigned byte _pos: what pos elevate to
   Output   none
*/
void manElevateTo(int16_t _pos)
{
  int _startImps = elevatorEnc.read();
  Serial.println(elevatorEnc.read());
  if (_pos > 0)
  {
    startElevateMan();
    Serial.println("elevating!");
    while (elevatorEnc.read() < _pos + _startImps
           && elevatorEnc.read() < ELEVATE_TOP_IMPS)
    {
      Serial.println(elevatorEnc.read());
    }
    stopElevateMan();
  }
  else
  {
    startDescendMan();
    Serial.println("descending!");
    while (elevatorEnc.read() > _pos + _startImps
           && digitalRead(ELEVATOR_LIMIT_PIN))
    {
      Serial.println(elevatorEnc.read());
    }
    stopDescendMan();
    if (!digitalRead(ELEVATOR_LIMIT_PIN))
    {
      elevatorEnc.write(0);
    }
  }
}


/*
   Function manElevateTo_mm
   Desc     Elevate manipulator for some millimeters
   Input    int _mm: what millimeters elevate to
   Outpur   none
*/
inline void manElevateTo_mm(int _mm)
{
  manElevateTo(_mm * ELEVATE_IMPS_PER_MM);
}


/*
   Function manTurnTo
   Desc     Turn manipulator on N degrees, range from -315 to +315
   Input    int16_t _degrees: num of degrees turn to
   Output   none
*/
void manTurnTo(int _imps)
{
  long int _startImps = turnManEnc.read();
  if (_imps > 0)
  {
    //turn ACW
    startTurnManACW();
    while (turnManEnc.read() < _imps + _startImps) //TODO: check top of ranging
    {
      Serial.println(turnManEnc.read());
    }
    stopTurnManACW();
  }
  else
  {
    //turn CW
    startTurnManCW();
    while (turnManEnc.read() > _imps + _startImps)
    {
      Serial.println(turnManEnc.read());
    }
    stopTurnManCW();
  }
}


/*
   Function manTurnTo_mm
   Desc     Turn manipulator for some degrees
   Input    int _degrees: what num of degrees turn to, (-380;+380)//TODO: change range
   Output   none
*/
inline void manTurnTo_deg(int _degrees)
{
  manTurnTo(_degrees * TURN_IMPS_PER_DEGREE);
}


/*
   Function manTurnToZero
   Desc     Turn manipulator to ZERO
   Input    none
   Output   none
*/
void manTurnToZero(void)
{
  //turn to limiter
  startTurnManCW();
  while (digitalRead(TURN_MAN_LIMIT_PIN)) {}
  stopTurnManCW();
}


/*
   Function manGrabOpen
   Desc     Open grabber on manipulator
   Input    none
   Output   none
*/
void manGrabOpen(void)
{
  if(manGrabStatus == CLOSED)
  {
    startExtendGrabber();
    delay(GRAB_OPEN_DELAY);
    stopExtendGrabber();
    manGrabStatus = OPENED;
  }
}



/*
   Function manGrabClose
   Desc     Close grabber on manipulator
   Input    none
   Output   none
*/
void manGrabClose(void)
{
  startRetractGrabber();
  while(digitalRead(GRABBER_LIMIT_PIN)){}
  stopRetractGrabber();
  manGrabStatus = CLOSED;
}
