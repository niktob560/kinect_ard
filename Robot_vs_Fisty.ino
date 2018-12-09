#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Encoder.h>


//TODO: remove
LiquidCrystal lcd(31, 30, 34, 35, 33, 32);

//TODO: switch softserial to hardware serial (Serial2(TX2;RX2), Serial3(TX3;RX3), etc)
SoftwareSerial mySerial(65, 64);

const int SwitchMode = A15;




//              MANIPULATOR MOTOR PINS
#define TURN_MAN_SPEED_PIN        (44)  //turn
#define TURN_MAN_ACW_PIN          (43)
#define TURN_MAN_CW_PIN           (42)
#define EXT_MAN_PIN               (A1)  //extend-retract
#define RETRACT_MAN_PIN           (A0)
#define ELEVATE_MAN_PIN           (A2)  //elevate-descend
#define DESCEND_MAN_PIN           (A3)
#define GRABBER_EXTEND_PIN        (52)
#define GRABBER_RETRACT_PIN       (53)
//              MOTORS PINS
#define LEFT_MOTOR_SPD_PIN        (5)   //left motor
#define LEFT_MOTOR_DIR_PIN        (6)
#define LEFT_MOTOR_EN_PIN         (7)
#define RIGHT_MOTOR_SPD_PIN       (9)   //right motor
#define RIGHT_MOTOR_DIR_PIN       (10)
#define RIGHT_MOTOR_EN_PIN        (11)
//              LIMITERS
#define GRABBER_LIMIT_PIN         (24)  //grabber
#define TURN_MAN_LIMIT_PIN        (25)  //turn
#define ELEVATOR_LIMIT_PIN        (22)  //elevate
#define EXTEND_LIMIT_PIN          (23)  //extend
//              ENCODER PINS
#define EXTEND_ENCODER_A          (18)  //extend
#define EXTEND_ENCODER_B          (19)
#define EXTEND_ENCODER_C          (16)
#define ELEVATOR_ENCODER_A        (20)  //elevate
#define ELEVATOR_ENCODER_B        (21)
#define TURN_MAN_ENCODER_A        (2)   //turn
#define TURN_MAN_ENCODER_B        (3)
//              CONVERT IMPS TO METRIC SYSTEM
#define TURN_IMPS_PER_DEGREE      (14.4)
#define ELEVATE_IMPS_PER_MM       (24)
#define EXT_IMPS_PER_MM           (58)
//              RANGES
#define ELEVATE_TOP_MM            (170)
#define ELEVATE_TOP_IMPS          (ELEVATE_TOP_MM * ELEVATE_IMPS_PER_MM)
#define EXT_MAX_RANGE_IMPS        (999999)


#define MICROS_IN_MILLIS          (pow(10, 3))
#define MICROS_IN_S               (pow(10, 6))

//              STATUS
#define FORWARD                   (1)
#define BACKWARD                  (0)
#define OPENED                    (1)
#define CLOSED                    (0)

//              DELAYS
#define DELAY_MICROS              (200)
#define DELAY                     (200)
#define GRAB_OPEN_DELAY           (3500)


#define DEFAULT_BAUD              (9600)


//              RADIO CMD
#define RD_MAN_EXT_START          (25)
#define RD_MAN_EXT_STOP           (26)
#define RD_MAN_RET_START          (27)
#define RD_MAN_RET_STOP           (28)
#define RD_MAN_TURN_CW_START      (7)
#define RD_MAN_TURN_CW_STOP       (8)
#define RD_MAN_TURN_ACW_START     (5)
#define RD_MAN_TURN_ACW_STOP      (6)
#define RD_MAN_ELEVATE_START      (9)
#define RD_MAN_ELEVATE_STOP       (10)
#define RD_MAN_DESCEND_START      (11)
#define RD_MAN_DESCEND_STOP       (12)
#define RD_MAN_GRAB_OPEN_START    (17)
#define RD_MAN_GRAB_OPEN_STOP     (18)
#define RD_MAN_GRAB_CLOSE_START   (19)
#define RD_MAN_GRAB_CLOSE_STOP    (20)
#define RD_LW_FORWARD_START       (0)
#define RD_LW_FORWARD_STOP        (0)
#define RD_LW_BACKWARD_START      (0)
#define RD_LW_BACKWARD_STOP       (0)
#define RD_RW_FORWARD_START       (0)
#define RD_RW_FORWARD_STOP        (0)
#define RD_RW_BACKWARD_START      (0)
#define RD_RW_BACKWARD_STOP       (0)


Encoder extEnc(EXTEND_ENCODER_A, EXTEND_ENCODER_B);
Encoder elevatorEnc(ELEVATOR_ENCODER_B, ELEVATOR_ENCODER_A);
Encoder turnManEnc(TURN_MAN_ENCODER_A, TURN_MAN_ENCODER_B);


uint8_t manTurnSpeed = 100;     //what speed will use to turn manipulator
bool manGrabStatus = OPENED;



//TODO: cleanup
#define st1 2
#define st2 54
#define st3 8
#define st4 11
#define dir1 3
#define dir2 6
#define dir3 9
#define dir4 12
#define en1 4
const int F = 0;
const int B = 1;
const int L = 2;
const int R = 3;
const int sL = 4;
const int sR = 5;
const int motorA = 10;
const int motorB = 11;
const int centerBut = 12;
const int ForwardAngle[4] = {60, 227, 207, 30};
const int SideAngle = 143;

//int s[4] = {2, 4, 10, 8};
//int d[4] = {3, 5, 7, 9};

int s[4] = {2, 5, 11, 8};
int d[4] = {3, 6, 12, 9}; //2, 3
//6 7 10 11 12
int angle_sensors[] = {A0, A1, A2, A3};
/*
	0 -> 158
	269 -> 392
	0 -> 170
	0 -> 148
*/

const float devide = 38 / 17;
const float stepsInRotate = 200;
const float diametr = 26.5; //mm
const float len = PI * diametr; //mm

const float Len = PI * 370;

int speeds[4] = {50, 50, 50, 50};

void setup2()
{
  //Serial1.begin(DEFAULT_BAUD);
  //Serial1.setTimeout(150);
  mySerial.begin(DEFAULT_BAUD);
  //pinMode(SwitchMode, INPUT_PULLUP);
}

void setup()
{
  //  upgreat();
  delay(100);
  Serial.begin(DEFAULT_BAUD);
  Serial.setTimeout(150);

  for (int i = 0; i <= 53; i++)
  {
    pinMode(i, OUTPUT);
  }
  for (int i = A0; i <= A10; i++)
  {
    pinMode(i, OUTPUT);
  }
  pinMode(GRABBER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(EXTEND_LIMIT_PIN, INPUT_PULLUP);
  pinMode(EXTEND_ENCODER_A, INPUT_PULLUP);
  pinMode(EXTEND_ENCODER_B, INPUT_PULLUP);
  pinMode(EXTEND_ENCODER_C, INPUT_PULLUP);
  pinMode(ELEVATOR_LIMIT_PIN, INPUT_PULLUP);
  pinMode(ELEVATOR_ENCODER_A, INPUT_PULLUP);
  pinMode(ELEVATOR_ENCODER_B, INPUT_PULLUP);
  pinMode(TURN_MAN_ENCODER_A, INPUT_PULLUP);
  pinMode(TURN_MAN_ENCODER_B, INPUT_PULLUP);
  pinMode(TURN_MAN_LIMIT_PIN, INPUT_PULLUP);
  analogWrite(TURN_MAN_SPEED_PIN, manTurnSpeed);
  setup2();
  //TODO: cleanup
  /*for (int i = 0; i < 4; i++) {
    pinMode(s[i], OUTPUT);
    pinMode(d[i], OUTPUT);
    pinMode(angle_sensors[i], INPUT);
    }
    for (int i = 0; i < 30; i++) {
    pinMode(i, OUTPUT);
    }

    pinMode(motorA, OUTPUT);
    pinMode(motorB, OUTPUT);
    pinMode(centerBut, INPUT_PULLUP);
    //init();

    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
  */
  //setup2();
  //lcd.begin(8, 2);
}

int prevMode = -1;

void loop()
{

  // Serial1.write(9);
  // delay(1000);
  // Serial1.write(10);
  // delay(1000);

  int mode = 0;//digitalRead(SwitchMode);
  if (mode == 1)
  {
    //printD(mode, "Auto", "Mode");
    loop1();
  }
  else
  {
    //printD(mode, "Hand", "Mode");
    loop2();
  }
  //prevMode = mode;
}

//TODO: remove
void printD(int m, String a, String b)
{
  if (m != prevMode)
  {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print(a);

    lcd.setCursor(2, 1);
    lcd.print(b);
  }
}

void loop2() 
{
  while (mySerial.available())
  {
    //Serial.println("FUCK");
    int in = mySerial.read();
    if (in != "")
    {
      Serial.println(in);
      if (in == 0)
      {
        
      }
      //        EXTEND
      else if (in == RD_MAN_EXT_START)
      {
        startExtendMan();
      }
      else if (in == RD_MAN_EXT_STOP)
      {
        stopExtendMan();
      }
      //        RETRACT
      else if (in == RD_MAN_RET_START)
      {
        startRetractMan();
      }
      else if (in == RD_MAN_RET_STOP)
      {
        stopRetractMan();
      }
      //        TURN CW
      else if (in == RD_MAN_TURN_CW_START)
      {
        startTurnManCW();
      }
      else if (in == RD_MAN_TURN_CW_STOP)
      {
        stopTurnManCW();
      }
      //        TURN ACW
      else if (in == RD_MAN_TURN_ACW_START)
      {
        startTurnManACW();
      }
      else if (in == RD_MAN_TURN_ACW_STOP)
      {
        stopTurnManACW();
      }
      //        ELEVATE
      else if (in == RD_MAN_ELEVATE_START)
      {
        startElevateMan();
      }
      else if (in == RD_MAN_ELEVATE_STOP)
      {
        stopElevateMan();
      }
      //        DESCEND
      else if (in == RD_MAN_DESCEND_START)
      {
        startDescendMan();
      }
      else if (in == RD_MAN_DESCEND_STOP)
      {
        stopDescendMan();
      }
      else if (in == RD_MAN_GRAB_OPEN_START)
      {
        startExtendGrabber();
      }
      else if (in == RD_MAN_GRAB_OPEN_STOP)
      {
        stopExtendGrabber();
      }
      else if (in == RD_MAN_GRAB_CLOSE_START)
      {
        startRetractGrabber();
      }
      else if (in == RD_MAN_GRAB_CLOSE_STOP)
      {
        stopRetractGrabber();
      }
      else
      {
        
      }
      //check extend limiters
      if(!digitalRead(EXTEND_LIMIT_PIN))
      {
        stopExtendMan();
        extEnc.write(0);
      }
      if(extEnc.read() >= EXT_MAX_RANGE_IMPS)
      {
        stopRetractMan();
      }
      //check elevate limiters
      if(!digitalRead(ELEVATOR_LIMIT_PIN))
      {
        stopDescendMan();
        elevatorEnc.write(0);
      }
      if(elevatorEnc.read() >= ELEVATE_TOP_IMPS)
      {
        stopElevateMan();
      }
      //check grab limiters
      if(!digitalRead(GRABBER_LIMIT_PIN))
      {
        stopRetractGrabber();z
      }
    }
  }
}

//TODO: cleanup
void loop1()
{
  //  upgreat();
  while (Serial.available() > 0)
  {
    String in = String(Serial.readString());
    if (in != "")
    {
      String cmd = split(in, '(', 0);
      String params = split(split(in, '(', 1), ')', 0);
      Serial.println(in);
      if (cmd == "f")
      {
        //drive forward
        int imps = split(params, ",", 0).toInt();
        /*
          setRightMotorDir(FORWARD);
          setLeftMotorDir(FORWARD);//TODO: refactor
          turnBothMotors_imps(imps);
        */
        driveForward(imps);
      }
      else if (cmd == "b")
      {
        //drive backward
        int imps = split(params, ",", 0).toInt();
        /*
          setRightMotorDir(BACKWARD);
          setLeftMotorDir(BACKWARD);//TODO: refactor
          turnBothMotors_imps(imps);
        */
        driveBackward(imps);
      }
      else if (cmd == "r")
      {
        //drive right
        int imps = split(params, ",", 0).toInt();
        /*
          setRightMotorDir(BACKWARD);
          setLeftMotorDir(FORWARD);//TODO: refactor
          turnBothMotors_imps(imps);
        */
        driveRight(imps);
      }
      else if (cmd == "l")
      {
        //drive left
        int imps = split(params, ",", 0).toInt();
        /*
          setRightMotorDir(FORWARD);
          setLeftMotorDir(BACKWARD);//TODO: refactor
          turnBothMotors_imps(imps);
        */
        driveLeft(imps);
      }
      else if (cmd == "MNT")
      {
        //manipulator turn (-315:315)
        int degree = split(params, ",", 0).toInt();
        Serial.println(degree);
        //manTurnTo(degree * TURN_IMPS_PER_DEG);
        manTurnTo_deg(degree);
      }
      else if (cmd == "MNSetSpeed")
      {
        //set speed of manipulator turn(0%:100%)
        manTurnSpeed = split(params, ",", 0).toInt();
        analogWrite(TURN_MAN_SPEED_PIN, manTurnSpeed);
      }
      else if (cmd == "MNTzero")
      {
        //manipulator turn to ZERO
        manTurnToZero();
      }
      else if (cmd == "MNZ")
      {
        //elevate manipulator, mm (0:100)
        int pos = split(params, ",", 0).toInt();
        //Serial.println(pos);
        //manElevateTo(pos * ELEVATE_IMPS_PER_MM);
        manElevateTo_mm(pos);
      }
      else if (cmd == "MNZzero")
      {
        //elevate manipulator to ZERO
        manElevateToZero();
      }
      else if (cmd == "MNX")
      {
        //extend/retract manipulator, mm (0: 250)
        int pos = split(params, ",", 0).toInt();
        //manExtendTo(pos * EXT_IMPS_PER_MM);//TODO: refactor
        manExtendTo_mm(pos);
      }
      else if (cmd == "MNXzero")
      {
        //extend/retract manipulator to ZERO
        manExtendToZero();
      }
      else if (cmd == "MNzero")//not tested
      {
        //set all axis of manipulator to ZERO(strange algorithm: call MNTzero(); call MNXzero(); call MNX(150); call MNZzero();)
        manTurnToZero();
        manExtendToZero();
        manExtendTo_mm(150);//150 mm
        manElevateToZero();
      }
      else if (cmd == "MNG")
      {
        int param = split(params, ",", 0).toInt();
        if(param)
          manGrabOpen();
        else
          manGrabClose();
      }
      //TODO: add grabber

      //Serial.write("!");
      //Serial.flush();
    }
  }
}


//TODO: cleanup
void stepper(int na)
{

  if (na == 3)
  {

    digitalWrite(dir3, ! HIGH); digitalWrite(dir2, !LOW);
    digitalWrite(dir1, !

                 LOW); digitalWrite(dir4, !HIGH);

  } else if (na == 1)
  {
    digitalWrite(dir3, !!HIGH); digitalWrite(dir2, !!LOW);
    digitalWrite(dir1, !!LOW); digitalWrite(dir4, !!HIGH);

  } else if (na == 21)
  {
    digitalWrite(dir3, !HIGH); digitalWrite(dir2, LOW);
    digitalWrite(dir1, LOW); digitalWrite(dir4, !HIGH);

  } else if (na == 23)
  {
    digitalWrite(dir3, HIGH); digitalWrite(dir2, !LOW);
    digitalWrite(dir1, !LOW); digitalWrite(dir4, HIGH);

  }
  int tim = 15;
  while (mySerial.read() != na + 1)
  {
    digitalWrite(st1, HIGH);
    digitalWrite(st2, HIGH);
    digitalWrite(st3, HIGH);
    digitalWrite(st4, HIGH);
    delay(tim / 2);
    digitalWrite(st1, LOW);
    digitalWrite(st2, LOW);
    digitalWrite(st3, LOW);
    digitalWrite(st4, LOW);
    delay(tim / 2);
  }
  //  upgreat();
}

//void upgreat()
//{
//  digitalWrite(en1, HIGH);
//  delay(1000);
//  digitalWrite(en1, LOW);
//}
