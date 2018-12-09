//TODO: cleanup
int seed[6][4] = {
  {F, F, B, B}, //forward
  {B, B, F, F}, //backward
  {F, F, F, F}, //turn left
  {B, B, F, F}, //turn right
  {F, F, F, F},
  {B, B, B, B}
};


//void init() {
//  for (int i = 0; i < 4; i++) {
//    pinMode(s[i], OUTPUT);
//    pinMode(d[i], OUTPUT);
//    pinMode(e[i], OUTPUT);
//  }
//}

int lock[4] = {ForwardAngle, ForwardAngle, ForwardAngle, ForwardAngle};
int curAngle[4] = {getAngle(0), getAngle(1), getAngle(2), getAngle(3)};


void turn(int side) {
  //if ((side && turned != -1) || (!side && turned != 1)) {
  mov(side, 100);

  // }
}

void goTo(int angle, int dist) {
  int r1 = sR;
  int r2 = sL;
  if (angle < 0) {
    r1 = sL;
    r2 = sR;
  }
  angle = abs(angle);
  //  angle = radians(angle);

  int a = dist / tan(radians(90 - angle));
  int d1 = a / cos(radians(45));
  int d2 = a * tan(radians(45));

  int d = dist - d2;
  //  int c = b/sin(radians(45));

  Serial.println((String)d1 + "\t" + (String)d);
  Serial1.print("rb(-1)");
  while(Serial1.read() != 100){}

  turn4(45, r1);
  mov(F, d1 * 1.45);
  turn4(45, r2);
  mov(F, d);
}

void setCenter() {
  while (digitalRead(centerBut) == 1) {
    if (analogRead(angle_sensors[0]) > 1020) {
      goR();
      Serial.println("Move Right");
    } else if (analogRead(angle_sensors[0]) < 1020) {
      goL();
      Serial.println("Move Left");
    }
  }
  digitalWrite(motorB, LOW); //STOP
  digitalWrite(motorA, HIGH);
}

void goR() {
  int pack2[4] = {0, 0, 1, 1};
  movs(R, 1, pack2);
  digitalWrite(motorB, LOW); //RIGHT
  digitalWrite(motorA, LOW);
}

void goL() {
  int pack2[4] = {0, 0, 1, 1};
  movs(L, 1, pack2);
  digitalWrite(motorB, HIGH); //LEFT
  digitalWrite(motorA, HIGH);
}

void setA(int a) {
  float del = abs((111.1 * (a)) - 1000);
  Serial.println(del);
  if (a < 0) {
    goR();
  } else {
    goL();
  }
  delay(del);
  digitalWrite(motorA, HIGH);
  digitalWrite(motorB, LOW);
}


void turn4(int angle, int side) {
  float dist = Len / (360 / angle);
  dist *= 3;
  dist *= 0.7254;
  dist *= 0.76923;
  dist *= 0.666;
  Serial.println(Len);
  mov(side, dist);
}

void turns(int dist, int side) {
  //if ((side && turned != -1) || (!side && turned != 1)) {
  dist *= 0.2857;
  int pack[4] = {1, 1, 0, 0};
  movs(side, dist * 1.3, pack);
  delay(300);
  int pack2[4] = {0, 0, 1, 1};
  movs(side, dist * 0.8, pack2);

  // }
}

void setSpeed(int ns[4]) {
  speeds[0] = ns[0];
}

void mov_ticks(int dir, long int dist) {
  for (int i = 0; i < dist; i++) {
    for (int j = 0; j < 4; j++) {
      int del = speeds[j];
      digitalWrite(s[j], HIGH);
      delayMicroseconds(del);
      digitalWrite(s[j], LOW);
      delayMicroseconds(del);
    }
  }
}

void mov(int dir, long int dist) {
  d[2] = 12;
  set(seed[dir]);

  dist = (dist / len) * stepsInRotate * devide;
  dist *= 2.06492;
  dist *= 0.81;
  dist /= 5;

  for (int i = 0; i < dist; i++) {
    for (int j = 0; j < 4; j++) {
      int del = map(speeds[j], 0, 100, 1200, 300);
      digitalWrite(s[j], HIGH);
      delayMicroseconds(del);
      digitalWrite(s[j], LOW);
      delayMicroseconds(del);
    }
  }
}



void setAngle(int motor, int angle) {
  int prevSpeed[4] = {speeds[0], speeds[1], speeds[2], speeds[3]};
  int nspeed[4] = {40, 40, 40, 40};
  setSpeed(nspeed);
  int side = F;
  if (curAngle[motor] != angle) {

    int set[4] = {0, 0, 0, 0};
    set[motor] = 1;
    int prevAngle = getAngle(motor);
    while (abs(getAngle(motor) - angle) >= 2) {
      if (angle - getAngle(motor) < 0) {
        side = F;
      } else {
        side = B;
      }
      movs( motor == 2 || motor == 0 ? (side == B ? F : B) : side, 1, set);
      if (getAngle(motor) == angle) {
        //Serial.println("GOAL");
        break;
      }
      prevAngle = getAngle(motor);
      Serial.println("GOAL: " + (String)angle + "\tTo goal: " + (String)(angle - prevAngle) + "\tSide: " + (String)side);
    }
  }
  curAngle[motor] = getAngle(motor);
  setSpeed(prevSpeed);
}

void lookForward() {
  setAngle(0, ForwardAngle[0]);
  setAngle(1, ForwardAngle[1]);
  setAngle(2, ForwardAngle[2]);
  setAngle(3, ForwardAngle[3]);
}


void lockAngle(int motor, int angle) {
  lock[motor] = angle;
}

void movs(int dir, long int dist, int st[]) {
  set(seed[dir]);

  dist = (dist / len) * stepsInRotate * devide;
  dist *= 2.06492;

  for (int i = 0; i < dist; i++) {
    for (int j = 0; j < 4; j++) {
      int del = map(speeds[j], 0, 100, 1200, 100);
      if (st[j] == 1) {
        digitalWrite(s[j], HIGH);
        delayMicroseconds(900);
        digitalWrite(s[j], LOW);
        delayMicroseconds(900);
      }
    }
  }
}

void set(int in[]) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(d[i], in[i]);
    if (i == 2) {
      digitalWrite(42, in[i]);
    }
  }
}
