
float getAngle(int id) {
  int preMap = 0;
  int from = 0;
  int to = 0;

  float mult = 1;

  if (id == 0) {
    to = 613;
    from = 1020;
    mult = 1.083;
  }

  if (id == 1) {
    from = 330;
    to = 13;
  }

  if (id == 2) {
    from = 640;
    to = 1020;
    mult = 1.083;
  }

  if (id == 3) {
    to = 400;
    from = 0;
  }

  return map(analogRead(angle_sensors[id]), from, to, 0, 240) * mult;
}

