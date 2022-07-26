#define LEFT_JOY_X        A0 // analog pin connected to X output
#define LEFT_JOY_Y        A1 // analog pin connected to Y output
#define LEFT_JOY_SWITCH   6  // digital pin connected to switch output

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_JOY_SWITCH, INPUT_PULLUP);
}

void loop() {
  int previous_raw = 0;
  int previous_calibrated = 0;
  int raw_throttle = analogRead(LEFT_JOY_X);
  int calibrated_throttle = mapThrottleJoystickValue(raw_throttle, 23, 508, 1005, true);

  if (different(previous_raw, raw_throttle) || different(previous_calibrated, calibrated_throttle)) {
    Serial.print("Raw:"); 
    Serial.print(raw_throttle);
    Serial.print("\t");
    Serial.print("Calibrated:"); 
    Serial.println(calibrated_throttle);
    previous_raw = raw_throttle;
    previous_calibrated = calibrated_throttle;
  }

  delay(100);
}

bool different(int i1, int i2) {
  return abs(i1 - i2) > 10;
}

int mapJoystickValue(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 512);
  else
    val = map(val, middle, upper, 512, 1023);
  return ( reverse ? 1023 - val : val );
}

int mapThrottleJoystickValue(int val, int lower, int middle, int upper, bool reverse) {
  val = mapJoystickValue(val, lower, middle, upper, reverse);
  if (val < 512)
    return 0;
  else
    return map(val, 512, 1023, 0, 1023);
}
