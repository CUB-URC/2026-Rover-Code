// Reference motor bridge for an Arduino/Teensy-class board.
// Receives "M,<left>,<right>,<enabled>,<led>\n" from the rover computer.
//
// Independent watchdog: if no valid line arrives for 250 ms (computer crashed,
// USB unplugged, Python hung), the motors stop. Never rely on one layer only.
//
// PLACEHOLDER OUTPUTS: replace setMotor()/setLed() with whatever your
// electrical team chooses (PWM to ESCs, CAN to SparkMAX/ODrive, etc.).

const unsigned long WATCHDOG_MS = 250;
unsigned long lastValidMs = 0;
char buf[48];
uint8_t len = 0;

void setMotor(int side, float speed) {
  // TODO: e.g. RC-style ESC: writeMicroseconds(1500 + speed * 500)
  (void)side; (void)speed;
}

void setLed(int state) {
  // 0=off 1=red(autonomous) 2=blue(teleop) 3=flashing green(arrived)
  (void)state;
}

void stopAll() { setMotor(0, 0); setMotor(1, 0); }

void handleLine(char* line) {
  float l, r; int en, led;
  if (sscanf(line, "M,%f,%f,%d,%d", &l, &r, &en, &led) != 4) return;  // note: some AVR cores lack %f; parse with strtod if needed
  l = constrain(l, -1.0f, 1.0f);
  r = constrain(r, -1.0f, 1.0f);
  if (en) { setMotor(0, l); setMotor(1, r); } else { stopAll(); }
  setLed(led);
  lastValidMs = millis();
}

void setup() {
  Serial.begin(115200);
  stopAll();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { buf[len] = '\0'; handleLine(buf); len = 0; }
    else if (len < sizeof(buf) - 1) { buf[len++] = c; }
    else { len = 0; }  // overflow: drop the line
  }
  if (millis() - lastValidMs > WATCHDOG_MS) stopAll();
}
