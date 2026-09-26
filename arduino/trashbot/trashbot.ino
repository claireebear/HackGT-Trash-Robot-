// TrashBot Arduino firmware: TB6612 motors, HC-SR04 ultrasonic, dump servo,
// and a sensor that detects trash in the compartment.
//
// Serial protocol (115200 baud, newline-terminated), see robot.py:
//   Pi -> Arduino:  "M <left> <right>"  (-255..255)   "S" stop   "T" throw
//   Arduino -> Pi:  "D <cm> L <0|1>"  every 50 ms
//
// Safety: motors stop if no command arrives for 500 ms (e.g. the Pi crashes).

#include <Servo.h>

// ---- TB6612 (motor A = left, motor B = right) ----
const int PWMA = 5, AIN1 = 7, AIN2 = 8;
const int PWMB = 6, BIN1 = 9, BIN2 = 10;  // 9/10 used as digital only (Servo lib takes their PWM)
const int STBY = 4;

// ---- sensors / actuators ----
const int TRIG = 11, ECHO = 12;
const int SERVO_PIN = 3;
const int TRASH_SENSOR = A0;       // IR obstacle sensor aimed into the compartment
const bool TRASH_ACTIVE_LOW = true; // most IR modules pull LOW when they see something

const int SERVO_REST = 0, SERVO_DUMP = 120;
const unsigned long WATCHDOG_MS = 500, REPORT_MS = 50;

// Flip these if a wheel spins the wrong way.
const bool INVERT_LEFT = false, INVERT_RIGHT = false;

Servo dumpServo;
unsigned long lastCommand = 0, lastReport = 0;
String line;

void setMotor(int pwmPin, int in1, int in2, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
  analogWrite(pwmPin, abs(speed));
}

void drive(int left, int right) {
  setMotor(PWMA, AIN1, AIN2, left, INVERT_LEFT);
  setMotor(PWMB, BIN1, BIN2, right, INVERT_RIGHT);
}

long readDistanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 25000);  // ~4 m max; 0 = no echo
  return us / 58;
}

bool trashPresent() {
  bool v = digitalRead(TRASH_SENSOR);
  return TRASH_ACTIVE_LOW ? !v : v;
}

void throwTrash() {
  drive(0, 0);
  for (int a = SERVO_REST; a <= SERVO_DUMP; a += 2) { dumpServo.write(a); delay(15); }
  delay(800);  // let the trash fall out
  for (int a = SERVO_DUMP; a >= SERVO_REST; a -= 2) { dumpServo.write(a); delay(15); }
}

void handle(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  lastCommand = millis();
  char c = cmd.charAt(0);
  if (c == 'M') {
    int sp = cmd.indexOf(' ', 2);
    if (sp < 0) return;
    drive(cmd.substring(2, sp).toInt(), cmd.substring(sp + 1).toInt());
  } else if (c == 'S') {
    drive(0, 0);
  } else if (c == 'T') {
    throwTrash();
    lastCommand = millis();
  }
}

void setup() {
  Serial.begin(115200);
  int outs[] = {PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY, TRIG};
  for (int p : outs) pinMode(p, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRASH_SENSOR, INPUT_PULLUP);
  digitalWrite(STBY, HIGH);
  dumpServo.attach(SERVO_PIN);
  dumpServo.write(SERVO_REST);
  drive(0, 0);
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { handle(line); line = ""; }
    else if (line.length() < 32) line += ch;
  }

  if (millis() - lastCommand > WATCHDOG_MS) drive(0, 0);

  if (millis() - lastReport > REPORT_MS) {
    lastReport = millis();
    Serial.print("D ");
    Serial.print(readDistanceCm());
    Serial.print(" L ");
    Serial.println(trashPresent() ? 1 : 0);
  }
}
