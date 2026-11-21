/*
  Smart Parking Gate - Arduino Uno

  - Parking assist: an ultrasonic sensor in front of the gate measures the distance to the
    car. The buzzer beeps faster as the car gets closer (continuous tone under 20 cm) and a
    green / yellow / red LED bar shows the distance.
  - Entry: a car that waits at the gate for 1 s opens the barrier (servo), but only if
    there is a free space. When the car park is full the barrier stays closed.
  - Safety: a second ultrasonic sensor watches the space under the barrier. The barrier
    never closes on a car: if something appears while it is closing, it goes back up.
  - Counting: a car is counted only after it has really passed under the barrier.
    The number of occupied spaces is kept in EEPROM, so it survives a power cut.
  - Buttons: EXIT (car leaving), MANUAL (attendant opens the gate, not counted),
    RESET (hold 2 s: car park empty).
  - 16x2 I2C LCD with free spaces, gate state and messages; event log on Serial (115200).

  Everything is non-blocking (millis), except the ultrasonic pulse itself (max 25 ms).
*/
#include <Arduino.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>

// ---------------------------------------------------------------- types
// Declared before any function: the Arduino builder inserts function prototypes
// above the first function, and they need these types.
enum GateState : uint8_t { CLOSED, OPENING, OPEN, CLOSING };
enum Purpose : uint8_t { NONE, ENTRY, EXIT, MANUAL };
const char* const STATE_TEXT[] = {"SHUT", "UP", "OPEN", "DOWN"};  // fits the 16-char LCD line

// ---------------------------------------------------------------- pins
const uint8_t PIN_APPROACH_TRIG = 2;
const uint8_t PIN_APPROACH_ECHO = 3;
const uint8_t PIN_GATE_TRIG = 4;
const uint8_t PIN_GATE_ECHO = 5;
const uint8_t PIN_BTN_EXIT = 6;
const uint8_t PIN_BTN_MANUAL = 7;
const uint8_t PIN_BUZZER = 8;
const uint8_t PIN_SERVO = 9;
const uint8_t PIN_LED_GREEN = 10;
const uint8_t PIN_LED_YELLOW = 11;
const uint8_t PIN_LED_RED = 12;
const uint8_t PIN_LAMP_RED = A1;    // traffic light at the gate
const uint8_t PIN_LAMP_GREEN = A2;
const uint8_t PIN_BTN_RESET = A3;

// ---------------------------------------------------------------- settings
const uint8_t CAPACITY = 10;
const uint16_t ASSIST_RANGE_CM = 150;   // parking assist starts beeping
const uint16_t STOP_CM = 20;            // continuous tone: stop!
const uint16_t REQUEST_CM = 40;         // car waiting at the gate
const uint32_t REQUEST_HOLD_MS = 1000;
const uint16_t UNDER_GATE_CM = 50;      // something under the barrier
const uint32_t PASSED_CLEAR_MS = 800;   // under-gate sensor clear this long = car has passed
const uint32_t OPEN_TIMEOUT_MS = 10000; // nobody drove through: close again
const uint8_t ANGLE_CLOSED = 0;
const uint8_t ANGLE_OPEN = 90;
const uint16_t SERVO_STEP_MS = 12;      // 1 degree per step: ~1.1 s for a full stroke
const uint8_t SENSOR_FAULT_COUNT = 5;   // missing echoes in a row before a sensor fault
const uint32_t RESET_HOLD_MS = 2000;
const uint32_t MESSAGE_MS = 3000;
const int EEPROM_ADDR = 0;
const uint8_t EEPROM_MAGIC = 0xA7;

// ---------------------------------------------------------------- helpers
uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

void logEvent(const __FlashStringHelper* text, int value = -1) {
  Serial.print('[');
  Serial.print(millis());
  Serial.print(F(" ms] "));
  Serial.print(text);
  if (value >= 0) {
    Serial.print(' ');
    Serial.print(value);
  }
  Serial.println();
}

// HC-SR04 with a median-of-3 filter and fault detection (no echo several times in a row)
class Ultrasonic {
 public:
  Ultrasonic(uint8_t trig, uint8_t echo) : trig_(trig), echo_(echo) {}
  void begin() {
    pinMode(trig_, OUTPUT);
    pinMode(echo_, INPUT);
    digitalWrite(trig_, LOW);
  }
  void measure() {
    digitalWrite(trig_, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_, LOW);
    unsigned long us = pulseIn(echo_, HIGH, 25000UL);  // ~4 m max
    if (us == 0) {
      if (misses_ < 255) misses_++;
      fault = misses_ >= SENSOR_FAULT_COUNT;
      return;
    }
    misses_ = 0;
    fault = false;
    samples_[idx_] = (uint16_t)(us / 58UL);
    idx_ = (idx_ + 1) % 3;
    cm = median3(samples_[0], samples_[1], samples_[2]);
  }
  uint16_t cm = 999;
  bool fault = false;

 private:
  uint8_t trig_, echo_;
  uint16_t samples_[3] = {999, 999, 999};
  uint8_t idx_ = 0;
  uint8_t misses_ = 0;
};

// Debounced push button (to GND, internal pull-up) with press edge and long-press
class Button {
 public:
  explicit Button(uint8_t pin) : pin_(pin) {}
  void begin() { pinMode(pin_, INPUT_PULLUP); }
  void update(uint32_t now) {
    bool raw = digitalRead(pin_) == LOW;
    pressed = false;
    if (raw != last_) {
      last_ = raw;
      changedAt_ = now;
    }
    if (now - changedAt_ > 25 && raw != down) {
      down = raw;
      if (down) {
        pressed = true;
        downAt_ = now;
        longDone_ = false;
      }
    }
  }
  bool heldFor(uint32_t now, uint32_t ms) {
    if (down && !longDone_ && now - downAt_ >= ms) {
      longDone_ = true;
      return true;
    }
    return false;
  }
  bool down = false;
  bool pressed = false;

 private:
  uint8_t pin_;
  bool last_ = false;
  uint32_t changedAt_ = 0;
  uint32_t downAt_ = 0;
  bool longDone_ = false;
};

// ---------------------------------------------------------------- state
Ultrasonic approach(PIN_APPROACH_TRIG, PIN_APPROACH_ECHO);
Ultrasonic underGate(PIN_GATE_TRIG, PIN_GATE_ECHO);
Button btnExit(PIN_BTN_EXIT);
Button btnManual(PIN_BTN_MANUAL);
Button btnReset(PIN_BTN_RESET);
Servo barrier;
LiquidCrystal_I2C lcd(0x27, 16, 2);

GateState gate = CLOSED;
Purpose purpose = NONE;
uint8_t angle = ANGLE_CLOSED;
uint8_t occupied = 0;
bool carSeen = false;          // a car has been under the barrier during this opening
bool requestArmed = true;      // the car must leave the gate area before it can request again
uint32_t nearSince = 0;
uint32_t openedAt = 0;
uint32_t clearSince = 0;
uint32_t lastServoStep = 0;
uint32_t lastSensor = 0;
bool sensorTurn = false;
char message[17] = "";
uint32_t messageAt = 0;

// ---------------------------------------------------------------- persistence
void loadCount() {
  if (EEPROM.read(EEPROM_ADDR) == EEPROM_MAGIC) {
    occupied = EEPROM.read(EEPROM_ADDR + 1);
    if (occupied > CAPACITY) occupied = 0;
  } else {
    occupied = 0;
  }
}

void saveCount() {
  EEPROM.update(EEPROM_ADDR, EEPROM_MAGIC);
  EEPROM.update(EEPROM_ADDR + 1, occupied);
}

void showMessage(const char* text) {
  strncpy(message, text, 16);
  message[16] = '\0';
  messageAt = millis();
}

// ---------------------------------------------------------------- gate
void openGate(Purpose why) {
  purpose = why;
  carSeen = false;
  gate = OPENING;
}

void carPassed() {
  if (purpose == ENTRY && occupied < CAPACITY) {
    occupied++;
    saveCount();
    logEvent(F("car entered, occupied"), occupied);
  } else if (purpose == EXIT && occupied > 0) {
    occupied--;
    saveCount();
    logEvent(F("car left, occupied"), occupied);
  } else {
    logEvent(F("car passed (not counted)"));
  }
  purpose = NONE;  // counted once, even if the barrier has to reopen
  gate = CLOSING;
}

void updateGate(uint32_t now) {
  bool somethingUnder = underGate.cm < UNDER_GATE_CM || underGate.fault;  // fault = fail safe

  switch (gate) {
    case CLOSED:
      break;

    case OPENING:
      if (angle >= ANGLE_OPEN) {
        gate = OPEN;
        openedAt = now;
        clearSince = now;
        logEvent(F("gate open"));
      }
      break;

    case OPEN:
      if (somethingUnder) {
        carSeen = true;
        clearSince = now;
      } else if (carSeen && now - clearSince >= PASSED_CLEAR_MS) {
        carPassed();
      } else if (!carSeen && now - openedAt >= OPEN_TIMEOUT_MS) {
        logEvent(F("nobody drove through, closing"));
        showMessage("No car: closing");
        purpose = NONE;
        gate = CLOSING;
      }
      break;

    case CLOSING:
      if (somethingUnder) {
        // safety: never close on a car
        logEvent(F("obstacle under the barrier, reopening"));
        showMessage("Obstacle! Up");
        carSeen = true;
        gate = OPENING;
      } else if (angle <= ANGLE_CLOSED) {
        gate = CLOSED;
        logEvent(F("gate closed"));
      }
      break;
  }

  // smooth servo movement, one degree per step
  if (now - lastServoStep >= SERVO_STEP_MS) {
    lastServoStep = now;
    uint8_t target = (gate == OPENING || gate == OPEN) ? ANGLE_OPEN : ANGLE_CLOSED;
    if (angle < target) angle++;
    else if (angle > target) angle--;
    barrier.write(angle);
  }
}

// ---------------------------------------------------------------- requests
void handleRequests(uint32_t now) {
  // RESET held for 2 s: car park empty
  if (btnReset.heldFor(now, RESET_HOLD_MS)) {
    occupied = 0;
    saveCount();
    showMessage("Count reset");
    logEvent(F("count reset by attendant"));
  }

  if (btnManual.pressed) {
    showMessage("Manual opening");
    logEvent(F("manual opening"));
    openGate(MANUAL);
  }

  if (btnExit.pressed) {
    if (occupied == 0) {
      showMessage("No cars inside");
    } else if (gate == CLOSED || gate == CLOSING) {
      showMessage("Goodbye!");
      logEvent(F("exit request"));
      openGate(EXIT);
    }
  }

  // Entry: a car waiting in front of the gate
  bool atGate = !approach.fault && approach.cm < REQUEST_CM;
  if (!atGate) {
    nearSince = now;
    if (approach.cm > REQUEST_CM + 20) requestArmed = true;
  } else if (requestArmed && now - nearSince >= REQUEST_HOLD_MS && gate == CLOSED) {
    requestArmed = false;
    if (occupied >= CAPACITY) {
      showMessage("FULL - sorry!");
      logEvent(F("entry refused: car park full"));
    } else {
      showMessage("Welcome! Go on");
      logEvent(F("entry request"));
      openGate(ENTRY);
    }
  }
}

// ---------------------------------------------------------------- parking assist
void updateAssist(uint32_t now) {
  uint16_t d = approach.cm;
  bool active = !approach.fault && d <= ASSIST_RANGE_CM && gate == CLOSED;

  digitalWrite(PIN_LED_GREEN, active && d > 100);
  digitalWrite(PIN_LED_YELLOW, active && d > 50 && d <= 100);
  bool redBlink = d <= STOP_CM ? (now / 150) % 2 : true;
  digitalWrite(PIN_LED_RED, active && d <= 50 && redBlink);

  static bool toneOn = false;
  bool beep = false;
  if (active) {
    if (d <= STOP_CM) {
      beep = true;
    } else {
      uint32_t period = map(d, STOP_CM, ASSIST_RANGE_CM, 80, 700);
      beep = (now % period) < period / 2;
    }
  }
  if (beep && !toneOn) tone(PIN_BUZZER, 1800);
  if (!beep && toneOn) noTone(PIN_BUZZER);
  toneOn = beep;
}

// ---------------------------------------------------------------- display
void updateLamps() {
  bool go = gate == OPEN || (gate == OPENING && angle > ANGLE_OPEN - 20);
  digitalWrite(PIN_LAMP_GREEN, go);
  digitalWrite(PIN_LAMP_RED, !go);
}

void updateLcd(uint32_t now) {
  static uint32_t last = 0;
  if (now - last < 250) return;
  last = now;

  char line[17];
  uint8_t freeSpaces = CAPACITY - occupied;
  snprintf(line, sizeof(line), "%2u/%-2u free  %-4s", freeSpaces, CAPACITY, STATE_TEXT[gate]);
  lcd.setCursor(0, 0);
  lcd.print(line);

  if (approach.fault || underGate.fault) {
    snprintf(line, sizeof(line), "%-16s", approach.fault ? "Sensor 1 fault!" : "Sensor 2 fault!");
  } else if (message[0] && now - messageAt < MESSAGE_MS) {
    snprintf(line, sizeof(line), "%-16s", message);
  } else if (freeSpaces == 0) {
    snprintf(line, sizeof(line), "%-16s", "PARKING FULL");
  } else if (approach.cm <= ASSIST_RANGE_CM) {
    snprintf(line, sizeof(line), "Distance %3u cm ", approach.cm);
  } else {
    snprintf(line, sizeof(line), "%-16s", "Welcome");
  }
  lcd.setCursor(0, 1);
  lcd.print(line);
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  approach.begin();
  underGate.begin();
  btnExit.begin();
  btnManual.begin();
  btnReset.begin();
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LAMP_RED, OUTPUT);
  pinMode(PIN_LAMP_GREEN, OUTPUT);

  barrier.attach(PIN_SERVO);
  barrier.write(ANGLE_CLOSED);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F(" Smart Parking  "));
  lcd.setCursor(0, 1);
  lcd.print(F("  Gate  v1.0    "));

  loadCount();
  logEvent(F("start, occupied"), occupied);
  delay(1200);
  lcd.clear();
}

void loop() {
  uint32_t now = millis();

  // one ultrasonic sensor every 30 ms, alternating, so their echoes never mix
  if (now - lastSensor >= 30) {
    lastSensor = now;
    if (sensorTurn) underGate.measure();
    else approach.measure();
    sensorTurn = !sensorTurn;
  }

  btnExit.update(now);
  btnManual.update(now);
  btnReset.update(now);

  handleRequests(now);
  updateGate(now);
  updateAssist(now);
  updateLamps();
  updateLcd(now);
}
