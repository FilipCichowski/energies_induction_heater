#include <Arduino.h>
#include <EEPROM.h>

#define SHIFT_DATA 7
#define SHIFT_CLOCK 8
#define SHIFT_LATCH 9

#define BUTTON_MODE A4
#define POWER_UP A5
#define POWER_DOWN 13
#define MANUAL_LED 6
#define TIMER_LED 4
#define EEPROM_ADDR_MODE 0
#define EEPROM_ADDR_POWER 1

#define TIME_UP 12
#define TIME_DOWN 11
#define EEPROM_ADDR_TIME 2

#define START_INPUT A2
#define STOP_INPUT A3  // Dodano definicję pinu STOP

volatile uint8_t currentDigit = 0;
bool manualMode;
uint8_t powerLevel;
bool lastButtonModeState = HIGH;
bool modeChangeRequested = false;

unsigned long buttonUpHoldStart = 0;
unsigned long buttonDownHoldStart = 0;
bool buttonUpHeld = false;
bool buttonDownHeld = false;

unsigned long buttonTimeUpHoldStart = 0;
unsigned long buttonTimeDownHoldStart = 0;
bool buttonTimeUpHeld = false;
bool buttonTimeDownHeld = false;
uint16_t timerValue;

unsigned long lastTimerDecrement = 0;

bool countingEnabled = false;  // Zmienna do przechowywania stanu odliczania
bool toggleMode = false;       // Tryb toggle (gdy STOP jest wysoki)
bool lastStartState = HIGH;    // Poprzedni stan START
bool lastStopState = HIGH;     // Poprzedni stan STOP
unsigned long lastDebounceTime = 0;  // Czas ostatniego debounce'u
const unsigned long debounceDelay = 50;  // Opóźnienie debounce'u

const uint8_t digitMap[10] = {
  0b01111110, 0b00001100, 0b10110110, 0b10011110,
  0b11001100, 0b11011010, 0b11111010, 0b00001110,
  0b11111110, 0b11011110
};

uint8_t digitBuffer[6];

void setup() {
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLOCK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(POWER_UP, INPUT_PULLUP);
  pinMode(POWER_DOWN, INPUT_PULLUP);
  pinMode(MANUAL_LED, OUTPUT);
  pinMode(TIMER_LED, OUTPUT);
  pinMode(TIME_UP, INPUT_PULLUP);
  pinMode(TIME_DOWN, INPUT_PULLUP);
  pinMode(START_INPUT, INPUT);
  pinMode(STOP_INPUT, INPUT);  // Ustawienie pinu STOP jako wejście

  uint8_t savedMode = EEPROM.read(EEPROM_ADDR_MODE);
  manualMode = (savedMode == 0xFF) ? true : savedMode;
  EEPROM.update(EEPROM_ADDR_MODE, manualMode);

  uint8_t savedPower = EEPROM.read(EEPROM_ADDR_POWER);
  powerLevel = (savedPower > 100) ? 50 : savedPower;
  EEPROM.update(EEPROM_ADDR_POWER, powerLevel);

  uint16_t savedTime = EEPROM.read(EEPROM_ADDR_TIME);
  timerValue = (savedTime > 999) ? 100 : savedTime;
  EEPROM.update(EEPROM_ADDR_TIME, timerValue);

  delay(100);
  // Sprawdzenie stanu pinu STOP przy starcie
  toggleMode = (digitalRead(STOP_INPUT) == HIGH);

  updateLEDs();
  updatePowerDisplay();
  updateTimerDisplay();

  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 1999;
  TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, (1 << (6 - currentDigit)));
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, digitMap[digitBuffer[currentDigit]]);
  digitalWrite(SHIFT_LATCH, HIGH);
  currentDigit = (currentDigit + 1) % 6;

  static bool lastModeState = HIGH;
  bool modeState = digitalRead(BUTTON_MODE);
  
  if (modeState == LOW && lastModeState == HIGH) {
    modeChangeRequested = true;
  }
  lastModeState = modeState;
}

void handleButtonPress(bool* held, unsigned long* holdStart, unsigned long now, void (*changeValue)(int), int change) {
  static unsigned long lastButtonPress = 0;

  if (!*held) {
    *holdStart = now;
    *held = true;
    changeValue(change);
    lastButtonPress = now;
  }

  unsigned long holdTime = now - *holdStart;
  unsigned long interval = (holdTime >= 2000) ? 100 : (holdTime >= 400) ? 400 : 1000;

  if (now - lastButtonPress > interval) {
    changeValue(change);
    lastButtonPress = now;
  }
}

void checkButtons(int buttonUp, int buttonDown, bool* upHeld, bool* downHeld, unsigned long* upHoldStart, unsigned long* downHoldStart, void (*changeValue)(int), int minValue, int maxValue) {
  unsigned long now = millis();

  if (digitalRead(buttonUp) == LOW && !*downHeld) {
    handleButtonPress(upHeld, upHoldStart, now, changeValue, 1);
  } else {
    *upHeld = false;
  }

  if (digitalRead(buttonDown) == LOW && !*upHeld) {
    handleButtonPress(downHeld, downHoldStart, now, changeValue, -1);
  } else {
    *downHeld = false;
  }
}

void loop() {
  if (modeChangeRequested) {
    modeChangeRequested = false;
    manualMode = !manualMode;
    EEPROM.update(EEPROM_ADDR_MODE, manualMode);
    updateLEDs();
  }
  
  checkButtons(POWER_UP, POWER_DOWN, &buttonUpHeld, &buttonDownHeld, &buttonUpHoldStart, &buttonDownHoldStart, changePowerLevel, 0, 100);
  checkButtons(TIME_UP, TIME_DOWN, &buttonTimeUpHeld, &buttonTimeDownHeld, &buttonTimeUpHoldStart, &buttonTimeDownHoldStart, changeTimerValue, 0, 999);

  // Obsługa START i STOP z debouncingiem
  bool startState = digitalRead(START_INPUT);
  bool stopState = digitalRead(STOP_INPUT);
  unsigned long now = millis();

  if (now - lastDebounceTime > debounceDelay) {
    if (toggleMode) {
      // Tryb toggle: START działa jako przełącznik
      if (startState == HIGH && lastStartState == LOW) {
        countingEnabled = !countingEnabled;
      }
    } else {
      // Tryb normalny: START włącza, STOP wyłącza
      if (startState == HIGH && lastStartState == LOW) {
        countingEnabled = true;
      }
      if (stopState == HIGH && lastStopState == LOW) {
        countingEnabled = false;
      }
    }
    lastStartState = startState;
    lastStopState = stopState;
    lastDebounceTime = now;
  }

  // Odliczanie czasu, jeśli jest włączone
  if (!manualMode && timerValue > 0 && countingEnabled) {
    if (now - lastTimerDecrement >= 1000) {
      lastTimerDecrement = now;
      timerValue--;
      EEPROM.update(EEPROM_ADDR_TIME, timerValue);
      updateTimerDisplay();
    }
  }
}

void changePowerLevel(int change) {
  powerLevel = constrain(powerLevel + change, 0, 100);
  EEPROM.update(EEPROM_ADDR_POWER, powerLevel);
  updatePowerDisplay();
}

void changeTimerValue(int change) {
  timerValue = constrain(timerValue + change, 0, 999);
  EEPROM.update(EEPROM_ADDR_TIME, timerValue);
  updateTimerDisplay();
}

void updateLEDs() {
  digitalWrite(MANUAL_LED, manualMode);
  digitalWrite(TIMER_LED, !manualMode);
}

void updatePowerDisplay() {
  digitBuffer[3] = powerLevel / 100;
  digitBuffer[4] = (powerLevel / 10) % 10;
  digitBuffer[5] = powerLevel % 10;
}

void updateTimerDisplay() {
  digitBuffer[0] = timerValue / 100;
  digitBuffer[1] = (timerValue / 10) % 10;
  digitBuffer[2] = timerValue % 10;
}