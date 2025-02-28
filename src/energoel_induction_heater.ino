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
#define EEPROM_ADDR_TIME 2
#define EEPROM_ADDR_LAST 3
#define HEATER_PIN A0

#define TIME_UP 12
#define TIME_DOWN 11

#define START_INPUT A2
#define STOP_INPUT A3
#define BUZZER 5
#define PWM_PIN 10

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

bool countingEnabled = false;
bool toggleMode = false;
bool lastStartState = HIGH;
bool lastStopState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

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
  pinMode(STOP_INPUT, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Odczytanie zapisanych ustawień z EEPROM
  uint8_t savedMode = EEPROM.read(EEPROM_ADDR_MODE);
  manualMode = (savedMode == 0xFF) ? true : savedMode;
  EEPROM.update(EEPROM_ADDR_MODE, manualMode);

  uint8_t savedPower = EEPROM.read(EEPROM_ADDR_POWER);
  powerLevel = (savedPower > 100) ? 50 : savedPower;
  EEPROM.update(EEPROM_ADDR_POWER, powerLevel);

  uint16_t savedTime = (EEPROM.read(EEPROM_ADDR_TIME) << 8) | EEPROM.read(EEPROM_ADDR_TIME + 1);  // Odczytanie dwóch bajtów
  timerValue = (savedTime > 999) ? 100 : savedTime;
  EEPROM.update(EEPROM_ADDR_TIME, (timerValue >> 8) & 0xFF);
  EEPROM.update(EEPROM_ADDR_TIME + 1, timerValue & 0xFF);  // Zapisz timerValue do dwóch bajtów

  delay(100);

  toggleMode = (digitalRead(STOP_INPUT) == HIGH);

  updateLEDs();
  updatePowerDisplay();
  updateTimerDisplay();
  analogWrite(PWM_PIN, map(powerLevel, 0, 100, 0, 200));
  //analogWrite(PWM_PIN, 50); 

  TCCR2A = (1 << WGM21); // Tryb CTC (Clear Timer on Compare Match)
  TCCR2B = (1 << CS22);  // Preskaler 64 (16MHz / 64 = 250 kHz, OCR2A = 249 → 1kHz)
  OCR2A = 249; // Przerwanie co 1 ms
  TIMSK2 |= (1 << OCIE2A); // Włącz przerwanie dla Timer2
}

ISR(TIMER2_COMPA_vect) {
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, (1 << (6 - currentDigit)));
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, digitMap[digitBuffer[currentDigit]]);
  digitalWrite(SHIFT_LATCH, HIGH);
  currentDigit = (currentDigit + 1) % 6;
}

void handleButtonPress(bool* held, unsigned long* holdStart, unsigned long now, void (*changeValue)(int), int change) {
  static unsigned long lastButtonPress = 0;

  if (!*held) {
    *holdStart = now;
    *held = true;
    changeValue(change);
    lastButtonPress = now;
    beep(50);
  }

  unsigned long holdTime = now - *holdStart;
  unsigned long interval = (holdTime >= 2000) ? 100 : (holdTime >= 400) ? 400 : 1000;

  if (now - lastButtonPress > interval) {
    changeValue(change);
    lastButtonPress = now;
    beep(50);
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

  bool startState = digitalRead(START_INPUT);
  bool stopState = digitalRead(STOP_INPUT);
  unsigned long now = millis();

  if (now - lastDebounceTime > debounceDelay) {
    if (toggleMode) {
      if (startState == HIGH && lastStartState == LOW) {
        countingEnabled = !countingEnabled;
      }
    } else {
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

  if (!manualMode && countingEnabled) {
    if (now - lastTimerDecrement >= 1000) {
      lastTimerDecrement = now;
      
      if (timerValue > 0) {
        timerValue--;
        beep(50);
        updateTimerDisplay();
        EEPROM.update(EEPROM_ADDR_TIME, (timerValue >> 8) & 0xFF);
        EEPROM.update(EEPROM_ADDR_TIME + 1, timerValue & 0xFF);  // Zapisz timerValue do dwóch bajtów
      }
  
      if (timerValue == 0) {
        beep(500);
        countingEnabled = false;
      }
    }
  }

    // Obsługa pinu A0 (nagrzewnica) w zależności od trybu
    if (!manualMode && countingEnabled) {
      // W trybie odliczania - jeśli odliczanie jest włączone
      digitalWrite(HEATER_PIN, HIGH);  // Ustawienie pinu A0 (nagrzewnicy) na HIGH
    } else if (!manualMode && !countingEnabled) {
      // W trybie odliczania - jeśli odliczanie jest wyłączone
      digitalWrite(HEATER_PIN, LOW);  // Ustawienie pinu A0 (nagrzewnicy) na LOW
    } else if (manualMode && toggleMode) {
      // W trybie manualnym - włączony toggle mode
      if (!startState && lastStartState) {
        countingEnabled = !countingEnabled;  // Zmiana stanu nagrzewnicy
        digitalWrite(HEATER_PIN, countingEnabled);  // Toggle nagrzewnicy (A0)
      }
    } else {
      digitalWrite(HEATER_PIN, startState); 
    }

  static bool lastModeState = HIGH;
  bool modeState = digitalRead(BUTTON_MODE);
  
  if (modeState == LOW && lastModeState == HIGH) {
    modeChangeRequested = true;
    beep(50);
  }
  lastModeState = modeState;
}

void changePowerLevel(int change) {
  powerLevel = constrain(powerLevel + change, 0, 100);
  analogWrite(PWM_PIN, map(powerLevel, 0, 100, 0, 200));
  updatePowerDisplay();
  EEPROM.update(EEPROM_ADDR_POWER, powerLevel);
}

void changeTimerValue(int change) {
  timerValue = constrain(timerValue + change, 0, 999);
  updateTimerDisplay();
  EEPROM.update(EEPROM_ADDR_TIME, (timerValue >> 8) & 0xFF);
  EEPROM.update(EEPROM_ADDR_TIME + 1, timerValue & 0xFF);
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

void beep(unsigned long duration) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
      digitalWrite(BUZZER, HIGH);
      delayMicroseconds(500);  // 1 kHz (1000 µs = 1 ms → 500 µs na pół okresu)
      digitalWrite(BUZZER, LOW);
      delayMicroseconds(500);
  }
}