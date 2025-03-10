#include <Arduino.h>
#include <EEPROM.h>

#define SHIFT_DATA_PIN 7
#define SHIFT_CLOCK_PIN 8
#define SHIFT_LATCH_PIN 9

#define MODE_BUTTON A4
#define POWER_UP_BUTTON A5
#define POWER_DOWN_BUTTON 13
#define TIME_UP_BUTTON 12
#define TIME_DOWN_BUTTON 11
#define MANUAL_LED 6
#define TIMER_LED 4
#define HEATER_OUTPUT A0
#define ERR_INPUT A1
#define START_INPUT A2
#define BUZZER_OUTPUT 5
#define PWM_OUTPUT 10

#define EEPROM_ADDR_MODE 0
#define EEPROM_ADDR_POWER 1
#define EEPROM_ADDR_TIME 2
#define EEPROM_ADDR_LAST 3

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
bool lastStartState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool errorState = false;
unsigned long errorTime = 0;
unsigned long lastErrorReadTime = 0;

const uint8_t digitMap[12] = {
  0b01111110, 0b00001100, 0b10110110, 0b10011110,  // 0-3
  0b11001100, 0b11011010, 0b11111010, 0b00001110,  // 4-7
  0b11111110, 0b11011110,  // 8-9
  0b11110010,  // 'E'
  0b10100000   // 'r'
};

uint8_t digitBuffer[6];

void setup() {
  pinMode(SHIFT_DATA_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_LATCH_PIN, OUTPUT);
  pinMode(MODE_BUTTON, INPUT_PULLUP);
  pinMode(POWER_UP_BUTTON, INPUT_PULLUP);
  pinMode(POWER_DOWN_BUTTON, INPUT_PULLUP);
  pinMode(TIME_UP_BUTTON, INPUT_PULLUP);
  pinMode(TIME_DOWN_BUTTON, INPUT_PULLUP);
  pinMode(MANUAL_LED, OUTPUT);
  pinMode(TIMER_LED, OUTPUT);
  pinMode(START_INPUT, INPUT);
  pinMode(BUZZER_OUTPUT, OUTPUT);
  pinMode(PWM_OUTPUT, OUTPUT);
  pinMode(ERR_INPUT, INPUT);
  pinMode(HEATER_OUTPUT, OUTPUT);

  uint8_t savedMode = EEPROM.read(EEPROM_ADDR_MODE);
  manualMode = (savedMode == 0xFF) ? true : savedMode;
  EEPROM.update(EEPROM_ADDR_MODE, manualMode);

  uint8_t savedPower = EEPROM.read(EEPROM_ADDR_POWER);
  powerLevel = (savedPower > 100) ? 50 : savedPower;
  EEPROM.update(EEPROM_ADDR_POWER, powerLevel);

  uint16_t savedTime = (EEPROM.read(EEPROM_ADDR_TIME) << 8) | EEPROM.read(EEPROM_ADDR_TIME + 1);
  timerValue = (savedTime > 999) ? 100 : savedTime;
  EEPROM.update(EEPROM_ADDR_TIME, (timerValue >> 8) & 0xFF);
  EEPROM.update(EEPROM_ADDR_TIME + 1, timerValue & 0xFF);

  delay(100);

  updateLEDs();
  updatePowerDisplay();
  updateTimerDisplay();
  analogWrite(PWM_OUTPUT, powerLevel * 2.2);

  TCCR2A = (1 << WGM21); // CTC (Clear Timer on Compare Match)
  TCCR2B = (1 << CS22);  // Prescaler 64 (16MHz / 64 = 250 kHz, OCR2A = 249 → 1kHz)
  OCR2A = 249; // Interrupt every 1ms
  TIMSK2 |= (1 << OCIE2A); // Enable interrupt for timer 2

  delay(500);
}

ISR(TIMER2_COMPA_vect) {
  digitalWrite(SHIFT_LATCH_PIN, LOW);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, (1 << (6 - currentDigit)));
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, digitMap[digitBuffer[currentDigit]]);
  digitalWrite(SHIFT_LATCH_PIN, HIGH);
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

void handleErrorState() {
  static unsigned long errorBlockTime = 0;

  if (millis() - errorBlockTime > 1000) {
    errorState = false;
    errorTime = 0;
    lastErrorReadTime = 0;

    if (digitalRead(ERR_INPUT) == HIGH && !errorState) {
      unsigned long currentTime = millis();
      if (currentTime - lastErrorReadTime > debounceDelay) {
        lastErrorReadTime = currentTime;
        errorState = true;
        errorTime = currentTime;
        countingEnabled = false;
        digitBuffer[0] = 10;  // 'E'
        digitBuffer[1] = 11;  // 'r'
        digitBuffer[2] = 11;  // 'r'
        for (int i = 0; i < 3; i++) {
          beep(500);
          delay(500);
        }
        errorBlockTime = millis();  // Block ERR_INPUT for one second
      }
    }
  }

  // Allow clearing the error immediately by pressing START button
  if (errorState) {
    if (digitalRead(START_INPUT) == HIGH) {
      errorState = false;
      updateTimerDisplay();
    }
  }
}

void handleModeChange() {
  if (modeChangeRequested) {
    modeChangeRequested = false;
    manualMode = !manualMode;
    EEPROM.update(EEPROM_ADDR_MODE, manualMode);
    updateLEDs();
  }
}

void handleButtonChecks() {
  checkButtons(POWER_UP_BUTTON, POWER_DOWN_BUTTON, &buttonUpHeld, &buttonDownHeld, &buttonUpHoldStart, &buttonDownHoldStart, changePowerLevel, 0, 100);
  checkButtons(TIME_UP_BUTTON, TIME_DOWN_BUTTON, &buttonTimeUpHeld, &buttonTimeDownHeld, &buttonTimeUpHoldStart, &buttonTimeDownHoldStart, changeTimerValue, 0, 999);
}

void handleTimer() {
  unsigned long now = millis();
  bool startState = digitalRead(START_INPUT);

  if (!manualMode) {
    if (startState == HIGH) {
      countingEnabled = true;
    } else {
      countingEnabled = false;
    }

    if (countingEnabled && (now - lastTimerDecrement >= 1000)) {
      lastTimerDecrement = now;

      if (timerValue > 0) {
        timerValue--;
        beep(50);
        updateTimerDisplay();
        EEPROM.update(EEPROM_ADDR_TIME, (timerValue >> 8) & 0xFF);
        EEPROM.update(EEPROM_ADDR_TIME + 1, timerValue & 0xFF);
      }

      if (timerValue == 0) {
        beep(500);
        countingEnabled = false;
      }
    }
  }
}

void handleHeaterOutput() {
  bool startState = digitalRead(START_INPUT);

  if (!manualMode && countingEnabled) {
    digitalWrite(HEATER_OUTPUT, HIGH);
  } else if (!manualMode && !countingEnabled) {
    digitalWrite(HEATER_OUTPUT, LOW);
  } else if (manualMode) {
    digitalWrite(HEATER_OUTPUT, startState);
  }
}

void handleModeButton() {
  static bool lastModeState = HIGH;
  bool modeState = digitalRead(MODE_BUTTON);

  if (modeState == LOW && lastModeState == HIGH) {
    modeChangeRequested = true;
    beep(50);
  }
  lastModeState = modeState;
}

void loop() {
  handleErrorState();

  if (!errorState) {
    handleModeChange();
    handleButtonChecks();
    handleTimer();
    handleHeaterOutput();
    handleModeButton();
  }
}

void changePowerLevel(int change) {
  powerLevel = constrain(powerLevel + change, 0, 100);
  analogWrite(PWM_OUTPUT, powerLevel * 2.2);
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
    digitalWrite(BUZZER_OUTPUT, HIGH);
    delayMicroseconds(500);  // 1 kHz
    digitalWrite(BUZZER_OUTPUT, LOW);
    delayMicroseconds(500);
  }
}