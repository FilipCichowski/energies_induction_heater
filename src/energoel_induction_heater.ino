#include <Arduino.h>
#include <EEPROM.h>

#define SHIFT_DATA 7
#define SHIFT_CLOCK 8
#define SHIFT_LATCH 9

#define BUTTON_MODE A4
#define BUTTON_UP A5
#define BUTTON_DOWN 13
#define MANUAL_LED 6
#define TIMER_LED 4
#define EEPROM_ADDR_MODE 0
#define EEPROM_ADDR_POWER 1

volatile uint8_t currentDigit = 0;
bool manualMode;
uint8_t powerLevel;
bool lastButtonModeState = HIGH;
bool modeChangeRequested = false;

// Przechowywanie czasu trzymania przycisków mocy
unsigned long buttonUpHoldStart = 0;
unsigned long buttonDownHoldStart = 0;
bool buttonUpHeld = false;
bool buttonDownHeld = false;

// Mapa segmentów: g, f, e, d, c, b, a (LSB = 0)
const uint8_t digitMap[10] = {
  0b01111110, // 0
  0b00001100, // 1
  0b10110110, // 2
  0b10011110, // 3
  0b11001100, // 4
  0b11011010, // 5
  0b11111010, // 6
  0b00001110, // 7
  0b11111110, // 8
  0b11011110  // 9
};

// Bufor cyfr do wyświetlacza
uint8_t digitBuffer[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLOCK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(MANUAL_LED, OUTPUT);
  pinMode(TIMER_LED, OUTPUT);

  // Odczytaj zapisane wartości z EEPROM
  uint8_t savedMode = EEPROM.read(EEPROM_ADDR_MODE);
  manualMode = (savedMode == 0xFF) ? true : savedMode;
  EEPROM.update(EEPROM_ADDR_MODE, manualMode);

  uint8_t savedPower = EEPROM.read(EEPROM_ADDR_POWER);
  powerLevel = (savedPower > 100) ? 50 : savedPower;
  EEPROM.update(EEPROM_ADDR_POWER, powerLevel);

  updateLEDs();
  updatePowerDisplay();

  // Timer do multipleksowania co 2ms
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 1999;
  TIMSK1 |= (1 << OCIE1A);
}

// ISR do multipleksowania wyświetlacza
ISR(TIMER1_COMPA_vect) {
  digitalWrite(SHIFT_LATCH, LOW);
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, (1 << (6 - currentDigit)));
  shiftOut(SHIFT_DATA, SHIFT_CLOCK, MSBFIRST, digitMap[digitBuffer[currentDigit]]);
  digitalWrite(SHIFT_LATCH, HIGH);
  currentDigit = (currentDigit + 1) % 6;

  // Obsługa zmiany trybu w przerwaniu
  static bool lastModeState = HIGH;
  bool modeState = digitalRead(BUTTON_MODE);
  
  if (modeState == LOW && lastModeState == HIGH) {
    modeChangeRequested = true;
  }
  lastModeState = modeState;
}

void loop() {
  if (modeChangeRequested) {
    modeChangeRequested = false;
    manualMode = !manualMode;
    EEPROM.update(EEPROM_ADDR_MODE, manualMode);
    updateLEDs();
  }
  
  checkPowerButtons();
}

// **Obsługa przycisków zmiany mocy**
void checkPowerButtons() {
  handlePowerButton(BUTTON_UP, &buttonUpHeld, &buttonUpHoldStart, 1);
  handlePowerButton(BUTTON_DOWN, &buttonDownHeld, &buttonDownHoldStart, -1);
}

void handlePowerButton(int button, bool* held, unsigned long* holdStart, int change) {
  static unsigned long lastButtonPress = 0;
  bool reading = digitalRead(button);
  unsigned long now = millis();

  if (reading == LOW) {
    if (!*held) {
      *holdStart = now;
      *held = true;
      changePowerLevel(change); // Zmieniamy wartość od razu
      lastButtonPress = now;
    }

    unsigned long holdTime = now - *holdStart;
    unsigned long interval = (holdTime >= 3000) ? 200 : (holdTime >= 1500) ? 400 : 1000;

    if (now - lastButtonPress > interval) {
      changePowerLevel(change);
      lastButtonPress = now;
    }
  } else {
    *held = false;
  }
}

// **Zmiana wartości mocy**
void changePowerLevel(int change) {
  if ((change == 1 && powerLevel < 100) || (change == -1 && powerLevel > 0)) {
    powerLevel += change;
    powerLevel = constrain(powerLevel, 0, 100);
    EEPROM.update(EEPROM_ADDR_POWER, powerLevel);
    updatePowerDisplay();
  }
}

// **Aktualizacja LEDów trybu**
void updateLEDs() {
  digitalWrite(MANUAL_LED, manualMode);
  digitalWrite(TIMER_LED, !manualMode);
}

// **Wyświetlanie wartości mocy**
void updatePowerDisplay() {
  digitBuffer[3] = powerLevel / 100; // Setki
  digitBuffer[4] = (powerLevel / 10) % 10; // Dziesiątki
  digitBuffer[5] = powerLevel % 10; // Jedności
}
