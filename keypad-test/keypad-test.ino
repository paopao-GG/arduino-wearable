// Keypad Pin Test - Find correct row/column mapping
// Upload this to test different pin configurations

#include <Keypad.h>

const byte ROWS = 4;
const byte COLS = 4;

// Standard layout
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// TEST CONFIGURATION 1: Original (Rows then Columns)
byte rowPins[ROWS] = {2, 3, 4, 5};
byte colPins[COLS] = {6, A1, A2, A3};

// Uncomment below to test CONFIGURATION 2: Reversed pins
// byte rowPins[ROWS] = {5, 4, 3, 2};
// byte colPins[COLS] = {A3, A2, A1, 6};

// Uncomment below to test CONFIGURATION 3: Columns first, then Rows
// byte colPins[COLS] = {2, 3, 4, 5};
// byte rowPins[ROWS] = {6, A1, A2, A3};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(9600);
  delay(100);

  Serial.println(F("================================="));
  Serial.println(F("Keypad Pin Configuration Test"));
  Serial.println(F("================================="));
  Serial.println(F("Current Configuration:"));
  Serial.print(F("  Rows: D"));
  for (int i = 0; i < ROWS; i++) {
    Serial.print(rowPins[i]);
    if (i < ROWS - 1) Serial.print(F(", D"));
  }
  Serial.println();

  Serial.print(F("  Cols: "));
  for (int i = 0; i < COLS; i++) {
    if (colPins[i] >= A0) {
      Serial.print(F("A"));
      Serial.print(colPins[i] - A0);
    } else {
      Serial.print(F("D"));
      Serial.print(colPins[i]);
    }
    if (i < COLS - 1) Serial.print(F(", "));
  }
  Serial.println();
  Serial.println(F("================================="));
  Serial.println(F("Press any key on the keypad..."));
  Serial.println();
}

void loop() {
  char key = keypad.getKey();

  if (key) {
    Serial.print(F(">>> KEY DETECTED: "));
    Serial.print(key);
    Serial.println(F(" <<<"));

    // Show which physical position was pressed
    Serial.print(F("    Expected key: "));
    Serial.println(key);
    Serial.println();
  }
}
