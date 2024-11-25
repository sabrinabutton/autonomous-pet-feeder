// This is a useful arduino program to clear the EEPROM in case you change the order of what is stored.
#include <EEPROM.h> // Include the EEPROM library

void setup() {
  Serial.begin(115200);
  Serial.println("Clearing EEPROM...");

  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0xFF);
  }

  Serial.println("EEPROM has been cleared. Verifying...");

  // Verify the EEPROM
  bool cleared = true;
  for (int i = 0; i < EEPROM.length(); i++) {
    if (EEPROM.read(i) != 0xFF) {
      Serial.print("Address ");
      Serial.print(i);
      Serial.println(" not cleared.");
      cleared = false;
    }
  }

  if (cleared) {
    Serial.println("EEPROM verification passed.");
  } else {
    Serial.println("EEPROM verification failed.");
  }
}

void loop() {
  // Do nothing in the loop
}
