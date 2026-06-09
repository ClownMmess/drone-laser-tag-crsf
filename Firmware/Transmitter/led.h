uint8_t ledState = LOW;
unsigned long previousMillis = 0;

void blinkLED(int ledPin, uint16_t blinkRate) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= blinkRate) {
        previousMillis = currentMillis;     // save the last time you blinked the LED
        ledState ^= 1;                      // if the LED is off turn it on and vice-versa
        digitalWrite(ledPin, ledState);
    }
}
