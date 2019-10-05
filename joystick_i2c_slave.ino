#define DEBUG

#include <Wire.h>
union u_tag {
  uint8_t b[4];
  uint16_t i[2];
} tmp_data[3];

// joystick 
const int SW_pin = 12; // digital pin connected to switch output
const int X_pin = A0; // analog pin connected to X output
const int Y_pin = A1; // analog pin connected to Y output

int keyPress = 0;
bool keyPressStatus = false;

unsigned long joyPreviousMillis = 0;
const long joyInterval = 250;

// led
const int ledPin =  LED_BUILTIN;
int ledState = LOW;
unsigned long ledPreviousMillis = 0;
const long ledInterval = 500;

void setup() {
  // joystick
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  pinMode(ledPin, OUTPUT);

  // i2c
  Wire.begin(8);
  Wire.onRequest(requestEvent);

  // on esp8266 first byte cannot be used above 127
  // so we skip the first two bytes
  tmp_data[0].i[0] = 0;
  tmp_data[0].i[1] = 0;
  
  Serial.begin(115200);
}

void requestEvent() {
  Wire.write((uint8_t*)&tmp_data, 8);
  keyPressStatus = false;

}

void loop() {

  uint16_t x, y;

  unsigned long currentMillis = millis();
  
  if (currentMillis - joyPreviousMillis >= joyInterval) {
    joyPreviousMillis = currentMillis;

    x = analogRead(X_pin);
    y = analogRead(Y_pin);

    if(!keyPressStatus) {
      keyPress = digitalRead(SW_pin);
      keyPressStatus = true;
    }

    tmp_data[0].i[1] = x;
    tmp_data[1].i[0] = y;
    tmp_data[1].i[1] = keyPress;

    #ifdef DEBUG
    Serial.print(tmp_data[1].i[0]); Serial.print("\t"); Serial.println(tmp_data[1].i[1]);
    #endif
  }

  // internal led blink
  if (currentMillis - ledPreviousMillis >= ledInterval) {
    ledPreviousMillis = currentMillis;
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
  }
  delay(50);
}
