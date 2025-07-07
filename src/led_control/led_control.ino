#include <Adafruit_NeoPixel.h>

#define INPUT_PIN 2
#define LED_PIN 6
#define NUM_LEDS 2

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pinMode(INPUT_PIN, INPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Zacinam sledovat signal...");

  strip.begin();            
  strip.setBrightness(50);  
  strip.show();             
}

void loop() {
  int state = digitalRead(INPUT_PIN);

  if (state == HIGH) {
    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.setPixelColor(1, strip.Color(0, 0, 0));
  } else {
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.setPixelColor(1, strip.Color(255, 0, 0));
  }

  strip.show();

  Serial.print("Signal na vstupe: ");
  Serial.println(state);

  delay(100);
}
