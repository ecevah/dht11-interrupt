#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>


#define DHTPIN 23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


#define LED_PIN 21
#define NUM_PIXELS 32 
Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);


#define GREEN_LED 33
#define YELLOW_LED 32
#define BLUE_LED 25
#define RED_LED 26


unsigned long previousGreenMillis = 0;
unsigned long previousYellowMillis = 0;
unsigned long previousRedBlinkMillis = 0;
unsigned long previousSensorMillis = 0;

bool redBlinkState = false;
bool greenState = false;
bool yellowState = false;

void updateNeoPixels(float humidity, float temperature);
void updateTemperatureLEDs(float temperature);

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  dht.begin();
  pixels.begin();
  pixels.clear();
  pixels.show();

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);

  Serial.println("Sistem hazır!");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousSensorMillis >= 2000) {
    previousSensorMillis = currentMillis;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("HATA: DHT11 sensöründen veri okunamıyor!");
      pixels.clear();
      pixels.show();
      return;
    }

    Serial.print("Sicaklik: ");
    Serial.print(t);
    Serial.println(" °C");

    Serial.print("Nem: ");
    Serial.print(h);
    Serial.println(" %");

    updateNeoPixels(h, t);
    updateTemperatureLEDs(t);
  }

  if (currentMillis - previousGreenMillis >= 2000) {
    greenState = !greenState;
    digitalWrite(GREEN_LED, greenState ? HIGH : LOW);
    previousGreenMillis = currentMillis;
  }

  if (currentMillis - previousYellowMillis >= 3000) {
    yellowState = !yellowState;
    digitalWrite(YELLOW_LED, yellowState ? HIGH : LOW);
    previousYellowMillis = currentMillis;
  }

  if (currentMillis - previousRedBlinkMillis >= 500) {
    if (digitalRead(BLUE_LED) == HIGH) {
      redBlinkState = !redBlinkState;
      digitalWrite(RED_LED, redBlinkState ? HIGH : LOW);
    }
    previousRedBlinkMillis = currentMillis;
  }
}

void updateNeoPixels(float humidity, float temperature) {
  int humidLEDs = map(constrain(humidity, 0, 100), 0, 100, 0, 16);
  int tempLEDs = map(constrain(temperature, 0, 50), 0, 50, 0, 16);

  pixels.clear();

  for (int i = 0; i < humidLEDs; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 0, 255));
  }
  for (int i = 0; i < tempLEDs; i++) {
    pixels.setPixelColor(16 + i, pixels.Color(255, 0, 0));
  }

  pixels.show();
}

void updateTemperatureLEDs(float temperature) {
  if (temperature < 25) {
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  } else if (temperature <= 30) {
    digitalWrite(BLUE_LED, HIGH);
    
  } else {
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  }
}