// Nano RP2040 Connect
// Uses Serial1 for UART communication (pins 0 = TX, 1 = RX)
// SSD1306 128x64 OLED via I2C (SDA = A4/GPIO12, SCL = A5/GPIO13)

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1      // no reset pin
#define OLED_ADDR      0x3C    // typical I2C address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float distance1 = 0.0;
float distance2 = 0.0;
float distance3 = 0.0;
float distance4 = 0.0;

String incomingData = "";

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 150;  // refresh every 150 ms

void setup() {
    Serial.begin(9600);   // USB serial for debugging
    Serial1.begin(9600);  // Hardware UART on pins 0/1

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("SSD1306 init failed"));
        while (true);  // halt if display not found
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(20, 28);
    display.print(F("Waiting for data"));
    display.display();
}

void parseDistances(String data) {
    // Expected format: "D1:xxx.x,D2:xxx.x,D3:xxx.x,D4:xxx.x"

    int idx1 = data.indexOf("D1:") + 3;
    int idx2 = data.indexOf("D2:") + 3;
    int idx3 = data.indexOf("D3:") + 3;
    int idx4 = data.indexOf("D4:") + 3;

    int comma1 = data.indexOf(',', idx1);
    int comma2 = data.indexOf(',', idx2);
    int comma3 = data.indexOf(',', idx3);

    if (idx1 > 2 && idx2 > 2 && idx3 > 2 && idx4 > 2) {
        distance1 = data.substring(idx1, comma1).toFloat();
        distance2 = data.substring(idx2, comma2).toFloat();
        distance3 = data.substring(idx3, comma3).toFloat();
        distance4 = data.substring(idx4).toFloat();
    }
}

void updateDisplay() {
    display.clearDisplay();

    // ---- title bar ----
    display.setTextSize(1);
    display.setCursor(12, 0);
    display.print(F("ULTRASONIC SENSORS"));
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    // ---- sensor readings (size 1 = 6x8 px per char) ----
    // Layout:  label left-aligned, value right-aligned with "cm"

    display.setCursor(0, 16);
    display.print(F("Front:"));
    display.setCursor(72, 16);
    display.print(distance1, 1);
    display.print(F(" cm"));

    display.setCursor(0, 28);
    display.print(F("Right:"));
    display.setCursor(72, 28);
    display.print(distance2, 1);
    display.print(F(" cm"));

    display.setCursor(0, 40);
    display.print(F("Back:"));
    display.setCursor(72, 40);
    display.print(distance3, 1);
    display.print(F(" cm"));

    display.setCursor(0, 52);
    display.print(F("Left:"));
    display.setCursor(72, 52);
    display.print(distance4, 1);
    display.print(F(" cm"));

    display.display();
}

void loop() {
    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n') {
            parseDistances(incomingData);
            incomingData = "";

            // Debug output via USB
            Serial.print("Front: ");
            Serial.print(distance1, 1);
            Serial.print(" | Right: ");
            Serial.print(distance2, 1);
            Serial.print(" | Back: ");
            Serial.print(distance3, 1);
            Serial.print(" | Left: ");
            Serial.println(distance4, 1);
        } else if (c != '\r') {
            incomingData += c;
        }
    }

    // Refresh OLED at a fixed interval to avoid blocking UART reads
    if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
        lastDisplayUpdate = millis();
        updateDisplay();
    }
}
