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

#define AVG_COUNT 5  // number of readings to average

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Circular buffers for rolling average
float buf1[AVG_COUNT] = {0};
float buf2[AVG_COUNT] = {0};
float buf3[AVG_COUNT] = {0};
float buf4[AVG_COUNT] = {0};
int   bufIdx = 0;
int   sampleCount = 0;

// Averaged values
float avg1 = 0.0;
float avg2 = 0.0;
float avg3 = 0.0;
float avg4 = 0.0;

// Interpreted differences
float interpFB = 0.0;  // front - back
float interpLR = 0.0;  // left  - right

// ---- Tunable exponent for interpretation curve ----
float b = 1.0;

String incomingData = "";

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 150;

// Applies y = 0.6 * (|x| / 0.6)^b  when |x| < 0.6, preserves sign.
// Pass-through when |x| >= 0.6.
float interpretValue(float x) {
    float absX = fabs(x);
    float sign = (x >= 0.0) ? 1.0 : -1.0;
    if (absX < 0.6) {
        return sign * 0.6 * pow(absX / 0.6, b);
    }
    return x;
}

void interpretDifferences() {
    float diffFB = avg1 - avg3;  // front minus back
    float diffLR = avg4 - avg2;  // left  minus right
    interpFB = interpretValue(diffFB);
    interpLR = interpretValue(diffLR);
}

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("SSD1306 init failed"));
        while (true);
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(20, 28);
    display.print(F("Waiting for data"));
    display.display();
}

void parseDistances(String data) {
    int idx1 = data.indexOf("D1:") + 3;
    int idx2 = data.indexOf("D2:") + 3;
    int idx3 = data.indexOf("D3:") + 3;
    int idx4 = data.indexOf("D4:") + 3;

    int comma1 = data.indexOf(',', idx1);
    int comma2 = data.indexOf(',', idx2);
    int comma3 = data.indexOf(',', idx3);

    if (idx1 > 2 && idx2 > 2 && idx3 > 2 && idx4 > 2) {
        buf1[bufIdx] = data.substring(idx1, comma1).toFloat();
        buf2[bufIdx] = data.substring(idx2, comma2).toFloat();
        buf3[bufIdx] = data.substring(idx3, comma3).toFloat();
        buf4[bufIdx] = data.substring(idx4).toFloat();

        bufIdx = (bufIdx + 1) % AVG_COUNT;
        if (sampleCount < AVG_COUNT) sampleCount++;

        float s1 = 0, s2 = 0, s3 = 0, s4 = 0;
        for (int i = 0; i < sampleCount; i++) {
            s1 += buf1[i];
            s2 += buf2[i];
            s3 += buf3[i];
            s4 += buf4[i];
        }
        avg1 = s1 / sampleCount;
        avg2 = s2 / sampleCount;
        avg3 = s3 / sampleCount;
        avg4 = s4 / sampleCount;

        interpretDifferences();
    }
}

void updateDisplay() {
    display.clearDisplay();

    // ---- title bar ----
    display.setTextSize(1);
    display.setCursor(12, 0);
    display.print(F("ULTRASONIC SENSORS"));
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    // ---- sensor readings ----
    display.setCursor(0, 16);
    display.print(F("Front:"));
    display.setCursor(72, 16);
    display.print(avg1, 1);
    display.print(F(" cm"));

    display.setCursor(0, 26);
    display.print(F("Right:"));
    display.setCursor(72, 26);
    display.print(avg2, 1);
    display.print(F(" cm"));

    display.setCursor(0, 36);
    display.print(F("Back:"));
    display.setCursor(72, 36);
    display.print(avg3, 1);
    display.print(F(" cm"));

    display.setCursor(0, 46);
    display.print(F("Left:"));
    display.setCursor(72, 46);
    display.print(avg4, 1);
    display.print(F(" cm"));

    // ---- interpreted differences ----
    display.drawLine(0, 55, 127, 55, SSD1306_WHITE);
    display.setCursor(0, 57);
    display.print(F("FB:"));
    display.print(interpFB, 2);
    display.setCursor(68, 57);
    display.print(F("LR:"));
    display.print(interpLR, 2);

    display.display();
}

void loop() {
    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n') {
            parseDistances(incomingData);
            incomingData = "";

            Serial.print("Front: ");
            Serial.print(avg1, 1);
            Serial.print(" | Right: ");
            Serial.print(avg2, 1);
            Serial.print(" | Back: ");
            Serial.print(avg3, 1);
            Serial.print(" | Left: ");
            Serial.print(avg4, 1);
            Serial.print(" | FB: ");
            Serial.print(interpFB, 2);
            Serial.print(" | LR: ");
            Serial.println(interpLR, 2);
        } else if (c != '\r') {
            incomingData += c;
        }
    }

    if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
        lastDisplayUpdate = millis();
        updateDisplay();
    }
}
