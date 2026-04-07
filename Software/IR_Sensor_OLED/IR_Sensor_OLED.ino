// IR_Sensor_OLED.ino
// Displays 4 corner IR proximity sensor states on a 128x64 I2C OLED
// Pins: FL=D16, FR=D17, RL=D14, RR=D15
// OLED: SDA/SCL on the Nano 2040 Connect's default I2C pins
// Requires: Adafruit SSD1306, Adafruit GFX Library (install via Library Manager)

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_W 128
#define SCREEN_H  64
#define OLED_ADDR 0x3C  // typical address — try 0x3D if this doesn't work

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// IR sensor pins
const int PIN_IR_FL = 16;
const int PIN_IR_FR = 17;
const int PIN_IR_RL = 14;
const int PIN_IR_RR = 15;

void setup() {
  pinMode(PIN_IR_FL, INPUT);
  pinMode(PIN_IR_FR, INPUT);
  pinMode(PIN_IR_RL, INPUT);
  pinMode(PIN_IR_RR, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // If OLED fails, halt — nothing to show
    while (true);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void loop() {
  int fl = digitalRead(PIN_IR_FL);
  int fr = digitalRead(PIN_IR_FR);
  int rl = digitalRead(PIN_IR_RL);
  int rr = digitalRead(PIN_IR_RR);

  display.clearDisplay();

  // ── Title ──
  display.setTextSize(1);
  display.setCursor(28, 0);
  display.print("IR SENSOR TEST");

  // ── Draw robot body (centred rectangle) ──
  //    Body: 36x28, centred at (64, 36)
  int bx = 46, by = 22, bw = 36, bh = 28;
  display.drawRect(bx, by, bw, bh, SSD1306_WHITE);

  // North arrow inside body
  display.setCursor(bx + 13, by + 4);
  display.print("N");
  display.drawTriangle(
    bx + bw/2,     by + 2,    // top
    bx + bw/2 - 3, by + 8,    // bottom-left
    bx + bw/2 + 3, by + 8,    // bottom-right
    SSD1306_WHITE
  );

  // ── Draw each corner sensor ──
  // FL — top-left of body
  drawCorner(bx - 4,      by - 4,      fl, "FL");
  // FR — top-right of body
  drawCorner(bx + bw - 8, by - 4,      fr, "FR");
  // RL — bottom-left of body
  drawCorner(bx - 4,      by + bh - 6, rl, "RL");
  // RR — bottom-right of body
  drawCorner(bx + bw - 8, by + bh - 6, rr, "RR");

  // ── Status text row at bottom ──
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print("FL:");
  display.print(fl ? "W" : "-");
  display.print(" FR:");
  display.print(fr ? "W" : "-");
  display.print(" RL:");
  display.print(rl ? "W" : "-");
  display.print(" RR:");
  display.print(rr ? "W" : "-");

  display.display();
  delay(100);  // 10 Hz refresh
}

// Draw a filled (wall) or hollow (clear) circle at a corner with its label
void drawCorner(int cx, int cy, int state, const char* label) {
  int r = 5;
  if (state) {
    display.fillCircle(cx, cy, r, SSD1306_WHITE);   // filled = wall
  } else {
    display.drawCircle(cx, cy, r, SSD1306_WHITE);   // hollow = clear
  }

  // Place label offset from the circle so it doesn't overlap the body
  int lx, ly;
  if (label[1] == 'L') {  // left-side sensors
    lx = cx - 18;
  } else {                 // right-side sensors
    lx = cx + 10;
  }
  if (label[0] == 'F') {  // front sensors
    ly = cy - 4;
  } else {                 // rear sensors
    ly = cy - 2;
  }
  display.setCursor(lx, ly);
  display.print(label);
}
