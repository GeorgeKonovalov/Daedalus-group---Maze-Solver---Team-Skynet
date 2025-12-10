#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Optional: custom GFX font
#include <Fonts/FreeSans9pt7b.h>

#define SCREEN_WIDTH 128   // OLED display width, in pixels
#define SCREEN_HEIGHT 64   // OLED display height, in pixels
#define OLED_RESET    -1   // Reset pin (or -1 if sharing reset with Arduino)
#define SCREEN_ADDRESS 0x3C   // Most 0.96" I2C OLEDs: 0x3C, some: 0x3D

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);

  // For AVR (Uno/Nano/Mega), DO NOT pass SDA/SCL pins here.
  Wire.begin();   // Uses the hardware I2C pins (A4/A5 or 20/21)

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);  // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();
  delay(500);
}

void loop() {
  
  char event = 't';               // T-junction
  int event_number = 5;
  bool IR_FR = true;
  bool IR_FL = false;

  float distFront = 12.34;
  float distLeft  = 56.78;
  float distRight = 90.12;

  Show_Event_Data(event,
                  event_number,
                  IR_FR,
                  IR_FL,
                  &distFront,
                  &distLeft,
                  &distRight);
  // If you want to go back to default font later:
  // display.setFont();
}

void Show_Event_Data(char event,           // single char instead of char*
                     int event_number,
                     bool IR_front_right,
                     bool IR_front_left,
                     float* Distance_front,
                     float* Distance_left,
                     float* Distance_right)
{
  display.clearDisplay();

  // Map event code to human-readable name
  const char* event_name = "";

  switch (event)
  {
    case 'd':
      event_name = "Dead-end";
      break;
    case 'l':
      event_name = "Left-turning";
      break;
    case 'r':
      event_name = "Right-turning";
      break;
    case 't':
      event_name = "T-junction";
      break;
    case 's':
      event_name = "Straight";
      break;
    case 'e':
      event_name = "Entrance";
      break;
    case 'f':
      event_name = "Finish";
      break;
    default:
      event_name = "Unknown";
      break;
  }

  display.setFont();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0); // start at top

  // Line 1: Event number + event name (same line)
  display.print(F("Event "));
  display.print(event_number);
  display.print(F(": "));
  display.println(event_name);

  // Line 2: Both IR sensors on same line
  display.print(F("IR FR: "));
  display.print(IR_front_right ? F("ON") : F("OFF"));
  display.print(F("  IR FL: "));
  display.println(IR_front_left ? F("ON") : F("OFF"));

  // Line 3: Distance front
  display.print(F("Dist front: "));
  if (Distance_front != nullptr) {
    display.println(*Distance_front, 2);  // 2 decimal places
  } else {
    display.println(F("N/A"));
  }

  // Line 4: Distance left
  display.print(F("Dist left : "));
  if (Distance_left != nullptr) {
    display.println(*Distance_left, 2);
  } else {
    display.println(F("N/A"));
  }

  // Line 5: Distance right
  display.print(F("Dist right: "));
  if (Distance_right != nullptr) {
    display.println(*Distance_right, 2);
  } else {
    display.println(F("N/A"));
  }

  display.display();  // push to the screen (if using Adafruit_SSD1306)
}

