/*
 * DFS Maze Solver with OLED Display
 * Memory-optimised version
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============== PIN DEFINITIONS ==============
#define TRIG_WEST 11
#define ECHO_WEST 13
#define TRIG_EAST 10
#define ECHO_EAST 8
#define TRIG_NORTH 2
#define ECHO_NORTH 4

// IR Sensors
#define GROVE_IR A2
#define SHARP_IR A0

// ============== CONSTANTS ==============
#define WALL_THRESHOLD_CM 15.0
#define TURN_COMPLETE_THRESHOLD 18.0
#define MAX_EVENTS 20  // Reduced from 50

// ============== ENUMS ==============
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum EventType { EVENT_TURN_LEFT, EVENT_TURN_RIGHT, EVENT_DEAD_END, EVENT_STRAIGHT, EVENT_NONE };
enum RobotMode { STRAIGHT_LINE_MODE, EVENT_MODE };

// ============== GLOBAL VARIABLES ==============
Direction currentDirection = EAST;
RobotMode currentMode = STRAIGHT_LINE_MODE;
EventType currentEventType = EVENT_NONE;
EventType pendingEventType;
Direction targetDirection;
int eventCount = 0;

// Sensor readings
float distNorth = 0;
float distEast = 0;
float distWest = 0;

// Wall flags
bool wallNorth = false;
bool wallEast = false;
bool wallWest = false;

// IR sensor readings
int groveValue = 0;
bool groveDetect = false;
float sharpDistance = 0;

// ============== FUNCTIONS ==============

float readUltrasonic(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 999.0;
    return (duration * 0.0343) / 2.0;
}

void readAllSensors() {
    // Ultrasonic sensors
    distNorth = readUltrasonic(TRIG_NORTH, ECHO_NORTH);
    distEast = readUltrasonic(TRIG_EAST, ECHO_EAST);
    distWest = readUltrasonic(TRIG_WEST, ECHO_WEST);
    
    wallNorth = (distNorth < WALL_THRESHOLD_CM);
    wallEast = (distEast < WALL_THRESHOLD_CM);
    wallWest = (distWest < WALL_THRESHOLD_CM);
    
    // IR sensors
    groveValue = analogRead(GROVE_IR);
    groveDetect = (groveValue < 500);  // Adjust threshold as needed
    
    int sharpValue = analogRead(SHARP_IR);
    float voltage = sharpValue * (5.0 / 1023.0);
    sharpDistance = 12.08 * pow(voltage, -1.058);
}

Direction turnLeft(Direction d) { return (Direction)((d + 3) % 4); }
Direction turnRight(Direction d) { return (Direction)((d + 1) % 4); }
Direction turnAround(Direction d) { return (Direction)((d + 2) % 4); }

void updateDisplay() {
    display.clearDisplay();
    display.setFont();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    
    // Line 1: Mode
    display.print(F("Mode: "));
    display.println(currentMode == STRAIGHT_LINE_MODE ? F("STRAIGHT") : F("EVENT"));
    
    // Line 2: Event
    display.print(F("Event #"));
    display.print(eventCount);
    display.print(F(": "));
    switch (currentEventType) {
        case EVENT_TURN_LEFT:  display.println(F("Left")); break;
        case EVENT_TURN_RIGHT: display.println(F("Right")); break;
        case EVENT_DEAD_END:   display.println(F("Dead-end")); break;
        case EVENT_STRAIGHT:   display.println(F("Straight")); break;
        default:               display.println(F("---")); break;
    }
    
    // Line 3: Walls
    display.print(F("Walls N:"));
    display.print(wallNorth ? F("Y") : F("N"));
    display.print(F(" E:"));
    display.print(wallEast ? F("Y") : F("N"));
    display.print(F(" W:"));
    display.println(wallWest ? F("Y") : F("N"));
    
    // Lines 4-6: Distances
    display.print(F("Front: "));
    display.print(distNorth, 1);
    display.println(F("cm"));
    
    display.print(F("Left:  "));
    display.print(distWest, 1);
    display.println(F("cm"));
    
    display.print(F("Right: "));
    display.print(distEast, 1);
    display.println(F("cm"));
    
    // Line 7: IR sensors (replaces direction)
    display.print(F("Grove:"));
    display.print(groveDetect ? F("ON ") : F("OFF"));
    display.print(F(" Sharp:"));
    display.print(sharpDistance, 1);
    display.println(F("cm"));
    
    display.display();
}

void detectEvent() {
    // Dead end
    if (wallNorth && wallEast && wallWest) {
        Serial.println(F("DEAD END - Turn 180"));
        pendingEventType = EVENT_DEAD_END;
        currentEventType = EVENT_DEAD_END;
        targetDirection = turnAround(currentDirection);
        currentMode = EVENT_MODE;
        return;
    }
    
    // Straight corridor
    if (!wallNorth && wallEast && wallWest) {
        currentEventType = EVENT_STRAIGHT;
        return;
    }
    
    // Left turn
    if (wallNorth && !wallWest && wallEast) {
        Serial.println(F("LEFT TURN - Turn left"));
        pendingEventType = EVENT_TURN_LEFT;
        currentEventType = EVENT_TURN_LEFT;
        targetDirection = turnLeft(currentDirection);
        currentMode = EVENT_MODE;
        return;
    }
    
    // Right turn
    if (wallNorth && wallWest && !wallEast) {
        Serial.println(F("RIGHT TURN - Turn right"));
        pendingEventType = EVENT_TURN_RIGHT;
        currentEventType = EVENT_TURN_RIGHT;
        targetDirection = turnRight(currentDirection);
        currentMode = EVENT_MODE;
        return;
    }
    
    // Junction or open - continue forward
    currentEventType = EVENT_STRAIGHT;
}

void handleTurning() {
    bool turnComplete = false;
    
    if (!wallNorth && distNorth > TURN_COMPLETE_THRESHOLD) {
        turnComplete = true;
    }
    
    if (pendingEventType == EVENT_DEAD_END && !wallNorth) {
        turnComplete = true;
    }
    
    if (turnComplete) {
        currentDirection = targetDirection;
        eventCount++;
        currentMode = STRAIGHT_LINE_MODE;
        Serial.print(F("Turn complete. Events: "));
        Serial.println(eventCount);
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 failed"));
        for (;;);
    }
    
    display.clearDisplay();
    display.display();
    delay(500);
    
    pinMode(TRIG_WEST, OUTPUT);
    pinMode(ECHO_WEST, INPUT);
    pinMode(TRIG_EAST, OUTPUT);
    pinMode(ECHO_EAST, INPUT);
    pinMode(TRIG_NORTH, OUTPUT);
    pinMode(ECHO_NORTH, INPUT);
    pinMode(GROVE_IR, INPUT);
    pinMode(SHARP_IR, INPUT);
    
    Serial.println(F("Maze Solver Ready"));
}

void loop() {
    readAllSensors();
    
    if (currentMode == STRAIGHT_LINE_MODE) {
        detectEvent();
    } else {
        handleTurning();
    }
    
    updateDisplay();
    delay(300);
}
