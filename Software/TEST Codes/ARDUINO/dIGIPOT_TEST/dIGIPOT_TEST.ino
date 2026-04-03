// Pins (adjust if needed)
const int pinA = 2;
const int pinB = 3;
const int buttonPin = 4;
// Encoder state
volatile int encoderDelta = 0;
volatile bool moved = false;
// Button state
bool lastButtonState = HIGH;
bool START = LOW;

void buttonPressed()
  {
    if (digitalRead(buttonPin) == LOW)
    {
      lastButtonState = LOW;
    }
    else
    {
      lastButtonState = HIGH;
    }
  }
  
  void handleEncoder() 
{
  bool A = digitalRead(pinA);
  bool B = digitalRead(pinB);

  if (A == B) //A triggered before, then if B is also changed, then clockwise +
  {
    encoderDelta++;
  } else {
    encoderDelta--;
  }

  moved = true;
}

struct MenuItem 
{
  const char* name;
  int value;          // current value
  int minValue;
  int maxValue;
  int step;           // normal increment
  void (*action)();   // optional function to execute
};

void setup()
{
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP); 

  Serial.begin(115200);

  // Attach interrupt on pin A
  attachInterrupt(digitalPinToInterrupt(pinA), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, CHANGE);

  while (!START)
  {
    // ----- ROTATION -----
  if (moved) {
      noInterrupts();
      int value = encoderDelta;
      encoderDelta = 0;
      moved = false;
      interrupts();
    }
  if (!lastButtonState)
    {
      noInterrupts();

      interrupts();
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
