// Arduino Uno - Fake Distance Sender
// Sends simulated distance data via hardware UART

float fakeDistance1 = 25.5;
float fakeDistance2 = 40.2;
float fakeDistance3 = 15.8;
float fakeDistance4 = 88.0;

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Send in same format as the ATmega328P code
    Serial.print("D1:");
    Serial.print(fakeDistance1, 1);
    Serial.print(",D2:");
    Serial.print(fakeDistance2, 1);
    Serial.print(",D3:");
    Serial.print(fakeDistance3, 1);
    Serial.print(",D4:");
    Serial.println(fakeDistance4, 1);
    
    // Vary values slightly to confirm updates
    fakeDistance1 += 0.5;
    if (fakeDistance1 > 50.0) fakeDistance1 = 20.0;
    
    delay(1000);
}
