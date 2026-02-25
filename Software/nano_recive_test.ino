// Nano RP2040 Connect
// Uses Serial1 for UART communication (pins 0 = TX, 1 = RX)

float distance1 = 0.0;
float distance2 = 0.0;
float distance3 = 0.0;
float distance4 = 0.0;

String incomingData = "";

void setup() {
    Serial.begin(9600);   // USB serial for debugging
    Serial1.begin(9600);  // Hardware UART on pins 0/1
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
    
    // Your navigation logic here using distance1-4
}
