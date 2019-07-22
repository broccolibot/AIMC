#include "protocol.hpp"
#include <Wire.h>

void setup() {
    Wire.begin(25);
    Wire.onReceive(received_something);
    Serial.begin(9600);
}

void received_something(int n_bytes) {
    Serial.println("Received ");
    Serial.println(n_bytes);
    Serial.println(" bytes.");
}

void loop() {
    delay(500);
}
