#include <Arduino.h>
#include <Servo.h>

const int NUM_VALUES = 3;
const int TOTAL_BITS = NUM_VALUES * 11;
const int TOTAL_BYTES = (TOTAL_BITS + 7) / 8;

Servo coxa;
Servo femur;
Servo tibia;

void initLeg(int coxaPin, int femurPin, int tibiaPin) {
// Attach Servos  
  coxa.attach(coxaPin);
  femur.attach(femurPin);
  tibia.attach(tibiaPin);
}

void setAngs(float coxaAngle, float femurAngle, float tibiaAngle) {
  coxa.writeMicroseconds(deg2ms(coxaAngle));
  femur.writeMicroseconds(deg2ms(femurAngle));
  tibia.writeMicroseconds(deg2ms(tibiaAngle));
}

void decodeBytes(const uint8_t* bytes, float* floats, int numValues) {
    bool bitBuffer[TOTAL_BITS] = {0};

    for (int i = 0; i < TOTAL_BITS; ++i) {
        bitBuffer[i] = (bytes[i / 8] & (1 << (i % 8))) != 0;
    }

    for (int i = 0; i < numValues; ++i) {
        uint16_t value = 0;
        for (int j = 0; j < 11; ++j) {
            if (bitBuffer[i * 11 + j]) {
                value |= (1 << j);
            }
        }
        floats[i] = static_cast<float>(value)/10;
    }
}

int deg2ms(double angle) {
    double min = 500.0;
    double max = 2500.0;
    double ms = min + (((max-min)/180)*angle);
    return (int)ms;
}

void setup() {
    Serial.begin(921600);
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB
    }
//    Serial.println("Send the encoded data...");

    initLeg(22, 24, 26);

    setAngs(90,90,90);

    delay(800);
}

void loop() {
    static uint8_t encodedData[TOTAL_BYTES];
    static int bytesRead = 0;
    static bool dataReceived = false;

    if (Serial.available() > 0) {
        encodedData[bytesRead] = Serial.read();
        bytesRead++;
        if (bytesRead >= TOTAL_BYTES) {
            dataReceived = true;
            bytesRead = 0; // Reset for next reading
        }
    }

    if (dataReceived) {
        float decodedFloats[NUM_VALUES];
        decodeBytes(encodedData, decodedFloats, NUM_VALUES);

//        Serial.println("Decoded values:");
//        for (int i = 0; i < NUM_VALUES; ++i) {
//            Serial.println(decodedFloats[i]);
//        }

        setAngs(decodedFloats[0], decodedFloats[1], decodedFloats[2]);
        
        dataReceived = false; // Reset for next batch of data
    }
}
