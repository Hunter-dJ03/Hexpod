#include <Arduino.h>
#include <Servo.h>

const int NUM_VALUES = 18;
const int TOTAL_BITS = NUM_VALUES * 11;
const int TOTAL_BYTES = (TOTAL_BITS + 7) / 8;

Servo coxa1;
Servo femur1;
Servo tibia1;
Servo coxa2;
Servo femur2;
Servo tibia2;
Servo coxa3;
Servo femur3;
Servo tibia3;

//Servo coxa4;
//Servo femur4;
//Servo tibia4;
//Servo coxa5;
//Servo femur5;
//Servo tibia5;
//Servo coxa6;
//Servo femur6;
//Servo tibia6;

void initLegs() {
// Attach Servos  
  coxa1.attach(29);
  femur1.attach(31);
  tibia1.attach(33);

  coxa2.attach(23);
  femur2.attach(25);
  tibia2.attach(27);

  coxa3.attach(22);
  femur3.attach(24);
  tibia3.attach(26);

//  coxa4.attach();
//  femur4.attach();
//  tibia4.attach();
//
//  coxa5.attach();
//  femur5.attach();
//  tibia5.attach();
//
//  coxa6.attach();
//  femur6.attach();
//  tibia6.attach();
};

void setAngs(float decodedFloats[18]) {
  coxa1.writeMicroseconds(deg2ms(decodedFloats[0]));
  femur1.writeMicroseconds(deg2ms(decodedFloats[1]));
  tibia1.writeMicroseconds(deg2ms(decodedFloats[2]));

  coxa2.writeMicroseconds(deg2ms(decodedFloats[3]));
  femur2.writeMicroseconds(deg2ms(decodedFloats[4]));
  tibia2.writeMicroseconds(deg2ms(decodedFloats[5]));

  coxa3.writeMicroseconds(deg2ms(decodedFloats[6]));
  femur3.writeMicroseconds(deg2ms(decodedFloats[7]));
  tibia3.writeMicroseconds(deg2ms(decodedFloats[8]));
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

    initLegs();

//    setAngs(90,90,90);/

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

        setAngs(decodedFloats);
        
        dataReceived = false; // Reset for next batch of data
    }
}
