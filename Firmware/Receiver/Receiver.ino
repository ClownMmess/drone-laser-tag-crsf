#include <SoftwareSerial.h>
#include "PPMEncoder.h"

// --- Hardware Pins ---
#define OUTPUT_PIN 3
#define RX_PIN 10
#define TX_PIN 11
#define PHOTO_RESISTOR_PIN A0
#define LASER_PIN 5

// --- Configuration Constants ---
#define CHANNEL_OFFSET 10
#define HIT_COOLDOWN_MS 5000
#define LASER_COOLDOWN_MS 5000
#define HIT_THRESHOLD_OFFSET 20
#define PACKAGE_MAX_SIZE 64

// Set up a new SoftwareSerial object
SoftwareSerial Serial1(RX_PIN, TX_PIN);

uint8_t crcTable[256];

#define WIDTH  (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

// Initialize CRC table
void crcInit(void) {
    uint8_t remainder;
    for (int dividend = 0; dividend < 256; ++dividend) {
        remainder = dividend << (WIDTH - 8);
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (remainder & TOPBIT) {
                remainder = (remainder << 1) ^ 0xD5;
            } else {
                remainder = (remainder << 1);
            }
        }
        crcTable[dividend] = remainder;
    }
}

// Calculate CRC8
uint8_t crcFast(uint8_t const message[], int nBytes) {
    uint8_t data;
    uint8_t remainder = 0;
    for (int byte = 0; byte < nBytes; ++byte) {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }
    return (remainder);
}

// CRSF Channels structure (11 bits per channel)
typedef struct crsf_channels_t {
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} PACKED;

// --- Global Variables ---
byte BF[PACKAGE_MAX_SIZE];
int BFix = 0;
uint8_t PackageLength = 0;
uint8_t PackageType = 0;
bool PackageIsReading = false;
bool PackageJustRead = false;

byte BF_channels[PACKAGE_MAX_SIZE];
crsf_channels_t *cr_channels = (crsf_channels_t *)BF_channels;
bool channels_are_read = false;
long channels_read_millis = -1;

int ch[8] = {0};
int fotoRMax = 0;

// State tracking for non-blocking delays
unsigned long laser_time = 0;
unsigned long hit_time = 0;
bool is_hit_active = false;

// Read incoming CRSF package from Serial
void channels_read() {
    PackageJustRead = false;
    
    if (Serial1.available()) {
        while (Serial1.available() > 0) {
            byte bb = Serial1.read();

            if (PackageIsReading) {
                BF[BFix++] = bb;
            } else {
                // Check for valid CRSF sync bytes
                if (bb == 0xc8 || bb == 0xee || bb == 0xea || bb == 0xec) {
                    PackageType = bb;
                    PackageLength = Serial1.read();
                    if (PackageLength <= PACKAGE_MAX_SIZE) {
                        PackageIsReading = true;
                        BFix = 0;
                    }
                }
            }
            
            // Package fully read
            if (PackageLength <= BFix) {
                PackageIsReading = false;
                PackageJustRead = true;
                
                uint8_t crc8 = crcFast(BF, PackageLength - 1);
                
                // CRC check passed
                if (crc8 == BF[PackageLength - 1]) {
                    if (BF[0] == 0x16) {
                        for (int ix = 1; ix < PackageLength; ix++) {
                            BF_channels[ix - 1] = BF[ix];
                        }
                        channels_read_millis = millis();
                        channels_are_read = true;
                    }
                }
            }
        }
    }
}

// Map CRSF values (174-1811) to PWM values (1000-2000)
void channels_write(int *ch) {
    ch[0] = map(cr_channels->ch0, 174, 1811, 1000, 2000) - CHANNEL_OFFSET;
    ch[1] = map(cr_channels->ch1, 174, 1811, 1000, 2000) - CHANNEL_OFFSET;
    ch[2] = map(cr_channels->ch2, 174, 1811, 1000, 2000) - CHANNEL_OFFSET;
    ch[3] = map(cr_channels->ch3, 174, 1811, 1000, 2000) - CHANNEL_OFFSET;
    ch[4] = map(cr_channels->ch4, 174, 1811, 1000, 2000) - CHANNEL_OFFSET;
    ch[5] = map(cr_channels->ch5, 174, 1811, 1000, 2000) - CHANNEL_OFFSET;
    ch[6] = map(cr_channels->ch6, 174, 1811, 1000, 2000) - CHANNEL_OFFSET;
    ch[7] = map(cr_channels->ch7, 174, 1811, 1000, 2000) - CHANNEL_OFFSET;
}

// Send values to PPM encoder
void channels_send(int *ch) {
    for (int i = 0; i < 8; i++) {
        ppmEncoder.setChannel(i, ch[i]);
    }
}

// Calibrate ambient light for the photoresistor
int fotoR_calibration() {
    int temp = 0;
    int tempMax = 0;
    for (int i = 0; i < 100; i++) {
        temp = analogRead(PHOTO_RESISTOR_PIN);
        if (temp > tempMax) tempMax = temp;
        delay(10);
    }
    return tempMax;
}

// Check if photoresistor detects a laser hit
bool is_laser_hit() {
    return (analogRead(PHOTO_RESISTOR_PIN) > (fotoRMax + HIT_THRESHOLD_OFFSET));
}

void setup() {
    crcInit();

    pinMode(RX_PIN, INPUT);
    pinMode(TX_PIN, OUTPUT);
    pinMode(LASER_PIN, OUTPUT);

    Serial.begin(9600);
    Serial1.begin(19200);

    ppmEncoder.begin(OUTPUT_PIN);
    ppmEncoder.setChannel(9, 1000); // Default hit state (no hit)

    fotoRMax = fotoR_calibration();
    laser_time = millis();
}

void loop() {
    unsigned long current_millis = millis();

    channels_read();
    channels_write(ch);

    // --- Non-blocking hit detection logic ---
    if (is_laser_hit() && !is_hit_active) {
        is_hit_active = true;
        hit_time = current_millis;
        ppmEncoder.setChannel(9, 1900); // Send hit signal
    }

    // Reset hit state after cooldown
    if (is_hit_active && (current_millis - hit_time >= HIT_COOLDOWN_MS)) {
        is_hit_active = false;
        ppmEncoder.setChannel(9, 1000); // Reset hit signal
    }

    // --- Laser firing logic ---
    if ((ch[7] >= 1500) && (current_millis - laser_time >= LASER_COOLDOWN_MS)) {
        digitalWrite(LASER_PIN, HIGH);
        laser_time = current_millis;
    } else {
        digitalWrite(LASER_PIN, LOW);
    }

    channels_send(ch);

    // Debug output
    Serial.println(ch[3]);
}