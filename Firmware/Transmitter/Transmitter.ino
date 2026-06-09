#include <Arduino.h>
#include "NonBlockingRtttl.h"
#include "EEPROM.h"
#include "config.h"
#include "crsf.h"
#include "led.h"
#include "tone.h"

const int numReadings = 10;
int readings[numReadings] = {0};
int index = 0;
int total = 0;
int average = 0;

int Aileron_value = 0;
int Elevator_value = 0;
int Throttle_value = 0;
int Rudder_value = 0;
int previous_throttle = 191;

int loopCount = 0;
int AUX1_Arm = 0;

#ifdef USE_3POS_SWITCH_AS_CHANNEL_6
    int AUX2_value_HIGH = 0;
    int AUX2_value_LOW = 0;
#else
    int AUX2_value_LOW = 0;
#endif

int AUX2_value = 0;
int AUX3_value = 0;
int AUX4_value = 0;

float batteryVoltage;

int currentPktRate = 0;
int currentPower = 0;
int currentDynamic = 0;
int currentSetting = 0;
int stickMoved = 0;
int stickInt = 0;
uint32_t stickMovedMillis = 0;
uint32_t currentMillis = 0;

uint8_t crsfPacket[CRSF_PACKET_SIZE];
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
uint32_t crsfTime = 0;

CRSF crsfClass;

bool calStatus = false;

// --- Calibration Constants ---
#define CALIB_MARK      0x55
#define CALIB_MARK_ADDR 0x00
#define CALIB_VAL_ADDR  CALIB_MARK_ADDR + 1
#define CALIB_CNT       5
#define CALIB_CENT_TMO  5000
#define CALIB_TMO       20000

int cal_reset = 0;

struct CalibValues {
    int aileronMin;
    int aileronMax;
    int aileronCenter;
    int elevatorMin;
    int elevatorMax;
    int elevatorCenter;
    int thrMin;
    int thrMax;
    int rudderMin;
    int rudderMax;
    int rudderCenter;
};

CalibValues calValues;

bool calibrationPresent() {
    return EEPROM.read(CALIB_MARK_ADDR) == CALIB_MARK;
}

void calibrationSave() {
    EEPROM.update(CALIB_MARK_ADDR, CALIB_MARK);
    EEPROM.put(CALIB_VAL_ADDR, calValues);
}

void calibrationLoad() {
    EEPROM.get(CALIB_VAL_ADDR, calValues);
}

void calibrationReset() {
    EEPROM.put(CALIB_MARK_ADDR, (uint8_t)0xFF);
    calValues = {
        .aileronMin     = ANALOG_CUTOFF,
        .aileronMax     = 1023 - ANALOG_CUTOFF,
        .aileronCenter  = 988,
        .elevatorMin    = ANALOG_CUTOFF,
        .elevatorMax    = 1023 - ANALOG_CUTOFF,
        .elevatorCenter = 988,
        .thrMin         = ANALOG_CUTOFF,
        .thrMax         = 1023 - ANALOG_CUTOFF,
        .rudderMin      = ANALOG_CUTOFF,
        .rudderMax      = 1023 - ANALOG_CUTOFF,
        .rudderCenter   = 988,
    };
    EEPROM.put(CALIB_VAL_ADDR, calValues);
}

uint8_t aux2cnt = 0;
unsigned long calibrationTimerStart;

void calibrationChirp(uint8_t times) {
    digitalWrite(DIGITAL_PIN_LED, HIGH);
    delay(1000);
    digitalWrite(DIGITAL_PIN_LED, LOW);
    delay(1000);

    for (uint8_t i = 0; i < times; i++) {
#ifdef ACTIVE_BUZZER
        digitalWrite(DIGITAL_PIN_BUZZER, HIGH);
#else
        tone(DIGITAL_PIN_BUZZER, 5000, 100);
#endif
        digitalWrite(DIGITAL_PIN_LED, HIGH);
        delay(100);
#ifdef ACTIVE_BUZZER
        digitalWrite(DIGITAL_PIN_BUZZER, LOW);
#endif
        digitalWrite(DIGITAL_PIN_LED, LOW);
        delay(100);
    }
    digitalWrite(DIGITAL_PIN_LED, LOW);
}

void calibrationCount(int aux1, int aux2) {
    static uint8_t preAux2 = 0;

    if (aux1 != 0) {
        aux2cnt = 0;
        preAux2 = 0;
        return;
    } else {
        if (aux2cnt == 0) {
            calibrationTimerStart = millis();
        }

        if (calibrationTimerStart + CALIB_TMO > millis()) {
            aux2cnt = 0;
            preAux2 = 0;
            return;
        }

        if (aux2 == 1 && preAux2 == 0) {
            aux2cnt++;
            preAux2 = 1;
        } else if (aux2 == 0 && preAux2 == 1) {
            preAux2 = 0;
        }
    }
}

bool calibrationRequested() {
    return calStatus;
}

bool calibrationProcess() {
    aux2cnt = 0;

    while (cal_reset < 1) {
        const int centerValue = (1023 - ANALOG_CUTOFF - ANALOG_CUTOFF) / 2;
        calValues.aileronMin     = centerValue;
        calValues.aileronMax     = centerValue;
        calValues.aileronCenter  = centerValue;
        calValues.elevatorMin    = centerValue;
        calValues.elevatorMax    = centerValue;
        calValues.elevatorCenter = centerValue;
        calValues.thrMin         = centerValue;
        calValues.thrMax         = centerValue;
        calValues.rudderMin      = centerValue;
        calValues.rudderMax      = centerValue;
        calValues.rudderCenter   = centerValue;
        cal_reset++;
    }

    currentMillis = millis();

    if (currentMillis < CALIB_CENT_TMO) {
        calValues.aileronCenter = analogRead(analogInPinAileron);
        calValues.elevatorCenter = analogRead(analogInPinElevator);
        calValues.rudderCenter = analogRead(analogInPinRudder);
    } else if (currentMillis > CALIB_CENT_TMO && currentMillis < CALIB_TMO) {
        int val = analogRead(analogInPinAileron);
        if (val < calValues.aileronMin) calValues.aileronMin = val;
        if (val > calValues.aileronMax) calValues.aileronMax = val;

        val = analogRead(analogInPinElevator);
        if (val < calValues.elevatorMin) calValues.elevatorMin = val;
        if (val > calValues.elevatorMax) calValues.elevatorMax = val;

        val = analogRead(analogInPinThrottle);
        if (val < calValues.thrMin) calValues.thrMin = val;
        if (val > calValues.thrMax) calValues.thrMax = val;

        val = analogRead(analogInPinRudder);
        if (val < calValues.rudderMin) calValues.rudderMin = val;
        if (val > calValues.rudderMax) calValues.rudderMax = val;
    } else {
        calibrationSave();
        calStatus = false;
    }
    return true;
}

void calibrationRun(int aux1, int aux2) {
    if (calibrationRequested()) {
        if (!calibrationProcess()) {
            // Error handling placeholder
        }
    } else {
        calibrationCount(aux1, aux2);
    }
}

void selectSetting() {
    if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) {
        currentPktRate = SETTING_1_PktRate;
        currentPower = SETTING_1_Power;
        currentDynamic = SETTING_1_Dynamic;
        currentSetting = 1;
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] > RC_MAX_COMMAND) {
        currentPktRate = SETTING_2_PktRate;
        currentPower = SETTING_2_Power;
        currentDynamic = SETTING_2_Dynamic;
        currentSetting = 2;
    } else if (rcChannels[AILERON] < RC_MIN_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) {
        currentSetting = 3; 
    } else if (rcChannels[AILERON] > RC_MAX_COMMAND && rcChannels[ELEVATOR] < RC_MIN_COMMAND) {
        currentSetting = 4;
    } else {
        currentSetting = 0;
    }
}

bool checkStickMove() {
    if (abs(previous_throttle - rcChannels[THROTTLE]) < 30) {
        stickMoved = 0;
    } else {
        previous_throttle = rcChannels[THROTTLE];
        stickMovedMillis = millis();
        stickMoved = 1;
    }

    return (millis() - stickMovedMillis > STICK_ALARM_TIME);
}

void setup() {
    for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
        rcChannels[i] = CRSF_DIGITAL_CHANNEL_MIN;
    }

    pinMode(DIGITAL_PIN_SWITCH_ARM, INPUT_PULLUP);
#ifdef USE_3POS_SWITCH_AS_CHANNEL_6
    pinMode(DIGITAL_PIN_SWITCH_AUX2_HIGH, INPUT_PULLUP);
    pinMode(DIGITAL_PIN_SWITCH_AUX2_LOW, INPUT_PULLUP);
#else
    pinMode(DIGITAL_PIN_SWITCH_AUX2, INPUT_PULLUP);
#endif
    pinMode(DIGITAL_PIN_SWITCH_AUX3, INPUT_PULLUP);
    
    if (DIGITAL_PIN_SWITCH_AUX4 != 0) {
        pinMode(DIGITAL_PIN_SWITCH_AUX4, INPUT_PULLUP);
    }
    
    pinMode(DIGITAL_PIN_LED, OUTPUT);
    pinMode(DIGITAL_PIN_BUZZER, OUTPUT);

#ifdef PASSIVE_BUZZER
    digitalWrite(DIGITAL_PIN_BUZZER, LOW);
    if (STARTUP_MELODY != "") {
        rtttl::begin(DIGITAL_PIN_BUZZER, STARTUP_MELODY);
    }
#endif

    batteryVoltage = 0.0;

#ifdef PPMOUTPUT
    pinMode(ppmPin, OUTPUT);
    digitalWrite(ppmPin, !onState);
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 100;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
    sei();
#endif

#ifdef PASSIVE_BUZZER
    if (STARTUP_MELODY != "") {
        while (!rtttl::done()) {
            rtttl::play();
        }
    } else {
        delay(2000);
    }
#else  
    digitalWrite(DIGITAL_PIN_BUZZER, LOW);
    delay(100);
    digitalWrite(DIGITAL_PIN_BUZZER, HIGH);
    delay(200);
    digitalWrite(DIGITAL_PIN_BUZZER, LOW);
    delay(100);
    digitalWrite(DIGITAL_PIN_BUZZER, HIGH);
    delay(200);
    digitalWrite(DIGITAL_PIN_BUZZER, LOW);
    delay(200);
    digitalWrite(DIGITAL_PIN_BUZZER, HIGH);
    delay(2000);
#endif

#ifdef DEBUG
    Serial.begin(115200);
#else
    crsfClass.begin();
#endif

#ifdef GIMBAL_CALIBRATION
    Serial.begin(115200);
    calStatus = true;
#endif

    digitalWrite(DIGITAL_PIN_LED, HIGH);
    calibrationLoad();
    
    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_BIND_COMMAND, ELRS_START_COMMAND);
    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
}

void loop() {
    total -= readings[index];
    readings[index] = analogRead(A0);
    total += readings[index];
    index = (index + 1) % numReadings;

    average = total / numReadings;

    rcChannels[ELEVATOR] = map(analogRead(A1) + 20, 150, 832, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[RUDDER] = 1020;
    rcChannels[THROTTLE] = map(average, 90, 800, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
    rcChannels[AILERON] = map(analogRead(A2), 190, 800, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);

    crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
    crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
}

ISR(TIMER1_COMPA_vect) {
    static boolean state = true;
    TCNT1 = 0;

    if (state) {
        digitalWrite(ppmPin, onState);
        OCR1A = PULSE_LENGTH * 2;
        state = false;
    } else {
        static byte cur_chan_numb;
        static unsigned int calc_rest;

        digitalWrite(ppmPin, !onState);
        state = true;

        if (cur_chan_numb >= CHANNEL_NUMBER) {
            cur_chan_numb = 0;
            calc_rest = calc_rest + PULSE_LENGTH;
            OCR1A = (FRAME_LENGTH - calc_rest) * 2;
            calc_rest = 0;
        } else {
            OCR1A = (map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000) - PULSE_LENGTH) * 2;
            calc_rest = calc_rest + map(rcChannels[cur_chan_numb], CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX, 1000, 2000);
            cur_chan_numb++;
        }
    }
}