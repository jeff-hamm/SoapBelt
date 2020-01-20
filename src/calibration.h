#ifndef CALIBRATION_H_INCLUDED
#define CALIBRATION_H_INCLUDED
#include "Particle.h"

#define LIMIT_SWITCH_PIN D4
#define DISTANCE_SENSOR_PIN A0
#define LED_STATUS_PIN D7
#define FORWARD_PIN D6
#define BACK_PIN D5
#define TRIGGER_PIN D2
#define SENSOR_EVENT "sensor_trigger"


#define VERSION 2

#define SLOW_DUTY_CYCLE .5
#define SLOW_WINDOW 300


#define DEBOUNCE_TIME_MS 100
#define DEFAULT_DURATION_MS 1500
#define MAXIMUM_PUSH_FLEX_MS 800

enum RelayMode {
    Off = HIGH,
    On = LOW,
    Slow = 2,
};

namespace Calibration {
    void setupCalibration();
    uint16_t getPushDuration();
    void updatePushDuration(system_tick_t startTime, system_tick_t endTime, uint16_t relayDurationMs);
    void printCalibrationStatus();
    RelayMode checkSensors(system_tick_t time);
}

#endif