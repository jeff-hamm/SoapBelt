#include "calibration.h"
#include <cmath>
#include "movingAvg.h" 

const double MAX_VOLTAGE = 3.3;
const double MAX_SENSOR_VOLTAGE = 3.2;
const int ADC_BITS = 12;
const int MAX_ADC_VALUE = pow(2,ADC_BITS);
const double MAX_SENSOR_VALUE = (MAX_SENSOR_VOLTAGE/MAX_VOLTAGE) * (double)MAX_ADC_VALUE;
const double SENSOR_DISTANCE_CM = 5.0;
#define NUM_READINGS 32
static movingAvg avgTime(NUM_READINGS);
void limitSwitchInterrupt();
/**
 * Calibration
 * 
 * 
 **/
uint16_t avgDuration = DEFAULT_DURATION_MS;

void  Calibration::setupCalibration() {
   pinMode(DISTANCE_SENSOR_PIN, INPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(LIMIT_SWITCH_PIN, limitSwitchInterrupt, CHANGE);
    uint8_t version;
    EEPROM.get(0, version);
    avgTime.begin();
    if(version != VERSION) {
      EEPROM.put(1,avgDuration);
      EEPROM.put(0,version);
      Serial.printf("No calibration found, loading defaults: ");
    }

    // seed the average
    for(int i =0; i < 4; i++)
        avgTime.reading(avgDuration);

    Serial.printf("Calibration version %u, duration %u, window %u\n",version, avgDuration, MAXIMUM_PUSH_FLEX_MS);
}


void Calibration::updatePushDuration(system_tick_t startTime, system_tick_t endTime, uint16_t relayDurationMs) {
    uint16_t totalDuration = endTime-startTime;
    Serial.printf("Push completed with duration %u and pushDuration %u. ", totalDuration, relayDurationMs);
    uint16_t durationBoundary = avgDuration - MAXIMUM_PUSH_FLEX_MS;
    if(relayDurationMs < durationBoundary){
        Serial.printf("Duration %u is too fast, calibrating with minimumDuration %u.\n", relayDurationMs, durationBoundary);
        relayDurationMs = durationBoundary;
    }
    else{
        durationBoundary = avgDuration + MAXIMUM_PUSH_FLEX_MS;
        if(relayDurationMs > durationBoundary){
            Serial.printf("Duration %u is too slow, calibrating with maximumDuration %u.\n", relayDurationMs, durationBoundary);
            relayDurationMs = durationBoundary;
        }
    }
    avgDuration  = avgTime.reading(relayDurationMs);
    Serial.printf("New average duration is %u.\n", avgDuration);
    EEPROM.put(1,avgDuration);
}

uint16_t Calibration::getPushDuration() {
    return avgDuration;
}


double getDistance() {
  int val =analogRead(DISTANCE_SENSOR_PIN);
  return map((double)val, 0.0, MAX_SENSOR_VALUE,0.0,SENSOR_DISTANCE_CM);
}

volatile bool switchPressed = false;
volatile system_tick_t switchPressTime = 0;
volatile system_tick_t switchReleaseTime = 0;
RelayMode Calibration::checkSensors(system_tick_t time) {
    if(!switchPressed) return On;
    if(switchReleaseTime > 0 && time-switchReleaseTime > 500) {
        Serial.println("Switch released");
        switchPressed = false;
        return Off;
    }
    return Slow;
}

void limitSwitchInterrupt() {
  static system_tick_t lastInterrupt =0;
  system_tick_t newTime = millis();
  if(newTime - lastInterrupt < DEBOUNCE_TIME_MS) {
      return;
  }
  lastInterrupt = newTime;

  int switchState = digitalRead(LIMIT_SWITCH_PIN);
  Serial.printf("switchState %u\n",switchState);
  if(!switchPressed) {
      if(switchState == LOW)  {
        switchPressed = true;
        switchPressTime = newTime;
        switchReleaseTime = 0;
        Serial.printf("Switch pressed %.2f \n", switchPressTime/1000.0);
      }
  }
  else {
    switchReleaseTime = newTime;
    Serial.printf("Switch released? %.2f \n", switchReleaseTime/1000.0);
  }
}

void Calibration::printCalibrationStatus() {
    Serial.printf("Distance: %.2f, LimitPressed: %u, PressTime %.2f, ReleaseTime %.2f\n", getDistance(), switchPressed, switchPressTime/1000.0,switchReleaseTime/1000.0);
}