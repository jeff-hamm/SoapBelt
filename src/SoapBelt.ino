/*
 * Project SoapBelt
 * Description:
 * Author:
 * Date:
 */

#include "calibration.h"

//#define NO_MESH_GATEWAY
#ifdef NO_MESH_GATEWAY
SYSTEM_MODE(MANUAL);
#endif


#define RELAY_ON(pin) digitalWrite(pin,LOW)
#define RELAY_OFF(pin) digitalWrite(pin,HIGH)

uint16_t pushDuration = DEFAULT_DURATION_MS;
   

 
void setup() {
  // ensure relays are off
  pinMode(FORWARD_PIN,OUTPUT);
  RELAY_OFF(FORWARD_PIN);
  pinMode(BACK_PIN, OUTPUT);
  RELAY_OFF(BACK_PIN);
  // settle delay
  delay(1500);
  Serial.begin(9600);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT); 
  attachInterrupt(TRIGGER_PIN,sensorTriggerHandler, FALLING);
  Mesh.subscribe(SENSOR_EVENT,publishTriggerHandler);
  Calibration::setupCalibration();
  Serial.println("SoapBelt Started");
}


static bool isPushing = false;
volatile bool sensorTriggered = false;
static system_tick_t pushStartTime = 0;
void loop() {
  system_tick_t time = millis();
  RelayMode pushState = updatePushState(time);
  setRelayMode(FORWARD_PIN, pushState, time);
  if(pushState != Off) {
    if(blink(time,100)) {
        Serial.printf("Pushing, Countdown %, state %u \n", pushDuration-(time-pushStartTime), pushState);
    }
  }
  else if(sensorTriggered) {
    startPushing(time);
  }
  else if(blink(time, 1000)) {
    printStatus(time);
  }
}

static uint16_t relayOnDurationMs = 0;
void toggleRelay(int pin, bool toggle, system_tick_t time) {
  static bool currentToggle = false;
  static system_tick_t lastToggleTime = 0;
  if(currentToggle) {
    relayOnDurationMs += time - lastToggleTime;
  }
  if(toggle) {
    RELAY_ON(pin);
  }
  else {
    RELAY_OFF(pin);
  }
  currentToggle = toggle;
  lastToggleTime = time;
}

  
void setRelayMode(int pin, RelayMode mode, system_tick_t time) {
  const uint16_t SLOW_ON_CYCLE=SLOW_WINDOW*SLOW_DUTY_CYCLE;
  static system_tick_t slowCycleBegin = 0;
  if(mode != Slow) {
    toggleRelay(pin, mode == On,time);
    return;
  }
  uint16_t cycle = time-slowCycleBegin;
  if(cycle > SLOW_WINDOW) {
    slowCycleBegin=time;
    cycle = 0;
  }
  if(cycle < SLOW_ON_CYCLE)
    toggleRelay(pin,true, time);
  else
    toggleRelay(pin, false, time);
}


volatile system_tick_t sensorTriggerTime = 0;
void publishTriggerHandler(const char *event, const char *data) {
  sensorTriggerHandler();
}
void sensorTriggerHandler() {
  static system_tick_t lastInterrupt =0;
  sensorTriggerTime = millis();
  if(sensorTriggerTime - lastInterrupt < DEBOUNCE_TIME_MS) {
      return;
  }
//  Serial.printf("Sensor triggered at %.2f. Current push state is %u. Sensor trigger state is %u.\n",sensorTriggerTime/1000.0, isPushing,sensorTriggered);
  sensorTriggered = true;
}
void startPushing(system_tick_t time) {
  sensorTriggered = false;
  pushStartTime = time;
  isPushing = true;
  pushDuration = Calibration::getPushDuration();
  relayOnDurationMs = 0;
  toggleRelay(FORWARD_PIN, true,time);
  Serial.printf("Started pushing at %.2f, with expected duration %u. \n",time/1000.0,pushDuration);
}
system_tick_t pushEndTime = 0;
void stopPushing(system_tick_t time) {
  isPushing = false;
  pushEndTime = time;
  toggleRelay(FORWARD_PIN, false, time);
  Calibration::updatePushDuration(pushStartTime,pushEndTime, relayOnDurationMs);
  sensorTriggered = false;
}

RelayMode updatePushState(system_tick_t time) {
  uint16_t duration = time-pushStartTime;
  RelayMode sensorMode = Calibration::checkSensors(time);
  if(!isPushing)
    return Off;
  if(duration > pushDuration + MAXIMUM_PUSH_FLEX_MS) {
    Serial.printf("Duration %u out of range. Stopping.\n",duration);
    sensorMode = Off;
  }
  if(sensorMode == On && duration > pushDuration) {
    sensorMode = Slow;
  }
  if(sensorMode == Off) {
    stopPushing(time);
  }
  return sensorMode;
}





bool blink(system_tick_t time, uint16_t speed) {
  static int BLINK = LOW;
  static system_tick_t lastBlink;
  if(time-lastBlink > speed) {
    lastBlink = time;
    digitalWrite(LED_STATUS_PIN, (BLINK = !BLINK));
    return true;
  }
  return false;
}

void printStatus(system_tick_t time) {

    Serial.printf("Time: %.2f, Last Trigger: %.2f, AvgDuration: %u \n", 
      time/1000.0, 
      sensorTriggerTime/1000.0, 
//      pushEndTime > 0 ? pushEndTime-pushStartTime : 0.0,
      pushDuration);
    Calibration::printCalibrationStatus();

}