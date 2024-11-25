/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file RangingTag.ino
 * Use this to test two-way ranging functionality with two DW1000. This is
 * the tag component's code which polls for range computation. Addressing and
 * frame filtering is currently done in a custom way, as no MAC features are
 * implemented yet.
 *
 * Complements the "RangingAnchor" example sketch.
 *
 * @todo
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

#include <SPI.h>
#include <DW1000.h>
#include <assert.h>

//UWB Globals
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin
const uint8_t PIN_EXTON = 13;

// Define pins for ADXL335
const int xPin = A0;
const int yPin = A1;
const int zPin = A4;

int steadyStateAccMagnitude = 10500;

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

// sleep timeout period
uint32_t resetSleepPeriod = 0;

// Device sleep state
enum SleepType {
  IDLE,
  DEEP_SLEEP,
};

boolean inSleep = false;
enum SleepType sleep_type = SleepType::DEEP_SLEEP; 


uint32_t sampling_window = 500000;
int32_t last_motion_detected = -sampling_window;
bool sleep_to_wake = false;
uint32_t transmit_poll_time;

void reset_DW1000_config() {
    DW1000.select(PIN_SS);
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(2);
    DW1000.setNetworkId(10);
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000.commitConfiguration();
}

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("### DW1000-arduino-ranging-tag ###"));
    // initialize the driver
    DW1000.begin(PIN_IRQ, PIN_RST);
    Serial.println("DW1000 initialized ...");
    reset_DW1000_config();
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000.getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000.getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000.getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    // anchor starts by transmitting a POLL message
    receiver();
    transmitPoll();
    noteActivity();
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

typedef struct IMUReading {
    int x;
    int y;
    int z;
} IMUReading_t;

struct IMUReading getCalibratedIMUReading() {
    // Read the analog values for X, Y, Z axes of the ADXL335 accelerometer
    int x = analogRead(xPin);
    int y = analogRead(yPin);
    int z = analogRead(zPin);

    int base_x = 515;
    int base_y = 510;
    int base_z = 525;

    x -= base_x;
    y -= base_y;
    z -= base_z;

    // Serial.print("X: ");
    // Serial.print(x);
    // Serial.print("\tY: ");
    // Serial.print(y);
    // Serial.print("\tZ: ");
    // Serial.print(z);

    return {x, y, z};
}

bool checkIMUmotion() {
    IMUReading_t imuReading = getCalibratedIMUReading();
    uint32_t accMagnitude = imuReading.x * imuReading.x + imuReading.y * imuReading.y + imuReading.z * imuReading.z;
    double fractional_threshold = 0.2;

    uint32_t curr_time = micros();

    bool condition = accMagnitude < (steadyStateAccMagnitude * (1-fractional_threshold)) ||
    accMagnitude > (steadyStateAccMagnitude * (1+fractional_threshold));

    if(condition) {
      last_motion_detected = curr_time;
    }

    // Serial.print("\taccMagnitude: ");
    // Serial.print(accMagnitude);
    // Serial.print("\tsteadyStateAccMagnitude: ");
    // Serial.println(steadyStateAccMagnitude);

    // Serial.print("IMU_Detected:");
    // Serial.print(condition);
    // Serial.print(",");
    // Serial.print("Transmission:");
    // Serial.println((curr_time - last_motion_detected) < sampling_window);

    return (curr_time - last_motion_detected) < sampling_window;
}

void DW1000_sleep() {
    assert(!inSleep);
    switch(sleep_type) {
      case SleepType::IDLE:
        DW1000.idle();
        break;
      default:
        // Serial.print("Previous time: ");
        // Serial.println(micros());
        // DW1000Time prev_time;
        // DW1000.getSystemTimestamp(prev_time);
        // Serial.print("Current time before going to sleep: ");
        // Serial.println(prev_time);
        DW1000.deepSleep();
        // delay(10);
        // DW1000Time curr_time;
        // DW1000.getSystemTimestamp(curr_time);
        // Serial.print("Current time after going to sleep: ");
        // Serial.println(curr_time);
        // Serial.print("Current time: ");
        // Serial.println(micros());
        break;
    }
    inSleep = true;
    // mark led for sleep
    digitalWrite(LED_BUILTIN, LOW);
}

void DW1000_wakeup() {
    assert(inSleep);
    switch(sleep_type) {
      case SleepType::DEEP_SLEEP:
        DW1000.spiWakeup();
        reset_DW1000_config();
        break;
      default:
        break;
    }
    sleep_to_wake = true;
    inSleep = false;
    receiver();
    // mark led for wakeup
    // digitalWrite(LED_BUILTIN, HIGH);
}

void 
checkIMUandRetransmit() {
    // Serial.print("Inside check IMU and Retransmit");
    sleep_to_wake = false;
    if (checkIMUmotion()) {
        if (inSleep) {
            DW1000_wakeup();
        }
        expectedMsgId = POLL_ACK;
        transmitPoll();
    }
    else {
        if (!inSleep) {
            DW1000_sleep();
        }
    }
    noteActivity();
}

void resetInactive() {
    // tag sends POLL and listens for POLL_ACK
    checkIMUandRetransmit();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
    // Serial.println("Send complete");
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
    // Serial.println("Receive complete");
}

void transmitPoll() {
    transmit_poll_time = micros();
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRange() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;
    // delay sending the message and remember expected future sent timestamp
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 1);
    timePollAckReceived.getTimestamp(data + 6);
    timeRangeSent.getTimestamp(data + 11);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
    //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void loop() {
    if (inSleep) {
        if (millis() - lastActivity > resetSleepPeriod) {
            resetInactive();
        }
    }
    else {
        if (!sentAck && !receivedAck) {
            // check if inactive
            if (millis() - lastActivity > resetPeriod) {
                resetInactive();
            }
            return;
        }
        // continue on any success confirmation
        if (sentAck) {
            sentAck = false;
            byte msgId = data[0];
            if (msgId == POLL) {
                DW1000.getTransmitTimestamp(timePollSent);
                //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
            } else if (msgId == RANGE) {
                DW1000.getTransmitTimestamp(timeRangeSent);
                noteActivity();
            }
        }
        if (receivedAck) {
            receivedAck = false;
            // get message and parse
            DW1000.getData(data, LEN_DATA);
            byte msgId = data[0];
            if (msgId != expectedMsgId) {
                // unexpected message, start over again
                //Serial.print("Received wrong message # "); Serial.println(msgId);
                checkIMUandRetransmit();
                return;
            }
            if (msgId == POLL_ACK) {
                DW1000.getReceiveTimestamp(timePollAckReceived);
                expectedMsgId = RANGE_REPORT;
                transmitRange();
                noteActivity();
            } else if (msgId == RANGE_REPORT) {
                float curRange;
                memcpy(&curRange, data + 1, 4);
                uint32_t curr_time = micros();
                uint32_t wakeup_latency, transmission_latency;
                if(sleep_to_wake) {
                  // Serial.print("Last motion detected: ");
                  // Serial.print(last_motion_detected);
                  // Serial.print("\tCurr time: ");
                  // Serial.print(curr_time);
                  // Serial.print("\tLatency: ");
                  // Serial.println(curr_time - last_motion_detected);
                  wakeup_latency = transmit_poll_time - last_motion_detected;
                  transmission_latency = curr_time - transmit_poll_time;
                  Serial.print("Wakeup_latency:");
                  Serial.print(wakeup_latency);
                  Serial.print(",");
                  Serial.print("Transmision_latency:");
                  Serial.println(transmission_latency);
                }
                // Serial.print("Transmit poll time: ");
                // Serial.print(transmit_poll_time);
                // Serial.print("\tCurr time: ");
                // Serial.print(curr_time);
                // Serial.print("\tLatency: ");
                // Serial.println(curr_time - transmit_poll_time);
                
                checkIMUandRetransmit();
            } else if (msgId == RANGE_FAILED) {
                checkIMUandRetransmit();
            }
        }
    }
}
