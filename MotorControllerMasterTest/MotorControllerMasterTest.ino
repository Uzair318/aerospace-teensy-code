#include "CAN_interpreter.h"
IntervalTimer myTimer;
CAN_interpreter CAN_int(&canSniff);
char command[32];
uint16_t count = 0;
unsigned long before;
unsigned long after;
unsigned long diff = 0;
uint16_t cycles = 0;

void setup() {
    Serial.begin(115200);
    while(!Serial.available()){}  // pause until we want it to start
    uint8_t startupResponse = CAN_int.startup();
    Serial.println("CAN setup finished");
    CAN_int.genTrajectory(2, true);
    Serial.print("Trajectory Length: ");
    Serial.println(CAN_int.trajectoryLength);
    myTimer.begin(sendPoint, 10000);  // sendPoint to run at 100 kHz (in microseconds)
    // resetTimer();
}

void loop() {
  CAN_int.can.events();
}

void canSniff(const CAN_message_t &msg) {
    CAN_int.setResponse(msg);
    CAN_int.newMessage = true;
}

// The interrupt will blink the LED, and keep
// track of how many times it has blinked.

void sendPoint() {
  before = micros();
  if(count < CAN_int.trajectoryLength) {
    //get data from trajectory array
    int32_t position = CAN_int.trajectory[count];
    String str_pos = String(position);
    String CANcommand = "607A,00,d" + str_pos + "w"; //create entire command (index, subindex, data in decimal format, write)
    int length = CANcommand.length() + 1;
    char input[length];
    CANcommand.toCharArray(input, length); // convert string to character array

    // send CAN message
    CAN_message_t message;
    int err = CAN_int.createMsg(input, &message);
    if(err > 0) {
      Serial.print("error:");
      Serial.println(err);
    }
    else {
      CAN_int.can.write(message);
    }
    // Serial.println(position);
    count++; // increase counter
    after = micros();
    Serial.println(after - before);
    diff += after - before;
    cycles ++;
  } // if
  else {
    Serial.println("ending timer...");

    myTimer.end(); // end timer
    CAN_int.newMessage = false; // get new message instead of whats still in CAN buffer
    CAN_int.getPosition();
    Serial.print("Average interrupt time (ms): ");
    Serial.println((float) diff / (float) count);
  }
}

/*
 * Reset timer and counter
 */
void resetTimer() {
  count = 0;
  myTimer.begin(sendPoint, 10000);  // sendPoint to run at 100 Hz (in microseconds);
}

/*
 * updateInterval updates the period for the interrupt frequency
 * interval given in microseconds
 */
void updateInterval(float interval) {
  myTimer.update(interval);
}
