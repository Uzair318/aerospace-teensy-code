#include "CAN_interpreter.h"
// Create an IntervalTimer object 
IntervalTimer myTimer;
CAN_interpreter CAN_int(&canSniff);
char command[32];


#define MAX_TRAJECTORY 1500

bool ran = false;
bool newTrajectory = false;
uint16_t trajectoryLength;
int32_t trajectory[MAX_TRAJECTORY];
int count = 0; // counter for timer

void setup() {
  Serial.begin(9600);
  myTimer.begin(sendPoint, 10000);  // sendPoint to run at 100 Hz (in microseconds)
}

// The interrupt will blink the LED, and keep
// track of how many times it has blinked.

void sendPoint() {
  if(count < trajectoryLength) {
    //get data from trajectory array
    int32_t position = trajectory[count];
    String str_pos = String(position);
    String CANcommand = "607A,00,d" + str_pos + "w"; //create entire command (index, subindex, data in decimal format, write)
    CANcommand.toCharArray(CAN_int.input, length); // convert string to character array

     // send CAN message
     CAN_message_t message;
     int err = CAN_int.createMsg(command, &message);
     if(err > 0) {
       Serial.print("error:");
       Serial.println(err);
     }
     else {
       Serial.println("Sending...");
       Serial.println(position);
      //  CAN_int.interpretMsg(message);
      //  can1.write(message);
     }
    count++; // increase counter
  } // if
  else {
     myTimer.end(); // end timer
  } 
}

void loop() {
  String incomingData;
  char incomingByte;
  bool newMsg = false; //initially there is no new message
  while (Serial.available()) {
    incomingByte = Serial.read();
    // do something with incomingByte
    incomingData = incomingData + incomingByte;
    newMsg = true;
  }
  if(newMsg) { //only print if a new message has arrived
    float interval = incomingData.toFloat();
    Serial.println(interval);
    updateInterval(interval);
    newMsg = false;
  }
}

/*
 * Reset timer and counter
 */
void resetTimer() {
  count = 0;
  myTimer.begin(sendPoint, 10000);  // sendPoint to run at 100 Hz (in microseconds);
}


/**
 * updateInterval updates the period for the interrupt frequency
 * interval given in microseconds
 */
void updateInterval(float interval) {
  myTimer.update(interval);
}

void canSniff(const CAN_message_t &msg) {
    Serial.println("Received...");
    CAN_int.interpretMsg(msg);
    CAN_int.setResponse(msg);
    CAN_int.newMessage = true;
}
