#include "CAN_interpreter.h"
IntervalTimer myTimer;
CAN_interpreter CAN_int(&canSniff);
char command[32];
uint16_t count = 0;

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

  // if(Serial.available()){
  //   CAN_int.genTrajectory(0.2, true);
  //   Serial.println(CAN_int.trajectoryLength);
  //   Serial.println(CAN_int.T);
  //   Serial.println(CAN_int.Tc);
  //   // myTimer.begin(sendPoint, 10000);  // sendPoint to run at 100 kHz (in microseconds)
  //   // resetTimer();
  //   Serial.flush();
  //   delay(2000);

  // }
//  if (Serial.available()) {
//    String input = Serial.readString();
//    input.toCharArray(command, 32);
//    CAN_message_t msg;
//    int err = CAN_int.createMsg(command, &msg);   
//    
//    if(err > 0) {
//      Serial.print("error:");
//      Serial.println(err);
//    } else {
//      Serial.println("Sending...");
//      CAN_int.interpretMsg(msg);
//      can1.write(msg);
//    }
//  }
}

void canSniff(const CAN_message_t &msg) {
    // Serial.println("Received...");
    // CAN_int.interpretMsg(msg);
    CAN_int.setResponse(msg);
    CAN_int.newMessage = true;
}

// The interrupt will blink the LED, and keep
// track of how many times it has blinked.

void sendPoint() {
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
      // Serial.print("Sending Position: ");
      // Serial.println(position);
      // CAN_int.interpretMsg(message);
      CAN_int.can.write(message);
    }
    // Serial.println(position);
    count++; // increase counter
  } // if
  else {
     Serial.println("ending timer...");
     Serial.print("ending position: ");
     CAN_int.getPosition();
     myTimer.end(); // end timer
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
