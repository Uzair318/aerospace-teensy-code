#include "CAN_interpreter.h"

// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_interpreter CAN_int(&canSniff);//(can1);
char command[32];
// void canSniff(const CAN_message_t &msg);

void setup() {
    Serial.begin(115200);
    // can1.begin();
    // can1.setBaudRate(250000);
    // can1.enableFIFO();
    // can1.enableFIFOInterrupt();
    // can1.onReceive(FIFO, canSniff);
    while(!Serial.available()){}
    uint8_t startupResponse = CAN_int.startup();
    Serial.println("CAN setup finished");
 
}

void loop() {
//   can1.events();
  CAN_int.can.events();
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
    Serial.println("Received...");
    CAN_int.interpretMsg(msg);
    CAN_int.setResponse(msg);
    CAN_int.newMessage = true;
}
