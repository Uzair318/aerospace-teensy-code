#include "CAN_interpreter.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_interpreter CAN_int;
char command[32];
void canSniff(const CAN_message_t &msg);

void setup() {
    Serial.begin(115200);
    can1.begin();
    can1.setBaudRate(250000);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.onReceive(FIFO, canSniff);

    CAN_int.startup(can1);
    Serial.println("CAN setup finished");
 
}

void loop() {
  can1.events();
  if (Serial.available()) {
    String input = Serial.readString();
    input.toCharArray(command, 32);
    CAN_message_t msg;
    int err = CAN_int.createMsg(command, &msg);   
    
    if(err > 0) {
      Serial.print("error:");
      Serial.println(err);
    } else {
      Serial.println("Sending...");
      CAN_int.interpretMsg(msg);
      can1.write(msg);
    }
  }
}

void canSniff(const CAN_message_t &msg) {
    Serial.println("Received...");
    CAN_int.interpretMsg(msg);
    CAN_int.setResponse(msg);
    CAN_int.newMessage = true;
//    Serial.println("Interrupted");
//    Serial.print("MB ");
//    Serial.print(msg.mb);
//    Serial.print(" OVERRUN: ");
//    Serial.print(msg.flags.overrun);
//    Serial.print(" LEN: ");
//    Serial.print(msg.len);
//    Serial.print(" EXT: ");
//    Serial.print(msg.flags.extended);
//    Serial.print(" TS: ");
//    Serial.print(msg.timestamp);
//    Serial.print(" ID: ");
//    Serial.print(msg.id, HEX);
//    Serial.print(" Buffer: ");
//    for ( uint8_t i = 0; i < msg.len; i++ ) {
//        Serial.print(msg.buf[i], HEX);
//        Serial.print(" ");
//    }
//    Serial.println();
}
