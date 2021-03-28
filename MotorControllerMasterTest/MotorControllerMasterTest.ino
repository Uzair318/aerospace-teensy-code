#include "CAN_interpreter.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_Interpreter CAN_int;
char[32] command;
void canSniff(const CAN_message_t &msg);

void setup() {
    Serial.begin(115200);
    can1.begin();
    can1.setBaudRate(250000);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.onReceive(FIFO, canSniff);
    
//    delay(1000);
    Serial.println("CAN setup finished");
//    delay(500);
  

}

void loop() {
  can1.events();
    if (Serial.available()) {
      String input = Serial.readString();
      input.toCharArray(command, 32);
      CAN_message_t msg;
      int err = CAN_int.createMsg(input, &msg);
//        char c = Serial.read();
//        if (c == 'a') {
//            Serial.println("Sending ...");
//            CAN_message_t msg;
//            
//            msg.len = 8;
//            // Write «Target position» (Index 0x607A, Subindex 0x00: Data 0x000008AE  2222dec) to node 1:
//            msg.id = 0x601;  // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x22;    // ccs = 2 (read object)
//            msg.buf[1] = 0x7A;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0x00;  // subindex
//            msg.buf[4] = 0x00;
//            msg.buf[5] = 0x7D;
//            msg.buf[6] = 0x00;
//            msg.buf[7] = 0x00;
//
//            can1.write(msg);
//            c = 'z';
//        } else if(c == 'b') {
//            Serial.println("Sending ...");
//            CAN_message_t msg;
//            msg.len = 8;
//            // Read «Position actual value» (Index 0x6064, Subindex 0x00) from node 1:
//            msg.id = 0x601;  // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x40;    // ccs = 2 (read object)
//            msg.buf[1] = 0x64;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0x00;  // subindex
//            msg.buf[4] = 0x00;
//            msg.buf[5] = 0x00;
//            msg.buf[6] = 0x00;
//            msg.buf[7] = 0x00;
//            can1.write(msg);
//            c = 'z';
//        } else if(c == 's') {
//            Serial.println("Sending setup command");
//            CAN_message_t msg;
//            msg.len = 8;
//            // 
//            msg.id = 0x0601;  // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x22;    // ccs = 1 (write object)
//            msg.buf[1] = 0x40;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0x00;  // subindex
//            msg.buf[4] = 0b1011;//0x0F;
//            msg.buf[5] = 0;
//            msg.buf[6] = 0;
//            msg.buf[7] = 0;
//            can1.write(msg);
//
//            Serial.println("Check status of controller");
//            msg.len = 8;
//            // 
//            msg.id = 0x0601;      // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x40;    // ccs = 2 (read object)
//            msg.buf[1] = 0x41;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0;     // subindex
//            msg.buf[4] = 0;
//            msg.buf[5] = 0;
//            msg.buf[6] = 0;
//            msg.buf[7] = 0;
//            can1.write(msg);
//        } else if(c == 'x') {
//            Serial.println("Sending SHUTDOWN command");
//            CAN_message_t msg;
//            msg.len = 8;
//            // 
//            msg.id = 0x0601;  // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x22;    // ccs = 1 (write object)
//            msg.buf[1] = 0x40;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0x00;  // subindex
//            msg.buf[4] = 0;
//            msg.buf[5] = 0;
//            msg.buf[6] = 0;
//            msg.buf[7] = 0;
//            can1.write(msg);
//            
//            Serial.println("Check status of controller");
//            msg.len = 8;
//            // 
//            msg.id = 0x0601;      // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x40;    // ccs = 2 (read object)
//            msg.buf[1] = 0x41;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0;     // subindex
//            msg.buf[4] = 0;
//            msg.buf[5] = 0;
//            msg.buf[6] = 0;
//            msg.buf[7] = 0;
//            can1.write(msg);
//        } else if(c == 'v') {
//            Serial.println("Sending Velocity Demand Value");
//            CAN_message_t msg;
//            msg.len = 8;
////            // Send Velocity Demand Value
////            msg.id = 0x601;  // 0x600 + node_id (SDO client to server)
////            msg.buf[0] = 0x22;    // ccs = 1 (write object)
////            msg.buf[1] = 0x6B;  // index low byte
////            msg.buf[2] = 0x60;  // index high byte
////            msg.buf[3] = 0x00;  // subindex
////            msg.buf[4] = 0x64;
////            msg.buf[5] = 0x00;
////            msg.buf[6] = 0x00;
////            msg.buf[7] = 0x00;
////            can1.write(msg);
////            c = 'z';
//            // Send Velocity Demand Value
//            msg.id = 0x601;  // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x22;    // ccs = 1 (write object)
//            msg.buf[1] = 0xFF;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0x00;  // subindex
//            msg.buf[4] = 0xE8;
//            msg.buf[5] = 0x03;
//            msg.buf[6] = 0x00;
//            msg.buf[7] = 0x00;
//            can1.write(msg);
//            c = 'z';
//        } else if(c == 'r') {
//            Serial.println("Sending ...");
//            CAN_message_t msg;
//            msg.len = 8;
//            // Get Velocity Actual Value
//            msg.id = 0x601;  // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x40;    // ccs = 2 (read object)
//            msg.buf[1] = 0x6C;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0x00;  // subindex
//            msg.buf[4] = 0x00;
//            msg.buf[5] = 0x00;
//            msg.buf[6] = 0x00;
//            msg.buf[7] = 0x00;
//            can1.write(msg);
//            c = 'z';
//        } else if(c == 'm') {
//            Serial.println("Sending request for Mode of Operation");
//            CAN_message_t msg;
//            msg.len = 8;
//            // Mode of Operation
//            msg.id = 0x601;  // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x40;    // ccs = 2 (read object)
//            msg.buf[1] = 0x61;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0x00;  // subindex
//            msg.buf[4] = 0x00;
//            msg.buf[5] = 0x00;
//            msg.buf[6] = 0x00;
//            msg.buf[7] = 0x00;
//            can1.write(msg);
//            c = 'z';
//        } else if(c == 'n') {
//            Serial.println("Setting Mode of Operation to Velocity");
//            CAN_message_t msg;
//            msg.len = 8;
//            // Mode of Operation
//            msg.id = 0x601;  // 0x600 + node_id (SDO client to server)
//            msg.buf[0] = 0x22;    // ccs = 2 (read object)
//            msg.buf[1] = 0x60;  // index low byte
//            msg.buf[2] = 0x60;  // index high byte
//            msg.buf[3] = 0x00;  // subindex
//            msg.buf[4] = 0x03;
//            msg.buf[5] = 0x00;
//            msg.buf[6] = 0x00;
//            msg.buf[7] = 0x00;
//            can1.write(msg);
//            c = 'z';
        }
    }
}

void canSniff(const CAN_message_t &msg) {
  CAN_int.interpretMsg(&msg);
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
