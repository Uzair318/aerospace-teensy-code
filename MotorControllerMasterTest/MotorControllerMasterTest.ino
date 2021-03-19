#include "Arduino.h"
#include "FlexCAN_T4.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void canSniff(const CAN_message_t &msg);

void setup() {
    Serial.begin(115200);
    can1.begin();
    can1.setBaudRate(250000);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.onReceive(FIFO, canSniff);
    delay(1000);
    Serial.println("CAN setup finished");
    delay(500);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'a') {
            Serial.println("Sending ...");
            CAN_message_t msg;
            msg.len = 8;
            msg.id = 0x0601;  // 0x600 + node_id (SDO client to server)
            msg.buf[0] = 40;    // ccs = 2 (read object)
            msg.buf[1] = 0x18;  // index low byte
            msg.buf[2] = 0x10;  // index high byte
            msg.buf[3] = 0x01;  // subindex
            msg.buf[4] = 0;
            msg.buf[5] = 0;
            msg.buf[6] = 0;
            msg.buf[7] = 0;

            can1.write(msg);
        }
    }
    delay(100);
}

void canSniff(const CAN_message_t &msg) {
    Serial.println("Interrupted");
    Serial.print("MB ");
    Serial.print(msg.mb);
    Serial.print(" OVERRUN: ");
    Serial.print(msg.flags.overrun);
    Serial.print(" LEN: ");
    Serial.print(msg.len);
    Serial.print(" EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print(" TS: ");
    Serial.print(msg.timestamp);
    Serial.print(" ID: ");
    Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}
