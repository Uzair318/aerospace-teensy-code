/**
 * CAN Reciever Node 
 * Uzair Ahmed
 * Blinks built in LED x times where x is number received over CAN
 */
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; 
const ledPin = LED_BUILTIN;

void setup() {
  Serial.begin(115200); delay(400);
  can1.begin();
  can1.setBaudRate(250000);
  can1.setMB(MB11,RX,STD);
  can1.onReceive(MB11, canSniff)
}

void canSniff(const CANFD_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void loop() {
  can1.events();
}
