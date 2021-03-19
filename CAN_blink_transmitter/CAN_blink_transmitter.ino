/**
 * CAN Transmitter Node 
 * Uzair Ahmed
 * Blinks built in LED x times where x is number received over CAN
 */
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; 
CAN_message_t msg;
uint8_t blinkNumber;

void setup() {
  Serial.begin(115200); delay(400);
  can1.begin();
  can1.setBaudRate(250000);
  can1.setMB(MB9,TX);

  blinkNumber = 5;
  for(uint8_t i = 0; i < 8; i++) {
    msg.buf[i] = i;
  }
  msg.buf[0] = blinkNumber;
  can1.write(MB9, msg);
  can1.mailboxStatus();
  Serial.write("Sent ");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'a') {
      blinkNumber = 5;//Serial.parseInt();
      msg.id = MB11;
      for(uint8_t i = 0; i < 8; i++) {
        msg.buf[i] = i;
      }
      msg.buf[0] = blinkNumber;
      can1.write(MB9, msg);
      can1.mailboxStatus();
    }
  }
}
