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
  can1.setMB(MB11,TX);
  
  
}

void loop() {
//  for ( uint8_t i = 0; i < msg.len; i++ ) {
//    Serial.print(msg.buf[i], DEC); Serial.print(" ");
//  } Serial.println();
  if (Serial.available() > 0) {
    blinkNumber = Serial.parseInt();
    msg.buf[0] = blinkNumber;
    can1.write(MB9, msg);
  }
}
