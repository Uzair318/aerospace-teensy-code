/**
 * CAN Reciever Node 
 * Uzair Ahmed
 * Blinks built in LED x times where x is number received over CAN
 */
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; 
const int ledPin = LED_BUILTIN;
CAN_message_t msg;

void setup() {
  Serial.begin(115200); delay(400);
  can1.begin();
  can1.setBaudRate(250000); 
  can1.setMB(MB11,RX,STD);
////  can1.onReceive(MB11, canSniff);
}

void loop() {
//  can1.mailboxStatus();
  if ( can1.read(msg) ) {
    Serial.print("CAN1 "); 
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
    Serial.print("  EXT: "); Serial.print(msg.flags.extended );
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for ( uint8_t i = 0; i < 8; i++ ) {
      Serial.print("0x"); Serial.print(msg.buf[i], HEX); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(msg.timestamp);
//    uint8_t blinkNum = msg.buf[0];
//    for(int i = 0; i < blinkNum; i++) {
//      digitalWrite(ledPin, HIGH);
//      delay(500);
//      digitalWrite(ledPin, LOW);
//      delay(200);
//    }
    delay(100);
    msg.id = MB11;
    for(uint8_t i = 0; i < 8; i++) {
        msg.buf[i] = i;
    }
    can1.write(MB9, msg);
    Serial.println("replying...");
    delay(400);
  }
}
