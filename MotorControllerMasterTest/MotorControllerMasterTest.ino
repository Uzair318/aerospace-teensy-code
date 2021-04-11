#include "CAN_interpreter.h"

IntervalTimer myTimer;
CAN_interpreter CAN_int_az(&canSniff_az), CAN_int_el(&canSniff_el);
char command[32], input_az[32], input_el[32]; // command[] is used in loop()
uint16_t count = 0;

// for sendPoint
int32_t position_az, position_el;
String CANcommand_az, CANcommand_el;
CAN_message_t message_az, message_el;
uint16_t err_az, err_el;


void setup() {
    Serial.begin(115200);
    while(!Serial.available()){}  // pause until we want it to start

    uint8_t startupResponse = CAN_int_az.startup();
    Serial.println("Azimuth CAN setup finished");

    startupResponse = CAN_int_el.startup();
    Serial.println("Elevation CAN setup finished");
    
    CAN_int_az.genTrajectory(2, true);
    CAN_int_el.genTrajectory(2, true);

    Serial.println("Trajectory Lengths: ");
    Serial.println(CAN_int_az.trajectoryLength);
    Serial.println(CAN_int_el.trajectoryLength);
    myTimer.begin(sendPoint, 10000);  // sendPoint to run at 100 kHz (in microseconds)
    // resetTimer();
}

void loop() {
  CAN_int_az.can.events();

  // if(Serial.available()){
  //   CAN_int_az.genTrajectory(0.2, true);
  //   Serial.println(CAN_int_az.trajectoryLength);
  //   Serial.println(CAN_int_az.T);
  //   Serial.println(CAN_int_az.Tc);
  //   // myTimer.begin(sendPoint, 10000);  // sendPoint to run at 100 kHz (in microseconds)
  //   // resetTimer();
  //   Serial.flush();
  //   delay(2000);

  // }
//  if (Serial.available()) {
//    String input = Serial.readString();
//    input.toCharArray(command, 32);
//    CAN_message_t msg;
//    int err = CAN_int_az.createMsg(command, &msg);   
//    
//    if(err > 0) {
//      Serial.print("error:");
//      Serial.println(err);
//    } else {
//      Serial.println("Sending...");
//      CAN_int_az.interpretMsg(msg);
//      can1.write(msg);
//    }
//  }
}

void canSniff_az(const CAN_message_t &msg) {
    // Serial.println("Received...");
    // CAN_int_az.interpretMsg(msg);
    CAN_int_az.setResponse(msg);
    CAN_int_az.newMessage = true;
}

void canSniff_el(const CAN_message_t &msg) {
    // Serial.println("Received...");
    // CAN_int_az.interpretMsg(msg);
    CAN_int_el.setResponse(msg);
    CAN_int_el.newMessage = true;
}

void sendPoint() {
  if(count < CAN_int_az.trajectoryLength || count < CAN_int_el.trajectoryLength) {
    //get data from trajectory array
    position_az = CAN_int_az.trajectory[count];
    position_el = CAN_int_el.trajectory[count];
    // String str_pos = String(position);

    //create entire command (index, subindex, data in decimal format, write)
    CANcommand_az = "607A,00,d" + String(position_az) + "w";
    CANcommand_el = "607A,00,d" + String(position_el) + "w";
    // int length = CANcommand.length() + 1;
    // char input[length];

    // convert string to character array
    CANcommand_az.toCharArray(input_az, CANcommand_az.length() + 1);
    CANcommand_el.toCharArray(input_el, CANcommand_el.length() + 1);

    // send CAN message
    // CAN_message_t message;
    err_az = CAN_int_az.createMsg(input_az, &message_az);
    err_el = CAN_int_el.createMsg(input_el, &message_el);
    if(err_az > 0 || err_el > 0) {
      Serial.print("error az:");
      Serial.println(err_az);
      Serial.print("error el:");
      Serial.println(err_el);
    }
    else{
      if(count < CAN_int_az.trajectoryLength){
        // Serial.print("Sending Position: ");
        // Serial.println(position);
        // CAN_int_az.interpretMsg(message);
        CAN_int_az.can.write(message_az);
      }
      if(count < CAN_int_el.trajectoryLength){
        // Serial.print("Sending Position: ");
        // Serial.println(position);
        // CAN_int_az.interpretMsg(message);
        CAN_int_el.can.write(message_el);
      }
    }
    
    // Serial.println(position);
    count++; // increase counter
  } // if
  else {
    Serial.println("ending timer...");

    myTimer.end(); // end timer

    // get new message instead of whats still in CAN buffer
    CAN_int_az.newMessage = false;
    CAN_int_az.getPosition();
    CAN_int_el.newMessage = false;
    CAN_int_el.getPosition();
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
