#include "CAN_interpreter.h"

IntervalTimer myTimer;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_interpreter<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>> CAN_int_az(&canSniff_az, can1);
CAN_interpreter<FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>> CAN_int_el(&canSniff_el, can2);

char command[32], input_az[32], input_el[32]; // command[] is used in loop()
uint16_t count = 0, finished_count = 0;

// for sendPoint
int32_t position_az, position_el;
String CANcommand_az, CANcommand_el;
CAN_message_t message_az, message_el;
uint16_t err_az, err_el;
bool el_reached = true, az_reached = false;

void setup() {
    Serial.begin(115200);
    while(!Serial.available()){}  // pause until we want it to start
    Serial.println("Starting");

    uint8_t startupResponse = CAN_int_az.startup();
    Serial.println("Azimuth CAN setup finished");
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    delay(500);

    startupResponse = CAN_int_el.startup();
    Serial.println("Elevation CAN setup finished");
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();

    // sendSinglePoint(-1,-1);
    Serial.println("Point 1");
    sendSinglePoint(.3,0);
    count = 0;
    myTimer.begin(poll, 10000);  // poll to run at 100 Hz (in microseconds)
    delay(10000);

    Serial.println("Point 2");
    sendSinglePoint(0,.3);
    count = 0;
    myTimer.begin(poll, 10000);  // poll to run at 100 Hz (in microseconds)
    delay(4000);

    Serial.println("Point 3");
    sendSinglePoint(-.3,0);
    count = 0;
    myTimer.begin(poll, 10000);  // poll to run at 100 Hz (in microseconds)
    delay(4000);

    Serial.println("Point 4");
    sendSinglePoint(0,-.3);
    count = 0;
    myTimer.begin(poll, 10000);  // poll to run at 100 Hz (in microseconds)
    // delay(5000);

    // CAN_int_az.newMessage = false;
    // CAN_int_az.getPosition();
    // CAN_int_el.newMessage = false;
    // CAN_int_el.getPosition();
}

void loop() {
  CAN_int_az.can.events();
  CAN_int_el.can.events();

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

/**
 * poll both controllers until they are done moving
 * send status to see if it is done, then send position request to get current position
 */
void poll() {
  // Send status requests
  myTimer.end();
  CANcommand_az = "6041,00,x0000r";
  CANcommand_el = "6041,00,x0000r";
  CANcommand_az.toCharArray(input_az, CANcommand_az.length() + 1);
  CANcommand_el.toCharArray(input_el, CANcommand_el.length() + 1);
  err_az = CAN_int_az.createMsg(input_az, &message_az);
  err_el = CAN_int_el.createMsg(input_el, &message_el);
  if(err_az > 0 || err_el > 0) {
    Serial.print("error az traj:");
    Serial.println(err_az);
    Serial.print("error el traj:");
    Serial.println(err_el);
  }
  else{
    CAN_int_az.can.write(message_az);
    CAN_int_el.can.write(message_el);
  }

  CAN_int_az.awaitResponse();
  CAN_int_el.awaitResponse();

  el_reached = CAN_int_el.targetReach();
  az_reached = CAN_int_az.targetReach();

  CAN_int_az.newMessage = false;
  CAN_int_az.getPosition();
  CAN_int_el.newMessage = false;
  CAN_int_el.getPosition();

  CAN_int_az.trajectory[count] = CAN_int_az.position;
  CAN_int_el.trajectory[count] = CAN_int_el.position;

  if (el_reached && az_reached && finished_count == 0) {
    finished_count = count;
    count ++;
    myTimer.begin(poll,10000);
  }
  else if(count > 200) {
    // print both trajectories in Matlab input format
    Serial.print("trajectory = [");
    for (uint16_t m = 0; m < count; m++){
      Serial.print(CAN_int_az.trajectory[m]);
      Serial.print(",");
      Serial.print(CAN_int_el.trajectory[m]);
      Serial.println(";");
    }
    Serial.print(CAN_int_az.trajectory[count]);
    Serial.print(",");
    Serial.print(CAN_int_el.trajectory[count]);
    Serial.println("];");
  } else {
    count ++;
    myTimer.begin(poll,10000);
  }
}

/**
 * params: az and el angles in rad
 * needs to:
 *  - convert them to ticks
 *  - create commands
 *  - send single point to each controller
 */ 
void sendSinglePoint(double angle_az, double angle_el) {
  // convert angles to number of encoder ticks
  position_az = (int32_t) (angle_az * RAD_TO_TICKS);
  position_el = (int32_t) (angle_el * RAD_TO_TICKS);

  Serial.print("input_az = ");
  Serial.println(position_az);
  Serial.print("input_el = ");
  Serial.println(position_el);

  //create entire command (index, subindex, data in decimal format, write)
  CANcommand_az = "607A,00,d" + String(position_az) + "w";
  CANcommand_el = "607A,00,d" + String(position_el) + "w";

  // convert string to character array
  CANcommand_az.toCharArray(input_az, CANcommand_az.length() + 1);
  CANcommand_el.toCharArray(input_el, CANcommand_el.length() + 1);

  // send CAN position
  // CAN_message_t message;
  err_az = CAN_int_az.createMsg(input_az, &message_az);
  err_el = CAN_int_el.createMsg(input_el, &message_el);
  if(err_az > 0 || err_el > 0) {
    Serial.print("error az traj:");
    Serial.println(err_az);
    Serial.print("error el traj:");
    Serial.println(err_el);
  }
  else{
    CAN_int_az.can.write(message_az);
    CAN_int_el.can.write(message_el);
  }
  
  CAN_int_az.awaitResponse();
  CAN_int_el.awaitResponse();

  // send CAN contorlword
  // CAN_message_t message;           // FEDCBA9786543210    
  err_az = CAN_int_az.createMsg("6040,00,b000000001111111w", &message_az);
  err_el = CAN_int_el.createMsg("6040,00,b000000001111111w", &message_el);
  if(err_az > 0 || err_el > 0) {
    Serial.print("error az:");
    Serial.println(err_az);
    Serial.print("error el:");
    Serial.println(err_el);
  }
  else{
    CAN_int_az.can.write(message_az);
    CAN_int_el.can.write(message_el);
  }
  
  CAN_int_az.awaitResponse();
  CAN_int_el.awaitResponse();

  // send CAN controlword - activate new setpoint
  // CAN_message_t message;           // FEDCBA9786543210    
  err_az = CAN_int_az.createMsg("6040,00,b000000001101111w", &message_az);
  err_el = CAN_int_el.createMsg("6040,00,b000000001101111w", &message_el);
  if(err_az > 0 || err_el > 0) {
    Serial.print("error az:");
    Serial.println(err_az);
    Serial.print("error el:");
    Serial.println(err_el);
  }
  else{
    CAN_int_az.can.write(message_az);
    CAN_int_el.can.write(message_el);
  }
  
  CAN_int_az.awaitResponse();
  // CAN_int_el.awaitResponse();
}

/*
 * Reset timer and counter
 */
void resetTimer() {
  count = 0;
  myTimer.begin(poll, 10000);  // poll to run at 100 Hz (in microseconds);
}

/*
 * updateInterval updates the period for the interrupt frequency
 * interval given in microseconds
 */
void updateInterval(float interval) {
  myTimer.update(interval);
}
