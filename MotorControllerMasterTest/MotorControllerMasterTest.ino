#include "CAN_interpreter.h"

#define AZ_ID 0x601
#define EL_ID 0x602
#define TIMER_LENGTH 5000 // 200 Hz

IntervalTimer myTimer;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_interpreter CAN_int_az<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>> (AZ_ID);
CAN_interpreter CAN_int_el<FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>>(EL_ID);

char command[32], input[32]; // command[] is used in loop()
uint16_t count = 0, count_az = 0, count_el = 0;

// for sendPoint
int32_t position;
String CANcommand;
CAN_message_t message;
uint16_t err;


void setup() {
    Serial.begin(115200);
    while(!Serial.available()){}  // pause until we want it to start

    CAN_int_az.can.setCAN(can1);

    CAN_int_az.can.begin();
    CAN_int_az.can.setBaudRate(250000);
    CAN_int_az.can.enableFIFO();
    CAN_int_az.can.enableFIFOInterrupt();
    CAN_int_az.can.onReceive(FIFO,canSniff);

    uint8_t startupResponse = CAN_int_az.startup();
    Serial.println("Azimuth CAN setup finished");
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    delay(500);

    startupResponse = CAN_int_el.startup();
    Serial.println("Elevation CAN setup finished");
    
    CAN_int_az.genTrajectory(2, true);
    CAN_int_el.genTrajectory(2, true);

    Serial.println("Trajectory Lengths: ");
    Serial.println(CAN_int_az.trajectoryLength);
    Serial.println(CAN_int_el.trajectoryLength);
    myTimer.begin(sendPoint, TIMER_LENGTH);  // sendPoint to run at 100 kHz (in microseconds)
    resetTimer();
}

void loop() {
  CAN_int_az.can.events();

  // if(Serial.available()){
  //   CAN_int_az.genTrajectory(0.2, true);
  //   Serial.println(CAN_int_az.trajectoryLength);
  //   Serial.println(CAN_int_az.T);
  //   Serial.println(CAN_int_az.Tc);
  //   // myTimer.begin(sendPoint, TIMER_LENGTH);  // sendPoint to run at 100 kHz (in microseconds)
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

void canSniff(const CAN_message_t &msg) {
    // Serial.println("Received...");
    // CAN_int_az.interpretMsg(msg);
    if(msg.id == AZ_ID) {
      CAN_int_az.setResponse(msg);
      CAN_int_az.newMessage = true;
    }
    else if(msg.id == EL_ID) {
      CAN_int_el.setResponse(msg);
      CAN_int_el.newMessage = true;
    }
}

void sendPoint() {
  // even number - send azimuth command
  if(count % 2 == 0 && count_az < CAN_int_az.trajectoryLength) {
    //get data from trajectory array
    position = CAN_int_az.trajectory[count_az];

    //create entire command (index, subindex, data in decimal format, write)
    CANcommand = "607A,00,d" + String(position) + "w";
    
    // convert string to character array
    CANcommand.toCharArray(input, CANcommand.length() + 1);

    // send CAN message
    // CAN_message_t message;
    err = CAN_int_az.createMsg(input, &message);

    if(err > 0) {
      Serial.print("error az:");
      Serial.println(err);
    }
    else{
      // Serial.print("Sending Position: ");
      // Serial.println(position_az);
      // CAN_int_az.interpretMsg(message_az);
      CAN_int_az.can.write(message);
    }
    
    count_az ++;
  }
  // odd number - send elevation command
  else if(count % 2 == 1 && count_el < CAN_int_el.trajectoryLength) {
    //get data from trajectory array
    position = CAN_int_el.trajectory[count_az];

    //create entire command (index, subindex, data in decimal format, write)
    CANcommand = "607A,00,d" + String(position) + "w";
    
    // convert string to character array
    CANcommand.toCharArray(input, CANcommand.length() + 1);

    // send CAN message
    // CAN_message_t message;
    err = CAN_int_el.createMsg(input, &message);

    if(err > 0) {
      Serial.print("error el:");
      Serial.println(err);
    }
    else{
      // Serial.print("Sending Position: ");
      // Serial.println(position_az);
      // CAN_int_az.interpretMsg(message_az);
      CAN_int_el.can.write(message);
    }
    
    count_el ++;
  }
  // both paths are done, disbale timer
  else {
    Serial.println("ending timer...");

    myTimer.end(); // end timer

    // get new message instead of whats still in CAN buffer
    CAN_int_az.newMessage = false;
    CAN_int_az.getPosition();
    CAN_int_el.newMessage = false;
    CAN_int_el.getPosition();
  }

  count ++;
}

/*
 * Reset timer and counter
 */
void resetTimer() {
  count = 0;
  myTimer.begin(sendPoint, TIMER_LENGTH);  // sendPoint to run at 100 Hz (in microseconds);
}

/*
 * updateInterval updates the period for the interrupt frequency
 * interval given in microseconds
 */
void updateInterval(float interval) {
  myTimer.update(interval);
}
