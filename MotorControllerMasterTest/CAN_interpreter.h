/*
    CAN interpreter library to take user input and create Flexcan4 messages for Maxon EPOS4 controller
    Alex Broz
    Uzair Ahmed
*/
#ifndef CAN_interpreter_h
#define CAN_interpreter_h

#include "Arduino.h"
#include "FlexCAN_T4.h"
#include <stdlib.h>



class CAN_interpreter
{
  public:
    CAN_interpreter();
    enum state_t{NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn, OperationEnabled,
      QuickStopActive, FaultReactionActive, Fault, notRecognized};
    state_t state;
    uint8_t createMsg(char *input_ptr,CAN_message_t *msg_ptr);
    void interpretMsg(CAN_message_t msg_ptr);
    uint8_t startup(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1);
    void setResponse(CAN_message_t msg_ptr);
    void setState(CAN_message_t &message);
    void awaitResponse();
    bool newMessage;

  private:
    CAN_message_t _res; // response that will be checked
    uint8_t err;
    //char _input[32];
    // CAN_message_t _msg;
    // char _temp[33];
    // uint8_t _length = 0;
    // uint32_t _data;
};
#endif