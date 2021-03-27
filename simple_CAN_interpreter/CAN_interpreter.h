/*
    CAN interpreter library to take user input and create Flexcan4 messages for Maxon EPOS4 controller
    Alex Broz
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
    uint8_t createMsg(char * input_ptr,CAN_message_t * msg_ptr);
    void interpretMsg(CAN_message_t * msg_ptr);
  private:
    char _input[32];
    char _output[32];
    CAN_message_t _msg;
    char _temp[33];
    uint8_t _length = 0;
    uint32_t _data;

};

#endif