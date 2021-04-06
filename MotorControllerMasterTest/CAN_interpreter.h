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
#include <math.h>

#define MAX_TRAJECTORY 1500
#define RAD_TO_TICKS 102943.7081

class CAN_interpreter
{
  public:
    CAN_interpreter(void (*cb)());//FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can1);
    enum state_t{NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn, OperationEnabled,
      QuickStopActive, FaultReactionActive, Fault, notRecognized};
    state_t state;
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
    uint8_t createMsg(char *input_ptr,CAN_message_t *msg_ptr);
    void interpretMsg(CAN_message_t msg_ptr);
    uint8_t startup();
    void setResponse(CAN_message_t msg_ptr);
    void getState(CAN_message_t &message);
    void awaitResponse();
    void setTrajectoryParams(double f, double v, double a);
    uint8_t genTrajectory(double target_rad, bool absolute);
    bool newMessage;
    int32_t trajectory[MAX_TRAJECTORY];
    uint16_t trajectoryLength;
    int32_t position;
    int32_t target;    // encoder increments
    char input[32];

  private:
    void canSniff(const CAN_message_t &msg);
    CAN_message_t _res; // response that will be checked
    uint8_t err;
    double freq = 100; // Hz
    double maxV = .2;  // rad/s
    double maxA = .4;  // rad/s^2

    // for trajectory generation
    double T;
    double t = 0;
    double Tc;
    double T_pi2;
    double pi_T;
};



#endif
