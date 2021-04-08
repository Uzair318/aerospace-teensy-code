/*
    CAN interpreter library to take user input and create Flexcan4 messages for Maxon EPOS4 controller
    Alex Broz, Uzair Ahmed
    Input format is INDEX,  SUBINDEX,DATA,w/r

    INDEX and SUBINDEX in hexadecimal, chars 1-4
    DATA format specified by b, x, or d for binary, hex, and decimal, chars 5 and 6
        indicator letter on char 7
    w/r for read or write  - case doesn't matter

    ex: 6061,00,x12345678w
        ACD1,42,b0000011011011011
        1111,FF,d64321
*/

#include "CAN_interpreter.h"

CAN_interpreter::CAN_interpreter(void (*cb)()){
    can.begin();
    can.setBaudRate(250000);
    can.enableFIFO();
    can.enableFIFOInterrupt();
    can.onReceive(FIFO, *cb);
}

// void CAN_interpreter::canSniff(const CAN_message_t &msg) {
//     Serial.println("Received...");
//     this->interpretMsg(msg);
//     this->setResponse(msg);
//     this->newMessage = true;
// }


// Create a FLEXCAN_T4 message from char array input
uint8_t CAN_interpreter::createMsg(char _input[],CAN_message_t *msg_ptr){
    //_input = input_ptr;
    CAN_message_t _msg;
    char _temp[33];
    uint8_t _length = 0;
    uint32_t _data;

    _msg.id = 0x0601;  // 0x600 + node_id (SDO client to server)
    
    //check for valid comma placement
    if(_input[4] != ',' || _input[7] != ','){
        return 1;
    }

    //index low byte - chars 3 and 4
    _temp[0] = _input[2];
    _temp[1] = _input[3];
    _temp[3] = '\0';
    _msg.buf[1] = (uint8_t) strtol(_temp, nullptr, 16);
    
    //index high byte - chars 1 and 2
    _temp[0] = _input[0];
    _temp[1] = _input[1];
    _temp[3] = '\0';
    _msg.buf[2] = (uint8_t) strtol(_temp, nullptr, 16);

    //subindex - chars 5 and 6
    _temp[0] = _input[5];
    _temp[1] = _input[6];
    _temp[3] = '\0';
    _msg.buf[3] = (uint8_t) strtol(_temp, nullptr, 16);

    //determine data length and read/write
    _length = 0;
    
    //determine input length and create substring for conversion
    bool end = false;
    while(_input[_length + 9] != '\0' && !end){
        switch(_input[_length + 9]){
            //write
            case 'w':
                end = true;
                _temp[_length] = '\0'; //terminate string
                _msg.buf[0] = 0x22;
                break;
            case 'W':
                end = true;
                _temp[_length] = '\0'; //terminate string
                _msg.buf[0] = 0x22;
                break;

            //read
            case 'r':
                end = true;
                _temp[_length] = '\0'; //terminate string
                _msg.buf[0] = 0x40;
                break;
            case 'R':
                end = true;
                _temp[_length] = '\0'; //terminate string
                _msg.buf[0] = 0x40;
                break;

            //still data
            default:
                _temp[_length] = _input[_length + 9]; //add to substring
                _length ++;
        }
    }

    //determine data type
    switch(_input[8]){
        //hex
        case 'x':
            _data = strtol(_temp,nullptr,16);
            break;
        case 'X':
            _data = strtol(_temp,nullptr,16);
            break;

        //decimal
        case 'd':
            _data = strtol(_temp,nullptr,10);
            break;
        case 'D':
            _data = strtol(_temp,nullptr,10);
            break;

        //binary
        case 'b':
            _data = strtol(_temp,nullptr,2);
            break;
        case 'B':
            _data = strtol(_temp,nullptr,2);
            break;
        
        //none of the above means error
        default:
            return 2;
    }


    //convert data to big endian and add to message
    _msg.buf[4] = (uint8_t)((_data & 0x000000ff));
    _msg.buf[5] = (uint8_t)((_data & 0x0000ff00) >> 8u);
    _msg.buf[6] = (uint8_t)((_data & 0x00ff0000) >> 16u);
    _msg.buf[7] = (uint8_t)((_data & 0xff000000) >> 24u);

    //finished
    *msg_ptr = _msg;
    return 0;
}

// Print all info from FLEXCAN_T4 message
void CAN_interpreter::interpretMsg(CAN_message_t message){
    CAN_message_t _msg;
    uint32_t _data;

    _msg = message;

    //message data to little endian
    _data = (uint32_t) _msg.buf[4];
    _data += (uint32_t) _msg.buf[5] << 8u;
    _data += (uint32_t) _msg.buf[6] << 16u;
    _data += (uint32_t) _msg.buf[7] << 24u;

    //print message info
    Serial.print("ID: ");
    Serial.print(_msg.id,HEX);
    Serial.print("\tW/R: ");
    Serial.print(_msg.buf[0], HEX);
    Serial.print("\tIndex: ");
    Serial.print(_msg.buf[2], HEX);
    Serial.print(_msg.buf[1], HEX);
    Serial.print("\tSubindex: ");
    Serial.println(_msg.buf[3], HEX);

    //print all data as hex
    Serial.print("HEX: ");
    Serial.print(_msg.buf[7],HEX);
    Serial.print(" ");
    Serial.print(_msg.buf[6],HEX);
    Serial.print(" ");
    Serial.print(_msg.buf[5],HEX);
    Serial.print(" ");
    Serial.println(_msg.buf[4],HEX);

    //print low 2 bytes as binary
    Serial.print("BIN: ");
    Serial.print(_msg.buf[5],BIN);
    Serial.print(" ");
    Serial.println(_msg.buf[4],BIN);

    //print all data as int
    Serial.print("DEC: ");
    Serial.println(_data);
    Serial.println();
}

// Run setup commands for controller
uint8_t CAN_interpreter::startup(){
    CAN_message_t msg;
    Serial.println("Running setup...");

    // send statusword   
    Serial.println("Checking Statusword...");
    // input = "6041,00,x0000r";  // statusword
    strcpy(input, "6041,00,x0000r");
    err = this->createMsg(input, &msg);
    if(err > 0) {
        Serial.print("error before while loop in CAN_interpreter.startup():");
        Serial.println(err);
        return 1;
      } else {
        Serial.println("Sending...");
        // this->interpretMsg(msg);
        can.write(msg);
        this->awaitResponse();
        // Serial.println("waiting for response....");
        // delay(1000);
        // can.events();
        // this->interpretMsg(_res);
    }

    

    // check the response and update the state
    // this->interpretMsg(_res);
    this->getState(_res);

    // while statusword is not "operation enabled" 
    while(state != OperationEnabled) {
        // transition from the current state to the next state (and verify?)
        switch(state) {
            case NotReadyToSwitchOn:
                Serial.println("Not Ready To Switch On");
                strcpy(input, "6040,00,b0001w"); 
                err = this->createMsg(input, &msg);
                if(err > 0) {
                    Serial.print("error in NotReadyToSwitchOn case of CAN_interpreter.startup():");
                    Serial.println(err);
                    return 1;
                } else {
                    Serial.println("Sending controlword to execute transition 1...");
                    // this->interpretMsg(msg);
                    can.write(msg);
                    this->awaitResponse();
                }
                break;
            case SwitchOnDisabled:
                Serial.println("Switch On Disabled");
                strcpy(input, "6040,00,b0110w");
                err = this->createMsg(input, &msg);
                if(err > 0) {
                    Serial.print("error in SwitchOnDisabled case of CAN_interpreter.startup():");
                    Serial.println(err);
                    return 1;
                } else {
                    Serial.println("Sending controlword to execute transition 2...");
                    // this->interpretMsg(msg);
                    can.write(msg);
                    this->awaitResponse();
                } 
                break;
            case ReadyToSwitchOn:
                Serial.println("Ready to Switch On");
                strcpy(input, "6040,00,b0111w");
                err = this->createMsg(input, &msg);
                if(err > 0) {
                    Serial.print("error in ReadyToSwitchOn case of CAN_interpreter.startup():");
                    Serial.println(err);
                    return 1;
                } else {
                    Serial.println("Sending controlword to execute transition 3...");
                    // this->interpretMsg(msg);
                    can.write(msg);
                    this->awaitResponse();
                }
                break;
            case SwitchedOn:
                Serial.println("Switched On");
                strcpy(input, "6040,00,b1111w");
                err = this->createMsg(input, &msg);
                if(err > 0) {
                    Serial.print("error in SwitchedOn case of CAN_interpreter.startup():");
                    Serial.println(err);
                    return 1;
                } else {
                    Serial.println("Sending controlword to execute transition 4...");
                    // this->interpretMsg(msg);
                    can.write(msg);
                    this->awaitResponse();
                }
                break;
            case QuickStopActive:
                Serial.println("Quick Stop Active");
                strcpy(input, "6040,00,b1111w");
                err = this->createMsg(input, &msg);
                if(err > 0) {
                    Serial.print("error in QuickStopActive case of CAN_interpreter.startup():");
                    Serial.println(err);
                    return 1;
                } else {
                    Serial.println("Sending controlword to execute transition 16...");
                    // this->interpretMsg(msg);
                    can.write(msg);
                    this->awaitResponse();
                }
                break;
            case FaultReactionActive:
                Serial.println("Fault Reaction Active");
                strcpy(input, "6040,00,b00000000w");
                err = this->createMsg(input, &msg);
                if(err > 0) {
                    Serial.print("error in FaultReactionActive case of CAN_interpreter.startup():");
                    Serial.println(err);
                    return 1;
                } else {
                    Serial.println("Sending controlword to execute transition 14...");
                    // this->interpretMsg(msg);
                    can.write(msg);
                    this->awaitResponse();
                }
                break;
            case Fault:
                Serial.println("Fault");
                strcpy(input, "6040,00,b10000000w");
                err = this->createMsg(input, &msg);
                if(err > 0) {
                    Serial.print("error in Fault case of CAN_interpreter.startup():");
                    Serial.println(err);
                    return 1;
                } else {
                    Serial.println("Sending controlword to execute transition 15...");
                    // this->interpretMsg(msg);
                    can.write(msg);
                    this->awaitResponse();
                }
                break;
            default:
                Serial.println();
            
        }

        // send statusword   
        // Serial.println("Checking Statusword...");
        strcpy(input, "6041,00,x0000r");
        err = this->createMsg(input, &msg);
        if(err > 0) {
            Serial.print("error within while loop in CAN_interpreter.startup():");
            Serial.println(err);
            return 1;
        } else {
            // Serial.println("Sending...");
            // this->interpretMsg(msg);
            can.write(msg);
            this->awaitResponse();
        }

        // check the response and update the state
        // this->interpretMsg(_res);
        this->getState(_res);
    }

    Serial.println("Setup Complete: Controller in Operation Enabled state");
    return 0;
}

// void CAN_interpreter::setHomePosition(){
//     strcpy(input, "30B0,00,x0000w"); // set home command
//     err = this->createMsg(input, &msg);
//     if(err > 0) {
//         Serial.print("error setting home position");
//         Serial.println(err);
//         return 1;
//     } else {
//         can.write(msg);
//         this->awaitResponse();
//     }
// }

// set the incoming CAN message
void CAN_interpreter::setResponse(CAN_message_t msg_ptr){
    _res = msg_ptr;
}

// Delay until a new message is received
void CAN_interpreter::awaitResponse(){
    while(!newMessage){
        delay(10);
        // Serial.println("we love senior design!");        
        can.events();
    }
    // Serial.println("Response received");
    newMessage = false;
}

// Get driver state from CAN message
void CAN_interpreter::getState(CAN_message_t &message) {
    
    // uint32_t word = message.buf[0] | (message.buf[1] << 8) | (message.buf[2] << 16) | (message.buf[3] << 24);
    //message data to little endian
    uint32_t word = (uint32_t) message.buf[4];
    word += (uint32_t) message.buf[5] << 8u;
    word += (uint32_t) message.buf[6] << 16u;
    word += (uint32_t) message.buf[7] << 24u;

    // Serial.print("word before: ");
    // Serial.println(word, BIN);
    // we only care about bits 0,1,2,3,5,6
    // set all other bits to 0
    //        76543210
    word &= 0b01101111;
    // Serial.print("word after: ");
    // Serial.println(word, BIN);
    switch(word) {

        // Not ready to switch on
        case 0b00000000:
            state = NotReadyToSwitchOn;
            Serial.println("NotReadyToSwitchOn");
            break;

        // Switch on disabled 
        case 0b01000000:
            state = SwitchOnDisabled;
            Serial.println("SwitchOnDisabled");
            break;

        // Ready to switch on
        case 0b00100001:
            state = ReadyToSwitchOn;
            Serial.println("ReadyToSwitchOn");
            break;

        // Switched on
        case 0b00100011:
            state = SwitchedOn;
            Serial.println("SwitchedOn");
            break;

        // Operation enabled
        case 0b00100111:
            state = OperationEnabled;
            Serial.println("In OperationEnabled state");
            break;

        // Quick stop active
        case 0b00000111:
            state = QuickStopActive;
            Serial.println("In QuickStopActive state");
            break;

        // Fault reaction active
        case 0b00001111:
            state = FaultReactionActive;
            Serial.println("In FaultReactionActive state");
            break;

        // Fault
        case 0b00001000:
            state = Fault;
            Serial.println("In Fault state");
            break;

        //set an error if the state isn"t recognized
        default:
            state = notRecognized;
            Serial.println("In notRecognized state");
            break;            
    };
}

void CAN_interpreter::getPosition() {
    // get the Position Actual Value from the controller
    strcpy(input, "6064,00,x0000r");
    err = this->createMsg(input, &msg);
    if(err > 0) {
        Serial.print("error in getPosition():");
        Serial.println(err);
        return 1;
    } else {
        can.write(msg);
        this->awaitResponse();
        // this->interpretMsg(_res);
        
        //message data to little endian
        position = (int32_t) _res.buf[4];
        position += (int32_t) _res.buf[5] << 8u;
        position += (int32_t) _res.buf[6] << 16u;
        position += (int32_t) _res.buf[7] << 24u;

        Serial.print("Current Position: ");
        Serial.println(position);
    }
}

// Set trajectory parameters
void CAN_interpreter::setTrajectoryParams(double f, double v, double a){
    freq = f;
    maxV = v * RAD_TO_TICKS;
    maxA = a * RAD_TO_TICKS;
};

// generate trajectory
// absolute = true: treat input as absolute target position
// absolute = false: treat input as change in position
uint8_t CAN_interpreter::genTrajectory(double target_rad, bool absolute = true){
    // convert target from rads to encoder ticks
    this->getPosition();

    target = (int32_t) (target_rad * RAD_TO_TICKS);

    // change target if not using absolute position
    if(!absolute){
        target += position;
    }
    
    int8_t direction = 1; // 1 is forward, -1 is backwards
    if(target < position){
        direction = -1;
    }
    
    T = maxV / maxA;
    Tc = (abs(target - position) - maxV*T) / (maxA*T);

    // CASE 1: constant velocity section
    if(Tc > 0){
        // constants for easier writing
        T_pi2 = pow(T/(2*M_PI),2);
        pi_T = 2*M_PI/T;

        // calculate array length and make sure it is not too long
        trajectoryLength = (uint16_t) ((2*T+Tc) * freq);

        if(trajectoryLength >= MAX_TRAJECTORY){
            Serial.println("Trajectory too long. (Const. V)");
            return 1;
        }

        // calculate trajectory
        for(uint16_t i = 0; i <= trajectoryLength; i++){
            t = i / freq;

            // first section - acceleration
            if(t >= 0 && t <T){
                trajectory[i] = position + (int32_t) (direction * (maxA * (pow(t,2)/2 + T_pi2*cos(pi_T*t) - T_pi2)));
            }
            // second section - constant velocity
            else if(t >= T && t <= T+Tc){
                trajectory[i] = position + (int32_t) (direction * (maxA * (pow(T,2)/2 + T*(t-T))));
            }
            // third section - deceleration
            else{
                trajectory[i] = position + (int32_t) ((direction * maxA * (pow(T,2)/2) + maxA * (T*Tc) + maxV*(t-T-Tc) - maxA * (pow(t-T-Tc,2)/2 + T_pi2*cos(pi_T*(t-T-Tc)) - T_pi2)));
                //                          s23 = am*(T^2)/2 + am*T*Tc + Vm*(t23-T-Tc) - am * (((t23-T-Tc).^2)/2 + T_pi2*cos(pi_T*(t23-T-Tc)) - T_pi2);
            }
            // Serial.println(trajectory[i]);
        }
    }
    // CASE 2: no constant velocity section
    else{
        // recalculate the period
        T = sqrt(abs(target - position) / maxA);

        // calculate array length and make sure it is not too long
        trajectoryLength = (uint16_t) ((2*T) / freq);
        if(trajectoryLength >= MAX_TRAJECTORY){
            Serial.println("Trajectory too long. (No Const. V)");
            return 2;
        }
        
        // constants for easier writing
        T_pi2 = pow(T/(2*M_PI),2);
        pi_T = 2*M_PI/T;

        // calculate trajectory`
        for(uint16_t i = 0; i <= trajectoryLength; i++){
            t = i / freq;

            // first section - acceleration (same as above)
            if(t >= 0 && t <T){
                trajectory[i] = position + (int32_t) (direction * (maxA * (pow(t,2)/2 + T_pi2*cos(pi_T*t) - T_pi2)));
            }
            // second section - deceleration
            else{
                trajectory[i] = position + (int32_t) (direction * (maxA * (pow(T,2)/2 + T*(t-T) - pow(t-T,2)/2 + T_pi2*cos(pi_T*(t-T)) - T_pi2)));
            }
        }
    }
    return 0;
}
