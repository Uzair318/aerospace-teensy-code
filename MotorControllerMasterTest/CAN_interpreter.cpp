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

CAN_interpreter::CAN_interpreter(){}

enum state_t{NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn, OperationEnabled, QuickStopActive, FaultReactionActive, Fault};

state_t state;

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

void CAN_interpreter::interpretMsg(CAN_message_t msg_ptr){
    CAN_message_t _msg;
    uint32_t _data;

    _msg = msg_ptr;

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

uint8_t CAN_interpreter::startup(FlexCAN_T4 &can1){
    CAN_message_t msg;
    Serial.println("Running setup...");

    // send statusword   
    Serial.println("Checking Statusword...");
    char input[32] = "6041,00,x0000r";  // statusword
    int err = this.createMsg(input, &msg)
    if(err > 0) {
        Serial.print("error before while loop in CAN_interpreter.startup():");
        Serial.println(err);
      } else {
        Serial.println("Sending...");
        this.interpretMsg(msg);
        can1.write(msg);
    } 

    // check the response and update the state
    this.interpretMsg(_res);
    uint32_t resData = _res.buf[0] | (_res.buf[1] << 8) | (_res.buf[2] << 16) | (_res.buf[3] << 24);
    this.setState(resData);

    // while statusword is not "operation enabled" 
    while(state != OperationEnabled) {
        // transition from the current state to the next state (and verify?)
        switch(state) {
            case NotReadyToSwitchOn:
                Serial.println("Not Ready To Switch On");
                char input[32] = "6040,00,b1111w";  // send controlword THIS ONE MIGHT BE WRONG
                int err = this.createMsg(input, &msg)
                if(err > 0) {
                    Serial.print("error in NotReadyToSwitchOn case of CAN_interpreter.startup():");
                    Serial.println(err);
                } else {
                    Serial.println("Sending controlword to execute transition 1...");
                    this.interpretMsg(msg);
                    can1.write(msg);
                } 
                break;
            case SwitchOnDisabled:
                Serial.println("Switch On Disabled");
                char input[32] = "6040,00,b0110w";  // send controlword 
                int err = this.createMsg(input, &msg)
                if(err > 0) {
                    Serial.print("error in SwitchOnDisabled case of CAN_interpreter.startup():");
                    Serial.println(err);
                } else {
                    Serial.println("Sending controlword to execute transition 2...");
                    this.interpretMsg(msg);
                    can1.write(msg);
                } 
                break;
            case ReadyToSwitchOn:
                Serial.println("Ready to Switch On");
                char input[32] = "6040,00,b0111w";  // send controlword 
                int err = this.createMsg(input, &msg)
                if(err > 0) {
                    Serial.print("error in ReadyToSwitchOn case of CAN_interpreter.startup():");
                    Serial.println(err);
                } else {
                    Serial.println("Sending controlword to execute transition 3...");
                    this.interpretMsg(msg);
                    can1.write(msg);
                }
                break;
            case SwitchedOn:
                Serial.println("Switched On");
                char input[32] = "6040,00,b1111w";  // send controlword 
                int err = this.createMsg(input, &msg)
                if(err > 0) {
                    Serial.print("error in SwitchedOn case of CAN_interpreter.startup():");
                    Serial.println(err);
                } else {
                    Serial.println("Sending controlword to execute transition 4...");
                    this.interpretMsg(msg);
                    can1.write(msg);
                }
                break;
            case QuickStopActive:
                Serial.println("Quick Stop Active");
                char input[32] = "6040,00,b1111w";  // send controlword 
                int err = this.createMsg(input, &msg)
                if(err > 0) {
                    Serial.print("error in QuickStopActive case of CAN_interpreter.startup():");
                    Serial.println(err);
                } else {
                    Serial.println("Sending controlword to execute transition 16...");
                    this.interpretMsg(msg);
                    can1.write(msg);
                }
                break;
            case FaultReactionActive:
                Serial.println("Fault Reaction Active");
                char input[32] = "6040,00,b00000000w";  // send controlword 
                int err = this.createMsg(input, &msg)
                if(err > 0) {
                    Serial.print("error in FaultReactionActive case of CAN_interpreter.startup():");
                    Serial.println(err);
                } else {
                    Serial.println("Sending controlword to execute transition 14...");
                    this.interpretMsg(msg);
                    can1.write(msg);
                }
                break;
            case Fault:
                Serial.println("Fault");
                char input[32] = "6040,b10000000w";  // send controlword 
                int err = this.createMsg(input, &msg)
                if(err > 0) {
                    Serial.print("error in Fault case of CAN_interpreter.startup():");
                    Serial.println(err);
                } else {
                    Serial.println("Sending controlword to execute transition 15...");
                    this.interpretMsg(msg);
                    can1.write(msg);
                }
                break;
            case default:
                Serial.println
            
        }

        // send statusword   
        Serial.println("Checking Statusword...");
        char input[32] = "6041,00,x0000r";  // statusword
        int err = this.createMsg(input, &msg)
        if(err > 0) {
            Serial.print("error within while loop in CAN_interpreter.startup():");
            Serial.println(err);
        } else {
            Serial.println("Sending...");
            this.interpretMsg(msg);
            can1.write(msg);
        } 

        // check the response and update the state
        this.interpretMsg(_res);
        resData = _res.buf[0] | (_res.buf[1] << 8) | (_res.buf[2] << 16) | (_res.buf[3] << 24);

        this.setState(resData);
    }

    Serial.println("Setup Complete: Controller in Operation Enabled state");
    return 0;
}
void CAN_interpreter::setResponse(CAN_message_t msg_ptr){
    _res = msg_ptr;
}

void CAN_interpreter::setState(uint32_t word) {
    // we only care about bits 0,1,2,3,5,6
    // set all other bits to 0
    //        76543210
    word &= 0b01101111;

    switch(word) {

        // Not ready to switch on
        case 0b00000000:
            state = NotReadyToSwitchOn;
            break;

        // Switch on disabled 
        case 0b01000000:
            state = SwitchOnDisabled;
            break;

        // Ready to switch on
        case 0b00100001:
            state = ReadyToSwitchOn;
            break;

        // Switched on
        case 0b00100011:
            state = SwitchedOn;
            break;;

        // Operation enabled
        case 0b00100111:
            state = OperationEnabled;
            break;

        // Quick stop active
        case 0b00000111:
            state = QuickStopActive;
            break;

        // Fault reaction active
        case 0b00001111:
            state = FaultReactionActive;
            break;

        // Fault
        case 0b00001000:
            state = Fault;
            break;

        //set an error if the state isn't recognized
        default:
            state = notRecognized;
            break;
        }
    }
}