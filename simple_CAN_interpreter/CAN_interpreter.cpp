/*
    CAN interpreter library to take user input and create Flexcan4 messages for Maxon EPOS4 controller
    Alex Broz
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

uint8_t CAN_interpreter::createMsg(char _input[],CAN_message_t *msg_ptr){
    //_input = input_ptr;
    
    _msg.id = 0x0601;  // 0x600 + node_id (SDO client to server)
    
    //check for valid comma placement
    if(_input[4] != "," || _input[7] != ","){
        return 1;
    }

    //index low byte - chars 3 and 4
    _temp[0] = _input[2];
    _temp[1] = _input[3];
    _temp[3] = "\0";
    _msg.buf[1] = (uint8_t) strtol(_temp, nullptr, 16);
    
    //index high byte - chars 1 and 2
    _temp[0] = _input[0];
    _temp[1] = _input[1];
    _temp[3] = "\0";
    _msg.buf[2] = (uint8_t) strtol(_temp, nullptr, 16);

    //subindex - chars 5 and 6
    _temp[0] = _input[5];
    _temp[1] = _input[6];
    _temp[3] = "\0";
    _msg.buf[3] = (uint8_t) strtol(_temp, nullptr, 16);

    //determine data length and read/write
    _length = 0;
    
    //determine input length and create substring for conversion
    bool end = false;
    while(_input[_length + 9] != "\0" && !end){
        switch(_input[_length + 9]){
            //write
            case 'w':
                end = true;
                _temp[_length] = "\0"; //terminate string
                _msg.buf[0] = 0x22;
                break;
            case 'W':
                end = true;
                _temp[_length] = "\0"; //terminate string
                _msg.buf[0] = 0x22;
                break;

            //read
            case 'r':
                end = true;
                _temp[_length] = "\0"; //terminate string
                _msg.buf[0] = 0x40;
                break;
            case 'R':
                end = true;
                _temp[_length] = "\0"; //terminate string
                _msg.buf[0] = 0x40;
                break;

            //still data
            default:
                _temp[_length] = _input[_length + 9]; //add to substring
                _length ++;
        }
    }

    //determine data type
    switch(_input[6]){
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
            return 1;
    }

    //convert data to big endian and add to message
    _msg.buf[4] = (_data & 0x000000ff) << 24u;
    _msg.buf[5] = (_data & 0x0000ff00) << 8u;
    _msg.buf[6] = (_data & 0x00ff0000) >> 8u;
    _msg.buf[7] = (_data & 0xff000000) >> 24u;

    //finished
    *msg_ptr = _msg;
    return 0;
}

void CAN_interpreter::interpretMsg(CAN_message_t *msg_ptr){
    _msg = *msg_ptr;

    //message data to little endian
    _data = (uint32_t) _msg.buf[4];
    _data += (uint32_t) _msg.buf[5] << 8u;
    _data += (uint32_t) _msg.buf[6] << 16u;
    _data += (uint32_t) _msg.buf[7] << 24u;

    //print message info
    Serial.print("ID: ");
    Serial.print(_msg.id,HEX);
    Serial.print("\tW/R: ");
    Serial.print(_msg.buf[0]);
    Serial.print("\tIndex: ");
    Serial.print(_msg.buf[2], HEX);
    Serial.print(_msg.buf[1], HEX);
    Serial.print("\tSubindex: ");
    Serial.println(_msg.buf[3], HEX);

    //print all data as hex
    Serial.print(_msg.buf[7],HEX);
    Serial.print(_msg.buf[6],HEX);
    Serial.print(_msg.buf[5],HEX);
    Serial.println(_msg.buf[4],HEX);

    //print low 2 bytes as binary
    Serial.print(_msg.buf[7],BIN);
    Serial.println(_msg.buf[6],HEX);

    //print all data as int
    Serial.println(_data);
    Serial.println();
}
