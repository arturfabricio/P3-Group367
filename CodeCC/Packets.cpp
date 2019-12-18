
#include <Wire.h>
#include <Arduino.h>
#include "packets.h"
#include <SoftwareSerial.h>
  

PacketsClass::PacketsClass() : mySerial(SoftwareSerial(rxPin, txPin))
{
  
  
  pinMode(rxPin, OUTPUT);
  pinMode(txPin, OUTPUT);
  pinMode(Direction_Pin, OUTPUT);
  mySerial.begin(SERVO_SET_Baudrate);
  }


void PacketsClass::setTorquePacket(unsigned char ID, bool torque_status) {   //Set torque
  unsigned char torqueInstruction[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ID,
    0x06,
    PACKET_LENGTH_H,
    WRITE_INSTRUCTION,
    TORQUE_ENABLE,            //Is put to 0x40
    0x00,
    torque_status        //False eller True - Off or On
  };

  transmitInstructionPacket(torqueInstruction, 11);
}

void PacketsClass::setNTorquePacket(bool OFF_ON) {    //Set torque
  unsigned char torqueSyncInstruction[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ALL_SERVOS,       //0xFE defined in the top
    0x11,             //Low byte lenght without Andress bytes
    PACKET_LENGTH_H,
    0x83,             //Intruction
    0x40,            //Low order byte from the starting address
    0x00,             //High order byte from the starting address
    0x01,             //Low-order byte from the data length(X)
    0x00,             //High-order byte from the data length(X)
    0x01,             //First device ID
    OFF_ON,           //First byte
    0x02,             //Second ID and so on..
    OFF_ON,
    0x03,
    OFF_ON,
    0x04,
    OFF_ON,
    0x05,
    OFF_ON
  };


  transmitInstructionPacket(torqueSyncInstruction, 22);
}


void PacketsClass::SetPGain(unsigned char ID, int Pgain) {

  unsigned char PGain[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ID,
    0x07,                          //LByte of length
    PACKET_LENGTH_H,               //HByte of length
    WRITE_INSTRUCTION,
    0x54,               //LByte of address
    0x00,               //HByte of address
    (Pgain & 0xFF),        //byte 1 of Data
    (Pgain & 0xFF00) >> 8,   //byte 2 of Data
  };

  transmitInstructionPacket(PGain, 12);
}


void PacketsClass::setIGain(unsigned char ID, int Igain) {

  unsigned char IGain[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ID,
    0x07,                          //LByte of length
    PACKET_LENGTH_H,               //HByte of length
    WRITE_INSTRUCTION,
    0x52,               //LByte of address
    0x00,               //HByte of address
    (Igain & 0xFF),        //byte 1 of Data
    (Igain & 0xFF00) >> 8,   //byte 2 of Data
  };

  transmitInstructionPacket(IGain, 12);
}

void PacketsClass::setDGain(unsigned char ID, int Dgain) {

  unsigned char DGain[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ID,
    0x07,                          //LByte of length
    PACKET_LENGTH_H,               //HByte of length
    WRITE_INSTRUCTION,
    0x50,
    0x00,
    (Dgain & 0xFF),
    (Dgain & 0xFF00) >> 8
  };

  transmitInstructionPacket(DGain, 12);
}

void PacketsClass::setProfileVelocity(int velocity) {

  unsigned char velocityPacket[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ALL_SERVOS,
    0x16,
    PACKET_LENGTH_H,
    0x83,             //Instruction
    0x70,             //Low byte of adress
    0x00,             //High byte of adress
    0x04,
    0x00,
    0x01,
    (velocity & 0xFF),        //byte 1 of Data
    (velocity & 0xFF00) >> 8,   //byte 2 of Data
    (velocity & 0xFF0000) >> 16,  //byte 3 of Data
    (velocity & 0xFF000000) >> 24,  //byte 4 of Data
    0x02,
    (velocity & 0xFF),        //byte 1 of Data
    (velocity & 0xFF00) >> 8,   //byte 2 of Data
    (velocity & 0xFF0000) >> 16,  //byte 3 of Data
    (velocity & 0xFF000000) >> 24,  //byte 4 of Data
    0x03,
    (velocity & 0xFF),        //byte 1 of Data
    (velocity & 0xFF00) >> 8,   //byte 2 of Data
    (velocity & 0xFF0000) >> 16,  //byte 3 of Data
    (velocity & 0xFF000000) >> 24,  //byte 4 of Data
  };
  transmitInstructionPacket(velocityPacket, 27);
}

void PacketsClass::setProfileAcceleration(int acceleration) {

  unsigned char accelerationPacket[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ALL_SERVOS,
    0x16,
    PACKET_LENGTH_H,
    0x83,             //Instruction
    0x6C,             //Low byte of adress
    0x00,             //High byte of adress
    0x04,
    0x00,
    0x01,
    (acceleration & 0xFF),        //byte 1 of Data
    (acceleration & 0xFF00) >> 8,   //byte 2 of Data
    (acceleration & 0xFF0000) >> 16,  //byte 3 of Data
    (acceleration & 0xFF000000) >> 24,  //byte 4 of Data
    0x02,
    (acceleration & 0xFF),        //byte 1 of Data
    (acceleration & 0xFF00) >> 8,   //byte 2 of Data
    (acceleration & 0xFF0000) >> 16,  //byte 3 of Data
    (acceleration & 0xFF000000) >> 24,  //byte 4 of Data
    0x03,
    (acceleration & 0xFF),        //byte 1 of Data
    (acceleration & 0xFF00) >> 8,   //byte 2 of Data
    (acceleration & 0xFF0000) >> 16,  //byte 3 of Data
    (acceleration & 0xFF000000) >> 24,  //byte 4 of Data
  };
  transmitInstructionPacket(accelerationPacket, 27);
}


void PacketsClass::setGoalVelocityPacket(int velocity, unsigned char ID) {

  unsigned char Velocity[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ID,
    0x09,
    PACKET_LENGTH_H,
    WRITE_INSTRUCTION,
    0x68,                     //Low-order byte from the starting address
    0x00,                     //High-order byte from the starting address
    (velocity & 0xFF),        //byte 1 of Data
    (velocity & 0xFF00) >> 8,   //byte 2 of Data
    (velocity & 0xFF0000) >> 16,  //byte 3 of Data
    (velocity & 0xFF000000) >> 24,  //byte 4 of Data
  };

  transmitInstructionPacket(Velocity, 14);
}



void PacketsClass::setGoalPositionPacket(int positionByte, unsigned char ID) {

  unsigned char instruction[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ID,
    0x09,                          //LByte of length
    PACKET_LENGTH_H,               //HByte of length
    WRITE_INSTRUCTION,          //Instruction byte
    0x74,               //LByte of address
    0x00,               //HByte of address
    (positionByte & 0xFF),        //byte 1 of Data
    (positionByte & 0xFF00) >> 8,   //byte 2 of Data
    (positionByte & 0xFF0000) >> 16,  //byte 3 of Data
    (positionByte & 0xFF000000) >> 24,  //byte 4 of Data
  };

  transmitInstructionPacket(instruction, 14);
}


void PacketsClass::setNGoalPositionPacket(int p1, int p2, int p3, int p4, int p5) {
  unsigned char positionInstruction[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED,
    ALL_SERVOS,       //Writes to all servos
    0x20,
    0x00,
    0x83,
    0x74,
    0x00,
    0x04,
    0x00,
    0x01,
    (p1 & 0xFF),
    (p1 & 0xFF00) >> 8,
    (p1 & 0xFF0000) >> 16,
    (p1 & 0xFF000000) >> 24,
    0x02,
    (p2 & 0xFF),
    (p2 & 0xFF00) >> 8,
    (p2 & 0xFF0000) >> 16,
    (p2 & 0xFF000000) >> 24,
    0x03,
    (p3 & 0xFF),
    (p3 & 0xFF00) >> 8,
    (p3 & 0xFF0000) >> 16,
    (p3 & 0xFF000000) >> 24,
    0x04,
    (p4 & 0xFF),
    (p4 & 0xFF00) >> 8,
    (p4 & 0xFF0000) >> 16,
    (p4 & 0xFF000000) >> 24,
    0x05,
    (p5 & 0xFF),
    (p5 & 0xFF00) >> 8,
    (p5 & 0xFF0000) >> 16,
    (p5 & 0xFF000000) >> 24,
  };
  transmitInstructionPacket(positionInstruction, 37);
}


void PacketsClass::rebootDynamixelPacket(unsigned char  ID) {
  unsigned char rebootInstructionPacket[] {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED_BYTE,
    ID,
    0x03,                     //Lenght
    PACKET_LENGTH_H,
    0x08                    //Intruction
  };

  transmitInstructionPacket(rebootInstructionPacket, 8);
}

void PacketsClass::ReadTemp(unsigned char ID) {
  unsigned char Temperature[] {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED_BYTE,
    ID,
    0x07,
    PACKET_LENGTH_H,
    0x02,               //Instruction
    0x92,               //Low-order byte from the starting address
    0x00,               //High-order byte from the starting address
    0x01,
    0x00
  };

  transmitInstructionPacket(Temperature, 12);
}


void PacketsClass::ReadPosition(){

  unsigned char ReadPos[] = {
    HEADER,
    HEADER,
    HEADER_3,
    RESERVED_BYTE,
    ALL_SERVOS,
    0x0A,
    0x00,
    0x82,
    0x74,
    0x00,
    0x04,
    0x00,
    0x01,
    0x02,
    0x03,
  };
    
   transmitInstructionPacket(ReadPos, 15);
  }

void PacketsClass::transmitInstructionPacket(unsigned char* var1, int var2) {     //Every packet will be send to this function with two parameters being the instruction packet and the lenght of the packet


  digitalWrite(Direction_Pin, HIGH); // Set TX Buffer pin to HIGH to be able to transmit
  unsigned short crc = update_crc(0, var1, var2);             // crc = update.crc and feeds it the three variables it need 0, intruction and lenght of packet
  // Serial.print("Transmitting   ");
  for (int i = 0; i < var2; i++)
  {
    mySerial.write(*var1);
    var1++;

  }

  unsigned char CRC_L = (crc & 0x00FF);
  unsigned char CRC_H = (crc >> 8) & 0x00FF;


  noInterrupts();


  mySerial.write(CRC_L);
  mySerial.write(CRC_H);

  if ((UCSR1A & B01100000) != B01100000) {
    mySerial.flush(); //wait for TX data to be sent
  }

  digitalWrite(Direction_Pin, LOW); //Set TX Buffer pin to LOW after data has been sent

  interrupts();

  readStatusPacket();
}


void PacketsClass::readStatusPacket() {
  int i = 0;
  while (mySerial.available() > 0) {
    Status_Packet_Array[i] = mySerial.read();
    Serial.print(Status_Packet_Array[i], HEX);
    Serial.print("  ");
    i++;
  }
  Serial.println();
}


unsigned short PacketsClass::update_crc(unsigned short crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size) //crc calculater - this makes sure all bytes are recieved as supposed
{
  unsigned short i, j;
  unsigned short crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
  };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}
