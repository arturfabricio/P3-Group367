#ifndef Packets_h                 
#define Packets_h

#define HEADER     0xFF             //Defining Header 1 and 2
#define HEADER_3   0xFD             //Defining Header 3
#define RESERVED   0x00             //4. bytes is allways reserved
#define SERVO_SET_Baudrate  57600  //Baudrate is set to the right number
#define READ_INSTRUCTION    0x02    //Read instruct is always 2
#define WRITE_INSTRUCTION   0x03    //Write instruct is always 3
#define PACKET_LENGTH_H     0x00    //Almost allways 0
#define TORQUE_ENABLE       0x40     
#define RESERVED_BYTE       0x00
#define ALL_SERVOS          0XFE


#include <Arduino.h>
#include <SoftwareSerial.h>


class PacketsClass
{
  public:
    int Direction_Pin = 2;
    int rxPin = 11;
    int txPin = 10;
    PacketsClass();
    void setTorquePacket(unsigned char ID, bool torque_status);
    void setNTorquePacket(bool OFF_ON);
    void SetPGain(unsigned char ID, int Pgain);
    void setIGain(unsigned char ID, int Igain);
    void setDGain(unsigned char ID, int Dgain);
    void setProfileVelocity(int velocity);
    void setProfileAcceleration(int acceleration);
    void setGoalVelocityPacket(int velocity, unsigned char ID);
    void setGoalPositionPacket(int positionByte, unsigned char ID);
    void setNGoalPositionPacket(int p1, int p2, int p3, int p4, int p5);
    void rebootDynamixelPacket(unsigned char  ID);
    void ReadTemp(unsigned char ID);
    void transmitInstructionPacket(unsigned char* var1, int var2);
    void readStatusPacket();
    void ReadPosition();
    unsigned short update_crc(unsigned short crc_accum, unsigned char* data_blk_ptr, unsigned short data_blk_size);

  SoftwareSerial mySerial;
 

  private:
  unsigned char  Status_Packet_Array[20];
  
};


#endif
