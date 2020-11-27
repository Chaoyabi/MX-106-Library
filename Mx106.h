/*
Using Dynamixel Protocol 1.0 to control Dynamixel 2.0 Firmware
UART feature is programmed with HAL.
 */

#ifndef Mx106_h
#define Mx106_h

 //-------------------------------------------------------------------------------------------------------------------------------
 // define - Dynamixel Hex code table 
 //-------------------------------------------------------------------------------------------------------------------------------
 // EEPROM AREA
#define EEPROM_MODEL_NUMBER_L           0x00
#define EEPROM_MODEL_NUMBER_H           0x01
#define EEPROM_MODEL_INFO_1             0x02
#define EEPROM_MODEL_INFO_2             0x03
#define EEPROM_MODEL_INFO_3             0x04
#define EEPROM_MODEL_INFO_4             0x05
#define EEPROM_VERSION                  0x06
#define EEPROM_ID                       0x07
#define EEPROM_BAUDRATE                 0x08

#define EEPROM_OPERATION_MODE           0x0B

#define EEPROM_MAX_VOLTAGE_LIMIT_1      0x20
#define EEPROM_MAX_VOLTAGE_LIMIT_2      0x21
#define EEPROM_MIN_VOLTAGE_LIMIT_1      0x22
#define EEPROM_MIN_VOLTAGE_LIMIT_2      0x23

#define EEPROM_CURRENT_LIMIT_1          0x26
#define EEPROM_CURRENT_LIMIT_2          0x27

#define EEPROM_MAX_VELOCITY_LIMIT_1     0x2C
#define EEPROM_MAX_VELOCITY_LIMIT_2     0x2D
#define EEPROM_MAX_VELOCITY_LIMIT_3     0x2E
#define EEPROM_MAX_VELOCITY_LIMIT_4     0x2F

#define EEPROM_MAX_POSITION_LIMIT_1     0x30
#define EEPROM_MAX_POSITION_LIMIT_2     0x31
#define EEPROM_MAX_POSITION_LIMIT_3     0x32
#define EEPROM_MAX_POSITION_LIMIT_4     0x33
#define EEPROM_MIN_POSITION_LIMIT_1     0x34
#define EEPROM_MIN_POSITION_LIMIT_2     0x35
#define EEPROM_MIN_POSITION_LIMIT_3     0x36
#define EEPROM_MIN_POSITION_LIMIT_4     0x37
#define EEPROM_SHUTDOWN                 0x3F

// RAM AREA  
#define RAM_TORQUE_ENABLE               0x40
#define RAM_LED                         0x41

#define RAM_STATUS_RETURN_LEVEL         0x44

#define RAM_VELOCITY_I_GAIN_L           0x4C
#define RAM_VELOCITY_I_GAIN_H           0x4D
#define RAM_VELOCITY_P_GAIN_L           0x4E
#define RAM_VELOCITY_P_GAIN_H           0x4F

#define RAM_POSITION_D_GAIN_L           0x50
#define RAM_POSITION_D_GAIN_H           0x51
#define RAM_POSITION_I_GAIN_L           0x52
#define RAM_POSITION_I_GAIN_H           0x53
#define RAM_POSITION_P_GAIN_L           0x54
#define RAM_POSITION_P_GAIN_H           0x55

#define RAM_GOAL_PWM_1                  0x64
#define RAM_GOAL_PWM_2                  0x65

#define RAM_GOAL_CURRENT_1              0x66
#define RAM_GOAL_CURRENT_2              0x67

#define RAM_GOAL_VELOCITY_1             0x68
#define RAM_GOAL_VELOCITY_2             0x69
#define RAM_GOAL_VELOCITY_3             0x6A
#define RAM_GOAL_VELOCITY_4             0x6B

#define RAM_MOVING_ACCELERATION_1       0x6C
#define RAM_MOVING_ACCELERATION_2       0x6D
#define RAM_MOVING_ACCELERATION_3       0x6E
#define RAM_MOVING_ACCELERATION_4       0x6F

#define RAM_MOVING_VELOCITY_1           0x70
#define RAM_MOVING_VELOCITY_2           0x71
#define RAM_MOVING_VELOCITY_3           0x72
#define RAM_MOVING_VELOCITY_4           0x73

#define RAM_GOAL_POSITION_1             0x74
#define RAM_GOAL_POSITION_2             0x75
#define RAM_GOAL_POSITION_3             0x76
#define RAM_GOAL_POSITION_4             0x77

#define RAM_REALTIME_TICK_L             0x78
#define RAM_REALTIME_TICK_H             0x79

#define RAM_PRESENT_PWM_1               0x7C
#define RAM_PRESENT_PWM_2               0x7D

#define RAM_PRESENT_CURRENT_1           0x7E
#define RAM_PRESENT_CURRENT_2           0x7F

#define RAM_PRESENT_VELOCITY_1          0x80
#define RAM_PRESENT_VELOCITY_2          0x81
#define RAM_PRESENT_VELOCITY_3          0x82
#define RAM_PRESENT_VELOCITY_4          0x83

#define RAM_PRESENT_POSITION_1          0x84
#define RAM_PRESENT_POSITION_2          0x85
#define RAM_PRESENT_POSITION_3          0x86
#define RAM_PRESENT_POSITION_4          0x87

#define RAM_PRESENT_VOLTAGE             0x90

#define RAM_PRESENT_TEMPERATURE         0x92

//-------------------------------------------------------------------------------------------------------------------------------
// Instruction commands Set 
//-------------------------------------------------------------------------------------------------------------------------------
#define COMMAND_PING                    0x01
#define COMMAND_READ_DATA               0x02
#define COMMAND_WRITE_DATA              0x03
#define COMMAND_REG_WRITE_DATA          0x04
#define COMMAND_ACTION                  0x05
#define COMMAND_RESET                   0x06
#define COMMAND_REBOOT                  0x08
#define COMMAND_SYNC_WRITE              0x83
#define COMMAND_BULK_READ               0x92


//-------------------------------------------------------------------------------------------------------------------------------
//Instruction packet lengths 
#define READ_ONE_BYTE_LENGTH            0x01
#define READ_TWO_BYTE_LENGTH            0x02
#define READ_FOUR_BYTE_LENGTH           0x04
#define RESET_LENGTH                    0x02
#define LED_LENGTH                      0x04

#define SET_ID_LENGTH                   0x04
#define SET_MAX_VELOCITY_LENGTH         0x07

#define READ_POS_LENGTH                 0x04
#define READ_SPEED_LENGTH               0x04

#define WHEEL_LENGTH                    0x07
#define SERVO_GOAL_LENGTH               0x0B
#define SET_MODE_LENGTH                 0x04
#define SET_OPERATION_MODE_LENGTH       0x04
#define SET_POSITION_PID_LENGTH         0x09
#define SET_VELOCITY_PI_LENGTH          0x07
#define TORQUE_ENABLE_LENGTH            0x04
//-------------------------------------------------------------------------------------------------------------------------------
// Specials 
//-------------------------------------------------------------------------------------------------------------------------------

#define OFF                             0x00
#define ON                              0x01

#define VELOCITY                        0x01
#define POSITION                        0x03

#define LEFT                            0x00
#define RIGHT                           0x01

#define PING                            0x00
#define READ                            0x01
#define ALL                             0x02

#define BROADCAST_ID                    0xFE

#define HEADER                          0xFF

#define TIMEOUT                         1      // in millis()


class DynamixelClass {

private:
    DigitalOut* servoSerialDir;

    void debugInstructionframe(void);
    void debugStatusframe(void);
    void transmitInstructionPacket(void);
    void readStatusPacket(void);

public:
    DynamixelClass(int baud, PinName D_Pin);    //Constructor
    ~DynamixelClass(void);                 //destruktor

    //-------------------------------------------------------------------------------------------------------------------------------
    // EEPROM AREA  
    uint8_t OperationMode(uint8_t ID, uint8_t OPEARTION_MODE);
    //    unsigned int setTemp(unsigned char,unsigned char);
    uint8_t MaxMinVoltageLimit(uint8_t ID, uint16_t Volt_L, uint16_t Volt_H);
    uint8_t VelocityLimit(uint8_t ID, uint32_t Velocity_Limit);
    //    unsigned int Shutdown(unsigned char,unsigned char);

    //-------------------------------------------------------------------------------------------------------------------------------
    // RAM AREA  

    uint8_t TorqueEnable(uint8_t ID, bool Status);
    uint8_t LED(uint8_t ID, bool Status);
    uint8_t StatusReturnLevel(uint8_t ID, uint8_t level);
    uint8_t Velocity_PI(uint8_t ID, uint16_t P, uint16_t I);
    uint8_t Position_PID(uint8_t, uint16_t P, uint16_t I, uint16_t D);
    uint8_t PWM(uint8_t ID, int16_t PWM);
    uint8_t Current(uint8_t ID, int16_t Current);
    uint8_t Velocity(uint8_t, int32_t);
    uint8_t Position(uint8_t, int32_t, int32_t);
    //    unsigned int checkMovement(uint8_t);
    int16_t ReadPWM(uint8_t);
    int16_t ReadCurrent(uint8_t);
    int32_t ReadVelocity(uint8_t);
    int32_t ReadPosition(uint8_t);
    int16_t ReadVoltage(uint8_t);
    //    unsigned int readTemperature(uint8_t);
    int8_t SetIndirectAddress(uint8_t ID, uint8_t Indirect, uint8_t Addr);

    //-------------------------------------------------------------------------------------------------------------------------------
    // Special Command

    //    unsigned int ping(uint8_t);
    //    unsigned int action(uint8_t);
    uint8_t reset(uint8_t);

    void SyncWrite_StatusReturnLevel(uint8_t level);
    void SyncWrite_SetIndirectAddress();
    void SyncWrite_n_dynamixels(uint8_t n, uint8_t *ID_list, int32_t *cmd);
    int32_t BulkRead_n_dynamixels(uint8_t n, uint8_t* ID_list, int32_t* position, int32_t* velocity);
        
    int32_t ReadRegister(uint8_t ID, uint16_t Register);
    int32_t testfunction(uint8_t ID);
};

#endif
