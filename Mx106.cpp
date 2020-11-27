#include "mbed.h"
#include "Mx106.h"

GPIO_InitTypeDef   huart4_gpio;
UART_HandleTypeDef huart4;

uint8_t   Instruction_Packet_Array[120];   // Array to hold instruction packet data 
uint8_t   Status_Packet_Array[15];        // Array to hold returned status packet data

// Because printing the packet directly maybe clean the data in UART,
// We need to copy packet into debug_packet
// and print the debug_packet after reading all data in packet we need.
uint8_t   debug_Instruction_Packet_Array[35];   // Array to debug instruction packet data
uint8_t   debug_Status_Packet_Array[15];        // Array to debug status packet data

uint8_t   Status_Return_Level = ALL;     // Status packet return states ( PING , READ , ALL )

uint16_t packet_length = 0;
uint8_t packet_header[2] = { HEADER, HEADER };

void uart_gpio_init() {
    __GPIOA_CLK_ENABLE();
    //PA0 -> TX ,PA1->RX
    huart4_gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    huart4_gpio.Mode = GPIO_MODE_AF_PP;
    huart4_gpio.Pull = GPIO_PULLUP;
    huart4_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    huart4_gpio.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &huart4_gpio);
}

void uart_init(int baud) {
    __UART4_CLK_ENABLE();
    huart4.Init.BaudRate = baud;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Instance = UART4;
    HAL_UART_Init(&huart4);

    /*
    // There are some problem using UART_IRQHandler while including mbed library.
    // By devoloping in other IDE(not Mbed online compiler), it might considered
    // using IRQ to improve the performance.
    NVIC_SetVector(UART4_IRQn,(uint32_t)UART_IRQHandler);
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    */
}

//-------------------------------------------------------------------------------------------------------------------------------
// Private Methods 
//-------------------------------------------------------------------------------------------------------------------------------
void DynamixelClass::debugInstructionframe(void) {
    for (int i = 0; i < 10; i++)
        debug_Instruction_Packet_Array[i] = Instruction_Packet_Array[i];
    //  for (int i = 0; i < 10; i++) printf("%x\t,",debug_Instruction_Packet_Array[i]);
    //  printf("\r\nyou transmit!\r\n"); 
}

void DynamixelClass::debugStatusframe(void) {
    for (int i = 0; i < 10; i++)
        debug_Status_Packet_Array[i] = Status_Packet_Array[i];
    //      for (int i = 0; i < 10; i++) printf("%x\t",debug_Status_Packet_Array[i]);
    //      printf("\r\nyou recieved!\r\n");
    //printf("\r");
}


void DynamixelClass::transmitInstructionPacket(void) {
// Transmit instruction packet to Dynamixel
    servoSerialDir->write(1);

    HAL_UART_Transmit(&huart4, Instruction_Packet_Array, Instruction_Packet_Array[3] + 4, TIMEOUT);

    while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == 0) {}
    __HAL_UART_FLUSH_DRREGISTER(&huart4);
    servoSerialDir->write(0);

    //  debugframe();
}


void DynamixelClass::readStatusPacket(void) {
    static uint8_t InBuff[5];

    HAL_UART_Receive(&huart4, InBuff, 2, TIMEOUT);
    if (InBuff[0] == HEADER && InBuff[1] == HEADER) {
        HAL_UART_Receive(&huart4, Status_Packet_Array, packet_length, TIMEOUT);
    }

    //    debugStatusframe();
}


//-------------------------------------------------------------------------------------------------------------------------------
// Public Methods 
//-------------------------------------------------------------------------------------------------------------------------------
DynamixelClass::DynamixelClass(int baud, PinName D_Pin) {
    servoSerialDir = new DigitalOut(D_Pin);
    servoSerialDir->write(0);

    HAL_Init();
    //uart gpio config
    uart_gpio_init();
    //uart config
    uart_init(baud);

    Instruction_Packet_Array[0] = HEADER;
    Instruction_Packet_Array[1] = HEADER;
}


DynamixelClass::~DynamixelClass() {
    if (servoSerialDir != NULL) delete servoSerialDir;
}


//-------------------------------------------------------------------------------------------------------------------------------
// EEPROM AREA  

uint8_t DynamixelClass::OperationMode(uint8_t ID, uint8_t OPEARTION_MODE) {
// Set Operation Mode: Current Mode 0x00, Velocity Mode 0x01, Position Mode 0x03
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = SET_OPERATION_MODE_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = EEPROM_OPERATION_MODE;
    Instruction_Packet_Array[6] = OPEARTION_MODE;
    Instruction_Packet_Array[7] = ~(ID + SET_OPERATION_MODE_LENGTH + COMMAND_WRITE_DATA + EEPROM_OPERATION_MODE + OPEARTION_MODE);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


//unsigned int DynamixelClass::setTemp(uint8_t ID,uint8_t temp){
//    
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = SET_TEMP_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//    Instruction_Packet_Array[3] = EEPROM_LIMIT_TEMPERATURE;
//    Instruction_Packet_Array[4] = temp;
//    Instruction_Packet_Array[5] = ~(ID + SET_TEMP_LENGTH + COMMAND_WRITE_DATA + EEPROM_LIMIT_TEMPERATURE + temp);   
//    
//    if (servoSerial->readable()) 
//      while (servoSerial->readable()) servoSerial->getc(); //empty buffer
//
//    transmitInstructionPacket();    
//    
//    if (ID == 0XFE || Status_Return_Level != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
//        return (0x00);
//    }else{
//        readStatusPacket();
//        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
//            return (Status_Packet_Array[0]);            // Return SERVO ID
//        }else{
//            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
//        }
//    }   
//}


uint8_t DynamixelClass::MaxMinVoltageLimit(uint8_t ID, uint16_t Volt_H, uint16_t Volt_L) {

    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x07;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = EEPROM_MAX_VOLTAGE_LIMIT_1;
    Instruction_Packet_Array[6] = (uint8_t)(Volt_H & 0xFF);
    Instruction_Packet_Array[7] = (uint8_t)((Volt_H >> 8) & 0xFF);
    Instruction_Packet_Array[8] = (uint8_t)(Volt_L & 0xFF);
    Instruction_Packet_Array[9] = (uint8_t)((Volt_L >> 8) & 0xFF);
    Instruction_Packet_Array[10] = ~(ID + 0x07 + COMMAND_WRITE_DATA + EEPROM_MAX_VOLTAGE_LIMIT_1 + Instruction_Packet_Array[6] + Instruction_Packet_Array[7] + Instruction_Packet_Array[8] + Instruction_Packet_Array[9]);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


uint8_t DynamixelClass::VelocityLimit(uint8_t ID, uint32_t Velocity_Limit) {
    /*
    Unit        = 0.229     rpm
    Value range = 0 ~ 1023
    (+)CCW (-)CW
    */

    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x07;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = EEPROM_MAX_VELOCITY_LIMIT_1;
    Instruction_Packet_Array[6] = (uint8_t)(Velocity_Limit & 0xFF);
    Instruction_Packet_Array[7] = (uint8_t)((Velocity_Limit >> 8) & 0xFF);
    Instruction_Packet_Array[8] = (uint8_t)((Velocity_Limit >> 16) & 0xFF);
    Instruction_Packet_Array[9] = (uint8_t)((Velocity_Limit >> 24) & 0xFF);
    Instruction_Packet_Array[10] = ~(ID + 0x07 + COMMAND_WRITE_DATA + EEPROM_MAX_VELOCITY_LIMIT_1 + Instruction_Packet_Array[6] + Instruction_Packet_Array[7] + Instruction_Packet_Array[8] + Instruction_Packet_Array[9]);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();
    if (Status_Return_Level == ALL) {
        readStatusPacket();
        if (Status_Packet_Array[2] != 0) {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
    return 0x00; //if no errors
}


//unsigned int DynamixelClass::Shutdown(uint8_t  ID,uint8_t Set){
//
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = SET_ALARM_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//    Instruction_Packet_Array[3] = EEPROM_ALARM_SHUTDOWN;
//    Instruction_Packet_Array[4] = Set;
//    Instruction_Packet_Array[5] = ~(ID + SET_ALARM_LENGTH + COMMAND_WRITE_DATA + EEPROM_ALARM_SHUTDOWN + Set);  
//    
//    if (servoSerial->readable()) 
//      while (servoSerial->readable()) servoSerial->getc(); //empty buffer
//
//    transmitInstructionPacket();    
//    
//    if (ID == 0XFE || Status_Return_Level != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
//        return (0x00);
//    }else{
//        readStatusPacket();
//        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
//            return (Status_Packet_Array[0]);            // Return SERVO ID
//        }else{
//            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
//        }
//    }           
//
//}


//-------------------------------------------------------------------------------------------------------------------------------
// RAM AREA  

uint8_t DynamixelClass::TorqueEnable(uint8_t ID, bool Status) {
    /*
    Must Enable it before any motion(Velocity or Position)
    When it is enabled, EEROM will be locked.
    */
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = TORQUE_ENABLE_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_TORQUE_ENABLE;
    Instruction_Packet_Array[6] = Status;
    Instruction_Packet_Array[7] = ~(ID + TORQUE_ENABLE_LENGTH + COMMAND_WRITE_DATA + RAM_TORQUE_ENABLE + Status);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


uint8_t DynamixelClass::LED(uint8_t ID, bool Status) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = LED_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_LED;
    Instruction_Packet_Array[6] = Status;
    Instruction_Packet_Array[7] = ~(ID + LED_LENGTH + COMMAND_WRITE_DATA + RAM_LED + Status);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


uint8_t DynamixelClass::StatusReturnLevel(uint8_t ID, uint8_t level) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x04;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_STATUS_RETURN_LEVEL;
    Instruction_Packet_Array[6] = level;
    Instruction_Packet_Array[7] = ~(ID + LED_LENGTH + COMMAND_WRITE_DATA + RAM_STATUS_RETURN_LEVEL + level);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    Status_Return_Level = level;

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


uint8_t DynamixelClass::Velocity_PI(uint8_t ID, uint16_t P, uint16_t I) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = SET_VELOCITY_PI_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_VELOCITY_I_GAIN_L;
    Instruction_Packet_Array[6] = (uint8_t)(I);
    Instruction_Packet_Array[7] = (uint8_t)((I & 0xFF00) >> 8);
    Instruction_Packet_Array[8] = (uint8_t)(P);
    Instruction_Packet_Array[9] = (uint8_t)((P & 0xFF00) >> 8);
    Instruction_Packet_Array[10] = ~(ID + SET_VELOCITY_PI_LENGTH + COMMAND_WRITE_DATA + RAM_VELOCITY_I_GAIN_L + (uint8_t)(P)+(uint8_t)((P & 0xFF00) >> 8) + (uint8_t)(I)+(uint8_t)((I & 0xFF00) >> 8));

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


uint8_t DynamixelClass::Position_PID(uint8_t ID, uint16_t P, uint16_t I, uint16_t D) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = SET_POSITION_PID_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_POSITION_D_GAIN_L;
    Instruction_Packet_Array[6] = (uint8_t)(D);
    Instruction_Packet_Array[7] = (uint8_t)((D & 0xFF00) >> 8);
    Instruction_Packet_Array[8] = (uint8_t)(I);
    Instruction_Packet_Array[9] = (uint8_t)((I & 0xFF00) >> 8);
    Instruction_Packet_Array[10] = (uint8_t)(P);
    Instruction_Packet_Array[11] = (uint8_t)((P & 0xFF00) >> 8);
    Instruction_Packet_Array[12] = ~(ID + SET_POSITION_PID_LENGTH + COMMAND_WRITE_DATA + RAM_POSITION_D_GAIN_L + Instruction_Packet_Array[6] + Instruction_Packet_Array[7] + Instruction_Packet_Array[8] + Instruction_Packet_Array[9] + Instruction_Packet_Array[10] + Instruction_Packet_Array[11]);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


uint8_t DynamixelClass::PWM(uint8_t ID, int16_t PWM) {
    
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x05;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_GOAL_PWM_1;
    Instruction_Packet_Array[6] = (uint8_t)(PWM & 0xFF);
    Instruction_Packet_Array[7] = (uint8_t)((PWM >> 8) & 0xFF);
    Instruction_Packet_Array[8] = ~(ID + 0x05 + COMMAND_WRITE_DATA + RAM_GOAL_PWM_1 + Instruction_Packet_Array[6] + Instruction_Packet_Array[7]);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2]);   // If there is a error Returns error value
        }
    }
}


uint8_t DynamixelClass::Current(uint8_t ID, int16_t Current) {
    
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x05;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_GOAL_CURRENT_1;
    Instruction_Packet_Array[6] = (uint8_t)(Current & 0xFF);
    Instruction_Packet_Array[7] = (uint8_t)((Current >> 8) & 0xFF);
    Instruction_Packet_Array[8] = ~(ID + 0x05 + COMMAND_WRITE_DATA + RAM_GOAL_CURRENT_1 + Instruction_Packet_Array[6] + Instruction_Packet_Array[7]);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2]);   // If there is a error Returns error value
        }
    }
}


uint8_t DynamixelClass::Velocity(uint8_t ID, int32_t Speed) {
    /*
    units = 0.229 rpm
    max velocity: 48 rpm (-210~210)
    */
    
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = WHEEL_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_GOAL_VELOCITY_1;
    Instruction_Packet_Array[6] = (uint8_t)(Speed & 0xFF);
    Instruction_Packet_Array[7] = (uint8_t)((Speed >> 8) & 0xFF);
    Instruction_Packet_Array[8] = (uint8_t)((Speed >> 16) & 0xFF);
    Instruction_Packet_Array[9] = (uint8_t)((Speed >> 24) & 0xFF);
    Instruction_Packet_Array[10] = ~(ID + WHEEL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_VELOCITY_1 + Instruction_Packet_Array[6] + Instruction_Packet_Array[7] + Instruction_Packet_Array[8] + Instruction_Packet_Array[9]);

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }

}


uint8_t DynamixelClass::Position(uint8_t ID, int32_t Position, int32_t Moving_Velocity) {
    /*
    units = 0.088 degree
    position range: 0~360 (0~4095)
    */
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = SERVO_GOAL_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = RAM_MOVING_VELOCITY_1;
    Instruction_Packet_Array[6] = (uint8_t)(Moving_Velocity);
    Instruction_Packet_Array[7] = (uint8_t)((Moving_Velocity & 0x0F00) >> 8);
    Instruction_Packet_Array[8] = 0x00;
    Instruction_Packet_Array[9] = 0x00;
    Instruction_Packet_Array[10] = (uint8_t)(Position);
    Instruction_Packet_Array[11] = (uint8_t)((Position & 0x0F00) >> 8);
    Instruction_Packet_Array[12] = 0x00;
    Instruction_Packet_Array[13] = 0x00;
    Instruction_Packet_Array[14] = ~(ID + SERVO_GOAL_LENGTH + COMMAND_WRITE_DATA + RAM_MOVING_VELOCITY_1 + (uint8_t)Moving_Velocity + (uint8_t)((Moving_Velocity & 0x0F00) >> 8) + (uint8_t)Position + (uint8_t)((Position & 0x0F00) >> 8));

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


//unsigned int DynamixelClass::checkMovement(uint8_t ID){    
//        
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = READ_MOVING_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
//    Instruction_Packet_Array[3] = RAM_MOVING;
//    Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
//    Instruction_Packet_Array[5] = ~(ID + READ_MOVING_LENGTH + COMMAND_READ_DATA + RAM_MOVING + READ_ONE_BYTE_LENGTH);
//
//    if (servoSerial->readable()) 
//      while (servoSerial->readable()) servoSerial->getc(); //empty buffer
//
//    transmitInstructionPacket();
//    readStatusPacket();
//    
//    if (Status_Packet_Array[2] == 0){                   // If there is no status packet error return value
//        return (Status_Packet_Array[3]);            // Return movement value
//    }else{
//        return (Status_Packet_Array[2] | 0xF000);            // If there is a error Returns error value
//    }
//}


int16_t DynamixelClass::ReadPWM(uint8_t ID) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x04;
    Instruction_Packet_Array[4] = COMMAND_READ_DATA;
    Instruction_Packet_Array[5] = RAM_PRESENT_PWM_1;
    Instruction_Packet_Array[6] = READ_TWO_BYTE_LENGTH;
    Instruction_Packet_Array[7] = ~(ID + 0x04 + COMMAND_READ_DATA + RAM_PRESENT_PWM_1 + READ_TWO_BYTE_LENGTH);

    packet_length = 6; //(ID + LEN + ERR + PARA(2) + CKSM)

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0) {                                           // If there is no status packet error return value
        return ((Status_Packet_Array[4] << 8) | Status_Packet_Array[3]);    // Return present load value
    }
    else {
        return (Status_Packet_Array[2]);                                   // If there is a error Returns error value
    }
}


int16_t DynamixelClass::ReadCurrent(uint8_t ID) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x04;
    Instruction_Packet_Array[4] = COMMAND_READ_DATA;
    Instruction_Packet_Array[5] = RAM_PRESENT_CURRENT_1;
    Instruction_Packet_Array[6] = READ_TWO_BYTE_LENGTH;
    Instruction_Packet_Array[7] = ~(ID + 0x04 + COMMAND_READ_DATA + RAM_PRESENT_CURRENT_1 + READ_TWO_BYTE_LENGTH);

    packet_length = 6; //(ID + LEN + ERR + PARA(2) + CKSM)

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0) {                                           // If there is no status packet error return value
        return ((Status_Packet_Array[4] << 8) | Status_Packet_Array[3]);    // Return present load value
    }
    else {
        return (Status_Packet_Array[2]);                                   // If there is a error Returns error value
    }
}


int32_t DynamixelClass::ReadVelocity(uint8_t ID) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = READ_SPEED_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_READ_DATA;
    Instruction_Packet_Array[5] = RAM_PRESENT_VELOCITY_1;
    Instruction_Packet_Array[6] = READ_FOUR_BYTE_LENGTH;
    Instruction_Packet_Array[7] = ~(ID + READ_SPEED_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_VELOCITY_1 + READ_FOUR_BYTE_LENGTH);

    packet_length = 8; //(ID + LEN + ERR + PARA(4) + CKSM)

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0) {                                           // If there is no status packet error return value
        return (Status_Packet_Array[6] << 24 | Status_Packet_Array[5] << 16 | Status_Packet_Array[4] << 8 | Status_Packet_Array[3]);  // Return present position value
    }
    else {
        return (Status_Packet_Array[2]);                           // If there is a error Returns error value
    }
}


int32_t DynamixelClass::ReadPosition(uint8_t ID) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = READ_POS_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_READ_DATA;
    Instruction_Packet_Array[5] = RAM_PRESENT_POSITION_1;
    Instruction_Packet_Array[6] = READ_FOUR_BYTE_LENGTH;
    Instruction_Packet_Array[7] = ~(ID + READ_POS_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_POSITION_1 + READ_FOUR_BYTE_LENGTH);

    packet_length = 8; //(ID + LEN + ERR + PARA(4) + CKSM)

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
        return (Status_Packet_Array[6] << 24 | Status_Packet_Array[5] << 16 | Status_Packet_Array[4] << 8 | Status_Packet_Array[3]); // Return present position value
    }
    else {
        return (Status_Packet_Array[2]);            // If there is a error Returns error value
    }
}


int16_t DynamixelClass::ReadVoltage(uint8_t ID) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x04;
    Instruction_Packet_Array[4] = COMMAND_READ_DATA;
    Instruction_Packet_Array[5] = RAM_PRESENT_VOLTAGE;
    Instruction_Packet_Array[6] = READ_TWO_BYTE_LENGTH;
    Instruction_Packet_Array[7] = ~(ID + 0x04 + COMMAND_READ_DATA + RAM_PRESENT_VOLTAGE + READ_TWO_BYTE_LENGTH);

    packet_length = 6; //(ID + LEN + ERR + PARA(2) + CKSM)

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0) {                   // If there is no status packet error return value
        return (Status_Packet_Array[4] << 8 | Status_Packet_Array[3]);                  // Return voltage value (value retured by Dynamixel is 10 times actual voltage)
    }
    else {
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
    }
}


//unsigned int DynamixelClass::readTemperature(uint8_t ID){ 
//        
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = READ_TEMP_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
//    Instruction_Packet_Array[3] = RAM_PRESENT_TEMPERATURE;
//    Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
//    Instruction_Packet_Array[5] = ~(ID + READ_TEMP_LENGTH  + COMMAND_READ_DATA + RAM_PRESENT_TEMPERATURE + READ_ONE_BYTE_LENGTH);
//    
//    if (servoSerial->readable()) 
//      while (servoSerial->readable()) servoSerial->getc(); //empty buffer
//
//    transmitInstructionPacket();
//    readStatusPacket(); 
//
//    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
//        return Status_Packet_Array[3];
//    }else{
//        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
//    }
//}


int8_t DynamixelClass::SetIndirectAddress(uint8_t ID, uint8_t Indirect, uint8_t Addr) {
    // Write specific address to indirect address
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x0B;
    Instruction_Packet_Array[4] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[5] = Indirect;
    Instruction_Packet_Array[6] = Addr;
    Instruction_Packet_Array[7] = 0x00;
    Instruction_Packet_Array[8] = Addr + 1;
    Instruction_Packet_Array[9] = 0x00;
    Instruction_Packet_Array[10] = Addr + 2;
    Instruction_Packet_Array[11] = 0x00;
    Instruction_Packet_Array[12] = Addr + 3;
    Instruction_Packet_Array[13] = 0x00;
    Instruction_Packet_Array[14] = ~(ID + 0x0B + COMMAND_WRITE_DATA + Indirect + Instruction_Packet_Array[4] + Instruction_Packet_Array[6] + Instruction_Packet_Array[8] + Instruction_Packet_Array[10]);
    
    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();
    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------
// Special Command

//unsigned int DynamixelClass::ping(uint8_t ID){
//    
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = PING_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_PING;
//    Instruction_Packet_Array[3] = ~(ID + PING_LENGTH + COMMAND_PING);
//    
//    if (servoSerial->readable()) 
//      while (servoSerial->readable()) servoSerial->getc(); //empty buffer
//
//    transmitInstructionPacket();    
//    readStatusPacket();
//    
//    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
//        return (Status_Packet_Array[0]);            // Return SERVO ID
//    }else{
//        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
//    }            
//}


//unsigned int DynamixelClass::action(uint8_t ID){
//    
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = RESET_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_ACTION;
//    Instruction_Packet_Array[3] = ~(ID + ACTION_LENGTH + COMMAND_ACTION);
//    
//    if (servoSerial->readable()) 
//      while (servoSerial->readable()) servoSerial->getc(); //empty buffer
//
//    transmitInstructionPacket();    
//    
//    if (ID == 0XFE || Status_Return_Level != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
//        return (0x00);
//    }else{
//        readStatusPacket();
//        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
//            return (Status_Packet_Array[0]);            // Return SERVO ID
//        }else{
//            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
//        }
//    }       
//}


uint8_t DynamixelClass::reset(uint8_t ID) {

    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = RESET_LENGTH;
    Instruction_Packet_Array[4] = COMMAND_RESET;
    Instruction_Packet_Array[5] = ~(ID + RESET_LENGTH + COMMAND_RESET); //Checksum;

    packet_length = 4; //(ID + LEN + ERR + CKSM)

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Level != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }
    else {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) {               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }
        else {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}


void DynamixelClass::SyncWrite_StatusReturnLevel(uint8_t level) {
    uint8_t checksum = 0;

    Instruction_Packet_Array[2] = 0xFE;
    Instruction_Packet_Array[3] = 2 * 12 + 4;  // total data frame length (3* Nth ID +3)
    Instruction_Packet_Array[4] = COMMAND_SYNC_WRITE;
    Instruction_Packet_Array[5] = RAM_STATUS_RETURN_LEVEL;     // start addr at indirect data1
    Instruction_Packet_Array[6] = 0x01;
    for (int i = 1; i <= 12; i++) {// There are n devices to read
        Instruction_Packet_Array[2 * i + 5] = i;
        Instruction_Packet_Array[2 * i + 6] = level;
    }
    for (int i = 2; i < 2 * 12 + 7; i++) { checksum += Instruction_Packet_Array[i]; }
    checksum = ~checksum;
    Instruction_Packet_Array[2 * 12 + 7] = checksum;

    Status_Return_Level = level;

    transmitInstructionPacket();
}


void DynamixelClass::SyncWrite_SetIndirectAddress() {
    uint8_t checksum = 0;

    Instruction_Packet_Array[2] = 0xFE;
    Instruction_Packet_Array[3] = 9 * 12 + 4;  // total data frame length (3* Nth ID +3)
    Instruction_Packet_Array[4] = COMMAND_SYNC_WRITE;
    Instruction_Packet_Array[5] = 0xA8;     // start addr at indirect data1
    Instruction_Packet_Array[6] = 0x08;
    for (int i = 1; i <= 12; i++) {// There are n devices to read
        Instruction_Packet_Array[9 * i - 2] = i;
        Instruction_Packet_Array[9 * i] = 0x00;
        Instruction_Packet_Array[9 * i + 2] = 0x00;
        Instruction_Packet_Array[9 * i + 4] = 0x00;
        Instruction_Packet_Array[9 * i + 6] = 0x00;
        if (i == 1 || i == 2 || i == 6 || i == 7 || i == 8 || i == 12) {
            Instruction_Packet_Array[9 * i - 1] = RAM_GOAL_POSITION_1;
            Instruction_Packet_Array[9 * i + 1] = RAM_GOAL_POSITION_2;
            Instruction_Packet_Array[9 * i + 3] = RAM_GOAL_POSITION_3;
            Instruction_Packet_Array[9 * i + 5] = RAM_GOAL_POSITION_4;
        }
        else {
            Instruction_Packet_Array[9 * i - 1] = RAM_GOAL_VELOCITY_1;
            Instruction_Packet_Array[9 * i + 1] = RAM_GOAL_VELOCITY_2;
            Instruction_Packet_Array[9 * i + 3] = RAM_GOAL_VELOCITY_3;
            Instruction_Packet_Array[9 * i + 5] = RAM_GOAL_VELOCITY_4;
        }
    }

    for (int i = 2; i < 9 * 12 + 7; i++) { checksum += Instruction_Packet_Array[i]; }
    checksum = ~checksum;
    Instruction_Packet_Array[9 * 12 + 7] = checksum;

    transmitInstructionPacket();
}


void DynamixelClass::SyncWrite_n_dynamixels(uint8_t n, uint8_t *ID_list, int32_t *cmd) {
    /*
    This feature (Bulk read) cannot read the same ID multiple times,
    or it will only return the first designed parameter
    */
    uint8_t checksum = 0;

    Instruction_Packet_Array[2] = 0xFE;
    Instruction_Packet_Array[3] = 5 * n + 4;  // total data frame length (3* Nth ID +3)
    Instruction_Packet_Array[4] = COMMAND_SYNC_WRITE;
    Instruction_Packet_Array[5] = 0xE0;     // start addr at indirect data1
    Instruction_Packet_Array[6] = 0x04;
    for (int i = 1; i <= n; i++) {// There are n devices to read
        Instruction_Packet_Array[5 * i + 2] = ID_list[i - 1];
        Instruction_Packet_Array[5 * i + 3] = (uint8_t)(cmd[ID_list[i - 1]] & 0x000000FF);
        Instruction_Packet_Array[5 * i + 4] = (uint8_t)((cmd[ID_list[i - 1]] >> 8) & 0x000000FF);
        Instruction_Packet_Array[5 * i + 5] = (uint8_t)((cmd[ID_list[i - 1]] >> 16) & 0x000000FF);
        Instruction_Packet_Array[5 * i + 6] = (uint8_t)((cmd[ID_list[i - 1]] >> 24) & 0x000000FF);
    }
    for (int i = 2; i < 5 * n + 7; i++) { checksum += Instruction_Packet_Array[i]; }
    checksum = ~checksum;
    Instruction_Packet_Array[5 * n + 7] = checksum;

    transmitInstructionPacket();
}


int32_t DynamixelClass::BulkRead_n_dynamixels(uint8_t n, uint8_t* ID_list, int32_t* position, int32_t* velocity) {
    // This function read each dynamixel's velocity and position.
    /*
    This feature (Bulk read) cannot read the same ID multiple times,
    or it will only return the first designed parameter
    */
    uint8_t checksum = 0;
    packet_length = 12; //(ID + LEN + ERR + PARA(8) + CKSM)

    Instruction_Packet_Array[2] = 0xFE;
    Instruction_Packet_Array[3] = 3 * n + 3;  // total data frame length (3* Nth ID +3)
    Instruction_Packet_Array[4] = COMMAND_BULK_READ;
    Instruction_Packet_Array[5] = 0x00;
    for (int i = 1; i <= n; i++) {// There are n devices to read
        Instruction_Packet_Array[3 * i + 3] = 0x08; // nth device data length
        Instruction_Packet_Array[3 * i + 4] = ID_list[i - 1];
        Instruction_Packet_Array[3 * i + 5] = RAM_PRESENT_VELOCITY_1; // nth data address
    }
    for (int i = 2; i < 3 * n + 6; i++) { checksum += Instruction_Packet_Array[i]; }
    checksum = ~checksum;
    Instruction_Packet_Array[3 * n + 6] = checksum;

    transmitInstructionPacket();

    for (int i = 1; i <= n; i++) {
        readStatusPacket();
        if (Status_Packet_Array[2] == 0) { // If there is no status packet error return value
            velocity[Status_Packet_Array[0]] = Status_Packet_Array[6] << 24 | Status_Packet_Array[5] << 16 | Status_Packet_Array[4] << 8 | Status_Packet_Array[3];
            position[Status_Packet_Array[0]] = Status_Packet_Array[10] << 24 | Status_Packet_Array[9] << 16 | Status_Packet_Array[8] << 8 | Status_Packet_Array[7];
        }
        else {
            return Status_Packet_Array[2];
        }
    }
    return 0;
}


int32_t DynamixelClass::ReadRegister(uint8_t ID, uint16_t Register) {
    Instruction_Packet_Array[2] = ID;
    Instruction_Packet_Array[3] = 0x04;
    Instruction_Packet_Array[4] = COMMAND_READ_DATA;
    Instruction_Packet_Array[5] = Register;
    Instruction_Packet_Array[6] = READ_FOUR_BYTE_LENGTH;
    Instruction_Packet_Array[7] = ~(ID + 0x04 + COMMAND_READ_DATA + Register + READ_FOUR_BYTE_LENGTH);

    packet_length = 8; //(ID + LEN + ERR + PARA(this might chage) + CKSM)

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0)
    {               // If there is no status packet error return value
        return (Status_Packet_Array[6] << 24 | Status_Packet_Array[5] << 16 | Status_Packet_Array[4] << 8 | Status_Packet_Array[3]);
    }
    else
    {
        return Status_Packet_Array[2];// | 0xF000);   // If there is a error Returns error value
    }
}


int32_t DynamixelClass::testfunction(uint8_t ID) {
    /*
    This function is used for devoloping or testing,
    the content in here would be changed often
    */
    /*
    This feature (Bulk read) cannot read the same ID multiple times,
    or it will only return the first designed parameter
    */
    uint8_t checksum = 0;
    uint8_t id_status[10][15];
    packet_length = 8; //(ID + LEN + ERR + PARA(4) + CKSM)
    //  printf("checksum = %x\r\n",checksum);
    Instruction_Packet_Array[2] = 0xFE;
    Instruction_Packet_Array[3] = 0x21;  // total data frame length (3* Nth ID +3)
    Instruction_Packet_Array[4] = COMMAND_SYNC_WRITE;
    Instruction_Packet_Array[5] = RAM_PRESENT_POSITION_1;
    Instruction_Packet_Array[6] = 0x04;
    Instruction_Packet_Array[7] = ID;
    /*
    for (int n = 1; n <= 10; n++) // There are n devices to read
    {
        Instruction_Packet_Array[3 * n + 1] = 0x04; // nth device data length
        Instruction_Packet_Array[3 * n + 2] = ID[n - 1];
        Instruction_Packet_Array[3 * n + 3] = RAM_PRESENT_POSITION_1; // nth data address
    }
    */
    for (int i = 0; i < 8; i++) { checksum += Instruction_Packet_Array[i]; }
    //    printf("checksum = %x\r\n",checksum);
    checksum = ~checksum;
    //  printf("checksum = %x\r\n", checksum);
    Instruction_Packet_Array[8] = checksum;

    transmitInstructionPacket();
    readStatusPacket();
    /*
    for (int n = 1; n <= 10; n++) {
        readStatusPacket();
        for (int i = 0; i < 10; i++) {
            id_status[n - 1][i] = Status_Packet_Array[i];
        }
    }
    */
    ////    for (int i=0; i<10; i++) {printf("%x\t",id1_status[i]);}
    ////    printf("\r\nid1 status packet !\r\n");
    ////    for (int i=0; i<10; i++) {printf("%x\t",id2_status[i]);}
    ////    printf("\r\nid2 status packet !\r\n");
    ////    debugStatusframe();
    if (Status_Packet_Array[2] == 0) { // If there is no status packet error return value
        return (Status_Packet_Array[6] << 24 | Status_Packet_Array[5] << 16 | Status_Packet_Array[4] << 8 | Status_Packet_Array[3]);
    }

//  if (id_status[0][2] == 0 || id_status[1][2] == 0) { // If there is no status packet error return value
//      position[0] = (id_status[0][6] << 24 | id_status[0][5] << 16 | id_status[0][4] << 8 | id_status[0][3]);
//      position[1] = (id_status[1][6] << 24 | id_status[1][5] << 16 | id_status[1][4] << 8 | id_status[1][3]);
//  }
//  else {
//      position[0] = id_status[0][2];// | 0xF000);   // If there is a error Returns error value
//      position[1] = id_status[1][2];// | 0xF000);
//  }
    return 0;
}
