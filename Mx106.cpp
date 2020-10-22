#include "mbed.h"
#include "Mx106.h"

GPIO_InitTypeDef   huart4_gpio;
UART_HandleTypeDef huart4;

uint8_t   Instruction_Packet_Array[35];   // Array to hold instruction packet data 
uint8_t   Status_Packet_Array[15];        // Array to hold returned status packet data

// Because printing the packet directly maybe clean the data in UART,
// We need to copy packet into debug_packet
// and print the debug_packet after reading all data in packet we need.
uint8_t   debug_Instruction_Packet_Array[35];   // Array to debug instruction packet data
uint8_t   debug_Status_Packet_Array[15];        // Array to debug status packet data

uint8_t   Status_Return_Value = READ;     // Status packet return states ( NON , READ , ALL )

uint16_t packet_length = 0;
uint8_t packet_header[2] = { HEADER, HEADER };

uint32_t reg1 = 0;
uint32_t reg2 = 0;
//-------------------------------------------------------------------------------------------------------------------------------
// Private Methods 
//-------------------------------------------------------------------------------------------------------------------------------
void DynamixelClass::debugInstructionframe(void) {
	for (int i = 0; i < 10; i++)
		debug_Instruction_Packet_Array[i] = Instruction_Packet_Array[i];
	//	for (int i = 0; i < 10; i++) printf("%x\t,",debug_Instruction_Packet_Array[i]);
	//	printf("\r\nyou transmit!\r\n"); 
}

void DynamixelClass::debugStatusframe(void) {
	for (int i = 0; i < 10; i++)
		debug_Status_Packet_Array[i] = Status_Packet_Array[i];
	//	    for (int i = 0; i < 10; i++) printf("%x\t",debug_Status_Packet_Array[i]);
	//	    printf("\r\nyou recieved!\r\n");
	//printf("\r");
}


void DynamixelClass::transmitInstructionPacket(void) {                           // Transmit instruction packet to Dynamixel
	servoSerialDir->write(1);

	HAL_UART_Transmit(&huart4, packet_header, 2, 1);
	HAL_UART_Transmit(&huart4, Instruction_Packet_Array, Instruction_Packet_Array[1] + 2, 1);

	while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == 0) {}
	__HAL_UART_FLUSH_DRREGISTER(&huart4);
	servoSerialDir->write(0);

	//	debugframe();
}


void DynamixelClass::readStatusPacket(void) {
	static unsigned char InBuff[5];

	HAL_UART_Receive(&huart4, InBuff, 2, 1);
	if (InBuff[0] == HEADER && InBuff[1] == HEADER) {
		//		HAL_UART_Receive(&huart4, InBuff, 2, 1);
		HAL_UART_Receive(&huart4, Status_Packet_Array, packet_length, 1);
	}

	//    debugStatusframe();
}


//-------------------------------------------------------------------------------------------------------------------------------
// Public Methods 
//-------------------------------------------------------------------------------------------------------------------------------
DynamixelClass::DynamixelClass(int baud, PinName D_Pin, PinName tx, PinName rx) {
	servoSerialDir = new DigitalOut(D_Pin);
	servoSerialDir->write(0);

	HAL_Init();

	//uart gpio config
	__GPIOA_CLK_ENABLE();
	//PA0 -> TX ,PA1->RX
	huart4_gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	huart4_gpio.Mode = GPIO_MODE_AF_PP;
	huart4_gpio.Pull = GPIO_PULLUP;
	huart4_gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	huart4_gpio.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOA, &huart4_gpio);

	//uart config
	__UART4_CLK_ENABLE();
	huart4.Init.BaudRate = baud;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Instance = UART4;
	HAL_UART_Init(&huart4);

	//    NVIC_SetVector(UART4_IRQn,(uint32_t)UART_IRQHandler);
	//    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
	//    HAL_NVIC_EnableIRQ(UART4_IRQn);
}


DynamixelClass::~DynamixelClass() {
	if (servoSerialDir != NULL) delete servoSerialDir;
}


//void DynamixelClass::MCU_BaudRate(int baud) {
////Change the baudrate on then MBED
//    int Baudrate_BPS = 0;
//    Baudrate_BPS  =(int) 2000000 / (baud + 1);                        // Calculate Baudrate as ber "Robotis e-manual"
//    servoSerial -> baud(Baudrate_BPS);
//}
//


//-------------------------------------------------------------------------------------------------------------------------------
// EEPROM AREA  

unsigned int DynamixelClass::setID(unsigned char ID, unsigned char New_ID) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_ID_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_ID;
	Instruction_Packet_Array[4] = New_ID;
	Instruction_Packet_Array[5] = ~(ID + SET_ID_LENGTH + COMMAND_WRITE_DATA + EEPROM_ID + New_ID);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


//unsigned int DynamixelClass::setStatusPaketReturnDelay(unsigned char ID,unsigned char ReturnDelay){
//    
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = SET_RETURN_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//    Instruction_Packet_Array[3] = EEPROM_RETURN_DELAY_TIME;
//    Instruction_Packet_Array[4] = (char) (ReturnDelay/2);
//    Instruction_Packet_Array[5] = ~(ID + SET_RETURN_LENGTH + COMMAND_WRITE_DATA + EEPROM_RETURN_DELAY_TIME + (char)(ReturnDelay/2));  
//    
//    if (servoSerial->readable()) 
//      while (servoSerial->readable()) servoSerial->getc(); //empty buffer
//
//    transmitInstructionPacket();    
//    
//    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::OperationMode(unsigned char ID, unsigned char OPEARTION_MODE) {
	/*
	Set Operation Mode: Current Mode 0x00, Velocity Mode 0x01, Position Mode 0x03
	*/
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_OPERATION_MODE_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_OPERATION_MODE;
	Instruction_Packet_Array[4] = OPEARTION_MODE;
	Instruction_Packet_Array[5] = ~(ID + SET_OPERATION_MODE_LENGTH + COMMAND_WRITE_DATA + EEPROM_OPERATION_MODE + OPEARTION_MODE);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


//unsigned int DynamixelClass::setTemp(unsigned char ID,unsigned char temp){
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
//    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::MaxMinVoltageLimit(unsigned char ID, unsigned short Volt_H, unsigned short Volt_L) {
	char Volt_L1, Volt_L2, Volt_H1, Volt_H2;
	Volt_L1 = Volt_L & 0xFF;
	Volt_L2 = (Volt_L >> 8) & 0xFF;
	Volt_H1 = Volt_H & 0xFF;
	Volt_H2 = (Volt_H >> 8) & 0xFF;

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x07;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_MAX_VOLTAGE_LIMIT_1;
	Instruction_Packet_Array[4] = Volt_H1;
	Instruction_Packet_Array[5] = Volt_H2;
	Instruction_Packet_Array[6] = Volt_L1;
	Instruction_Packet_Array[7] = Volt_L2;
	Instruction_Packet_Array[8] = ~(ID + 0x07 + COMMAND_WRITE_DATA + EEPROM_MAX_VOLTAGE_LIMIT_1 + Volt_H1 + Volt_H2 + Volt_L1 + Volt_L2);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::VelocityLimit(unsigned char ID, unsigned short Velocity_Limit) {
	/*
	Unit        = 0.229     rpm
	Value range = 0 ~ 1023
	*/
	char Limit_1, Limit_2, Limit_3, Limit_4; // (+)CCW (-)CW
	Limit_1 = Velocity_Limit & 0xFF;
	Limit_2 = (Velocity_Limit >> 8) & 0xFF;
	Limit_3 = (Velocity_Limit >> 16) & 0xFF;
	Limit_4 = (Velocity_Limit >> 24) & 0xFF;

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x07;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = EEPROM_MAX_VELOCITY_LIMIT_1;
	Instruction_Packet_Array[4] = Limit_1;
	Instruction_Packet_Array[5] = Limit_2;
	Instruction_Packet_Array[6] = Limit_3;
	Instruction_Packet_Array[7] = Limit_4;
	Instruction_Packet_Array[8] = ~(ID + 0x07 + COMMAND_WRITE_DATA + EEPROM_MAX_VELOCITY_LIMIT_1 + Limit_1 + Limit_2 + Limit_3 + Limit_4);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();
	if (Status_Return_Value == ALL) {
		readStatusPacket();
		if (Status_Packet_Array[2] != 0) {
			return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
		}
	}
	return 0x00; //if no errors
}


//unsigned int DynamixelClass::Shutdown(unsigned char  ID,unsigned char Set){
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
//    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
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

unsigned int DynamixelClass::TorqueEnable(unsigned char ID, bool Status) {
	/*
	Must Enable it before any motion(Velocity or Position)
	When it is enabled, EEROM will be locked.
	*/
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = TORQUE_ENABLE_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_TORQUE_ENABLE;
	Instruction_Packet_Array[4] = Status;
	Instruction_Packet_Array[5] = ~(ID + TORQUE_ENABLE_LENGTH + COMMAND_WRITE_DATA + RAM_TORQUE_ENABLE + Status);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::LED(unsigned char ID, unsigned char Status) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = LED_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_LED;
	Instruction_Packet_Array[4] = Status;
	Instruction_Packet_Array[5] = ~(ID + LED_LENGTH + COMMAND_WRITE_DATA + RAM_LED + Status);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::Velocity_PI(unsigned char ID, unsigned short P, unsigned short I) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_VELOCITY_PI_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_VELOCITY_I_GAIN_L;
	Instruction_Packet_Array[4] = (char)(I);
	Instruction_Packet_Array[5] = (char)((I & 0xFF00) >> 8);
	Instruction_Packet_Array[6] = (char)(P);
	Instruction_Packet_Array[7] = (char)((P & 0xFF00) >> 8);
	Instruction_Packet_Array[8] = ~(ID + SET_VELOCITY_PI_LENGTH + COMMAND_WRITE_DATA + RAM_VELOCITY_I_GAIN_L + (char)(P)+(char)((P & 0xFF00) >> 8) + (char)(I)+(char)((I & 0xFF00) >> 8));

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::Position_PID(unsigned char ID, unsigned char P, unsigned char I, unsigned char D) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SET_POSITION_PID_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_POSITION_D_GAIN_L;
	Instruction_Packet_Array[4] = D;
	Instruction_Packet_Array[5] = 0x00;
	Instruction_Packet_Array[6] = I;
	Instruction_Packet_Array[7] = 0x00;
	Instruction_Packet_Array[8] = P;
	Instruction_Packet_Array[9] = 0x00;
	Instruction_Packet_Array[10] = ~(ID + SET_POSITION_PID_LENGTH + COMMAND_WRITE_DATA + RAM_POSITION_D_GAIN_L + P + I + D);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::PWM(unsigned char ID, int PWM) {
	char PWM_1, PWM_2;
	PWM_1 = PWM & 0xFF;
	PWM_2 = (PWM >> 8) & 0xFF;

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x05;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_GOAL_PWM_1;
	Instruction_Packet_Array[4] = PWM_1;
	Instruction_Packet_Array[5] = PWM_2;
	Instruction_Packet_Array[6] = ~(ID + 0x05 + COMMAND_WRITE_DATA + RAM_GOAL_PWM_1 + PWM_1 + PWM_2);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::Current(unsigned char ID, int Current) {
	char Current_1, Current_2;
	Current_1 = Current & 0xFF;
	Current_2 = (Current >> 8) & 0xFF;

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x05;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_GOAL_CURRENT_1;
	Instruction_Packet_Array[4] = Current_1;
	Instruction_Packet_Array[5] = Current_2;
	Instruction_Packet_Array[6] = ~(ID + 0x05 + COMMAND_WRITE_DATA + RAM_GOAL_CURRENT_1 + Current_1 + Current_2);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::Velocity(unsigned char ID, int Speed) {
	/*
	units = 0.229 rpm
	max velocity: 48 rpm (-210~210)
	*/
	char Speed_1, Speed_2, Speed_3, Speed_4; // (+)CCW (-)CW
	Speed_1 = Speed & 0xFF;
	Speed_2 = (Speed >> 8) & 0xFF;
	Speed_3 = (Speed >> 16) & 0xFF;
	Speed_4 = (Speed >> 24) & 0xFF;

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = WHEEL_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_GOAL_VELOCITY_1;
	Instruction_Packet_Array[4] = Speed_1;
	Instruction_Packet_Array[5] = Speed_2;
	Instruction_Packet_Array[6] = Speed_3;
	Instruction_Packet_Array[7] = Speed_4;
	Instruction_Packet_Array[8] = ~(ID + WHEEL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_VELOCITY_1 + Speed_1 + Speed_2 + Speed_3 + Speed_4);

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::Position(unsigned char ID, unsigned int Position, unsigned int Moving_Velocity) {
	/*
	units = 0.088 degree
	position range: 0~360 (0~4095)
	*/
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
	Instruction_Packet_Array[3] = RAM_MOVING_VELOCITY_1;
	Instruction_Packet_Array[4] = (char)(Moving_Velocity);
	Instruction_Packet_Array[5] = (char)((Moving_Velocity & 0x0F00) >> 8);
	Instruction_Packet_Array[6] = 0x00;
	Instruction_Packet_Array[7] = 0x00;
	Instruction_Packet_Array[8] = (char)(Position);
	Instruction_Packet_Array[9] = (char)((Position & 0x0F00) >> 8);
	Instruction_Packet_Array[10] = 0x00;
	Instruction_Packet_Array[11] = 0x00;
	Instruction_Packet_Array[12] = ~(ID + SERVO_GOAL_LENGTH + COMMAND_WRITE_DATA + RAM_MOVING_VELOCITY_1 + Moving_Velocity + (char)((Moving_Velocity & 0x0F00) >> 8) + Position + (char)((Position & 0x0F00) >> 8));

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();


	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


//unsigned int DynamixelClass::checkMovement(unsigned char ID){    
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


short DynamixelClass::ReadPWM(unsigned char ID) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x04;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_PWM_1;
	Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + 0x04 + COMMAND_READ_DATA + RAM_PRESENT_PWM_1 + READ_TWO_BYTE_LENGTH);

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


short DynamixelClass::ReadCurrent(unsigned char ID) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x04;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_CURRENT_1;
	Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + 0x04 + COMMAND_READ_DATA + RAM_PRESENT_CURRENT_1 + READ_TWO_BYTE_LENGTH);

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


int DynamixelClass::ReadVelocity(unsigned char ID) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_SPEED_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_VELOCITY_1;
	Instruction_Packet_Array[4] = READ_FOUR_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_SPEED_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_VELOCITY_1 + READ_FOUR_BYTE_LENGTH);

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


unsigned int DynamixelClass::ReadPosition(unsigned char ID) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = READ_POS_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_POSITION_1;
	Instruction_Packet_Array[4] = READ_FOUR_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + READ_POS_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_POSITION_1 + READ_FOUR_BYTE_LENGTH);

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


unsigned short DynamixelClass::ReadVoltage(unsigned char ID) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x04;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_VOLTAGE;
	Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + 0x04 + COMMAND_READ_DATA + RAM_PRESENT_VOLTAGE + READ_TWO_BYTE_LENGTH);

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


//unsigned int DynamixelClass::readTemperature(unsigned char ID){ 
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


//-------------------------------------------------------------------------------------------------------------------------------
// Special Command

//unsigned int DynamixelClass::ping(unsigned char ID){
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


//unsigned int DynamixelClass::action(unsigned char ID){
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
//    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::reset(unsigned char ID) {

	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = RESET_LENGTH;
	Instruction_Packet_Array[2] = COMMAND_RESET;
	Instruction_Packet_Array[3] = ~(ID + RESET_LENGTH + COMMAND_RESET); //Checksum;

	packet_length = 4; //(ID + LEN + ERR + CKSM)

	transmitInstructionPacket();

	if (ID == 0XFE || Status_Return_Value != ALL) {     // If ID of FE is used no status packets are returned so we do not need to check it
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


unsigned int DynamixelClass::ReadRegister(unsigned char ID, unsigned char Register) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x04;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = Register;
	Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + 0x04 + COMMAND_READ_DATA + Register + READ_TWO_BYTE_LENGTH);
	
	packet_length = 8; //(ID + LEN + ERR + PARA(this might chage) + CKSM)

	transmitInstructionPacket();
	readStatusPacket();

	if (Status_Packet_Array[2] == 0)
	{               // If there is no status packet error return value
		return (Status_Packet_Array[4] << 8 | Status_Packet_Array[3]);
	}
	else
	{
		return Status_Packet_Array[2];// | 0xF000);   // If there is a error Returns error value
	}
}


unsigned int DynamixelClass::testfunction(unsigned char ID[10], unsigned int* position) {
	/*
	This function is used for devoloping or testing,
	the content in here would be changed often
	*/
	/*
	This feature (Bulk read) cannot read the same ID multiple times,
	or it will only return the first designed parameter
	*/
	unsigned char checksum = 0x00;
	unsigned char id_status[10][15];

	//	printf("checksum = %x\r\n",checksum);
	Instruction_Packet_Array[0] = 0xFE;
	Instruction_Packet_Array[1] = 0x21;  // total data frame length (3* Nth ID +3)
	Instruction_Packet_Array[2] = COMMAND_BULK_READ;
	Instruction_Packet_Array[3] = 0x00;
	for (int n = 1; n <= 10; n++) // There are n devices to read
	{
		Instruction_Packet_Array[3 * n + 1] = 0x04; // nth device data length
		Instruction_Packet_Array[3 * n + 2] = ID[n - 1];
		Instruction_Packet_Array[3 * n + 3] = RAM_PRESENT_POSITION_1; // nth data address
	}
	for (int i = 0; i < 34; i++) { checksum += Instruction_Packet_Array[i]; }
	//    printf("checksum = %x\r\n",checksum);
	checksum = ~checksum;
	printf("checksum = %x\r\n", checksum);
	Instruction_Packet_Array[34] = checksum;

	transmitInstructionPacket();
	//------------------  test  ----------------------------------
	//    while (i < 21) 
	//    {
	//    	if (servoSerial->readable())
	//    	{
	//    		Status_Packet_Array[i] = servoSerial->getc();
	//    		i++;   		
	//    	}
	//    }
	//    for (int j = 0; j < 30; j++)
	//    {
	//    	printf("%x\t",Status_Packet_Array[j]);
	//    }
	//    printf("\r\n");
	//------------------  test  ----------------------------------

	for (int n = 1; n <= 10; n++)
	{
		readStatusPacket();
		for (int i = 0; i < 10; i++)
		{
			id_status[n - 1][i] = Status_Packet_Array[i];
		}
	}

	////    for (int i=0; i<10; i++) {printf("%x\t",id1_status[i]);}
	////    printf("\r\nid1 status packet !\r\n");
	////    for (int i=0; i<10; i++) {printf("%x\t",id2_status[i]);}
	////    printf("\r\nid2 status packet !\r\n");
	////    debugStatusframe();


	if (id_status[0][2] == 0 && id_status[1][2] == 0)
	{ // If there is no status packet error return value
		position[0] = (id_status[0][6] << 24 | id_status[0][5] << 16 | id_status[0][4] << 8 | id_status[0][3]);
		position[1] = (id_status[1][6] << 24 | id_status[1][5] << 16 | id_status[1][4] << 8 | id_status[1][3]);
	}
	else
	{
		position[0] = id_status[0][2];// | 0xF000);   // If there is a error Returns error value
		position[1] = id_status[1][2];// | 0xF000);
	}
	return 0;
}
