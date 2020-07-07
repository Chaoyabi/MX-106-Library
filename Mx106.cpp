#include "mbed.h"
#include "Mx106.h"


unsigned char   Instruction_Packet_Array[35];   // Array to hold instruction packet data 
unsigned char   Status_Packet_Array[15];        // Array to hold returned status packet data

// Because printing the packet directly maybe clean the data in UART,
// We need to copy packet into debug_packet
// and print the debug_packet after reading all data in packet we need.
unsigned char   debug_Instruction_Packet_Array[35];   // Array to debug instruction packet data
unsigned char   debug_Status_Packet_Array[15];        // Array to debug status packet data

unsigned char   Status_Return_Value = READ;     // Status packet return states ( NON , READ , ALL )

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
//    for (int i = 0; i < 10; i++) printf("%x\t",debug_Status_Packet_Array[i]);
//    printf("\r\nyou recieved!\r\n"); 
}


void DynamixelClass::transmitInstructionPacket(void){                           // Transmit instruction packet to Dynamixel
//	Timer timer;
//    unsigned char Counter = 0;
    bool tx_complete = 0;
    
    servoSerialDir->write(1);                                                   // Set TX Buffer pin to HIGH    
//    timer.start();
	UART4->SR &= 0xFFFFFFBF;
    servoSerial->putc(HEADER);                                                  // Write Header (0xFF) data 1 to serial                     
    servoSerial->putc(HEADER);                                                  // Write Header (0xFF) data 2 to serial
    servoSerial->putc(Instruction_Packet_Array[0]);                             // Write Dynamixal ID to serial 
    servoSerial->putc(Instruction_Packet_Array[1]);                             // Write packet length to serial    
    
    for(int i = 0; i < Instruction_Packet_Array[1]; i++){
    	servoSerial->putc(Instruction_Packet_Array[i + 2]);						// Write Instuction & Parameters (if there is any) & check sum to serial
    }
    
//    wait_us((Counter + 4)*3);
//    wait_us(30);
	do{
		tx_complete = (UART4->SR >> 6) & 0x00000001;
	}while( tx_complete == 0 );
//    while(timer.read_us() > ((Instruction_Packet_Array[1]+5) *10)){}
    servoSerialDir->write(0);                                                   // Set TX Buffer pin to LOW after data has been sent
//	debugframe();
}


void DynamixelClass::readStatusPacket(void){

    unsigned char Counter = 0x00;

    static unsigned char InBuff[20];
    unsigned char i, j, RxState;

    Status_Packet_Array[0] = 0x00;
    Status_Packet_Array[1] = 0x00;
    Status_Packet_Array[2] = 0x00;                                                      
    Status_Packet_Array[3] = 0x00;
    i=0; RxState=0; j=0; InBuff[0]=0;
    
//    Timer timer;
//    timer.start();

    while (RxState<3)
    {// Wait for " header + header + frame length + error " RX data 

        if (servoSerial->readable()) 
        {
//            timer.reset();
            InBuff[i] = servoSerial->getc();
            if (InBuff[0] == 0xFF) i++;                             // When we have the first header we starts to inc data to inbuffer
            if ((i >= (STATUS_FRAME_BUFFER-1)) &&(RxState==0)) RxState++;       //read header

            switch (RxState) 
            {
                case 1: 
                { // Read ID, frame length, error 
                    if ((InBuff[j] == 0xFF) && (InBuff[j+1]== 0xFF))
                    { // checkes that we have got the buffer
                        j=2;
                        Status_Packet_Array[0] = InBuff[j++];                   // ID sent from Dynamixel
                        Status_Packet_Array[1] = InBuff[j++];                   // Frame Length of status packet
                        Status_Packet_Array[2] = InBuff[j++];                   // Error (char) 
                        RxState++; led2->write(0);                              //j = 5 , RxState = 2
                    }
                } 
                break;
                
                case 2: 
                { // Read data
                    if (i > Status_Packet_Array[1]+3) 
                    { // We have recieved all data
                        do{
                            Status_Packet_Array[3 + Counter] = InBuff[j++];
                            Counter++;              
                        }while(Status_Packet_Array[1] > (Counter+1));           // Read Parameter(s) into array

                        Status_Packet_Array[Counter + 4] = InBuff[j++];         // Read Check sum   
                        RxState++;
                    }
                }
                break;      
            } // switch 
        } // if
//        else // If there is nothing can be read in RX, then break while
//        {
//        	if (timer.read_ms() >= STATUS_PACKET_TIMEOUT)              // The timeout have some problem to be detected
//        	{
//            	printf("timer read : %d ms\r\n",timer.read_ms());
//            	printf("I recieved nothing!\r\n");
//        		break;
//        	}
//        }
	} //while..
//    timer.stop();

//    debugStatusframe();
}



//-------------------------------------------------------------------------------------------------------------------------------
// Public Methods 
//-------------------------------------------------------------------------------------------------------------------------------
DynamixelClass::DynamixelClass(int baud, PinName D_Pin, PinName tx, PinName rx){ 
    servoSerial = new Serial(tx, rx);
    servoSerial -> baud(baud);
    servoSerialDir = new DigitalOut(D_Pin);     
    servoSerialDir -> write(0);
    led2 = new DigitalOut(LED2); 
}   


DynamixelClass::~DynamixelClass(){
    if(servoSerial != NULL) delete servoSerial;
    if(servoSerialDir != NULL) delete servoSerialDir;
    if(led2 != NULL) delete led2;
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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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


//unsigned int DynamixelClass::MX_BaudRate(unsigned char ID, long Baud){                      
//    
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = SET_BD_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//    Instruction_Packet_Array[3] = EEPROM_BAUDRATE;
////  if (Baud > 2250000){
//if (Baud >= 1000000){
//    switch (Baud){
//        case 2250000:
//        Instruction_Packet_Array[4] = 0xFA;
//        break;
//        case 2500000:
//        Instruction_Packet_Array[4] = 0xFB;
//        break;
//        case 3000000:
//        Instruction_Packet_Array[4] = 0xFC;
//        break;
//        case 1000000:
//        Instruction_Packet_Array[4] = 0x01;
//        }
//    }else{
//    Instruction_Packet_Array[4] = (char)((2000000/Baud) - 1);
//    }
//    Instruction_Packet_Array[5] = ~(ID + SET_BD_LENGTH + COMMAND_WRITE_DATA + EEPROM_BAUDRATE + (char)((2000000/Baud) - 1) ); 
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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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


//unsigned int DynamixelClass::setStatusPaket(unsigned char  ID,unsigned char Set){
//
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = SET_RETURN_LEVEL_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//    Instruction_Packet_Array[3] = EEPROM_RETURN_LEVEL;
//    Instruction_Packet_Array[4] = Set;
//    Instruction_Packet_Array[5] = ~(ID + SET_RETURN_LEVEL_LENGTH + COMMAND_WRITE_DATA + EEPROM_RETURN_LEVEL + Set);
//    
//    Status_Return_Value = Set;
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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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


short DynamixelClass::ReadCurrent(unsigned char ID) {
	Instruction_Packet_Array[0] = ID;
	Instruction_Packet_Array[1] = 0x04;
	Instruction_Packet_Array[2] = COMMAND_READ_DATA;
	Instruction_Packet_Array[3] = RAM_PRESENT_CURRENT_1;
	Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
	Instruction_Packet_Array[5] = ~(ID + 0x04 + COMMAND_READ_DATA + RAM_PRESENT_CURRENT_1 + READ_TWO_BYTE_LENGTH);

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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

	if (servoSerial->readable())
		while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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


unsigned int DynamixelClass::reset(unsigned char ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = RESET_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_RESET;
    Instruction_Packet_Array[3] = ~(ID + RESET_LENGTH + COMMAND_RESET); //Checksum;
 
    if (servoSerial->readable()) 
        while (servoSerial->readable()) servoSerial->getc(); //empty buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }   
}


//void DynamixelClass::wheelSync(unsigned char ID1, bool Dir1, unsigned int Speed1, unsigned char ID2, bool Dir2, unsigned int Speed2, unsigned char ID3, bool Dir3, unsigned int Speed3){
//    
//    char Speed1_H,Speed1_L;
//    Speed1_L = Speed1; 
//        if (Dir1 == 0){                          // Move Left
//            Speed1_H = Speed1 >> 8;
//        }
//        else if (Dir1 == 1)                     // Move Right
//        {   
//            Speed1_H = (Speed1 >> 8)+4;
//        }   
//
//    char Speed2_H,Speed2_L;
//    Speed2_L = Speed2; 
//        if (Dir2 == 0){                          // Move Left
//            Speed2_H = Speed2 >> 8;
//        }
//        else if (Dir2 == 1)                     // Move Right
//        {   
//            Speed2_H = (Speed2 >> 8)+4;
//        }
//  
//    char Speed3_H,Speed3_L;
//    Speed3_L = Speed3; 
//        if (Dir3 == 0){                          // Move Left
//            Speed3_H = Speed3 >> 8;
//        }
//        else if (Dir3 == 1)                     // Move Right
//        {   
//            Speed3_H = (Speed3 >> 8)+4;
//        }       
//        
//    Instruction_Packet_Array[0] = 0xFE;         // When Writing a Sync comman you must address all(0xFE) servos
//    Instruction_Packet_Array[1] = SYNC_LOAD_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_SYNC_WRITE;
//    Instruction_Packet_Array[3] = RAM_GOAL_SPEED_L;
//    Instruction_Packet_Array[4] = SYNC_DATA_LENGTH;
//    Instruction_Packet_Array[5] = ID1;
//    Instruction_Packet_Array[6] = Speed1_L;
//    Instruction_Packet_Array[7] = Speed1_H;
//    Instruction_Packet_Array[8] = ID2;
//    Instruction_Packet_Array[9] = Speed2_L;
//    Instruction_Packet_Array[10] = Speed2_H;
//    Instruction_Packet_Array[11] = ID3;
//    Instruction_Packet_Array[12] = Speed3_L;
//    Instruction_Packet_Array[13] = Speed3_H;    
//    Instruction_Packet_Array[14] = (char)(~(0xFE + SYNC_LOAD_LENGTH + COMMAND_SYNC_WRITE + RAM_GOAL_SPEED_L + SYNC_DATA_LENGTH + ID1 + Speed1_L + Speed1_H + ID2 + Speed2_L + Speed2_H + ID3 + Speed3_L + Speed3_H));             
//    
//    transmitInstructionPacket();
// 
//}
//
//unsigned int DynamixelClass::wheelPreload(unsigned char ID, bool Dir,unsigned int Speed){
//    
//    char Speed_H,Speed_L;
//    Speed_L = Speed; 
//        if (Dir == 0){                          // Move Left
//            Speed_H = Speed >> 8;
//        }
//        else if (Dir == 1)                      // Move Right
//        {   
//            Speed_H = (Speed >> 8)+4;
//        }   
//        
//    Instruction_Packet_Array[0] = ID;
//    Instruction_Packet_Array[1] = WHEEL_LENGTH;
//    Instruction_Packet_Array[2] = COMMAND_REG_WRITE_DATA;
//    Instruction_Packet_Array[3] = RAM_GOAL_SPEED_L;
//    Instruction_Packet_Array[4] = Speed_L;
//    Instruction_Packet_Array[5] = Speed_H;
//    Instruction_Packet_Array[6] = ~(ID + WHEEL_LENGTH + COMMAND_REG_WRITE_DATA + RAM_GOAL_SPEED_L + Speed_L + Speed_H);             
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


unsigned int DynamixelClass::ReadRegister(unsigned char ID,unsigned char Register){
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = 0x04;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = Register;
    Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + 0x04 + COMMAND_READ_DATA + Register + READ_TWO_BYTE_LENGTH);
    //READ_TEMP_LENGTH
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //empty buffer

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


unsigned int DynamixelClass::testfunction(unsigned char ID[10], unsigned int *position) {
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
    for(int n = 1; n <= 10; n++) // There are n devices to read
    {
    	Instruction_Packet_Array[3*n+1] = 0x04; // nth device data length
    	Instruction_Packet_Array[3*n+2] = ID[n-1];
    	Instruction_Packet_Array[3*n+3] = RAM_PRESENT_POSITION_1; // nth data address
    }
    for (int i = 0; i < 34; i++) {checksum += Instruction_Packet_Array[i];}
//    printf("checksum = %x\r\n",checksum);
    checksum = ~checksum;
    printf("checksum = %x\r\n",checksum);
    Instruction_Packet_Array[34] = checksum;
    
    //READ_TEMP_LENGTH
    if (servoSerial->readable()) 
        while (servoSerial->readable()) servoSerial->getc(); //empty buffer
    
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
	    for(int i = 0; i < 10; i++)
    	{
    		id_status[n-1][i] = Status_Packet_Array[i];
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
