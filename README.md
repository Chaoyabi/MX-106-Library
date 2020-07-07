# MX-106-Library
This library is used for STM32 Nucleo-f446re on Mbed with protocal 1.0.

In little modify version, I modify transmitInstructionPacket function in MX106.cpp.
Which add the transmit-complete flag to identify when should the direction of uart change from TX to RX.
