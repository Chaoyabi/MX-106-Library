# MX-106-Library
This library is used for STM32 Nucleo-f446re on Mbed.

In little modify version, I modify transmitInstructionPacket function in MX106.cpp.

Which add the transmit-complete flag to identify when should the direction of uart change from TX to RX.
