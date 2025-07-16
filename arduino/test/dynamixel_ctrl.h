#pragma once

#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenCM904)
#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 22;
#elif defined(ARDUINO_OpenCR)
#define DXL_SERIAL Serial3
const int DXL_DIR_PIN = 84;
#elif defined(ARDUINO_OpenRB)
#define DXL_SERIAL Serial1
const int DXL_DIR_PIN = -1;
#endif

const unsigned long DXL_BUADRATE = 1000000;
const float DXL_PROTOCOL_VERSION = 2.0;

extern Dynamixel2Arduino dxl;

namespace DynamixelCtrl
{
    bool init();
    bool setOperationMode(uint8_t id, uint8_t mode);
    void WaitUntilScanDYNAMIXEL(Print& serial = Serial);
}