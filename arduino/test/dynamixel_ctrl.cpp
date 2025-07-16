#include "dynamixel_ctrl.h"

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

bool DynamixelCtrl::init()
{
    dxl.begin(DXL_BUADRATE);
    return dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

bool DynamixelCtrl::setOperationMode(uint8_t id, uint8_t mode)
{
    dxl.torqueOff(id);
    auto flag = dxl.setOperatingMode(id, mode);
    dxl.torqueOn(id);
    return flag;
}

void DynamixelCtrl::WaitUntilScanDYNAMIXEL(Print& serial)
{
    while(!dxl.scan())
    {
        serial.println("DYNAMIXEL not found. Retrying...");
        delay(10);   
    }
}