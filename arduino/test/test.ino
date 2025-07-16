#include "dynamixel_ctrl.h"
#include "arm.h"
#include "helper.h"

#define USB_SERIAL Serial
#define BT_SERIAL Serial2
const unsigned long USB_BUADRATE = 115200;
const unsigned long BT_BUADRATE  = 57600;

const int arm_num = 4;
Arm arms[arm_num] =
{
    Arm(1, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, 1.0,  0.0),
        2, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, 1.0,  0.0),
        3, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, 1.0,  0.0)),
    
    Arm(4, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, -1.0,  0.0),
        5, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -1.0,  0.0),
        6, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -1.0,  0.0)),
    
    Arm(7, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, 1.0,  0.0),
        8, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, 1.0,  0.0),
        9, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, 1.0,  0.0)),
    
    Arm(10, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, -1.0,  0.0),
        11, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -1.0,  0.0),
        12, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -1.0,  0.0))
};

struct AngleData {
    float angles[12];  // 4つのARM × 3つの角度
};

struct PosData
{
	uint16_t pos[12];
};

// 受信バッファ
const int BUFFER_SIZE = 24 + 5;  // ヘッダー(2) + データ長 + データ(24) + フッター(2)
uint8_t receive_buffer[0xFF];
int buffer_index = 0;
uint8_t header[2] = {0xFF, 0xFF};
uint8_t footer[2] = {0xFE, 0xFD};

// 受信状態
enum ReceiveState {
    WAITING_FOR_HEADER,
    RECEIVING_DATA,
    DATA_COMPLETE
};

ReceiveState state = WAITING_FOR_HEADER;
AngleData angle_data;
AngleData send_angle_data;
PosData receive_pos_data;
PosData send_pos_data;

void printAngle(const std::tuple<double, double, double>& angle, int num, Print& serial)
{
    serial.print("ARM" + String(num) + ":" + "Angle");
    serial.print(std::get<0>(angle), 6);
    serial.print(", ");
    serial.print(std::get<1>(angle), 6);
    serial.print(", ");
    serial.print(std::get<2>(angle), 6);
	serial.print(",");
    serial.println();
}

void processCommand(const String& command, Arm arms[])
{
    int colonIndex = command.indexOf(':');
    if (colonIndex == -1) return;
    
    String armPart = command.substring(0, colonIndex);
    String commandPart = command.substring(colonIndex + 1);
    
    // "ARM"で始まるかチェック
    if (!armPart.startsWith("ARM")) return;
    
    // アームIDを取得 (ARM0, ARM1, ARM2, ARM3)
    int armId = armPart.substring(3).toInt(); // "ARM"の後の数字を取得
    if (armId < 0 || armId >= arm_num) return;
    
    if (commandPart.startsWith("Angle"))
    {
        // "Angle"の後の値部分を取得 (例: "Angle10,20,30")
        String values = commandPart.substring(5);
        
        int firstComma = values.indexOf(',');
        int secondComma = values.indexOf(',', firstComma + 1);
        
        if (firstComma != -1 && secondComma != -1)
        {
            double angle1 = values.substring(0, firstComma).toFloat();
            double angle2 = values.substring(firstComma + 1, secondComma).toFloat();
            double angle3 = values.substring(secondComma + 1).toFloat();
            arms[armId].setAngle(angle1, angle2, angle3);
        }
    }
    else if (commandPart == "TorqueOn")
    {
        arms[armId].torqueOn();
    }
    else if (commandPart == "TorqueOff")
    {
        arms[armId].torqueOff();
    }
}

void processReceivedData() {
    // ヘッダーを除いて角度データを取得（バイト2-49）
    memcpy(&angle_data, &receive_buffer[2], sizeof(AngleData));
    
	for (int i = 0; i < arm_num; i++)
    {
        // 受信した角度データをアームに設定
        arms[i].setAngle(angle_data.angles[i * 3], angle_data.angles[i * 3 + 1], angle_data.angles[i * 3 + 2]);
    }
}

void setup()
{
	USB_SERIAL.begin(USB_BUADRATE);
	BT_SERIAL.begin(BT_BUADRATE);
	BT_SERIAL.setTimeout(1);
	// DynamixelCtrl::init();
	// DynamixelCtrl::WaitUntilScanDYNAMIXEL();
	// for (int i = 0; i < arm_num; i++)
	// {
    //     arms[i].init();
    // }
    delay(1000);
}

void loop() {
    while (BT_SERIAL.available())
    {
        uint8_t incoming = BT_SERIAL.read();

        switch (state)
        {
            case WAITING_FOR_HEADER:
                if (incoming == 0xFF && buffer_index == 0)
                {
                    receive_buffer[buffer_index++] = incoming;
                } 
                else if (buffer_index == 1 && incoming == 0xFF)
                {
                    receive_buffer[buffer_index++] = incoming;
                    state = RECEIVING_DATA;
                }
                else
                {
                    buffer_index = 0;
                }
                break;

            case RECEIVING_DATA:
                receive_buffer[buffer_index++] = incoming;
                if (buffer_index >= BUFFER_SIZE)
                {
                    if (receive_buffer[BUFFER_SIZE - 2] == footer[0] &&
                        receive_buffer[BUFFER_SIZE - 1] == footer[1])
                        {
                        // 正常パケット
                        memcpy(&receive_pos_data, &receive_buffer[3], sizeof(PosData));
                        for (int i = 0; i < arm_num; i++)
                        {
                            USB_SERIAL.print("ARM" + String(i) + ": ");
                            USB_SERIAL.print("Pos1: ");
                            USB_SERIAL.print(receive_pos_data.pos[i * 3 + 0]);
                            USB_SERIAL.print(", Pos2: ");
                            USB_SERIAL.print(receive_pos_data.pos[i * 3 + 1]);
                            USB_SERIAL.print(", Pos3: ");
                            USB_SERIAL.println(receive_pos_data.pos[i * 3 + 2]);
                            arms[i].setPos(receive_pos_data.pos[i * 3 + 0],
                                           receive_pos_data.pos[i * 3 + 1],
                                           receive_pos_data.pos[i * 3 + 2]);
                        }
                        USB_SERIAL.println("##################\n");
                    }
                    else
                    {
                        USB_SERIAL.println("Invalid packet (footer mismatch) - discarded");
                    }
                    buffer_index = 0;
                    state = WAITING_FOR_HEADER;
                }
                break;

            default:
                buffer_index = 0;
                state = WAITING_FOR_HEADER;
                break;
        }
    }
}
