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
const int BUFFER_SIZE = 52;  // ヘッダー(2) + データ(48) + フッター(2)
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
	BT_SERIAL.setTimeout(10);
	// DynamixelCtrl::init();
	// DynamixelCtrl::WaitUntilScanDYNAMIXEL();
	// for (int i = 0; i < arm_num; i++)
	// {
    //     arms[i].init();
    // }
    delay(1000);
}

int num = 0;

void loop()
{
    if (BT_SERIAL.available())
    {
        num = BT_SERIAL.available();
        USB_SERIAL.println(num);
        uint8_t byte[0xFFFF];
        int bytesRead = BT_SERIAL.readBytes(byte, sizeof(byte));
        USB_SERIAL.print("Read ");
        USB_SERIAL.print(bytesRead);
        USB_SERIAL.print(" bytes: ");
        for (int i= 0; i < bytesRead; i++)
        {
            USB_SERIAL.print(byte[i], HEX);
        }
        USB_SERIAL.println();

        if (bytesRead == byte[2] + 5 && 
            byte[0] == header[0] &&
            byte[1] == header[1] &&
            byte[bytesRead - 2] == footer[0] &&
            byte[bytesRead - 1] == footer[1])
        {
            if (byte[2] == 24)
            {
                memcpy(&receive_pos_data, &byte[3], 24);
                for (int i = 0; i < arm_num; i++)
                {
                    USB_SERIAL.print("ARM" + String(i) + ": ");
                    USB_SERIAL.print("Pos1: ");
                    USB_SERIAL.print(receive_pos_data.pos[i * 3 + 0]);
                    USB_SERIAL.print(", Pos2: ");
                    USB_SERIAL.print(receive_pos_data.pos[i * 3 + 1]);
                    USB_SERIAL.print(", Pos3: ");
                    USB_SERIAL.println(receive_pos_data.pos[i * 3 + 2]);
                }
            }
        }
    }
    USB_SERIAL.println("=================");
}

// void loop()
// {
//     // 受信処理を優先
//     if (BT_SERIAL.available() > 0)
//     {
//         static uint8_t temp_buffer[256];
//         static int temp_index = 0;
        
//         // 利用可能なバイト数だけ読み取る
//         int available = BT_SERIAL.available();
//         int bytes_to_read = (available < (int)(sizeof(temp_buffer) - temp_index)) ? available : (int)(sizeof(temp_buffer) - temp_index);
        
//         if (bytes_to_read > 0) {
//             int bytesRead = BT_SERIAL.readBytes(&temp_buffer[temp_index], bytes_to_read);
//             temp_index += bytesRead;
            
//             // デバッグ出力
//             USB_SERIAL.print("Read ");
//             USB_SERIAL.print(bytesRead);
//             USB_SERIAL.print(" bytes, total buffer: ");
//             USB_SERIAL.println(temp_index);
//         }
        
//         // バッファオーバーフロー対策
//         if (temp_index > 200) {
//             memmove(temp_buffer, &temp_buffer[100], temp_index - 100);
//             temp_index -= 100;
//             USB_SERIAL.println("Buffer overflow protection activated");
//         }
        
//         // メッセージ処理ループ（複数メッセージを一度に処理）
//         bool messageFound = true;
//         while (messageFound && temp_index >= 5) { // 最小メッセージサイズ（ヘッダー2+長さ1+フッター2）
//             messageFound = false;
            
//             // ヘッダーを検索
//             for (int i = 0; i <= temp_index - 5; i++) {
//                 if (temp_buffer[i] == header[0] && temp_buffer[i + 1] == header[1]) {
                    
//                     uint8_t data_length = temp_buffer[i + 2];
//                     int total_message_length = 2 + 1 + data_length + 2; // ヘッダー + 長さ + データ + フッター
                    
//                     // データ長の妥当性をチェック
//                     if (data_length != 24 && data_length != 48) {
//                         // USB_SERIAL.print("Invalid data length: ");
//                         // USB_SERIAL.println(data_length);
//                         continue;
//                     }
//                     USB_SERIAL.print("Found message with length: ");
                    
//                     // バッファに十分なデータがあるかチェック
//                     if (i + total_message_length <= temp_index) {
                        
//                         // フッターの位置を確認
//                         if (temp_buffer[i + total_message_length - 2] == footer[0] && 
//                             temp_buffer[i + total_message_length - 1] == footer[1]) {
                            
//                             if (data_length == 48) {
//                                 // 52バイトメッセージ処理（ヘッダー2+長さ1+データ48+フッター2）
//                                 memcpy(receive_buffer, &temp_buffer[i], total_message_length);
//                                 memcpy(&angle_data, &receive_buffer[3], sizeof(AngleData)); // データは3バイト目から
                                
//                                 for (int j = 0; j < arm_num; j++) {
//                                     arms[j].setAngle(angle_data.angles[j * 3],
//                                                     angle_data.angles[j * 3 + 1],
//                                                     angle_data.angles[j * 3 + 2]);
//                                 }
                                
//                                 USB_SERIAL.println("Processed 52-byte message");
                                
//                             } else if (data_length == 24) {
//                                 // 28バイトメッセージ処理（ヘッダー2+長さ1+データ24+フッター2）
//                                 memcpy(receive_buffer, &temp_buffer[i], total_message_length);
//                                 memcpy(&receive_pos_data, &receive_buffer[3], sizeof(PosData)); // データは3バイト目から
                                
//                                 for (int j = 0; j < arm_num; j++) {
//                                     arms[j].setAngle(dmap(receive_pos_data.pos[j * 3 + 0], 0, 4095, -180, 180),
//                                                      dmap(receive_pos_data.pos[j * 3 + 1], 0, 4095, -180, 180),
//                                                      dmap(receive_pos_data.pos[j * 3 + 2], 0, 4095, -180, 180));
//                                 }
                                
//                                 USB_SERIAL.println("Processed 28-byte message");
//                             }
                            
//                             // 処理したメッセージを削除
//                             int remaining = temp_index - i - total_message_length;
//                             if (remaining > 0) {
//                                 memmove(temp_buffer, &temp_buffer[i + total_message_length], remaining);
//                             }
//                             temp_index = remaining;
//                             messageFound = true;
//                             break;
//                         }
//                     }
//                 }
//             }
//         }
//     }
    
//     // 送信処理（送信頻度を制限）
//     // static unsigned long lastSendTime = 0;
//     // const unsigned long SEND_INTERVAL = 50; // 50ms間隔（20Hz）
//     // if (millis() - lastSendTime >= SEND_INTERVAL) {
//     //     for (int i = 0; i < arm_num; i++) {
//     //         send_pos_data.pos[i * 3]     = static_cast<uint16_t>(dmap(std::get<0>(arms[i].getAngle()), -180, 180, 0, 4095));
//     //         send_pos_data.pos[i * 3 + 1] = static_cast<uint16_t>(dmap(std::get<1>(arms[i].getAngle()), -180, 180, 0, 4095));
//     //         send_pos_data.pos[i * 3 + 2] = static_cast<uint16_t>(dmap(std::get<2>(arms[i].getAngle()), -180, 180, 0, 4095));
//     //     }
        
//     //     // 新しいフォーマットでの送信バッファ作成
//     //     uint8_t send_buffer[27]; // ヘッダー2 + 長さ1 + データ24 + フッター2
//     //     send_buffer[0] = header[0];  // 0xFF
//     //     send_buffer[1] = header[1];  // 0xFF
//     //     send_buffer[2] = 24;         // データ長
//     //     memcpy(&send_buffer[3], &send_pos_data, sizeof(PosData)); // データ24バイト
//     //     send_buffer[25] = footer[0]; // 0xFE
//     //     send_buffer[26] = footer[1]; // 0xFD
        
//     //     BT_SERIAL.write(send_buffer, 27);
//     //     lastSendTime = millis();
//     // }
//     // printAngle(arms[0].getAngle(), 0, BT_SERIAL);
//     // printAngle(arms[1].getAngle(), 1, BT_SERIAL);
//     // printAngle(arms[2].getAngle(), 2, BT_SERIAL);
//     // printAngle(arms[3].getAngle(), 3, BT_SERIAL);
// }