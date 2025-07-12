#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>
#include <vector>

class SerialCommunication {
private:
    int serial_fd;
    std::string port_name;

    speed_t getBaudRate(int baud_rate)
    {
        switch (baud_rate) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default: return B9600;
        }
    }
    
public:
    SerialCommunication(const std::string& port = "/dev/ttyUSB0") 
        : serial_fd(-1), port_name(port) {}
    
    ~SerialCommunication() {
        close_port();
    }
    
    bool open_port(int baud_rate = 9600)
    {
        close_port();
        std::cout << "シリアルポートを開く: " << port_name << std::endl;
        std::cout << "ボーレート: " << baud_rate << std::endl;
        
        // O_SYNCを削除し、O_NONBLOCKを追加してタイムアウトを防ぐ
        serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        std::cout << "open()結果: " << serial_fd << std::endl;
        
        if (serial_fd < 0) {
            std::cerr << "ポートを開けませんでした: " << port_name 
                      << " エラー: " << strerror(errno) << std::endl;
            return false;
        }
        
        // ノンブロッキングモードを解除
        int flags = fcntl(serial_fd, F_GETFL, 0);
        fcntl(serial_fd, F_SETFL, flags & ~O_NONBLOCK);
        
        // シリアルポートの設定
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(serial_fd, &tty) != 0)
        {
            std::cerr << "tcgetattr エラー: " << strerror(errno) << std::endl;
            close_port();
            return false;
        }
        
        // ボーレート設定
        speed_t speed = getBaudRate(baud_rate);
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        
        // Raw mode設定
        cfmakeraw(&tty);
        
        // 8N1設定 (8データビット、パリティなし、1ストップビット)
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        
        // 入力・出力設定
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;
        
        // タイムアウト設定
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0.01;
        
        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
            std::cerr << "tcsetattr エラー: " << strerror(errno) << std::endl;
            close_port();
            return false;
        }
        
        // バッファをクリア
        tcflush(serial_fd, TCIOFLUSH);
        
        // Arduinoのリセット時間を待つ
        std::cout << "Arduinoのリセット待機中..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        std::cout << "シリアルポート " << port_name << " を開きました (ボーレート: " << baud_rate << ")" << std::endl;
        return true;
    }
    
    void close_port()
    {
        if (serial_fd >= 0) {
            close(serial_fd);
            serial_fd = -1;
            std::cout << "シリアルポートを閉じました" << std::endl;
        }
    }
    
    bool send_data(const std::string& data) {
        if (serial_fd < 0) {
            std::cerr << "ポートが開かれていません" << std::endl;
            return false;
        }
        
        ssize_t bytes_written = write(serial_fd, data.c_str(), data.length());
        if (bytes_written < 0) {
            std::cerr << "データ送信エラー: " << strerror(errno) << std::endl;
            return false;
        }
        
        // std::cout << "送信 (" << bytes_written << " bytes): " << data;
        return true;
    }

    bool send_binary_data(const std::vector<uint8_t>& data) {
        if (serial_fd < 0) {
            std::cerr << "ポートが開かれていません" << std::endl;
            return false;
        }
        
        // ノンブロッキングモードで送信
        ssize_t bytes_written = write(serial_fd, data.data(), data.size());
        if (bytes_written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // バッファがフルの場合は一時的にスキップ
                return false;
            }
            std::cerr << "バイナリデータ送信エラー: " << strerror(errno) << std::endl;
            return false;
        }
        
        // 送信バッファをフラッシュ（オプション）
        // tcdrain(serial_fd);
        
        return bytes_written == static_cast<ssize_t>(data.size());
    }

    bool send_binary_data(const uint8_t* data, size_t size) {
        if (serial_fd < 0) {
            std::cerr << "ポートが開かれていません" << std::endl;
            return false;
        }
        
        ssize_t bytes_written = write(serial_fd, data, size);
        if (bytes_written < 0) {
            std::cerr << "バイナリデータ送信エラー: " << strerror(errno) << std::endl;
            return false;
        }
        return true;
    }
    
    std::string receive_data() {
        if (serial_fd < 0) {
            return "";
        }
        
        char buffer[1024]; // バッファサイズ拡大
        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
        
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            std::string received(buffer);
            return received;
        } else if (bytes_read < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "受信エラー: " << strerror(errno) << std::endl;
            }
        }
        
        return "";
    }

    std::vector<uint8_t> receive_bytes(size_t max_bytes = 1024)
    {
        std::vector<uint8_t> result;
        
        if (serial_fd < 0) {
            return result;
        }
        
        uint8_t buffer[max_bytes];
        ssize_t bytes_read = read(serial_fd, buffer, max_bytes);
        
        if (bytes_read > 0) {
            result.assign(buffer, buffer + bytes_read);
        } else if (bytes_read < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "バイナリ受信エラー: " << strerror(errno) << std::endl;
            }
        }
        
        return result;
    }
};