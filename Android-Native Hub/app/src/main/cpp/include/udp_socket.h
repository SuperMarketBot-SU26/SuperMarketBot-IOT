/**
 * udp_socket.h
 * UDP socket wrapper cho Android NDK — Phase 5.3 Mecanum bridge.
 *
 * Gửi 9-byte motor command packet xuống ESP32 qua WiFi UDP (port 4210).
 *
 * Packet format (xem MotorCommandPacket::encode trong RobotMotorCommand.h):
 *   [0]    0xAA        header
 *   [1]    0x01        cmd velocity
 *   [2-3]  vx int16 LE (mm/s)
 *   [4-5]  vy int16 LE (mm/s)
 *   [6-7]  omega int16 LE (mrad/s)
 *   [8]    XOR checksum (byte 0..7)
 */

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#include <string>
#include <cstdint>

class UdpSocket {
public:
    UdpSocket();
    ~UdpSocket();

    /**
     * Mở socket UDP. Phải gọi trước send().
     * @return true nếu OK
     */
    bool open();

    /** Đóng socket. */
    void close();

    /**
     * Bind local port để nhận telemetry (optional).
     * @param localPort Port local, 0 = OS chọn
     */
    bool bind(int localPort = 0);

    /**
     * Gửi raw bytes tới host:port.
     */
    bool sendTo(const uint8_t* data, size_t len, const std::string& host, int port);

    /**
     * Nhận 1 packet (non-blocking, timeout ms).
     * @return số byte nhận, -1 nếu lỗi/timeout
     */
    int recv(uint8_t* buf, size_t bufSize, int timeoutMs = 100);

    bool isOpen() const { return fd_ >= 0; }

private:
    int fd_ = -1;
};

#endif // UDP_SOCKET_H
