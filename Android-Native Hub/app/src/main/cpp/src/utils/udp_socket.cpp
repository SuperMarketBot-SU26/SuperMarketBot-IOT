/**
 * udp_socket.cpp — Android NDK UDP wrapper.
 *
 * Dùng BSD socket API (có sẵn trong Android NDK). Set socket non-blocking
 * + select() để có timeout khi recv.
 */

#include "udp_socket.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <cstring>
#include <android/log.h>

#define LOG_TAG "UdpSocket"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

UdpSocket::UdpSocket() = default;

UdpSocket::~UdpSocket() {
    close();
}

bool UdpSocket::open() {
    if (fd_ >= 0) return true;
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        LOGE("socket() failed: %s", std::strerror(errno));
        return false;
    }
    // Non-blocking để recv có timeout.
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    LOGI("UDP socket opened fd=%d", fd_);
    return true;
}

void UdpSocket::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UdpSocket::bind(int localPort) {
    if (fd_ < 0) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(localPort));
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOGE("bind(%d) failed: %s", localPort, std::strerror(errno));
        return false;
    }
    return true;
}

bool UdpSocket::sendTo(const uint8_t* data, size_t len, const std::string& host, int port) {
    if (fd_ < 0) return false;

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &dest.sin_addr) != 1) {
        LOGE("inet_pton failed for host=%s", host.c_str());
        return false;
    }

    ssize_t sent = ::sendto(fd_, data, len, 0,
                            reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
    if (sent < 0) {
        LOGE("sendto failed: %s", std::strerror(errno));
        return false;
    }
    return static_cast<size_t>(sent) == len;
}

int UdpSocket::recv(uint8_t* buf, size_t bufSize, int timeoutMs) {
    if (fd_ < 0) return -1;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);
    timeval tv{};
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;

    int ready = ::select(fd_ + 1, &fds, nullptr, nullptr, &tv);
    if (ready <= 0) return -1;  // timeout hoặc lỗi

    sockaddr_in src{};
    socklen_t srcLen = sizeof(src);
    ssize_t n = ::recvfrom(fd_, buf, bufSize, 0,
                           reinterpret_cast<sockaddr*>(&src), &srcLen);
    return n < 0 ? -1 : static_cast<int>(n);
}
