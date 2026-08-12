/* =====================================================================
 *  LogStream.h — Thread-safe Serial Log Redirector
 *
 *  Mục tiêu: Đưa toàn bộ log từ Serial Monitor (Serial.print, Serial.println)
 *  vào hàng đợi FreeRTOS, sau đó gửi qua WebSocket và MQTT đến Backend.
 *  Điều này giúp kiểm thử robot từ xa mà không cần cắm cáp USB.
 * =====================================================================*/
#ifndef LOG_STREAM_H
#define LOG_STREAM_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct LogMessage {
    char text[128];
};

extern QueueHandle_t g_logQueue;

// With USB CDC On Boot, Arduino maps `Serial` to either HWCDCSerial
// (USB-Serial/JTAG mode) or USBSerial (native USB mode). Keep an explicit alias
// because this project later redefines `Serial` to the queueing logger below.
#if defined(ARDUINO_USB_CDC_ON_BOOT) && (ARDUINO_USB_CDC_ON_BOOT == 1)
#if defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 1)
#define SMB_USB_SERIAL HWCDCSerial
#else
#define SMB_USB_SERIAL USBSerial
#endif
#endif

class LoggerSerial : public Print {
private:
    Stream& _realSerial;
    void (*_beginTransport)(unsigned long);
    char _logBuf[128];
    size_t _logIdx = 0;

    void bufferChar(char c) {
        if (c == '\n' || _logIdx >= sizeof(_logBuf) - 2) {
            _logBuf[_logIdx] = '\0';
            if (_logIdx > 0) {
                // Xoá ký tự \r ở cuối nếu có
                if (_logIdx > 1 && _logBuf[_logIdx - 1] == '\r') {
                    _logBuf[_logIdx - 1] = '\0';
                }
                
                // Đẩy log vào hàng đợi FreeRTOS (không block)
                if (g_logQueue != NULL) {
                    LogMessage msg;
                    strncpy(msg.text, _logBuf, sizeof(msg.text));
                    msg.text[sizeof(msg.text) - 1] = '\0';
                    xQueueSend(g_logQueue, &msg, 0);
                }
            }
            _logIdx = 0;
        } else if (c != '\r') {
            _logBuf[_logIdx++] = c;
        }
    }

public:
    bool muteRealSerial = false;

    LoggerSerial(Stream& real, void (*beginTransport)(unsigned long))
        : _realSerial(real), _beginTransport(beginTransport) {}

    void begin(unsigned long baud) {
        if (_beginTransport != nullptr) _beginTransport(baud);
    }

    size_t write(uint8_t c) override {
        size_t r = 0;
        if (!muteRealSerial) r = _realSerial.write(c);
        bufferChar((char)c);
        return r != 0 ? r : 1;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        size_t r = 0;
        if (!muteRealSerial) r = _realSerial.write(buffer, size);
        for (size_t i = 0; i < size; i++) {
            bufferChar((char)buffer[i]);
        }
        return r != 0 ? r : size;
    }
};

extern LoggerSerial logger;

// Redefine Serial to intercept all prints inside the project files
#define Serial logger

#endif // LOG_STREAM_H
