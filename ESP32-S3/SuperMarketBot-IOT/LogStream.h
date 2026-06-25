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

class LoggerSerial : public Print {
private:
    HardwareSerial& _realSerial;
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
    LoggerSerial(HardwareSerial& real) : _realSerial(real) {}

    void begin(unsigned long baud) {
        _realSerial.begin(baud);
    }

    size_t write(uint8_t c) override {
        size_t r = _realSerial.write(c);
        bufferChar((char)c);
        return r;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        size_t r = _realSerial.write(buffer, size);
        for (size_t i = 0; i < size; i++) {
            bufferChar((char)buffer[i]);
        }
        return r;
    }
};

extern LoggerSerial logger;

// Redefine Serial to intercept all prints inside the project files
#define Serial logger

#endif // LOG_STREAM_H
