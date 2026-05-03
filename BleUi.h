/* =====================================================================
 *  BleUi.h — BLE GATT (ESP32-S3: chỉ BLE, không Bluetooth Classic)
 *  • READ characteristic INFO: JSON mô tả AP, IP, UUID (demo nhanh)
 *  • WRITE RX: JSON lệnh giống WebSocket { "t":"joy", ... }
 *  • NOTIFY TX: telemetry ~mức web (field rút gọn, vừa MTU BLE)
 *  Đồng bộ UUID với Config.h; Web Bluetooth: tools/ble-dashboard.html
 * =====================================================================*/
#ifndef BLEUI_H
#define BLEUI_H

#include "Config.h"

#if SMB_BLE_ENABLE

#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <WiFi.h>
#include <cstdio>
#include <cstring>

#include "CtrlJson.h"
#include "RobotTelemetry.h"

static BLEServer *g_bleServer = nullptr;
static BLECharacteristic *g_bleTx = nullptr;
static bool g_bleClientConnected = false;

class SmbBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    g_bleClientConnected = true;
    Serial.printf("[BLE] Client ket noi (telemet try %d Hz tren task Web)\n", 10);
  }
  void onDisconnect(BLEServer *) override {
    g_bleClientConnected = false;
    Serial.println(F("[BLE] Ngat ket noi — tiep tuc advertising"));
    BLEDevice::startAdvertising();
  }
};

class SmbBleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    String raw = chr->getValue();
    if (!raw.length()) return;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, raw);
    if (err) return;
    robotApplyControlJson(doc);
  }
};

inline void bleUiInit() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(512);

  g_bleServer = BLEDevice::createServer();
  g_bleServer->setCallbacks(new SmbBleServerCallbacks());

  BLEService *svc = g_bleServer->createService(BLE_SERVICE_UUID);

  {
    char infoBuf[300];
    int n = snprintf(
        infoBuf, sizeof(infoBuf),
        "{\"name\":\"%s\",\"ap\":\"%s\",\"ip\":\"%s\",\"http\":%u,\"ws\":%u,"
        "\"svc\":\"%s\",\"rx\":\"%s\",\"tx\":\"%s\","
        "\"tip\":\"Mo tools/ble-dashboard.html (Chrome localhost) hoac "
        "http://%s/ tren WiFi\"}",
        BLE_DEVICE_NAME, AP_SSID, WiFi.softAPIP().toString().c_str(),
        WEB_PORT, WS_PORT, BLE_SERVICE_UUID, BLE_CHAR_RX_UUID, BLE_CHAR_TX_UUID,
        WiFi.softAPIP().toString().c_str());
    if (n > 0 && n < (int)sizeof(infoBuf)) {
      BLECharacteristic *infoChr = svc->createCharacteristic(
          BLE_CHAR_INFO_UUID, BLECharacteristic::PROPERTY_READ);
      infoChr->setValue((uint8_t *)infoBuf, (size_t)n);
    }
  }

  BLECharacteristic *rx = svc->createCharacteristic(
      BLE_CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE |
                           BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new SmbBleRxCallbacks());

  g_bleTx = svc->createCharacteristic(BLE_CHAR_TX_UUID,
                                        BLECharacteristic::PROPERTY_NOTIFY);
  g_bleTx->addDescriptor(new BLE2902());

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println(F("[BLE] GATT: INFO (READ) + RX (WRITE) + TX (NOTIFY)"));
  Serial.printf("[BLE] Ten quet: \"%s\" | Dashboard BLE: tools/ble-dashboard.html\n",
                BLE_DEVICE_NAME);
}

/** Telemetry BLE — đồng bộ nội dung với web (schema \"v\":1, field tối giản). */
inline void bleUiNotifyIfConnected() {
  if (!g_bleClientConnected || !g_bleTx) return;

  JsonDocument doc;
  robotTelemetryFillJsonBle(doc);

  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf) - 1);
  if (n == 0 || n >= sizeof(buf)) return;
  buf[n] = '\0';
  g_bleTx->setValue((uint8_t *)buf, n);
  g_bleTx->notify();
}

#else

inline void bleUiInit() {}
inline void bleUiNotifyIfConnected() {}

#endif // SMB_BLE_ENABLE

#endif // BLEUI_H
