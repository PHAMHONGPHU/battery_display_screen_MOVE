
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLERemoteCharacteristic.h>
#include <esp_system.h>
#include <esp_random.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include "driver/twai.h"

// ===================== SUA MAC TAI DAY =====================
static const char BMS_MAC[] = "D3:7B:AF:4D:57:DB";
// ==========================================================

static const gpio_num_t CAN_TX_PIN = GPIO_NUM_21;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_22;
static const uint32_t CAN_ID = 0x181150C1;
static const uint32_t CAN_PERIOD_MS = 500;
static const uint32_t SOC_TIMEOUT_MS = 5000;
static const uint32_t FIRST_SOC_TIMEOUT_MS = 10000;
static const uint32_t BLE_SILENCE_MS = 8000;
static const uint32_t BLE_STUCK_MS = 60000;
static const uint32_t RETRY_MIN_MS = 2000;
static const uint32_t RETRY_MAX_MS = 30000;
static const uint32_t SCAN_SECONDS = 3;
static const uint32_t STATUS_PERIOD_MS = 30000;
static const bool LOG_ENABLED = true;

// Giu gia dinh cua code goc. 0 = chua biet do dai goi chinh xac.
// Chi sua EXPECTED_NOTIFY_BYTES khi da biet frame DEL cua model nay.
static const size_t SOC_OFFSET = 11;
static const size_t EXPECTED_NOTIFY_BYTES = 0;

// Chi SOC duoc lay tu BLE. Cac truong sau KHONG phai so do tu BMS.
static const uint16_t FIXED_VOLTAGE_X10 = 635;       // 63.5 V
static const uint16_t FIXED_CURRENT_RAW = 10000;     // 0 A: A*10+10000
static const uint8_t FIXED_SOH = 100;
static const uint16_t FIXED_CAPACITY_X10 = 450;      // 45.0 Ah

static BLEAddress bmsAddress(BMS_MAC);
static BLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
static BLEUUID charUUID("0000FFE1-0000-1000-8000-00805F9B34FB");
static uint8_t loginCmd[] = {
  0xA5, 0x0B, 0x00, 0x58, 0x58, 0x19, 0x0A, 0x1E, 0x0E, 0x28, 0x0D
};

// Chi BLE worker thao tac client/service/characteristic.
// Callback va loop chia se du lieu qua critical section ngan.
static BLEClient* client = nullptr;
static BLEScan* scanner = nullptr;
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

struct GatewayState {
  uint8_t soc;
  bool linked;
  bool accepting;
  bool valid;
  bool sessionHasSoc;
  bool found;
  bool restartRequested;
  esp_ble_addr_type_t addressType;
  uint32_t lastSocMs;
  uint32_t heartbeatMs;
  BLERemoteCharacteristic* source;
};
static GatewayState state = {};   // valid=false: khong gui 0% luc boot

static bool canInstalled = false;
static uint32_t lastCanMs = 0;
static uint32_t canAccepted = 0;
static uint32_t canSuccessEvents = 0;
static uint32_t canFailureEvents = 0;
static uint32_t canRejected = 0;

// Khai bao ro de sketch preprocessor khong nham kieu GatewayState.
static GatewayState readState();
static void heartbeat();
static void invalidateSoc();
static void requestRestart();
static void logMessage(const char* message);
static bool parseSoc(const uint8_t* data, size_t len, uint8_t& result);
static void notifyCallback(BLERemoteCharacteristic* c, uint8_t* data,
                           size_t len, bool isNotify);
static void waitWithHeartbeat(uint32_t durationMs);
static bool disconnectBms();
static bool connectBms();
static void bleWorker(void* parameter);
static bool initWatchdog();
static bool installCan();
static void serviceCan(bool valid, uint8_t soc);
static void restartGateway(const char* reason);

static GatewayState readState() {
  portENTER_CRITICAL(&stateMux);
  GatewayState copy = state;
  portEXIT_CRITICAL(&stateMux);
  return copy;
}

static void heartbeat() {
  const uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);
  state.heartbeatMs = now;
  portEXIT_CRITICAL(&stateMux);
}

static void invalidateSoc() {
  portENTER_CRITICAL(&stateMux);
  state.accepting = false;
  state.valid = false;
  state.source = nullptr;
  portEXIT_CRITICAL(&stateMux);
}

static void requestRestart() {
  invalidateSoc();
  portENTER_CRITICAL(&stateMux);
  state.restartRequested = true;
  portEXIT_CRITICAL(&stateMux);
}

static void logMessage(const char* message) {
  if (LOG_ENABLED) Serial.println(message);
}

class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* c) override {
    portENTER_CRITICAL(&stateMux);
    state.linked = true;
    state.accepting = false;
    state.valid = false;
    state.sessionHasSoc = false;
    state.source = nullptr;
    portEXIT_CRITICAL(&stateMux);
  }

  void onDisconnect(BLEClient* c) override {
    portENTER_CRITICAL(&stateMux);
    state.linked = false;
    state.accepting = false;
    state.valid = false;
    state.source = nullptr;
    portEXIT_CRITICAL(&stateMux);
  }
};
static ClientCallbacks clientCallbacks;

class ScanCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) override {
    if (!device.getAddress().equals(bmsAddress)) return;
    portENTER_CRITICAL(&stateMux);
    state.addressType = static_cast<esp_ble_addr_type_t>(device.getAddressType());
    state.found = true;
    portEXIT_CRITICAL(&stateMux);
    // De scan ket thuc tu nhien; khong stop/clear tu callback.
  }
};
static ScanCallbacks scanCallbacks;

static bool parseSoc(const uint8_t* data, size_t len, uint8_t& result) {
  // TODO: them frame header/type/length/checksum THEO TAI LIEU DEL.
  // Khong tu dat checksum hay ghep fragment khi chua biet protocol.
  if (data == nullptr || len <= SOC_OFFSET) return false;
  if (EXPECTED_NOTIFY_BYTES != 0 && len != EXPECTED_NOTIFY_BYTES) return false;
  if (data[SOC_OFFSET] > 100) return false;
  result = data[SOC_OFFSET];
  return true;
}

static void notifyCallback(BLERemoteCharacteristic* c, uint8_t* data,
                           size_t len, bool isNotify) {
  uint8_t incomingSoc;
  if (!parseSoc(data, len, incomingSoc)) return;
  const uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);
  if (state.linked && state.accepting && state.source == c) {
    state.soc = incomingSoc;
    state.lastSocMs = now;
    state.valid = true;
    state.sessionHasSoc = true;
  }
  portEXIT_CRITICAL(&stateMux);
  // Khong log, delay, login hay reconnect trong callback.
}

static void waitWithHeartbeat(uint32_t durationMs) {
  const uint32_t started = millis();
  while ((uint32_t)(millis() - started) < durationMs) {
    heartbeat();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static bool disconnectBms() {
  invalidateSoc();
  heartbeat();
  if (client->isConnected()) client->disconnect();
  const uint32_t started = millis();
  while (client->isConnected() || readState().linked) {
    if ((uint32_t)(millis() - started) >= 5000) {
      logMessage("BLE: disconnect stuck; requesting restart");
      requestRestart();
      return false;
    }
    heartbeat();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  return true;
}

static bool connectBms() {
  invalidateSoc();
  portENTER_CRITICAL(&stateMux);
  state.found = false;
  state.sessionHasSoc = false;
  portEXIT_CRITICAL(&stateMux);

  logMessage("BLE: scanning for configured MAC");
  heartbeat();
  scanner->start(SCAN_SECONDS, false);  // bo qua return: core 2.x/3.x
  heartbeat();
  const GatewayState scanned = readState();
  if (!scanned.found) return false;

  logMessage("BLE: connecting");
  // Dung 2 tham so cho ca core 2.x/3.x. Loop giam sat API bi treo.
  if (!client->connect(bmsAddress, scanned.addressType)) return false;
  heartbeat();

  // Tai su dung client/cache cho CUNG MOT BMS; khong new/delete moi retry.
  BLERemoteService* service = client->getService(serviceUUID);
  heartbeat();
  if (service == nullptr || !client->isConnected()) return false;

  BLERemoteCharacteristic* c = service->getCharacteristic(charUUID);
  heartbeat();
  if (c == nullptr || !client->isConnected()) return false;
  if (!c->canNotify() || (!c->canWrite() && !c->canWriteNoResponse())) {
    logMessage("BLE: FFE1 must support notify and write");
    return false;
  }

  c->registerForNotify(notifyCallback);
  heartbeat();
  if (!client->isConnected()) return false;
  waitWithHeartbeat(200); // giu khoang doi sau subscribe cua code goc

  portENTER_CRITICAL(&stateMux);
  if (state.linked) {
    state.source = c;
    state.accepting = true;
  }
  portEXIT_CRITICAL(&stateMux);
  if (!readState().accepting) return false;

  // Nhan SOC ngay trong luc write cung khong bi bo mat.
  // Core 2.x tra void: khong the suy ra login thanh cong tu writeValue.
  c->writeValue(loginCmd, sizeof(loginCmd), c->canWrite());
  heartbeat();
  logMessage("BLE: subscribed/login sent; waiting for SOC");
  return client->isConnected() && readState().linked;
}

static void bleWorker(void* parameter) {
  heartbeat();
  BLEDevice::init("");
  heartbeat();
  client = BLEDevice::createClient();
  scanner = BLEDevice::getScan();
  if (client == nullptr || scanner == nullptr) {
    requestRestart();
    for (;;) waitWithHeartbeat(1000);
  }
  client->setClientCallbacks(&clientCallbacks);
  // wantDuplicates=true: khong luu map tat ca thiet bi xung quanh.
  // shouldParse=false: chi can MAC/type, khong can parse ten/service.
  scanner->setAdvertisedDeviceCallbacks(&scanCallbacks, true, false);
  scanner->setActiveScan(false);
  scanner->setInterval(100);
  scanner->setWindow(80);

  uint32_t retryMs = RETRY_MIN_MS;
  for (;;) {
    heartbeat();
    if (readState().restartRequested) {
      waitWithHeartbeat(1000);
      continue;
    }
    bool receivedSoc = false;
    if (connectBms()) {
      const uint32_t readyMs = millis();
      for (;;) {
        heartbeat();
        const GatewayState current = readState();
        if (!current.linked || !client->isConnected()) break;
        const uint32_t now = millis();
        if (current.sessionHasSoc) {
          receivedSoc = true;
          if ((uint32_t)(now - current.lastSocMs) >= BLE_SILENCE_MS) {
            logMessage("BLE: no new SOC; reconnecting");
            break;
          }
        } else if ((uint32_t)(now - readyMs) >= FIRST_SOC_TIMEOUT_MS) {
          logMessage("BLE: first SOC timeout; reconnecting");
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
    if (!disconnectBms()) continue;
    if (receivedSoc) retryMs = RETRY_MIN_MS;
    const uint32_t pauseMs = retryMs + (esp_random() % 501);
    waitWithHeartbeat(pauseMs);
    if (!receivedSoc) {
      retryMs = (retryMs >= RETRY_MAX_MS / 2) ? RETRY_MAX_MS : retryMs * 2;
    }
  }
}

static bool initWatchdog() {
  esp_err_t result;
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t config = {};
  config.timeout_ms = 15000;
  config.idle_core_mask = 0;
  config.trigger_panic = true;
  result = esp_task_wdt_init(&config);
  if (result == ESP_ERR_INVALID_STATE) result = esp_task_wdt_reconfigure(&config);
#else
  result = esp_task_wdt_init(15, true);
#endif
  if (result != ESP_OK) return false;
  if (esp_task_wdt_status(nullptr) == ESP_OK) return true;
  return esp_task_wdt_add(nullptr) == ESP_OK;
}

static bool installCan() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  g.tx_queue_len = 0;  // khong tich hang doi SOC cu
  g.rx_queue_len = 1;  // IDF 4.4 yeu cau > 0; se xoa trong loop
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
                     TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED;
  return twai_driver_install(&g, &t, &f) == ESP_OK;
  // Chua start: CAN chi chay khi co SOC hop le.
}

static void serviceCan(bool valid, uint8_t soc) {
  const uint32_t now = millis();
  static bool triedInstall = false;
  static uint32_t lastInstallMs = 0;
  static uint32_t lastStartMs = 0;
  static uint32_t lastRecoveryMs = 0;
  static bool statusFailing = false;
  static uint32_t statusFailedMs = 0;

  if (!canInstalled) {
    if (triedInstall && (uint32_t)(now - lastInstallMs) < 5000) return;
    triedInstall = true;
    lastInstallMs = now;
    canInstalled = installCan();
    logMessage(canInstalled ? "CAN: installed 250k EXT 0x181150C1" :
                              "CAN: install failed; retry in 5s");
    if (!canInstalled) return;
    lastStartMs = now - 1000;
  }

  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_TX_SUCCESS) ++canSuccessEvents;
    if (alerts & TWAI_ALERT_TX_FAILED) ++canFailureEvents;
  }
  twai_status_info_t info = {};
  if (twai_get_status_info(&info) != ESP_OK) {
    if (!statusFailing) {
      statusFailing = true;
      statusFailedMs = now;
      logMessage("CAN: cannot read driver status");
    } else if ((uint32_t)(now - statusFailedMs) >= 5000) {
      requestRestart(); // loi driver noi bo keo dai; khong phai loi mat ACK
    }
    return;
  }
  statusFailing = false;

  if (info.state == TWAI_STATE_BUS_OFF) {
    if ((uint32_t)(now - lastRecoveryMs) >= 1000) {
      lastRecoveryMs = now;
      if (twai_initiate_recovery() == ESP_OK) logMessage("CAN: bus-off recovery");
    }
    return;
  }
  if (info.state == TWAI_STATE_RECOVERING) return;

  if (!valid) {
    // Stop cung huy khung dang cho trong phan cung; chi bo qua TX chua du.
    if (info.state == TWAI_STATE_RUNNING) twai_stop();
    return;
  }

  if (info.state == TWAI_STATE_STOPPED) {
    if ((uint32_t)(now - lastStartMs) < 1000) return;
    lastStartMs = now;
    if (twai_start() != ESP_OK) return;
    lastCanMs = now - CAN_PERIOD_MS;
  } else if (info.state != TWAI_STATE_RUNNING) {
    return;
  }

  twai_clear_receive_queue(); // chi truyen, khong xu ly frame nhan
  if ((uint32_t)(now - lastCanMs) < CAN_PERIOD_MS) return;
  if (info.msgs_to_tx != 0) {
    // Khung bi ket qua mot chu ky: huy, khong de phat SOC cu ve sau.
    twai_stop();
    ++canRejected;
    return;
  }
  lastCanMs = now; // khong gui don cac chu ky bi lo

  twai_message_t message = {};
  message.identifier = CAN_ID;
  message.extd = 1;
  message.ss = 1;  // single-shot: chu ky sau se lay SOC moi nhat
  message.data_length_code = 8;
  message.data[0] = (uint8_t)FIXED_VOLTAGE_X10;
  message.data[1] = (uint8_t)(FIXED_VOLTAGE_X10 >> 8);
  message.data[2] = (uint8_t)FIXED_CURRENT_RAW;
  message.data[3] = (uint8_t)(FIXED_CURRENT_RAW >> 8);
  message.data[4] = soc;
  message.data[5] = FIXED_SOH;
  message.data[6] = (uint8_t)FIXED_CAPACITY_X10;
  message.data[7] = (uint8_t)(FIXED_CAPACITY_X10 >> 8);
  if (twai_transmit(&message, 0) == ESP_OK) ++canAccepted;
  else ++canRejected;
  // ESP_OK chi la driver chap nhan; TX_SUCCESS moi la su kien truyen thanh cong.
}

static void restartGateway(const char* reason) {
  invalidateSoc();
  if (canInstalled) twai_stop();
  logMessage(reason);
  delay(100);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);
  logMessage("DEL BMS -> CAN gateway; protocol byte11 assumption");
  if (!initWatchdog()) {
    restartGateway("Watchdog init failed");
    return;
  }
  heartbeat();
  if (xTaskCreate(bleWorker, "BMS_BLE", 8192, nullptr, 1, nullptr) != pdPASS) {
    restartGateway("Cannot create BLE worker");
  }
}

void loop() {
  esp_task_wdt_reset();
  uint32_t now;
  uint8_t currentSoc;
  bool valid;
  uint32_t lastBeat;
  bool restart;

  portENTER_CRITICAL(&stateMux);
  // Doc thoi gian SAU khi khoa: timestamp tu task khac khong duoc o tuong lai.
  now = millis();
  // Chot invalid de SOC cu khong song lai khi millis() quay vong.
  if (!state.linked || !state.accepting ||
      (uint32_t)(now - state.lastSocMs) >= SOC_TIMEOUT_MS) state.valid = false;
  valid = state.valid;
  currentSoc = state.soc;
  lastBeat = state.heartbeatMs;
  restart = state.restartRequested;
  portEXIT_CRITICAL(&stateMux);

  if (restart || (uint32_t)(now - lastBeat) >= BLE_STUCK_MS) {
    restartGateway("BLE worker blocked/error; restarting ESP32");
    return;
  }
  serviceCan(valid, currentSoc);

  static uint32_t lastStatusMs = 0;
  if (LOG_ENABLED && (uint32_t)(now - lastStatusMs) >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    Serial.printf("SOC=%u valid=%u CAN accepted=%lu success_evt=%lu "
                  "fail_evt=%lu rejected=%lu heap=%lu min_heap=%lu\n",
      (unsigned)currentSoc, (unsigned)valid,
      (unsigned long)canAccepted, (unsigned long)canSuccessEvents,
      (unsigned long)canFailureEvents, (unsigned long)canRejected,
      (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap());
  }
  delay(10);
}
