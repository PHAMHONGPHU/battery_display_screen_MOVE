#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

#define DEBUG_BLE 1   // Đổi thành 1 khi cần xem log Serial

#if DEBUG_BLE
  #define DBG(...) do { Serial.printf(__VA_ARGS__); } while (0)
#else
  #define DBG(...) do {} while (0)
#endif

constexpr uint8_t DAC_PIN = 25;

constexpr uint8_t DAC_AT_0   = 59;
constexpr uint8_t DAC_AT_100 = 73;

// Khoảng kết nối 200–300 ms: ổn định và tiết kiệm hơn cấu hình mặc định.
constexpr uint16_t BLE_MIN_INTERVAL = 160;
constexpr uint16_t BLE_MAX_INTERVAL = 240;
constexpr uint16_t BLE_LATENCY      = 0;
constexpr uint16_t BLE_TIMEOUT      = 1000; // 10 giây

constexpr uint32_t CONNECT_TIMEOUT_MS = 8000;
constexpr uint32_t RETRY_MIN_MS = 2000;
constexpr uint32_t RETRY_MAX_MS = 30000;

// Đặt bằng 0 nếu BMS chỉ trả lời một lần sau login.
constexpr uint32_t RX_SILENCE_TIMEOUT_MS = 120000;

BLEAddress addr("6C:2A:B9:33:91:EA");   // nhập địa chỉ MAC

BLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
BLEUUID charUUID("0000FFE1-0000-1000-8000-00805F9B34FB");

uint8_t loginCmd[] = {
  0xA5, 0x0B, 0x00, 0x58, 0x58,
  0x19, 0x0A, 0x1E,
  0x0E, 0x28, 0x0D
};

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pChar = nullptr;

bool connected = false;
int lastDacValue = -1;

uint32_t nextConnectMs = 0;
uint32_t retryDelayMs = RETRY_MIN_MS;
uint32_t connectedSinceMs = 0;

portMUX_TYPE rxLock = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t pendingSoc = 0;
volatile bool hasPendingSoc = false;
volatile uint32_t lastValidFrameMs = 0;

bool timeReached(uint32_t now, uint32_t target) {
  return (int32_t)(now - target) >= 0;
}

void resetRxState() {
  portENTER_CRITICAL(&rxLock);
  hasPendingSoc = false;
  lastValidFrameMs = 0;
  portEXIT_CRITICAL(&rxLock);
}

bool validSocPacket(const uint8_t* data, size_t len) {
  // Khi có cấu trúc gói tin BMS, thêm kiểm tra header/checksum tại đây.
  return data != nullptr && len >= 12 && data[11] <= 100;
}

void notifyCallback(BLERemoteCharacteristic* c,
                    uint8_t* data,
                    size_t len,
                    bool isNotify) {
  (void)c;
  (void)isNotify;

  if (!validSocPacket(data, len)) {
    return;
  }

  const uint32_t now = millis();

  // Callback chỉ nhận dữ liệu, tránh Serial và DAC trong tác vụ BLE.
  portENTER_CRITICAL(&rxLock);
  pendingSoc = data[11];
  hasPendingSoc = true;
  lastValidFrameMs = now;
  portEXIT_CRITICAL(&rxLock);
}

bool takePendingSoc(uint8_t& soc) {
  bool available = false;

  portENTER_CRITICAL(&rxLock);
  if (hasPendingSoc) {
    soc = pendingSoc;
    hasPendingSoc = false;
    available = true;
  }
  portEXIT_CRITICAL(&rxLock);

  return available;
}

uint32_t getLastFrameMs() {
  portENTER_CRITICAL(&rxLock);
  const uint32_t result = lastValidFrameMs;
  portEXIT_CRITICAL(&rxLock);

  return result;
}

void applySocToDac(uint8_t soc) {
  const int dacValue = map(soc, 0, 100, DAC_AT_0, DAC_AT_100);

  if (dacValue == lastDacValue) {
    return;
  }

  dacWrite(DAC_PIN, dacValue);
  lastDacValue = dacValue;

  DBG("SOC=%u%% DAC=%d V=%.2f\n",
      soc, dacValue, dacValue * 3.3f / 255.0f);
}

void disconnectBMS() {
  pChar = nullptr;
  connected = false;

  if (pClient != nullptr && pClient->isConnected()) {
    pClient->disconnect();
  }
}

bool connectBMS() {
  resetRxState();
  pChar = nullptr;

  // Chỉ tạo một BLE client, tránh rò rỉ RAM sau nhiều lần reconnect.
  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();

    if (pClient == nullptr) {
      DBG("Cannot create BLE client\n");
      return false;
    }
  }

  DBG("Connecting...\n");

  // 0xFF = tự nhận dạng loại địa chỉ, tương thích ESP32 core 3.3.8.
  if (!pClient->connect(addr, 0xFF, CONNECT_TIMEOUT_MS)) {
    DBG("Connect failed\n");
    return false;
  }

  if (!pClient->updateConnParams(
        BLE_MIN_INTERVAL,
        BLE_MAX_INTERVAL,
        BLE_LATENCY,
        BLE_TIMEOUT)) {
    DBG("Connection-parameter update failed\n");
  }

  BLERemoteService* service = pClient->getService(serviceUUID);
  if (service == nullptr) {
    DBG("Service FFE0 not found\n");
    disconnectBMS();
    return false;
  }

  pChar = service->getCharacteristic(charUUID);
  if (pChar == nullptr) {
    DBG("Characteristic FFE1 not found\n");
    disconnectBMS();
    return false;
  }

  // ESP32 core 3.3.8: hàm này trả về void.
  pChar->registerForNotify(notifyCallback);

  // writeValue() có trả về bool nên vẫn kiểm tra được lỗi.
  if (!pChar->writeValue(loginCmd, sizeof(loginCmd), true)) {
    DBG("Login write failed\n");
    disconnectBMS();
    return false;
  }

  connected = true;
  connectedSinceMs = millis();
  retryDelayMs = RETRY_MIN_MS;

  DBG("BMS ready\n");
  return true;
}

void scheduleRetry(uint32_t now) {
  nextConnectMs = now + retryDelayMs;

  if (retryDelayMs < RETRY_MAX_MS / 2) {
    retryDelayMs *= 2;
  } else {
    retryDelayMs = RETRY_MAX_MS;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(DAC_PIN, OUTPUT);
  dacWrite(DAC_PIN, DAC_AT_0);
  lastDacValue = DAC_AT_0;

  BLEDevice::init("");
}

void loop() {
  const uint32_t now = millis();
  const bool linkUp = (pClient != nullptr && pClient->isConnected());

  if (!linkUp) {
    connected = false;
    pChar = nullptr;

    if (timeReached(now, nextConnectMs)) {
      if (connectBMS()) {
        nextConnectMs = 0;
      } else {
        scheduleRetry(millis());
      }
    }
  } else if (connected && RX_SILENCE_TIMEOUT_MS > 0) {
    const uint32_t lastFrame = getLastFrameMs();
    const uint32_t referenceTime =
      (lastFrame != 0) ? lastFrame : connectedSinceMs;

    if ((uint32_t)(now - referenceTime) >= RX_SILENCE_TIMEOUT_MS) {
      DBG("BMS silent; reconnecting\n");
      disconnectBMS();
      nextConnectMs = millis() + RETRY_MIN_MS;
    }
  }

  uint8_t soc;
  if (takePendingSoc(soc)) {
    applySocToDac(soc);
  }

  delay(50);
}