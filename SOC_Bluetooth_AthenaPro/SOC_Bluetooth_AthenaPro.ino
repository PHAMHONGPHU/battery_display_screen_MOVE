#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

// ================= CONFIG =================
#define DEBUG 0

#define DAC_PIN         25
#define POWER_SENSE_PIN 4

#define DAC_MIN 199      // SOC 0%
#define DAC_MAX 236      // SOC 100%
#define DAC_OFF 1        // < 50 => màn hình tắt

#define KEY_DEBOUNCE_MS     80
#define BLE_RECONNECT_MS  5000

BLEAddress bmsAddress("20:CB:A9:44:BC:A8");
BLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
BLEUUID charUUID("0000FFE1-0000-1000-8000-00805F9B34FB");

uint8_t loginCmd[] = {
  0xA5, 0x0B, 0x00, 0x58, 0x58,
  0x19, 0x0A, 0x1E, 0x0E, 0x28, 0x0D
};

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pChar = nullptr;

volatile uint8_t currentSOC = 0;
volatile bool socValid = false;
volatile bool bleConnected = false;

bool keyOn = false;
int rawKey = LOW;
int stableKey = LOW;

uint8_t lastDAC = 255;
unsigned long keyChangeTime = 0;
unsigned long lastReconnect = 0;

#if DEBUG
  #define DBG(x) Serial.print(x)
  #define DBGLN(x) Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

// ================= DAC =================
void setDAC(uint8_t value) {
  if (value == lastDAC) return;
  lastDAC = value;
  dacWrite(DAC_PIN, value);
}

void outputSOC(uint8_t soc) {
  if (soc > 100) soc = 100;
  setDAC((uint8_t)map(soc, 0, 100, DAC_MIN, DAC_MAX));
}

// ================= BLE DATA =================
void notifyCallback(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  if (len < 12) return;

  uint8_t soc = data[11];
  if (soc > 100) return;

  bool changed = (!socValid || soc != currentSOC);
  currentSOC = soc;
  socValid = true;

  // Khóa OFF: vẫn cập nhật SOC nhưng DAC luôn giữ < 50
  if (keyOn && changed) outputSOC(currentSOC);

#if DEBUG
  if (changed) {
    DBG("SOC: ");
    DBG(currentSOC);
    DBGLN("%");
  }
#endif
}

// ================= BLE CALLBACK =================
class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient*) override {
    bleConnected = true;
    DBGLN("BLE CONNECTED");
  }

  void onDisconnect(BLEClient*) override {
    bleConnected = false;
    pChar = nullptr;
    DBGLN("BLE DISCONNECTED");
  }
};

ClientCallbacks clientCallbacks;

// ================= CONNECT BMS =================
bool connectBMS() {
  if (pClient && pClient->isConnected()) {
    bleConnected = true;
    return true;
  }

  if (!pClient) {
    pClient = BLEDevice::createClient();
    if (!pClient) return false;
    pClient->setClientCallbacks(&clientCallbacks);
  }

  if (!pClient->connect(bmsAddress)) {
    bleConnected = false;
    return false;
  }

  BLERemoteService* service = pClient->getService(serviceUUID);
  if (!service) {
    pClient->disconnect();
    return false;
  }

  pChar = service->getCharacteristic(charUUID);
  if (!pChar || !pChar->canNotify() || !pChar->canWrite()) {
    pClient->disconnect();
    pChar = nullptr;
    return false;
  }

  pChar->registerForNotify(notifyCallback);
  delay(300);
  pChar->writeValue(loginCmd, sizeof(loginCmd), true);

  bleConnected = true;
  DBGLN("BMS READY");
  return true;
}

// ================= KEY =================
void updateKey() {
  unsigned long now = millis();
  int state = digitalRead(POWER_SENSE_PIN);

  if (state != rawKey) {
    rawKey = state;
    keyChangeTime = now;
  }

  if (rawKey == stableKey) return;
  if (now - keyChangeTime < KEY_DEBOUNCE_MS) return;

  stableKey = rawKey;
  keyOn = (stableKey == HIGH);

  if (keyOn) {
    // Đánh thức màn hình
    setDAC(DAC_MIN);

    // BLE vẫn chạy từ trước nên có SOC thì đưa lên ngay
    if (socValid) outputSOC(currentSOC);

    DBGLN("KEY ON");
  } else {
    // Quan trọng: DAC < 50 để màn hình tắt
    setDAC(DAC_OFF);

    // KHÔNG disconnect BLE
    // KHÔNG reset currentSOC
    DBGLN("KEY OFF");
  }
}

// ================= BLE MANAGER =================
void updateBLE() {
  if (pClient && pClient->isConnected()) {
    bleConnected = true;
    return;
  }

  bleConnected = false;

  unsigned long now = millis();
  if (now - lastReconnect < BLE_RECONNECT_MS) return;

  lastReconnect = now;
  connectBMS();
}

// ================= SETUP =================
void setup() {
#if DEBUG
  Serial.begin(115200);
#endif

  pinMode(POWER_SENSE_PIN, INPUT_PULLDOWN);

  rawKey = digitalRead(POWER_SENSE_PIN);
  stableKey = rawKey;
  keyOn = (stableKey == HIGH);

  // Khởi động DAC đúng theo trạng thái khóa
  if (keyOn) setDAC(DAC_MIN);
  else setDAC(DAC_OFF);

  BLEDevice::init("");

  // Cho kết nối BMS ngay lần đầu
  lastReconnect = millis() - BLE_RECONNECT_MS;
}

// ================= LOOP =================
void loop() {
  updateKey();
  updateBLE();
  delay(10);
}