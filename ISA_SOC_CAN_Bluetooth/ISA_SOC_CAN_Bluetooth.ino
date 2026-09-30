#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include "driver/twai.h"

// =====================================================
// PIN
// =====================================================
#define CAN_TX GPIO_NUM_21
#define CAN_RX GPIO_NUM_22

// =====================================================
// CAN
// =====================================================
#define CAN_ID   0x181150C1
#define CAN_TIME 500        // ms
#define SOC_TIMEOUT 5000    // mất BLE >5s => SOC không còn hợp lệ

// =====================================================
// BLE
// =====================================================
BLEClient* client = nullptr;
BLERemoteCharacteristic* characteristic = nullptr;

BLEAddress bmsAddr("6c:2a:b9:33:91:ea");

BLEUUID serviceUUID(
  "0000FFE0-0000-1000-8000-00805F9B34FB"
);

BLEUUID charUUID(
  "0000FFE1-0000-1000-8000-00805F9B34FB"
);

// Login command
uint8_t loginCmd[] = {
  0xA5, 0x0B, 0x00, 0x58, 0x58,
  0x19, 0x0A, 0x1E, 0x0E, 0x28, 0x0D
};

// =====================================================
// BMS DATA
// =====================================================
volatile uint8_t soc = 0;
volatile uint32_t lastBLE = 0;

bool bleConnected = false;

uint32_t lastCAN = 0;
uint32_t lastReconnect = 0;

// =====================================================
// FIXED BMS PARAMETERS
// =====================================================
const uint8_t SOH = 100;
const float VOLTAGE = 63.5;
const float CURRENT = 0.0;
const float CAPACITY = 45.0;


// =====================================================
// CAN INIT
// =====================================================
bool initCAN()
{
  twai_general_config_t g =
    TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX,
      CAN_RX,
      TWAI_MODE_NORMAL
    );

  twai_timing_config_t t =
    TWAI_TIMING_CONFIG_250KBITS();

  twai_filter_config_t f =
    TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK)
    return false;

  if (twai_start() != ESP_OK)
    return false;

  return true;
}


// =====================================================
// BLE NOTIFY
// =====================================================
void notifyCallback(
  BLERemoteCharacteristic* c,
  uint8_t* data,
  size_t len,
  bool notify
)
{
  // Theo protocol BLE hiện tại:
  // data[11] = SOC

  if (len < 12)
    return;

  uint8_t newSOC = data[11];

  // Bảo vệ dữ liệu lỗi
  if (newSOC > 100)
    return;

  soc = newSOC;
  lastBLE = millis();
}


// =====================================================
// CONNECT BLE
// =====================================================
bool connectBMS()
{
  if (client == nullptr)
    client = BLEDevice::createClient();

  if (client->isConnected())
    return true;

  Serial.println("BLE connecting...");

  if (!client->connect(bmsAddr))
  {
    Serial.println("BLE connect failed");
    return false;
  }

  BLERemoteService* service =
    client->getService(serviceUUID);

  if (!service)
  {
    Serial.println("FFE0 not found");
    client->disconnect();
    return false;
  }

  characteristic =
    service->getCharacteristic(charUUID);

  if (!characteristic)
  {
    Serial.println("FFE1 not found");
    client->disconnect();
    return false;
  }

  if (characteristic->canNotify())
  {
    characteristic->registerForNotify(notifyCallback);
  }
  else
  {
    Serial.println("FFE1 no notify");
    client->disconnect();
    return false;
  }

  delay(200);

  // Login BMS
  characteristic->writeValue(
    loginCmd,
    sizeof(loginCmd),
    true
  );

  bleConnected = true;

  Serial.println("BLE BMS connected");

  return true;
}


// =====================================================
// SEND CAN
// =====================================================
void sendCAN()
{
  // Không có SOC hợp lệ
  if (millis() - lastBLE > SOC_TIMEOUT)
    return;

  uint8_t data[8];

  // -----------------------------
  // Voltage
  // 63.5V -> 635
  // -----------------------------
  uint16_t voltage =
    (uint16_t)(VOLTAGE * 10.0f);

  data[0] = voltage & 0xFF;
  data[1] = voltage >> 8;


  // -----------------------------
  // Current
  //
  // Raw = Current*10 + 10000
  // -----------------------------
  int16_t current =
    (int16_t)(CURRENT * 10.0f + 10000.0f);

  uint16_t currentRaw =
    (uint16_t)current;

  data[2] = currentRaw & 0xFF;
  data[3] = currentRaw >> 8;


  // -----------------------------
  // SOC
  // -----------------------------
  data[4] = soc;


  // -----------------------------
  // SOH
  // -----------------------------
  data[5] = SOH;


  // -----------------------------
  // Capacity
  // -----------------------------
  uint16_t capacity =
    (uint16_t)(CAPACITY * 10.0f);

  data[6] = capacity & 0xFF;
  data[7] = capacity >> 8;


  // -----------------------------
  // CAN frame
  // -----------------------------
  twai_message_t msg = {};

  msg.identifier = CAN_ID;
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = 8;

  memcpy(msg.data, data, 8);


  // -----------------------------
  // Send
  // -----------------------------
  esp_err_t result =
    twai_transmit(
      &msg,
      pdMS_TO_TICKS(20)
    );

  // Debug rất nhẹ
  static uint32_t lastPrint = 0;

  if (millis() - lastPrint > 2000)
  {
    lastPrint = millis();

    if (result == ESP_OK)
    {
      Serial.print("CAN SOC = ");
      Serial.print(soc);
      Serial.println("%");
    }
    else
    {
      Serial.println("CAN TX ERROR");
    }
  }
}


// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("BLE BMS -> CAN GATEWAY");

  // -----------------------------
  // CAN
  // -----------------------------
  if (!initCAN())
  {
    Serial.println("CAN INIT ERROR");

    while (1)
      delay(1000);
  }

  Serial.println("CAN OK");
  Serial.println("250kbps / EXT / 0x181150C1");


  // -----------------------------
  // BLE
  // -----------------------------
  BLEDevice::init("");

  connectBMS();
}


// =====================================================
// LOOP
// =====================================================
void loop()
{
  uint32_t now = millis();


  // ===================================================
  // BLE CONNECTION CHECK
  // ===================================================
  if (client != nullptr)
  {
    if (!client->isConnected())
    {
      if (bleConnected)
      {
        bleConnected = false;
        characteristic = nullptr;

        Serial.println("BLE disconnected");
      }

      // Không reconnect liên tục
      if (now - lastReconnect > 3000)
      {
        lastReconnect = now;
        connectBMS();
      }
    }
    else
    {
      bleConnected = true;
    }
  }


  // ===================================================
  // CAN SEND
  // ===================================================
  if (now - lastCAN >= CAN_TIME)
  {
    lastCAN = now;

    sendCAN();
  }


  // Không delay dài
  delay(5);
}