#include <Arduino.h>
#include "BluetoothSerial.h"

#define DAC_PIN 25   // GPIO25 = DAC1

BluetoothSerial SerialBT;

uint8_t socFake = 0;
int direction = 1;

// =====================================================
// Xuất DAC theo SOC
// SOC 0%   -> DAC 205
// SOC 100% -> DAC 240
// =====================================================
void outputGauge(uint8_t soc)
{
    // Giới hạn SOC
    soc = constrain(soc, 0, 100);

    // Map SOC sang DAC
    uint8_t dacValue = map(soc, 0, 100, 199, 236);
    // Xuất DAC
    dacWrite(DAC_PIN, dacValue);

    // Tính điện áp lý thuyết
    float voltage = (dacValue / 255.0f) * 3.3f;

    // =========================
    // Serial USB
    // =========================
    Serial.print("SOC: ");
    Serial.print(soc);
    Serial.print("%");

    Serial.print(" | DAC: ");
    Serial.print(dacValue);

    Serial.print(" | Voltage: ");
    Serial.print(voltage, 3);
    Serial.println(" V");

    // =========================
    // Bluetooth
    // =========================
    if (SerialBT.hasClient())
    {
        SerialBT.print("SOC: ");
        SerialBT.print(soc);
        SerialBT.print("%");

        SerialBT.print(" | DAC: ");
        SerialBT.print(dacValue);

        SerialBT.print(" | Voltage: ");
        SerialBT.print(voltage, 3);
        SerialBT.println(" V");
    }
}


// =====================================================
// SETUP
// =====================================================
void setup()
{
    Serial.begin(115200);

    // DAC
    pinMode(DAC_PIN, OUTPUT);
    dacWrite(DAC_PIN, 0);

    // Bluetooth Classic
    SerialBT.begin("ESP32_SOC_TEST");

    Serial.println();
    Serial.println("==============================");
    Serial.println(" ESP32 DAC + Bluetooth TEST");
    Serial.println("==============================");

    Serial.println("Bluetooth name: ESP32_SOC_TEST");
    Serial.println("Waiting for phone connection...");
}


// =====================================================
// LOOP
// =====================================================
void loop()
{
    // Xuất SOC
    outputGauge(socFake);

    // =========================
    // Giả lập SOC
    // =========================
    socFake += direction * 2;

    // Đạt 100%
    if (socFake >= 100)
    {
        socFake = 100;
        direction = -1;
    }

    // Đạt 0%
    if (socFake <= 0)
    {
        socFake = 0;
        direction = 1;
    }

    delay(2000);
}