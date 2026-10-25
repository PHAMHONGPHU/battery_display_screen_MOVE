#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>

BLEClient* pClient = NULL;
BLERemoteCharacteristic* pChar = NULL;
bool connected = false;

#define DAC_PIN 25

BLEAddress addr("D3:7B:AF:4D:57:DB"); // địa chỉ MAC
BLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
BLEUUID charUUID("0000FFE1-0000-1000-8000-00805F9B34FB");

uint8_t loginCmd[]=  {
0xA5,0x0B,0x00,0x58,0x58,
0x19,0x0A,0x1E,
0x0E,0x28,0x0D    };

void notifyCallback(BLERemoteCharacteristic* c,uint8_t* data,size_t len,bool isNotify)
{
   if(len<12)return;

  /* Serial.print("RX:");
   for(int i=0;i<len;i++)
   {
      Serial.printf("%02X-",data[i]);
   }
   Serial.println();
   */
   uint8_t soc=data[11];

   //Serial.print("SOC=");
   //Serial.print(soc);
   //Serial.println("%");

   // map SOC 0-100 -> DAC 130-170
   soc=constrain(soc,0,100);
   //int dacValue=map(soc,0,100,160,182); //911
   int dacValue=map(soc,0,100,59,73); //007    điều chỉnh số min max phù hợp 59,73
   //int dacValue=map(soc,0,100,127,155); // ISB
   dacWrite(DAC_PIN,dacValue);
   //Serial.print("DAC=");
   //Serial.print(dacValue);
   //Serial.print(" V=");
   //Serial.println(dacValue*3.3/255.0,2);
}
bool connectBMS()
{
   Serial.println("Connecting...");
   pClient=BLEDevice::createClient();
   if(!pClient->connect(addr))
   {
      Serial.println("Connect fail");
      return false;
   }

   Serial.println("Connected");
   BLERemoteService* service=
   pClient->getService(serviceUUID);
   if(service==nullptr)
   {
      Serial.println("No FFE0");
      pClient->disconnect();
      return false;
        }
   pChar=service->getCharacteristic(charUUID);
   if(pChar==nullptr)
   {
      Serial.println("No FFE1");
      pClient->disconnect();
      return false;
          }
   pChar->registerForNotify(notifyCallback);
   delay(500);
   Serial.println("Send Login");
   pChar->writeValue(loginCmd,sizeof(loginCmd),true);
   connected=true;
   Serial.println("Ready");
   return true;
}
void setup()
{
   Serial.begin(115200);
   pinMode(DAC_PIN,OUTPUT);
   dacWrite(DAC_PIN,127);
   BLEDevice::init("");
   connectBMS();
}
void loop()
{
   if(pClient && !pClient->isConnected())
   {
      connected=false;
      Serial.println("Disconnected");
      delay(2000);
      connectBMS();
   }
   delay(1000);
}
