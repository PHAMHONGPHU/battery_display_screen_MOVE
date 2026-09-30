/*
    JK BMS CAN -> Lingbo RS485
    ESP32 Version

    CAN:
        RX = GPIO22
        TX = GPIO21
       RS485
GPIO17 ---- MAX485 DI
GPIO16 <--- MAX485 RO 
GPIO4  ---- MAX485 DE+RE
*/

#include <driver/twai.h>

HardwareSerial RS485(2);

//=============================
// PIN
//=============================

#define CAN_RX      GPIO_NUM_22
#define CAN_TX      GPIO_NUM_21

#define RS485_RX    16
#define RS485_TX    17
#define RS485_EN    4

//=============================

volatile uint8_t SOC = 0;

//=============================

const uint8_t request[8]=
{
    0xC5,0x5C,0x5A,0xAA,
    0x01,0x97,0x96,0x0D
};

uint8_t pos=0;

//=============================

uint8_t CalcCRC(uint8_t *buf)
{
    uint8_t crc=buf[4];

    for(int i=5;i<=14;i++)
        crc^=buf[i];

    return crc;
}

//=============================

void InitCAN()
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX,CAN_RX,TWAI_MODE_NORMAL);

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_250KBITS();

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if(twai_driver_install(&g_config,&t_config,&f_config)!=ESP_OK)
    {
        Serial.println("CAN Driver Fail");

        while(1);
    }

    if(twai_start()!=ESP_OK)
    {
        Serial.println("CAN Start Fail");

        while(1);
    }

    Serial.println("CAN Ready");
}

//=============================

void ReadCAN()
{
    twai_message_t rx;

    if(twai_receive(&rx,0)!=ESP_OK)
        return;

    if(rx.identifier==0x02F4)
    {
        SOC=rx.data[4];

        if(SOC>100)
            SOC=100;
    }
}

//=============================

void SendBMS()
{
    uint8_t tx[17];

    tx[0]=0xB6;
    tx[1]=0x6B;
    tx[2]=0xAA;
    tx[3]=0x5A;

    tx[4]=0x0A;

    tx[5]=0x76;

    tx[6]=SOC;

    tx[7]=0x1E;

    tx[8]=0x00;

    tx[9]=0x00;
    tx[10]=0x00;

    tx[11]=0x00;
    tx[12]=0x00;

    tx[13]=0x00;

    tx[14]=0x00;

    tx[15]=CalcCRC(tx);

    tx[16]=0x0D;

    digitalWrite(RS485_EN,HIGH);

    RS485.write(tx,17);

    RS485.flush();

    digitalWrite(RS485_EN,LOW);

    Serial.print("Send SOC = ");
    Serial.println(SOC);
}

//=============================

void CheckRequest()
{
    while(RS485.available())
    {
        uint8_t b=RS485.read();

        if(b==request[pos])
        {
            pos++;

            if(pos==8)
            {
                Serial.println("VCU Request");

                SendBMS();

                pos=0;
            }
        }
        else
        {
            if(b==request[0])
                pos=1;
            else
                pos=0;
        }
    }
}

//=============================

void setup()
{
    Serial.begin(115200);

    pinMode(RS485_EN,OUTPUT);

    digitalWrite(RS485_EN,LOW);

    RS485.begin(9600,SERIAL_8N1,RS485_RX,RS485_TX);

    InitCAN();

    Serial.println();
    Serial.println("==============================");
    Serial.println(" JK CAN -> LINGBO RS485");
    Serial.println("==============================");
}

//=============================

void loop()
{
    ReadCAN();

    CheckRequest();
}