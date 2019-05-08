/*
 * File:      BLEBroadcaster.ino
 * Function:  BLEブロードキャスト通信のブロードキャスターとして動作します。
 *            設定した間隔で温湿度を計測して、宛先を問わず広く発信（ブロードキャスト）します。
 *            単一方向の送信だけで受信機能はありません。
 *            ボタンで模擬的に緊急事態の発生を通知、押すと異常発生のシグナルを送出して動作不能
 *            になります。この状態はリセットで解除されます。
 *            計測・送信時以外はディープスリープ状態に移行します。
 * Date:      2019/04/10
 * Author:    M.Ono
 * 
 * Hardware   MCU:  ESP32 (DOIT ESP32 DEVKIT V1 Board)
 *            ブレッドボードに上記開発ボードと温湿度センサー、プッシュボタン、LEDを配線
 *            温湿度センサーはDHT11 
 */
#include <BLEDevice.h>
#include "BLEBeacon.h"
#include <BLEServer.h>
#include <esp_deep_sleep.h>
//#include <DHT.h>                    // DHTセンサー用

/* 基本属性定義  */
//#define DEVICE_NAME "ESP32"         // デバイス名
#define DEVICE_NAME "SkOZ"         // デバイス名
#define DEVICE_NUMBER 1             // デバイス識別番号（1～8）
#define SPI_SPEED   115200          // SPI通信速度
#define DHTTYPE     DHT11           // DHTセンサーの型式

//#define BEACON_UUID           "8ec76ea3-6668-48da-9866-75be8bc86f4d" // UUID 1 128-Bit (may use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/)
#define BEACON_UUID             "9bdf717e-591e-69ae-6743-f605d90402dc" // UUID 1 128-Bit (may use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/)

RTC_DATA_ATTR static uint8_t seq_number;    // RTCメモリー上のシーケンス番号
const int sleeping_time = 10;       // ディープスリープ時間（秒）
const int advertising_time = 1;     // アドバータイジング時間（秒）

/* DHTセンサー*/
//const int DHTPin = 14;              // DHTセンサーの接続ピン
//DHT   dht(DHTPin, DHTTYPE);         // DHTクラスの生成

/* LEDピン */
const int ledPin = 25;              // LEDの接続ピン

/* プッシュボタン */
const int buttonPin = 32;           // プッシュボタンの接続ピン

bool bAbnormal;                     // デバイス異常判定

/*****************************************************************************
 *                          Predetermined Sequence                           *
 *****************************************************************************/
void setup() {
    doInitialize();                           // 初期化処理をして
    BLEDevice::init(DEVICE_NAME);             // BLEデバイスを初期化する

    int buttonState = digitalRead(buttonPin); // プッシュボタンが押されたら異常状態にする
    if (buttonState == HIGH) {
        bAbnormal = true;
        Serial.println("** Abnormal condition! **");
    }
    else {
        bAbnormal = false;
    }

    // BLEサーバーを作成してアドバタイズオブジェクトを取得する
    BLEServer *pServer = BLEDevice::createServer();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    // 送信情報を設定してシーケンス番号をインクリメントする
    setAdvertisementData(pAdvertising);
    seq_number++;

    // 所定の時間だけアドバタイズする
    pAdvertising->start();
    Serial.println("Advertising started!");
    delay(advertising_time * 1000);
    pAdvertising->stop();

    if (bAbnormal) {                          // 異常状態なら
        pinMode(ledPin, OUTPUT);
        int ledState = HIGH;
        for (;;) {
            digitalWrite(ledPin, ledState);    // LEDを点滅させて処理を停止する
            ledState = !ledState;
            delay(500);
        }
    }
    // 外部ウェイクアップを設定してディープスリープに移行する
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 1);
    Serial.println("... in deep dleep!");
    esp_deep_sleep(sleeping_time * 1000000LL);
}

void loop() {
}

/*  初期化処理  */
void doInitialize() {
    Serial.begin(SPI_SPEED);
    pinMode(buttonPin, INPUT);        // GPIO設定：プッシュボタン        
    //dht.begin();                      // センサーの起動
}

/*****************************< Other functions >*****************************/
/*
  アドバタイズデータに送信情報を設定する 
*/
void setAdvertisementData(BLEAdvertising* pAdvertising) {
    // 温度と湿度を読み取る
    //float t = dht.readTemperature();
    //float h = dht.readHumidity();
    
    float t = 0.01;
    float h = 0.02;


    uint16_t temp = (uint16_t)(t * 100);
    uint16_t humd = (uint16_t)(h * 100);

    // string領域に送信情報を連結する
    std::string strData = "";
    strData += (char)0xff;                  // Manufacturer specific data
    strData += (char)0xff;                  // manufacturer ID low byte
    strData += (char)0xff;                  // manufacturer ID high byte
    strData += (char)DEVICE_NUMBER;         // サーバー識別番号
    strData += (char)bAbnormal;             // 異常状態判定
    strData += (char)seq_number;            // シーケンス番号
    //strData += (char)(temp & 0xff);         // 温度の下位バイト
    //strData += (char)((temp >> 8) & 0xff);  // 温度の上位バイト
    //strData += (char)(humd & 0xff);         // 湿度の下位バイト
    //strData += (char)((humd >> 8) & 0xff);  // 湿度の上位バイト
    strData = (char)strData.length() + strData; // 先頭にLengthを設定

    //レスポンスデータをセット
    // string領域に送信情報を連結する
    //std::string strDataRsp = "";
    //strDataRsp += (char)0xff;                  // Manufacturer specific data
    //strDataRsp += (char)0xff;                  // manufacturer ID low byte
    //strDataRsp += (char)0xff;                  // manufacturer ID high byte

    //strDataRsp += (char)0x00;                  // major1
    //strDataRsp += (char)0x65;                  // major2
    //strDataRsp += (char)0x00;                  // minor1
    //strDataRsp += (char)0x07;                  // minor1

    //strDataRsp = (char)strDataRsp.length() + strDataRsp; // 先頭にLengthを設定

    // デバイス名とフラグをセットし、送信情報を組み込んでアドバタイズオブジェクトに設定する
    //BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
    //oAdvertisementData.setName(DEVICE_NAME);
    //oAdvertisementData.setFlags(0x06);      // LE General Discoverable Mode | BR_EDR_NOT_SUPPORTED
    //oAdvertisementData.addData(strDataRsp);
    //pAdvertising->setAdvertisementData(oAdvertisementData);


    //iBeaconオブジェクトを設定
    BLEBeacon oBeacon = BLEBeacon();
    oBeacon.setManufacturerId(0x4C00); // fake Apple 0x004C LSB (ENDIAN_CHANGE_U16!)
    oBeacon.setProximityUUID(BLEUUID(BEACON_UUID));
    oBeacon.setMajor(101 & 0xFFFF);
    oBeacon.setMinor(9 & 0xFFFF);
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();

    oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04

    std::string strServiceData = "";

    strServiceData += (char)26;     // Len
    strServiceData += (char)0xFF;   // Type
    strServiceData += oBeacon.getData();
    oAdvertisementData.addData(strServiceData);                                                      
    pAdvertising->setAdvertisementData(oAdvertisementData);


    // デバイス名とフラグをセットし、送信情報を組み込んでアドバタイズオブジェクトに設定する
    BLEAdvertisementData oScanResponseData = BLEAdvertisementData();
    oScanResponseData.setName(DEVICE_NAME);
    oScanResponseData.setFlags(0x06);      // LE General Discoverable Mode | BR_EDR_NOT_SUPPORTED
    oScanResponseData.addData(strData);

    pAdvertising->setScanResponse(true);
    pAdvertising->setScanResponseData(oScanResponseData);
}
