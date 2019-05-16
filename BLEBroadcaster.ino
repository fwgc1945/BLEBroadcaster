/*
    File:      BLEBroadcaster.ino
    Function:  BLEブロードキャスト通信のブロードキャスターとして動作します。
               設定した間隔で水位を計測して、宛先を問わず広く発信（ブロードキャスト）します。
                 1.ADV_IND(iBeacon)にてアドバタイズパケットを送信
                 2.SCAN_RSPにて水位データを送信しています。 
                 3.計測・送信時以外はディープスリープ状態に移行します。

    Date:      2019/05/08
    Author:    DENSIN Koori.A
  
    Hardware   MCU: ESP32 Dev Module
               水位センサー: MaxSonar WR MB7386
 
                 MaxSonar Arduino
                   GND GND
                   +5V VCC
                   TX 2
                   RX 3
                   AN Analog 0
                   PW 4
*/

#include <BLEDevice.h>
#include "BLEBeacon.h"
#include <BLEServer.h>
#include <esp_deep_sleep.h>

// 基本属性定義
#define DEVICE_NAME "DENSIN"        // デバイス名
//#define DEVICE_NAME "ESP32"       // デバイス名
//#define DEVICE_NAME "SkOZ"

#define SPI_SPEED   115200          // SPI通信速度
//#define BEACON_UUID           "8ec76ea3-6668-48da-9866-75be8bc86f4d" // UUID 1 128-Bit (may use linux tool uuidgen or random numbers via https://www.uuidgenerator.net/)
#define BEACON_UUID             "9bdf717e-591e-69ae-6743-f605d90402dc" // skeed作成の水位計（試作）のクローンになっています。

// 水位計属性定義
#define normally 100    // 平常時(cm)
#define marginTop 5     // 平常時の上マージン(cm)
#define marginBottom 5  // 平常時の下マージン(cm)
#define coefficient 0.2 // 係数

#define sendCycle 0     // 送信サイクル
#define number 10       // センサー計測回数

// RTCメモリー上の定義
RTC_DATA_ATTR static uint8_t seq_number;
RTC_DATA_ATTR static int counter = 1;
RTC_DATA_ATTR static float savDistance = normally;

const int sleeping_time = 300;      // ディープスリープ時間（秒）
const int advertising_time = 1;     // アドバータイジング時間（秒）

// LEDピン
const int ledPin = 13;              // 計測確認用LEDの接続ピン
const int ledPinSend = 12;          // 送信確認用LEDの接続ピン

// センサー入力ピン
const int sensorPin = 4;

// センサー用電源
const int powerPin = 14;

bool bAbnormal;                     // デバイス異常判定

/*****************************************************************************
 *                          Predetermined Sequence                           *
 *****************************************************************************/
void setup() {

    // 初期化処理
    doInitialize();
    
    //static int counter = 0;
    //static float savDistance = normally;

    Serial.print(F("\nRunning loop: ")); Serial.println(counter);

    // センサー用電源ON
    digitalWrite(powerPin, HIGH);

    delay(3000);

    float temperature = 0;
    float distance;

    // 距離を取得する
    distance = getDistance(temperature);

    // 変動値を求める
    float result = distance / savDistance;

    Serial.print(F("distance: ")); Serial.println(distance);
    Serial.print(F("savDistance: ")); Serial.println(savDistance);
    Serial.print(F("result1: ")); Serial.println(result);

    if (distance > savDistance)
    {
        result = result - 1;
    }
    else
    {
        result = 1 - result;
    }

    Serial.print(F("result2: ")); Serial.println(result);
    Serial.print(F("counter: ")); Serial.println(counter);

    // 水位が前回より上がっている場合、平常時－marginTopまでは平常とする
    // 水位が前回より下がっている場合、平常時＋marginBottomまでは異常とする
    // 前回計測値より係数値分変動している場合、
    // 過去6回計測して送信していない場合に送信
    if ((distance < savDistance  && distance <= normally - marginTop)
        || (distance > savDistance  && distance >= normally + marginBottom)
        || (result > coefficient)
        || (counter > sendCycle)) {

        // BLEデバイスを初期化する
        BLEDevice::init(DEVICE_NAME);             

        // BLEサーバーを作成してアドバタイズオブジェクトを取得する
        BLEServer *pServer = BLEDevice::createServer();
        BLEAdvertising *pAdvertising = pServer->getAdvertising();

        // 送信情報を設定
        setAdvertisementData(pAdvertising, distance);

        // 所定の時間だけアドバタイズする
        pAdvertising->start();
        Serial.println("Advertising started!");
        delay(advertising_time * 1000);
        pAdvertising->stop();

        // LED点減
        digitalWrite(ledPinSend, HIGH);
        delay(500);
        digitalWrite(ledPinSend, LOW);

        //計測回数をクリア
        counter = 0;
    }

    // 計測回数を加算
    counter++;

    // 計測値を退避
    savDistance = distance;

    // センサー用電源OFF
    digitalWrite(powerPin, LOW);

    // 外部ウェイクアップを設定してディープスリープに移行する
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 1);
    Serial.println("... in deep dleep!");
    esp_deep_sleep(sleeping_time * 1000000LL);
}

void loop() {
}

/*
    初期化処理
*/
void doInitialize() {
    Serial.begin(SPI_SPEED);
    pinMode(sensorPin, INPUT);
    pinMode(powerPin, OUTPUT);

    pinMode(ledPin, OUTPUT);
    pinMode(ledPinSend, OUTPUT);

}

/*****************************< Other functions >*****************************/
/*
    アドバタイズデータに送信情報を設定します。
    param: アドバタイズオブジェクト（参照渡し）,距離
*/
void setAdvertisementData(BLEAdvertising* pAdvertising, float distance) {

    // iBeaconオブジェクトを設定
    BLEBeacon oBeacon = BLEBeacon();
    oBeacon.setManufacturerId(0x4C00); // fake Apple 0x004C LSB (ENDIAN_CHANGE_U16!)
    oBeacon.setProximityUUID(BLEUUID(BEACON_UUID));
    oBeacon.setMajor(101 & 0xFFFF); // 電信予約値
    oBeacon.setMinor(9 & 0xFFFF);   // 距離センサー（水位含む）
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();

    oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04

    std::string strServiceData = "";

    strServiceData += (char)26;     // Len
    strServiceData += (char)0xFF;   // Type
    strServiceData += oBeacon.getData();
    oAdvertisementData.addData(strServiceData);                                                      
    pAdvertising->setAdvertisementData(oAdvertisementData);

    // intを2バイトの符号なし整数に変換
    uint16_t data = (uint16_t)(distance);

    // string領域に送信情報を連結する
    std::string strData = "";
    strData += (char)0xff;                  // Manufacturer specific data
    strData += (char)0xff;                  // manufacturer ID low byte
    strData += (char)0xff;                  // manufacturer ID high byte

    strData += (char)0x01;                  // ？？
    strData += (char)((data >> 8) & 0xff);  // 距離の上位バイト
    strData += (char)(data & 0xff);         // 距離の下位バイト

    strData = (char)strData.length() + strData; // 先頭にLengthを設定

    // デバイス名とフラグをセットし、送信情報を組み込んでアドバタイズオブジェクトに設定する
    BLEAdvertisementData oScanResponseData = BLEAdvertisementData();
    oScanResponseData.setName(DEVICE_NAME);
    oScanResponseData.setFlags(0x06);      // LE General Discoverable Mode | BR_EDR_NOT_SUPPORTED
    oScanResponseData.addData(strData);

    // 作成したアドバタイズオブジェクトをResponseDataセットする
    pAdvertising->setScanResponse(true);
    pAdvertising->setScanResponseData(oScanResponseData);
}

/*
    超音波距離センサーより距離を取得し、移動平均フィルタを通した値を返します。
    param: 温度
    return: 計測距離
*/
float getDistance(float temp) {

    int arrayDistance[number];
    int length = number;

    for (size_t i = 0; i < length; i++) {

        // パルスポートから読み込む
        unsigned long range_pw = pulseIn(sensorPin, HIGH);
        int Distance = range_pw / 10;

        Serial.print(Distance);
        Serial.println(" (cm) from Pulse");


        if (Distance > 0) {
            // 値を配列に格納
            arrayDistance[i] = Distance;
        }

        delay(500);
    }

    // 数値を昇順にソート
    int tmp;
    for (size_t i = 0; i < length; ++i) {
        for (size_t j = i + 1; j < length; ++j) {
            if (arrayDistance[i] > arrayDistance[j]) {
                tmp = arrayDistance[i];
                arrayDistance[i] = arrayDistance[j];
                arrayDistance[j] = tmp;
            }
        }
    }

    int ix1 = 0;
    int sumDistance = 0;
    for (size_t i = 1; i < length - 1; i++) {

        // ZERO、最小値、最大値を除くサマリーを作成
        if (arrayDistance[i] != 0) {
            sumDistance += arrayDistance[i];
            ix1 += 1;
        }
    }

    Serial.print(F("return: ")); Serial.println(sumDistance / ix1);
    Serial.print(F("sumDistance: ")); Serial.println(sumDistance);
    Serial.print(F("ix1: ")); Serial.println(ix1);

    // LED点減
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);

    // 平均値を返す
    return sumDistance / ix1;
}
