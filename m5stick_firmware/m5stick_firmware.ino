/*
 * MaCo (Marksmanship Coach) 射撃解析デバイス - M5StickS3 ファームウェア
 * 版：MaCo-FW 2026-10-26（400Hz・±16G）
 * -----------------------------------------------------------
 * 役割：
 *   ・内蔵IMU(BMI270)から角速度(dps)・加速度(G)を400Hzで取得
 *   ・BLE Notifyでバイナリパケットとしてリアルタイム送信（1回の通知に最大4サンプル）
 *   ・撃発検知やスコア計算は行わない（Web側で実施）
 *   ・バッテリー残量を本体画面右上に表示（BLEパケットには含めない）
 *
 * 【2026-10-26 改訂：100Hz → 400Hz、±8G → ±16G】
 *   旧版はセンサーを設定しておらず、BMI270の初期値（加速度100Hz・±8G、ジャイロ200Hz・±2000dps）を
 *   millis()の約10msごとに読んでいた（ジャイロは半分を読み捨て、50〜75Hz成分が折り返す）。
 *   本版はセンサーを 加速度・ジャイロとも 400Hz（normalフィルタ、帯域 約160Hz／約137Hz）、
 *   加速度 ±16G（GBB・小銃の反動の頭打ち対策）、ジャイロ ±2000dps に設定する。
 *   読み取りはセンサーのデータ準備フラグに合わせる（センサーの時計に同期し、重複・取りこぼしを防ぐ）。
 *
 * 【パケット形式】（リトルエンディアン）
 *   1サンプル28バイト（旧版と同じ）を n 個つなげて1回で通知する（n = 1〜4、MTUで決まる）。
 *     uint32_t timestamp_ms;          // 起動からの経過ms（サンプルを読んだ時刻）
 *     float    gyroX, gyroY, gyroZ;   // deg/s
 *     float    accelX, accelY, accelZ; // G（±16Gで換算済み）
 *   旧版のアプリは先頭の28バイトだけを読むため、約100Hzのデータとして従来どおり動く。
 *   新版のアプリは全サンプルを読み、採点・検知には100Hzに間引いて使う（既存のモデル・しきい値を維持）。
 *
 * 【I2Cの扱い】M5UnifiedのI2Cは排他制御がないため、IMU・電源(PMIC)の読み取りはIMUタスクだけで行う。
 *   M5.update()（ボタン）は使わない。画面（SPI）は loop() で描く。
 *
 * 開発環境：Arduino IDE 2.x / esp32 (Espressif) ボードパッケージ
 * 必要ライブラリ：M5Unified（更新すると軸の向きが変わる可能性があるため、更新後は axis_check.html で確認）
 */

#include <M5Unified.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define FW_VERSION "MaCo-FW 2026-10-26"

// ---- BLE UUID定義（独自定義。Nordic UARTのUUID体系を流用） ----
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// ---- BMI270 レジスタ ----
static const uint8_t REG_ACC_CONF  = 0x40;
static const uint8_t REG_ACC_RANGE = 0x41;
static const uint8_t REG_GYR_CONF  = 0x42;
static const uint8_t REG_GYR_RANGE = 0x43;
// ACC_CONF: acc_filter_perf=1(bit7) | acc_bwp=normal(0b010<<4) | acc_odr=400Hz(0x0A)
static const uint8_t ACC_CONF_400  = 0x80 | 0x20 | 0x0A;   // 0xAA
// GYR_CONF: gyr_filter_perf=1(bit7) | gyr_noise_perf=0 | gyr_bwp=normal(0b10<<4) | gyr_odr=400Hz(0x0A)
static const uint8_t GYR_CONF_400  = 0x80 | 0x20 | 0x0A;   // 0xAA
static const uint8_t ACC_RANGE_16G = 0x03;
static const uint8_t GYR_RANGE_2000 = 0x00;
// M5Unifiedは加速度を±8G前提で換算するため、±16G設定では2倍する
static const float ACCEL_SCALE = 2.0f;

static const int MAX_SAMPLES_PER_PACKET = 4;
static const uint32_t BATTERY_UPDATE_INTERVAL_MS = 5000;
static const uint8_t SCREEN_BRIGHTNESS = 30; // 0-255。三脚・銃に付けたままの常時点灯を想定し低め

BLEServer*         pServer = nullptr;
BLECharacteristic* pTxChar = nullptr;
volatile bool deviceConnected = false;

struct __attribute__((packed)) ImuPacket {
  uint32_t timestamp_ms;
  float gyroX, gyroY, gyroZ;    // deg/s
  float accelX, accelY, accelZ; // G
};

// ---- 画面表示用の共有値（IMUタスクが書き、loopが描く） ----
volatile int32_t  batteryLevel = -1;
volatile uint32_t lastRateHz = 0;
volatile uint16_t peerMtu = 23;
volatile int      samplesPerPacket = 1;
volatile bool     needRedraw = true;
bool imuConfOk = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* srv) override { deviceConnected = true; needRedraw = true; }
  void onDisconnect(BLEServer* srv) override {
    deviceConnected = false; needRedraw = true;
    BLEDevice::startAdvertising(); // 切断後、再度アドバタイズを開始
  }
};

// ---- センサー設定（M5.Imu.begin()の後に呼ぶ） ----
bool configureImu() {
  auto* imu = M5.Imu.getImuInstancePtr(0);
  if (!imu) return false;
  imu->writeRegister8(REG_ACC_CONF, ACC_CONF_400);
  imu->writeRegister8(REG_ACC_RANGE, ACC_RANGE_16G);
  imu->writeRegister8(REG_GYR_CONF, GYR_CONF_400);
  imu->writeRegister8(REG_GYR_RANGE, GYR_RANGE_2000);
  delay(5);
  // 読み返して確認（設定が反映されていなければ画面に表示）
  return imu->readRegister8(REG_ACC_CONF) == ACC_CONF_400
      && (imu->readRegister8(REG_ACC_RANGE) & 0x03) == ACC_RANGE_16G
      && (imu->readRegister8(REG_GYR_CONF) & 0xBF) == (GYR_CONF_400 & 0xBF)
      && (imu->readRegister8(REG_GYR_RANGE) & 0x07) == GYR_RANGE_2000;
}

// MTUから1回の通知に入るサンプル数を決める（ATTヘッダ3バイトを除く）
int calcSamplesPerPacket() {
  uint16_t mtu = 23;
  if (pServer) mtu = pServer->getPeerMTU(pServer->getConnId());
  peerMtu = mtu;
  int n = (int)(mtu - 3) / (int)sizeof(ImuPacket);
  if (n < 1) n = 1;
  if (n > MAX_SAMPLES_PER_PACKET) n = MAX_SAMPLES_PER_PACKET;
  return n;
}

// ---- IMUタスク：データ準備フラグを1msごとに確認し、加速度・ジャイロがそろったら1サンプルとする ----
void imuTask(void*) {
  ImuPacket buf[MAX_SAMPLES_PER_PACKET];
  int n = 0;
  uint8_t gotMask = 0;
  uint32_t rateT0 = millis(), rateN = 0, lastBattery = 0, lastSppCheck = 0;
  for (;;) {
    uint32_t mask = M5.Imu.update(); // 新しいデータがあれば読み込む（無ければ0）
    if (mask) {
      gotMask |= (mask & 0x03);       // bit0=加速度、bit1=ジャイロ
      if ((gotMask & 0x03) == 0x03) {
        gotMask = 0;
        const auto& d = M5.Imu.getImuData();
        uint32_t now = millis();
        rateN++;
        if (now - rateT0 >= 1000) {
          uint32_t r = rateN * 1000 / (now - rateT0); rateN = 0; rateT0 = now;
          if (r + 5 < lastRateHz || r > lastRateHz + 5) { lastRateHz = r; needRedraw = true; } // 画面のちらつきを避け、変化したときだけ描き直す
        }
        if (deviceConnected) {
          if (now - lastSppCheck >= 1000) { lastSppCheck = now; int s = calcSamplesPerPacket(); if (s != samplesPerPacket) { samplesPerPacket = s; n = 0; needRedraw = true; } }
          ImuPacket& p = buf[n++];
          p.timestamp_ms = now;
          p.gyroX = d.gyro.x;  p.gyroY = d.gyro.y;  p.gyroZ = d.gyro.z;
          p.accelX = d.accel.x * ACCEL_SCALE; p.accelY = d.accel.y * ACCEL_SCALE; p.accelZ = d.accel.z * ACCEL_SCALE;
          if (n >= samplesPerPacket) {
            pTxChar->setValue((uint8_t*)buf, sizeof(ImuPacket) * n);
            pTxChar->notify();
            n = 0;
          }
        } else {
          n = 0;
        }
        // 電源ICの読み取りもこのタスクで行う（I2Cの競合を避ける）。サンプル直後に行い、次のサンプルまでに終える
        if (now - lastBattery >= BATTERY_UPDATE_INTERVAL_MS) {
          lastBattery = now;
          batteryLevel = M5.Power.getBatteryLevel();
          needRedraw = true;
        }
      }
    }
    vTaskDelay(1); // 1ms（400Hz=2.5ms周期のデータを取りこぼさない間隔）
  }
}

void drawScreen() {
  M5.Display.fillScreen(deviceConnected ? TFT_DARKGREEN : TFT_NAVY);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(6, 6);
  M5.Display.print(deviceConnected ? "BLE Connected" : "Advertising...");
  M5.Display.setTextSize(1);
  M5.Display.setCursor(6, 34);
  M5.Display.printf("IMU %luHz  acc16G gyr2000", (unsigned long)lastRateHz);
  M5.Display.setCursor(6, 48);
  if (!imuConfOk) { M5.Display.setTextColor(TFT_YELLOW); M5.Display.print("IMU CONF NG (default 100Hz/8G)"); M5.Display.setTextColor(TFT_WHITE); }
  else M5.Display.print("IMU CONF OK");
  M5.Display.setCursor(6, 62);
  if (deviceConnected) M5.Display.printf("MTU %u  %d smp/pkt", (unsigned)peerMtu, (int)samplesPerPacket);
  M5.Display.setCursor(6, 76);
  M5.Display.print(FW_VERSION);
  // 右上にバッテリー
  M5.Display.setCursor(170, 6);
  if (batteryLevel >= 0) M5.Display.printf("BAT %d%%", (int)batteryLevel);
  else M5.Display.print("BAT --");
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(SCREEN_BRIGHTNESS);
  M5.Display.setRotation(1);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.print("MaCo  Init IMU...");

  M5.Imu.begin();
  imuConfOk = configureImu();
  batteryLevel = M5.Power.getBatteryLevel();

  // ---- BLE初期化 ----
  BLEDevice::init("MaCo-S3");
  BLEDevice::setMTU(185); // 4サンプル(112バイト)を1回で送れるMTUを要求（iPhone側の上限は185前後）
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);
  pTxChar = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxChar->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  // IMUタスク：loop()と同じコア1で、loop()より高い優先度（画面の描画で読み取りが遅れないようにする）
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 5, nullptr, 1);
  needRedraw = true;
}

void loop() {
  if (needRedraw) { needRedraw = false; drawScreen(); }
  delay(50);
}
