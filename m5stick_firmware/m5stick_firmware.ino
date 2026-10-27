/*
 * MaCo (Marksmanship Coach) 射撃解析デバイス - M5StickS3 ファームウェア
 * 版：MaCo-FW 2026-10-27（400Hz・±16G・FIFO読み取り）
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
 * 【2026-10-27 改訂：FIFO読み取り】
 *   データ準備フラグ方式では、読み取りの遅れ（BLE送信など）で約3%のサンプルが欠落した（実機VP9 50発、5ms間隔3%）。
 *   本版はセンサー内蔵のFIFO（2KB、約150サンプル分）に溜めて、4msごとにまとめて読む。読み遅れがあっても欠落しない。
 *   FIFOはヘッダ付き形式（1サンプル13バイト：ヘッダ0x8C＋ジャイロ6＋加速度6）。
 *   各サンプルの時刻は、前のサンプルに周期を足して付ける（周期は受信したサンプル数と経過時間から求める。実測約405Hz）。
 *   軸と換算はM5Unifiedと同じにする（StickS3：X=センサー+Y、Y=センサー−X、Z=+Z。加速度16G/32768、ジャイロ2000dps/32768）。
 *   FIFOの設定に失敗した場合は、従来のデータ準備フラグ方式で動く（画面に FIFO NG）。
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

#define FW_VERSION "MaCo-FW 2026-10-27 FIFO"

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
// M5Unifiedは加速度を±8G前提で換算するため、±16G設定では2倍する（FIFOが使えない場合の従来方式で使う）
static const float ACCEL_SCALE = 2.0f;

// ---- BMI270 FIFO ----
static const uint8_t REG_FIFO_LENGTH_0 = 0x24;  // 0x24-0x25（14ビット、バイト数）
static const uint8_t REG_FIFO_DATA     = 0x26;
static const uint8_t REG_FIFO_DOWNS    = 0x45;
static const uint8_t REG_FIFO_CONFIG_0 = 0x48;
static const uint8_t REG_FIFO_CONFIG_1 = 0x49;
static const uint8_t REG_CMD           = 0x7E;
static const uint8_t FIFO_DOWNS_FILT   = 0x88;  // 加速度・ジャイロともフィルタ後のデータ、間引きなし
static const uint8_t FIFO_CONFIG_0_VAL = 0x00;  // 満杯時は古いものから上書き、センサー時刻フレームなし
static const uint8_t FIFO_CONFIG_1_VAL = 0xD0;  // gyr_en | acc_en | header_en
static const uint8_t CMD_FIFO_FLUSH    = 0xB0;
static const uint8_t FH_GYR_ACC = 0x8C, FH_ACC = 0x84, FH_GYR = 0x88, FH_SKIP = 0x40, FH_TIME = 0x44, FH_CFG = 0x48;
static const float FIFO_ACC_RES = 16.0f / 32768.0f;    // G/LSB（±16G）
static const float FIFO_GYR_RES = 2000.0f / 32768.0f;  // dps/LSB（±2000dps）
static const uint32_t FIFO_POLL_MS = 4;
static uint8_t fifoBuf[2048 + 16];

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
volatile uint32_t fifoSkipped = 0;   // FIFOのあふれで失ったサンプル数（通常0）
bool imuConfOk = false;
bool fifoOk = false;

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

// ---- FIFOの設定（configureImu()の後に呼ぶ）。読み返して確認する ----
bool configureFifo() {
  auto* imu = M5.Imu.getImuInstancePtr(0);
  if (!imu) return false;
  imu->writeRegister8(REG_FIFO_CONFIG_1, 0x00);   // いったん止める
  imu->writeRegister8(REG_FIFO_DOWNS, FIFO_DOWNS_FILT);
  imu->writeRegister8(REG_FIFO_CONFIG_0, FIFO_CONFIG_0_VAL);
  imu->writeRegister8(REG_FIFO_CONFIG_1, FIFO_CONFIG_1_VAL);
  delay(2);
  imu->writeRegister8(REG_CMD, CMD_FIFO_FLUSH);
  delay(2);
  return imu->readRegister8(REG_FIFO_DOWNS) == FIFO_DOWNS_FILT
      && (imu->readRegister8(REG_FIFO_CONFIG_0) & 0x03) == FIFO_CONFIG_0_VAL
      && (imu->readRegister8(REG_FIFO_CONFIG_1) & 0xF0) == FIFO_CONFIG_1_VAL;
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

// ---- 送信：1サンプルを溜め、samplesPerPacket個たまったら通知する ----
static ImuPacket txBuf[MAX_SAMPLES_PER_PACKET];
static int txN = 0;
static uint32_t lastSppCheck = 0;
static inline void pushSample(uint32_t t, float gx, float gy, float gz, float ax, float ay, float az) {
  if (!deviceConnected) { txN = 0; return; }
  uint32_t now = millis();
  if (now - lastSppCheck >= 1000) { lastSppCheck = now; int s = calcSamplesPerPacket(); if (s != samplesPerPacket) { samplesPerPacket = s; txN = 0; needRedraw = true; } }
  ImuPacket& p = txBuf[txN++];
  p.timestamp_ms = t;
  p.gyroX = gx; p.gyroY = gy; p.gyroZ = gz;
  p.accelX = ax; p.accelY = ay; p.accelZ = az;
  if (txN >= samplesPerPacket) {
    pTxChar->setValue((uint8_t*)txBuf, sizeof(ImuPacket) * txN);
    pTxChar->notify();
    txN = 0;
  }
}

static inline int16_t le16(const uint8_t* b) { return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8)); }

// ---- IMUタスク（FIFO方式）：4msごとにFIFOをまとめて読み、1サンプルずつ時刻を付けて送る ----
void imuTaskFifo() {
  auto* imu = M5.Imu.getImuInstancePtr(0);
  double tCur = 0, period = 2.5;       // ms。周期は受信数と経過時間から求め直す
  uint32_t tFirst = 0; uint64_t nTot = 0; bool started = false;
  uint32_t rateT0 = millis(), rateN = 0, lastBattery = 0;
  for (;;) {
    uint8_t lb[2];
    uint16_t len = 0;
    if (imu->readRegister(REG_FIFO_LENGTH_0, lb, 2)) len = (uint16_t)lb[0] | ((uint16_t)(lb[1] & 0x3F) << 8);
    if (len > 2048) len = 2048;
    if (len > 0 && imu->readRegister(REG_FIFO_DATA, fifoBuf, len)) {
      uint32_t now = millis();
      if (!started) { started = true; tFirst = now; tCur = now; }
      size_t i = 0;
      while (i < len) {
        uint8_t h = fifoBuf[i] & 0xFC;   // 下位2ビットは割り込みタグ
        if (h == FH_GYR_ACC) {
          if (i + 13 > len) break;
          const uint8_t* g = &fifoBuf[i + 1]; const uint8_t* a = &fifoBuf[i + 7];
          int16_t gx = le16(g), gy = le16(g + 2), gz = le16(g + 4);
          int16_t ax = le16(a), ay = le16(a + 2), az = le16(a + 4);
          tCur += period; nTot++; rateN++;
          // StickS3の軸（M5Unifiedと同じ）：X=+Y、Y=−X、Z=+Z
          pushSample((uint32_t)(tCur + 0.5),
                     gy * FIFO_GYR_RES, -gx * FIFO_GYR_RES, gz * FIFO_GYR_RES,
                     ay * FIFO_ACC_RES, -ax * FIFO_ACC_RES, az * FIFO_ACC_RES);
          i += 13;
        } else if (h == FH_ACC || h == FH_GYR) { i += 7;              // 片方だけのフレーム（通常は出ない）
        } else if (h == FH_SKIP) {                                     // あふれで失ったサンプル数
          if (i + 2 > len) break;
          uint8_t k = fifoBuf[i + 1]; fifoSkipped += k; tCur += period * k; nTot += k; needRedraw = true; i += 2;
        } else if (h == FH_TIME) { i += 4;
        } else if (h == FH_CFG)  { i += 5;
        } else break;                                                   // 0x80（空）または不明
      }
      // 周期の推定：経過時間÷サンプル数（200サンプル以降、2.3〜2.7msの範囲）
      if (nTot >= 200) { double est = (double)(now - tFirst) / (double)nTot; if (est > 2.3 && est < 2.7) period = est; }
      if (now - rateT0 >= 1000) {
        uint32_t r = rateN * 1000 / (now - rateT0); rateN = 0; rateT0 = now;
        if (r + 5 < lastRateHz || r > lastRateHz + 5) { lastRateHz = r; needRedraw = true; }
      }
      // 電源ICの読み取りもこのタスクで行う（I2Cの競合を避ける）。FIFOに溜まるので読み遅れても欠落しない
      if (now - lastBattery >= BATTERY_UPDATE_INTERVAL_MS) { lastBattery = now; batteryLevel = M5.Power.getBatteryLevel(); needRedraw = true; }
    }
    vTaskDelay(pdMS_TO_TICKS(FIFO_POLL_MS));
  }
}

// ---- IMUタスク（従来方式。FIFOが使えない場合）：データ準備フラグを1msごとに確認する ----
void imuTaskPoll() {
  uint8_t gotMask = 0;
  uint32_t rateT0 = millis(), rateN = 0, lastBattery = 0;
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
          if (r + 5 < lastRateHz || r > lastRateHz + 5) { lastRateHz = r; needRedraw = true; }
        }
        pushSample(now, d.gyro.x, d.gyro.y, d.gyro.z,
                   d.accel.x * ACCEL_SCALE, d.accel.y * ACCEL_SCALE, d.accel.z * ACCEL_SCALE);
        if (now - lastBattery >= BATTERY_UPDATE_INTERVAL_MS) { lastBattery = now; batteryLevel = M5.Power.getBatteryLevel(); needRedraw = true; }
      }
    }
    vTaskDelay(1);
  }
}

void imuTask(void*) {
  if (fifoOk) imuTaskFifo(); else imuTaskPoll();
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
  M5.Display.setCursor(6, 90);
  if (!fifoOk) { M5.Display.setTextColor(TFT_YELLOW); M5.Display.print("FIFO NG"); M5.Display.setTextColor(TFT_WHITE); }
  else M5.Display.printf("FIFO OK  drop %lu", (unsigned long)fifoSkipped);
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
  fifoOk = imuConfOk && configureFifo();   // センサー設定が正しいときだけFIFOを使う
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
