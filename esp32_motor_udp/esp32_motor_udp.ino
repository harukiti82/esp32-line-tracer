// ============================================================================
//  XIAO ESP32-S3 Sense  ライントレース + UDPリモコン モーター制御
// ----------------------------------------------------------------------------
//  - カメラ(グレースケール)でラインを検出し、P制御で左右モーター出力(%)を算出する
//  - シリアルモニタ(115200)に検出状況を毎ループ出力し「機能しているか」を確認できる
//  - Mac(Processing) からの UDP リモコン駆動も従来どおり使える(モード切替)
//  - モーターは ESC/連続回転サーボ式(1500us 中立, ±SPEED_RANGE で正逆)
//
//  動作モード(シリアルに1文字入力で切替):
//    'i' … IDLE   (既定) モーターは回さず、ライン検出結果だけシリアル出力 = 検証用
//    'l' … LINE   算出した左右%で自律走行。ライン消失で停止
//    'r' … REMOTE 従来のUDPリモコンで駆動
//    's' … 即停止して IDLE へ
//    'c' … CALIB  床と線の輝度を実測してしきい値を自動較正(NVSに保存)
//    'x' … 較正データを消去して適応しきい値に戻す
//  ※どのモードでも検出処理は毎ループ走るので、車輪を回さず検証できる
//
//  対象ボード:   Seeed Studio XIAO ESP32-S3 Sense (ESP32-S3, 8MB PSRAM, OV2640)
//  必要ライブラリ: ESP32Servo (ライブラリマネージャ) / esp_camera (ESP32コア同梱)
//  Arduino IDE:  ボード "XIAO_ESP32S3"、PSRAM "OPI PSRAM" を有効化
// ============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <Preferences.h>     // 較正値を NVS に永続化(ESP32コア同梱、追加インストール不要)
#include "esp_camera.h"

// ---- WiFi 設定 -------------------------------------------------------------
// SoftAP モード: ESP32 自身がアクセスポイントになる(ルーター不要・IP固定)。
// Mac をこの SSID に接続すると ESP32 は 192.168.4.1 になる。
#define USE_SOFTAP   true
const char* AP_SSID  = "LineTracer";      // 接続先のWiFi名
const char* AP_PASS  = "12345678";        // 8文字以上

// STA モード(既存ルーターにぶら下げたい場合 USE_SOFTAP を false に)
const char* STA_SSID = "your-router-ssid";
const char* STA_PASS = "your-router-pass";

const uint16_t UDP_PORT = 4210;           // 受信ポート(Processing 側と一致させる)

// ---- モーター設定 ----------------------------------------------------------
// XIAO ESP32-S3 Sense でカメラ・SDが使わず外部ピンに出ている GPIO を使う。
// GPIO1=D0(A0) / GPIO2=D1(A1) はカメラ非使用でボード端子に出ている。
const int MOTOR_L_PIN = 1;                // 左モーター(ESC)信号ピン (D0)
const int MOTOR_R_PIN = 2;                // 右モーター(ESC)信号ピン (D1)

// 回転方向の反転。正の出力%で前進にならない(タイヤが逆回転する)ときに切り替える。
// 既定は左右ともミラー搭載を想定して反転済み。片輪だけ逆なら、その輪だけ true/false を変える。
const bool INVERT_L = true;               // 左モーターの回転方向を反転
const bool INVERT_R = false;              // 右モーターの回転方向を反転

const int SERVO_MIN_US = 1000;            // ESC最小パルス
const int SERVO_MAX_US = 2000;            // ESC最大パルス
const int SERVO_MID_US = 1500;            // 中立(停止)
const int SPEED_RANGE  = 500;             // 100% のときの中立からの振り幅(us)

// ---- フェイルセーフ(REMOTEモードのみ) -------------------------------------
const unsigned long FAILSAFE_MS = 500;    // この時間UDPが来なければ停止

// ---- カメラピン (Seeed Studio XIAO ESP32-S3 Sense) --------------------------
#define CAM_PWDN   -1
#define CAM_RESET  -1
#define CAM_XCLK   10
#define CAM_SIOD   40
#define CAM_SIOC   39
#define CAM_Y9     48
#define CAM_Y8     11
#define CAM_Y7     12
#define CAM_Y6     14
#define CAM_Y5     16
#define CAM_Y4     18
#define CAM_Y3     17
#define CAM_Y2     15
#define CAM_VSYNC  38
#define CAM_HREF   47
#define CAM_PCLK   13

// ---- ライントレース・チューニング ------------------------------------------
const bool  LINE_IS_DARK = true;   // 明るい床に黒線=true / 暗い床に白線=false
const int   LINE_MARGIN  = 35;     // 帯平均輝度からこれだけ離れた画素を「線」とみなす
const int   MIN_LINE_PIX = 8;      // ROI内でこの数未満なら「ライン消失」
const float ROI_TOP_R    = 0.65f;  // ROI(注目帯)の上端 (画像高さ比, 下に近いほど手前)
const float ROI_BOT_R    = 0.95f;  // ROI の下端 (画像高さ比)

const int   BASE_SPEED   = 30;     // 直進時の基本速度(%)
const float KP           = 0.9f;   // P制御ゲイン(誤差→旋回量)
const int   MAX_TURN     = 45;     // 旋回補正の上限(%)

const unsigned long PRINT_INTERVAL_MS = 150;  // シリアル出力の間引き間隔

// ---- キャリブレーション ----------------------------------------------------
// 'c' で床と線の輝度を実測し、線の明暗としきい値を自動決定して NVS に保存する。
// 未較正なら従来の適応しきい値(mean±LINE_MARGIN)で動く(後方互換)。
const int CALIB_SAMPLE_FRAMES = 15;       // 各ステップで平均するフレーム数
const int CALIB_MIN_SEP       = 25;       // 床と線の輝度差がこれ未満なら較正失敗(保存しない)
const unsigned long CALIB_KEY_TIMEOUT_MS = 15000;  // キー入力待ちのタイムアウト

// ---------------------------------------------------------------------------
enum Mode { MODE_IDLE, MODE_LINE, MODE_REMOTE };

WiFiUDP udp;
Servo   servoL;
Servo   servoR;

Mode g_mode = MODE_IDLE;                  // 既定は検証用IDLE(車輪を回さない)
bool g_camOk = false;                     // カメラ初期化成否

// ---- キャリブレーション結果(NVSに永続化) ---------------------------------
Preferences g_prefs;
bool g_calibrated    = false;             // 較正済みか(falseなら適応しきい値)
bool g_calLineIsDark = LINE_IS_DARK;      // 較正で判定した線の明暗
int  g_threshAbs     = 128;               // 較正で決めた絶対しきい値
int  g_calFloorLv    = 0;                 // 較正時の床の輝度(参考表示用)
int  g_calLineLv     = 0;                 // 較正時の線の輝度(参考表示用)

// 現在のモーター出力(%) -100〜100。これが唯一の「真の出力値」。
int g_leftPercent  = 0;
int g_rightPercent = 0;
unsigned long g_lastCmdMs   = 0;          // 最後に有効なUDP指令を受けた時刻
unsigned long g_lastPrintMs = 0;          // 最後にテレメトリを出した時刻

char rxBuf[64];

// ライン検出の1フレーム結果(シリアルに出してライントレースの動作を確認する)
struct LineResult {
  bool detected;   // ラインを検出できたか
  int  bandMean;   // ROI帯の平均輝度(露出確認用)
  int  centroidX;  // 線の重心X (0..width-1)、未検出は -1
  int  width;      // 画像幅
  int  error;      // centroidX - width/2 (右にずれていれば +)
  int  pixels;     // 線とみなした画素数
  int  outL;       // P制御で算出した左出力(%)  ※駆動するかはモード次第
  int  outR;       // P制御で算出した右出力(%)
};

// ROI帯の輝度統計(平均・最小・最大)。較正で床/線の代表輝度を実測するのに使う。
// ※Arduinoの自動プロトタイプ生成は最初の関数定義の直前に挿入されるため、
//   構造体を返す関数より前(=最初の関数より前)で型を定義しておく必要がある。
struct RoiStat { int mean; int minV; int maxV; };

// ---------------------------------------------------------------------------
// 指定した左右の出力(%)でモーターを駆動する。
// 制御ソース(UDPリモコン / ライントレース)を問わず、最終的にここを通す。
void setMotorPercent(int leftPct, int rightPct) {
  g_leftPercent  = constrain(leftPct,  -100, 100);
  g_rightPercent = constrain(rightPct, -100, 100);

  int lSpeed = map(g_leftPercent,  -100, 100, -SPEED_RANGE, SPEED_RANGE);
  int rSpeed = map(g_rightPercent, -100, 100, -SPEED_RANGE, SPEED_RANGE);

  // 回転方向の反転フラグを適用(タイヤが逆回転するときはフラグで直す)
  if (INVERT_L) lSpeed = -lSpeed;
  if (INVERT_R) rSpeed = -rSpeed;

  servoL.writeMicroseconds(SERVO_MID_US + lSpeed);
  servoR.writeMicroseconds(SERVO_MID_US + rSpeed);
}

void stopMotor() {
  setMotorPercent(0, 0);
}

// ---------------------------------------------------------------------------
// キャリブレーション関連
// ---------------------------------------------------------------------------

// ROI帯の輝度統計を frames フレーム平均で求める(RoiStat は上で定義済み)。
RoiStat sampleRoiStat(int frames) {
  long meanAcc = 0, minAcc = 0, maxAcc = 0;
  int  got = 0;
  for (int f = 0; f < frames; f++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { delay(20); continue; }
    const int W = fb->width, H = fb->height;
    const int rTop = (int)(H * ROI_TOP_R), rBot = (int)(H * ROI_BOT_R);
    long sum = 0; int cnt = 0, mn = 255, mx = 0;
    for (int y = rTop; y < rBot; y++) {
      const uint8_t* row = fb->buf + (long)y * W;
      for (int x = 0; x < W; x++) {
        int v = row[x];
        sum += v; cnt++;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
      }
    }
    esp_camera_fb_return(fb);
    if (cnt) { meanAcc += sum / cnt; minAcc += mn; maxAcc += mx; got++; }
    delay(20);
  }
  RoiStat s;
  if (got) { s.mean = meanAcc / got; s.minV = minAcc / got; s.maxV = maxAcc / got; }
  else     { s.mean = 128;          s.minV = 0;            s.maxV = 255; }
  return s;
}

// シリアルに溜まった入力を捨ててから、1文字入力かタイムアウトまで待つ。
// 戻り値: キーが押されたら true / タイムアウトなら false。
bool waitKey(unsigned long timeoutMs) {
  while (Serial.available()) Serial.read();   // 古い入力を捨てる
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (Serial.available()) { Serial.read(); return true; }
    delay(10);
  }
  return false;
}

void saveCalibration() {
  g_prefs.begin("linecal", false);
  g_prefs.putBool("valid", true);
  g_prefs.putInt ("thresh", g_threshAbs);
  g_prefs.putBool("dark",   g_calLineIsDark);
  g_prefs.putInt ("floor",  g_calFloorLv);
  g_prefs.putInt ("line",   g_calLineLv);
  g_prefs.end();
}

// 起動時に NVS から較正値を読む。無ければ適応しきい値で動く。
void loadCalibration() {
  g_prefs.begin("linecal", true);             // 読み取り専用
  g_calibrated = g_prefs.getBool("valid", false);
  if (g_calibrated) {
    g_threshAbs     = g_prefs.getInt ("thresh", 128);
    g_calLineIsDark = g_prefs.getBool("dark",  LINE_IS_DARK);
    g_calFloorLv    = g_prefs.getInt ("floor", 0);
    g_calLineLv     = g_prefs.getInt ("line",  0);
  }
  g_prefs.end();
  if (g_calibrated) {
    Serial.printf("較正データ読込: %s-line thresh=%d (floor=%d line=%d)\n",
                  g_calLineIsDark ? "DARK" : "BRIGHT", g_threshAbs, g_calFloorLv, g_calLineLv);
  } else {
    Serial.println("較正データなし -> 適応しきい値(mean±LINE_MARGIN)で動作。'c'で較正");
  }
}

void clearCalibration() {
  g_prefs.begin("linecal", false);
  g_prefs.putBool("valid", false);
  g_prefs.end();
  g_calibrated = false;
  Serial.println(">> 較正データ消去 -> 適応しきい値に戻す");
}

// 床と線を実測してしきい値・線の明暗を自動決定する2ステップ較正。
// モーターは停止し、車体を手で持ってカメラを向けながら操作する。
void runCalibration() {
  g_mode = MODE_IDLE;
  stopMotor();
  if (!g_camOk) { Serial.println("カメラNGのため較正できません"); return; }

  Serial.println("=== キャリブレーション ===");
  Serial.println("[1/2] カメラを『床だけ(線なし)』に向けて任意キー (15秒で中止)");
  if (!waitKey(CALIB_KEY_TIMEOUT_MS)) { Serial.println("タイムアウトで中止(較正は変更なし)"); return; }
  RoiStat fl = sampleRoiStat(CALIB_SAMPLE_FRAMES);
  Serial.printf("  床   : mean=%d (min=%d max=%d)\n", fl.mean, fl.minV, fl.maxV);

  Serial.println("[2/2] カメラを『線の上』に向けて任意キー");
  if (!waitKey(CALIB_KEY_TIMEOUT_MS)) { Serial.println("タイムアウトで中止(較正は変更なし)"); return; }
  RoiStat ln = sampleRoiStat(CALIB_SAMPLE_FRAMES);
  Serial.printf("  線上 : mean=%d (min=%d max=%d)\n", ln.mean, ln.minV, ln.maxV);

  // 床の代表輝度を基準に、線が「暗い側」か「明るい側」かを実測で判定する。
  // 線が細くてもROIの min/max が線画素を拾うので、平均より頑健に明暗を見分けられる。
  int floorLv       = fl.mean;
  int darkContrast  = floorLv - ln.minV;      // 暗い線ならここが大きい
  int brightContrast = ln.maxV - floorLv;     // 明るい線ならここが大きい
  bool lineIsDark   = (darkContrast >= brightContrast);
  int  lineLv       = lineIsDark ? ln.minV : ln.maxV;
  int  sep          = abs(floorLv - lineLv);

  if (sep < CALIB_MIN_SEP) {
    Serial.printf("コントラスト不足(差=%d < %d)。照明や対象を見直して再実行(c)\n", sep, CALIB_MIN_SEP);
    return;                                    // 保存しない(既存設定を維持)
  }

  g_calLineIsDark = lineIsDark;
  g_threshAbs     = (floorLv + lineLv) / 2;    // 床と線の中間を絶対しきい値に
  g_calFloorLv    = floorLv;
  g_calLineLv     = lineLv;
  g_calibrated    = true;
  saveCalibration();
  Serial.printf(">> 較正完了: %s-line floor=%d line=%d thresh=%d (NVS保存済)\n",
                lineIsDark ? "DARK" : "BRIGHT", floorLv, lineLv, g_threshAbs);
}

// ---------------------------------------------------------------------------
bool setupCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;     // ESP32Servo には timer0 を渡さない(競合回避)
  config.pin_d0 = CAM_Y2;  config.pin_d1 = CAM_Y3;
  config.pin_d2 = CAM_Y4;  config.pin_d3 = CAM_Y5;
  config.pin_d4 = CAM_Y6;  config.pin_d5 = CAM_Y7;
  config.pin_d6 = CAM_Y8;  config.pin_d7 = CAM_Y9;
  config.pin_xclk  = CAM_XCLK;  config.pin_pclk  = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC; config.pin_href  = CAM_HREF;
  config.pin_sccb_sda = CAM_SIOD; config.pin_sccb_scl = CAM_SIOC;
  config.pin_pwdn  = CAM_PWDN;  config.pin_reset = CAM_RESET;
  // XIAO ESP32-S3 Sense はカメラFPCが長く、20MHzだとDMAが取りこぼして
  // "FB-SIZE mismatch" を出すことがある。10MHzに落として確実に1フレーム取り切る。
  config.xclk_freq_hz = 10000000;
  config.frame_size   = FRAMESIZE_QQVGA;      // 160x120: ライン検出には十分で軽い
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // 明暗だけ見るのでグレースケール

  // フレームバッファの置き場: PSRAM があればそこへ、無ければ内蔵RAMへ。
  // QQVGAグレースケール(約19KB)は内蔵RAMにも収まるので PSRAM 無しでも動く。
  if (psramFound()) {
    Serial.printf("PSRAM 検出: %u bytes\n", (unsigned)ESP.getPsramSize());
    config.fb_count    = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode   = CAMERA_GRAB_LATEST;  // 常に最新フレーム(遅延を貯めない)
  } else {
    Serial.println("PSRAM 未検出 -> 内蔵RAMにフレームバッファを確保(QQVGAなら可)");
    config.fb_count    = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("カメラ初期化失敗: 0x%x\n", err);
    return false;
  }
  // OV2640 は起動直後の数フレームが壊れている(FB-SIZE mismatch の一因)。
  // 露出が安定するまで数枚読み捨てて、最初の lineTrace() に壊れたフレームを渡さない。
  for (int i = 0; i < 5; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(50);
  }

  Serial.println("カメラ初期化OK (QQVGA grayscale)");
  return true;
}

// 1フレーム取り込んでラインを検出し、P制御で左右出力を算出する。
// モードに関係なく毎ループ呼び、結果をシリアルに出して動作確認に使う。
LineResult lineTrace() {
  LineResult r = { false, 0, -1, 0, 0, 0, 0, 0 };
  if (!g_camOk) return r;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return r;

  const int W = fb->width;
  const int H = fb->height;
  const int rTop = (int)(H * ROI_TOP_R);
  const int rBot = (int)(H * ROI_BOT_R);
  r.width = W;

  // 1) ROI帯の平均輝度を求め、適応しきい値を作る(照明変化に強くする)
  long sum = 0;
  int  cnt = 0;
  for (int y = rTop; y < rBot; y++) {
    const uint8_t* row = fb->buf + (long)y * W;
    for (int x = 0; x < W; x++) { sum += row[x]; cnt++; }
  }
  int mean = cnt ? (int)(sum / cnt) : 128;
  r.bandMean = mean;

  // 較正済みなら実測した絶対しきい値を使う。未較正なら従来の適応しきい値。
  bool lineIsDark = g_calibrated ? g_calLineIsDark : LINE_IS_DARK;
  int  thresh     = g_calibrated ? g_threshAbs
                                 : (LINE_IS_DARK ? (mean - LINE_MARGIN)
                                                 : (mean + LINE_MARGIN));

  // 2) しきい値を超える(=線)画素の重心Xを求める
  long sx = 0;
  int  px = 0;
  for (int y = rTop; y < rBot; y++) {
    const uint8_t* row = fb->buf + (long)y * W;
    for (int x = 0; x < W; x++) {
      bool isLine = lineIsDark ? (row[x] < thresh) : (row[x] > thresh);
      if (isLine) { sx += x; px++; }
    }
  }
  esp_camera_fb_return(fb);

  r.pixels = px;
  if (px < MIN_LINE_PIX) {        // ライン消失
    r.detected  = false;
    r.centroidX = -1;
    r.error     = 0;
    r.outL = r.outR = 0;
    return r;
  }

  // 3) 誤差 → P制御で旋回量を算出
  r.detected  = true;
  r.centroidX = (int)(sx / px);
  r.error     = r.centroidX - W / 2;            // 右にずれていれば +

  float norm = (float)r.error / (W / 2.0f);     // -1..1
  int   turn = (int)(KP * norm * 100.0f);
  turn = constrain(turn, -MAX_TURN, MAX_TURN);
  // 線が右(error>0)なら右へ曲がる: 左を上げ・右を下げる
  r.outL = constrain(BASE_SPEED + turn, -100, 100);
  r.outR = constrain(BASE_SPEED - turn, -100, 100);
  return r;
}

// 検出結果をシリアルに出力(間引き)。これを見てライントレースの動作を確認する。
void printTelemetry(const LineResult& r) {
  if (millis() - g_lastPrintMs < PRINT_INTERVAL_MS) return;
  g_lastPrintMs = millis();

  const char* modeStr = (g_mode == MODE_IDLE)   ? "IDLE"
                      : (g_mode == MODE_LINE)   ? "LINE"
                      :                           "REMOTE";
  bool driving = (g_mode == MODE_LINE && r.detected);

  // 較正の有効/無効を表示(較正済みは線の明暗と絶対しきい値も出す)
  char calStr[24];
  if (g_calibrated) snprintf(calStr, sizeof(calStr), "cal=%s@%d", g_calLineIsDark ? "DARK" : "BRT", g_threshAbs);
  else              snprintf(calStr, sizeof(calStr), "cal=off");

  if (!g_camOk) {
    Serial.printf("[LINE] camera=NG  mode=%s\n", modeStr);
  } else if (r.detected) {
    Serial.printf("[LINE] det=Y mean=%3d cx=%3d/%d err=%+4d px=%4d -> L=%+4d%% R=%+4d%% | %s mode=%s%s\n",
                  r.bandMean, r.centroidX, r.width, r.error, r.pixels,
                  r.outL, r.outR, calStr, modeStr, driving ? " (駆動中)" : " (出力のみ)");
  } else {
    Serial.printf("[LINE] det=N mean=%3d px=%4d -- ライン消失 -- | %s mode=%s%s\n",
                  r.bandMean, r.pixels, calStr, modeStr,
                  (g_mode == MODE_LINE) ? " -> 停止" : "");
  }
}

// シリアル1文字でモード切替
void handleSerialCmd() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    switch (c) {
      case 'i': g_mode = MODE_IDLE;   stopMotor(); Serial.println(">> モード: IDLE (検証・車輪停止)"); break;
      case 'l': g_mode = MODE_LINE;                Serial.println(">> モード: LINE (自律走行)");       break;
      case 'r': g_mode = MODE_REMOTE; stopMotor(); g_lastCmdMs = millis();
                Serial.println(">> モード: REMOTE (UDPリモコン)"); break;
      case 's': g_mode = MODE_IDLE;   stopMotor(); Serial.println(">> 停止 (IDLEへ)");                 break;
      case 'c': runCalibration();     break;           // 床/線を実測して較正
      case 'x': clearCalibration();   break;           // 較正データを消去
      default: break;                                  // 改行など無視
    }
  }
}

// ---------------------------------------------------------------------------
void setupWiFi() {
  if (USE_SOFTAP) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("SoftAP起動: SSID=");
    Serial.println(AP_SSID);
    Serial.print("ESP32 IP : ");
    Serial.println(WiFi.softAPIP());   // 通常 192.168.4.1
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.print("WiFi接続中");
    while (WiFi.status() != WL_CONNECTED) {
      delay(300);
      Serial.print(".");
    }
    Serial.println();
    Serial.print("ESP32 IP : ");
    Serial.println(WiFi.localIP());    // このIPを Processing 側に設定
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ESP32Servo は PWM タイマーを確保する。カメラが LEDC_TIMER_0 を使うので、
  // servo には timer 2,3 だけを渡してタイマー競合を避ける。
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoL.setPeriodHertz(50);            // ESC標準の50Hz
  servoR.setPeriodHertz(50);
  servoL.attach(MOTOR_L_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoR.attach(MOTOR_R_PIN, SERVO_MIN_US, SERVO_MAX_US);
  stopMotor();                          // 起動時は必ず停止

  g_camOk = setupCamera();              // カメラ初期化(失敗してもリモコンは使える)
  loadCalibration();                    // NVSから較正値を復元(無ければ適応しきい値)

  setupWiFi();
  udp.begin(UDP_PORT);
  Serial.print("UDP受信開始 ポート ");
  Serial.println(UDP_PORT);

  Serial.println("------------------------------------------------------------");
  Serial.println("操作: i=IDLE(検証) / l=LINE(自律) / r=REMOTE(UDP) / s=停止");
  Serial.println("      c=較正(床→線を撮る) / x=較正消去");
  Serial.println("既定は IDLE。線を見せれば検出ログが出るが車輪は回りません。");
  Serial.println("------------------------------------------------------------");

  g_lastCmdMs = millis();
}

// ---------------------------------------------------------------------------
// UDP受信: "L,R"(例 "50,50") または 単一値 "V"(両輪同値) を受け取る。
// REMOTEモードのときだけモーターに反映する。
void handleUdp() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
  if (len <= 0) return;
  rxBuf[len] = '\0';

  int l = 0, r = 0;
  int parsed;
  if (sscanf(rxBuf, "%d,%d", &l, &r) == 2) {
    parsed = 2;
  } else if (sscanf(rxBuf, "%d", &l) == 1) {
    r = l;                               // 単一値は両輪同値
    parsed = 1;
  } else {
    return;                              // 解釈不能なパケットは無視
  }

  g_lastCmdMs = millis();
  if (g_mode == MODE_REMOTE) {
    setMotorPercent(l, r);
    Serial.printf("RX: \"%s\" -> L=%d%% R=%d%%\n", rxBuf, g_leftPercent, g_rightPercent);
  }
}

void loop() {
  // --- モード切替(シリアル) ---------------------------------------------
  handleSerialCmd();

  // --- 制御ソース1: UDPリモコン (REMOTEモードのみ駆動) -------------------
  handleUdp();

  // --- 制御ソース2: ライントレース(検出は常時、駆動はLINEモードのみ) ----
  LineResult lr = lineTrace();
  if (g_mode == MODE_LINE) {
    if (lr.detected) {
      setMotorPercent(lr.outL, lr.outR);
    } else {
      stopMotor();                       // ライン消失で停止
    }
  } else if (g_mode == MODE_IDLE) {
    stopMotor();                          // 検証専用: 車輪は回さない
  }

  // --- 検出結果をシリアルに出力(動作確認の本体) -------------------------
  printTelemetry(lr);

  // --- フェイルセーフ: REMOTEで指令が途切れたら停止 ---------------------
  if (g_mode == MODE_REMOTE && millis() - g_lastCmdMs > FAILSAFE_MS) {
    if (g_leftPercent != 0 || g_rightPercent != 0) {
      stopMotor();
      Serial.println("FAILSAFE: 指令途絶のため停止");
    }
  }
}
