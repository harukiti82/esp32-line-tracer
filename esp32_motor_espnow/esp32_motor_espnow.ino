// ============================================================================
//  XIAO ESP32-S3 Sense  ライントレース + ESP-NOWリモコン モーター制御
// ----------------------------------------------------------------------------
//  - カメラ(グレースケール)でラインを検出し、P制御で左右モーター出力(%)を算出する
//  - シリアルモニタ(115200)に検出状況を毎ループ出力し「機能しているか」を確認できる
//  - Mac(Processing) → 中継ESP32 → ESP-NOW のリモコン駆動も使える(モード切替)
//  - モーターは ESC/連続回転サーボ式(1500us 中立, ±SPEED_RANGE で正逆)
//
//  動作モード(シリアルに1文字入力で切替):
//    'i' … IDLE   (既定) モーターは回さず、ライン検出結果だけシリアル出力 = 検証用
//    'l' … LINE   算出した左右%で自律走行。基準速度はProcessingからの値で変更可。ライン消失で停止
//    'r' … REMOTE ESP-NOW中継経由のリモコンで駆動
//    's' … 即停止して IDLE へ
//  ※どのモードでも検出処理は毎ループ走るので、車輪を回さず検証できる
//  ※しきい値はローカル適応式(各画素を周囲の局所平均±LINE_MARGINと比較)で、影や
//    外光による明るさムラに強い。照明全体の明暗にも自動追従する。左右モーターの
//    個体差補正・床/線の輝度較正といったキャリブレーション機能は撤去済み(定数固定)。
//  ※起動時オートスタート: バッテリー単体(PCなし)なら数秒のカウントダウン後に
//    自動でLINEに入る。PCで何かキーを送れば IDLE のまま留まる(カメラNG時もIDLE)。
//
//  対象ボード:   Seeed Studio XIAO ESP32-S3 Sense (ESP32-S3, 8MB PSRAM, OV2640)
//  必要ライブラリ: ESP32Servo (ライブラリマネージャ) / esp_camera (ESP32コア同梱)
//  Arduino IDE:  ボード "XIAO_ESP32S3"、PSRAM "OPI PSRAM" を有効化
// ============================================================================

#include <WiFi.h>
#include <esp_now.h>         // ESP-NOW(コネクションレス。SoftAP/UDPの接続確立が不要)
#include <esp_wifi.h>        // esp_wifi_set_channel(送受信のチャンネル一致用)
#include <ESP32Servo.h>
#include "esp_camera.h"

// ---- ESP-NOW 設定 ----------------------------------------------------------
// 中継ESP32(Macにusb接続、esp32_espnow_bridge)からブロードキャストで送られてくる
// 指令文字列を受け取る。ESP-NOWは接続・認証・DHCP・SSIDという仕組みが無い
// コネクションレス通信なので、SoftAPで起きていた「SSIDが出ない/接続できない」が
// 構造的に発生しない。瞬断しても状態を持たないぶん次パケットで自動復帰する。
//
// 受信するデータ形式は従来のUDPと同一: "L,R"(例 "50,50") / 単一値 "V" / モード文字。
// チャンネルは送信側(中継)と一致必須。両方とも下記 ESPNOW_CHANNEL に固定する。
const uint8_t ESPNOW_CHANNEL = 1;         // 中継側 esp32_espnow_bridge と必ず同じ値に

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
// 100%指令時の中立からの振り幅(us)。percentToSwing()は100%でちょうどこの値を出す。
// このモーターは中立+約245us(≈1745us)で回転が頭打ちになり、それ以上パルスを離しても
// 速くならない(SPEED_RANGE=500だと45%で飽和=1745usだったため逆算で確定)。
// そこで250にして「100%=実機の飽和点(≈1750us)」に合わせ、0〜100%を線形フル活用する。
// ※これより大きくしても最高速は上がらず、上側にデッドゾーンが増えるだけ。
//   もっと速くしたい場合はソフトではなく電源電圧アップ/別モーター/別ESCが必要。
// ※この値はLINEモードの%→us変換にも効くので、LINEが速すぎたら BASE_SPEED を下げる。
const int SPEED_RANGE  = 250;

// モーターの不感帯補正(中立付近で回り出さない分の最小パルス幅, us)。
// 静止摩擦+ESC不感帯で、左15%/右20%以下ではモーターが回らなかったため、
// 非ゼロ指令時は1%でもこの幅だけ中立から離して「回り出す最小点」を底上げする。
// 実測: 左15%×SPEED_RANGE(200)=30us / 右20%×200=40us。左右個体差で別値にする。
//   ・低速で回り出さないなら増やす / 1%でいきなり動きすぎるなら減らす
const int DEADBAND_L_US = 30;             // 左モーターの不感帯(us)
const int DEADBAND_R_US = 40;             // 右モーターの不感帯(us)

// ---- フェイルセーフ(REMOTEモードのみ) -------------------------------------
const unsigned long FAILSAFE_MS = 500;    // この時間指令が来なければ停止

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
const int   LINE_MARGIN  = 35;     // 各画素を「周囲の局所平均」と比べ、これだけ離れていれば線とみなす
const float LOCAL_BG_R   = 0.22f;  // 局所背景ウィンドウの半幅(画像幅比)。線幅より十分広く取る
const int   MIN_LINE_PIX = 8;      // ROI内でこの数未満なら「ライン消失」
const float ROI_TOP_R    = 0.65f;  // ROI(注目帯)の上端 (画像高さ比, 下に近いほど手前)
const float ROI_BOT_R    = 0.95f;  // ROI の下端 (画像高さ比)

const int   BASE_SPEED   = 30;     // 直進時の基本速度(%)
const float KP           = 0.9f;   // P制御ゲイン(誤差→旋回量)
const int   MAX_TURN     = 45;     // 旋回補正の上限(%)
// コーナー自動減速: 誤差|norm|(0..1)が大きい急カーブほど基準速度を一時的に落とす。
// 0=減速なし。0.6なら最大舵角時に base を (1-0.6)=40% まで落とす。直線では据え置き。
// 高速(BPM180+)でコーナーを曲がりきれず脱線する問題への対策。下げ過ぎると失速。
const float CORNER_SLOWDOWN = 0.6f;
// PD制御の微分ゲイン。誤差の変化(縮まる速さ)に比例して操舵を弱め、高速時の
// 蛇行(オーバーシュート)を制動する。0=従来のP制御のみ。大きすぎるとカメラ
// ノイズに過敏になり逆に震えるので、まず KP の半分前後から現場調整する。
const float KD = 0.5f;
// 操舵の不感帯(ソフト)。直線でラインが中央付近(|正規化誤差|<この値)のときは直進扱いに
// して操舵を0にし、カメラ重心ノイズへの過剰反応(直線蛇行)を消す。境界を超えたら不感帯分を
// 差し引いた値で滑らかに操舵を立ち上げる(ハードに0で切るとカーブ入口の反応が遅れて
// 曲がりきれず消失停止するため)。直線では減速もしないので直線速度は犠牲にしない。
// 大きすぎると緩いカーブの追従が鈍るので小さめにする。
const float STEER_DEADZONE = 0.05f;
// 重心(誤差)のローパスフィルタ係数(指数移動平均, 0<A<=1)。カメラのフレーム毎の重心
// ノイズを平滑化し、P項・D項の震え(直線蛇行)を根本から抑える。小さいほど強く平滑化
// するが応答が遅れる(カーブで反応遅れ→消失方向)。1.0でフィルタなし(従来動作)。
const float NORM_FILT_A = 0.5f;

const unsigned long PRINT_INTERVAL_MS = 150;  // シリアル出力の間引き間隔

// ---- 起動時オートスタート --------------------------------------------------
// バッテリー単体起動(PCなし)を想定。起動後この秒数の間にシリアル入力が無ければ
// 自動で LINE(自律走行)に入る。PC接続時は何かキーを送れば IDLE のまま留まる。
// カメラ初期化に失敗したときは自動走行せず IDLE で待機する(安全策)。
const int BOOT_AUTOSTART_SEC = 3;

// ---------------------------------------------------------------------------
enum Mode { MODE_IDLE, MODE_LINE, MODE_REMOTE };

// ESP-NOW受信バッファ。受信コールバックはWiFiタスク文脈で走るため、そこでは
// 文字列のコピーとフラグ立てだけ行い、実際の解釈・モーター制御は loop() 側で行う
// (コールバック内で重いモーター処理を呼ばない)。
volatile bool g_espnowRx = false;         // 新着指令フラグ(コールバックで立てる)
char g_espnowBuf[64];                      // コールバックが書き込む受信文字列

Servo   servoL;
Servo   servoR;

Mode g_mode = MODE_IDLE;                  // 既定は検証用IDLE(車輪を回さない)
bool g_camOk = false;                     // カメラ初期化成否

// 現在のモーター出力(%) -100〜100。これが唯一の「真の出力値」。
int g_leftPercent  = 0;
int g_rightPercent = 0;

// LINEモードの基準速度(%)。起動時は定数 BASE_SPEED。LINE中に "L,R" 指令を
// 受けると、その平均を基準速度として上書きする(Processingから可変)。
int g_baseSpeed = BASE_SPEED;
unsigned long g_lastCmdMs   = 0;          // 最後に有効な指令を受けた時刻
unsigned long g_lastPrintMs = 0;          // 最後にテレメトリを出した時刻
float g_prevNorm = 0.0f;                   // 前フレームの正規化誤差(PD制御の微分項用)
float g_normFilt = 0.0f;                    // ローパス後の正規化誤差(フィルタ状態を保持)

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

// ---------------------------------------------------------------------------
// 指定した左右の出力(%)でモーターを駆動する。
// 制御ソース(ESP-NOWリモコン / ライントレース)を問わず、最終的にここを通す。
// %(-100..100)を中立からの振り幅(us, 符号付き)に変換する。
// 0%は完全停止(0)。非ゼロは最小deadUsから最大SPEED_RANGEまで線形に割り当て、
// 中立付近で回り出さない不感帯を1%でも飛び越えるようにする。
int percentToSwing(int pct, int deadUs) {
  if (pct == 0) return 0;
  int mag   = abs(pct);                                              // 1..100
  int swing = deadUs + (int)((long)(SPEED_RANGE - deadUs) * mag / 100);
  return (pct > 0) ? swing : -swing;
}

void setMotorPercent(int leftPct, int rightPct) {
  g_leftPercent  = constrain(leftPct,  -100, 100);
  g_rightPercent = constrain(rightPct, -100, 100);

  // 不感帯補正込みで%→振り幅(us)に変換(左右別の不感帯を適用)
  int lSpeed = percentToSwing(g_leftPercent,  DEADBAND_L_US);
  int rSpeed = percentToSwing(g_rightPercent, DEADBAND_R_US);

  // 回転方向の反転フラグを適用(タイヤが逆回転するときはフラグで直す)
  if (INVERT_L) lSpeed = -lSpeed;
  if (INVERT_R) rSpeed = -rSpeed;

  // 中立(停止)usを基準にパルス化し、ESC許容範囲にクランプする。
  int lUs = constrain(SERVO_MID_US + lSpeed, SERVO_MIN_US, SERVO_MAX_US);
  int rUs = constrain(SERVO_MID_US + rSpeed, SERVO_MIN_US, SERVO_MAX_US);
  servoL.writeMicroseconds(lUs);
  servoR.writeMicroseconds(rUs);
}

void stopMotor() {
  setMotorPercent(0, 0);
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

  // 1) ROI帯の平均輝度(露出確認用。しきい値判定には使わない)
  long sum = 0;
  int  cnt = 0;
  for (int y = rTop; y < rBot; y++) {
    const uint8_t* row = fb->buf + (long)y * W;
    for (int x = 0; x < W; x++) { sum += row[x]; cnt++; }
  }
  int mean = cnt ? (int)(sum / cnt) : 128;
  r.bandMean = mean;

  // 2) ローカル適応しきい値で線画素の重心Xを求める(影・外光に強くする)。
  //    各画素を画面全体の1つのしきい値ではなく「横方向に近い周囲の局所平均」と
  //    比べる。影や外光による緩やかな明るさの傾きは局所平均ごと差し引かれて消え、
  //    周囲より局所的に暗い(明るい)線だけが残る。局所平均は running sum で
  //    行ごとに O(W) で求める(追加メモリ不要)。
  const int HW = (int)(W * LOCAL_BG_R);   // 局所背景ウィンドウの半幅[px]
  long sx = 0;
  int  px = 0;
  for (int y = rTop; y < rBot; y++) {
    const uint8_t* row = fb->buf + (long)y * W;
    // x=0 のウィンドウ [0, HW] を初期化
    long wsum = 0;
    int  wn   = 0;
    for (int k = 0; k <= HW && k < W; k++) { wsum += row[k]; wn++; }
    for (int x = 0; x < W; x++) {
      if (x > 0) {                          // 右端を追加・左端を除去してスライド
        int addIdx = x + HW;
        int remIdx = x - HW - 1;
        if (addIdx < W)  { wsum += row[addIdx]; wn++; }
        if (remIdx >= 0) { wsum -= row[remIdx]; wn--; }
      }
      int localMean = (int)(wsum / wn);
      int thresh = LINE_IS_DARK ? (localMean - LINE_MARGIN) : (localMean + LINE_MARGIN);
      bool isLine = LINE_IS_DARK ? (row[x] < thresh) : (row[x] > thresh);
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
    g_prevNorm  = 0.0f;           // 消失中は微分項をリセット(再検出時の急操舵を防ぐ)
    g_normFilt  = 0.0f;           // フィルタ状態もリセット(再検出時に古い値を引きずらない)
    return r;
  }

  // 3) 誤差 → PD制御で旋回量を算出
  r.detected  = true;
  r.centroidX = (int)(sx / px);
  r.error     = r.centroidX - W / 2;            // 右にずれていれば +

  float rawNorm = (float)r.error / (W / 2.0f);  // -1..1(生の正規化誤差)
  // 重心ノイズをローパスで平滑化(直線蛇行の根本対策)。P項・D項の入力を安定させる。
  g_normFilt = NORM_FILT_A * rawNorm + (1.0f - NORM_FILT_A) * g_normFilt;
  float norm = g_normFilt;
  // 直線の微小ノイズによる蛇行を抑えるソフト不感帯。中央付近は直進扱い(操舵0)、
  // 境界を超えたら不感帯分を差し引いた値で滑らかに操舵を立ち上げる。これによりカーブ
  // 入口でいきなり大舵にならず、反応遅れによる消失停止を防ぐ。norm=0のときは P項・
  // D項・減速がすべて無効になり、直線を真っ直ぐ最速で走れる。
  if      (fabsf(norm) < STEER_DEADZONE) norm  = 0.0f;
  else if (norm > 0)                     norm -= STEER_DEADZONE;
  else                                   norm += STEER_DEADZONE;
  float aerr = fabsf(norm);                     // 0..1 コーナーの曲率指標(大きいほど急)
  float dNorm = norm - g_prevNorm;              // 誤差の変化(微分項)。蛇行を制動する
  g_prevNorm = norm;
  int   turn = (int)((KP * norm + KD * dNorm) * 100.0f);
  turn = constrain(turn, -MAX_TURN, MAX_TURN);

  // コーナー自動減速: 急カーブ(aerr大)ほど基準速度を一時的に落として曲がりやすくする。
  // 直線(aerr≈0)では g_baseSpeed 据え置きなので直線速度は犠牲にしない。
  int base = (int)(g_baseSpeed * (1.0f - CORNER_SLOWDOWN * aerr));

  // 線が右(error>0)なら右へ曲がる: 左を上げ・右を下げる
  int outL = base + turn;
  int outR = base - turn;
  // 外輪飽和補償: 外輪が100で頭打ちになると左右差(=旋回の鋭さ)が削られて曲がりきれない。
  // 100を超えた分を反対輪(内輪)から追加で引き、意図した左右差 2*turn を保つ。
  if (outL > 100) { outR -= (outL - 100); outL = 100; }
  if (outR > 100) { outL -= (outR - 100); outR = 100; }
  r.outL = constrain(outL, -100, 100);
  r.outR = constrain(outR, -100, 100);
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

  if (!g_camOk) {
    Serial.printf("[LINE] camera=NG  mode=%s\n", modeStr);
  } else if (r.detected) {
    Serial.printf("[LINE] det=Y mean=%3d cx=%3d/%d err=%+4d px=%4d base=%3d%% -> L=%+4d%% R=%+4d%% | mode=%s%s\n",
                  r.bandMean, r.centroidX, r.width, r.error, r.pixels, g_baseSpeed,
                  r.outL, r.outR, modeStr, driving ? " (駆動中)" : " (出力のみ)");
  } else {
    Serial.printf("[LINE] det=N mean=%3d px=%4d -- ライン消失 -- | mode=%s%s\n",
                  r.bandMean, r.pixels, modeStr,
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
                Serial.println(">> モード: REMOTE (ESP-NOWリモコン)"); break;
      case 's': g_mode = MODE_IDLE;   stopMotor(); Serial.println(">> 停止 (IDLEへ)");                 break;
      default: break;                                  // 改行など無視
    }
  }
}

// ---------------------------------------------------------------------------
// ESP-NOW受信コールバック。WiFiタスク文脈で呼ばれるので軽量に保つ。
// 受信文字列をコピーしてフラグを立てるだけにして、解釈・モーター制御は loop() で行う。
// ※引数 esp_now_recv_info_t* は ESP32 Arduino core 3.x のシグネチャ。
//   core 2.x を使う場合は第1引数を (const uint8_t* mac) に変える必要がある。
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len <= 0) return;
  if (len >= (int)sizeof(g_espnowBuf)) len = sizeof(g_espnowBuf) - 1;
  memcpy(g_espnowBuf, data, len);
  g_espnowBuf[len] = '\0';
  g_espnowRx = true;
}

// ESP-NOW初期化。受信側はpeer登録不要(誰からのブロードキャストでも受け取れる)。
// 送信チャンネルと一致させるため WIFI_STA でチャンネルを固定する。
void setupEspNow() {
  WiFi.mode(WIFI_STA);                  // APは立てない。ESP-NOWはSTAモードで動く
  WiFi.setSleep(false);                 // モデムスリープ無効化で受信取りこぼしを防ぐ
  // 送信側(中継)と同じチャンネルに固定。ここがズレると一切受信できない。
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW初期化失敗 -> 1秒後に再起動");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onEspNowRecv);

  Serial.print("ESP-NOW受信待機 ch=");
  Serial.print(ESPNOW_CHANNEL);
  Serial.print(" 自機MAC=");
  Serial.println(WiFi.macAddress());    // 参考表示(ブロードキャスト運用では未使用)
}

// 起動時オートスタート: カウントダウン中にキー入力が無ければ LINE を自動開始する。
// バッテリー単体(PCなし)なら誰も入力しないので自律走行に入り、PCデバッグ時は
// 何かキーを送れば IDLE のまま留まる。カメラNGなら走行せず IDLE。
void bootAutoStart() {
  if (!g_camOk) {
    g_mode = MODE_IDLE;
    Serial.println("[BOOT] カメラNGのため自動走行せず IDLE で待機");
    return;
  }
  Serial.printf("[BOOT] %d秒後にLINE自律走行を開始します\n", BOOT_AUTOSTART_SEC);
  Serial.println("       中止して検証(IDLE)するなら何かキーを送信...");
  while (Serial.available()) Serial.read();        // 古い入力を捨てる

  for (int i = BOOT_AUTOSTART_SEC; i >= 1; i--) {
    Serial.printf("  %d...\n", i);
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) {
      if (Serial.available()) {                    // キー入力 = PCで作業中
        while (Serial.available()) Serial.read();
        g_mode = MODE_IDLE;
        Serial.println(">> キー入力検出: 検証モード(IDLE)で待機");
        return;
      }
      delay(10);
    }
  }
  g_mode = MODE_LINE;
  Serial.println(">> モード: LINE (自律走行) を自動開始");
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
  stopMotor();                          // 中立usを即反映して停止

  setupEspNow();

  Serial.println("------------------------------------------------------------");
  Serial.println("操作: i=IDLE(検証) / l=LINE(自律) / r=REMOTE(ESP-NOW) / s=停止");
  Serial.println("------------------------------------------------------------");

  bootAutoStart();                      // PCなしなら数秒後にLINE自動開始(キーで中止)

  g_lastCmdMs = millis();
}

// ---------------------------------------------------------------------------
// 受信した1指令文字列を解釈する。ESP-NOW受信(loop経由)から呼ぶ。
// データ形式は従来UDPと同一: "L,R"(例 "50,50") / 単一値 "V"(両輪同値) / モード文字。
// REMOTEモードのときだけ速度指令をモーターに反映する。
void processCommand(const char* buf) {
  // 先頭が英字なら「モード切替コマンド」。Processing から数値指令("L,R")とは
  // 別系統でモードを切り替えられるようにする(シリアル接続なしで操作可能)。
  //   'l'=LINE(自律) / 'r'=REMOTE(リモコン) / 'i','s'=IDLE(停止)
  char cmd = buf[0];
  if (cmd == 'l' || cmd == 'L' || cmd == 'r' || cmd == 'R' ||
      cmd == 'i' || cmd == 'I' || cmd == 's' || cmd == 'S') {
    g_lastCmdMs = millis();
    if (cmd == 'l' || cmd == 'L') {
      g_mode = MODE_LINE;   Serial.println(">> [ESP-NOW]モード: LINE (自律走行)");
    } else if (cmd == 'r' || cmd == 'R') {
      g_mode = MODE_REMOTE; stopMotor(); Serial.println(">> [ESP-NOW]モード: REMOTE (リモコン)");
    } else {
      g_mode = MODE_IDLE;   stopMotor(); Serial.println(">> [ESP-NOW]モード: IDLE (停止)");
    }
    return;
  }

  int l = 0, r = 0;
  if (sscanf(buf, "%d,%d", &l, &r) == 2) {
    // 左右独立
  } else if (sscanf(buf, "%d", &l) == 1) {
    r = l;                             // 単一値は両輪同値
  } else {
    return;                            // 解釈不能なパケットは無視
  }

  g_lastCmdMs = millis();
  if (g_mode == MODE_REMOTE) {
    setMotorPercent(l, r);
    Serial.printf("RX: \"%s\" -> L=%d%% R=%d%%\n", buf, g_leftPercent, g_rightPercent);
  } else if (g_mode == MODE_LINE) {
    // LINE中は左右値の平均を基準速度として採用(0〜100にクランプ)。
    // キープアライブで同値が再送され続けるので、変化したときだけ反映・表示する。
    int base = constrain((l + r) / 2, 0, 100);
    if (base != g_baseSpeed) {
      g_baseSpeed = base;
      Serial.printf("RX: \"%s\" -> LINE基準速度=%d%%\n", buf, g_baseSpeed);
    }
  }
}

void loop() {
  // --- モード切替(シリアル) ---------------------------------------------
  handleSerialCmd();

  // --- 制御ソース1: ESP-NOWリモコン (REMOTEモードのみ駆動) ---------------
  // 受信コールバックが立てたフラグを見て、loop文脈で安全に解釈・反映する。
  if (g_espnowRx) {
    g_espnowRx = false;
    char buf[64];
    strncpy(buf, g_espnowBuf, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    processCommand(buf);
  }

  // --- 制御ソース2: ライントレース -------------------------------------
  // REMOTE中はカメラを回さない。カメラ取り込みでループが伸びる/止まると
  // 指令応答が遅れ、フェイルセーフ誤発火でガクつくため、リモコン時は無効化する。
  if (g_mode != MODE_REMOTE) {
    LineResult lr = lineTrace();
    if (g_mode == MODE_LINE) {
      if (lr.detected) {
        setMotorPercent(lr.outL, lr.outR);
      } else {
        stopMotor();                     // ライン消失で停止
      }
    } else {                             // IDLE: 検証専用、車輪は回さない
      stopMotor();
    }
    printTelemetry(lr);                  // 検出結果をシリアルに出力
  }

  // --- フェイルセーフ: REMOTEで指令が途切れたら停止 ---------------------
  if (g_mode == MODE_REMOTE && millis() - g_lastCmdMs > FAILSAFE_MS) {
    if (g_leftPercent != 0 || g_rightPercent != 0) {
      stopMotor();
      Serial.println("FAILSAFE: 指令途絶のため停止");
    }
  }
}
