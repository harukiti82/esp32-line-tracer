// ============================================================================
//  Processing UDPリモコン  (ESP32-S3 モーター制御)
// ----------------------------------------------------------------------------
//  - キーボードで BPM(40〜240) を入力して速度を管理し、ESP32 へモーター出力(%)を
//    UDP 送信する。BPM↔速度↔% の対応は speed_bpm_table.pdf に準拠。
//  - 現在の BPM / 速度[m/s] / 出力(%) を画面に表示＋左右バーで可視化
//  - 入力が無くてもフェイルセーフ防止に直近指令を一定間隔で再送(キープアライブ)
//
//  速度管理(BPM):
//    "120"     -> BPM120 相当の速度で前進(両輪同速)
//    Enter     -> 送信
//    Backspace -> 1文字削除
//    s / Space -> 即停止
//    ↑ / ↓     -> BPM を ±5 して即送信
//
//  BPM→速度→% の変換(speed_bpm_table.pdf):
//    v[m/s] = 0.05 + (BPM-40)/(240-40) × (0.33-0.05)   (表が等間隔なので線形)
//    %      = v / 0.33 × 100                            (100%=最高速0.33m/s)
//    例) BPM40→0.05m/s→15% / BPM140→0.19m/s→58% / BPM240→0.33m/s→100%
//
//  モード切替（ESP32へUDPで送信。シリアル接続不要）:
//    l -> LINE  (自律走行。送ったBPMが巡航速度になる)
//    r -> REMOTE(UDPリモコン。送ったBPMが前進速度になる)
//    i -> IDLE  (停止して待機)
//
//  ※ESP32側は従来どおり「%」を受け取る。BPM→% 変換は本スケッチ側で行う。
//
//  UDP送信は Java 標準ライブラリを使用（追加インストール不要）
// ============================================================================

import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.InetAddress;

// ---- 送信先設定（ESP32 に合わせる）----------------------------------------
String ESP32_IP   = "192.168.4.1";   // SoftAP時は 192.168.4.1 / STA時はSerial表示のIP
int    ESP32_PORT = 4210;            // ESP32 側 UDP_PORT と一致させる
int    KEEPALIVE_MS = 200;           // 直近指令の再送間隔(フェイルセーフ対策)

// ---------------------------------------------------------------------------
DatagramSocket socket;
InetAddress    espAddr;

String inputBuf = "";                // 入力中の文字列
int    leftPct  = 0;                 // 現在の左出力(%)
int    rightPct = 0;                 // 現在の右出力(%)
int    lastSendMs = 0;
String lastSent = "";                // 直近送信文字列(再送用)
String currentMode = "?";            // 最後に送ったモード(画面表示用。受信はしないので参考値)
int    currentBpm = 0;               // 現在のBPM(0=停止)。入力で更新し%へ変換して送る

// ---- BPM⇔速度⇔% の対応 (speed_bpm_table.pdf) -----------------------------
final int   BPM_MIN  = 40,    BPM_MAX = 240;   // 表の下限/上限BPM
final float V_MIN    = 0.05f,  V_MAX  = 0.33f; // 対応する最低/最高速度[m/s](実測)
final int   BPM_STEP = 5;                      // ↑↓での増減幅(BPM)

PFont  fontBig, fontMid, fontSmall;

void setup() {
  size(640, 420);
  fontBig   = createFont("SansSerif", 64);
  fontMid   = createFont("SansSerif", 28);
  fontSmall = createFont("SansSerif", 16);

  try {
    socket  = new DatagramSocket();
    espAddr = InetAddress.getByName(ESP32_IP);
  } catch (Exception e) {
    println("UDP初期化失敗: " + e.getMessage());
  }
}

void draw() {
  background(20, 24, 30);

  // --- タイトル ---
  fill(150, 170, 190);
  textFont(fontSmall);
  textAlign(LEFT, TOP);
  text("ESP32 " + ESP32_IP + ":" + ESP32_PORT, 16, 12);

  // 最後に送ったモードを右上に表示(ESP32からは受信しないので「送った値」)
  textAlign(RIGHT, TOP);
  fill(200, 180, 120);
  text("MODE: " + currentMode, width - 16, 12);

  // --- BPM 大表示 ---
  textAlign(CENTER, CENTER);
  textFont(fontBig);
  fill(255);
  text(currentBpm + " BPM", width/2, 80);

  // 速度[m/s] と モーター% を併記(停止時は 0)
  float vNow = (currentBpm <= 0) ? 0 : bpmToV(currentBpm);
  textFont(fontMid);
  fill(120, 200, 160);
  text(nf(vNow, 0, 3) + " m/s     " + leftPct + "%", width/2, 140);

  // --- 左右バー(中央0, 左マイナス/右プラス) ---
  drawBar(width/2 - 150, 205, leftPct,  "L");
  drawBar(width/2 + 50,  205, rightPct, "R");

  // --- 入力欄 ---
  textFont(fontMid);
  textAlign(LEFT, CENTER);
  fill(40, 46, 54);
  noStroke();
  rect(16, 340, width - 32, 44, 8);
  fill(255);
  String caret = (frameCount / 30) % 2 == 0 ? "_" : " ";
  text("> " + inputBuf + caret, 28, 362);

  textFont(fontSmall);
  fill(110, 130, 150);
  textAlign(LEFT, TOP);
  text("Enter:BPM送信 s/Space:停止 ↑↓:±5BPM l:LINE r:REMOTE i:IDLE  例 120", 16, 392);

  // --- キープアライブ再送（フェイルセーフが切れないように）---
  if (millis() - lastSendMs > KEEPALIVE_MS && lastSent.length() > 0) {
    sendRaw(lastSent);
  }
}

// 中央を0として左右に伸びるバーを描く
void drawBar(int x, int y, int pct, String label) {
  int w = 100, h = 24;
  stroke(70, 80, 90);
  noFill();
  rect(x, y, w, h);
  // 中央線
  line(x + w/2, y - 6, x + w/2, y + h + 6);
  noStroke();
  if (pct >= 0) {
    fill(80, 200, 120);
    rect(x + w/2, y, (w/2) * pct / 100.0, h);
  } else {
    fill(220, 90, 90);
    float bw = (w/2) * (-pct) / 100.0;
    rect(x + w/2 - bw, y, bw, h);
  }
  fill(150, 170, 190);
  textFont(fontSmall);
  textAlign(CENTER, TOP);
  text(label, x + w/2, y + h + 8);
}

void keyPressed() {
  if (key == ENTER || key == RETURN) {
    commitInput();
  } else if (key == BACKSPACE) {
    if (inputBuf.length() > 0) inputBuf = inputBuf.substring(0, inputBuf.length() - 1);
  } else if (key == 's' || key == 'S' || key == ' ') {
    inputBuf = "";
    setBpm(0);                                  // 即停止(BPM0)
  } else if (key == 'l' || key == 'L') {
    switchMode("LINE",   "l");              // 自律走行モードへ
  } else if (key == 'r' || key == 'R') {
    switchMode("REMOTE", "r");              // UDPリモコンモードへ
  } else if (key == 'i' || key == 'I') {
    switchMode("IDLE",   "i");              // 停止して待機
  } else if (key == CODED) {
    if (keyCode == UP) {
      setBpm(currentBpm == 0 ? BPM_MIN : currentBpm + BPM_STEP);  // BPMを上げる
    } else if (keyCode == DOWN) {
      setBpm(currentBpm - BPM_STEP);             // BPMを下げる(40未満は最小へ吸収)
    }
  } else if (key >= '0' && key <= '9') {
    inputBuf += key;                             // BPMは正の整数のみ
  }
}

// 入力文字列(BPM)を解釈して送信
void commitInput() {
  String s = inputBuf.trim();
  inputBuf = "";
  if (s.length() == 0) return;
  try {
    setBpm(int(s));               // 入力BPMを速度→%へ変換して送信
  } catch (Exception e) {
    println("入力解釈失敗: " + s);
  }
}

// 出力値を確定して送信
void send(int l, int r) {
  leftPct  = constrain(l, -100, 100);
  rightPct = constrain(r, -100, 100);
  sendRaw(leftPct + "," + rightPct);
}

// BPMを確定して送信する。BPM→速度→% に変換し、両輪同速で送る。
// bpm<=0 は停止、それ以外は表の範囲[40,240]にクランプする。
void setBpm(int bpm) {
  currentBpm = (bpm <= 0) ? 0 : constrain(bpm, BPM_MIN, BPM_MAX);
  int pct = (currentBpm == 0) ? 0 : bpmToPct(currentBpm);
  send(pct, pct);
}

// BPM → 車両速度[m/s]。表(speed_bpm_table.pdf)は BPM25刻み↔0.035m/s刻みで
// 等間隔なので線形でよい。BPM40→0.05, BPM240→0.33。
float bpmToV(int bpm) {
  return V_MIN + (bpm - BPM_MIN) * (V_MAX - V_MIN) / (BPM_MAX - BPM_MIN);
}

// 車両速度[m/s] → モーター出力%。100%=最高速 V_MAX(0.33m/s) を基準に線形。
int vToPct(float v) {
  return round(constrain(v / V_MAX * 100.0f, 0, 100));
}

// BPM → モーター出力%(上記2変換の合成)
int bpmToPct(int bpm) {
  return vToPct(bpmToV(bpm));
}

// モードを切り替える。切替時は速度を必ず0に中立化してから送る——
// 切替直後にキープアライブが直前の速度を送り続けると、REMOTEに入った瞬間に
// 前回値で走り出して危険なため。0,0にしておけば両モードとも停止状態で始まる。
void switchMode(String label, String cmd) {
  currentMode = label;
  currentBpm  = 0;
  inputBuf = "";
  sendCmd(cmd);     // モード切替(キープアライブは汚さない)
  send(0, 0);       // 速度を0に中立化(以後のキープアライブも"0,0"になる)
}

// モード切替などの単発コマンドを送る。キープアライブ(lastSent/lastSendMs)は
// 触らない —— ここを上書きすると REMOTE時の速度再送が止まり、ESP32側の
// フェイルセーフ(500ms)が誤発火して停止してしまうため。UDP欠落に備え数回送る。
void sendCmd(String msg) {
  if (socket == null || espAddr == null) return;
  try {
    byte[] data = msg.getBytes();
    DatagramPacket pkt = new DatagramPacket(data, data.length, espAddr, ESP32_PORT);
    for (int i = 0; i < 3; i++) socket.send(pkt);
  } catch (Exception e) {
    println("コマンド送信失敗: " + e.getMessage());
  }
}

// 文字列をそのままUDP送信（再送・キープアライブ共用）
void sendRaw(String msg) {
  if (socket == null || espAddr == null) return;
  try {
    byte[] data = msg.getBytes();
    DatagramPacket pkt = new DatagramPacket(data, data.length, espAddr, ESP32_PORT);
    socket.send(pkt);
    lastSent   = msg;
    lastSendMs = millis();
  } catch (Exception e) {
    println("送信失敗: " + e.getMessage());
  }
}
