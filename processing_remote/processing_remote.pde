// ============================================================================
//  Processing UDPリモコン  (ESP32-S3 モーター制御)
// ----------------------------------------------------------------------------
//  - キーボードで左右モーターの出力(%)を入力して ESP32 へ UDP 送信する
//  - 現在の出力(%)を画面に大きく表示＋左右バーで可視化
//  - 入力が無くてもフェイルセーフ防止に直近指令を一定間隔で再送(キープアライブ)
//
//  入力方法:
//    "50"      -> 両輪 50%
//    "50,30"   -> 左50% 右30%（差動。ライントレースのデバッグ用）
//    "-40"     -> 後退40%
//    Enter     -> 送信
//    Backspace -> 1文字削除
//    s / Space -> 即停止(0%)
//    ↑ / ↓     -> 両輪を ±5% 微調整して即送信
//
//  ※ESP32側のモードで送信値の意味が変わる:
//    REMOTE -> 左右モーター出力そのもの / LINE -> 自律走行の基準速度(左右平均)
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

  // --- 出力% 大表示 ---
  textAlign(CENTER, CENTER);
  textFont(fontBig);
  fill(255);
  text(leftPct + "%   " + rightPct + "%", width/2, 90);

  textFont(fontSmall);
  fill(120, 140, 160);
  text("LEFT            RIGHT", width/2, 145);

  // --- 左右バー(中央0, 左マイナス/右プラス) ---
  drawBar(width/2 - 150, 200, leftPct,  "L");
  drawBar(width/2 + 50,  200, rightPct, "R");

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
  text("Enter:送信  s/Space:停止  ↑↓:±5%  例 \"50\" or \"50,30\"", 16, 392);

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
    leftPct = 0; rightPct = 0;
    inputBuf = "";
    send(0, 0);
  } else if (key == CODED) {
    if (keyCode == UP) {
      send(constrain(leftPct + 5, -100, 100), constrain(rightPct + 5, -100, 100));
    } else if (keyCode == DOWN) {
      send(constrain(leftPct - 5, -100, 100), constrain(rightPct - 5, -100, 100));
    }
  } else if ((key >= '0' && key <= '9') || key == '-' || key == ',') {
    inputBuf += key;
  }
}

// 入力文字列を解釈して送信
void commitInput() {
  String s = inputBuf.trim();
  inputBuf = "";
  if (s.length() == 0) return;

  try {
    if (s.indexOf(',') >= 0) {
      String[] p = split(s, ',');
      int l = int(p[0].trim());
      int r = int(p[1].trim());
      send(l, r);
    } else {
      int v = int(s);
      send(v, v);
    }
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
