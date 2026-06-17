// ============================================================================
//  ESP-NOW 中継ブリッジ  (Mac/Processing → 走行ESP32)
// ----------------------------------------------------------------------------
//  Mac(Processing)とUSBシリアルで繋がり、受け取った指令文字列(改行終端)を
//  ESP-NOWブロードキャストで走行ESP32(esp32_motor_espnow)へ中継する。
//
//  なぜ必要か:
//    SoftAP/UDPは接続確立(SSID表示・認証・DHCP)が不安定で「SSIDが出ない/
//    接続できない」が頻発した。無線区間をESP-NOW(コネクションレス)に置き換えると
//    その問題が構造的に消える。だがMacはESP-NOWを直接話せないため、USB接続した
//    このブリッジがシリアル↔ESP-NOWの変換を担う。USB給電なので電源も安定。
//
//  データの流れ:
//    Mac(Processing) --USBシリアル(115200,改行終端)--> 本機 --ESP-NOW--> 走行ESP32
//
//  データ形式(走行側と一致): "L,R"(例 "50,50") / 単一値 "V" / モード文字 'l'/'r'/'i'/'s'
//  チャンネル: 走行側 ESPNOW_CHANNEL と必ず同じ値にすること。ズレると一切届かない。
//
//  対象ボード: 任意のESP32(走行側の予備機でよい)。Macに常時USB接続して使う。
// ============================================================================

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>        // esp_wifi_set_channel(送受信のチャンネル一致用)

const uint8_t ESPNOW_CHANNEL = 1;     // 走行側 esp32_motor_espnow と必ず同じ値に
// 宛先をブロードキャスト(全機宛)にすると相手MACの事前設定が不要でペアリングが
// 要らない。1台運用なら混信しない。複数台を別々に動かすときだけ走行側のMACに変える。
uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

char lineBuf[64];
int  lineLen = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);                 // APは立てない。ESP-NOWはSTAモードで動く
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);  // 受信側と一致必須

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW初期化失敗 -> 1秒後に再起動");
    delay(1000);
    ESP.restart();
  }
  // 送信側はpeer登録が必要(ブロードキャスト宛も登録する)。
  // ※送信完了コールバックは ESP32 core のバージョンでシグネチャが変わるため登録しない
  //   (ブロードキャスト送信に成否確認は不要)。
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("peer登録失敗 -> 1秒後に再起動");
    delay(1000);
    ESP.restart();
  }

  Serial.print("ESP-NOW中継ブリッジ起動 ch=");
  Serial.print(ESPNOW_CHANNEL);
  Serial.print(" 自機MAC=");
  Serial.println(WiFi.macAddress());
  Serial.println("Macからシリアル(115200)で指令を流すと走行ESP32へ中継します");
}

void loop() {
  // Processingが送る文字列を改行終端で1行ずつ取り出してESP-NOW送信する。
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        // 改行は送らない(走行側は先頭文字判定/sscanfで解釈する)。
        esp_now_send(broadcastMac, (const uint8_t*)lineBuf, lineLen);
        lineLen = 0;
      }
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                      // 異常に長い行は捨ててリセット(ノイズ対策)
    }
  }
}
