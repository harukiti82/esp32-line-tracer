# Mac 1台で「走行操作」と「test_base監視」を同時に使う

車を走らせながら、荷重センサ側（`test_base`）のBPM判定ログをMac 1台で同時に見るための手順。
スマホ操作化は不要（そもそもポートが競合しないため）。

## なぜ同時にできるのか

シリアルポートの奪い合いは「**同じ1つのポートを2つのプログラムが開く**」ときだけ起きる。
今回は物理的に別デバイス・別ポートなので競合しない。

| 用途 | デバイス | ポート | ボーレート | 使うアプリ |
|------|----------|--------|-----------|-----------|
| 車の操作 | ブリッジESP32 | `/dev/cu.usbmodem5C4D0346321` | 115200 | Processing（`processing_remote_serial`） |
| BPM監視 | 荷重センサArduino（`test_base`） | 別の `/dev/cu.…`（後述） | 9600 | シリアルモニタ（下記いずれか） |

```
[スマホ不要]

  Processing ──USB(115200)──> ブリッジESP32 ──ESP-NOW──> 走行機ESP32
                                                              │ 車が荷重センサを通過
                                                              ▼
  シリアルモニタ <──USB(9600)── test_base(Arduino) ──I2C──> 楽器4台
```

## 手順

### 1. 車の操作（Processing）

`processing_remote_serial/processing_remote_serial.pde` を実行するだけ。
`BRIDGE_PORT` はブリッジESP32のポートに**固定済み**なので、test_baseを後から挿しても
Processingがそちらを誤って掴むことはない。

```
String BRIDGE_PORT = "/dev/cu.usbmodem5C4D0346321";
```

> ⚠️ ブリッジのボードを別の個体に替えたら、このポート名も更新すること。
> 新しいポート名は、ブリッジだけ挿した状態でProcessingを起動し、コンソールに出る
> `Serial.list()` からコピーする。

### 2. test_baseのポート名を調べる

test_baseのArduinoをMacに挿してから、ターミナルで：

```bash
ls /dev/cu.usb*
```

`usbmodem5C4D0346321`（＝ブリッジ）**ではない方**が test_base。
Arduinoの種類により `/dev/cu.usbmodem…` / `/dev/cu.usbserial…` / `/dev/cu.wchusbserial…` の形になる。

### 3. test_baseを監視（別ウィンドウ）

以下のいずれか。Processingとは別アプリなので同時に開ける。

- **ターミナルの `screen`（追加インストール不要）**
  ```bash
  screen /dev/cu.usbXXXX 9600
  ```
  終了は `Ctrl-A` → `K` → `y`。

- **Arduino IDE のシリアルモニタ**
  ボードのポートを test_base 側に選び、ボーレートを **9600** にする。

- **CoolTerm**
  ポートを複数ウィンドウで並べられる。3つ以上のシリアルを同時に見たいときに便利。

これで、Processingで車を走らせ→車が荷重センサを通過→test_baseのウィンドウに
`Tmeas` と判定BPMが流れる、という一連を1台で確認できる。

## トラブルシューティング

- **`Port busy` / つながらない**
  同じポートを2つのアプリが開こうとしている。Arduino IDEのシリアルモニタを開いたまま
  Processingを起動していないか等を確認。`/dev/tty.*` ではなく `/dev/cu.*` を使う
  （`tty.*` は制御線待ちで busy になりやすい）。

- **Processingがtest_base側に繋がってしまう**
  `BRIDGE_PORT` の固定が効いていない。上記手順1のポート名が現物と一致しているか確認。

- **BPMが取れない**
  車の通過（前輪→後輪の2回検知）が必要。センサ閾値やチャタリング判定は
  `test_base/test_base.ino` の `CHATTER_THRESHOLD` / `MAX_GAP_MS` を参照。

## 補足：なぜスマホ操作にしなかったか

当初「Macを空けるため」にスマホ操作化を検討したが、Macは元々競合しておらず不要だった。
仮にやるとしてもWiFiが使用ESP32の不具合で使えないためBLE経由になり、iPhone Safariは
Web Bluetooth非対応・専用アプリが必要・壊れた無線を使う、と費用対効果が低い。
