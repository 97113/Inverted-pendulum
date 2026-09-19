# 車輪型倒立振子ラジコン

M5Stack Core2 と RobStride EduLite 02 × 2 で作る、Xbox コントローラ操縦の
車輪型倒立振子（バランシングロボット）のファームウェアです。

| 役割 | 使用部品 |
| --- | --- |
| 制御 + 姿勢センサ | M5Stack Core2（内蔵 IMU） |
| CAN + 電源 | M5Stack PwrCAN Module 13.2 |
| 駆動 | RobStride EduLite 02 × 2（CAN 1 Mbps / 29bit 拡張フレーム） |
| 操縦 | Xbox Series X\|S コントローラ（BLE） |
| 電源 | 18650 × 8 直列 |

> ⚠️ **PwrCAN Module 13.2 の電源入力は DC 9〜24 V です。**
> 18650 8 直列（満充電 33.6 V）を直結するとモジュールが壊れます。
> 必ず [docs/hardware.md](docs/hardware.md) の電源構成を読んでから配線してください。

---

## ビルドと書き込み

[PlatformIO](https://platformio.org/) を使います。

```bash
pio run              # ビルド
pio run -t upload    # Core2 に書き込み
pio device monitor   # シリアルモニタ (115200 baud)
```

依存ライブラリは `platformio.ini` に記載してあり、初回ビルド時に自動で取得されます。

- `m5stack/M5Unified` — 本体・画面・IMU・電源
- `asukiaaa/XboxSeriesXControllerESP32_asukiaaa` — Xbox コントローラの BLE 接続

CAN は ESP32 内蔵の TWAI コントローラを直接使っているため、追加ライブラリは不要です。

---

## 操作方法

電源投入後、ジャイロのゼロ点較正のため機体を **静止させたまま約 2 秒待ちます**。
`DISARMED` になったら操作できます。

| 操作 | 動作 |
| --- | --- |
| コントローラ **A** / 本体 **BtnA** | ARM（起立開始）。ほぼ直立していないと受け付けません |
| コントローラ **B** / 本体 **BtnB** | DISARM（モータ停止） |
| 本体 **BtnC** | ジャイロ再較正 + 走行距離のリセット |
| 左スティック **上下** | 前進 / 後退 |
| 右スティック **左右** | 旋回 |
| **RB** 押しっぱなし | 速度リミッタ解除（通常は最大値の 50 %） |

安全のため、以下の場合は自動でモータを止めます。

- 傾きが `kFallPitchDeg`（既定 35°）を超えた（転倒）
- モータからの CAN フィードバックが途絶えた
- CAN がバスオフになった
- モータが故障ビットを立てた
- コントローラの通信が 500 ms 途絶えた（指令が中立に戻る。バランス制御は継続）

---

## 画面表示

```
┌────────────────────────────────────┐
│ ARMED                    PAD OK    │  状態 / コントローラ・CAN の接続
│                          CAN OK    │
│  +1.23 deg      rate   +12.4 dps   │  姿勢角と角速度
│ ▓▓▓▓▓▓▓▓▓▓│░░░░░░░░░░░░░░░░░░░░░ │  傾きバー
│ vel +0.12 m/s  pos +0.34 m  trim.. │  走行速度・距離・学習した姿勢オフセット
│ cmd fwd +0.50  turn +0.00          │  操縦入力
│ out  L  +1.24   R  +1.31           │  車輪トルク指令 [N·m]
│ M1 ok 32.4C f=00   M2 ok 31.8C f=00│  モータ状態（温度・故障ビット）
│ VBUS  31.8V  Core2 4.05V  PAD  87% │  母線電圧・電池
└────────────────────────────────────┘
```

`DISARMED` / `CALIBRATING` のときは IMU の生値も表示されます。
Core2 を既定と違う向きに取り付けた場合、この値を見ながら
`include/config.h` の軸設定を決めてください。

---

## ソース構成

| ファイル | 内容 |
| --- | --- |
| `include/config.h` | **調整パラメータは全部ここ**。ピン、モータ ID、ゲイン、安全閾値 |
| `src/main.cpp` | タスク構成と状態遷移（BOOT → CALIB → DISARM → ARM → FALLEN/FAULT） |
| `src/robstride.{h,cpp}` | RobStride / CyberGear 系の CAN プロトコル（TWAI ドライバ） |
| `src/attitude.{h,cpp}` | IMU の読み出しと相補フィルタによる姿勢推定 |
| `src/balance.{h,cpp}` | 状態フィードバックによるバランス制御と旋回 |
| `src/gamepad.{h,cpp}` | Xbox コントローラの BLE 接続と入力の正規化 |
| `src/ui.{h,cpp}` | Core2 の画面描画 |
| `src/i2c_bus.{h,cpp}` | 内蔵 I2C（IMU / PMIC / タッチ）の排他制御 |

タスクは 2 本です。

- **core 1 / 200 Hz** — 姿勢推定 → バランス制御 → CAN 送受信
- **core 0 / 50 Hz** — BLE の接続処理、画面描画、本体ボタン

BLE のスキャンは内部で待ちが入るため、制御ループとは必ず別コアに分けています。

---

## ドキュメント

- [docs/hardware.md](docs/hardware.md) — 配線、電源、ディップスイッチ、初回通電チェック
- [docs/tuning.md](docs/tuning.md) — 制御ゲインの調整手順と症状別の対処

---

## 既知の要確認事項

実機のマニュアルと突き合わせが必要な箇所です。`include/config.h` にも同じ注意書きがあります。

- **モータのスケーリング定数** — `kPosMax` / `kVelMax` / `kTorqueMax` は
  RobStride 02 系の一般値（±4π rad、±44 rad/s、±17 N·m）を入れてあります。
  EduLite 02 のマニュアルの「パラメータ範囲」の表と必ず突き合わせてください。
  ここがずれると、トルク指令もフィードバックも定数倍ずれます。
- **EduLite 02 の動作電圧範囲** — 18650 8 直列（24〜33.6 V）が範囲内かを確認してください。
- **PwrCAN の絶縁トランシーバ側の電源** — `docs/hardware.md` の推奨構成
  （DC-DC で 12 V を供給）で確認することを勧めます。
