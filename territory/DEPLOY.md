# 本番機セットアップ手順（陣取りシーン音響）

開発機(kannosoのMBP)で検証済みの構成を、本番PCに移すためのチェックリスト。
開発中に実際に踏んだ罠を含む。

## 1. 前提ソフトウェア

- [ ] **SuperCollider** を `/Applications/SuperCollider.app` に（3.13以上で検証済み）
- [ ] **Python 3.11系** + 依存: `pip install python-osc noise paho-mqtt vpython "setuptools<81"`
  - ※ **setuptools は 81 未満に固定**。81以降は `pkg_resources` が消えて
    vpython の import が `ModuleNotFoundError: pkg_resources` で死ぬ（開発機で実際に発生）
- [ ] **mosquitto**（ローカルでMQTTブローカーを走らせる場合）: `brew install mosquitto`
- [ ] **BlackHole 16ch**: `brew install blackhole-16ch`（Max経由の音声経路用・要管理者パスワード）

## 2. オーディオ経路（SC → BlackHole → Max → RME）

本番IFは **RME**（開発機のMOTUとは異なる）。SC はハードウェアIFに一切触れず
BlackHole にのみ出力するので、**SC側の設定はIFが何であっても同一**。
実機に出すのは Max だけ。

- [ ] Audio MIDI 設定でアグリゲートデバイス作成: **RME + BlackHole 16ch**
- [ ] Max のオーディオドライバをアグリゲートに。`adc~` で BlackHole ch1〜4 を受けて
      マスター段へ、`dac~` は RME のchへ（BlackHoleのchはアグリゲート内で
      RMEの入力数の後ろに並ぶ。RMEは入力ch数が多いので番号に注意）
- [ ] `territory/config.json` を切り替え:
  ```json
  "dev_stereo": false,
  "output_device": "BlackHole 16ch",
  "speaker_channels": { "FL": 0, "FR": 1, "RL": 2, "RR": 3 }
  ```
  ※ この speaker_channels は BlackHole ch1-4 への割当。**観客位置との対応
  （どのchがどの隅か）は Max 側のルーティングで最終確認**（PRODUKCJA p.47参照、
  現行コメントの venue 割当メモも参照）

## 3. main.py 側

- [ ] `place = "venue"` に戻す（39行目。ブローカーが venue→localhost の順で
      フォールバックするので、自宅でも venue のままで動くが起動が遅くなる）
- [ ] `layout = "nmw"`（44行目）を確認
- [ ] シーン切替は Max → OSC `/menu 8` (port 8000)。SCのフェードイン/アウトは
      main.py が自動送信（5秒ごとキープアライブ付き＝SC再起動に自己復旧）

## 4. SCエンジンの自動起動

```sh
cd <repo>/territory
./install_launchd.sh        # ログイン時自動起動+クラッシュ自動再起動
./install_launchd.sh remove # 解除
```

- パスはインストール時にその場で解決される（マシン依存なし）
- ログ: `territory/territory_engine.log`
- 手動運用なら `./start_territory_sound.sh` / `./start_territory_sound.sh stop`

## 5. 動作確認（main.py なしで音だけ）

```sh
cd <repo>/territory
python3 territory_test_sender.py --seed 7
```

R/C/陣営数のライブ表示が動き、音が鳴ればOK（q で終了）。
本番系は main.py を起動して Max から `/menu 8`。

## 6. 既知の罠

- **sclang を kill すると scsynth が孤児で残り UDP 57120 を掴む**
  → 次の sclang が 57121 に逃げて無音になる。必ず `pkill -x sclang; pkill -x scsynth`
  をセットで（スクリプト類は全て対策済み）
- sclang はカレントディレクトリ相対でファイルを見つけられない（フルパス起動）
- エンジン起動ログの `FU territory engine ready: ... OSC on port 57120` を必ず確認。
  **57121 と出ていたら上記の罠**
- ポート一覧: SC=57120 / main.pyのOSC受信(Max→)=8000 / MQTT=1883
