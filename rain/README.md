# FU Rain Scene — 合成雨エンジン

29台のロボットのKuramoto位相を駆動源にした完全合成のマルチチャンネル雨。
録音素材なし。仕様書のM1–M4を実装済み(M5 = main.py統合は下記スニペット参照)。

## ファイル構成

| ファイル | 役割 |
|---|---|
| `rain_engine.scd` | SuperColliderエンジン本体(29ボイス+ウォッシュ層+風層+熱インタラクション+OSC受信) |
| `rain_client.py` | OSCプロトコルラッパ。main.py統合時はこれをimportするだけ |
| `rain_test_sender.py` | 本番コレオグラフィなしで全OSCを模擬するテスト送信(M2要件) |
| `robots.json` | ロボット天井座標(`choreography/node.csv` から生成、[0,1]²に正規化) |
| `config.json` | OSC宛先・スピーカーch割当・dev用ステレオフォールド設定 |
| `_syntax_check.scd` / `_integration_test.scd` | ヘッドレス検証用スクリプト |

## 使い方

### 1. SCエンジン起動

SC IDEで `rain_engine.scd` を開いて全選択評価(Cmd-A, Cmd-Enter)。
`FU rain engine ready: 29 voices, ...` が出れば稼働。
ファイル末尾にPython不要の手動テストスニペット(素材切替、半面豪雨、手動アノマリー、熱、録音)あり。

ヘッドレスでも可:

```sh
/Applications/SuperCollider.app/Contents/MacOS/sclang rain_engine.scd
```

### 2. テスト送信(Python)

```sh
python3 rain_test_sender.py                   # auto: 天候が自動で移り変わる + 状況ライブ表示 + 対話コマンド
python3 rain_test_sender.py steady            # 自動進行なし、対話コマンドのみ
python3 rain_test_sender.py arc               # §7のシーンアーク(約6分、アノマリー2回)
python3 rain_test_sender.py sync              # 20秒ごとにアノマリー(位相ゲートのデバッグ用)
```

全パラメータはSlew(指数スルー)経由で変化するので、コマンドや自動進行で
値が変わっても急なジャンプにはならない(雨量6秒・風8秒・素材5秒の時定数、
`Sender.__init__` で調整可)。

steadyモードのコマンド: `a`=アノマリー / `m 2.5`=素材モーフ / `w 0.7`=風 /
`d 1.57`=風向 / `r 2`=雨量倍率 / `h 7`=ロボット7の熱トグル / `hm 1`=熱モード切替 / `q`=終了

アノマリー形状は `--attack --hold --release` で調整(§7要件)。

### 3. 確認用録音

4chミックスのステレオフォールドを書き出し(各マイルストーン要件):

```supercollider
~startRec.();   // または ~startRec.("/path/to/file.aiff")
~stopRec.();
```

## OSCプロトコル(仕様書§3準拠)

宛先はsclang既定ポート `127.0.0.1:57120`(`config.json` で変更可)。

| アドレス | 型 | 内容 | レート |
|---|---|---|---|
| `/rain/phases` | 29×float | 位相 [0,2π) | 30 Hz(バンドル) |
| `/rain/rates` | 29×float | 基礎雨滴レート drops/sec | 30 Hz(バンドル) |
| `/rain/heat` | 29×int | 熱検知 0/1 | 30 Hz(バンドル) |
| `/rain/global` | 5×float | depth, material, wind_speed, wind_dir, master_gain_db | 5 Hz |
| `/rain/materials` | 29×float | per-robot素材(-1=グローバル追従)※§3.3の将来拡張、実装済み | 随時 |
| `/rain/heatmode` | int, float | 熱モード(0=umbrella, 1=material)、熱時素材 | 随時 |
| `/rain/scene` | int | 0=フェードアウト, 1=フェードイン(2秒、config変更可) | 随時 |

素材: 0=コンクリート, 1=葉, 2=金属屋根, 3=水面(Minnaert気泡チャープ)。非整数はブレンド。

## main.py 統合スニペット(M5)

main.pyの位相は `ag.firefly_phase` ∈ [0,1) なので **2πを掛けて送る**こと。
メインループは20Hzだが、エンジン側はLagで平滑するのでそのままで問題ない。

```python
from rain.rain_client import RainClient   # パスは配置に合わせて

rain_client = RainClient()                # config.json から宛先を読む

# シーン開始時(mode初期化ブロックで一度だけ)
rain_client.scene(True)
rain_client.set_heat_mode(0)              # 0=umbrella, 1=material

# 毎フレーム(雨シーンのelifブロック内)
phases = [ag.firefly_phase * 2 * math.pi for ag in agents]
rates  = [compute_rain_field(ag) for ag in agents]        # 場の計算はPython側の責務(§3.1)
heat   = [1 if ag.downlight_brightness > 0.5 else 0 for ag in agents]
rain_client.send_frame(phases, rates, heat)

# 低レート(5Hzか変化時)
rain_client.send_global(depth, material, wind_speed, wind_dir, master_gain_db=0)

# シーン終了時
rain_client.scene(False)
```

場の計算(雨セル・風・アノマリーの包絡)は `rain_test_sender.py` の
`RainField` / `Anomaly` / `Kuramoto` クラスがそのまま流用できる。

## チューニング

`rain_engine.scd` 冒頭の変数:

- `~voiceGain` / `~washGain` / `~windGain` — 3層のバランス
- `~numSub = 6` — ボイスあたりの並列雨滴ジェネレータ数。**CPUが重い場合はまずここを下げる**
  (M2チェック時: M1 Mac で scsynth avg CPU ≈ 30%)
- 素材ごとの音色パラメータは `\rainVoice` 内の各素材ブロック(decay・f0レンジ・ゲイン)

`config.json`:

- `dev_stereo: true` — 開発用。4chをステレオに畳んで出力。**本番では false にして
  `speaker_channels` を実機のch割当に合わせる**(scsynthの出力ch数は自動設定)
- `scene_fade_s`, `master_gain_db`

## 実装状況

- [x] M1 — 4素材インパクトSynthDef+素材モーフ(音決めはSC IDEで手動スニペットから)
- [x] M2 — 29ボイス、バイリニアパン、OSC受信、位相ゲート、テスト送信スクリプト
- [x] M3 — ウォッシュ層(局所レート連動・depth脈動)、風層(ガスト伝播・レートブースト)
- [x] M4 — 熱インタラクション両モード(umbrella / material、1秒スルー)
- [ ] M5 — main.pyへのシーン追加(上記スニペット。§10のシーン記述形式確認後)
- [ ] 本番オーディオI/F・スピーカー実配置の確認(§10)

※ M1の合格基準「実録と聞き分けられない」は耳での音決めが必要。エンジン側は
decay/f0/ゲインがすべて `\rainVoice` 内の定数なので、SC IDEで鳴らしながら調整する。

## 配置 (2026-09-09)
- `robots.json` は New Media Week の 25台配置 (`choreography/node_nmw.csv`)。旧29台は `robots_fu29.json`。
  再生成: `python3 choreography/gen_robots_json.py [node.csv]`。スピーカー対応は `config.json` のコメント参照。
