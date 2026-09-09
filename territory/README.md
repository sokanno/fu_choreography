# FU Territory Scene — 陣取りサウンドエンジン

意見ダイナミクス（陣取りコレオグラフィ）の状態をそのまま音楽にするエンジン。
**意見の円を五度圏にマップする**のが核:意見の距離＝和声の距離。対立する陣営は
トライトーンで軋み、近い意見は五度で協和し、合意はユニゾンに溶ける。

| モデルの変数 | 音 |
|---|---|
| 意見の角度 | ピッチクラス(五度圏、3レジスタ) — 争点転換で全声部がグリッサンド |
| 確信度 | 声量とフィルタの開き(迷い＝息のノイズ、確信＝声) |
| 疲弊 | デチューン+トレモロ(燃え尽きた声はかすれ、音程が揺れる) |
| 高さ | 近接感(降りてくるほど大きく明るい) |
| ピッチ回転±60° | +下向き=聴衆へ開いた声 / −上向き=顔を背けたこもった声 |
| ヨー角速度 | 空気の swish(回転が聞こえる) |
| 半回転ごと | ノック(打撃音)。**向きが同期すると勝手に行進になる** |
| R×C(斉一化) | 暗いユニゾンのサブドローン(全体主義の質量) |
| 争点の転換 | 低いブーム+空気のスウェル、その後全声部が新しい和音へ再調律 |

## ファイル構成

| ファイル | 役割 |
|---|---|
| `territory_engine.scd` | SCエンジン本体(29ドローン+ティック+マス層+転換ジェスチャ+OSC受信) |
| `territory_client.py` | OSCプロトコルラッパ。main.py統合時はこれをimportするだけ |
| `territory_test_sender.py` | **シミュレーション本体入り**のテスト送信。ウィジェットで設計したモデル(意見ベクトル+素質+顕在軸の漂流/自動転換+疲弊+トルク式の向き+高さ+チルト)を完全に含む |
| `robots.json` | ロボット天井座標(rain/と同一、world_* がメートル) |
| `config.json` | OSC宛先・スピーカーch割当・dev用ステレオフォールド設定 |
| `_syntax_check.scd` | ヘッドレス構文チェック |

## 使い方

### 1. SCエンジン起動

SC IDEで `territory_engine.scd` を開いて全選択評価(Cmd-A, Cmd-Enter)。
`FU territory engine ready: 29 voices, ...` が出れば稼働。
ファイル末尾にPython不要の手動スニペット(二陣営のトライトーン、合意ユニゾン、
息の群衆、燃え尽き、行進、転換ジェスチャ)あり。

ヘッドレスでも可:

```sh
/Applications/SuperCollider.app/Contents/MacOS/sclang territory_engine.scd
```

### 2. テスト送信(Python)

```sh
python3 territory_test_sender.py
```

社会は自律的に動く(自動転換ON)。ターミナルに R / C / 陣営数 / 疲弊 / 回転 /
高さのライブ表示と、閾値イベントのタイムスタンプ付きログ(全体主義的整列の検出、
斉一化、崩壊、転換)が流れる。

コマンド: `s`=手動転換 / `auto 0|1` / `scan 0.09` / `ali 0.04` / `stare 0.07` /
`unif 1.5` / `g -6`=ゲインdB / `q`=終了

`--seed 7` で再現可能な社会、`--no-auto` で自動転換なし。

### 3. 確認用録音

```supercollider
~startRec.();   // ステレオフォールドをterritory/に書き出し
~stopRec.();
```

## スピーカー

rain と同じ:**本番4ch / 開発2ch**。`config.json` の `dev_stereo: true` が
ステレオフォールド。本番では `false` にして `speaker_channels` を実機ch割当に
合わせる。パンはロボット天井座標のバイリニア(rain と同一方式)。

## OSCプロトコル

宛先 `127.0.0.1:57120`(config.jsonで変更可)。

| アドレス | 型 | 内容 | レート |
|---|---|---|---|
| `/terr/op` | 29×float | 意見角(**アンラップ済み連続値**、rad) | 20 Hz(バンドル) |
| `/terr/conv` | 29×float | 確信度 0..1 | 20 Hz |
| `/terr/fat` | 29×float | 疲弊 0..1 | 20 Hz |
| `/terr/ht` | 29×float | 高さ 0=天井..1=降下 | 20 Hz |
| `/terr/tilt` | 29×float | チルト -1..+1(+が下=聴衆向き) | 20 Hz |
| `/terr/om` | 29×float | ヨー角速度の絶対値 rad/s | 20 Hz |
| `/terr/global` | 5×float | R, C, 陣営数, 平均意見角, gain dB | 5 Hz |
| `/terr/tick` | int,float,int | ロボットidx(0始まり), 強さ0..1, ピッチクラス | 半回転ごと |
| `/terr/shift` | int | 1=自動転換, 0=手動 | 転換時 |
| `/terr/scene` | int | 0=フェードアウト, 1=フェードイン | 随時 |

意見角はSC側のLagがそのままグリッサンドになるので**必ずアンラップして送る**
(±πで折り返すと転換のたびに逆回りの滑走が出る)。

## main.py 統合スニペット

シミュレーションのクラスは `territory_test_sender.py` の `Territory` が
そのまま使える(20Hzループ前提、dtは可変)。実機の向き制御は `om`(rad/tick)
を実角速度に変換して使い、高さ・チルトは `ht` / `tilt` 配列を実レンジへ。

```python
from territory.territory_client import TerritoryClient
from territory.territory_test_sender import Territory, pitch_class

terr_client = TerritoryClient()
terr = Territory([(r.world_x, r.world_y) for r in robots])

# シーン開始時
terr_client.scene(True)

# 毎フレーム(20Hz)
ticks, auto_shifted = terr.step(dt)
if auto_shifted:
    terr_client.shift(True)
terr_client.send_state(terr.op_unwrap, terr.conviction(), terr.w,
                       terr.ht, terr.tilt, terr.omega_rad_s())
for i, vel in ticks:
    terr_client.tick(i, vel, pitch_class(terr.op_unwrap[i]))

# 低レート(5Hz)
terr_client.send_global(terr.R, terr.C, terr.camps, terr.mean_op())

# シーン終了時
terr_client.scene(False)
```

展示検証用にはこのループ内で R / C / 陣営数 / 平均疲弊 / 転換イベントを
毎秒CSVへ落とすことを推奨(相転移の周期をあとから確認できる)。

## チューニング

`territory_engine.scd` 冒頭:

- `~droneGain` / `~tickGain` / `~massGain` / `~shiftGain` — 4層のバランス
- `~baseMidi = 39` — 全体の音域(レジスタは +0/+12/+24 の3段)
- `~revMix` — リバーブ量
- 音色は `\terrDrone` 内(cutoffの式、breathクロスフェード閾値、デチューン幅)

モデル側(音の「振る舞い」を変える): `territory_test_sender.py` の REPL で
`scan / ali / stare / unif` をライブ調整。徘徊スピンを上げると秒針の群れ、
凝視を上げると睨み合いの静寂が増える。

## 実装状況

- [x] エンジン(29ドローン+ティック+マス+転換、4ch/2ch出力、OSC受信)
- [x] シミュレーション内蔵テスト送信(自動転換・イベントログ・REPL)
- [x] SC構文チェック / Python構文チェック / 12秒スモークラン
- [ ] 音決め(SC IDEで手動スニペット+テスト送信を鳴らしながら)
- [ ] main.py へのシーン統合(上記スニペット)
- [ ] 実機の向き・高さ・チルト実レンジとの対応付け

## 配置 (2026-09-09: New Media Week 用に更新)
- `robots.json` は **NMW 25台配置** (`choreography/node_nmw.csv`: 1m ライン5本×5台、偶数ラインは 0.5m ずらし、5.1m×4m)。
  従来の29台配置は `robots_fu29.json` に退避。再生成は `python3 choreography/gen_robots_json.py [node.csv]`
  (rain/ と territory/ の両方に書き出す)。main.py 側は `layout = "nmw"|"fu"` で同じ配置を選ぶこと。
- スピーカー: 配置図 (PRODUKCJA p.47) の audio1=左上 / audio2=右上 / audio3=左下 / audio4=右下 + SUB(左中央)。
  `config.json` の speaker_channels はこの4隅にパン四隅 FL/FR/RL/RR を対応させたもの(**出力ch番号は現場で要確認**)。
- viz.html は robots.json の extent_m から自動でスケール(長辺が横)。
- **ID/正面 (2026-09-09)**: 正面=配置図の右側(PC側)。`choreography/gen_node_nmw.py` (FRONT="right") が観客視点で番号付け: 行1=手前の5台を左→右 (ID1=手前左, ID5=手前右), 行5=奥 (ID25=奥右)。Max用パン表 `choreography/nmw_pan_table.txt` (coll: id, lr fb; 0=左/手前)。スピーカー 1手前左 2手前右 3奥左 4奥右 = 出力ch0..3。正面を変えるなら FRONT と main.py の cameraAngle と config.json の speaker_channels を同時に変更。
