# 雨エンジン 動かし方手順書

## 0. 録音ファイルを聞くだけなら

`.aiff` はMac標準で開けます。**ダブルクリックすればQuickTime Playerで再生**されます
(デスクトップに `rain_check.aiff` を置いてあります)。ターミナルからなら:

```sh
afplay ~/Desktop/rain_check.aiff        # 再生(Ctrl-Cで停止)
open ~/Desktop/rain_check.aiff          # QuickTimeで開く
```

チャットに送られてくる音声ファイルも、クリックして保存→ダブルクリックで同じです。

---

## 1. 前提確認(初回のみ)

ターミナルで:

```sh
python3 -c "import pythonosc, noise; print('OK')"
```

`OK` と出れば準備完了。`/Applications/SuperCollider.app` が入っていること
(確認済み: SC 3.13)。

> 注意: リポジトリ直下の `bin/` のvenvは**Raspberry Pi用なのでMacでは使えません**。
> 必ず素の `python3` を使ってください。

## 2. SCエンジンを起動する

1. **SuperColliderのIDEを開く**: Finderで `/Applications/SuperCollider.app` をダブルクリック
2. **File > Open** で `rain/rain_engine.scd` を開く
3. **Cmd-A(全選択)→ Cmd-Enter(評価)**
4. 右側のポストウィンドウに数秒後こう出れば稼働中:

   ```
   FU rain engine ready: 29 voices, stereo (dev fold) output, OSC on port 57120
   ```

この時点ではまだ無音です(シーンはフェードアウト状態で待機)。
**音量を一度下げておくこと。**

## 3. 鳴らす

### 方法A: Pythonテスト送信(本番に近い形)

別ターミナルで:

```sh
cd ~/works/25_FU/_dev/python/mqtt_python/rain
python3 rain_test_sender.py
```

2秒フェードインして雨が鳴り始めます。**デフォルトは自動モード** —
雨量・風・素材が数分スケールで勝手に移り変わり、たまに同期アノマリーも
自動で起こります(`--no-anomaly` で止められます)。ターミナルには現在状況が
ライブ表示されます:

```
⏱ 3:12 │ 雨 ▅▅▅▅▅··· 38滴/s ↗ │ 風 0.42 │ 素材 葉→金属屋根 │ 次の異変 ~74s
```

「本降りになってきた」「風が立ち上がってきた」などのイベントも流れます。

自動進行中でも **1行コマンド+Enter** でいつでも介入できます
(値は即ジャンプせず数秒かけて滑らかに移行。介入したパラメータは90秒間
自動から保護され、その後また勝手に動き始めます):

| 入力 | 効果 |
|---|---|
| `a` | 同期アノマリー(2.5秒で同期→5秒パルス雨→5秒で溶け戻る) |
| `m 0` / `m 1` / `m 2` / `m 3` | 素材: コンクリート / 葉 / 金属屋根 / 水面 |
| `m 2.5` | 素材ブレンド(金属と水面の中間) |
| `w 0.7` | 風の強さ(0–1) |
| `d 1.57` | 風向(ラジアン) |
| `r 3` | 雨量3倍(`r 0.3` で小雨) |
| `h 7` | ロボット7番の直下に観客が立つ/去る(トグル) |
| `hm 1` | 熱モード切替: `0`=傘(雨が止む) `1`=素材化(水たまり) |
| `q` | フェードアウトして終了 |

ほかのモード:

```sh
python3 rain_test_sender.py steady  # 自動進行なし、コマンド操作のみ
python3 rain_test_sender.py arc     # 仕様書§7の約6分シーン(3:10と5:00にアノマリー)
python3 rain_test_sender.py sync    # 20秒ごとにアノマリー(位相ゲート確認用)
```

### 方法B: SC IDEだけで鳴らす(音色調整向き)

`rain_engine.scd` の**ファイル末尾のコメントブロック**に手動スニペットがあります。
カーソルを行に置いて **Cmd-Enter** で1行ずつ評価:

```supercollider
~sceneStart.();              // フェードイン
~rateBus.setn(40 ! 29);      // 全ロボット40滴/秒
~matBus.set(3);              // 水面に
~sceneStop.();               // フェードアウト
```

半面豪雨・手動同期パルス・熱・録音のスニペットも同じ場所にあります。

## 4. 録音する

SCのポストウィンドウ下の入力欄(またはエディタ)で:

```supercollider
~startRec.();    // rain/ フォルダに rain_check_<日時>.aiff を書き出し開始
~stopRec.();     // 停止
```

できたファイルはダブルクリックで再生(§0)。

## 5. 終了する

1. Python側: `q` + Enter(または Ctrl-C)→ 2秒フェードアウト
2. SC側: **Cmd-.**(ピリオド)で全停止、SuperColliderを終了

## トラブルシューティング

- **音が出ない** →
  - Pythonを使わない場合は `~sceneStart.()` を評価したか(scene 1が来るまで無音)
  - レートが0ではないか(方法Bなら `~rateBus.setn(40 ! 29)`)
  - Macの出力デバイス・音量を確認(システム設定 > サウンド)
- **`Address already in use` 等でSCが起動失敗** → SuperColliderを二重に起動していないか。
  全部終了して開き直す
- **音がブツブツ途切れる(CPU不足)** → `rain_engine.scd` 冒頭の `~numSub = 6` を
  `4` や `3` に下げて再評価(Cmd-A, Cmd-Enter し直し)
- **設定を変えたい** → `config.json`(フェード時間、マスターゲイン、
  本番4ch化は `dev_stereo: false` + `speaker_channels`)。変更後はSCを評価し直す
