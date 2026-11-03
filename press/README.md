# press/ — 縮む部屋 (パントマイム)

上と下から見えない壁が迫る。筒は壁の間を上下にバウンスして壁を可視化し、
バウンスのたびに音が鳴る。壁は時々 ±30° 傾き、行ったり来たりしながら少しずつ
閉じ、最後は平らに戻って筒を潰す。約2秒の微振動ののち壁は急に消え、封じられて
いたバウンスのエコーがあふれ、筒はそれぞれの高さへふわっと離散する。
rain/ ・ territory/ と同じ流儀の独立ディレクトリ。

## ファイル
- `press_model.py` — コレオグラフィ本体 (純Python, 20Hz)。main.py と test sender が共有。
  定数 (閉じる時間・傾き・速度など) はファイル冒頭にまとめてある。
- `press_client.py` — OSCラッパ (main.py が import)。プロトコルは docstring 参照。
- `press_engine.scd` — SuperCollider 音響エンジン (port 57120, `/press/*`)。
- `press_test_sender.py` — VPythonなしでモデル+音を回す開発ツール。

## 設定の共有
config.json / robots.json は **territory/ のものを共有**する (出力デバイス・
スピーカーch・配置を二重管理しない)。配置を変えたら従来どおり
`choreography/gen_robots_json.py` で territory 側を再生成すればよい。

## 起動
- 開発: `/Applications/SuperCollider.app/Contents/MacOS/sclang press_engine.scd`
- 本番: territory_engine.scd の末尾が press_engine.scd を同じ sclang に load する
  ので、launchd (install_launchd.sh) はそのままで両シーンの音が出る。
  環境変数は `~pr*` / OSC は `/press/*` で territory と衝突しない。

## main.py 統合
モード名「縮む部屋」(modes 末尾, /menu index 9 — UI.maxpat への追加は別途)。
マッピング: モデルが z/yaw/pitch/flash を直接出す。色は青白ベース+バウンス発光、
解放後は暖色へ。壁は3D表示に半透明の板として描画 (実機には存在しない)。

## 音の構造 (press_engine.scd ヘッダにも記載)
- バウンス打音: 壁間隔が狭いほど高く短い → 終盤に向かって自動クレッシェンド
- 圧力ドローン: 狭まるほど唸り + 鳴きが迫り上がる。解放で即断
- 潰しのラトル: 微振動2秒の高密度きしみ
- エコー槽: 壁がある間はゲート0.1で「密室」。/press/release で revtime 9s に
  開き、溜まった響きがあふれて自然減衰
