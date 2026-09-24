#!/bin/zsh
# FU territory sound engine — 起動スクリプト(本番/自宅共通)
#
#   ./start_territory_sound.sh          # エンジン起動(既存があれば入れ替え)
#   ./start_territory_sound.sh stop     # 停止
#
# 封じ込めている落とし穴:
#  - sclangをkillするとscsynthが孤児で残りUDP 57120を掴む
#    → 新しいsclangが57121に逃げて無音になる(必ずペアで掃除)
#  - sclangは相対パスだとファイルを見つけられない(フルパス必須)
#  - シーンONはmain.pyが5秒ごとに再送するので、エンジン再起動後の
#    フェードインは自動復旧する

set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
SCLANG="/Applications/SuperCollider.app/Contents/MacOS/sclang"
LOG="$DIR/territory_engine.log"

stop_engine() {
  pkill -x sclang 2>/dev/null
  pkill -x scsynth 2>/dev/null
  local n=0
  while lsof -nP -iUDP:57120 >/dev/null 2>&1; do
    sleep 1
    n=$((n+1))
    if [ $n -ge 15 ]; then
      echo "!! UDP 57120 が解放されません:" >&2
      lsof -nP -iUDP:57120 >&2
      exit 1
    fi
  done
}

if [ "${1:-}" = "stop" ]; then
  stop_engine
  echo "territory engine stopped."
  exit 0
fi

stop_engine
echo "starting territory engine..."
nohup "$SCLANG" "$DIR/territory_engine.scd" > "$LOG" 2>&1 &

n=0
until grep -q 'engine ready' "$LOG" 2>/dev/null; do
  sleep 1
  n=$((n+1))
  if [ $n -ge 60 ]; then
    echo "!! 起動タイムアウト。ログ: $LOG" >&2
    tail -5 "$LOG" >&2
    exit 1
  fi
done
grep 'engine ready' "$LOG" | tail -1
if grep -q 'port 57121' "$LOG"; then
  echo "!! ポートが57121に逃げています。もう一度 ./start_territory_sound.sh を実行してください" >&2
  exit 1
fi
echo "OK (log: $LOG)"
