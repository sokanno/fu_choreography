#!/bin/zsh
# launchd用ラッパー: 孤児scsynthを掃除→ポート解放を待って→sclangをフォアグラウンド起動。
# launchdのKeepAliveがこのプロセスを監視し、死んだら自動で再実行する。
# (sclangが死ぬとscsynthが孤児でUDP 57120を掴み続け、再起動したsclangが
#  57121に逃げて無音になる——それをここで毎回封じる)
set -u
DIR="$(cd "$(dirname "$0")" && pwd)"
SCLANG="/Applications/SuperCollider.app/Contents/MacOS/sclang"

pkill -x scsynth 2>/dev/null
n=0
while lsof -nP -iUDP:57120 >/dev/null 2>&1; do
  sleep 1
  n=$((n+1))
  [ $n -ge 20 ] && exit 1   # launchdが少し置いて再実行してくれる
done
exec "$SCLANG" "$DIR/territory_engine.scd"
