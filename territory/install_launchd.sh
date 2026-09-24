#!/bin/zsh
# SC音響エンジンの自動起動を「このマシンに」インストールする。
# リポジトリの場所とユーザー名をその場で解決するので、どのMacでも使える。
#
#   ./install_launchd.sh          # インストール(既存があれば入れ替え)
#   ./install_launchd.sh remove   # アンインストール
set -eu
DIR="$(cd "$(dirname "$0")" && pwd)"
LABEL="com.fu.territory-sound"
PLIST="$HOME/Library/LaunchAgents/$LABEL.plist"

if [ "${1:-}" = "remove" ]; then
  launchctl unload "$PLIST" 2>/dev/null || true
  rm -f "$PLIST"
  pkill -x sclang 2>/dev/null || true
  pkill -x scsynth 2>/dev/null || true
  echo "removed."
  exit 0
fi

if [ ! -x /Applications/SuperCollider.app/Contents/MacOS/sclang ]; then
  echo "!! SuperCollider が /Applications にありません" >&2
  exit 1
fi

launchctl unload "$PLIST" 2>/dev/null || true
pkill -x sclang 2>/dev/null || true
pkill -x scsynth 2>/dev/null || true

mkdir -p "$HOME/Library/LaunchAgents"
cat > "$PLIST" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN"
  "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>Label</key>
  <string>$LABEL</string>
  <key>ProgramArguments</key>
  <array>
    <string>$DIR/launchd_wrapper.sh</string>
  </array>
  <key>RunAtLoad</key>
  <true/>
  <key>KeepAlive</key>
  <true/>
  <key>ThrottleInterval</key>
  <integer>5</integer>
  <key>StandardOutPath</key>
  <string>$DIR/territory_engine.log</string>
  <key>StandardErrorPath</key>
  <string>$DIR/territory_engine.log</string>
</dict>
</plist>
EOF

launchctl load "$PLIST"
echo "installed: $PLIST"
echo "waiting for engine..."
n=0
until grep -q 'engine ready' "$DIR/territory_engine.log" 2>/dev/null; do
  sleep 1
  n=$((n+1))
  if [ $n -ge 60 ]; then
    echo "!! 起動タイムアウト。ログ: $DIR/territory_engine.log" >&2
    exit 1
  fi
done
grep 'engine ready' "$DIR/territory_engine.log" | tail -1
