#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-4175}"
URL="http://${HOST}:${PORT}/index.html"

cd "$ROOT_DIR"

if command -v lsof >/dev/null 2>&1 && lsof -iTCP:"$PORT" -sTCP:LISTEN -n -P >/dev/null 2>&1; then
  echo "Port ${PORT} is already in use. Reusing existing server."
else
  python3 -m http.server "$PORT" --bind "$HOST" >/tmp/open-endovascular-trainer-${PORT}.log 2>&1 &
  echo $! >/tmp/open-endovascular-trainer-${PORT}.pid
fi

echo "$URL"
