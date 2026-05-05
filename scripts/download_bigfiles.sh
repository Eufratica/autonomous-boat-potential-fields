#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIST="$ROOT/scripts/drive_bigfiles.txt"
REMOTE="${1:-lar_gdrive:xav-sim}"

if [ ! -f "$LIST" ]; then
  echo "Lista não encontrada: $LIST" >&2
  exit 1
fi

rclone copy "$REMOTE" "$ROOT" --files-from "$LIST" -P
