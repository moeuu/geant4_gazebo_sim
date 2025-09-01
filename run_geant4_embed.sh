#!/usr/bin/env bash
set -eo pipefail

echo "[*] Launching geant4_embed_node (env auto-setup)"

# 環境変数の初期化（省略）

# 環境セットアップは元のスクリプトを踏襲:contentReference[oaicite:7]{index=7}

# --- パラメータ処理 ---
# オプション:
#   --config <yaml|json file> : 設定ファイルを読み込み
#   --param  key=value        : 個別パラメータを追加/上書き（複数指定可）
CONFIG_FILE=""
declare -a PARAMS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config|-c)
      CONFIG_FILE="$2"
      shift 2 ;;
    --param|-p)
      PARAMS+=("$2")
      shift 2 ;;
    --help|-h)
      echo "Usage: $0 [--config <file>] [--param key=value ...]"
      exit 0 ;;
    *)
      echo "[!] Unknown option: $1" >&2
      exit 1 ;;
  esac
done

# 設定ファイルが指定された場合は Python で JSON/YAML を解析し PARAMS に追加
if [[ -n "$CONFIG_FILE" ]] && [[ -f "$CONFIG_FILE" ]]; then
  mapfile -t __kvs < <(python3 - <<'PY' "$CONFIG_FILE"
import sys, json, re
try:
    import yaml  # YAML 読み込み用
except Exception:
    yaml = None

path = sys.argv[1]
with open(path, 'r', encoding='utf-8') as f:
    text = f.read()
data = None
try:
    data = json.loads(text)
except Exception:
    if yaml is None:
        raise
    data = yaml.safe_load(text)
if not isinstance(data, dict):
    raise SystemExit("Config file must contain a top-level dictionary")
for k, v in data.items():
    # リストはブラケット付き文字列に変換
    if isinstance(v, (list, tuple)):
        v_str = '[' + ', '.join(str(item) for item in v) + ']'
    else:
        v_str = str(v)
    k = re.sub(r'\s+','',k)
    print(f"{k}={v_str}")
PY
)
  for kv in "${__kvs[@]}"; do
    PARAMS+=("$kv")
  done
  unset __kvs
fi

# パラメータが存在する場合は --ros-args を付与して起動
declare -a ROS_ARGS=()
if [[ ${#PARAMS[@]} -gt 0 ]]; then
  ROS_ARGS+=("--ros-args")
  for param in "${PARAMS[@]}"; do
    ROS_ARGS+=("-p" "$param")
  done
fi

# geant4_embed_node の起動
exec ros2 run geant4_embed geant4_embed_node "${ROS_ARGS[@]}"
