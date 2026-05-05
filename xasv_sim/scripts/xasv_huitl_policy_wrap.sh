#!/usr/bin/env bash
set -euo pipefail

# Script "wrapper" para o nó de política HITL:
# - Antes: SYSID_MYGCS = 1  (libera RC_OVERRIDE do MAVROS)
# - Depois / Ctrl+C: SYSID_MYGCS = 255 (volta ao padrão)
#
# USO:
#   ./xasv_huitl_policy_wrap.sh ~/ros_ws/trained_rc_policy.pt
#
# OBS:
#   O caminho do modelo é passado para o node via parâmetro ROS privado:
#     _model_path:=/caminho/absoluto/para/modelo.pt

PARAM_NAME="SYSID_MYGCS"
ON_VALUE=1        # valor que você usa para a política
OFF_VALUE=255     # valor padrão que você quer restaurar

MODEL_IN="${1:-}"
if [[ -z "${MODEL_IN}" ]]; then
  echo "[HITL-WRAP] ERRO: você precisa passar o caminho do modelo .pt"
  echo "[HITL-WRAP] Uso: $0 /caminho/para/modelo.pt"
  exit 1
fi

# Expande "~" e gera caminho absoluto de forma confiável
MODEL_PATH="$(python3 - <<'PY' "${MODEL_IN}"
import os, sys
p = os.path.abspath(os.path.expanduser(sys.argv[1]))
print(p)
PY
)"

if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "[HITL-WRAP] ERRO: modelo não encontrado em: ${MODEL_PATH}"
  exit 1
fi

cleanup() {
    echo "[HITL-WRAP] Restaurando $PARAM_NAME para $OFF_VALUE..."
    rosrun mavros mavparam set "$PARAM_NAME" "$OFF_VALUE" || \
        echo "[HITL-WRAP] Falha ao restaurar $PARAM_NAME (MAVROS/FCU vivo?)"
    exit 0
}

# Quando der Ctrl+C nesse terminal, chama cleanup em vez de matar seco
trap cleanup INT

echo "[HITL-WRAP] Setando $PARAM_NAME = $ON_VALUE..."
rosrun mavros mavparam set "$PARAM_NAME" "$ON_VALUE" || {
    echo "[HITL-WRAP] Não consegui setar $PARAM_NAME = $ON_VALUE. Abortando."
    exit 1
}

echo "[HITL-WRAP] Modelo encontrado em: ${MODEL_PATH}"
echo "[HITL-WRAP] Iniciando nó xasv_huitl_policy_node.py..."

# PASSO CRÍTICO: passar como parâmetro privado do node (prefixo "_")
rosrun xasv_sim xasv_huitl_policy_node.py _model_path:="${MODEL_PATH}"

# Se o nó sair naturalmente (sem Ctrl+C), ainda assim restauramos:
cleanup

