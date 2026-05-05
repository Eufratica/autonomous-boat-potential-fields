#!/usr/bin/env bash
set -euo pipefail

# ============================================
# Configuráveis (ajuste aqui se quiser)
# ============================================

# Fork do Tiago
FORK_URL="${FORK_URL:-https://github.com/ttrindader/ardupilot.git}"

# Onde manter o clone "de trabalho" do fork
REPO_DIR="${REPO_DIR:-$HOME/ardupilot_ttr}"

# Raiz onde ficarão as pastas de patches geradas
PATCH_ROOT="${PATCH_ROOT:-$HOME/hitl_patches}"

# Branches
MASTER_BRANCH="${MASTER_BRANCH:-master}"
HITL_BRANCH="${HITL_BRANCH:-hitl}"
HITL_CLEAN_BRANCH="${HITL_CLEAN_BRANCH:-hitl_clean}"

# ============================================
# Início
# ============================================

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
PATCH_DIR="${PATCH_ROOT}/${TIMESTAMP}"

echo "==> Pasta de patches: ${PATCH_DIR}"
mkdir -p "${PATCH_DIR}"

# --------------------------------------------
# 1) Clonar fork se ainda não existir
# --------------------------------------------
if [ ! -d "${REPO_DIR}/.git" ]; then
  echo "==> Clonando fork em ${REPO_DIR}"
  git clone "${FORK_URL}" "${REPO_DIR}"
else
  echo "==> Usando repositório existente em ${REPO_DIR}"
fi

cd "${REPO_DIR}"

# --------------------------------------------
# 2) Atualizar master e hitl a partir do origin
# --------------------------------------------
echo "==> Buscando últimas alterações do origin..."
git fetch origin

echo "==> Garantindo ${MASTER_BRANCH} limpo em origin/${MASTER_BRANCH}..."
git checkout "${MASTER_BRANCH}"
git reset --hard "origin/${MASTER_BRANCH}"

echo "==> Garantindo ${HITL_BRANCH} limpo em origin/${HITL_BRANCH}..."
git checkout "${HITL_BRANCH}"
git reset --hard "origin/${HITL_BRANCH}"

# --------------------------------------------
# 3) Criar branch hitl_clean e fazer rebase
# --------------------------------------------
echo "==> Criando branch limpa: ${HITL_CLEAN_BRANCH}"
if git rev-parse --verify "${HITL_CLEAN_BRANCH}" >/dev/null 2>&1; then
  echo "   (Removendo branch local antiga ${HITL_CLEAN_BRANCH})"
  git branch -D "${HITL_CLEAN_BRANCH}"
fi

git checkout -b "${HITL_CLEAN_BRANCH}"

echo "==> Rebase de ${HITL_CLEAN_BRANCH} em ${MASTER_BRANCH}..."
git rebase "${MASTER_BRANCH}"

# --------------------------------------------
# 4) Gerar patches, logs e diffs
# --------------------------------------------
echo "==> Gerando série de patches..."
git format-patch "${MASTER_BRANCH}..${HITL_CLEAN_BRANCH}" -o "${PATCH_DIR}"

echo "==> Salvando lista de commits..."
git log --oneline "${MASTER_BRANCH}..${HITL_CLEAN_BRANCH}" > "${PATCH_DIR}/HITL_COMMITS.txt"

echo "==> Salvando diffstat..."
git diff --stat "${MASTER_BRANCH}..${HITL_CLEAN_BRANCH}" > "${PATCH_DIR}/HITL_DIFFSTAT.txt"

echo "==> Salvando diff completo (apenas referência, NÃO usado pelo git am)..."
git diff "${MASTER_BRANCH}..${HITL_CLEAN_BRANCH}" > "${PATCH_DIR}/HITL_FULL_DIFF.diff"

# --------------------------------------------
# 5) Gerar automaticamente o apply_ardupilot_hitl_series.sh
# --------------------------------------------
echo "==> Criando script de aplicação em: ${PATCH_DIR}/apply_ardupilot_hitl_series.sh"

cat > "${PATCH_DIR}/apply_ardupilot_hitl_series.sh" << 'EOF'
#!/usr/bin/env bash
set -euo pipefail

# Diretório onde estão os .patch
# - Por padrão, pega a pasta onde este script está
# - Pode ser sobrescrito com: PATCH_DIR=/outra/pasta ./apply_...
PATCH_DIR="${PATCH_DIR:-$(cd "$(dirname "$0")" && pwd)}"

# Repositório alvo (por padrão, diretório atual)
REPO_DIR="${1:-.}"

echo "==> Aplicando patches HITL"
echo "    PATCH_DIR: ${PATCH_DIR}"
echo "    REPO_DIR : ${REPO_DIR}"

if [ ! -d "${REPO_DIR}/.git" ]; then
  echo "ERRO: ${REPO_DIR} não parece ser um repositório git."
  exit 1
fi

cd "${REPO_DIR}"

# Verificar se o repositório está limpo
if ! git diff-index --quiet HEAD --; then
  echo "ERRO: repositório com modificações locais."
  echo "      Faça commit/stash ou use outro diretório antes de aplicar os patches."
  exit 1
fi

# Aplicar todos os .patch na ordem
for patch in "${PATCH_DIR}"/*.patch; do
  [ -e "${patch}" ] || continue

  base="$(basename "${patch}")"

  # Segurança extra: se por acaso existir um HITL_FULL_DIFF.patch, ignorar
  if [[ "${base}" == "HITL_FULL_DIFF.patch" ]] || [[ "${base}" == "HITL_FULL_DIFF.diff" ]]; then
    echo "==> Ignorando ${base} (diff agregado apenas para referência)"
    continue
  fi

  echo "==> Aplicando ${base}"
  git am --3way "${patch}"
done

echo "==> Série HITL aplicada com sucesso!"
EOF

chmod +x "${PATCH_DIR}/apply_ardupilot_hitl_series.sh"

# --------------------------------------------
# 6) Mensagem final
# --------------------------------------------
echo
echo "=============================================================="
echo "Concluído!"
echo
echo "Pasta de patches gerada:"
echo "  ${PATCH_DIR}"
echo
echo "Arquivos úteis:"
echo "  - HITL_COMMITS.txt      (lista de commits)"
echo "  - HITL_DIFFSTAT.txt     (resumo de alterações)"
echo "  - HITL_FULL_DIFF.diff   (diff único completo, só referência)"
echo "  - apply_ardupilot_hitl_series.sh (aplica os .patch)"
echo
echo "Exemplo de uso para aplicar em um clone LIMPO do ArduPilot:"
echo
echo "  git clone https://github.com/ArduPilot/ardupilot.git ardupilot_clean"
echo "  cd ardupilot_clean"
echo "  ${PATCH_DIR}/apply_ardupilot_hitl_series.sh ."
echo
echo "Ou, de fora do repositório:"
echo
echo "  PATCH_DIR=\"${PATCH_DIR}\" ${PATCH_DIR}/apply_ardupilot_hitl_series.sh /caminho/para/ardupilot_clean"
echo "=============================================================="

