#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${APP_DIR}/.." && pwd)"
PYTHON_BIN="${XENSE_TACCAP_PYTHON:-${PYTHON:-python}}"

cd "${REPO_ROOT}"

if ! "${PYTHON_BIN}" -c "import PySide6" >/dev/null 2>&1; then
    echo "PySide6 is missing in ${PYTHON_BIN}." >&2
    echo "Install it with: ${PYTHON_BIN} -m pip install -r production_ui/requirements.txt" >&2
    exit 1
fi

if ! "${PYTHON_BIN}" -m PyInstaller --version >/dev/null 2>&1; then
    echo "PyInstaller is missing in ${PYTHON_BIN}." >&2
    echo "Install it with: ${PYTHON_BIN} -m pip install pyinstaller" >&2
    exit 1
fi

"${PYTHON_BIN}" -m PyInstaller \
    --noconfirm \
    --clean \
    --windowed \
    --name xense-taccap-production-ui \
    --distpath "${APP_DIR}/dist" \
    --workpath "${APP_DIR}/build" \
    --specpath "${APP_DIR}/build" \
    --paths "${REPO_ROOT}/src" \
    --add-data "${APP_DIR}/xense.png:production_ui" \
    --add-data "${APP_DIR}/ui_config.example.json:production_ui" \
    --hidden-import PySide6.QtCore \
    --hidden-import PySide6.QtGui \
    --hidden-import PySide6.QtWidgets \
    "${APP_DIR}/app.py"

PYSIDE6_PLUGIN_DIR="$("${PYTHON_BIN}" - <<'PY'
from pathlib import Path
import PySide6
print(Path(PySide6.__file__).resolve().parent / "Qt" / "plugins")
PY
)"
PYSIDE6_LIB_DIR="$("${PYTHON_BIN}" - <<'PY'
from pathlib import Path
import PySide6
print(Path(PySide6.__file__).resolve().parent / "Qt" / "lib")
PY
)"
DIST_PLUGIN_DIR="${APP_DIR}/dist/xense-taccap-production-ui/_internal/PySide6/Qt/plugins"
DIST_INTERNAL_DIR="${APP_DIR}/dist/xense-taccap-production-ui/_internal"

if [[ -d "${PYSIDE6_PLUGIN_DIR}" && -d "${DIST_PLUGIN_DIR}" ]]; then
    cp -a "${PYSIDE6_PLUGIN_DIR}/." "${DIST_PLUGIN_DIR}/"
fi

if [[ -d "${PYSIDE6_LIB_DIR}" && -d "${DIST_INTERNAL_DIR}" ]]; then
    cp -a "${PYSIDE6_LIB_DIR}"/libQt6*.so* "${DIST_INTERNAL_DIR}/"
fi

echo "Built: ${APP_DIR}/dist/xense-taccap-production-ui/xense-taccap-production-ui"
