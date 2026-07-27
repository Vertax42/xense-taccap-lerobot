#!/usr/bin/env bash
set -euo pipefail

APP_ID="xense-taccap-production-ui"
APP_NAME="XENSE-TACCAP"
APP_COMMENT="Xense TacCap Production UI"

usage() {
    cat <<USAGE
Usage:
  ./install_ui.sh [options]

Options:
  --prefix DIR       Install app files to DIR.
                     Default: \$HOME/.local/share/${APP_ID}
  --no-desktop       Do not create a desktop shortcut.
  --help             Show this help.
USAGE
}

log() {
    printf '==> %s\n' "$*"
}

die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

expand_path() {
    case "$1" in
        "~") printf '%s\n' "$HOME" ;;
        "~/"*) printf '%s/%s\n' "$HOME" "${1#"~/"}" ;;
        *) printf '%s\n' "$1" ;;
    esac
}

quote_desktop_value() {
    local value=$1
    value=${value//\\/\\\\}
    value=${value//\"/\\\"}
    printf '"%s"\n' "$value"
}

escape_desktop_path() {
    local value=$1
    value=${value//\\/\\\\}
    printf '%s\n' "$value"
}

desktop_dir() {
    local dir=""
    if command -v xdg-user-dir >/dev/null 2>&1; then
        dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
    fi
    if [[ -z "$dir" || "$dir" == "$HOME" ]]; then
        dir="$HOME/Desktop"
    fi
    printf '%s\n' "$dir"
}

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_APP_DIR="${SOURCE_ROOT}/${APP_ID}"
INSTALL_DIR="${HOME}/.local/share/${APP_ID}"
CREATE_DESKTOP=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --prefix)
            [[ $# -ge 2 ]] || die "--prefix requires a directory"
            INSTALL_DIR="$(expand_path "$2")"
            shift
            ;;
        --no-desktop)
            CREATE_DESKTOP=0
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            die "unknown option: $1"
            ;;
    esac
    shift
done

if [[ ! -x "${SOURCE_APP_DIR}/${APP_ID}" ]]; then
    die "packaged app not found: ${SOURCE_APP_DIR}/${APP_ID}"
fi

BIN_DIR="${HOME}/.local/bin"
WRAPPER_PATH="${BIN_DIR}/${APP_ID}"
APPLICATIONS_DIR="${XDG_DATA_HOME:-${HOME}/.local/share}/applications"
APP_DESKTOP_FILE="${APPLICATIONS_DIR}/${APP_ID}.desktop"
DESKTOP_FILE="$(desktop_dir)/${APP_NAME}.desktop"
ICON_PATH="${INSTALL_DIR}/_internal/production_ui/xense.png"

log "Installing ${APP_NAME}"
log "Source: ${SOURCE_APP_DIR}"
log "Target: ${INSTALL_DIR}"

TMP_INSTALL_DIR="${INSTALL_DIR}.tmp.$$"
rm -rf "$TMP_INSTALL_DIR"
mkdir -p "$TMP_INSTALL_DIR"
cp -a "${SOURCE_APP_DIR}/." "$TMP_INSTALL_DIR/"
chmod +x "${TMP_INSTALL_DIR}/${APP_ID}"

rm -rf "$INSTALL_DIR"
mv "$TMP_INSTALL_DIR" "$INSTALL_DIR"

mkdir -p "$BIN_DIR"
cat > "$WRAPPER_PATH" <<EOF
#!/usr/bin/env bash
set -euo pipefail
exec "$(printf '%s' "${INSTALL_DIR}/${APP_ID}")" "\$@"
EOF
chmod +x "$WRAPPER_PATH"

mkdir -p "$APPLICATIONS_DIR"
cat > "$APP_DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=${APP_NAME}
Comment=${APP_COMMENT}
Exec=$(quote_desktop_value "$WRAPPER_PATH")
Icon=$(escape_desktop_path "$ICON_PATH")
Terminal=false
Categories=Utility;
StartupNotify=true
EOF
chmod +x "$APP_DESKTOP_FILE"

if [[ "$CREATE_DESKTOP" -eq 1 ]]; then
    mkdir -p "$(dirname "$DESKTOP_FILE")"
    cp "$APP_DESKTOP_FILE" "$DESKTOP_FILE"
    chmod +x "$DESKTOP_FILE"
    if command -v gio >/dev/null 2>&1; then
        gio set "$DESKTOP_FILE" metadata::trusted true >/dev/null 2>&1 || true
    fi
    log "Desktop shortcut: ${DESKTOP_FILE}"
fi

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "$APPLICATIONS_DIR" >/dev/null 2>&1 || true
fi

log "Launcher: ${WRAPPER_PATH}"
log "Application menu entry: ${APP_DESKTOP_FILE}"
log "Done. Start with: ${WRAPPER_PATH}"
