#!/usr/bin/env bash
set -euo pipefail

MARKER="__XENSE_TACCAP_PRODUCTION_UI_ARCHIVE_BELOW__"
SELF="$0"
TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/xense-taccap-ui-installer.XXXXXX")"

cleanup() {
    rm -rf "$TMP_DIR"
}
trap cleanup EXIT

marker_line="$(awk -v marker="$MARKER" '$0 == marker { print NR; exit }' "$SELF")"
if [[ -z "$marker_line" ]]; then
    printf 'ERROR: installer archive marker not found.\n' >&2
    exit 1
fi

tail -n +"$((marker_line + 1))" "$SELF" | tar -xzf - -C "$TMP_DIR"
exec "$TMP_DIR/install_ui.sh" "$@"
exit 0

__XENSE_TACCAP_PRODUCTION_UI_ARCHIVE_BELOW__
