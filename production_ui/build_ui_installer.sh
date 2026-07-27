#!/usr/bin/env bash
set -euo pipefail

APP_ID="xense-taccap-production-ui"

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_APP_DIR="${APP_DIR}/dist/${APP_ID}"
BUILD_DIR="${APP_DIR}/build/ui-installer"
RELEASE_DIR="${APP_DIR}/release"
RUN_BUILD=0

usage() {
    cat <<USAGE
Usage:
  ./build_ui_installer.sh [options]

Options:
  --build            Rebuild the PyInstaller executable first.
  --output DIR       Write release files to DIR.
                     Default: production_ui/release
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

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            RUN_BUILD=1
            ;;
        --output)
            [[ $# -ge 2 ]] || die "--output requires a directory"
            RELEASE_DIR="$2"
            shift
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

if [[ "$RUN_BUILD" -eq 1 || ! -x "${DIST_APP_DIR}/${APP_ID}" ]]; then
    log "Building PyInstaller executable"
    "${APP_DIR}/build_executable.sh"
fi

[[ -x "${DIST_APP_DIR}/${APP_ID}" ]] || die "missing executable: ${DIST_APP_DIR}/${APP_ID}"
[[ -x "${APP_DIR}/install_ui.sh" ]] || die "missing installer script: ${APP_DIR}/install_ui.sh"
[[ -f "${APP_DIR}/installer_stub.sh" ]] || die "missing installer stub: ${APP_DIR}/installer_stub.sh"

platform="ubuntu$(lsb_release -rs 2>/dev/null || echo unknown)-$(uname -m)"
package_name="${APP_ID}-${platform}"
payload_dir="${BUILD_DIR}/payload"
payload_tar="${BUILD_DIR}/${package_name}-payload.tar.gz"
archive_tar="${RELEASE_DIR}/${package_name}.tar.gz"
run_installer="${RELEASE_DIR}/${package_name}.run"

rm -rf "$BUILD_DIR"
mkdir -p "$payload_dir" "$RELEASE_DIR"

log "Preparing payload"
cp -a "$DIST_APP_DIR" "$payload_dir/${APP_ID}"
cp -a "${APP_DIR}/install_ui.sh" "$payload_dir/install_ui.sh"
chmod +x "$payload_dir/install_ui.sh" "$payload_dir/${APP_ID}/${APP_ID}"

log "Writing tar package: ${archive_tar}"
tar -C "$payload_dir" -czf "$archive_tar" .

log "Writing one-click installer: ${run_installer}"
tar -C "$payload_dir" -czf "$payload_tar" .
awk '/^__XENSE_TACCAP_PRODUCTION_UI_ARCHIVE_BELOW__$/ { print; exit } { print }' \
    "${APP_DIR}/installer_stub.sh" > "$run_installer"
cat "$payload_tar" >> "$run_installer"
chmod +x "$run_installer"

log "Done"
printf 'Installer: %s\n' "$run_installer"
printf 'Archive:   %s\n' "$archive_tar"
