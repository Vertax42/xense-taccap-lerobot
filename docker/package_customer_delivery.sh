#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_REPOSITORY="${XENSE_IMAGE_REPOSITORY:-xense-taccap-lerobot}"
IMAGE_TAG="${1:-${LEROBOT_IMAGE_TAG:-latest}}"
IMAGE_REF="${IMAGE_REPOSITORY}:${IMAGE_TAG}"
DIST_ROOT="${XENSE_DIST_DIR:-${ROOT_DIR}/dist/customer}"
SAFE_TAG="${IMAGE_TAG//[^a-zA-Z0-9_.-]/-}"
BUNDLE_DIR="${DIST_ROOT}/xense-taccap-lerobot-${SAFE_TAG}-linux-amd64"
ARCHIVE_NAME="xense-taccap-lerobot-${SAFE_TAG}-linux-amd64.tar"
ARCHIVE_PATH="${BUNDLE_DIR}/${ARCHIVE_NAME}"

log() {
  printf '[package_customer_delivery] %s\n' "$*"
}

fail() {
  printf '[package_customer_delivery] ERROR: %s\n' "$*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "Missing required command: $1"
}

usage() {
  cat <<'EOF'
Usage: ./docker/package_customer_delivery.sh [IMAGE_TAG]

Packages xense-taccap-lerobot:<IMAGE_TAG> for offline customer delivery.
IMAGE_TAG defaults to LEROBOT_IMAGE_TAG, then latest.

Environment:
  XENSE_IMAGE_REPOSITORY   Image repository name (default: xense-taccap-lerobot)
  XENSE_DIST_DIR           Output root (default: dist/customer)
  XENSE_FORCE_PACKAGE=1    Allow overwriting an existing image archive
EOF
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    return
  fi

  require_command docker
  require_command sha256sum

  docker image inspect "${IMAGE_REF}" >/dev/null 2>&1 || \
    fail "Docker image not found: ${IMAGE_REF}"

  local image_arch
  image_arch="$(docker image inspect --format '{{.Architecture}}' "${IMAGE_REF}")"
  [[ "${image_arch}" == "amd64" ]] || \
    fail "Expected an amd64 image, got: ${image_arch}"

  mkdir -p "${BUNDLE_DIR}"
  if [[ -e "${ARCHIVE_PATH}" && "${XENSE_FORCE_PACKAGE:-0}" != "1" ]]; then
    fail "Archive already exists: ${ARCHIVE_PATH}. Set XENSE_FORCE_PACKAGE=1 to overwrite it."
  fi

  log "Saving ${IMAGE_REF} to ${ARCHIVE_PATH}"
  docker save --output "${ARCHIVE_PATH}" "${IMAGE_REF}"

  (
    cd "${BUNDLE_DIR}"
    sha256sum "${ARCHIVE_NAME}" > SHA256SUMS
  )

  install -m 0644 "${ROOT_DIR}/compose.yaml" "${BUNDLE_DIR}/compose.yaml"
  install -m 0644 "${ROOT_DIR}/docker/README.md" "${BUNDLE_DIR}/DOCKER_README_ZH.md"
  install -m 0755 "${ROOT_DIR}/docker/install_customer.sh" "${BUNDLE_DIR}/install_customer.sh"
  printf 'LEROBOT_IMAGE_TAG=%s\n' "${IMAGE_TAG}" > "${BUNDLE_DIR}/.env"
  printf 'LEROBOT_IMAGE_TAG=%s\n' "${IMAGE_TAG}" > "${BUNDLE_DIR}/delivery.env"

  log "Customer delivery package is ready:"
  log "  ${BUNDLE_DIR}"
  log "Copy the entire directory to the customer machine, then run:"
  log "  ./install_customer.sh"
}

main "$@"
