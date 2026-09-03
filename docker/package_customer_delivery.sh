#!/usr/bin/env bash
set -euo pipefail

# Internal fallback, not the customer path. Customers install online: the image
# is public on GHCR and docker/install_customer.sh pulls it. This exists for a
# machine that cannot reach ghcr.io at all, and moving 21 GB by hand is the
# reason it is the exception rather than the rule.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Matches compose.yaml's default image name, so this packages whatever
# `docker compose build` just produced.
IMAGE_REPOSITORY="${XENSE_IMAGE_REPOSITORY:-ghcr.io/xenserobotics-ai/xense-taccap-lerobot}"
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
  cat <<EOF
Usage: ./docker/package_customer_delivery.sh [IMAGE_TAG]

Packages an already-built image into a tar bundle for a customer machine that
cannot reach ghcr.io. The normal install is online — see docker/README.md.

IMAGE_TAG defaults to LEROBOT_IMAGE_TAG, then latest. Note that
\`docker compose build\` tags its output \`latest\`, so packaging a release tag
means building it under that tag in the first place:

    LEROBOT_IMAGE_TAG=0.0.4 docker compose build
    LEROBOT_IMAGE_TAG=0.0.4 ./docker/package_customer_delivery.sh

Environment:
  XENSE_IMAGE_REPOSITORY   Image repository name
                           (default: ${IMAGE_REPOSITORY})
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

  # `docker compose build` tags `latest` unless LEROBOT_IMAGE_TAG was set for
  # the build too, which is the usual reason this check trips.
  docker image inspect "${IMAGE_REF}" >/dev/null 2>&1 || \
    fail "Docker image not found: ${IMAGE_REF}. Build it under that tag with \`LEROBOT_IMAGE_TAG=${IMAGE_TAG} docker compose build\`, or retag an existing build."

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
