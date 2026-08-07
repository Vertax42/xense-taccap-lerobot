#!/usr/bin/env bash
set -euo pipefail

# Pushes a locally built xense-taccap-lerobot image to GHCR.
#
# The normal publishing path is .github/workflows/docker-publish.yml, which
# builds on a hosted runner and uploads over GitHub's own network. This script
# is the manual route: it pushes an image that is already on this machine,
# which is what you want when the workflow is unavailable or when the image you
# want published is the one you just validated against real hardware.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_REPOSITORY="${XENSE_IMAGE_REPOSITORY:-xense-taccap-lerobot}"
REGISTRY="${GHCR_REGISTRY:-ghcr.io}"
PUSH_LATEST=1
IMAGE_TAG=""

log() {
  printf '[push_ghcr] %s\n' "$*"
}

fail() {
  printf '[push_ghcr] ERROR: %s\n' "$*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || fail "Missing required command: $1"
}

usage() {
  cat <<'EOF'
Usage: ./docker/push_ghcr.sh [IMAGE_TAG] [--no-latest]

Pushes the local image xense-taccap-lerobot:<IMAGE_TAG> to GHCR as
ghcr.io/<owner>/xense-taccap-lerobot:<IMAGE_TAG>.

IMAGE_TAG defaults to LEROBOT_IMAGE_TAG, then to the fork version in
pyproject.toml (the part after `+xtac.`).

Options:
  --no-latest              Do not also push the `latest` tag

Environment:
  GHCR_TOKEN               GitHub token used to log in (falls back to
                           GITHUB_TOKEN; skipped entirely if already logged in)
  GHCR_USER                Login username (default: the owner below)
  GHCR_OWNER               GHCR namespace (default: parsed from `origin`)
  GHCR_IMAGE               Full target repository, overriding registry/owner
  GHCR_REGISTRY            Registry host (default: ghcr.io)
  XENSE_IMAGE_REPOSITORY   Local image name (default: xense-taccap-lerobot)
  GHCR_PUSH_RETRIES        Push attempts per tag (default: 3)
EOF
}

# Mirrors the tag resolution in .github/workflows/docker-publish.yml. Keep the
# two in step: a Docker tag cannot contain `+`, so the PEP 440 local version
# `0.5.1+xtac.0.0.3` publishes as `0.0.3`.
derive_tag_from_pyproject() {
  sed -n 's/^version *= *".*+xtac\.\([^"]*\)"/\1/p' "${ROOT_DIR}/pyproject.toml"
}

# GHCR namespaces are lowercase, but the GitHub account (`Vertax42`) is not, so
# a reference built from the remote URL verbatim is rejected at push time.
resolve_owner() {
  local url owner
  url="$(git -C "${ROOT_DIR}" remote get-url origin 2>/dev/null || true)"
  [[ -n "${url}" ]] || fail "No git remote 'origin'; set GHCR_OWNER or GHCR_IMAGE."
  owner="${url##*github.com}"
  owner="${owner#[:/]}"
  owner="${owner%%/*}"
  [[ -n "${owner}" ]] || fail "Could not parse an owner from origin: ${url}"
  printf '%s' "${owner,,}"
}

registry_login() {
  local token="${GHCR_TOKEN:-${GITHUB_TOKEN:-}}"

  if [[ -z "${token}" ]]; then
    # Someone who ran `docker login ghcr.io` earlier does not need a token in
    # the environment, so check the stored credential before refusing.
    if grep -q "\"${REGISTRY}\"" "${HOME}/.docker/config.json" 2>/dev/null; then
      log "Using the existing ${REGISTRY} credential from docker login."
      return
    fi
    fail "$(
      cat <<EOF
No GHCR credential. Either run \`docker login ${REGISTRY}\` first, or export a
token:

  export GHCR_TOKEN=<classic PAT with the write:packages scope>

Create one at https://github.com/settings/tokens (classic). Fine-grained tokens
need the 'Packages: write' permission on the package's owner.
EOF
    )"
  fi

  log "Logging in to ${REGISTRY} as ${GHCR_USER}"
  printf '%s' "${token}" | docker login "${REGISTRY}" --username "${GHCR_USER}" --password-stdin
}

push_tag() {
  local ref="$1"
  local attempts="${GHCR_PUSH_RETRIES:-3}"
  local attempt=1

  # This image is over 20 GB unpacked, so a push from a slow uplink flaking
  # part-way through is routine rather than exceptional. Docker re-pushes only
  # the layers that did not land, so a retry resumes rather than restarts.
  until docker push "${ref}"; do
    if [[ "${attempt}" -ge "${attempts}" ]]; then
      fail "docker push ${ref} failed after ${attempt} attempts"
    fi
    log "Push attempt ${attempt} failed; retrying ${ref}"
    attempt=$((attempt + 1))
    sleep 10
  done
}

main() {
  local arg
  for arg in "$@"; do
    case "${arg}" in
      -h | --help)
        usage
        return
        ;;
      --no-latest)
        PUSH_LATEST=0
        ;;
      -*)
        fail "Unknown option: ${arg}"
        ;;
      *)
        [[ -z "${IMAGE_TAG}" ]] || fail "Unexpected extra argument: ${arg}"
        IMAGE_TAG="${arg}"
        ;;
    esac
  done

  require_command docker
  require_command git

  [[ -n "${IMAGE_TAG}" ]] || IMAGE_TAG="${LEROBOT_IMAGE_TAG:-}"
  [[ -n "${IMAGE_TAG}" ]] || IMAGE_TAG="$(derive_tag_from_pyproject)"
  [[ -n "${IMAGE_TAG}" ]] || fail "Could not resolve an image tag; pass one explicitly."

  local owner target_repository local_ref
  owner="${GHCR_OWNER:-$(resolve_owner)}"
  target_repository="${GHCR_IMAGE:-${REGISTRY}/${owner}/${LOCAL_REPOSITORY}}"
  GHCR_USER="${GHCR_USER:-${owner}}"
  local_ref="${LOCAL_REPOSITORY}:${IMAGE_TAG}"

  # Same two preflight checks as package_customer_delivery.sh: the image has to
  # exist locally, and it has to be the amd64 build the Dockerfile enforces.
  docker image inspect "${local_ref}" >/dev/null 2>&1 ||
    fail "Docker image not found: ${local_ref}. Build it first with \`docker compose build\`."

  local image_arch
  image_arch="$(docker image inspect --format '{{.Architecture}}' "${local_ref}")"
  [[ "${image_arch}" == "amd64" ]] || fail "Expected an amd64 image, got: ${image_arch}"

  registry_login

  local refs=("${target_repository}:${IMAGE_TAG}")
  if [[ "${PUSH_LATEST}" -eq 1 && "${IMAGE_TAG}" != "latest" ]]; then
    refs+=("${target_repository}:latest")
  fi

  local ref
  for ref in "${refs[@]}"; do
    log "Tagging ${local_ref} -> ${ref}"
    docker tag "${local_ref}" "${ref}"
    log "Pushing ${ref} (this uploads several GB on a first push)"
    push_tag "${ref}"
  done

  local digest
  digest="$(docker image inspect --format '{{index .RepoDigests 0}}' "${target_repository}:${IMAGE_TAG}" 2>/dev/null || true)"
  log "Pushed:"
  for ref in "${refs[@]}"; do
    log "  ${ref}"
  done
  if [[ -n "${digest}" ]]; then
    log "Digest: ${digest}"
  fi
  log "Customers pull it by putting this in the delivery directory's .env:"
  log "  LEROBOT_IMAGE=${target_repository}"
  log "  LEROBOT_IMAGE_TAG=${IMAGE_TAG}"
}

main "$@"
