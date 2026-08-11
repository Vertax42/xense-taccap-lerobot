#!/usr/bin/env bash
set -euo pipefail

# Installs a customer machine. The normal path is online: prepare the host,
# then pull the published image from GHCR. The package is public, so the pull
# needs no credentials.
#
# An image archive is the offline fallback, for a machine that cannot reach
# ghcr.io at all. It is used when one is passed explicitly, or when exactly one
# sits next to this script — which is the layout docker/package_customer_delivery.sh
# produces. Neither is required any more: with no archive in sight the script
# pulls instead of failing.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCHIVE_PATH="${1:-${XENSE_IMAGE_ARCHIVE:-}}"
PROXY_URL="${XENSE_PROXY_URL:-}"
IMAGE_REPOSITORY="${XENSE_IMAGE_REPOSITORY:-}"
IMAGE_TAG="${LEROBOT_IMAGE_TAG:-}"
DEFAULT_IMAGE_REPOSITORY="ghcr.io/vertax42/xense-taccap-lerobot"
TEMP_DIR=""
DOCKER_CMD=(docker)

log() {
  printf '[install_customer] %s\n' "$*"
}

fail() {
  printf '[install_customer] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<EOF
Usage: ./install_customer.sh [IMAGE_ARCHIVE]

Installs/checks Docker and NVIDIA Container Toolkit, installs host udev rules,
obtains the container image, and runs a CUDA smoke test.

The image is pulled from ${DEFAULT_IMAGE_REPOSITORY} unless an
archive is given or found next to this script, in which case it is verified
against SHA256SUMS and loaded from disk instead.

Environment:
  XENSE_PROXY_URL          Optional HTTP/HTTPS proxy, e.g. http://127.0.0.1:7897
  XENSE_IMAGE_ARCHIVE      Image archive path when not passed as an argument
  LEROBOT_IMAGE            Override the image repository
  LEROBOT_IMAGE_TAG        Override the image tag (default: latest)
  XENSE_IMAGE_REPOSITORY   Same as LEROBOT_IMAGE, kept for older delivery dirs
EOF
}

cleanup() {
  if [[ -n "${TEMP_DIR}" && -d "${TEMP_DIR}" ]]; then
    rm -rf "${TEMP_DIR}"
  fi
}
trap cleanup EXIT

require_normal_user() {
  if [[ "${EUID}" -eq 0 ]]; then
    fail "Run this script as a normal user with sudo privileges, not as root."
  fi
  command -v sudo >/dev/null 2>&1 || fail "sudo is required."
  sudo -v
}

load_os_release() {
  [[ -r /etc/os-release ]] || fail "Cannot read /etc/os-release."
  # shellcheck disable=SC1091
  . /etc/os-release
  [[ "${ID}" == "ubuntu" || "${ID}" == "debian" ]] || \
    fail "Only Ubuntu and Debian are currently supported; detected ${ID}."
  [[ "$(dpkg --print-architecture)" == "amd64" ]] || \
    fail "Only linux/amd64 hosts are supported."
  DISTRO_ID="${ID}"
  DISTRO_CODENAME="${VERSION_CODENAME:-}"
  [[ -n "${DISTRO_CODENAME}" ]] || fail "VERSION_CODENAME is missing from /etc/os-release."
}

apt_get() {
  local apt_options=(-o Acquire::Retries=10 -o Acquire::http::Timeout=60 -o Acquire::https::Timeout=120)
  if [[ -n "${PROXY_URL}" ]]; then
    apt_options+=(-o Acquire::http::Proxy=false -o "Acquire::https::Proxy=${PROXY_URL}")
  fi
  sudo apt-get "${apt_options[@]}" "$@"
}

fetch_url() {
  local url="$1"
  local destination="$2"
  local curl_args=(--http1.1 --fail --show-error --location --retry 10 --retry-delay 5 --retry-all-errors)
  if [[ -n "${PROXY_URL}" ]]; then
    curl_args+=(--proxy "${PROXY_URL}")
  fi
  curl "${curl_args[@]}" "${url}" --output "${destination}"
}

configure_docker_repository() {
  local key_path="${TEMP_DIR}/docker.asc"
  local source_path="${TEMP_DIR}/docker.sources"

  fetch_url "https://download.docker.com/linux/${DISTRO_ID}/gpg" "${key_path}"
  printf '%s\n' \
    'Types: deb' \
    "URIs: https://download.docker.com/linux/${DISTRO_ID}" \
    "Suites: ${DISTRO_CODENAME}" \
    'Components: stable' \
    'Architectures: amd64' \
    'Signed-By: /etc/apt/keyrings/docker.asc' \
    > "${source_path}"

  sudo install -d -m 0755 /etc/apt/keyrings
  sudo install -m 0644 "${key_path}" /etc/apt/keyrings/docker.asc
  sudo install -m 0644 "${source_path}" /etc/apt/sources.list.d/docker.sources
}

install_docker() {
  if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    log "Docker Engine and Docker Compose are already installed."
    return
  fi

  if command -v docker >/dev/null 2>&1; then
    log "Docker Engine exists; installing only the Docker Compose plugin..."
    apt_get update
    apt_get install -y --no-install-recommends ca-certificates curl gnupg
    configure_docker_repository
    apt_get update
    apt_get install -y docker-compose-plugin
    return
  fi

  log "Installing Docker Engine and Docker Compose plugin..."
  apt_get update
  apt_get install -y --no-install-recommends ca-certificates curl gnupg
  configure_docker_repository
  apt_get update
  apt_get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  sudo systemctl enable --now docker
}

configure_docker_proxy() {
  [[ -n "${PROXY_URL}" ]] || return

  local proxy_conf="${TEMP_DIR}/http-proxy.conf"
  printf '%s\n' \
    '[Service]' \
    "Environment=\"HTTP_PROXY=${PROXY_URL}\"" \
    "Environment=\"HTTPS_PROXY=${PROXY_URL}\"" \
    'Environment="NO_PROXY=localhost,127.0.0.1"' \
    > "${proxy_conf}"

  sudo install -d -m 0755 /etc/systemd/system/docker.service.d
  sudo install -m 0644 "${proxy_conf}" /etc/systemd/system/docker.service.d/http-proxy.conf
  sudo systemctl daemon-reload
}

configure_docker_access() {
  sudo groupadd --force docker
  sudo usermod -aG docker "${USER}"
  if docker info >/dev/null 2>&1; then
    DOCKER_CMD=(docker)
  else
    DOCKER_CMD=(sudo docker)
  fi
}

install_nvidia_container_toolkit() {
  command -v nvidia-smi >/dev/null 2>&1 || \
    fail "nvidia-smi not found. Install a compatible host NVIDIA driver first, reboot, then rerun this script."
  nvidia-smi >/dev/null 2>&1 || \
    fail "The host NVIDIA driver is installed but nvidia-smi failed. Fix the driver before continuing."

  local driver_version
  driver_version="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | sed -n '1p' | tr -d '[:space:]')"
  dpkg --compare-versions "${driver_version}" ge "570.144" || \
    fail "NVIDIA driver ${driver_version} is too old; this image requires driver 570.144 or newer."
  log "Host NVIDIA driver ${driver_version} is compatible."

  if command -v nvidia-ctk >/dev/null 2>&1; then
    log "NVIDIA Container Toolkit is already installed; refreshing Docker/CDI configuration."
  else
    log "Installing NVIDIA Container Toolkit..."
    apt_get update
    apt_get install -y --no-install-recommends ca-certificates curl gnupg

    local gpg_key="${TEMP_DIR}/nvidia-container-toolkit-keyring.gpg"
    local source_list="${TEMP_DIR}/nvidia-container-toolkit.list"
    local raw_key="${TEMP_DIR}/nvidia-container-toolkit.key"
    fetch_url "https://nvidia.github.io/libnvidia-container/gpgkey" "${raw_key}"
    gpg --dearmor --yes --output "${gpg_key}" "${raw_key}"
    fetch_url \
      "https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list" \
      "${source_list}"
    sed -i \
      's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
      "${source_list}"

    sudo install -m 0644 "${gpg_key}" /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    sudo install -m 0644 "${source_list}" /etc/apt/sources.list.d/nvidia-container-toolkit.list
    apt_get update
    apt_get install -y nvidia-container-toolkit nvidia-container-toolkit-base
  fi

  sudo nvidia-ctk runtime configure --runtime=docker
  sudo install -d -m 0755 /etc/cdi
  sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
}

install_udev_rules() {
  local taccap_rules="${TEMP_DIR}/99-taccap-ignore-modemmanager.rules"

  # The gripper MCU is a CH343 (1a86:55d2, CDC-ACM). ModemManager probes every
  # fresh CDC-ACM port with AT commands and holds it open for a few seconds, so
  # a hot-plug followed by an immediate connect() dies with "Device or resource
  # busy" on the serial port. This rule is the permanent fix.
  printf '%s\n' \
    '# TacCap-Gripper CH343 USB serial: keep ModemManager off the device.' \
    'ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"' \
    > "${taccap_rules}"

  sudo install -m 0644 "${taccap_rules}" /etc/udev/rules.d/99-taccap-ignore-modemmanager.rules
  sudo groupadd --force plugdev
  sudo usermod -aG plugdev,video "${USER}"
  sudo udevadm control --reload-rules
  sudo udevadm trigger
}

# The script sits at docker/ in a repository checkout but at the top of a
# delivery directory, so compose's .env is one level up in the first layout and
# alongside in the second. Look in both rather than guessing which one this is.
ENV_LOOKUP_RESULT=""
TAG_EXPLICIT=0

# Returns through a global on purpose: a $(...) capture would run this in a
# subshell, so anything it recorded would be discarded with that subshell.
env_lookup() {
  local key="$1" candidate value
  ENV_LOOKUP_RESULT=""
  for candidate in "${ROOT_DIR}/.env" "${ROOT_DIR}/delivery.env" "${ROOT_DIR}/../.env"; do
    [[ -f "${candidate}" ]] || continue
    value="$(awk -F= -v k="${key}" '$1 == k {print substr($0, index($0, "=") + 1); exit}' "${candidate}")"
    if [[ -n "${value}" ]]; then
      ENV_LOOKUP_RESULT="${value}"
      return
    fi
  done
}

resolve_image_ref() {
  if [[ -z "${IMAGE_REPOSITORY}" ]]; then
    env_lookup LEROBOT_IMAGE
    IMAGE_REPOSITORY="${ENV_LOOKUP_RESULT:-${DEFAULT_IMAGE_REPOSITORY}}"
  fi

  if [[ -n "${IMAGE_TAG}" ]]; then
    TAG_EXPLICIT=1
  else
    env_lookup LEROBOT_IMAGE_TAG
    if [[ -n "${ENV_LOOKUP_RESULT}" ]]; then
      IMAGE_TAG="${ENV_LOOKUP_RESULT}"
      TAG_EXPLICIT=1
    else
      IMAGE_TAG="latest"
    fi
  fi
}

# Auto-detection, not a requirement: zero archives is the normal online case and
# must stay silent. Two or more is genuinely ambiguous — picking one at random
# would install an image the operator did not choose, so that still fails.
detect_archive() {
  if [[ -n "${ARCHIVE_PATH}" ]]; then
    ARCHIVE_PATH="$(realpath "${ARCHIVE_PATH}")"
    return 0
  fi

  local matches=()
  mapfile -t matches < <(find "${ROOT_DIR}" -maxdepth 1 -type f -name 'xense-taccap-lerobot-*.tar' -print | sort)
  case "${#matches[@]}" in
    0) return 1 ;;
    1) ARCHIVE_PATH="${matches[0]}" ;;
    *) fail "Found ${#matches[@]} xense-taccap-lerobot-*.tar files in ${ROOT_DIR}. Pass the one to install as the first argument." ;;
  esac
}

derive_tag_from_archive() {
  # The archive name is the weakest source of a tag: it loses to an explicit
  # LEROBOT_IMAGE_TAG and to one pinned in an env file.
  if [[ "${TAG_EXPLICIT}" -eq 1 ]]; then
    return
  fi

  local archive_name
  archive_name="$(basename "${ARCHIVE_PATH}")"
  if [[ "${archive_name}" == xense-taccap-lerobot-*-linux-amd64.tar ]]; then
    IMAGE_TAG="${archive_name#xense-taccap-lerobot-}"
    IMAGE_TAG="${IMAGE_TAG%-linux-amd64.tar}"
  fi
}

verify_archive() {
  [[ -f "${ARCHIVE_PATH}" ]] || fail "Image archive not found: ${ARCHIVE_PATH}"
  local checksum_file="$(dirname "${ARCHIVE_PATH}")/SHA256SUMS"
  [[ -f "${checksum_file}" ]] || fail "Checksum file not found: ${checksum_file}"
  log "Verifying image archive checksum..."
  (
    cd "$(dirname "${ARCHIVE_PATH}")"
    sha256sum --check "$(basename "${checksum_file}")"
  )
}

load_image_from_archive() {
  local image_ref="${IMAGE_REPOSITORY}:${IMAGE_TAG}"
  log "Loading Docker image from ${ARCHIVE_PATH}"
  "${DOCKER_CMD[@]}" load --input "${ARCHIVE_PATH}"
  # A bundle built before the image was renamed to its GHCR reference carries
  # the bare `xense-taccap-lerobot` name, which no longer matches the default.
  "${DOCKER_CMD[@]}" image inspect "${image_ref}" >/dev/null 2>&1 || \
    fail "$(
      cat <<EOF
Expected image was not found after docker load: ${image_ref}

The archive loaded, but under a different name. \`docker images\` will show it;
if this is an older delivery directory, rerun with:

  XENSE_IMAGE_REPOSITORY=xense-taccap-lerobot ./install_customer.sh
EOF
    )"
}

pull_image() {
  local image_ref="${IMAGE_REPOSITORY}:${IMAGE_TAG}"
  log "Pulling ${image_ref} (this downloads about 21 GB on a first install)"
  # Deliberately `docker pull` rather than `docker compose pull`: this script
  # runs both from a repository checkout and from a delivery directory, and only
  # the image reference is common to both. The package is public, so no login.
  "${DOCKER_CMD[@]}" pull "${image_ref}" || \
    fail "Could not pull ${image_ref}. Check network/proxy, or install offline by passing an image archive."
}

# Published images carry the release here: docker/metadata-action sets
# org.opencontainers.image.version from the tag it pushed. Used only to make the
# "pin your tag" hint name a real version instead of a placeholder.
docker_image_version() {
  local version=""
  version="$("${DOCKER_CMD[@]}" image inspect \
    --format '{{index .Config.Labels "org.opencontainers.image.version"}}' \
    "${IMAGE_REPOSITORY}:${IMAGE_TAG}" 2>/dev/null || true)"
  printf '%s' "${version:-<release>}"
}

verify_image() {
  local image_ref="${IMAGE_REPOSITORY}:${IMAGE_TAG}"
  # The same GPU wiring compose.yaml uses. It deliberately is not `--gpus all`:
  # that only asks for compute+utility, so it would test a configuration the
  # user never runs and would pass on a host where graphics is broken.
  local -a gpu_args=(
    --runtime=nvidia
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all
  )

  log "Running GPU and Python smoke test with ${image_ref}"
  "${DOCKER_CMD[@]}" run --rm "${gpu_args[@]}" \
    --entrypoint python "${image_ref}" \
    -c 'import torch; print("torch:", torch.__version__); print("cuda:", torch.cuda.is_available()); raise SystemExit(0 if torch.cuda.is_available() else 1)'

  # Graphics is a separate driver capability from compute, and it fails
  # separately: CUDA and nvidia-smi keep working while Rerun dies with
  # "Failed to create surface for any enabled backend". Checking it here turns
  # that into an install-time failure with a fix, instead of a puzzle the
  # operator meets the first time they pass --display_data=true.
  log "Checking the NVIDIA Vulkan ICD (Rerun needs it)"
  if ! "${DOCKER_CMD[@]}" run --rm "${gpu_args[@]}" \
      --entrypoint bash "${image_ref}" \
      -c 'test -s /etc/vulkan/icd.d/nvidia_icd.json' 2>/dev/null; then
    fail "$(
      cat <<'EOF'
The container did not receive the NVIDIA Vulkan ICD
(/etc/vulkan/icd.d/nvidia_icd.json). CUDA works, but Rerun and any other
GPU-rendered window will fail with:

    WGPU error: Failed to create surface for any enabled backend

The ICD is not part of the image — the NVIDIA container runtime mounts it from
the host. Check, in this order:

  1. The host has it:      ls /etc/vulkan/icd.d/nvidia_icd.json
     Missing means the driver's graphics half is not installed (a headless or
     compute-only driver). Install the matching libnvidia-gl-<branch> package.

  2. The runtime is registered:   docker info --format '{{json .Runtimes}}'
     Must list "nvidia". If not: sudo nvidia-ctk runtime configure
     --runtime=docker && sudo systemctl restart docker

  3. Regenerate the CDI spec if you use one:
     sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
EOF
    )"
  fi
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    return
  fi

  require_normal_user
  load_os_release
  TEMP_DIR="$(mktemp -d)"
  resolve_image_ref

  # Decide the image source before touching the host, so an ambiguous archive
  # or a bad checksum stops the run before it starts installing packages.
  local offline=0
  if detect_archive; then
    offline=1
    derive_tag_from_archive
    verify_archive
    log "Installing offline from ${ARCHIVE_PATH}"
  else
    log "Installing online from ${IMAGE_REPOSITORY}:${IMAGE_TAG}"
  fi

  install_docker
  configure_docker_proxy
  configure_docker_access
  install_nvidia_container_toolkit
  install_udev_rules
  sudo systemctl restart docker
  configure_docker_access

  if [[ "${offline}" -eq 1 ]]; then
    load_image_from_archive
  else
    pull_image
  fi
  verify_image

  log "Installation completed successfully."
  log "Log out and back in once so docker/video/plugdev group membership takes effect."
  log "Then start the container from this directory with:"
  log "  xhost +SI:localuser:root"
  log "  docker compose run --rm xense-taccap"
  if [[ "${IMAGE_TAG}" == "latest" ]]; then
    log ""
    log "NOTE: this machine is on the floating \`latest\` tag. Before recording"
    log "data, pin it in .env so a later release cannot change the image underneath:"
    log "  LEROBOT_IMAGE_TAG=$(docker_image_version)"
  fi
}

main "$@"
