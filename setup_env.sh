#!/bin/bash

# Check the operating system
OS_NAME=$(uname -s)
OS_VERSION=""

if [[ "$OS_NAME" == "Linux" ]]; then
    if command -v lsb_release &>/dev/null; then
        OS_VERSION=$(lsb_release -rs)
    elif [[ -f /etc/os-release ]]; then
        . /etc/os-release
        OS_VERSION=$VERSION_ID
    fi
    # Tested on Ubuntu 22.04 (jammy) and 24.04 (noble). The host was upgraded
    # 22.04 → 24.04, which bumps the system toolchain to GCC 13 — fine because
    # the hardware SDKs build against the conda cross-compiler, not system GCC.
    if [[ "$OS_VERSION" != "22.04" && "$OS_VERSION" != "24.04" ]]; then
        echo "Warning: This script has only been tested on Ubuntu 22.04 and 24.04"
        echo "Your system is running Ubuntu $OS_VERSION."
        read -p "Do you want to continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Installation cancelled."
            exit 1
        fi
    fi
else
    echo "Unsupported operating system: $OS_NAME"
    exit 1
fi

echo "Operating system check passed: $OS_NAME $OS_VERSION"

# ── System packages ───────────────────────────────────────────────────────────
# The vendor SDKs are compiled here, so a bare Ubuntu install is missing things
# the build needs — and the failure lands much later, deep in a CMake or linker
# error that reads like a bug in the SDK rather than a missing apt package.
# Customers hit this repeatedly; check up front and print the one command that
# fixes it.
#
# Deliberately split: REQUIRED stops the run, RECOMMENDED only warns. v4l-utils
# is not needed to *run* anything — it is how you diagnose a camera that will
# not open (`v4l2-ctl --list-formats-ext`), which on this hardware is the single
# most common bring-up problem, so a host without it is a host that cannot be
# debugged.
check_system_packages() {
    local -a missing_required=() missing_recommended=()

    # command → package that provides it
    local -a required=(
        "cmake:cmake"
        "g++:build-essential"
        "make:build-essential"
        "pkg-config:pkg-config"
        "git:git"
        "curl:curl"
    )
    # Not ffmpeg: torchcodec loads the FFmpeg *shared libraries*, which the conda
    # env supplies, so a host without the system binary works fine — warning
    # about it would be a false alarm, and false alarms teach people to skip the
    # real ones.
    local -a recommended=(
        "v4l2-ctl:v4l-utils"
    )

    local entry cmd pkg
    for entry in "${required[@]}"; do
        cmd="${entry%%:*}"; pkg="${entry##*:}"
        command -v "$cmd" &>/dev/null || missing_required+=("$pkg")
    done
    for entry in "${recommended[@]}"; do
        cmd="${entry%%:*}"; pkg="${entry##*:}"
        command -v "$cmd" &>/dev/null || missing_recommended+=("$pkg")
    done

    # No libudev-dev / libusb-1.0-0-dev check. Nothing here compiles against
    # either header: taccap-gripper's only find_package is Threads, the pico4
    # bindings link the SDK from the .deb, and both libraries are reached at
    # *runtime* through prebuilt wheels — pyrealsense2 NEEDs libudev.so.1 and
    # libusb-1.0.so.0, and pyudev (under xensesdk) dlopens libudev by name,
    # which cameras/xense/camera_xense.py already resolves via ldconfig.
    # Those come from libudev1 and libusb-1.0-0, which are base packages, not
    # the -dev ones this used to demand.
    #
    # libusb-0.1 IS checked, as a warning — the prebuilt camera stack loads
    # libusb-0.1.so.4 at runtime, and after a kernel upgrade a stale or absent
    # libusb is the usual reason cameras stop connecting. It is not fatal here
    # because nothing *builds* against it: the install would succeed and then
    # fail at connect(), so say it now rather than on the rig. `libusb-dev` is
    # the package that carries the runtime (via libusb-0.1-4) plus headers.
    if ! ldconfig -p 2>/dev/null | grep -q 'libusb-0\.1\.so\.4'; then
        echo ""
        echo "  WARNING: libusb-0.1.so.4 not found — the cameras will not connect."
        echo "    sudo apt update && sudo apt install -y libusb-dev"
        echo "  (Kernel upgrades need a matching libusb; a stale one breaks camera"
        echo "   enumeration long after this script has finished successfully.)"
    fi

    # De-duplicate (build-essential is named twice above).
    local -a uniq_required=() uniq_recommended=()
    mapfile -t uniq_required < <(printf '%s\n' "${missing_required[@]}" | sort -u | sed '/^$/d')
    mapfile -t uniq_recommended < <(printf '%s\n' "${missing_recommended[@]}" | sort -u | sed '/^$/d')

    if [[ ${#uniq_recommended[@]} -gt 0 ]]; then
        echo ""
        echo "  NOTE: missing diagnostic tools — install them before you need them:"
        echo "    sudo apt install -y ${uniq_recommended[*]}"
        echo "  (v4l-utils is what you use to work out why a camera will not open.)"
    fi

    if [[ ${#uniq_required[@]} -gt 0 ]]; then
        echo ""
        echo "ERROR: the hardware SDK build needs system packages that are not installed:"
        echo "    sudo apt install -y ${uniq_required[*]}"
        echo ""
        echo "Install them and re-run this script. Continuing would fail later inside"
        echo "CMake or the linker, where the cause is much harder to see."
        exit 1
    fi

    echo "System package check passed."
}

check_system_packages

# Large PyPI wheels such as torch can exceed uv's default 30s timeout on
# slower links. Allow callers to override, but use a safer default for setup.
export UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-300}"

# Resolve script directory so files can be referenced reliably
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
PROJECT_ROOT="$SCRIPT_DIR"

# Conda environment yaml file (canonical name)
CONDA_ENV_FILE="$SCRIPT_DIR/conda_environment.yaml"
if [[ ! -f "$CONDA_ENV_FILE" ]]; then
    echo "Error: conda environment yaml not found: $CONDA_ENV_FILE"
    exit 1
fi

# ── Conda environment creation ────────────────────────────────────────────────

create_environment() {
    local CONDA_CMD=$1
    local ENV_NAME=$2

    # Deactivate current environment if any (use conda deactivate for both conda and mamba)
    conda deactivate 2>/dev/null || true

    # Remove existing environment if it exists
    if $CONDA_CMD env list | grep -q "^$ENV_NAME "; then
        echo "Removing existing environment '$ENV_NAME'..."
        if ! $CONDA_CMD env remove -n "$ENV_NAME" -y; then
            echo "Error: Failed to remove existing environment '$ENV_NAME'."
            return 1
        fi
    fi

    # Create new environment from conda_environment.yaml
    if ! $CONDA_CMD env create -f "$CONDA_ENV_FILE" -n "$ENV_NAME"; then
        echo "Error: Failed to create $CONDA_CMD environment '$ENV_NAME' from: $CONDA_ENV_FILE"
        return 1
    fi

    echo "$CONDA_CMD environment '$ENV_NAME' created from: $CONDA_ENV_FILE"

    echo -e "[INFO] Created $CONDA_CMD environment named '$ENV_NAME'.\n"
    echo -e "\t\t1. To activate the environment, run:                $CONDA_CMD activate $ENV_NAME"
    echo -e "\t\t2. To install all dependencies, run:                bash $SCRIPT_NAME --install"
    echo -e "\t\t3. To deactivate the environment, run:              conda deactivate"
    echo -e "\n"
}

# ── Shared helpers for hardware installation ──────────────────────────────────

# conda installs a udev/ directory that confuses ctypes.util.find_library("udev"),
# causing pyudev/xensesdk to crash with "OSError: .../lib/udev: Is a directory".
fix_udev_discovery() {
    if [[ -d "${CONDA_PREFIX}/lib/udev" ]]; then
        echo "[udev] Renaming ${CONDA_PREFIX}/lib/udev to avoid pyudev clash..."
        local target="${CONDA_PREFIX}/lib/udev.rules.d"
        [[ -e "$target" ]] && target="${target}.bak.$(date +%s)"
        mv "${CONDA_PREFIX}/lib/udev" "$target" || true
    fi
    if [[ -e "${CONDA_PREFIX}/lib/libudev.so.1" && ! -e "${CONDA_PREFIX}/lib/libudev.so" ]]; then
        ln -s libudev.so.1 "${CONDA_PREFIX}/lib/libudev.so" || true
    fi
}

# TorchCodec follows an explicit compatibility matrix with PyTorch rather than
# a generic semver rule. Keep the expected version here so setup_env.sh can
# validate the installed wheel against the active torch release.
expected_torchcodec_version() {
    case "$1" in
        2.10) echo "0.10.0" ;;
        2.9) echo "0.9.1" ;;
        2.8) echo "0.7.0" ;;
        2.7) echo "0.5" ;;
        2.6) echo "0.2.1" ;;
        2.5) echo "0.1.1" ;;
        2.4) echo "0.0.3" ;;
        *) echo "" ;;
    esac
}

get_torch_major_minor() {
    python - <<'PY'
import re

try:
    import torch
except Exception:
    print("")
    raise SystemExit(0)

match = re.match(r"^(\d+)\.(\d+)", torch.__version__)
print(".".join(match.groups()) if match else "")
PY
}

version_matches_expected_release() {
    python - "$1" "$2" <<'PY'
import sys

from packaging.version import Version

actual, expected = sys.argv[1], sys.argv[2]

try:
    actual_release = Version(actual.split("+", 1)[0]).release
    expected_release = Version(expected).release
except Exception:
    print("no")
    raise SystemExit(0)

print("yes" if actual_release == expected_release else "no")
PY
}

ensure_python_package_min_version() {
    local package_name=$1
    local min_version=$2

    local current_version
    current_version="$(python - "$package_name" <<'PY'
import sys
from importlib.metadata import PackageNotFoundError, version

package_name = sys.argv[1]

try:
    print(version(package_name))
except PackageNotFoundError:
    print("")
PY
)"

    if [[ -z "$current_version" ]]; then
        echo "[python] Installing missing package: $package_name>=$min_version"
        uv pip install --upgrade "$package_name>=$min_version"
        return
    fi

    local is_compatible
    is_compatible="$(python - "$current_version" "$min_version" <<'PY'
import sys

from packaging.version import Version

current_version, min_version = sys.argv[1], sys.argv[2]
print("yes" if Version(current_version) >= Version(min_version) else "no")
PY
)"

    if [[ "$is_compatible" == "yes" ]]; then
        echo "[python] Keeping $package_name==$current_version"
    else
        echo "[python] Upgrading $package_name from $current_version to >=$min_version"
        uv pip install --upgrade "$package_name>=$min_version"
    fi
}

# ── Hardware module: Pico4 ────────────────────────────────────────────────────

# Install the XenseVR PC Service daemon from its .deb. The package (~100 MB) is
# the binary service the Pico4 teleop/tracker talks to (installs to
# /opt/apps/roboticsservice); it is NOT vendored in-repo. By default,
# setup_env.sh downloads the matching-arch asset directly from the GitHub release
# ($XENSEVR_DEB_URL overrides the default release URL).
# $XENSEVR_DEB remains an explicit path override for offline/patched builds; no
# implicit dist/ or ~/Downloads cache lookup is done.
# Non-fatal: a failed/absent .deb only warns (the Python SDK still builds; the
# service can be installed later). Idempotent: same installed version is skipped.
install_xensevr_service() {
    echo ""
    echo "── XenseVR PC Service (.deb daemon) ──"

    local ARCH DEB_VER DEB_URL DEB
    ARCH="$(dpkg --print-architecture 2>/dev/null || echo amd64)"   # amd64 | arm64
    DEB_VER="0.2.1"

    # 0.2.x ships amd64 only. Without this, an arm64 host would build a URL
    # for an asset that does not exist and fail with a bare 404 — pinning it
    # to the last release that has an arm64 build is both truthful and
    # working, at the cost of no Pico camera support there.
    if [[ "$ARCH" == "arm64" && -z "${XENSEVR_DEB_URL:-}${XENSEVR_DEB:-}" ]]; then
        DEB_VER="0.1.0"
        echo "  NOTE: arm64 detected — pinning to v${DEB_VER}, the newest release with"
        echo "        an arm64 asset. 0.2.x (Pico camera support) is amd64-only, and"
        echo "        XenseVR-PC-Service dropped its aarch64 tree, so building one is"
        echo "        no longer a matter of running a script in that repository."
    fi

    DEB_URL="${XENSEVR_DEB_URL:-https://github.com/Vertax42/XenseVR-PC-Service/releases/download/v${DEB_VER}/XenseVR-PC-Service_${DEB_VER}_${ARCH}.deb}"

    local WANT INSTALLED STATUS
    # Check the status, not just the version. `dpkg -r` leaves the package in
    # `deinstall ok config-files`, and dpkg-query still reports its version
    # there — so matching on the version alone declares a daemon installed
    # when /opt/apps/roboticsservice is gone, skips the install, and fails
    # later somewhere far less obvious.
    STATUS="$(dpkg-query -W -f='${Status}' xensevr-pc-service 2>/dev/null || true)"
    INSTALLED="$(dpkg-query -W -f='${Version}' xensevr-pc-service 2>/dev/null || true)"

    # Decide before downloading. DEB_VER already names the version this script
    # would install, so fetching 116 MB only to read the same number back out of
    # the package is waste on every re-run of --install. An explicit override is
    # exempt: it can be a different build carrying the same version, and only
    # the file itself can say.
    if [[ -z "${XENSEVR_DEB:-}${XENSEVR_DEB_URL:-}" && "$STATUS" == "install ok installed" && "$INSTALLED" == "$DEB_VER" ]]; then
        echo "  xensevr-pc-service $INSTALLED already installed — skipping."
        return 0
    fi

    DEB="${XENSEVR_DEB:-}"
    if [[ -n "$DEB" ]]; then
        if [[ ! -f "$DEB" ]]; then
            echo "  WARN: XENSEVR_DEB points to a missing file: $DEB"
            echo "  WARN: skipping service install."
            return 0
        fi
        echo "  Using explicit .deb override: $DEB"
    else
        DEB="${TMPDIR:-/tmp}/XenseVR-PC-Service_${DEB_VER}_${ARCH}.deb"
        if [[ -f "$DEB" ]] && dpkg-deb -f "$DEB" Version >/dev/null 2>&1; then
            echo "  Reusing previously downloaded $DEB"
        else
            # An unreadable leftover is worth less than the bandwidth to replace it.
            rm -f "$DEB"
            echo "  Downloading ${ARCH} asset from:"
            echo "    $DEB_URL"
            # ~116 MB. Retry and resume rather than losing the whole transfer to
            # one dropped connection — this package is now the only source of the
            # client SDK the Python bindings link against, so a failure here is
            # not something a local build can paper over.
            #
            # Staged through .part deliberately: handing a *complete* file to
            # `curl -C -` asks for a range that starts at EOF, the server answers
            # 416, and --retry-all-errors then retries that five times before
            # giving up. Only a partial transfer is ever resumed.
            local RETRY=(--retry 5 --retry-delay 2)
            if curl --help all 2>/dev/null | grep -q -- '--retry-all-errors'; then
                RETRY+=(--retry-all-errors)   # curl >= 7.71; covers a mid-transfer drop
            fi
            if ! curl -fL "${RETRY[@]}" -C - "$DEB_URL" -o "$DEB.part"; then
                echo "  WARN: download failed — skipping service install."
                echo "  The partial file is kept at $DEB.part — re-run to resume it."
                echo "  Or get it from https://github.com/Vertax42/XenseVR-PC-Service/releases"
                echo "  then: sudo dpkg -i XenseVR-PC-Service_*_${ARCH}.deb"
                return 0
            fi
            mv "$DEB.part" "$DEB"
        fi
    fi

    WANT="$(dpkg-deb -f "$DEB" Version 2>/dev/null)"
    if [[ "$STATUS" == "install ok installed" && "$INSTALLED" == "$WANT" ]]; then
        echo "  xensevr-pc-service $INSTALLED already installed — skipping."
        return 0
    fi

    echo "  Installing xensevr-pc-service ${WANT:-?} from: $DEB"
    sudo dpkg -i "$DEB" || sudo apt-get install -f -y
    echo "  Installed to /opt/apps/roboticsservice. Start the service with:"
    echo "    /opt/apps/roboticsservice/runService.sh"
}

install_pico4() {
    echo ""
    echo "══════════════════════════════════════════"
    echo " XenseVR-PC-Service  →  xensevr_pc_service_sdk"
    echo "══════════════════════════════════════════"

    local PYBIND_DIR="$PROJECT_ROOT/src/lerobot/teleoperators/pico4/xensevr-pc-service-pybind"

    # Install the PC Service daemon (.deb) the Python SDK will talk to. It also
    # ships the client SDK the bindings link against, which is why there is no
    # longer a XenseVR-PC-Service submodule to compile: the .deb's
    # libPXREARobotSDK.so is the same artifact that build used to produce, and
    # keeping a 31 MiB checkout of prebuilt gRPC archives around to rebuild it
    # cost more than it was worth. The trade is that an SDK source fix now has
    # to travel through a .deb release rather than through `--install`.
    install_xensevr_service

    # Take the header and .so straight out of the installed package. SDK/x64 on
    # amd64, SDK/arm64 on arm64 — the names come from the service's own install
    # step, not from dpkg's architecture strings.
    local SDK_ROOT="/opt/apps/roboticsservice/SDK"
    local SDK_LIBDIR
    case "$(dpkg --print-architecture 2>/dev/null || echo amd64)" in
        arm64) SDK_LIBDIR="$SDK_ROOT/arm64" ;;
        *)     SDK_LIBDIR="$SDK_ROOT/x64" ;;
    esac

    if [[ ! -f "$SDK_ROOT/include/PXREARobotSDK.h" || ! -f "$SDK_LIBDIR/libPXREARobotSDK.so" ]]; then
        echo "ERROR: the XenseVR PC Service SDK is not on this host."
        echo "  Expected:"
        echo "    $SDK_ROOT/include/PXREARobotSDK.h"
        echo "    $SDK_LIBDIR/libPXREARobotSDK.so"
        echo "  Both come from the xensevr-pc-service .deb, which the step above"
        echo "  installs. If that step warned about a failed download, fix that"
        echo "  first — the bindings cannot be built without it."
        return 1
    fi

    # nlohmann is not in the .deb; it comes from conda (nlohmann_json in
    # conda_environment.yaml), and the pybind CMakeLists find_package()s it.
    mkdir -p "$PYBIND_DIR/include" "$PYBIND_DIR/lib"
    cp "$SDK_ROOT/include/PXREARobotSDK.h" "$PYBIND_DIR/include/"
    cp "$SDK_LIBDIR/libPXREARobotSDK.so" "$PYBIND_DIR/lib/"

    # Build and install the Python bindings
    pushd "$PYBIND_DIR" > /dev/null
    rm -rf build *.egg-info
    uv pip uninstall xensevr_pc_service_sdk 2>/dev/null || true
    uv pip install . --no-build-isolation
    popd > /dev/null

    echo "[pico4] Done. Verify with: python -c 'import xensevr_pc_service_sdk; print(xensevr_pc_service_sdk)'"
}

# ── Hardware module: Xense ────────────────────────────────────────────────────

install_xense() {
    echo ""
    echo "══════════════════════════════════════════"
    echo " xensesdk (PyPI)"
    echo "══════════════════════════════════════════"

    fix_udev_discovery

    # Install xensesdk runtime deps explicitly because the wheel is installed
    # with --no-deps below (keeps the shared Robostack env's numpy/opencv/
    # cryptography pins from being disturbed by the wheel's own constraints).
    # xensesdk 2.1.1 requires cypack/ormsgpack/cyclonedds-nightly as
    # mandatory runtime deps — cypack is the FIRST import in xensesdk/__init__.py
    # and ormsgpack/cyclonedds are needed by the ezros layer — so they must be
    # listed here or `import xensesdk` fails with ModuleNotFoundError.
    uv pip install \
        "numpy>=1.26.4,<2.3.0" \
        "opencv-python>=4.10" \
        "pillow>=12.0" \
        "cryptography>=46.0" \
        "PyYAML>=6.0" \
        "h5py>=3.10" \
        "scipy>=1.14" \
        "lz4>=4.0" \
        "psutil>=7.0" \
        "spdlog>=2.0" \
        "cypack>=0.1.2" \
        "ormsgpack>=1.11.0" \
        "cyclonedds-nightly==2025.7.29" \
        "pyudev; platform_system=='Linux'"
    # Install the validated xensesdk 2.1.1 release by name. The
    # published wheel bundles the patched libxense_c.so flash reader
    # (concurrent-connect EBADF fix), so no separate xense_xu / pyxensexu build
    # is needed. --no-deps keeps the shared Robostack env's numpy/opencv/
    # cryptography pins: xensesdk's own metadata hard-pins cryptography==43.0.3 /
    # numpy<=2.2.4, which would otherwise fight the env (the explicit runtime
    # deps installed above already cover what it needs at runtime).
    # For an offline or patched build, set XENSESDK_WHEEL=/path/to/xensesdk-*.whl
    # to install that file instead of pulling from PyPI.
    if [[ -n "${XENSESDK_WHEEL:-}" && -f "${XENSESDK_WHEEL}" ]]; then
        echo "[xense] Installing xensesdk from override wheel: $XENSESDK_WHEEL"
        uv pip install --no-deps --force-reinstall "$XENSESDK_WHEEL"
    else
        echo "[xense] Installing xensesdk from PyPI..."
        uv pip install --no-deps --upgrade "xensesdk==2.1.1"
    fi
    # xensesdk requires a specific av version
    uv pip install av==15.1.0

    local PY_VER
    PY_VER="$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"

    # onnxruntime-gpu uses dlopen() and ignores LD_LIBRARY_PATH — patch its RPATH.
    local ONNX_SO="${CONDA_PREFIX}/lib/python${PY_VER}/site-packages/onnxruntime/capi/libonnxruntime_providers_cuda.so"
    if [[ -f "$ONNX_SO" ]]; then
        echo "[xense] Fixing onnxruntime-gpu RPATH..."
        uv pip install patchelf
        patchelf --set-rpath "${CONDA_PREFIX}/lib" "$ONNX_SO"
        echo "[xense] RPATH set: $(patchelf --print-rpath "$ONNX_SO")"
    fi

    # OpenCV's bundled Qt plugin causes XCB errors inside conda envs.
    local QXCB="${CONDA_PREFIX}/lib/python${PY_VER}/site-packages/cv2/qt/plugins/platforms/libqxcb.so"
    if [[ -f "$QXCB" ]]; then
        echo "[xense] Removing OpenCV bundled Qt plugin: $QXCB"
        rm -f "$QXCB"
    fi

    if python - <<'PY'
from xensesdk.flash.linux_backend import LinuxFlashBackend
raise SystemExit(0 if LinuxFlashBackend().available else 1)
PY
    then
        echo "[xense] xensesdk flash backend (libxense_c) is available."
    else
        echo "[xense] ERROR: xensesdk flash backend verification failed."
        return 1
    fi

    echo "[xense] Done. Verify with: python -c 'import xensesdk; print(xensesdk)'"
}

# ── Hardware module: TacCap-Gripper (taccap_gripper UMI device) ───────────────

install_taccap() {
    echo ""
    echo "══════════════════════════════════════════"
    echo " TacCap-Gripper SDK  →  xense.taccap"
    echo "══════════════════════════════════════════"

    local SDK_DIR="$PROJECT_ROOT/third_party/taccap-gripper"
    if [[ ! -f "$SDK_DIR/pyproject.toml" ]]; then
        echo "ERROR: $SDK_DIR not found (submodule not initialized)."
        echo "  Run: git submodule update --init --recursive third_party/taccap-gripper"
        return 1
    fi
    # The wrist camera path uses libudev via pyudev — same find_library("udev")
    # pitfall as xensesdk.
    fix_udev_discovery

    # taccap's C++ core needs the OpenCV C++ headers (Camera component) and
    # spdlog (shared logger), which the lerobot env does NOT otherwise ship (it
    # carries only the cv2 wheel, not libopencv C++). These are declared in
    # conda_environment.yaml so the env-create solves them together with the
    # robostack ROS stack (one coherent solve) — this fallback normally no-ops
    # because the guard below finds the headers already present.
    # cmake/ninja/gcc14 are already present. libopencv is the C++ runtime only —
    # it does not disturb the pip-installed opencv-python. (libxensesdk and its
    # eigen/openssl/zlib/nlohmann_json deps were dropped in taccap-gripper 0.1.4.)
    if ! compgen -G "${CONDA_PREFIX}/include/opencv4/opencv2/opencv.hpp" > /dev/null 2>&1 || \
       [[ ! -f "${CONDA_PREFIX}/include/spdlog/spdlog.h" ]]; then
        echo "[taccap] Installing C++ build deps (libopencv, spdlog, pkg-config, make)..."
        # Pin libopencv to 4.12 to match conda_environment.yaml. Include
        # robostack-staging so the solve coexists with the ROS stack.
        ${CONDA_CMD:-mamba} install -c robostack-staging -c conda-forge -y \
            "libopencv=4.12" "spdlog=1.14.1" pkg-config make
    fi

    # --no-build-isolation (used below to keep the build on the env's cmake/ninja
    # instead of a freshly fetched PyPI one) requires the PEP 517 backend and
    # pybind11 to be present in-env — pre-install them.
    uv pip install "scikit-build-core>=0.10" "pybind11>=2.12"

    # scikit-build-core + pybind11 build. LIBRARY_PATH gives the conda cross-
    # compiler the conda libdir (it does not search /usr/lib). --no-deps keeps the
    # shared env's numpy/opencv pins; --no-build-isolation uses the env's
    # cmake/ninja instead of a freshly fetched PyPI one.
    LIBRARY_PATH="${CONDA_PREFIX}/lib${LIBRARY_PATH:+:$LIBRARY_PATH}" \
        uv pip install -e "$SDK_DIR" --no-deps --no-build-isolation

    echo "[taccap] Done. Verify with: python -c 'import xense.taccap; print(xense.taccap.__file__)'"
}

# ── Argument parsing ──────────────────────────────────────────────────────────

# Check if an environment name is provided
if [[ -n "$2" ]]; then
    ENV_NAME="$2"
else
    ENV_NAME="xense-taccap"
fi

# Check if the --conda parameter is passed
if [[ "$1" == "--conda" ]]; then
    # Initialize conda
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        . "$HOME/miniconda3/etc/profile.d/conda.sh"
    elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
        . "$HOME/anaconda3/etc/profile.d/conda.sh"
    else
        echo "Conda initialization script not found. Please install Miniconda3 or Anaconda3 or Miniforge3."
        exit 1
    fi
    create_environment "conda" "$ENV_NAME" || exit 1

# Check if the --mamba parameter is passed
elif [[ "$1" == "--mamba" ]]; then
    # Initialize mamba (miniforge)
    if [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ]; then
        . "$HOME/miniforge3/etc/profile.d/conda.sh"
    elif [ -f "$HOME/mambaforge/etc/profile.d/conda.sh" ]; then
        . "$HOME/mambaforge/etc/profile.d/conda.sh"
    else
        echo "Mamba initialization script not found. Please install Miniforge3 or Mambaforge."
        exit 1
    fi
    # Also source mamba.sh if available for full mamba support
    if [ -f "$HOME/miniforge3/etc/profile.d/mamba.sh" ]; then
        . "$HOME/miniforge3/etc/profile.d/mamba.sh"
    elif [ -f "$HOME/mambaforge/etc/profile.d/mamba.sh" ]; then
        . "$HOME/mambaforge/etc/profile.d/mamba.sh"
    fi
    create_environment "mamba" "$ENV_NAME" || exit 1

# Check if the --install parameter is passed
elif [[ "$1" == "--install" ]]; then
    # Get the currently activated conda environment name
    if [[ -z "${CONDA_DEFAULT_ENV}" ]]; then
        echo "Error: No conda/mamba environment is currently activated."
        echo "Please activate an environment first with: conda/mamba activate <env_name>"
        exit 1
    fi
    ENV_NAME=${CONDA_DEFAULT_ENV}

    # Detect the manager that owns the *currently active* environment, and drive
    # every env operation below (env update + the taccap `install` in
    # install_taccap) with it. mamba is preferred whenever the active install
    # ships it — it is faster and is the intended solver for the robostack stack
    # this env is built on — with conda as the fallback.
    #
    # We key off MAMBA_EXE/CONDA_EXE (exported by the shell hook of the active
    # base) rather than a bare `-f ~/miniforge3/...` check, which mis-detects a
    # mambaforge install (`~/mambaforge`) as conda and force-picks mamba whenever
    # ~/miniforge3 merely exists — even for an active anaconda3 env.
    if [[ -n "${MAMBA_EXE:-}" ]] && command -v mamba &>/dev/null; then
        CONDA_CMD="mamba"                                  # miniforge / mambaforge
    elif [[ -n "${CONDA_EXE:-}" && -x "$(dirname "$CONDA_EXE")/mamba" ]]; then
        CONDA_CMD="mamba"                                  # mamba beside active conda
    elif command -v mamba &>/dev/null; then
        CONDA_CMD="mamba"
    elif command -v conda &>/dev/null; then
        CONDA_CMD="conda"
    else
        echo "[ERROR] Neither 'mamba' nor 'conda' is available on PATH."
        echo "        Activate your environment (conda/mamba activate <env>) and retry."
        exit 1
    fi
    echo "[INFO] Using '$CONDA_CMD' to manage environment '$ENV_NAME'."

    echo "[INFO] Updating conda environment '$ENV_NAME' from: $CONDA_ENV_FILE"
    if ! $CONDA_CMD env update -f "$CONDA_ENV_FILE" -n "$ENV_NAME"; then
        echo "[ERROR] Failed to update conda environment '$ENV_NAME' from: $CONDA_ENV_FILE"
        exit 1
    fi

    echo -e "\n[INFO] Conda dependencies installed/updated for '$ENV_NAME'.\n"
    uv pip install --upgrade pip
    # Ensure editable installs (PEP 660) work without downgrading newer compatible packages.
    ensure_python_package_min_version setuptools 71.0.0
    ensure_python_package_min_version wheel 0.40.0

    # Workaround for Python ctypes.util.find_library("udev") on conda envs:
    # Hacking the udev library discovery to avoid issues with pyudev/xensesdk.
    # If $CONDA_PREFIX/lib/udev exists as a directory, Python may return that directory as the "udev" library,
    # causing pyudev/xensesdk to crash with: "OSError: .../lib/udev: Is a directory".
    fix_udev_discovery

    # evdev (pulled in by pynput) generates ecodes.c from /usr/include/linux/ at
    # build time, then compiles it with the conda cross-compiler which uses its
    # own sysroot. Ubuntu's linux-libc-dev now ships KEY_LINK_PHONE (added in
    # Linux 6.14), so evdev references it in ecodes.c, but the conda sysroot's
    # older kernel headers lack it.  Patch the sysroot header in place so the
    # conda compiler can find the constant without mixing incompatible glibc headers.
    _SYSROOT_CODES="${CONDA_PREFIX}/x86_64-conda-linux-gnu/sysroot/usr/include/linux/input-event-codes.h"
    if [[ -f "$_SYSROOT_CODES" ]] && ! grep -q "KEY_LINK_PHONE" "$_SYSROOT_CODES"; then
        echo "[INFO] Patching conda sysroot: adding KEY_LINK_PHONE to input-event-codes.h"
        printf '\n#ifndef KEY_LINK_PHONE\n#define KEY_LINK_PHONE 0x1bf  /* AL Phone Syncing, Linux 6.14 */\n#endif\n' >> "$_SYSROOT_CODES"
    fi

    echo "[INFO] Installing Lerobot from pyproject.toml"
    if uv pip install -e .; then
        echo "[INFO] Lerobot installed successfully!"
    else
        echo "[ERROR] Lerobot installation failed. See the error output above."
        exit 1
    fi

    # Verify critical package versions
    echo "[INFO] Verifying package versions..."
    TORCHCODEC_VER=$(python -c "import torchcodec; print(torchcodec.__version__)" 2>/dev/null || echo "NOT INSTALLED")
    AV_VER=$(python -c "import av; print(av.__version__)" 2>/dev/null || echo "NOT INSTALLED")
    TORCH_VER=$(python -c "import torch; print(torch.__version__)" 2>/dev/null || echo "NOT INSTALLED")
    TORCH_MAJOR_MINOR=$(get_torch_major_minor)
    EXPECTED_TORCHCODEC_VER=$(expected_torchcodec_version "$TORCH_MAJOR_MINOR")
    echo "  - torch: $TORCH_VER"
    if [[ -n "$EXPECTED_TORCHCODEC_VER" ]]; then
        echo "  - torchcodec: $TORCHCODEC_VER (should be $EXPECTED_TORCHCODEC_VER for torch $TORCH_MAJOR_MINOR)"
    else
        echo "  - torchcodec: $TORCHCODEC_VER (no compatibility override defined for torch $TORCH_VER)"
    fi
    echo "  - av (pyav): $AV_VER (should be 15.1.0)"

    if [[ -n "$EXPECTED_TORCHCODEC_VER" ]]; then
        if [[ "$(version_matches_expected_release "$TORCHCODEC_VER" "$EXPECTED_TORCHCODEC_VER")" != "yes" ]]; then
            echo "[WARN] torchcodec version mismatch! Expected $EXPECTED_TORCHCODEC_VER for torch $TORCH_MAJOR_MINOR, got $TORCHCODEC_VER"
            uv pip install "torchcodec==$EXPECTED_TORCHCODEC_VER" --force-reinstall
        fi
    else
        echo "[WARN] Skipping torchcodec pin verification for unsupported torch version: $TORCH_VER"
    fi

    if [[ "$AV_VER" != "15.1.0" ]]; then
        echo "[WARN] av (pyav) version mismatch! Expected 15.1.0, got $AV_VER"
        uv pip install av==15.1.0 --force-reinstall
    fi

    echo ""
    echo "[INFO] Installing hardware SDK bindings..."
    echo ""

    ( install_pico4 ) || echo "[WARN] pico4 installation skipped or failed (see above)"
    install_xense     || echo "[WARN] xense installation skipped or failed (see above)"
    install_taccap    || echo "[WARN] taccap installation skipped or failed (see above)"


    # ── Post-install verification ────────────────────────────────────────────
    echo ""
    echo "══════════════════════════════════════════"
    echo " Post-install verification"
    echo "══════════════════════════════════════════"
    _VERIFY_FAIL=0
    while IFS='|' read -r _pkg _import; do
        _out="$(python -c "$_import" 2>&1)" && \
            echo "[OK]    $_pkg: $_out" || \
            { echo "[ERROR] $_pkg: $_out"; _VERIFY_FAIL=1; }
    done <<'VERIFY'
lerobot|import importlib.metadata as M, lerobot; print("v"+M.version("lerobot"), "->", lerobot.__file__)
xensevr_pc_service_sdk|import importlib.metadata as M, xensevr_pc_service_sdk; print("v"+M.version("xensevr_pc_service_sdk"), "->", xensevr_pc_service_sdk.__file__)
xensesdk|import importlib.metadata as M, xensesdk; print("v"+M.version("xensesdk"), "->", xensesdk.__file__)
xensesdk flash|from xensesdk.flash.linux_backend import LinuxFlashBackend; print("available" if LinuxFlashBackend().available else "NOT available")
taccap-gripper|import importlib.metadata as M, xense.taccap; print("v"+M.version("taccap-gripper"), "->", xense.taccap.__file__)
VERIFY

    echo ""
    if [[ $_VERIFY_FAIL -eq 0 ]]; then
        echo "[INFO] All packages verified successfully."
    else
        echo "[ERROR] Some packages failed verification. Review the [ERROR] lines above."
    fi

    echo ""
    echo "[INFO] Lerobot-Xense installation complete."
    exit $_VERIFY_FAIL
else
    echo "Invalid argument. Usage:"
    echo "  --conda [env_name]   Create a conda environment (requires Miniconda/Anaconda)"
    echo "  --mamba [env_name]   Create a mamba environment (requires Miniforge)"
    echo "  --install            Install base package + all hardware SDK bindings"
    exit 1
fi
