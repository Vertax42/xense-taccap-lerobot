#!/usr/bin/env bash

set -Eeuo pipefail

SERVICE_NAME="roboticsservice"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
SERVICE_WRAPPER="/usr/local/bin/${SERVICE_NAME}-systemd-start.sh"
APP_DIR="/opt/apps/roboticsservice"
APP_BIN="${APP_DIR}/RoboticsServiceProcess"
APP_PROCESS_PATTERN="[R]oboticsServiceProcess"
PID_FILE="/run/${SERVICE_NAME}/${SERVICE_NAME}.pid"
DEFAULT_CHECK_TIMEOUT_SECONDS=15

usage() {
    cat <<'USAGE'
Usage:
  roboticsservice_autostart.sh [command] [--user USER] [--stop-existing]

Commands:
  install      Create and enable the systemd service. Default command.
  start        Start the service now.
  stop         Stop the service.
  restart      Restart the service.
  status       Show service status.
  check        Verify service file, autostart state, active state, and process.
  logs         Follow service logs.
  uninstall    Disable and remove the systemd service.
  help         Show this help.

Examples:
  scripts/roboticsservice_autostart.sh install
  scripts/roboticsservice_autostart.sh install --stop-existing
  scripts/roboticsservice_autostart.sh status
  scripts/roboticsservice_autostart.sh check
  scripts/roboticsservice_autostart.sh logs
USAGE
}

log() {
    printf '==> %s\n' "$*"
}

die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

check_ok() {
    printf '[OK]   %s\n' "$*"
}

check_fail() {
    printf '[FAIL] %s\n' "$*" >&2
}

have_command() {
    command -v "$1" >/dev/null 2>&1
}

require_systemctl() {
    have_command systemctl || die "systemctl not found; this script requires systemd."
}

run_root() {
    if [[ "${EUID}" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

detect_user() {
    local candidate
    local current_user

    if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
        printf '%s\n' "${SUDO_USER}"
        return 0
    fi

    current_user="$(id -un)"
    if [[ "${current_user}" != "root" ]]; then
        printf '%s\n' "${current_user}"
        return 0
    fi

    for candidate in "${LOGNAME:-}" "${USER:-}"; do
        [[ -n "${candidate}" && "${candidate}" != "root" ]] || continue
        if id -u "${candidate}" >/dev/null 2>&1; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done

    if have_command logname; then
        candidate="$(logname 2>/dev/null || true)"
        if [[ -n "${candidate}" && "${candidate}" != "root" ]] && id -u "${candidate}" >/dev/null 2>&1; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    fi

    printf '%s\n' "${current_user}"
}

validate_app() {
    [[ -d "${APP_DIR}" ]] || die "application directory not found: ${APP_DIR}. Install the original XenseVR-PC-Service .deb first, for example with setup_env.sh --install or: sudo dpkg -i XenseVR-PC-Service_*_\$(dpkg --print-architecture).deb"
    [[ -x "${APP_BIN}" ]] || die "application binary is not executable: ${APP_BIN}. Reinstall the original XenseVR-PC-Service .deb."
}

get_service_main_pid() {
    systemctl show "${SERVICE_NAME}.service" --property MainPID --value 2>/dev/null || true
}

get_app_pids() {
    have_command pgrep || return 0
    pgrep -f "${APP_PROCESS_PATTERN}" || true
}

wait_for_pid_exit() {
    local pid=$1
    local attempt

    for attempt in {1..10}; do
        if [[ ! -d "/proc/${pid}" ]]; then
            return 0
        fi
        sleep 1
    done

    return 1
}

ensure_no_external_processes() {
    local main_pid
    local pid
    local service_pid=${1:-}
    local stop_existing=${2:-0}
    local external_pids=()

    main_pid="$(get_service_main_pid)"
    while IFS= read -r pid; do
        [[ -n "${pid}" ]] || continue
        [[ "${pid}" == "${main_pid}" ]] && continue
        [[ -n "${service_pid}" && "${pid}" == "${service_pid}" ]] && continue
        external_pids+=("${pid}")
    done < <(get_app_pids)

    if [[ "${#external_pids[@]}" -eq 0 ]]; then
        return 0
    fi

    if [[ "${stop_existing}" -ne 1 ]]; then
        die "RoboticsServiceProcess is already running outside this service: PID(s) ${external_pids[*]}. Stop it first or rerun with --stop-existing."
    fi

    log "Stopping existing RoboticsServiceProcess PID(s): ${external_pids[*]}"
    run_root kill -TERM "${external_pids[@]}"
    for pid in "${external_pids[@]}"; do
        wait_for_pid_exit "${pid}" || die "process did not exit after SIGTERM: PID ${pid}"
    done
}

write_wrapper_file() {
    local temp_file

    temp_file="$(mktemp)"
    cat >"${temp_file}" <<WRAPPER
#!/usr/bin/env bash

set -Eeuo pipefail

APP_DIR="${APP_DIR}"
APP_BIN="${APP_BIN}"
APP_PROCESS_PATTERN="${APP_PROCESS_PATTERN}"
PID_FILE="${PID_FILE}"

export LD_LIBRARY_PATH="\${APP_DIR}:\${APP_DIR}/lib:\${APP_DIR}/SDK/x64"
export QT_PLUGIN_PATH="\${APP_DIR}/plugins/"
export QT_QML_PATH="\${APP_DIR}/qml/"

cd "\${APP_DIR}"
rm -f "\${PID_FILE}"

"\${APP_BIN}" &
pid=\$!

sleep 1
if kill -0 "\${pid}" >/dev/null 2>&1; then
    printf '%s\n' "\${pid}" >"\${PID_FILE}"
    exit 0
fi

fallback_pid="\$(pgrep -n -f "\${APP_PROCESS_PATTERN}" || true)"
if [[ -n "\${fallback_pid}" && -d "/proc/\${fallback_pid}" ]]; then
    printf '%s\n' "\${fallback_pid}" >"\${PID_FILE}"
    exit 0
fi

wait "\${pid}" >/dev/null 2>&1 || true
printf 'RoboticsServiceProcess exited during startup\n' >&2
exit 1
WRAPPER

    run_root install -m 0755 "${temp_file}" "${SERVICE_WRAPPER}"
    rm -f "${temp_file}"
}

write_service_file() {
    local service_user=$1
    local service_group=$2
    local temp_file

    temp_file="$(mktemp)"
    cat >"${temp_file}" <<SERVICE
[Unit]
Description=Robotics Service
Wants=network-online.target
After=network-online.target
StartLimitIntervalSec=30
StartLimitBurst=3

[Service]
Type=forking
User=${service_user}
Group=${service_group}
WorkingDirectory=${APP_DIR}
RuntimeDirectory=${SERVICE_NAME}
PIDFile=${PID_FILE}
ExecStart=${SERVICE_WRAPPER}
Restart=on-failure
RestartSec=5
KillSignal=SIGTERM
KillMode=control-group
StandardOutput=journal
StandardError=journal
SyslogIdentifier=${SERVICE_NAME}
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
SERVICE

    run_root install -m 0644 "${temp_file}" "${SERVICE_FILE}"
    rm -f "${temp_file}"
}

install_service() {
    local service_user=$1
    local stop_existing=$2
    local service_group

    require_systemctl
    validate_app
    id -u "${service_user}" >/dev/null 2>&1 || die "service user does not exist: ${service_user}"
    service_group="$(id -gn "${service_user}")"

    run_root systemctl stop "${SERVICE_NAME}.service" >/dev/null 2>&1 || true
    ensure_no_external_processes "" "${stop_existing}"

    log "Writing ${SERVICE_WRAPPER}"
    write_wrapper_file

    log "Writing ${SERVICE_FILE}"
    write_service_file "${service_user}" "${service_group}"

    log "Reloading systemd"
    run_root systemctl daemon-reload

    log "Enabling ${SERVICE_NAME}.service"
    run_root systemctl enable "${SERVICE_NAME}.service"

    log "Starting ${SERVICE_NAME}.service"
    run_root systemctl restart "${SERVICE_NAME}.service"

    run_root systemctl --no-pager --full status "${SERVICE_NAME}.service" || true
    check_service
}

uninstall_service() {
    require_systemctl

    log "Stopping ${SERVICE_NAME}.service"
    run_root systemctl stop "${SERVICE_NAME}.service" || true

    log "Disabling ${SERVICE_NAME}.service"
    run_root systemctl disable "${SERVICE_NAME}.service" || true

    log "Removing ${SERVICE_FILE} and ${SERVICE_WRAPPER}"
    run_root rm -f "${SERVICE_FILE}" "${SERVICE_WRAPPER}" "${PID_FILE}"

    log "Reloading systemd"
    run_root systemctl daemon-reload
    run_root systemctl reset-failed "${SERVICE_NAME}.service" || true
}

find_app_pid() {
    local main_pid=${1:-}

    if [[ "${main_pid}" =~ ^[0-9]+$ && "${main_pid}" -gt 0 && -d "/proc/${main_pid}" ]]; then
        printf '%s\n' "${main_pid}"
        return 0
    fi

    have_command pgrep || return 0
    pgrep -n -f "${APP_PROCESS_PATTERN}" || true
}

check_service() {
    local active_state
    local attempt
    local enabled_state
    local failed=0
    local main_pid
    local process_pid
    local timeout

    require_systemctl
    timeout="${ROBOTICSSERVICE_CHECK_TIMEOUT:-${DEFAULT_CHECK_TIMEOUT_SECONDS}}"
    [[ "${timeout}" =~ ^[0-9]+$ ]] || timeout="${DEFAULT_CHECK_TIMEOUT_SECONDS}"

    if [[ -f "${SERVICE_FILE}" ]]; then
        check_ok "service file exists: ${SERVICE_FILE}"
    else
        check_fail "service file is missing: ${SERVICE_FILE}"
        failed=1
    fi

    enabled_state="$(systemctl is-enabled "${SERVICE_NAME}.service" 2>/dev/null || true)"
    if [[ "${enabled_state}" == "enabled" || "${enabled_state}" == "enabled-runtime" ]]; then
        check_ok "${SERVICE_NAME}.service is enabled for boot"
    else
        check_fail "${SERVICE_NAME}.service is not enabled for boot (state: ${enabled_state:-unknown})"
        failed=1
    fi

    for ((attempt = 0; attempt <= timeout; attempt++)); do
        active_state="$(systemctl is-active "${SERVICE_NAME}.service" 2>/dev/null || true)"
        main_pid="$(systemctl show "${SERVICE_NAME}.service" --property MainPID --value 2>/dev/null || true)"
        process_pid="$(find_app_pid "${main_pid}")"

        if [[ "${active_state}" == "active" && -n "${process_pid}" ]]; then
            break
        fi

        if [[ "${attempt}" -lt "${timeout}" ]]; then
            sleep 1
        fi
    done

    if [[ "${active_state}" == "active" ]]; then
        check_ok "${SERVICE_NAME}.service is active"
    else
        check_fail "${SERVICE_NAME}.service is not active (state: ${active_state:-unknown})"
        failed=1
    fi

    if [[ -n "${process_pid}" && -d "/proc/${process_pid}" ]]; then
        check_ok "main process is running: PID ${process_pid}"
    else
        check_fail "main process is not running"
        failed=1
    fi

    if [[ "${failed}" -eq 0 ]]; then
        check_ok "Robotics Service autostart check passed"
    else
        die "Robotics Service autostart check failed"
    fi
}

service_action() {
    local action=$1
    local stop_existing=${2:-0}

    require_systemctl
    case "${action}" in
        start|restart)
            if [[ "${action}" == "restart" ]]; then
                run_root systemctl stop "${SERVICE_NAME}.service" >/dev/null 2>&1 || true
            fi
            ensure_no_external_processes "" "${stop_existing}"
            run_root systemctl "${action}" "${SERVICE_NAME}.service"
            ;;
        stop)
            run_root systemctl "${action}" "${SERVICE_NAME}.service"
            ;;
        status)
            run_root systemctl --no-pager --full status "${SERVICE_NAME}.service"
            ;;
        check)
            check_service
            ;;
        logs)
            run_root journalctl -u "${SERVICE_NAME}.service" -f
            ;;
        *)
            die "unsupported service action: ${action}"
            ;;
    esac
}

main() {
    local command="install"
    local service_user
    local stop_existing=0

    service_user="$(detect_user)"

    if [[ $# -gt 0 && "$1" != --* ]]; then
        command=$1
        shift
    fi

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --user)
                [[ $# -ge 2 ]] || die "--user requires a value"
                service_user=$2
                shift
                ;;
            --stop-existing)
                stop_existing=1
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                die "unknown option: $1"
                ;;
        esac
        shift
    done

    case "${command}" in
        install)
            install_service "${service_user}" "${stop_existing}"
            ;;
        uninstall)
            uninstall_service
            ;;
        start|stop|restart|status|check|logs)
            service_action "${command}" "${stop_existing}"
            ;;
        help|-h|--help)
            usage
            ;;
        *)
            die "unknown command: ${command}"
            ;;
    esac
}

main "$@"
