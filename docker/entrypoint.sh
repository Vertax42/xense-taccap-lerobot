#!/usr/bin/env bash
set -euo pipefail

# The daemon is installed by setup_env.sh and serves Pico4 tracker data over the
# network. It is optional so dataset-only jobs can disable it cleanly.
if [[ "${START_XENSEVR_SERVICE:-1}" == "1" ]]; then
    service_script="/opt/apps/roboticsservice/runService.sh"
    if [[ -x "${service_script}" ]]; then
        service_dir="$(dirname "${service_script}")"
        service_log="${XENSEVR_SERVICE_LOG:-/tmp/xensevr-service.log}"
        echo "[entrypoint] Starting XenseVR PC Service (log: ${service_log})"
        (
            cd "${service_dir}"
            exec "${service_script}"
        ) >"${service_log}" 2>&1 &
    else
        echo "[entrypoint] XenseVR PC Service is not installed; continuing." >&2
    fi
fi

if [[ "$#" -eq 0 ]]; then
    set -- bash
fi

exec "$@"
