#!/usr/bin/env bash
#
# Register a custom airframe name into PX4's posix airframe list so PX4's
# build system copies it into the SITL rootfs.
#
# PX4 does NOT glob the airframes directory; every airframe must be listed
# explicitly inside px4_add_romfs_files(...) in
#   ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
# Copying the airframe file in is not enough on its own -- without this
# registration PX4 fails at startup with "Unknown model gz_<model>".
#
# Idempotent in both directions.
#
# Usage:
#   register_airframe.sh <airframes-CMakeLists.txt> <airframe_name> [--unregister]

set -euo pipefail

LIST_FILE="${1:?path to airframes CMakeLists.txt required}"
AIRFRAME="${2:?airframe name required}"
ACTION="${3:-register}"

if [[ ! -f "${LIST_FILE}" ]]; then
    echo "ERROR: airframe list not found: ${LIST_FILE}" >&2
    exit 1
fi

already_listed() {
    grep -qE "^[[:space:]]*${AIRFRAME}[[:space:]]*$" "${LIST_FILE}"
}

if [[ "${ACTION}" == "--unregister" ]]; then
    if ! already_listed; then
        echo "Airframe '${AIRFRAME}' not present in PX4 airframe list; nothing to do."
        exit 0
    fi
    tmp="$(mktemp)"
    grep -vE "^[[:space:]]*${AIRFRAME}[[:space:]]*$" "${LIST_FILE}" > "${tmp}"
    mv "${tmp}" "${LIST_FILE}"
    echo "Unregistered airframe '${AIRFRAME}' from PX4 airframe list."
    exit 0
fi

if already_listed; then
    echo "Airframe '${AIRFRAME}' already registered in PX4 airframe list."
    exit 0
fi

# Insert the airframe name just before the first line that closes the
# px4_add_romfs_files(...) call (a line beginning with ')').
tmp="$(mktemp)"
awk -v name="${AIRFRAME}" '
    /^\)/ && !done { print "\t" name; done = 1 }
    { print }
' "${LIST_FILE}" > "${tmp}"

mv "${tmp}" "${LIST_FILE}"
echo "Registered airframe '${AIRFRAME}' in PX4 airframe list."
