#!/usr/bin/env bash
#
# Insert (or remove) the ArUco landing pad into a PX4 Gazebo world file.
#
# PX4's world (Tools/simulation/gz/worlds/<world>.sdf) is not globbed for
# extra models, so the pad must be added with an explicit <include> before
# the closing </world> tag. The pad model itself is resolved via model://
# from PX4's Gazebo models directory (installed by the CMake setup target).
#
# The inserted block is bracketed with BEGIN/END marker comments so this is
# idempotent and cleanly reversible.
#
# Usage:
#   register_landing_pad.sh <world.sdf> [--unregister]

set -euo pipefail

WORLD_FILE="${1:?path to world .sdf required}"
ACTION="${2:-register}"

BEGIN="<!-- BEGIN aruco_landing_pad (drone_dev_sim) -->"
END="<!-- END aruco_landing_pad (drone_dev_sim) -->"

if [[ ! -f "${WORLD_FILE}" ]]; then
    echo "ERROR: world file not found: ${WORLD_FILE}" >&2
    exit 1
fi

already_present() {
    grep -qF "${BEGIN}" "${WORLD_FILE}"
}

if [[ "${ACTION}" == "--unregister" ]]; then
    if ! already_present; then
        echo "Landing pad not present in ${WORLD_FILE}; nothing to do."
        exit 0
    fi
    tmp="$(mktemp)"
    # Drop every line from BEGIN to END inclusive.
    awk -v b="${BEGIN}" -v e="${END}" '
        index($0, b) { skip = 1 }
        !skip { print }
        index($0, e) { skip = 0 }
    ' "${WORLD_FILE}" > "${tmp}"
    mv "${tmp}" "${WORLD_FILE}"
    echo "Removed ArUco landing pad from ${WORLD_FILE}."
    exit 0
fi

if already_present; then
    echo "ArUco landing pad already present in ${WORLD_FILE}."
    exit 0
fi

# Insert the include block just before the first </world> closing tag.
tmp="$(mktemp)"
awk -v b="${BEGIN}" -v e="${END}" '
    /<\/world>/ && !done {
        print "    " b
        print "    <include>"
        print "      <uri>model://aruco_landing_pad</uri>"
        print "      <name>aruco_landing_pad</name>"
        print "      <pose>0 0 0.001 0 0 0</pose>"
        print "    </include>"
        print "    " e
        done = 1
    }
    { print }
' "${WORLD_FILE}" > "${tmp}"

mv "${tmp}" "${WORLD_FILE}"
echo "Added ArUco landing pad to ${WORLD_FILE} at origin (0, 0)."
