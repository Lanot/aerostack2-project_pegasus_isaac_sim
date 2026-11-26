#!/usr/bin/env bash
# Run only the simulator with the supplied trusted Bash tuning environment.
set -euo pipefail
px4_tuning_file="${1:?Missing PX4 environment file}"
shift
if (($# == 0)); then
    echo "Missing command to launch with PX4 environment" >&2
    exit 2
fi
if [[ ! -f "$px4_tuning_file" || ! -r "$px4_tuning_file" ]]; then
    echo "PX4 environment file is not a readable file: $px4_tuning_file" >&2
    exit 2
fi
set -a
source "$px4_tuning_file"
set +a
exec "$@"
