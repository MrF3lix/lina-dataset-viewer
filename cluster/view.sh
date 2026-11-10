#!/bin/bash
# Submit the viewer job, wait until it serves, and print the URL and the ssh tunnel command.
# Run on the login node from anywhere:
#
#   cluster/view.sh /raid/persistent_scratch/saaf/linasim/datasets/<name> [extra sbatch args]
#   cluster/view.sh /path/to/dataset --time=120
#
# The job keeps running after this script exits; stop it with `scancel <job>`.

set -euo pipefail
cd "$(dirname "$0")/.."

DATASET="${1:?usage: cluster/view.sh <dataset folder> [sbatch args]}"
shift
DATASET="$(cd "${DATASET}" && pwd)"
mkdir -p logs/slurm

JOB=$(sbatch --parsable "$@" cluster/viewer.submit "${DATASET}")
JOB="${JOB%%;*}"
echo "submitted job ${JOB}; waiting for a CPU node (Ctrl-C stops waiting, not the job)"

while true; do
    STATE=$(squeue -h -j "${JOB}" -o %T 2>/dev/null || true)
    if [ -z "${STATE}" ]; then
        echo "job ${JOB} is no longer queued or running; its log:" >&2
        cat logs/slurm/viewer_"${JOB}"_*.out 2>/dev/null >&2 || true
        exit 1
    fi
    LOG=$(ls logs/slurm/viewer_"${JOB}"_*.out 2>/dev/null | head -1 || true)
    if [ "${STATE}" = "RUNNING" ] && [ -n "${LOG}" ] && grep -q '^open ' "${LOG}"; then
        echo
        cat "${LOG}"
        echo
        echo "stop it with: scancel ${JOB}"
        exit 0
    fi
    sleep 3
done
