#!/usr/bin/env bash
# cleanup_logs.sh - Delete flight logs older than 30 days
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_DIR="${PROJECT_ROOT}/logs"

echo "Cleaning logs older than 30 days in ${LOG_DIR}"
find "${LOG_DIR}" -type f \( -name "*.tlog" -o -name "*.csv" -o -name "*.log" -o -name "*.gz" \) -mtime +30 -print -delete
echo "Log cleanup complete"
