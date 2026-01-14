#!/bin/bash
# Wrapper script to use custom Python environment for torch_service_server.py

CUSTOM_PYTHON="/Users/Gary/pracsys/mushr_mujoco_sysid/env/bin/python"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "SCRIPT_DIR ${SCRIPT_DIR}"
echo "$CUSTOM_PYTHON" "$SCRIPT_DIR/torch_service_server.py" "$@"
exec "$CUSTOM_PYTHON" "$SCRIPT_DIR/torch_service_server.py" "$@"

