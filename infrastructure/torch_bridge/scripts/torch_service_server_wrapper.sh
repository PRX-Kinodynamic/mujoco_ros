#!/bin/bash
# Wrapper script to use custom Python environment for torch_service_server.py

CUSTOM_PYTHON="/common/home/st1122/Projects/mushr_mujoco_sysid/env/bin/python"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "$CUSTOM_PYTHON" "$SCRIPT_DIR/torch_service_server.py" "$@"

