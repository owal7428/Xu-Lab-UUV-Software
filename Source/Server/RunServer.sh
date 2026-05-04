#!/usr/bin/env bash
set -e

# Gets absolute script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"

PYTHON="$VENV_DIR/bin/python"

# Run server in virtual environment
exec "$PYTHON" "$SCRIPT_DIR/RunServer.py"
