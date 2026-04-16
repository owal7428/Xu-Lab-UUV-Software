#!/usr/bin/env bash
set -e

# Gets absolute script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/venv"
REQ_FILE="$SCRIPT_DIR/requirements.txt"

# Create venv if missing
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
fi

PYTHON="$VENV_DIR/bin/python"
PIP="$VENV_DIR/bin/pip"

# Upgrade pip (quiet after first time)
$PYTHON -m pip install --upgrade pip

# Install dependencies (only installs missing/outdated)
$PIP install -r "$REQ_FILE"

# Run server
exec "$PYTHON" "$SCRIPT_DIR/RunServer.py"
