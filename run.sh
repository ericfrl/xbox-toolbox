#!/bin/bash
# Xbox Toolbox Launcher
# Activates virtual environment and runs the toolbox

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PATH="${VENV_PATH:-/home/frl/Documents/repoar4/ar4repo}"

# Check if venv exists
if [ -d "$VENV_PATH" ]; then
    source "$VENV_PATH/bin/activate"
    echo "Virtual environment activated: $VENV_PATH"
else
    echo "Warning: Virtual environment not found at $VENV_PATH"
    echo "Using system Python..."
fi

# Run the toolbox
python3 "$SCRIPT_DIR/xbox_toolbox.py" "$@"
