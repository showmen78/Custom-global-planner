#!/usr/bin/env bash

# PROJECT_ROOT stores this project folder path as a shell string.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Load conda, activate the admap environment, and then load the built map package.
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate admap

source "${PROJECT_ROOT}/map_repo/install/setup.bash"

# Add the local scripts folder to PYTHONPATH so the runner can import the helper modules.
export PYTHONPATH="${PROJECT_ROOT}/scripts:${PYTHONPATH:-}"

echo "AD Map test environment activated"
echo "Python: $(which python)"
