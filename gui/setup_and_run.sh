#!/bin/bash
# ============================================================
#  setup_and_run.sh
#  Builds all required ROS 2 packages (if not already built),
#  installs missing Python dependencies, and starts the GUI.
#
#  Usage:
#    bash ~/ros2_ws/src/megoldas_sim24/gui/setup_and_run.sh
# ============================================================

set -e  # exit on any error

WORKSPACE=~/ros2_ws
VENV=~/venv
ROS_SETUP=/opt/ros/jazzy/setup.bash
WS_SETUP=$WORKSPACE/install/setup.bash

# ── Colors ───────────────────────────────────────────────────
GREEN="\e[42;30m"
YELLOW="\e[43;30m"
RED="\e[41;97m"
RESET="\e[0m"
INFO="\e[36m"

ok()   { echo -e "${GREEN}  ✔ $* ${RESET}"; }
warn() { echo -e "${YELLOW}  ⚠ $* ${RESET}"; }
err()  { echo -e "${RED}  ✘ $* ${RESET}"; exit 1; }
info() { echo -e "${INFO}▶ $*${RESET}"; }

echo ""
echo "============================================================"
echo "  Robot Setup & Launch Script"
echo "============================================================"
echo ""

# ── 1. Check ROS is installed ────────────────────────────────
info "Checking ROS 2 (jazzy) installation..."
if [ ! -f "$ROS_SETUP" ]; then
    err "ROS 2 jazzy not found at $ROS_SETUP. Please install ROS 2 first."
fi
source "$ROS_SETUP"
ok "ROS 2 jazzy sourced"

# ── 2. Activate virtual environment ──────────────────────────
info "Activating Python virtual environment..."
if [ ! -f "$VENV/bin/activate" ]; then
    err "Virtual environment not found at $VENV. Please create it first:\n  python3 -m venv $VENV"
fi
source "$VENV/bin/activate"
ok "Virtual environment activated: $VENV"

# ── 3. Install required Python packages ──────────────────────
info "Checking Python dependencies..."
MISSING_PY=()
python -c "import dash" 2>/dev/null              || MISSING_PY+=("dash")
python -c "import dash_bootstrap_components" 2>/dev/null || MISSING_PY+=("dash-bootstrap-components")
python -c "import psutil" 2>/dev/null            || MISSING_PY+=("psutil")
python -c "import Cython" 2>/dev/null            || MISSING_PY+=("Cython")
python -c "import lark" 2>/dev/null              || MISSING_PY+=("lark")

if [ ${#MISSING_PY[@]} -gt 0 ]; then
    warn "Installing missing Python packages: ${MISSING_PY[*]}"
    pip install --quiet "${MISSING_PY[@]}"
    ok "Python packages installed"
else
    ok "All Python packages already installed"
fi

# ── 4. Build ROS 2 packages ───────────────────────────────────
cd "$WORKSPACE"

# Strip venv from PATH/PYTHONPATH so colcon uses ROS Python
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "$VENV" | tr '\n' ':' | sed 's/:$//')
export PYTHONPATH=$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v "$VENV" | tr '\n' ':' | sed 's/:$//')
unset VIRTUAL_ENV

PACKAGES=(
    "lslidar_msgs"
    "lslidar_driver"
    "serial"
    "wheeltec_robot_msg"
    "wheeltec_robot_urdf"
    "wheeltec_py_package"
    "usb_cam_launcher"
    "turn_on_wheeltec_robot"
    "megoldas_sim24"
)

NEEDS_BUILD=()
for pkg in "${PACKAGES[@]}"; do
    if [ ! -f "$WORKSPACE/install/$pkg/share/$pkg/package.xml" ]; then
        NEEDS_BUILD+=("$pkg")
    fi
done

if [ ${#NEEDS_BUILD[@]} -gt 0 ]; then
    info "Building missing packages: ${NEEDS_BUILD[*]}"
    colcon build \
        --packages-select "${NEEDS_BUILD[@]}" \
        --symlink-install \
        --base-paths "$WORKSPACE" \
        2>&1 | grep -E "^(Starting|Finished|Failed|Summary|---|\[)" || true

    # Check colcon result by looking for failed packages in summary
    BUILD_RC=${PIPESTATUS[0]}
    if [ "$BUILD_RC" -ne 0 ]; then
        err "One or more packages failed to build. Check the output above."
    fi
    ok "Build complete"
else
    ok "All packages already built"
fi

# ── 5. Source workspace ───────────────────────────────────────
info "Sourcing workspace..."
if [ ! -f "$WS_SETUP" ]; then
    err "Workspace setup.bash not found. Build may have failed."
fi
source "$WS_SETUP"
ok "Workspace sourced"

# ── 6. Re-activate venv (sourcing ROS may override Python) ───
source "$VENV/bin/activate"

# ── 7. Launch the GUI ─────────────────────────────────────────
GUI_SCRIPT="$WORKSPACE/src/megoldas_sim24/gui/robot_gui.py"
if [ ! -f "$GUI_SCRIPT" ]; then
    err "GUI script not found: $GUI_SCRIPT"
fi

echo ""
echo "============================================================"
ok "All ready! Starting Robot Control GUI..."
echo "  Open your browser at:  http://localhost:8050"
echo "============================================================"
echo ""

python "$GUI_SCRIPT"
