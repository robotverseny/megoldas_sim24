#!/usr/bin/env python3
"""
Robot Control GUI - Dash App
Handles startup, running and shutdown of the wheeltec robot and megoldas_sim24 solution.

Usage:
    source /home/wheeltec/venv/bin/activate
    python ~/ros2_ws/src/megoldas_sim24/gui/robot_gui.py
Then open: http://localhost:8050
"""

import subprocess
import signal
import os
import threading
import time
import psutil
import dash
import dash_bootstrap_components as dbc
from dash import dcc, html, Input, Output, State, callback_context

# ── ROS Environment ───────────────────────────────────────────────────────────
SETUP_BASH = (
    "source /opt/ros/jazzy/setup.bash && "
    "source /home/wheeltec/ros2_ws/install/setup.bash"
)

def _ros_env():
    env = os.environ.copy()
    venv = "/home/wheeltec/venv"
    env["PATH"] = ":".join(
        p for p in env.get("PATH", "").split(":") if not p.startswith(venv)
    )
    env["PYTHONPATH"] = ":".join(
        p for p in env.get("PYTHONPATH", "").split(":") if not p.startswith(venv)
    )
    env.pop("VIRTUAL_ENV", None)
    return env

ROS_ENV = _ros_env()

# ── Process definitions ───────────────────────────────────────────────────────
PROCESSES = {
    "driver": {
        "label": "Robot Driver",
        "description": "turn_on_wheeltec_robot — hardware, LiDAR, camera, EKF",
        "cmd": f"bash -c '{SETUP_BASH} && ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py'",
        "match": "turn_on_wheeltec_robot",
        "color": "primary",
    },
    "megoldas1": {
        "label": "Megoldás 1 — Simple Pursuit",
        "description": "megoldas1.launch.py — simple_pursuit.py",
        "cmd": f"bash -c '{SETUP_BASH} && ros2 launch megoldas_sim24 megoldas1.launch.py'",
        "match": "megoldas1",
        "color": "success",
    },
    "megoldas2": {
        "label": "Megoldás 2 — Follow the Gap",
        "description": "megoldas2.launch.py — follow_the_gap.py",
        "cmd": f"bash -c '{SETUP_BASH} && ros2 launch megoldas_sim24 megoldas2.launch.py'",
        "match": "megoldas2",
        "color": "success",
    },
}

STOP_ALL_SCRIPT = os.path.expanduser("~/ros2_ws/src/drivers/shell/stop_all.sh")

# ── ROS parameter definitions per solution ────────────────────────────────────
SOLUTION_PARAMS = {
    "megoldas1": {
        "node": "simple_pursuit",
        "params": [
            {"name": "velocity",  "label": "Speed Scale (max = ×0.5 m/s)",     "type": "double", "min": 0.1, "max": 3.0, "step": 0.05, "default": 1.0, "marks": {0.1: "0.1", 1.0: "1.0", 2.0: "2.0", 3.0: "3.0"}},
            {"name": "wheelbase", "label": "Wheelbase Scale (×0.3187 m)",       "type": "double", "min": 0.5, "max": 2.0, "step": 0.05, "default": 1.0, "marks": {0.5: "0.5×", 1.0: "1.0×", 1.5: "1.5×", 2.0: "2.0×"},
             "multiplier": 0.3187},
        ],
    },
    "megoldas2": {
        "node": "follow_the_gap",
        "params": [
            {"name": "max_throttle",         "label": "Max Throttle (m/s)",   "type": "double", "min": 0.1, "max": 2.0, "step": 0.05, "default": 0.5, "marks": {0.1: "0.1", 0.5: "0.5", 1.0: "1.0", 2.0: "2.0"}},
            {"name": "safety_radius",        "label": "Safety Radius (m)",    "type": "double", "min": 0.2, "max": 5.0, "step": 0.1,  "default": 2.0, "marks": {0.2: "0.2", 1.0: "1.0", 2.0: "2.0", 5.0: "5.0"}},
            {"name": "steering_sensitivity", "label": "Steering Sensitivity", "type": "double", "min": 0.1, "max": 2.0, "step": 0.05, "default": 0.7, "marks": {0.1: "0.1", 0.7: "0.7", 1.0: "1.0", 2.0: "2.0"}},
        ],
    },
}

# ── Process helpers ───────────────────────────────────────────────────────────
_procs: dict[str, subprocess.Popen] = {}

# Pending param values — updated by sliders regardless of running state
# { "megoldas1": {"velocity": 1.0, ...}, "megoldas2": {...} }
_pending_params: dict[str, dict] = {
    sol: {p["name"]: p["default"] for p in cfg["params"]}
    for sol, cfg in SOLUTION_PARAMS.items()
}

def is_running(key: str) -> bool:
    proc = _procs.get(key)
    if proc is not None and proc.poll() is None:
        return True
    match = PROCESSES[key]["match"]
    for p in psutil.process_iter(["cmdline"]):
        try:
            cmdline = p.info["cmdline"] or []
            # Skip the GUI itself to avoid false positives
            if any("robot_gui.py" in arg for arg in cmdline):
                continue
            if any(match in arg for arg in cmdline):
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return False

def start_process(key: str) -> str:
    msgs = []

    # If starting a solution, stop any other solution that is currently running
    if key in SOLUTION_PARAMS:
        other_solutions = [k for k in SOLUTION_PARAMS if k != key]
        for other in other_solutions:
            if is_running(other):
                msgs.append(f"⚠️ Auto-stopping {PROCESSES[other]['label']} before starting {PROCESSES[key]['label']}...")
                msgs.append(stop_process(other))

    if is_running(key):
        return "\n".join(msgs) + f"\n⚠️ {PROCESSES[key]['label']} is already running." if msgs else f"⚠️ {PROCESSES[key]['label']} is already running."

    proc = subprocess.Popen(
        PROCESSES[key]["cmd"], shell=True, preexec_fn=os.setsid,
        env=ROS_ENV, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        executable="/bin/bash",
    )
    _procs[key] = proc
    # Push all pending param values once the node is ready
    if key in SOLUTION_PARAMS:
        threading.Thread(
            target=_push_params_when_ready,
            args=(key,), daemon=True
        ).start()
    msgs.append(f"✅ Started {PROCESSES[key]['label']} (pid {proc.pid})")
    return "\n".join(msgs)


def _push_params_when_ready(solution_key: str):
    """Poll until the ROS node responds, then push all pending param values."""
    cfg = SOLUTION_PARAMS[solution_key]
    node = cfg["node"]
    first_param = cfg["params"][0]["name"]
    try:
        # Poll until node is ready (max 10s, check every 0.4s)
        ready = False
        for _ in range(25):
            time.sleep(0.4)
            check_cmd = f"bash -c '{SETUP_BASH} && ros2 param get /{node} {first_param}'"
            result = subprocess.run(
                check_cmd, shell=True, capture_output=True, text=True,
                timeout=2, env=ROS_ENV, executable="/bin/bash"
            )
            if result.returncode == 0 and first_param in result.stdout:
                ready = True
                break
        if not ready:
            print(f"[WARN] _push_params_when_ready: /{node} did not respond in time")
            return
        for p in cfg["params"]:
            value = _pending_params[solution_key][p["name"]]
            ros_value = value * p["multiplier"] if "multiplier" in p else value
            ros_param_set(node, p["name"], ros_value, p["type"])
    except Exception as e:
        print(f"[WARN] _push_params_when_ready error: {e}")

def _kill_procs_by_match(match: str) -> list[str]:
    """Find all processes matching 'match' in their cmdline and kill them SIGINT→SIGTERM→SIGKILL."""
    targets = []
    for p in psutil.process_iter(["pid", "cmdline"]):
        try:
            cmdline = p.info["cmdline"] or []
            # Skip the GUI process itself to avoid self-kill
            if any(match in arg for arg in cmdline) and not any("robot_gui.py" in arg for arg in cmdline):
                targets.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    killed = []
    for p in targets:
        try:
            p.send_signal(signal.SIGINT)
            killed.append(str(p.pid))
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    # Wait up to 4 seconds for SIGINT to take effect
    time.sleep(1.0)
    for p in targets:
        try:
            if p.is_running():
                p.send_signal(signal.SIGTERM)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    time.sleep(0.5)
    for p in targets:
        try:
            if p.is_running():
                p.send_signal(signal.SIGKILL)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    return killed


def stop_process(key: str) -> str:
    proc = _procs.pop(key, None)
    stopped = []

    # Kill the tracked process group first
    if proc is not None:
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            stopped.append(str(proc.pid))
        except Exception:
            pass

    # Kill any remaining matching processes (covers external starts + children)
    match = PROCESSES[key]["match"]
    extra = _kill_procs_by_match(match)
    stopped.extend(extra)

    # Final SIGKILL on tracked process group
    if proc is not None:
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass

    if stopped:
        return f"🛑 Stopped {PROCESSES[key]['label']} (pids: {', '.join(dict.fromkeys(stopped))})"
    return f"ℹ️ {PROCESSES[key]['label']} was not running."

def stop_all() -> str:
    msgs = []
    for key in ["megoldas1", "megoldas2", "driver"]:
        msgs.append(stop_process(key))
    try:
        result = subprocess.run(
            ["bash", STOP_ALL_SCRIPT], capture_output=True, text=True,
            timeout=15, env={**os.environ, "HOME": os.path.expanduser("~")}
        )
        msgs.append("🔧 stop_all.sh: " + (result.stdout.strip() or "done"))
    except Exception as e:
        msgs.append(f"⚠️ stop_all.sh error: {e}")
    return "\n".join(msgs)

def ros_param_set(node: str, param_name: str, value, param_type: str) -> str:
    val_str = str(int(value)) if param_type == "integer" else str(round(float(value), 6))
    cmd = f"bash -c '{SETUP_BASH} && ros2 param set /{node} {param_name} {val_str}'"
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True,
            timeout=5, env=ROS_ENV, executable="/bin/bash"
        )
        out = result.stdout.strip() or result.stderr.strip()
        return f"🎚️ /{node} {param_name}={val_str}  →  {out}"
    except subprocess.TimeoutExpired:
        return f"⚠️ Timeout setting /{node} {param_name}"
    except Exception as e:
        return f"⚠️ Error: {e}"

def status_badge(key: str):
    running = is_running(key)
    return dbc.Badge(
        "● RUNNING" if running else "○ STOPPED",
        color="success" if running else "secondary",
        className="ms-2",
    )

# ── Slider builder ────────────────────────────────────────────────────────────
def build_slider(solution_key: str, p: dict):
    sid = f"slider-{solution_key}-{p['name']}"
    vid = f"slider-val-{solution_key}-{p['name']}"
    return html.Div([
        html.Div([
            html.Label(p["label"], className="slider-label"),
            html.Span(id=vid,
                      children=str(p["default"]) if p["type"] == "integer" else f"{p['default']:.3f}",
                      className="slider-value"),
        ], className="d-flex align-items-center mb-1"),
        dcc.Slider(
            id=sid,
            min=p["min"], max=p["max"], step=p["step"],
            value=p["default"],
            marks=p["marks"],
            tooltip={"placement": "bottom", "always_visible": False},
            updatemode="mouseup",
        ),
    ], className="mb-4")

def build_params_panel(solution_key: str):
    cfg = SOLUTION_PARAMS[solution_key]
    return dbc.Card([
        dbc.CardHeader(html.B(f"⚙️ ROS Parameters  —  /{cfg['node']}")),
        dbc.CardBody([build_slider(solution_key, p) for p in cfg["params"]]),
    ], className="mb-3 shadow-sm")

# ── App layout ────────────────────────────────────────────────────────────────
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.DARKLY],
                title="Robot Control Panel")

app.layout = dbc.Container([
    dbc.Row([dbc.Col([
        html.H2("🤖 Robot Control Panel", className="mt-3 mb-1"),
        html.P("Wheeltec · megoldas_sim24", className="text-muted"),
        html.Hr(),
    ])]),

    dbc.Alert([
        html.B("ℹ️ Robot Driver auto-starts when GUI launches. "),
        html.Span("Stop order: Solution first → Driver last.")
    ], color="info", className="mb-3 py-2"),

    # ── Driver ────────────────────────────────────────────────────────────────
    dbc.Row([dbc.Col([
        dbc.Card([dbc.CardBody([
            html.Div([
                html.H5(PROCESSES["driver"]["label"], className="d-inline"),
                html.Span(id="badge-driver"),
            ]),
            html.Small(PROCESSES["driver"]["description"], className="text-muted"),
            html.Div([
                dbc.Button("▶ Start", id="btn-start-driver", color="primary",
                           outline=True, size="sm", className="me-2 mt-2"),
                dbc.Button("■ Stop",  id="btn-stop-driver",  color="danger",
                           outline=True, size="sm", className="mt-2"),
            ]),
        ])], className="mb-3 shadow-sm"),
    ], md=12)]),

    html.Hr(),

    # ── Solution selector ─────────────────────────────────────────────────────
    dbc.Row([dbc.Col([
        html.H5("🧠 Select Solution", className="mb-2"),
        dbc.RadioItems(
            id="solution-selector",
            options=[
                {"label": "  Megoldás 1 — Simple Pursuit",  "value": "megoldas1"},
                {"label": "  Megoldás 2 — Follow the Gap",  "value": "megoldas2"},
            ],
            value="megoldas1",
            inline=True,
            inputClassName="me-1",
            labelClassName="me-5 fs-6",
        ),
    ], md=12)], className="mb-3"),

    # ── Active solution card ──────────────────────────────────────────────────
    dbc.Row([dbc.Col([
        dbc.Card([dbc.CardBody([
            html.Div([
                html.H5(id="solution-label", className="d-inline"),
                html.Span(id="badge-solution"),
            ]),
            html.Small(id="solution-desc", className="text-muted"),
            html.Div([
                dbc.Button("▶ Start", id="btn-start-solution", color="success",
                           outline=True, size="sm", className="me-2 mt-2"),
                dbc.Button("■ Stop",  id="btn-stop-solution",  color="danger",
                           outline=True, size="sm", className="mt-2"),
            ]),
        ])], className="mb-3 shadow-sm"),
    ], md=12)]),

    # ── Parameter panels ──────────────────────────────────────────────────────
    dbc.Row([dbc.Col([
        html.Div(id="params-panel-m1", children=build_params_panel("megoldas1")),
        html.Div(id="params-panel-m2", children=build_params_panel("megoldas2"),
                 style={"display": "none"}),
    ], md=12)]),

    html.Hr(),

    # ── Controls ──────────────────────────────────────────────────────────────
    dbc.Row([
        dbc.Col([dbc.Button("🚨 STOP ALL", id="btn-stop-all", color="danger",
                            size="lg", className="w-100 fw-bold")], md=4),
        dbc.Col([dbc.Button("🔄 Refresh",  id="btn-refresh",   color="secondary",
                            size="lg", className="w-100")], md=4),
    ], className="mb-3"),

    # ── Log ───────────────────────────────────────────────────────────────────
    dbc.Row([dbc.Col([
        html.H6("Log"),
        dcc.Textarea(
            id="log-output",
            value="🚀 GUI started — Robot Driver is being launched automatically.\n"
                  "Wait a few seconds, then start your solution.\n"
                  "Stop order: Solution first → Driver last.\n",
            readOnly=True,
        ),
    ])]),

    dcc.Interval(id="interval", interval=3000, n_intervals=0),

], fluid=True)


# ── Callback: selector → show/hide panels + update card labels ────────────────
@app.callback(
    Output("params-panel-m1", "style"),
    Output("params-panel-m2", "style"),
    Output("solution-label",  "children"),
    Output("solution-desc",   "children"),
    Input("solution-selector", "value"),
)
def switch_solution(selected):
    label = PROCESSES[selected]["label"]
    desc  = PROCESSES[selected]["description"]
    if selected == "megoldas1":
        return {}, {"display": "none"}, label, desc
    return {"display": "none"}, {}, label, desc


# ── Callback: buttons + interval → log + badges ───────────────────────────────
@app.callback(
    Output("log-output",     "value"),
    Output("badge-driver",   "children"),
    Output("badge-solution", "children"),
    Input("btn-start-driver",   "n_clicks"),
    Input("btn-stop-driver",    "n_clicks"),
    Input("btn-start-solution", "n_clicks"),
    Input("btn-stop-solution",  "n_clicks"),
    Input("btn-stop-all",       "n_clicks"),
    Input("btn-refresh",        "n_clicks"),
    Input("interval",           "n_intervals"),
    State("solution-selector",  "value"),
    State("log-output",         "value"),
    prevent_initial_call=False,
)
def handle_buttons(
    _sd_start, _sd_stop, _ss_start, _ss_stop,
    _stop_all, _refresh, _interval,
    selected_solution, log
):
    log = log or ""
    ctx = callback_context
    msg = ""
    if ctx.triggered:
        tid = ctx.triggered[0]["prop_id"].split(".")[0]
        if   tid == "btn-start-driver":   msg = start_process("driver")
        elif tid == "btn-stop-driver":    msg = stop_process("driver")
        elif tid == "btn-start-solution": msg = start_process(selected_solution)
        elif tid == "btn-stop-solution":  msg = stop_process(selected_solution)
        elif tid == "btn-stop-all":       msg = stop_all()
        elif tid == "btn-refresh":        msg = "🔄 Status refreshed."
    if msg:
        log = log + msg + "\n"
    return log, status_badge("driver"), status_badge(selected_solution)


# ── Callbacks: sliders → ros2 param set ──────────────────────────────────────
def _register_slider_callback(solution_key: str, p: dict):
    sid  = f"slider-{solution_key}-{p['name']}"
    vid  = f"slider-val-{solution_key}-{p['name']}"
    node = SOLUTION_PARAMS[solution_key]["node"]

    @app.callback(
        Output(vid, "children"),
        Output("log-output", "value", allow_duplicate=True),
        Input(sid, "value"),
        State("log-output", "value"),
        prevent_initial_call=True,
    )
    def _cb(value, log, _p=p, _node=node, _sol=solution_key):
        if value is None:
            return dash.no_update, dash.no_update
        display = str(int(value)) if _p["type"] == "integer" else f"{value:.3f}"
        log = log or ""
        # Always store the raw slider value for use when solution starts
        _pending_params[_sol][_p["name"]] = value
        # Apply multiplier if defined (e.g. wheelbase scale factor)
        ros_value = value * _p["multiplier"] if "multiplier" in _p else value
        if is_running(_sol):
            msg = ros_param_set(_node, _p["name"], ros_value, _p["type"])
        else:
            ros_display = f"{ros_value:.4f}" if "multiplier" in _p else display
            msg = f"📋 {_p['name']}={ros_display}  (saved — will apply when solution starts)"
        return display, log + msg + "\n"

for _sol_key, _sol_cfg in SOLUTION_PARAMS.items():
    for _param in _sol_cfg["params"]:
        _register_slider_callback(_sol_key, _param)


if __name__ == "__main__":
    print("=" * 55)
    print("  Robot Control Panel running at http://localhost:8050")
    print("=" * 55)
    print("[INFO] Auto-starting Robot Driver...")
    print(f"[INFO] {start_process('driver')}")
    app.run(debug=False, host="0.0.0.0", port=8050)
