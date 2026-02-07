from controller import Supervisor
import tkinter as tk
from tkinter import messagebox
import math
import tempfile

# ----------------------------
# Webots init
# ----------------------------

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

motor_names = ["A motor", "B motor", "C motor", "D motor", "E motor", "F motor"]
sensor_names = ["A sensor", "B sensor", "C sensor", "D sensor", "E sensor", "F sensor"]

motors = []
sensors = []

for mn, sn in zip(motor_names, sensor_names):
    m = supervisor.getDevice(mn)
    s = supervisor.getDevice(sn)
    s.enable(timestep)
    m.setVelocity(1.5)
    motors.append(m)
    sensors.append(s)

def clamp(x, lo, hi):
    if lo is None:
        lo = -float("inf")
    if hi is None:
        hi = float("inf")
    return max(lo, min(hi, x))

supervisor.step(timestep)

limits = [(m.getMinPosition(), m.getMaxPosition()) for m in motors]
home_positions = [s.getValue() for s in sensors]
targets = home_positions.copy()

def apply_targets():
    for i, m in enumerate(motors):
        lo, hi = limits[i]
        m.setPosition(clamp(targets[i], lo, hi))

pen = None
try:
    pen = supervisor.getDevice("pen")
except Exception:
    pen = None

def go_home():
    global targets
    targets = home_positions.copy()
    apply_targets()
    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass

apply_targets()

# ----------------------------
# IK setup (ikpy)
# ----------------------------

IKPY_AVAILABLE = False
arm_chain = None
IKPY_MAX_ITERATIONS = 20

try:
    from ikpy.chain import Chain

    urdf_text = supervisor.getUrdf()
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as f:
        f.write(urdf_text.encode("utf-8"))
        urdf_path = f.name

    arm_chain = Chain.from_urdf_file(urdf_path)
    IKPY_AVAILABLE = True
except Exception:
    IKPY_AVAILABLE = False
    arm_chain = None

def current_ik_initial_position():
    return [0] + [s.getValue() for s in sensors] + [0]

# ----------------------------
# TARGET + ARM nodes
# ----------------------------

target_def_name = "TARGET"
target_node = supervisor.getFromDef(target_def_name)
arm_node = supervisor.getSelf()

def get_target_abs_position():
    if target_node is None:
        return None
    p = target_node.getPosition()
    return (p[0], p[1], p[2])

# ----------------------------
# Circle executor 
# ----------------------------

circle_running = False
circle_start_time = 0.0

def start_circle_default():
    global circle_running, circle_start_time
    if not IKPY_AVAILABLE:
        messagebox.showerror("Draw Circle", "Failed.")
        return
    circle_running = True
    circle_start_time = supervisor.getTime()
    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass

def stop_circle():
    global circle_running
    circle_running = False
    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass

def update_circle_loop():
    global circle_running
    if not circle_running:
        return

    t = supervisor.getTime() - circle_start_time

    x = 0.25 * math.cos(t) + 1.1
    y = 0.25 * math.sin(t) - 0.95
    z = 0.05

    initial_position = current_ik_initial_position()
    ikResults = arm_chain.inverse_kinematics(
        [x, y, z],
        max_iter=IKPY_MAX_ITERATIONS,
        initial_position=initial_position
    )

    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])

    motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
    motors[5].setPosition(ikResults[1])

    if t > 2 * math.pi + 1.5:
        stop_circle()
    elif t > 1.5:
        if pen is not None:
            try:
                pen.write(True)
            except Exception:
                pass
    else:
        if pen is not None:
            try:
                pen.write(False)
            except Exception:
                pass

# ----------------------------
# Target follow executor
# ----------------------------

FOLLOW_EVERY_N_STEPS = 2
_follow_counter = 0

def update_target_follow_loop(active_screen, status_label=None):
    global _follow_counter

    if active_screen != "target":
        return

    if target_node is None:
        if status_label is not None:
            status_label.config(text="Follow: TARGET not found", fg="#B00020")
        return

    if not IKPY_AVAILABLE:
        if status_label is not None:
            status_label.config(text="Follow: OFF", fg="#B00020")
        return

    _follow_counter += 1
    if _follow_counter % FOLLOW_EVERY_N_STEPS != 0:
        return

    targetPosition = target_node.getPosition()
    armPosition = arm_node.getPosition()

    x = -(targetPosition[1] - armPosition[1])
    y = targetPosition[0] - armPosition[0]
    z = targetPosition[2] - armPosition[2]

    initial_position = current_ik_initial_position()
    ikResults = arm_chain.inverse_kinematics(
        [x, y, z],
        max_iter=IKPY_MAX_ITERATIONS,
        initial_position=initial_position
    )

    position = arm_chain.forward_kinematics(ikResults)
    squared_distance = (position[0, 3] - x) ** 2 + (position[1, 3] - y) ** 2 + (position[2, 3] - z) ** 2
    if math.sqrt(squared_distance) > 0.03:
        ikResults = arm_chain.inverse_kinematics([x, y, z])

    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])

    if status_label is not None:
        status_label.config(text="Follow: ON (using TARGET)", fg="#1B5E20")

# ----------------------------
# Tkinter GUI
# ----------------------------

root = tk.Tk()
root.title("Robot Controller")
root.geometry("650x650")
root.configure(bg="#f0f0f0")

container = tk.Frame(root, bg="#f0f0f0")
container.pack(fill="both", expand=True)
container.grid_rowconfigure(0, weight=1)
container.grid_columnconfigure(0, weight=1)

frames = {}
current_screen = "main"

def show_frame(name):
    global current_screen
    current_screen = name
    frames[name].tkraise()

BTN_W = 28
BTN_H = 2

# ----------------------------
# Main Screen
# ----------------------------

main_frame = tk.Frame(container, bg="#f0f0f0")
main_frame.grid(row=0, column=0, sticky="nsew")
frames["main"] = main_frame

tk.Label(main_frame, text="Robot Controller Menu", font=("Arial", 18), bg="#f0f0f0").pack(pady=20)

tk.Button(main_frame, text="Move Joints", font=("Arial", 14),
          bg="#4CAF50", fg="white", height=BTN_H, width=BTN_W,
          command=lambda: show_frame("joints")).pack(pady=10)

tk.Button(main_frame, text="Move Endeffector", font=("Arial", 14),
          bg="#607D8B", fg="white", height=BTN_H, width=BTN_W,
          command=lambda: show_frame("target")).pack(pady=10)

tk.Button(main_frame, text="Draw Circle", font=("Arial", 14),
          bg="#3F51B5", fg="white", height=BTN_H, width=BTN_W,
          command=lambda: show_frame("circle")).pack(pady=10)

tk.Button(main_frame, text="Go Home", font=("Arial", 14),
          bg="#9C27B0", fg="white", height=BTN_H, width=BTN_W,
          command=go_home).pack(pady=20)

# ----------------------------
# Joints Screen
# ----------------------------

joints_frame = tk.Frame(container, bg="#f0f0f0")
joints_frame.grid(row=0, column=0, sticky="nsew")
frames["joints"] = joints_frame

tk.Label(joints_frame, text="Hold < or > to rotate", font=("Arial", 16), bg="#f0f0f0").pack(pady=15)

rotation_speed = 0.05
repeat_ms = 30

rotate_dir = [0] * 6

def rotate_joint_step(index):
    d = rotate_dir[index]
    if d == 0:
        return
    targets[index] += d * rotation_speed
    targets[index] = clamp(targets[index], *limits[index])
    apply_targets()
    root.after(repeat_ms, lambda: rotate_joint_step(index))

def press_left(i, btn):
    rotate_dir[i] = -1
    btn.config(relief=tk.SUNKEN)
    rotate_joint_step(i)

def release_left(i, btn):
    if rotate_dir[i] == -1:
        rotate_dir[i] = 0
    btn.config(relief=tk.RAISED)

def press_right(i, btn):
    rotate_dir[i] = +1
    btn.config(relief=tk.SUNKEN)
    rotate_joint_step(i)

def release_right(i, btn):
    if rotate_dir[i] == +1:
        rotate_dir[i] = 0
    btn.config(relief=tk.RAISED)

for i in range(6):
    row = tk.Frame(joints_frame, bg="#f0f0f0")
    row.pack(pady=8)

    left_btn = tk.Button(row, text="<", font=("Arial", 16), width=4, bg="#4CAF50", fg="white")
    mid_lbl = tk.Label(row, text=f"Joint {i+1}", font=("Arial", 14), width=12, bg="#f0f0f0")
    right_btn = tk.Button(row, text=">", font=("Arial", 16), width=4, bg="#4CAF50", fg="white")

    left_btn.pack(side="left", padx=6)
    mid_lbl.pack(side="left", padx=6)
    right_btn.pack(side="left", padx=6)

    left_btn.bind("<ButtonPress-1>", lambda e, idx=i, b=left_btn: press_left(idx, b))
    left_btn.bind("<ButtonRelease-1>", lambda e, idx=i, b=left_btn: release_left(idx, b))

    right_btn.bind("<ButtonPress-1>", lambda e, idx=i, b=right_btn: press_right(idx, b))
    right_btn.bind("<ButtonRelease-1>", lambda e, idx=i, b=right_btn: release_right(idx, b))

tk.Button(joints_frame, text="Go Home", font=("Arial", 14),
          bg="#FF9800", fg="white", height=BTN_H, width=BTN_W,
          command=go_home).pack(pady=12)

tk.Button(joints_frame, text="Back to Menu", font=("Arial", 13),
          bg="#9E9E9E", fg="white", height=BTN_H, width=BTN_W,
          command=lambda: show_frame("main")).pack(pady=6)

# ----------------------------
# Circle Screen
# ----------------------------

circle_frame = tk.Frame(container, bg="#f0f0f0")
circle_frame.grid(row=0, column=0, sticky="nsew")
frames["circle"] = circle_frame

tk.Label(circle_frame, text="Draw Circle", font=("Arial", 16), bg="#f0f0f0").pack(pady=15)

circle_status = tk.Label(circle_frame, text="Status: idle", font=("Arial", 12), bg="#f0f0f0")
circle_status.pack(pady=8)

def circle_start_ui():
    start_circle_default()
    circle_status.config(text="Status: running" if circle_running else "Status: idle")

def circle_stop_ui():
    stop_circle()
    circle_status.config(text="Status: stopped")

tk.Button(circle_frame, text="Start Circle", font=("Arial", 14),
          bg="#3F51B5", fg="white", height=BTN_H, width=BTN_W,
          command=circle_start_ui).pack(pady=10)

tk.Button(circle_frame, text="Stop Circle", font=("Arial", 14),
          bg="#E91E63", fg="white", height=BTN_H, width=BTN_W,
          command=circle_stop_ui).pack(pady=10)

tk.Button(circle_frame, text="Go Home", font=("Arial", 14),
          bg="#9C27B0", fg="white", height=BTN_H, width=BTN_W,
          command=go_home).pack(pady=10)

tk.Button(circle_frame, text="Back to Menu", font=("Arial", 13),
          bg="#9E9E9E", fg="white", height=BTN_H, width=BTN_W,
          command=lambda: show_frame("main")).pack(pady=10)

# ----------------------------
# Target Screen 
# ----------------------------

target_frame = tk.Frame(container, bg="#f0f0f0")
target_frame.grid(row=0, column=0, sticky="nsew")
frames["target"] = target_frame

tk.Label(target_frame, text="Manual Endeffector (Follow TARGET)", font=("Arial", 16), bg="#f0f0f0").pack(pady=15)

tk.Label(
    target_frame,
    text=("Move the TARGET object manually in the 3D view."),
    font=("Arial", 11),
    bg="#f0f0f0"
).pack(pady=10)

target_pos_label = tk.Label(target_frame, text="Target position: (not found)", font=("Arial", 12), bg="#f0f0f0")
target_pos_label.pack(pady=8)

target_follow_label = tk.Label(target_frame, text="Follow: ON", font=("Arial", 12), bg="#f0f0f0")
target_follow_label.pack(pady=8)

tk.Button(target_frame, text="Go Home", font=("Arial", 14),
          bg="#9C27B0", fg="white", height=BTN_H, width=BTN_W,
          command=go_home).pack(pady=10)

tk.Button(target_frame, text="Back to Menu", font=("Arial", 13),
          bg="#9E9E9E", fg="white", height=BTN_H, width=BTN_W,
          command=lambda: show_frame("main")).pack(pady=10)

show_frame("main")

# ----------------------------
# Main loop
# ----------------------------

while supervisor.step(timestep) != -1:
    update_circle_loop()

    if circle_running:
        if "running" not in circle_status.cget("text"):
            circle_status.config(text="Status: running (default circle path)")
    else:
        if "running" in circle_status.cget("text"):
            circle_status.config(text="Status: idle")

    pos = get_target_abs_position()
    if pos is None:
        target_pos_label.config(text="Target position: (not found)")
    else:
        target_pos_label.config(text=f"Target position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

    update_target_follow_loop(current_screen, status_label=target_follow_label)

    try:
        root.update_idletasks()
        root.update()
    except tk.TclError:
        break
