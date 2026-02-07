from controller import Supervisor
import tkinter as tk
import math

# =========================
# Webots setup
# =========================

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

motor_names = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_gripper_link",
]
sensor_names = [mn + "_sensor" for mn in motor_names]

def is_finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)

def safe_value(x, fallback=0.0):
    return x if is_finite(x) else fallback

def clamp(x, lo, hi):
    if lo is None or hi is None:
        return x
    return max(lo, min(hi, x))

motors, sensors, limits = [], [], []

for mn, sn in zip(motor_names, sensor_names):
    m = supervisor.getDevice(mn)
    s = supervisor.getDevice(sn)
    s.enable(timestep)
    m.setVelocity(min(2.0, m.getMaxVelocity()))
    motors.append(m)
    sensors.append(s)

supervisor.step(timestep)

for m in motors:
    mn = m.getMinPosition()
    mx = m.getMaxPosition()
    if not is_finite(mn) or not is_finite(mx) or mx < mn:
        limits.append((None, None))
    else:
        limits.append((mn, mx))

home_positions = [safe_value(s.getValue(), 0.0) for s in sensors]
targets = home_positions.copy()

def apply_targets():
    for i, m in enumerate(motors):
        t = safe_value(targets[i], 0.0)
        mn, mx = limits[i]
        targets[i] = clamp(t, mn, mx)
        m.setPosition(targets[i])

def go_home():
    global targets
    targets = home_positions.copy()
    apply_targets()

apply_targets()

# =========================
# Tkinter UI
# =========================
root = tk.Tk()
root.title("RoArm Controller")
root.geometry("560x560")
root.configure(bg="#f0f0f0")
root.resizable(False, False)

frames = {}
current_screen = "main"

def show_frame(name):
    global current_screen
    current_screen = name
    frames[name].tkraise()

container = tk.Frame(root, bg="#f0f0f0")
container.pack(fill="both", expand=True)
container.grid_rowconfigure(0, weight=1)
container.grid_columnconfigure(0, weight=1)

# -------------------------
# Main Menu
# -------------------------

main_frame = tk.Frame(container, bg="#f0f0f0")
main_frame.grid(row=0, column=0, sticky="nsew")
frames["main"] = main_frame

tk.Label(main_frame, text="RoArm Controller", font=("Arial", 20), bg="#f0f0f0").pack(pady=30)

tk.Button(
    main_frame,
    text="Move Joints",
    font=("Arial", 16),
    bg="#4CAF50",
    fg="white",
    width=18,
    height=2,
    command=lambda: show_frame("joints"),
).pack(pady=10)

tk.Button(
    main_frame,
    text="Go Home",
    font=("Arial", 16),
    bg="#9C27B0",
    fg="white",
    width=18,
    height=2,
    command=go_home,
).pack(pady=10)

# -------------------------
# Move Joints Screen
# -------------------------

joints_frame = tk.Frame(container, bg="#f0f0f0")
joints_frame.grid(row=0, column=0, sticky="nsew")
frames["joints"] = joints_frame

tk.Label(joints_frame, text="Move Joints", font=("Arial", 18), bg="#f0f0f0").pack(pady=16)

rows_container = tk.Frame(joints_frame, bg="#f0f0f0")
rows_container.pack(pady=8)

rotation_speed = 0.02
repeat_ms = 30
rotate_dir = [0] * len(motor_names)

def rotate_joint_step(index):
    d = rotate_dir[index]
    if d == 0:
        return
    targets[index] = safe_value(targets[index], 0.0) + d * rotation_speed
    mn, mx = limits[index]
    targets[index] = clamp(targets[index], mn, mx)
    apply_targets()
    root.after(repeat_ms, lambda: rotate_joint_step(index))

def start_rotate(index, direction):
    rotate_dir[index] = direction
    rotate_joint_step(index)

def stop_rotate(index):
    rotate_dir[index] = 0

for i, name in enumerate(motor_names):
    row = tk.Frame(rows_container, bg="#f0f0f0")
    row.pack(pady=8)

    tk.Label(row, text=name, font=("Arial", 11), width=22, anchor="w", bg="#f0f0f0").pack(side="left")

    btn_minus = tk.Button(row, text="-", width=5, font=("Arial", 12), bg="#FF7043", fg="white")
    btn_minus.pack(side="left", padx=6)
    btn_minus.bind("<ButtonPress-1>", lambda e, idx=i: start_rotate(idx, -1))
    btn_minus.bind("<ButtonRelease-1>", lambda e, idx=i: stop_rotate(idx))
    btn_minus.bind("<Leave>", lambda e, idx=i: stop_rotate(idx))

    btn_plus = tk.Button(row, text="+", width=5, font=("Arial", 12), bg="#66BB6A", fg="white")
    btn_plus.pack(side="left", padx=6)
    btn_plus.bind("<ButtonPress-1>", lambda e, idx=i: start_rotate(idx, 1))
    btn_plus.bind("<ButtonRelease-1>", lambda e, idx=i: stop_rotate(idx))
    btn_plus.bind("<Leave>", lambda e, idx=i: stop_rotate(idx))

bottom = tk.Frame(joints_frame, bg="#f0f0f0")
bottom.pack(pady=18)

tk.Button(
    bottom,
    text="Go Home",
    font=("Arial", 13),
    bg="#9C27B0",
    fg="white",
    width=14,
    height=2,
    command=go_home,
).pack(side="left", padx=10)

tk.Button(
    bottom,
    text="Back to Menu",
    font=("Arial", 13),
    bg="#9E9E9E",
    fg="white",
    width=14,
    height=2,
    command=lambda: show_frame("main"),
).pack(side="left", padx=10)


show_frame("main")

# =========================
# Main loop
# =========================

while supervisor.step(timestep) != -1:
    try:
        root.update_idletasks()
        root.update()
    except tk.TclError:
        break
