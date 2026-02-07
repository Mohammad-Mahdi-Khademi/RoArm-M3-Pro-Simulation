# ==============================
# Windows OpenMP
# ==============================

import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"  
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import math
import time
import tempfile
import tkinter as tk
from tkinter import Toplevel, messagebox
from controller import Supervisor
from PIL import Image, ImageTk, ImageDraw, ImageFont

# YOLOv8n (Ultralytics)
YOLO_AVAILABLE = True
try:
    from ultralytics import YOLO
except Exception:
    YOLO_AVAILABLE = False
    YOLO = None

# torch thread cap 
try:
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
except Exception:
    pass

#==============
# Settings
#==============

IKPY_MAX_ITERATIONS = 35
FOLLOW_EVERY_N_STEPS = 2

MOTOR_NAMES = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_gripper_link",
]

ROBOT_DEF = "RoArm"
TARGET_DEF = "RoArmTARGET"

STARTUP_WARMUP_STEPS = 25

# YOLO settings
YOLO_WEIGHTS_PATH = "yolov8n.pt"
YOLO_IMGSZ = 320
YOLO_EVERY_N_FRAMES = 2           

#==============
# Webots setup
#==============

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

robot_node = supervisor.getFromDef(ROBOT_DEF)
target_node = supervisor.getFromDef(TARGET_DEF)

def is_finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)

def safe_value(x, fallback=0.0):
    return x if is_finite(x) else fallback

def clamp(x, lo, hi):
    if lo is None or hi is None:
        return x
    return max(lo, min(hi, x))

#==============
# Devices
#==============

motors = []
sensors = []
limits = []

for name in MOTOR_NAMES:
    m = supervisor.getDevice(name)
    s = supervisor.getDevice(name + "_sensor")
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

targets = [0.0] * len(motors)
home_positions = [0.0] * len(motors)

def apply_targets():
    for i, m in enumerate(motors):
        mn, mx = limits[i]
        targets[i] = clamp(safe_value(targets[i], 0.0), mn, mx)
        m.setPosition(targets[i])

def warmup_and_engage_motors(n_steps):
    for _ in range(max(1, n_steps)):
        if supervisor.step(timestep) == -1:
            return False

    for i in range(len(motors)):
        v = sensors[i].getValue()
        if is_finite(v):
            targets[i] = v
        else:
            mn, mx = limits[i]
            if mn is not None and mx is not None and mx >= mn:
                targets[i] = 0.5 * (mn + mx)
            else:
                targets[i] = 0.0

    apply_targets()

    for _ in range(3):
        if supervisor.step(timestep) == -1:
            return False

    return True

def capture_home_positions():
    global home_positions
    home_positions = []
    for i, s in enumerate(sensors):
        v = s.getValue()
        if is_finite(v):
            home_positions.append(v)
        else:
            home_positions.append(targets[i])

def go_home():
    global targets
    targets = home_positions.copy()
    apply_targets()

warmup_and_engage_motors(STARTUP_WARMUP_STEPS)
capture_home_positions()
go_home()

#==============
# Camera (Webots)
#==============

camera = None
try:
    camera = supervisor.getDevice("camera")
    camera.enable(timestep)
except Exception:
    camera = None

def webots_bgra_to_pil_rgb(img_bgra_bytes, w, h):
    im_rgba = Image.frombytes("RGBA", (w, h), img_bgra_bytes, "raw", "BGRA")
    return im_rgba.convert("RGB")

#==============
# IKPy setup
#==============

IKPY_AVAILABLE = True
armChain = None
joint_indices = []
last_ik = None

try:
    from ikpy.chain import Chain
except Exception:
    IKPY_AVAILABLE = False
    Chain = None

def load_chain_and_mapping():
    global armChain, joint_indices, last_ik
    armChain = None
    joint_indices = []
    last_ik = None

    if not IKPY_AVAILABLE:
        return

    urdf = supervisor.getUrdf()
    if not urdf:
        return

    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as f:
        f.write(urdf.encode("utf-8"))
        urdf_path = f.name

    chain = Chain.from_urdf_file(urdf_path)

    name_to_index = {link.name: i for i, link in enumerate(chain.links)}
    missing = [n for n in MOTOR_NAMES if n not in name_to_index]
    if missing:
        armChain = None
        joint_indices = []
        return

    joint_indices = [name_to_index[n] for n in MOTOR_NAMES]

    mask = [False] * len(chain.links)
    for idx in joint_indices:
        try:
            if getattr(chain.links[idx], "joint_type", None) == "fixed":
                continue
        except Exception:
            pass
        mask[idx] = True
    chain.active_links_mask = mask

    armChain = chain

load_chain_and_mapping()

def compute_initial_position():
    if armChain is None:
        return None

    init = [0.0] * len(armChain.links)

    if last_ik is not None and len(last_ik) == len(init):
        for i in range(len(init)):
            init[i] = safe_value(last_ik[i], 0.0)
        return init

    for mi, ci in enumerate(joint_indices):
        init[ci] = safe_value(targets[mi], 0.0)

    return init

def apply_ik_to_motors(x, y, z):
    global last_ik

    if armChain is None or not joint_indices:
        return False, None

    init = compute_initial_position()
    if init is None:
        return False, None

    try:
        ik = armChain.inverse_kinematics(
            [x, y, z],
            max_iter=IKPY_MAX_ITERATIONS,
            initial_position=init
        )
    except Exception:
        return False, None

    for ci in joint_indices:
        if not is_finite(ik[ci]):
            try:
                ik = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS)
            except Exception:
                return False, None
            break

    for mi, ci in enumerate(joint_indices):
        val = safe_value(ik[ci], targets[mi])
        motors[mi].setPosition(val)
        targets[mi] = val

    last_ik = ik
    return True, ik

#==============
# Transforms
#==============

def mat33_from_list(o):
    return [
        [o[0], o[1], o[2]],
        [o[3], o[4], o[5]],
        [o[6], o[7], o[8]],
    ]

def transpose33(m):
    return [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]

def mul33v(m, v):
    return [
        m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
        m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
        m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
    ]

def refresh_defs():
    global robot_node, target_node
    if robot_node is None:
        robot_node = supervisor.getFromDef(ROBOT_DEF)
    if target_node is None:
        target_node = supervisor.getFromDef(TARGET_DEF)

def world_to_robot_local(world_xyz):
    if robot_node is None:
        return None
    rp = robot_node.getPosition()
    rel_world = [world_xyz[0] - rp[0], world_xyz[1] - rp[1], world_xyz[2] - rp[2]]
    R_l2w = mat33_from_list(robot_node.getOrientation())
    R_w2l = transpose33(R_l2w)
    return mul33v(R_w2l, rel_world)

def get_target_local_and_world():
    if target_node is None:
        return None
    tp = target_node.getPosition()
    local = world_to_robot_local([tp[0], tp[1], tp[2]])
    if local is None:
        return None
    return local[0], local[1], local[2], tp

def ee_world_from_ik(ik):
    if armChain is None or ik is None or robot_node is None:
        return None, None
    M = armChain.forward_kinematics(ik)
    ee_local = [float(M[0, 3]), float(M[1, 3]), float(M[2, 3])]
    rp = robot_node.getPosition()
    R_l2w = mat33_from_list(robot_node.getOrientation())
    ee_world_rel = mul33v(R_l2w, ee_local)
    ee_world = [rp[0] + ee_world_rel[0], rp[1] + ee_world_rel[1], rp[2] + ee_world_rel[2]]
    return ee_local, ee_world

#==============
# Bootstrap IK from target
#==============

def bootstrap_ik_from_target(x, y, z):
    global last_ik

    if armChain is None or not joint_indices:
        return False

    init_candidates = []
    init_candidates.append([0.0] * len(armChain.links))

    init_mid = [0.0] * len(armChain.links)
    for mi, ci in enumerate(joint_indices):
        mn, mx = limits[mi]
        if mn is not None and mx is not None and mx >= mn:
            init_mid[ci] = 0.5 * (mn + mx)
        else:
            init_mid[ci] = 0.0
    init_candidates.append(init_mid)

    init_t = [0.0] * len(armChain.links)
    for mi, ci in enumerate(joint_indices):
        init_t[ci] = safe_value(targets[mi], 0.0)
    init_candidates.append(init_t)

    best_ik = None
    best_err = 1e9

    for init in init_candidates:
        try:
            ik = armChain.inverse_kinematics(
                [x, y, z],
                max_iter=IKPY_MAX_ITERATIONS,
                initial_position=init
            )
            M = armChain.forward_kinematics(ik)
            ex, ey, ez = float(M[0, 3]), float(M[1, 3]), float(M[2, 3])
            err = (ex - x) ** 2 + (ey - y) ** 2 + (ez - z) ** 2
        except Exception:
            continue

        if err < best_err:
            best_err = err
            best_ik = ik

    if best_ik is None:
        return False

    for mi, ci in enumerate(joint_indices):
        val = safe_value(best_ik[ci], targets[mi])
        motors[mi].setPosition(val)
        targets[mi] = val

    last_ik = best_ik
    return True

#==============
# Tkinter UI
#==============

root = tk.Tk()
root.title("RoArm Controller")
root.geometry("640x640")
root.configure(bg="#f0f0f0")
root.resizable(False, False)

frames = {}
current_screen = "main"

ee_control_mode = "follow"
manual_xyz = [0.0, 0.0, 0.0]
coord_mode_var = tk.StringVar(value="local")

def show_frame(name):
    global current_screen
    current_screen = name
    frames[name].tkraise()

def set_follow_mode():
    global ee_control_mode
    ee_control_mode = "follow"

def set_manual_mode():
    global ee_control_mode
    ee_control_mode = "manual"

def enter_ee_mode():
    show_frame("ee")
    refresh_defs()
    set_follow_mode()

    out = get_target_local_and_world()
    if out is None:
        return
    x, y, z, _ = out
    bootstrap_ik_from_target(x, y, z)

def move_endeffector_to_xyz_from_ui(x_entry, y_entry, z_entry, status_label=None):
    global ee_control_mode, manual_xyz

    try:
        x_in = float(x_entry.get())
        y_in = float(y_entry.get())
        z_in = float(z_entry.get())
    except Exception:
        if status_label is not None:
            status_label.config(text="Status: invalid x,y,z", fg="#B00020")
        return

    refresh_defs()

    if coord_mode_var.get() == "world":
        local = world_to_robot_local([x_in, y_in, z_in])
        if local is None:
            if status_label is not None:
                status_label.config(text="Status: robot DEF not ready", fg="#B00020")
            return
        x, y, z = local[0], local[1], local[2]
    else:
        x, y, z = x_in, y_in, z_in

    manual_xyz = [x, y, z]
    ee_control_mode = "manual"

    ok, _ = apply_ik_to_motors(x, y, z)
    if status_label is not None:
        if ok:
            status_label.config(text="Status: MANUAL (holding x,y,z)", fg="#1B5E20")
        else:
            status_label.config(text="Status: MANUAL (IK failed)", fg="#B00020")

container = tk.Frame(root, bg="#f0f0f0")
container.pack(fill="both", expand=True)
container.grid_rowconfigure(0, weight=1)
container.grid_columnconfigure(0, weight=1)

BTN_W = 18
BTN_H = 2

#==============
# YOLO model
#==============

yolo_model = None
yolo_enabled = True
_yolo_frame_counter = 0
_yolo_last_boxes = []  
yolo_conf_var = tk.DoubleVar(value=0.35)

def ensure_yolo_loaded():
    global yolo_model
    if not YOLO_AVAILABLE:
        return False
    if yolo_model is None:
        try:
            yolo_model = YOLO(YOLO_WEIGHTS_PATH)
        except Exception as e:
            yolo_model = None
            messagebox.showerror(
                "YOLO",
                "Failed to load YOLOv8n.\n"
                f"Make sure '{YOLO_WEIGHTS_PATH}' exists and Ultralytics is installed.\n\n"
                f"Error: {e}"
            )
            return False
    return True

def detect_bottles_pil(im_rgb):
    global _yolo_frame_counter, _yolo_last_boxes

    if not yolo_enabled:
        _yolo_last_boxes = []
        return _yolo_last_boxes

    if not ensure_yolo_loaded():
        _yolo_last_boxes = []
        return _yolo_last_boxes

    _yolo_frame_counter += 1
    if _yolo_frame_counter % YOLO_EVERY_N_FRAMES != 0:
        return _yolo_last_boxes

    conf_thr = float(yolo_conf_var.get())

    try:
        results = yolo_model.predict(
            source=im_rgb,
            imgsz=YOLO_IMGSZ,
            conf=conf_thr,
            verbose=False,
            device="cpu"
        )
    except Exception:
        return _yolo_last_boxes

    boxes_out = []
    if not results:
        _yolo_last_boxes = boxes_out
        return boxes_out

    r0 = results[0]
    names = getattr(r0, "names", None)
    boxes = getattr(r0, "boxes", None)
    if boxes is None:
        _yolo_last_boxes = boxes_out
        return boxes_out
    
    for b in boxes:
        try:
            cls_id = int(b.cls[0].item()) if hasattr(b, "cls") else None
            conf = float(b.conf[0].item()) if hasattr(b, "conf") else 0.0
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]

            is_bottle = False
            if names and cls_id is not None and cls_id in names:
                is_bottle = (str(names[cls_id]).lower() == "bottle")
            else:
                is_bottle = (cls_id == 39)

            if is_bottle:
                boxes_out.append((x1, y1, x2, y2, conf))
        except Exception:
            continue

    _yolo_last_boxes = boxes_out
    return boxes_out

#==============
# Boxing UI
#==============

def draw_boxes(im_rgb, boxes, fps_text=None):
    if not boxes and not fps_text:
        return im_rgb

    AA = 3
    W, H = im_rgb.size
    base = im_rgb.convert("RGBA")
    base_hi = base.resize((W * AA, H * AA), Image.BILINEAR)

    overlay_hi = Image.new("RGBA", base_hi.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(overlay_hi)

    try:
        font = ImageFont.truetype("arial.ttf", 14 * AA)
    except Exception:
        font = ImageFont.load_default()

    if fps_text:
        pad = 8 * AA
        x0, y0 = 10 * AA, 10 * AA
        bbox = d.textbbox((0, 0), fps_text, font=font)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        x1, y1 = x0 + tw + pad * 2, y0 + th + pad * 2
        r = 12 * AA
        d.rounded_rectangle([x0, y0, x1, y1], radius=r, fill=(0, 0, 0, 140))
        d.text((x0 + pad, y0 + pad), fps_text, font=font, fill=(255, 255, 255, 230))

    outline = (0, 255, 120, 230)
    w = 3 * AA
    radius = 18 * AA
    pad_label = 8 * AA

    for (x1, y1, x2, y2, conf) in boxes:
        x1 = int(max(0, min(W - 1, x1)) * AA)
        y1 = int(max(0, min(H - 1, y1)) * AA)
        x2 = int(max(0, min(W - 1, x2)) * AA)
        y2 = int(max(0, min(H - 1, y2)) * AA)
        if x2 <= x1 or y2 <= y1:
            continue

        d.rounded_rectangle([x1, y1, x2, y2], radius=radius, outline=outline, width=w)

        label = f"bottle  {conf:.2f}"
        tb = d.textbbox((0, 0), label, font=font)
        tw, th = tb[2] - tb[0], tb[3] - tb[1]
        pill_w = tw + pad_label * 2
        pill_h = th + pad_label * 2

        lx = x1
        ly = max(0, y1 - pill_h - 6 * AA)

        d.rounded_rectangle([lx, ly, lx + pill_w, ly + pill_h], radius=12 * AA, fill=(0, 0, 0, 170))
        d.text((lx + pad_label, ly + pad_label), label, font=font, fill=(255, 255, 255, 240))

        bar_x0 = lx + pad_label
        bar_x1 = lx + pill_w - pad_label

        bar_y0 = ly + pill_h - 2 * AA
        bar_y1 = bar_y0 + 3 * AA

        bar_r = (bar_y1 - bar_y0) // 2

        d.rounded_rectangle([bar_x0, bar_y0, bar_x1, bar_y1], radius=bar_r, fill=(255, 255, 255, 80))

        conf_c = max(0.0, min(1.0, float(conf)))
        fill_w = int((bar_x1 - bar_x0) * conf_c)

        if fill_w > 0:
            fx1 = min(bar_x0 + max(fill_w, (bar_y1 - bar_y0)), bar_x1)
            d.rounded_rectangle([bar_x0, bar_y0, fx1, bar_y1], radius=bar_r, fill=(0, 255, 120, 200))

    out_hi = Image.alpha_composite(base_hi, overlay_hi)
    out = out_hi.resize((W, H), Image.LANCZOS).convert("RGB")
    return out

#==============
# Camera viewer
#==============

camera_win = None
camera_label = None
camera_photo = None
camera_running = False
camera_after_id = None
camera_status = None
fps_label = None

_last_frame_time = None
_fps_smoothed = 0.0

def toggle_yolo():
    global yolo_enabled
    yolo_enabled = not yolo_enabled
    if camera_status is not None:
        camera_status.config(text=f"YOLO: {'ON' if yolo_enabled else 'OFF'}")

def open_camera_viewer():
    global camera_win, camera_label, camera_running, camera_status, fps_label, camera_after_id
    global _last_frame_time, _fps_smoothed

    if camera is None:
        messagebox.showerror("Camera", "Camera device 'camera' not found.")
        return

    if camera_win is not None and camera_win.winfo_exists():
        camera_win.lift()
        return

    camera_win = Toplevel(root)
    camera_win.title("Camera View ")
    camera_win.configure(bg="#000000")

    topbar = tk.Frame(camera_win, bg="#111111")
    topbar.pack(fill="x")

    camera_status = tk.Label(topbar, text=f"YOLO: {'ON' if yolo_enabled else 'OFF'}", fg="white", bg="#111111")
    camera_status.pack(side="left", padx=8, pady=6)

    fps_label = tk.Label(topbar, text="FPS: --", fg="#CCCCCC", bg="#111111")
    fps_label.pack(side="left", padx=10, pady=6)

    tk.Label(topbar, text="Conf", fg="#CCCCCC", bg="#111111").pack(side="left", padx=(12, 4))
    conf_slider = tk.Scale(
        topbar, from_=0.05, to=0.90, resolution=0.01,
        orient="horizontal", variable=yolo_conf_var,
        length=160, bg="#111111", fg="#CCCCCC",
        highlightthickness=0, troughcolor="#333333"
    )
    conf_slider.pack(side="left", pady=6)

    tk.Button(
        topbar, text="Toggle YOLO", command=toggle_yolo,
        bg="#444444", fg="white"
    ).pack(side="right", padx=8, pady=6)

    camera_label = tk.Label(camera_win, bg="#000000")
    camera_label.pack(padx=8, pady=8)

    camera_running = True
    camera_after_id = None

    _last_frame_time = None
    _fps_smoothed = 0.0

    def on_close():
        global camera_running, camera_win, camera_after_id
        camera_running = False
        if camera_after_id is not None:
            try:
                camera_win.after_cancel(camera_after_id)
            except Exception:
                pass
            camera_after_id = None
        if camera_win is not None and camera_win.winfo_exists():
            camera_win.destroy()
        camera_win = None

    camera_win.protocol("WM_DELETE_WINDOW", on_close)
    update_camera_frame()

def update_camera_frame():
    global camera_photo, camera_running, camera_after_id
    global _last_frame_time, _fps_smoothed

    if not camera_running:
        return
    if camera is None or camera_win is None or (not camera_win.winfo_exists()):
        return

    now = time.time()
    if _last_frame_time is None:
        inst_fps = 0.0
    else:
        dt = max(1e-6, now - _last_frame_time)
        inst_fps = 1.0 / dt
    _last_frame_time = now
    _fps_smoothed = 0.90 * _fps_smoothed + 0.10 * inst_fps if _fps_smoothed > 0 else inst_fps

    if fps_label is not None:
        fps_label.config(text=f"FPS: {(_fps_smoothed):.1f}")

    w = camera.getWidth()
    h = camera.getHeight()

    img_bgra = camera.getImage()
    if img_bgra:
        im_rgb = webots_bgra_to_pil_rgb(img_bgra, w, h)

        boxes = detect_bottles_pil(im_rgb)

        im_out = draw_boxes(im_rgb, boxes)
        
        im_out = im_out.resize((w, h), Image.LANCZOS)

        camera_photo = ImageTk.PhotoImage(im_out)
        camera_label.config(image=camera_photo)

    try:
        camera_after_id = camera_win.after(33, update_camera_frame) 
    except tk.TclError:
        camera_running = False
        camera_after_id = None

#==============
# Main screen
#==============

main_frame = tk.Frame(container, bg="#f0f0f0")
main_frame.grid(row=0, column=0, sticky="nsew")
frames["main"] = main_frame

tk.Label(main_frame, text="RoArm Controller", font=("Arial", 20), bg="#f0f0f0").pack(pady=30)

tk.Button(
    main_frame, text="Move Joints",
    font=("Arial", 16), bg="#4CAF50", fg="white",
    width=BTN_W, height=BTN_H,
    command=lambda: show_frame("joints")
).pack(pady=10)

tk.Button(
    main_frame, text="Move Endeffector",
    font=("Arial", 16), bg="#607D8B", fg="white",
    width=BTN_W, height=BTN_H,
    command=enter_ee_mode
).pack(pady=10)

tk.Button(
    main_frame, text="Camera",
    font=("Arial", 16), bg="#2196F3", fg="white",
    width=BTN_W, height=BTN_H,
    command=open_camera_viewer
).pack(pady=10)

tk.Button(
    main_frame, text="Go Home",
    font=("Arial", 16), bg="#9C27B0", fg="white",
    width=BTN_W, height=BTN_H,
    command=go_home
).pack(pady=10)

#==============
# Joints screen
#==============

joints_frame = tk.Frame(container, bg="#f0f0f0")
joints_frame.grid(row=0, column=0, sticky="nsew")
frames["joints"] = joints_frame

tk.Label(joints_frame, text="Move Joints", font=("Arial", 18), bg="#f0f0f0").pack(pady=16)

rows_container = tk.Frame(joints_frame, bg="#f0f0f0")
rows_container.pack(pady=8)

rotation_speed = 0.02
repeat_ms = 30
rotate_dir = [0] * len(MOTOR_NAMES)

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

for i, name in enumerate(MOTOR_NAMES):
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

bottom_j = tk.Frame(joints_frame, bg="#f0f0f0")
bottom_j.pack(pady=18)

tk.Button(
    bottom_j, text="Go Home",
    font=("Arial", 13), bg="#9C27B0", fg="white",
    width=14, height=2, command=go_home
).pack(side="left", padx=10)

tk.Button(
    bottom_j, text="Back to Menu",
    font=("Arial", 13), bg="#9E9E9E", fg="white",
    width=14, height=2, command=lambda: show_frame("main")
).pack(side="left", padx=10)

#==============
# Endeffector screen
#==============

ee_frame = tk.Frame(container, bg="#f0f0f0")
ee_frame.grid(row=0, column=0, sticky="nsew")
frames["ee"] = ee_frame

tk.Label(ee_frame, text="Move Endeffector", font=("Arial", 18), bg="#f0f0f0").pack(pady=16)

ee_status = tk.Label(ee_frame, text="Status: idle", font=("Arial", 12), bg="#f0f0f0")
ee_status.pack(pady=8)

ee_target_label = tk.Label(ee_frame, text="", font=("Arial", 11), bg="#f0f0f0")
ee_target_label.pack(pady=6)

ee_local_label = tk.Label(ee_frame, text="", font=("Arial", 10), bg="#f0f0f0")
ee_local_label.pack(pady=4)

ee_ee_label = tk.Label(ee_frame, text="", font=("Arial", 10), bg="#f0f0f0")
ee_ee_label.pack(pady=6)

mode_row = tk.Frame(ee_frame, bg="#f0f0f0")
mode_row.pack(pady=6)

tk.Button(
    mode_row, text="Follow TARGET",
    font=("Arial", 11), bg="#3F51B5", fg="white",
    width=14, command=set_follow_mode
).pack(side="left", padx=6)

tk.Button(
    mode_row, text="Manual (x,y,z)",
    font=("Arial", 11), bg="#607D8B", fg="white",
    width=14, command=set_manual_mode
).pack(side="left", padx=6)

coord_row = tk.Frame(ee_frame, bg="#f0f0f0")
coord_row.pack(pady=4)

tk.Radiobutton(coord_row, text="Local", variable=coord_mode_var, value="local", bg="#f0f0f0").pack(side="left", padx=8)
tk.Radiobutton(coord_row, text="World", variable=coord_mode_var, value="world", bg="#f0f0f0").pack(side="left", padx=8)

entry_frame = tk.Frame(ee_frame, bg="#f0f0f0")
entry_frame.pack(pady=8)

tk.Label(entry_frame, text="x:", bg="#f0f0f0").grid(row=0, column=0, padx=4, pady=3, sticky="e")
tk.Label(entry_frame, text="y:", bg="#f0f0f0").grid(row=1, column=0, padx=4, pady=3, sticky="e")
tk.Label(entry_frame, text="z:", bg="#f0f0f0").grid(row=2, column=0, padx=4, pady=3, sticky="e")

x_entry = tk.Entry(entry_frame, width=12, font=("Arial", 11))
y_entry = tk.Entry(entry_frame, width=12, font=("Arial", 11))
z_entry = tk.Entry(entry_frame, width=12, font=("Arial", 11))

x_entry.grid(row=0, column=1, padx=4, pady=3)
y_entry.grid(row=1, column=1, padx=4, pady=3)
z_entry.grid(row=2, column=1, padx=4, pady=3)

tk.Button(
    ee_frame, text="Move to x,y,z",
    font=("Arial", 12), bg="#4CAF50", fg="white",
    width=16, height=2,
    command=lambda: move_endeffector_to_xyz_from_ui(x_entry, y_entry, z_entry, status_label=ee_status)
).pack(pady=10)

tk.Label(
    ee_frame,
    text=f"Move DEF {TARGET_DEF} in the scene (FOLLOW mode).\nOr enter x,y,z (MANUAL mode).",
    font=("Arial", 11),
    bg="#f0f0f0"
).pack(pady=8)

tk.Button(
    ee_frame, text="Back to Menu",
    font=("Arial", 13), bg="#9E9E9E", fg="white",
    width=14, height=2, command=lambda: show_frame("main")
).pack(pady=18)

show_frame("main")

#==============
# Main loop
#==============

follow_counter = 0

while supervisor.step(timestep) != -1:
    if current_screen == "ee":
        refresh_defs()

        if not IKPY_AVAILABLE:
            ee_status.config(text="Status: IKPy not installed", fg="#B00020")
        elif robot_node is None:
            ee_status.config(text=f"Status: DEF {ROBOT_DEF} not found", fg="#B00020")
        elif armChain is None or not joint_indices:
            ee_status.config(text="Status: IK chain mapping failed", fg="#B00020")
        else:
            if ee_control_mode == "manual":
                x, y, z = manual_xyz
                ok, ik = apply_ik_to_motors(x, y, z)
                if ok:
                    ee_status.config(text="Status: MANUAL (holding x,y,z)", fg="#1B5E20")
                    ee_local, ee_world = ee_world_from_ik(ik)
                    if ee_local is not None and ee_world is not None:
                        ee_ee_label.config(
                            text=(
                                f"EE local: x={ee_local[0]:.3f}, y={ee_local[1]:.3f}, z={ee_local[2]:.3f}\n"
                                f"EE world: x={ee_world[0]:.3f}, y={ee_world[1]:.3f}, z={ee_world[2]:.3f}"
                            )
                        )
                else:
                    ee_status.config(text="Status: MANUAL (IK failed / unreachable)", fg="#B00020")
            else:
                if target_node is None:
                    ee_status.config(text=f"Status: DEF {TARGET_DEF} not found", fg="#B00020")
                else:
                    follow_counter += 1
                    if follow_counter % FOLLOW_EVERY_N_STEPS == 0:
                        out = get_target_local_and_world()
                        if out is None:
                            ee_status.config(text="Status: target/robot not ready", fg="#B00020")
                        else:
                            x, y, z, tp = out
                            ee_target_label.config(text=f"Target world: x={tp[0]:.3f}, y={tp[1]:.3f}, z={tp[2]:.3f}")
                            ee_local_label.config(text=f"Target local: x={x:.3f}, y={y:.3f}, z={z:.3f}")

                            ok, ik = apply_ik_to_motors(x, y, z)
                            if ok:
                                ee_status.config(text="Status: FOLLOWING", fg="#1B5E20")
                                ee_local2, ee_world2 = ee_world_from_ik(ik)
                                if ee_local2 is not None and ee_world2 is not None:
                                    ee_ee_label.config(
                                        text=(
                                            f"EE local: x={ee_local2[0]:.3f}, y={ee_local2[1]:.3f}, z={ee_local2[2]:.3f}\n"
                                            f"EE world: x={ee_world2[0]:.3f}, y={ee_world2[1]:.3f}, z={ee_world2[2]:.3f}"
                                        )
                                    )
                            else:
                                ee_status.config(text="Status: IK failed / unreachable", fg="#B00020")

    try:
        root.update_idletasks()
        root.update()
    except tk.TclError:
        break
