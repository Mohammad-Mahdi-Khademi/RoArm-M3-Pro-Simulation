from controller import Supervisor
import tkinter as tk
from tkinter import messagebox
import math
import tempfile

# =============================================================================
# Webots runtime: devices, timing, and joint state
# =============================================================================

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

motor_names = ["A motor", "B motor", "C motor", "D motor", "E motor", "F motor"]
sensor_names = ["A sensor", "B sensor", "C sensor", "D sensor", "E sensor", "F sensor"]

motors = []
sensors = []

for mn, sn in zip(motor_names, sensor_names):
    motor = supervisor.getDevice(mn)
    sensor = supervisor.getDevice(sn)

    sensor.enable(timestep)
    motor.setVelocity(1.5)

    motors.append(motor)
    sensors.append(sensor)


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
    for i, motor in enumerate(motors):
        motor.setPosition(targets[i])


# =============================================================================
# Drawing tool
# =============================================================================

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

# =============================================================================
# Inverse kinematics: load IKPy chain from the URDF provided by Webots
# =============================================================================

IKPY_AVAILABLE = False
arm_chain = None
IKPY_MAX_ITERATIONS = 20

try:
    from ikpy.chain import Chain

    IKPY_AVAILABLE = True
except Exception:
    IKPY_AVAILABLE = False


def load_ik_chain_from_urdf():
    global arm_chain

    if not IKPY_AVAILABLE:
        arm_chain = None
        return

    urdf = supervisor.getUrdf()
    if not urdf:
        arm_chain = None
        return

    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    tmp.write(urdf.encode("utf-8"))
    tmp.flush()
    tmp.close()

    try:
        arm_chain = Chain.from_urdf_file(tmp.name)
    except Exception:
        arm_chain = None


load_ik_chain_from_urdf()


def current_ik_initial_position():
    q = [0.0] + [s.getValue() for s in sensors]
    if arm_chain is not None:
        while len(q) < len(arm_chain.links):
            q.append(0.0)
        if len(q) > len(arm_chain.links):
            q = q[: len(arm_chain.links)]
    return q


# =============================================================================
# Scene anchors: TARGET and the robot node itself
# =============================================================================

target_def_name = "TARGET"
target_node = supervisor.getFromDef(target_def_name)
arm_node = supervisor.getSelf()


paper_def_name = "PAPER"
paper_node = supervisor.getFromDef(paper_def_name)
paper_center_fallback_world = (-5.8, 1.1, 0.741)

def get_paper_center_world():
    if paper_node is not None:
        p = paper_node.getPosition()
        return (p[0], p[1], p[2])
    return paper_center_fallback_world

def world_to_robot_xyz(wx, wy, wz):
    armPosition = arm_node.getPosition()
    x = -(wy - armPosition[1])
    y = wx - armPosition[0]
    z = wz - armPosition[2]
    return x, y, z

circle_hover_z_offset = 0.2
circle_draw_z_offset = 0.04
circle_approach_time = 1.0
circle_lower_time = 2.0
circle_lift_time = 1.0

def get_target_abs_position():
    if target_node is None:
        return None
    p = target_node.getPosition()
    return (p[0], p[1], p[2])

# =============================================================================
# Motion primitive: timed circle drawing 
# =============================================================================

circle_running = False
circle_start_time = 0.0
circle_center_xy = None
circle_paper_z = None
circle_radius = 0.25
circle_period = 10.0


def start_circle_default():
    global circle_running, circle_start_time, circle_center_xy, circle_paper_z
    if not IKPY_AVAILABLE:
        messagebox.showerror("Draw Circle", "Failed.")
        return
    wx, wy, wz = get_paper_center_world()
    cx, cy, cz = world_to_robot_xyz(wx, wy, wz)
    circle_center_xy = (cx, cy)
    circle_paper_z = cz
    circle_running = True
    circle_start_time = supervisor.getTime()
    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass
    hover_z = circle_paper_z + circle_hover_z_offset
    _apply_ik_to_motors(cx + circle_radius, cy, hover_z)


def stop_circle():
    global circle_running, circle_center_xy, circle_paper_z
    circle_running = False
    circle_center_xy = None
    circle_paper_z = None
    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass

def update_circle_loop():
    global circle_running, circle_center_xy, circle_paper_z
    if not circle_running:
        return
    if circle_center_xy is None or circle_paper_z is None:
        stop_circle()
        return
    t = supervisor.getTime() - circle_start_time
    cx, cy = circle_center_xy
    hover_z = circle_paper_z + circle_hover_z_offset
    draw_z = circle_paper_z + circle_draw_z_offset
    t1 = circle_approach_time
    t2 = t1 + circle_lower_time
    t3 = t2 + circle_period
    t4 = t3 + circle_lift_time
    pen_on = False
    if t < t1:
        x = cx + circle_radius
        y = cy
        z = hover_z
    elif t < t2:
        u = (t - t1) / max(1e-6, (t2 - t1))
        x = cx + circle_radius
        y = cy
        z = hover_z + u * (draw_z - hover_z)
    elif t < t3:
        theta = (t - t2) * (2 * math.pi / circle_period)
        x = cx + circle_radius * math.cos(theta)
        y = cy + circle_radius * math.sin(theta)
        z = draw_z
        pen_on = True
    elif t < t4:
        u = (t - t3) / max(1e-6, (t4 - t3))
        x = cx + circle_radius
        y = cy
        z = draw_z + u * (hover_z - draw_z)
    else:
        stop_circle()
        return
    if pen is not None:
        try:
            pen.write(pen_on)
        except Exception:
            pass
    _apply_ik_to_motors(x, y, z)
    
    
# =============================================================================
# Motion primitive: timed polyline drawing (square / rectangle)
# =============================================================================

poly_running = False
poly_start_time = 0.0
poly_points_xy = None          # list[(x,y)] in robot coords
poly_paper_z = None
poly_total_len = 0.0
poly_cumlen = None             # cumulative lengths
poly_shape_name = ""

POLY_DRAW_SPEED = 0.08         # m/s (drawing speed)
SQUARE_SIDE = 0.30             # meters
RECT_W = 0.40                  # meters
RECT_H = 0.25                  # meters


def _poly_prepare(points_xy):
    cum = [0.0]
    total = 0.0
    for i in range(len(points_xy) - 1):
        x0, y0 = points_xy[i]
        x1, y1 = points_xy[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        total += seg
        cum.append(total)
    return total, cum


def start_polyline(points_xy, shape_name="shape"):
    global poly_running, poly_start_time, poly_points_xy, poly_paper_z
    global poly_total_len, poly_cumlen, poly_shape_name

    if not IKPY_AVAILABLE:
        messagebox.showerror(f"Draw {shape_name.title()}", "Failed.")
        return
    
    stop_circle()

    wx, wy, wz = get_paper_center_world()
    cx, cy, cz = world_to_robot_xyz(wx, wy, wz)

    abs_pts = [(cx + px, cy + py) for (px, py) in points_xy]
    if abs_pts[0] != abs_pts[-1]:
        abs_pts.append(abs_pts[0])

    poly_total_len, poly_cumlen = _poly_prepare(abs_pts)
    poly_points_xy = abs_pts
    poly_paper_z = cz
    poly_shape_name = shape_name

    poly_running = True
    poly_start_time = supervisor.getTime()

    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass

    hover_z = poly_paper_z + circle_hover_z_offset
    x0, y0 = poly_points_xy[0]
    _apply_ik_to_motors(x0, y0, hover_z)


def stop_polyline():
    global poly_running, poly_points_xy, poly_paper_z, poly_cumlen, poly_shape_name
    poly_running = False
    poly_points_xy = None
    poly_paper_z = None
    poly_cumlen = None
    poly_shape_name = ""
    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass


def _poly_point_at_distance(s):
    if poly_points_xy is None or poly_cumlen is None:
        return None

    s = max(0.0, min(poly_total_len, s))

    for i in range(len(poly_cumlen) - 1):
        if poly_cumlen[i] <= s <= poly_cumlen[i + 1]:
            x0, y0 = poly_points_xy[i]
            x1, y1 = poly_points_xy[i + 1]
            seg_len = max(1e-9, (poly_cumlen[i + 1] - poly_cumlen[i]))
            u = (s - poly_cumlen[i]) / seg_len
            return (x0 + u * (x1 - x0), y0 + u * (y1 - y0))
    return poly_points_xy[-1]


def update_polyline_loop():
    global poly_running

    if not poly_running:
        return
    if poly_points_xy is None or poly_paper_z is None or poly_total_len <= 1e-9:
        stop_polyline()
        return

    t = supervisor.getTime() - poly_start_time

    hover_z = poly_paper_z + circle_hover_z_offset
    draw_z = poly_paper_z + circle_draw_z_offset

    t1 = circle_approach_time
    t2 = t1 + circle_lower_time

    draw_time = poly_total_len / max(1e-6, POLY_DRAW_SPEED)
    t3 = t2 + draw_time
    t4 = t3 + circle_lift_time

    pen_on = False

    if t < t1:
        x, y = poly_points_xy[0]
        z = hover_z
    elif t < t2:
        u = (t - t1) / max(1e-6, (t2 - t1))
        x, y = poly_points_xy[0]
        z = hover_z + u * (draw_z - hover_z)
    elif t < t3:
        s = (t - t2) * POLY_DRAW_SPEED
        p = _poly_point_at_distance(s)
        if p is None:
            stop_polyline()
            return
        x, y = p
        z = draw_z
        pen_on = True
    elif t < t4:
        u = (t - t3) / max(1e-6, (t4 - t3))
        x, y = poly_points_xy[0]
        z = draw_z + u * (hover_z - draw_z)
    else:
        stop_polyline()
        return

    if pen is not None:
        try:
            pen.write(pen_on)
        except Exception:
            pass

    _apply_ik_to_motors(x, y, z)


def start_square_default():
    h = SQUARE_SIDE / 2.0
    pts = [(-h, -h), (h, -h), (h, h), (-h, h)]
    start_polyline(pts, shape_name="square")


def start_rectangle_default():
    hw = RECT_W / 2.0
    hh = RECT_H / 2.0
    pts = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    start_polyline(pts, shape_name="rectangle")
    
    
    
# =============================================================================
# Imitate (freehand) drawing
# =============================================================================

imitate_running = False
imitate_start_time = 0.0
imitate_points_xy = None      # list[(x,y)] absolute robot coords
imitate_paper_z = None
imitate_total_len = 0.0
imitate_cumlen = None

IMITATE_DRAW_SPEED = 0.10     # m/s
IMITATE_CANVAS_W = 520        # px
IMITATE_CANVAS_H = 360        # px

# Physical size of drawing area on the paper
IMITATE_AREA_W = 0.45         # meters (left-right span)
IMITATE_AREA_H = 0.30         # meters (up-down span)

IMITATE_MIN_PX_STEP = 3       
IMITATE_MIN_M_STEP = 0.0025  


def _cumlen_prepare(points_xy_abs):
    cum = [0.0]
    total = 0.0
    for i in range(len(points_xy_abs) - 1):
        x0, y0 = points_xy_abs[i]
        x1, y1 = points_xy_abs[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        total += seg
        cum.append(total)
    return total, cum


def _point_at_distance(points_xy_abs, cumlen, total_len, s):
    if not points_xy_abs or not cumlen or total_len <= 1e-9:
        return None
    s = max(0.0, min(total_len, s))
    for i in range(len(cumlen) - 1):
        if cumlen[i] <= s <= cumlen[i + 1]:
            x0, y0 = points_xy_abs[i]
            x1, y1 = points_xy_abs[i + 1]
            seg_len = max(1e-9, (cumlen[i + 1] - cumlen[i]))
            u = (s - cumlen[i]) / seg_len
            return (x0 + u * (x1 - x0), y0 + u * (y1 - y0))
    return points_xy_abs[-1]


def stop_imitate():
    global imitate_running, imitate_points_xy, imitate_paper_z, imitate_total_len, imitate_cumlen
    imitate_running = False
    imitate_points_xy = None
    imitate_paper_z = None
    imitate_total_len = 0.0
    imitate_cumlen = None
    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass


def _simplify_points_m(points_xy_abs, min_step_m=IMITATE_MIN_M_STEP):
    if not points_xy_abs:
        return []
    out = [points_xy_abs[0]]
    lastx, lasty = out[0]
    for (x, y) in points_xy_abs[1:]:
        if math.hypot(x - lastx, y - lasty) >= min_step_m:
            out.append((x, y))
            lastx, lasty = x, y
    if len(out) == 1 and len(points_xy_abs) > 1:
        out.append(points_xy_abs[-1])
    return out


def canvas_pixels_to_robot_offsets(px, py, canvas_w, canvas_h):
    nx = (px / max(1, canvas_w)) - 0.5
    ny = (py / max(1, canvas_h)) - 0.5

    dx = nx * IMITATE_AREA_W
    dy = ny * IMITATE_AREA_H

    dy = -dy
    return dx, dy


def start_imitate_from_canvas_points(canvas_points_px, canvas_w, canvas_h):
    global imitate_running, imitate_start_time, imitate_points_xy, imitate_paper_z, imitate_total_len, imitate_cumlen

    if not IKPY_AVAILABLE or arm_chain is None:
        messagebox.showerror("Imitate", "IK is not available.")
        return

    stop_circle()
    stop_polyline()
    stop_imitate()

    if not canvas_points_px or len(canvas_points_px) < 2:
        messagebox.showwarning("Imitate", "Draw a stroke first.")
        return

    wx, wy, wz = get_paper_center_world()
    cx, cy, cz = world_to_robot_xyz(wx, wy, wz)

    pts_abs = []
    for (px, py) in canvas_points_px:
        dx, dy = canvas_pixels_to_robot_offsets(px, py, canvas_w, canvas_h)
        pts_abs.append((cx + dx, cy + dy))

    pts_abs = _simplify_points_m(pts_abs, min_step_m=IMITATE_MIN_M_STEP)
    if len(pts_abs) < 2:
        messagebox.showwarning("Imitate", "Stroke too short after simplification.")
        return

    imitate_total_len, imitate_cumlen = _cumlen_prepare(pts_abs)
    if imitate_total_len <= 1e-6:
        messagebox.showwarning("Imitate", "Stroke length is too small.")
        return

    imitate_points_xy = pts_abs
    imitate_paper_z = cz

    imitate_running = True
    imitate_start_time = supervisor.getTime()

    if pen is not None:
        try:
            pen.write(False)
        except Exception:
            pass
        
    hover_z = imitate_paper_z + circle_hover_z_offset
    x0, y0 = imitate_points_xy[0]
    _apply_ik_to_motors(x0, y0, hover_z)


def update_imitate_loop():
    global imitate_running
    if not imitate_running:
        return
    if not imitate_points_xy or imitate_paper_z is None or imitate_total_len <= 1e-9:
        stop_imitate()
        return

    t = supervisor.getTime() - imitate_start_time
    hover_z = imitate_paper_z + circle_hover_z_offset
    draw_z = imitate_paper_z + circle_draw_z_offset

    t1 = circle_approach_time
    t2 = t1 + circle_lower_time

    draw_time = imitate_total_len / max(1e-6, IMITATE_DRAW_SPEED)
    t3 = t2 + draw_time
    t4 = t3 + circle_lift_time

    pen_on = False

    if t < t1:
        x, y = imitate_points_xy[0]
        z = hover_z
    elif t < t2:
        u = (t - t1) / max(1e-6, (t2 - t1))
        x, y = imitate_points_xy[0]
        z = hover_z + u * (draw_z - hover_z)
    elif t < t3:
        s = (t - t2) * IMITATE_DRAW_SPEED
        p = _point_at_distance(imitate_points_xy, imitate_cumlen, imitate_total_len, s)
        if p is None:
            stop_imitate()
            return
        x, y = p
        z = draw_z
        pen_on = True
    elif t < t4:
        u = (t - t3) / max(1e-6, (t4 - t3))
        x, y = imitate_points_xy[-1]
        z = draw_z + u * (hover_z - draw_z)
    else:
        stop_imitate()
        return

    if pen is not None:
        try:
            pen.write(pen_on)
        except Exception:
            pass

    _apply_ik_to_motors(x, y, z)


# =============================================================================
# End-effector control: follow TARGET or hold a user-entered (x, y, z)
# =============================================================================

ee_control_mode = "follow" 
manual_xyz = [0.0, 0.0, 0.0]


def _apply_ik_to_motors(x, y, z):
    if not IKPY_AVAILABLE or arm_chain is None:
        return False

    initial_position = current_ik_initial_position()
    ikResults = arm_chain.inverse_kinematics(
        [x, y, z],
        max_iter=IKPY_MAX_ITERATIONS,
        initial_position=initial_position,
    )

    position = arm_chain.forward_kinematics(ikResults)
    squared_distance = (position[0, 3] - x) ** 2 + (position[1, 3] - y) ** 2 + (position[2, 3] - z) ** 2
    if math.sqrt(squared_distance) > 0.03:
        ikResults = arm_chain.inverse_kinematics([x, y, z])

    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])

    return True


def move_endeffector_to_xyz(x, y, z):
    global ee_control_mode, manual_xyz
    if not IKPY_AVAILABLE:
        messagebox.showerror("Move Endeffector", "IK is not available.")
        return

    manual_xyz = [x, y, z]
    ee_control_mode = "manual"
    _apply_ik_to_motors(x, y, z)


def update_manual_ee_hold_loop(active_screen, status_label=None):
    if active_screen != "target":
        return
    if ee_control_mode != "manual":
        return

    x, y, z = manual_xyz
    ok = _apply_ik_to_motors(x, y, z)

    if status_label is not None:
        if ok:
            status_label.config(text="Mode: MANUAL (holding x,y,z)", fg="#1B5E20")
        else:
            status_label.config(text="Mode: MANUAL (IK unavailable)", fg="#B00020")


# =============================================================================
# TARGET tracking: periodic IK updates driven by simulation ticks
# =============================================================================

FOLLOW_EVERY_N_STEPS = 2
_follow_counter = 0


def update_target_follow_loop(active_screen, status_label=None):
    global _follow_counter

    if active_screen != "target":
        return
    if ee_control_mode != "follow":
        return

    if target_node is None:
        if status_label is not None:
            status_label.config(text="Mode: FOLLOW (TARGET not found)", fg="#B00020")
        return

    if not IKPY_AVAILABLE:
        if status_label is not None:
            status_label.config(text="Mode: FOLLOW (IK unavailable)", fg="#B00020")
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
        initial_position=initial_position,
    )

    position = arm_chain.forward_kinematics(ikResults)
    squared_distance = (position[0, 3] - x) ** 2 + (position[1, 3] - y) ** 2 + (position[2, 3] - z) ** 2
    if math.sqrt(squared_distance) > 0.03:
        ikResults = arm_chain.inverse_kinematics([x, y, z])

    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])

    if status_label is not None:
        status_label.config(text="Mode: FOLLOW (using TARGET)", fg="#1B5E20")


# =============================================================================
# Tkinter UI
# =============================================================================

root = tk.Tk()
root.title("Robot Controller")
root.geometry("650x650")
root.configure(bg="#f0f0f0")

BTN_W = 18
BTN_H = 2

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


# =============================================================================
# Imitate window
# =============================================================================

def open_imitate_window():
    win = tk.Toplevel(root)
    win.title("Imitate - Draw a stroke")
    win.configure(bg="#f0f0f0")
    win.resizable(False, False)

    header = tk.Label(
        win,
        text="Draw with mouse (hold left button). Then click 'Send to Robot'.",
        bg="#f0f0f0",
        font=("Arial", 11),
    )
    header.pack(padx=10, pady=(10, 6))

    canvas = tk.Canvas(win, width=IMITATE_CANVAS_W, height=IMITATE_CANVAS_H, bg="white", highlightthickness=1)
    canvas.pack(padx=10, pady=8)

    btn_row = tk.Frame(win, bg="#f0f0f0")
    btn_row.pack(padx=10, pady=(0, 12), fill="x")

    status = tk.Label(win, text="Status: idle", bg="#f0f0f0", font=("Arial", 10))
    status.pack(padx=10, pady=(0, 10))

    stroke = {"pts": [], "last": None, "drawing": False}

    def _add_point(px, py):
        if stroke["last"] is None:
            stroke["pts"].append((px, py))
            stroke["last"] = (px, py)
            return
        lx, ly = stroke["last"]
        if math.hypot(px - lx, py - ly) >= IMITATE_MIN_PX_STEP:
            stroke["pts"].append((px, py))
            stroke["last"] = (px, py)

    def on_press(event):
        stroke["drawing"] = True
        stroke["last"] = None
        stroke["pts"] = []
        canvas.delete("all")
        _add_point(event.x, event.y)
        status.config(text="Status: drawing...")

    def on_drag(event):
        if not stroke["drawing"]:
            return
        px, py = event.x, event.y
        if stroke["last"] is not None:
            lx, ly = stroke["last"]
            if math.hypot(px - lx, py - ly) < IMITATE_MIN_PX_STEP:
                return
            canvas.create_line(lx, ly, px, py, width=2)
        _add_point(px, py)

    def on_release(event):
        stroke["drawing"] = False
        status.config(text=f"Status: stroke captured ({len(stroke['pts'])} pts)")

    canvas.bind("<ButtonPress-1>", on_press)
    canvas.bind("<B1-Motion>", on_drag)
    canvas.bind("<ButtonRelease-1>", on_release)

    def clear_canvas():
        stroke["pts"] = []
        stroke["last"] = None
        stroke["drawing"] = False
        canvas.delete("all")
        status.config(text="Status: cleared")

    def send_to_robot():
        if len(stroke["pts"]) < 2:
            messagebox.showwarning("Imitate", "Please draw a stroke first.")
            return
        start_imitate_from_canvas_points(stroke["pts"], IMITATE_CANVAS_W, IMITATE_CANVAS_H)
        status.config(text="Status: sent to robot (robot is drawing...)")

    tk.Button(
        btn_row, text="Clear", font=("Arial", 11), bg="#9E9E9E", fg="white",
        width=10, command=clear_canvas
    ).pack(side="left", padx=4)

    tk.Button(
        btn_row, text="Send to Robot", font=("Arial", 11), bg="#3F51B5", fg="white",
        width=14, command=send_to_robot
    ).pack(side="left", padx=4)

    tk.Button(
        btn_row, text="Close", font=("Arial", 11), bg="#E91E63", fg="white",
        width=10, command=win.destroy
    ).pack(side="right", padx=4)


# =============================================================================
# Screen: Main menu
# =============================================================================

main_frame = tk.Frame(container, bg="#f0f0f0")
main_frame.grid(row=0, column=0, sticky="nsew")
frames["main"] = main_frame

tk.Label(main_frame, text="Robot Controller", font=("Arial", 18), bg="#f0f0f0").pack(pady=20)

tk.Button(
    main_frame,
    text="Move Joints",
    font=("Arial", 14),
    bg="#4CAF50",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("joints"),
).pack(pady=10)

tk.Button(
    main_frame,
    text="Move Endeffector",
    font=("Arial", 14),
    bg="#607D8B",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("target"),
).pack(pady=10)

tk.Button(
    main_frame,
    text="Draw Circle",
    font=("Arial", 14),
    bg="#3F51B5",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("circle"),
).pack(pady=10)

tk.Button(
    main_frame,
    text="Draw Square",
    font=("Arial", 14),
    bg="#3F51B5",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("square"),
).pack(pady=10)

tk.Button(
    main_frame,
    text="Draw Rectangle",
    font=("Arial", 14),
    bg="#3F51B5",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("rectangle"),
).pack(pady=10)

tk.Button(
    main_frame,
    text="Imitate",
    font=("Arial", 14),
    bg="#FF9800",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=open_imitate_window,
).pack(pady=10)

tk.Button(
    main_frame,
    text="Go Home",
    font=("Arial", 14),
    bg="#9C27B0",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=go_home,
).pack(pady=10)

# =============================================================================
# Screen: Joint control
# =============================================================================

joints_frame = tk.Frame(container, bg="#f0f0f0")
joints_frame.grid(row=0, column=0, sticky="nsew")
frames["joints"] = joints_frame

tk.Label(joints_frame, text="Move Joints", font=("Arial", 16), bg="#f0f0f0").pack(pady=10)

rotation_speed = 0.02
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


def start_rotate(index, direction):
    rotate_dir[index] = direction
    rotate_joint_step(index)


def stop_rotate(index):
    rotate_dir[index] = 0


for i, name in enumerate(motor_names):
    row = tk.Frame(joints_frame, bg="#f0f0f0")
    row.pack(pady=5)

    tk.Label(row, text=name, font=("Arial", 12), width=12, bg="#f0f0f0").pack(side="left")

    btn_minus = tk.Button(row, text="-", width=4, font=("Arial", 12), bg="#FF7043", fg="white")
    btn_minus.pack(side="left", padx=5)
    btn_minus.bind("<ButtonPress-1>", lambda e, idx=i: start_rotate(idx, -1))
    btn_minus.bind("<ButtonRelease-1>", lambda e, idx=i: stop_rotate(idx))

    btn_plus = tk.Button(row, text="+", width=4, font=("Arial", 12), bg="#66BB6A", fg="white")
    btn_plus.pack(side="left", padx=5)
    btn_plus.bind("<ButtonPress-1>", lambda e, idx=i: start_rotate(idx, 1))
    btn_plus.bind("<ButtonRelease-1>", lambda e, idx=i: stop_rotate(idx))

tk.Button(
    joints_frame,
    text="Go Home",
    font=("Arial", 14),
    bg="#9C27B0",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=go_home,
).pack(pady=12)

tk.Button(
    joints_frame,
    text="Back to Menu",
    font=("Arial", 13),
    bg="#9E9E9E",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("main"),
).pack(pady=6)

# =============================================================================
# Screen: End-effector view
# =============================================================================

target_frame = tk.Frame(container, bg="#f0f0f0")
target_frame.grid(row=0, column=0, sticky="nsew")
frames["target"] = target_frame

tk.Label(target_frame, text="Move Endeffector", font=("Arial", 16), bg="#f0f0f0").pack(pady=10)

target_follow_label = tk.Label(target_frame, text="Mode: FOLLOW (using TARGET)", font=("Arial", 12), bg="#f0f0f0")
target_follow_label.pack(pady=8)

target_pos_label = tk.Label(target_frame, text="Target position: (not found)", font=("Arial", 12), bg="#f0f0f0")
target_pos_label.pack(pady=5)

mode_frame = tk.Frame(target_frame, bg="#f0f0f0")
mode_frame.pack(pady=8)

def set_follow_mode():
    global ee_control_mode
    ee_control_mode = "follow"

def set_manual_mode():
    global ee_control_mode
    ee_control_mode = "manual"

tk.Button(
    mode_frame,
    text="Follow TARGET",
    font=("Arial", 12),
    bg="#3F51B5",
    fg="white",
    width=14,
    command=set_follow_mode,
).pack(side="left", padx=6)

tk.Button(
    mode_frame,
    text="Manual (x,y,z)",
    font=("Arial", 12),
    bg="#607D8B",
    fg="white",
    width=14,
    command=set_manual_mode,
).pack(side="left", padx=6)

entry_frame = tk.Frame(target_frame, bg="#f0f0f0")
entry_frame.pack(pady=10)

tk.Label(entry_frame, text="x:", font=("Arial", 12), bg="#f0f0f0").grid(row=0, column=0, padx=3, pady=3, sticky="e")
tk.Label(entry_frame, text="y:", font=("Arial", 12), bg="#f0f0f0").grid(row=1, column=0, padx=3, pady=3, sticky="e")
tk.Label(entry_frame, text="z:", font=("Arial", 12), bg="#f0f0f0").grid(row=2, column=0, padx=3, pady=3, sticky="e")

x_entry = tk.Entry(entry_frame, width=12, font=("Arial", 12))
y_entry = tk.Entry(entry_frame, width=12, font=("Arial", 12))
z_entry = tk.Entry(entry_frame, width=12, font=("Arial", 12))

x_entry.grid(row=0, column=1, padx=3, pady=3)
y_entry.grid(row=1, column=1, padx=3, pady=3)
z_entry.grid(row=2, column=1, padx=3, pady=3)

def manual_move_ui():
    try:
        x = float(x_entry.get())
        y = float(y_entry.get())
        z = float(z_entry.get())
    except Exception:
        messagebox.showerror("Manual Move", "Invalid numbers.")
        return
    move_endeffector_to_xyz(x, y, z)

tk.Button(
    target_frame,
    text="Move to x,y,z",
    font=("Arial", 14),
    bg="#4CAF50",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=manual_move_ui,
).pack(pady=8)

tk.Button(
    target_frame,
    text="Back to Menu",
    font=("Arial", 13),
    bg="#9E9E9E",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("main"),
).pack(pady=10)

# =============================================================================
# Screen: Circle routine controls
# =============================================================================

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

tk.Button(
    circle_frame,
    text="Start Circle",
    font=("Arial", 14),
    bg="#3F51B5",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=circle_start_ui,
).pack(pady=10)

tk.Button(
    circle_frame,
    text="Stop Circle",
    font=("Arial", 14),
    bg="#E91E63",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=circle_stop_ui,
).pack(pady=10)

tk.Button(
    circle_frame,
    text="Go Home",
    font=("Arial", 14),
    bg="#9C27B0",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=go_home,
).pack(pady=10)

tk.Button(
    circle_frame,
    text="Back to Menu",
    font=("Arial", 13),
    bg="#9E9E9E",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("main"),
).pack(pady=10)


# =============================================================================
# Screen: Square routine controls
# =============================================================================

square_frame = tk.Frame(container, bg="#f0f0f0")
square_frame.grid(row=0, column=0, sticky="nsew")
frames["square"] = square_frame

tk.Label(square_frame, text="Draw Square", font=("Arial", 16), bg="#f0f0f0").pack(pady=15)

square_status = tk.Label(square_frame, text="Status: idle", font=("Arial", 12), bg="#f0f0f0")
square_status.pack(pady=8)

def square_start_ui():
    start_square_default()
    square_status.config(text="Status: running" if poly_running else "Status: idle")

def square_stop_ui():
    stop_polyline()
    square_status.config(text="Status: stopped")

tk.Button(
    square_frame,
    text="Start Square",
    font=("Arial", 14),
    bg="#3F51B5",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=square_start_ui,
).pack(pady=10)

tk.Button(
    square_frame,
    text="Stop",
    font=("Arial", 14),
    bg="#E91E63",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=square_stop_ui,
).pack(pady=10)

tk.Button(
    square_frame,
    text="Go Home",
    font=("Arial", 14),
    bg="#9C27B0",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=go_home,
).pack(pady=10)

tk.Button(
    square_frame,
    text="Back to Menu",
    font=("Arial", 13),
    bg="#9E9E9E",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("main"),
).pack(pady=10)


# =============================================================================
# Screen: Rectangle routine controls
# =============================================================================

rectangle_frame = tk.Frame(container, bg="#f0f0f0")
rectangle_frame.grid(row=0, column=0, sticky="nsew")
frames["rectangle"] = rectangle_frame

tk.Label(rectangle_frame, text="Draw Rectangle", font=("Arial", 16), bg="#f0f0f0").pack(pady=15)

rectangle_status = tk.Label(rectangle_frame, text="Status: idle", font=("Arial", 12), bg="#f0f0f0")
rectangle_status.pack(pady=8)

def rectangle_start_ui():
    start_rectangle_default()
    rectangle_status.config(text="Status: running" if poly_running else "Status: idle")

def rectangle_stop_ui():
    stop_polyline()
    rectangle_status.config(text="Status: stopped")

tk.Button(
    rectangle_frame,
    text="Start Rectangle",
    font=("Arial", 14),
    bg="#3F51B5",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=rectangle_start_ui,
).pack(pady=10)

tk.Button(
    rectangle_frame,
    text="Stop",
    font=("Arial", 14),
    bg="#E91E63",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=rectangle_stop_ui,
).pack(pady=10)

tk.Button(
    rectangle_frame,
    text="Go Home",
    font=("Arial", 14),
    bg="#9C27B0",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=go_home,
).pack(pady=10)

tk.Button(
    rectangle_frame,
    text="Back to Menu",
    font=("Arial", 13),
    bg="#9E9E9E",
    fg="white",
    height=BTN_H,
    width=BTN_W,
    command=lambda: show_frame("main"),
).pack(pady=10)

show_frame("main")

# =============================================================================
# Unified tick loop
# =============================================================================

while supervisor.step(timestep) != -1:
    update_circle_loop()
    update_polyline_loop()
    update_imitate_loop()
    
    if circle_running:
        if "running" not in circle_status.cget("text"):
            circle_status.config(text="Status: running")
    else:
        if "running" in circle_status.cget("text"):
            circle_status.config(text="Status: idle")

    pos = get_target_abs_position()
    if pos is None:
        target_pos_label.config(text="Target position: (not found)")
    else:
        target_pos_label.config(text=f"Target position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

    update_manual_ee_hold_loop(current_screen, status_label=target_follow_label)
    update_target_follow_loop(current_screen, status_label=target_follow_label)

    try:
        root.update_idletasks()
        root.update()
    except tk.TclError:
        break