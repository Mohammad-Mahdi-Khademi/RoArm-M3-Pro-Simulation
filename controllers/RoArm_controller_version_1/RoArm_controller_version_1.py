import math
from controller import Robot, Keyboard

TIME_STEP = 32

MOTOR_NAMES = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_gripper_link",
]

SENSOR_SUFFIX = "_sensor"

def is_finite(x):
    return isinstance(x, (int, float)) and math.isfinite(x)

def safe_value(x, fallback=0.0):
    return x if is_finite(x) else fallback

def clamp(x, lo, hi):
    if lo is None or hi is None:
        return x
    return max(lo, min(hi, x))

robot = Robot()
kb = Keyboard()
kb.enable(TIME_STEP)

motors = []
sensors = []
limits = []   
targets = []

for name in MOTOR_NAMES:
    m = robot.getDevice(name)
    motors.append(m)

    s = None
    try:
        s = robot.getDevice(name + SENSOR_SUFFIX)
        s.enable(TIME_STEP)
    except BaseException:
        s = None
    sensors.append(s)

    min_pos = m.getMinPosition()
    max_pos = m.getMaxPosition()

    if not is_finite(min_pos) or not is_finite(max_pos) or max_pos < min_pos:
        limits.append((None, None))
    else:
        limits.append((min_pos, max_pos))

robot.step(TIME_STEP)

for i, m in enumerate(motors):
    s = sensors[i]
    q0 = 0.0
    if s is not None:
        q0 = safe_value(s.getValue(), 0.0)

    targets.append(q0)

    m.setVelocity(min(2.0, m.getMaxVelocity()))
    m.setPosition(q0)

selected = 0
step_rad = 0.05

print("Keyboard control:")
print("  1..5 : select joint")
print("  ↑ / ↓ : increase/decrease selected joint target")
print("  ← / → : smaller/bigger step")
print("  SPACE : reset all targets to 0")
print("  P     : print sensor angles")

SPACE_KEY = 32 

while robot.step(TIME_STEP) != -1:
    key = kb.getKey()
    if key == -1:
        continue

    if ord('1') <= key <= ord('5'):
        idx = key - ord('1')
        if idx < len(motors):
            selected = idx
            print(f"Selected joint {selected + 1}: {MOTOR_NAMES[selected]}")
        continue

    if key == Keyboard.LEFT:
        step_rad = max(0.005, step_rad * 0.5)
        print(f"Step = {step_rad:.4f} rad")
        continue
    if key == Keyboard.RIGHT:
        step_rad = min(0.5, step_rad * 2.0)
        print(f"Step = {step_rad:.4f} rad")
        continue

    if key in (ord('p'), ord('P')):
        vals = []
        for s in sensors:
            vals.append(safe_value(s.getValue(), 0.0) if s else 0.0)
        print(" | ".join([f"q{i+1}={vals[i]:+.3f}" for i in range(len(vals))]))
        continue

    if key == SPACE_KEY:
        targets = [0.0] * len(targets)
        print("Targets reset to 0")

    elif key == Keyboard.UP:
        targets[selected] += step_rad
    elif key == Keyboard.DOWN:
        targets[selected] -= step_rad
    else:
        continue

    for i, m in enumerate(motors):
        mn, mx = limits[i]
        targets[i] = clamp(safe_value(targets[i], 0.0), mn, mx)
        m.setPosition(targets[i])
