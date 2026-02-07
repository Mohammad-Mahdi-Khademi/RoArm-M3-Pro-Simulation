import sys
import tempfile
import ikpy
from ikpy.chain import Chain
import math
from controller import Supervisor

IKPY_MAX_ITERATIONS = 4

supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))
armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True, False])

motors = []
for link in armChain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)

target = supervisor.getFromDef('TARGET')
arm = supervisor.getSelf()

print('Draw a circle on the paper sheet...')
while supervisor.step(timeStep) != -1:
    t = supervisor.getTime()

    x = 0.25 * math.cos(t) + 1.1
    y = 0.25 * math.sin(t) - 0.95
    z = 0.05

    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
    ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

    for i in range(3):
        motors[i].setPosition(ikResults[i + 1])
    motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
    motors[5].setPosition(ikResults[1])

    if supervisor.getTime() > 2 * math.pi + 1.5:
        break
    elif supervisor.getTime() > 1.5:
        supervisor.getDevice('pen').write(True)

print('Move the yellow and black sphere to move the arm...')
while supervisor.step(timeStep) != -1:
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()

    x = -(targetPosition[1] - armPosition[1])
    y = targetPosition[0] - armPosition[0]
    z = targetPosition[2] - armPosition[2]

    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
    ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

    position = armChain.forward_kinematics(ikResults)
    squared_distance = (position[0, 3] - x)**2 + (position[1, 3] - y)**2 + (position[2, 3] - z)**2
    if math.sqrt(squared_distance) > 0.03:
        ikResults = armChain.inverse_kinematics([x, y, z])

    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])
