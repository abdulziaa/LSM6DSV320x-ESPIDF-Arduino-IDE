import serial
import re
from vpython import box, vector, scene

# --- Serial Port Setup ---
ser = serial.Serial('COM3', 115200)  # Change COM3 to your port

# --- 3D Scene Setup ---
scene.title = "Quaternion Cube Orientation"
scene.width = 800
scene.height = 600
scene.center = vector(0, 0, 0)
scene.forward = vector(-1, -1, -1)

cube = box(length=1, height=1, width=1, color=vector(0, 0.6, 1))

# --- Helper: Apply Quaternion Rotation ---
def quaternion_to_matrix(qx, qy, qz, qw):
    # Rotation matrix from quaternion
    return [
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),       1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),       2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
    ]

def apply_rotation(cube, qx, qy, qz, qw):
    R = quaternion_to_matrix(qx, qy, qz, qw)
    # Update cube orientation using axis vectors
    cube.axis = vector(R[0][0], R[1][0], R[2][0])  # X-axis
    cube.up   = vector(R[0][1], R[1][1], R[2][1])  # Y-axis

# --- Main Loop ---
pattern = re.compile(r"X=(-?\d+\.\d+)\s+Y=(-?\d+\.\d+)\s+Z=(-?\d+\.\d+)\s+W=(-?\d+\.\d+)")

while True:
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    match = pattern.search(line)
    if match:
        qx, qy, qz, qw = map(float, match.groups())
        apply_rotation(cube, qx, qy, qz, qw)