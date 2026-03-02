import serial
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

PORT = 'COM10'
BAUD = 115200

WINDOW = 120
STD_THRESHOLD = 0.006
TARGET_POINTS = 70

ser = serial.Serial(PORT, BAUD, timeout=1)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

buffer = []
static_points = []
octant_count = np.zeros(8)

print("开始工业级椭球标定（带覆盖检测）...")

def get_octant(v):
    idx = 0
    if v[0] < 0: idx |= 1
    if v[1] < 0: idx |= 2
    if v[2] < 0: idx |= 4
    return idx

while len(static_points) < TARGET_POINTS:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue

        vec = np.array(list(map(float, line.split(','))))
        buffer.append(vec)

        if len(buffer) > WINDOW:
            buffer.pop(0)

        if len(buffer) == WINDOW:
            arr = np.array(buffer)
            if np.linalg.norm(np.std(arr, axis=0)) < STD_THRESHOLD:
                mean_vec = np.mean(arr, axis=0)
                static_points.append(mean_vec)

                idx = get_octant(mean_vec)
                octant_count[idx] += 1

                print(f"✔ 静止点 {len(static_points)}")

                for i in range(8):
                    if octant_count[i] < 5:
                        print(f"⚠ 八分体 {i} 数据不足")

                buffer.clear()

                ax.cla()
                sp = np.array(static_points)
                ax.scatter(sp[:,0], sp[:,1], sp[:,2], s=40)
                ax.set_xlim([-1.2,1.2])
                ax.set_ylim([-1.2,1.2])
                ax.set_zlim([-1.2,1.2])
                plt.pause(0.01)

    except:
        continue

ser.close()
plt.close()

data = np.array(static_points)

# ===== 真正椭球拟合 =====

D = np.column_stack([
    data[:,0]**2,
    data[:,1]**2,
    data[:,2]**2,
    2*data[:,1]*data[:,2],
    2*data[:,0]*data[:,2],
    2*data[:,0]*data[:,1],
    2*data[:,0],
    2*data[:,1],
    2*data[:,2],
    np.ones(data.shape[0])
])

U,S,V = np.linalg.svd(D)
p = V[-1]

A = np.array([
    [p[0], p[5], p[4], p[6]],
    [p[5], p[1], p[3], p[7]],
    [p[4], p[3], p[2], p[8]],
    [p[6], p[7], p[8], p[9]]
])

A3 = A[:3,:3]
b = p[6:9]

center = -np.linalg.solve(A3, b)

T = np.eye(4)
T[3,:3] = center
R = T @ A @ T.T

M = R[:3,:3] / -R[3,3]

eigvals, eigvecs = np.linalg.eigh(M)
eigvals = np.maximum(eigvals, 1e-8)

scale = np.diag(np.sqrt(eigvals))
calibration_matrix = eigvecs @ scale @ eigvecs.T

# 归一化
calibrated = np.array([calibration_matrix @ (x - center) for x in data])
mean_norm = np.mean(np.linalg.norm(calibrated, axis=1))
calibration_matrix /= mean_norm

norms = np.linalg.norm(
    np.array([calibration_matrix @ (x - center) for x in data]),
    axis=1
)

print("\n==============================")
print("平均值:", np.mean(norms))
print("标准差:", np.std(norms))
print("==============================")

print("\nC++ 常量:\n")
print("static constexpr AccCaliParams_s ACC_CALI = {")
print("    .accelT = {")
for i in range(3):
    print("        {" + ", ".join(f"{v:.9f}f" for v in calibration_matrix[i]) + "},")
print("    },")
print("    .accelOffs = {" + ", ".join(f"{v:.9f}f" for v in center) + "}")
print("};")