import serial
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

PORT = 'COM10'
BAUD = 115200
DURATION = 40

ser = serial.Serial(PORT, BAUD, timeout=1)

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

data = []
start_time = time.time()

print("开始实时采集...")

while time.time() - start_time < DURATION:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue

        ax_val, ay_val, az_val = map(float, line.split(','))
        data.append([ax_val, ay_val, az_val])

        if len(data) % 20 == 0:
            ax.cla()

            arr = np.array(data)
            norms = np.linalg.norm(arr, axis=1)

            sc = ax.scatter(
                arr[:,0],
                arr[:,1],
                arr[:,2],
                c=norms,
                s=5
            )

            ax.set_xlim([-1.5,1.5])
            ax.set_ylim([-1.5,1.5])
            ax.set_zlim([-1.5,1.5])

            ax.set_title("Real-Time 3D Accelerometer Data")
            plt.pause(0.001)

    except:
        continue

ser.close()
plt.ioff()
plt.show()

data = np.array(data)
print(f"采集完成，共 {len(data)} 个点")

# ==========================
# 3️⃣ 自动异常值剔除
# ==========================
norms = np.linalg.norm(data, axis=1)
mean_norm = np.mean(norms)

# 保留模长在 0.5g ~ 1.5g 范围的数据
mask = (norms > 0.5) & (norms < 1.5)
data = data[mask]

print(f"异常值剔除后剩余 {len(data)} 个点")

# ==========================
# 4️⃣ 实时3D散点图
# ==========================
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(data[:,0], data[:,1], data[:,2], s=2)
# ax.set_title("Raw Accelerometer Data")
# plt.show()

# ==========================
# 5️⃣ 椭球拟合
# ==========================
print("开始椭球拟合...")

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

_, _, V = np.linalg.svd(D)
p = V[-1]

A = np.array([
    [p[0], p[5], p[4], p[6]],
    [p[5], p[1], p[3], p[7]],
    [p[4], p[3], p[2], p[8]],
    [p[6], p[7], p[8], p[9]]
])

A3 = A[0:3,0:3]
b = p[6:9]

center = -np.linalg.solve(A3, b)

# 平移
T = np.eye(4)
T[3,0:3] = center
R = T @ A @ T.T

M = R[0:3,0:3] / -R[3,3]

# ==========================
# 6️⃣ 强制正定修复
# ==========================
eigvals, eigvecs = eig(M)

# 修复负特征值
eigvals = np.abs(eigvals)

scale = np.diag(np.sqrt(eigvals))
calibration_matrix = eigvecs @ scale @ eigvecs.T

# ==========================
# 7️⃣ 归一化到1g
# ==========================
calibrated = np.array([
    calibration_matrix @ (x - center)
    for x in data
])

norms = np.linalg.norm(calibrated, axis=1)
calibration_matrix /= np.mean(norms)

# ==========================
# 8️⃣ 再次验证
# ==========================
calibrated = np.array([
    calibration_matrix @ (x - center)
    for x in data
])

norms = np.linalg.norm(calibrated, axis=1)

print("\n==============================")
print("平均值(≈1.0):", np.mean(norms))
print("标准差(<0.02):", np.std(norms))
print("==============================")

# ==========================
# 9️⃣ 生成C++常量
# ==========================
print("\n🔥 可直接复制的C++格式:\n")

print("static constexpr AccCaliParams_s ACC_CALI = {")
print("    .accelT = {")

for i in range(3):
    row = calibration_matrix[i]
    row_str = ", ".join([f"{val:.9f}f" for val in row])
    print(f"        {{{row_str}}},")

offs_str = ", ".join([f"{val:.9f}f" for val in center])
print(f"    .accelOffs = {{{offs_str}}}")
print("};")