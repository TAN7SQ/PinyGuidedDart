import serial
import numpy as np
import time
from numpy.linalg import eig, inv

# ==========================
# 1️⃣ 串口参数设置
# ==========================
PORT = 'COM10'      # 改成你的串口
BAUD = 115200
DURATION = 40      # 采集时间（秒）

# ==========================
# 2️⃣ 开始采集
# ==========================
ser = serial.Serial(PORT, BAUD, timeout=1)
print("开始采集数据...")

data = []
start_time = time.time()

while time.time() - start_time < DURATION:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue

        ax, ay, az = map(float, line.split(','))
        data.append([ax, ay, az])

    except:
        continue

ser.close()
data = np.array(data)

print(f"采集完成，共 {len(data)} 个数据点")

# ==========================
# 3️⃣ 椭球拟合
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

center = -inv(A3) @ b

T = np.eye(4)
T[3,0:3] = center
R = T @ A @ T.T

M = R[0:3,0:3] / -R[3,3]

eigvals, eigvecs = eig(M)
scale = np.diag(np.sqrt(eigvals))
calibration_matrix = eigvecs @ scale @ eigvecs.T

# ==========================
# 4️⃣ 归一化到1g
# ==========================
mean_norm = np.mean([
    np.linalg.norm(calibration_matrix @ (x - center))
    for x in data
])

calibration_matrix /= mean_norm

# ==========================
# 5️⃣ 输出结果
# ==========================
print("\n==============================")
print("偏置 (bias):")
print(center)

print("\n校准矩阵:")
print(calibration_matrix)

print("\n验证：校准后模长统计")
calibrated = np.array([
    calibration_matrix @ (x - center)
    for x in data
])

norms = np.linalg.norm(calibrated, axis=1)
print(f"平均值: {np.mean(norms):.5f}")
print(f"标准差: {np.std(norms):.5f}")
print("==============================")