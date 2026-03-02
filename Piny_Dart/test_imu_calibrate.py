import numpy as np
from scipy.optimize import least_squares

# ---------------------- 1. 采集的6个姿态原始数据 ----------------------
# 格式：[[X+,Y+,Z+], [X-,Y-,Z-], [Y+,Y-,Z+], [Y-,Y+,Z+], [Z+,Y+,X+], [Z-,Y+,X+]]
# raw_data = np.array([
#     [1.02, 0.86, 9.90],  # 姿态1：Z+朝上
#     [0.10, -0.09, -9.75], # 姿态2：Z-朝下
#     [9.85, 0.05, 0.15],   # 姿态3：X+朝上
#     [-9.78, 0.06, 0.13],  # 姿态4：X-朝下
#     [0.07, 9.82, 0.11],   # 姿态5：Y+朝上
#     [0.08, -9.79, 0.10]   # 姿态6：Y-朝下
# ])
raw_data = np.array([
    [0.0105,0.0014,0.9994],  # 姿态1：Z+朝上
    [0.0105,0.0019,-1.0090],  # 姿态2：Z-朝下
    [1.0065,0.0291,0.0038], # 姿态3：X+朝上
    [-1.0015,0.0114,0.0103], # 姿态4：X-朝下
    [0.0141,1.0034,-0.0062],   # 姿态5：Y+朝上
    [0.0178,-1.0007,0.0296]  # 姿态6：Y-朝下
])
g = 9.80665  # 当地重力加速度（可根据纬度调整）

# ---------------------- 2. 定义误差函数 ----------------------
def calibration_error(params, data, g):
    # params = [off_x, off_y, off_z, T00, T01, T02, T10, T11, T12, T20, T21, T22]
    off = params[:3]          # 零偏
    T = params[3:].reshape(3,3)# 校准矩阵
    
    errors = []
    for point in data:
        # 校准计算：T × (原始值 - 零偏)
        cal_point = T @ (point - off)
        # 计算模长与g的误差
        errors.append(np.linalg.norm(cal_point) - g)
    return np.array(errors)

# ---------------------- 3. 初始化参数（单位矩阵+零偏0） ----------------------
initial_guess = np.zeros(12)
initial_guess[3:12] = np.eye(3).flatten()  # T初始为单位矩阵

# ---------------------- 4. 最小二乘求解 ----------------------
result = least_squares(calibration_error, initial_guess, args=(raw_data, g))

# ---------------------- 5. 提取校准参数 ----------------------
accelOffs = result.x[:3]  # 加速度计零偏
accelT = result.x[3:].reshape(3,3)  # 3x3校准矩阵

# ---------------------- 6. 输出C++格式的结果（核心修改） ----------------------
print("=== C++ 校准参数（可直接复制）===")
print("static constexpr AccCaliParams_s ACC_CALI = {")
# 输出校准矩阵 accelT
print("    .accelT = {", end="")
for i in range(3):
    row = accelT[i]
    # 每行保留9位小数，末尾加f表示float类型
    row_str = [f"{val:.9f}f" for val in row]
    if i < 2:
        print(f"{{{', '.join(row_str)}}},")
        print("               ", end="")
    else:
        print(f"{{{', '.join(row_str)}}},")
# 输出零偏 accelOffs
offs_str = [f"{val:.9f}f" for val in accelOffs]
print(f"    .accelOffs = {{{', '.join(offs_str)}}};")

# ---------------------- 7. 原有验证校准效果（保留） ----------------------
print("\n=== 校准效果验证 ===")
print("加速度计零偏 (off_x, off_y, off_z):")
print(accelOffs)
print("\n3x3校准矩阵 T:")
print(accelT)
print("\n=== 各姿态校准后模长验证 ===")
for i, point in enumerate(raw_data):
    cal_point = accelT @ (point - accelOffs)
    norm = np.linalg.norm(cal_point)
    print(f"姿态{i+1} - 校准后模长: {norm:.4f} m/s² (理论值: {g:.4f})")