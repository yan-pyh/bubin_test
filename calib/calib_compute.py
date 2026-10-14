#!/usr/bin/env python3
# 相机标定 - 计算脚本
# 用法: python3 calib_compute.py <方格边长mm> [图像目录]
# 例:   python3 calib_compute.py 25
import sys, os, glob
import numpy as np, cv2

SQUARE_MM = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
IMG_DIR = sys.argv[2] if len(sys.argv) > 2 else "/home/nvidia/vision_ws/calib/calib_imgs"
PATTERN = (8, 6)  # 内角点 (你的棋盘是 9x7 格子)

imgs = sorted(glob.glob(os.path.join(IMG_DIR, "*.png")))
print("找到 %d 张图像" % len(imgs))
if len(imgs) < 8:
    print("太少! 至少 8~10 张, 建议 20~30 张"); sys.exit(1)

# 棋盘3D坐标 (z=0 平面)
objp = np.zeros((PATTERN[0]*PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM

objpoints, imgpoints = [], []
crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
ok = fail = 0
for f in imgs:
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, PATTERN,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if found:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), crit)
        objpoints.append(objp); imgpoints.append(corners2)
        ok += 1
    else:
        fail += 1
        print("  检测失败: %s" % os.path.basename(f))
print("检测成功 %d 张, 失败 %d 张" % (ok, fail))
if ok < 8:
    print("成功数不足, 请补拍或调整棋盘亮度/角度"); sys.exit(1)

# 标定
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
    gray.shape[::-1], None, None)
print("\n===== 标定结果 =====")
print("重投影误差 rms = %.4f px (越小越好, <1 可接受)" % ret)
print("\n内参矩阵 K:")
print(mtx)
print("\n畸变系数 D (k1 k2 p1 p2 k3):")
print(dist.ravel())

# 生成 ROS camera_info.yaml
h, w = gray.shape
yaml_text = """image_width: %d
image_height: %d
camera_name: mv_cs050_10uc
camera_matrix:
  rows: 3
  cols: 3
  data: [%.6f, 0.0, %.6f, 0.0, %.6f, %.6f, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [%.8f, %.8f, %.8f, %.8f, %.8f]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [%.6f, 0.0, %.6f, 0.0, 0.0, %.6f, %.6f, 0.0, 0.0, 0.0, 1.0, 0.0]
""" % (w, h,
    mtx[0,0], mtx[0,2], mtx[1,1], mtx[1,2],
    dist[0,0], dist[0,1], dist[0,2], dist[0,3], dist[0,4],
    mtx[0,0], mtx[0,2], mtx[1,1], mtx[1,2])
out = os.path.join(IMG_DIR, "..", "camera_info_calib.yaml")
with open(out, "w") as f:
    f.write(yaml_text)
print("\n已生成: %s" % os.path.abspath(out))
