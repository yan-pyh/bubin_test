#!/usr/bin/env python3
# 相机标定 - 图像采集脚本 (MV-CS050-10UC)
# 用法: python3 calib_capture.py [宽 高]
#   默认 960x768
# 保存方式:
#   鼠标左键点击画面  保存当前帧 (最可靠, 推荐!)
#   空格 或 s         保存当前帧
#   +/-               增加/减少曝光
#   q                 退出
import sys, os, time
from ctypes import *
import numpy as np, cv2
sys.path.append("/opt/MVS/Samples/aarch64/Python/MvImport")
from MvCameraControl_class import *

# ---- 配置 ----
W = int(sys.argv[1]) if len(sys.argv) > 1 else 960
H = int(sys.argv[2]) if len(sys.argv) > 2 else 768
SAVE_DIR = "/home/nvidia/vision_ws/calib/calib_imgs"
EXPOSURE_US = 30000.0   # 初始曝光 (us), 用 +/- 调节
MIN_EXP, MAX_EXP = 1000.0, 100000.0

os.makedirs(SAVE_DIR, exist_ok=True)

MvCamera.MV_CC_Initialize()
deviceList = MV_CC_DEVICE_INFO_LIST()
ret = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, deviceList)
if ret != 0 or deviceList.nDeviceNum == 0:
    print("no device"); sys.exit(1)
stDev = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
cam = MvCamera()
cam.MV_CC_CreateHandle(stDev)
cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)

# 设置分辨率
cam.MV_CC_SetIntValueEx("Width", W)
cam.MV_CC_SetIntValueEx("Height", H)
# 固定曝光 (屏幕会闪, 自动曝光会来回跳动)
cam.MV_CC_SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF)
cam.MV_CC_SetEnumValue("GainAuto", MV_EXPOSURE_AUTO_MODE_OFF)
cam.MV_CC_SetFloatValue("Gain", 8.0)
cam.MV_CC_SetFloatValue("ExposureTime", EXPOSURE_US)

cam.MV_CC_StartGrabbing()
time.sleep(0.5)

def grab_frame():
    stFrame = MV_FRAME_OUT(); memset(byref(stFrame),0,sizeof(stFrame))
    ret = cam.MV_CC_GetImageBuffer(stFrame, 3000)
    if ret != 0:
        return None
    buf = np.ctypeslib.as_array(cast(stFrame.pBufAddr, POINTER(c_ubyte)), shape=(stFrame.stFrameInfo.nFrameLen,)).copy()
    cam.MV_CC_FreeImageBuffer(stFrame)
    bayer = buf.reshape(H, W)
    return cv2.cvtColor(bayer, cv2.COLOR_BayerRG2BGR)

count = 0

def on_mouse(event, x, y, flags, param):
    global count
    if event == cv2.EVENT_LBUTTONDOWN:
        fname = os.path.join(SAVE_DIR, "calib_%03d.png" % count)
        cv2.imwrite(fname, param["last_frame"])
        count += 1
        print("saved %s (click)" % fname)

# 窗口置顶, 便于获得焦点
cv2.namedWindow("calib", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("calib", cv2.WND_PROP_TOPMOST, 1)
state = {"last_frame": None}
cv2.setMouseCallback("calib", on_mouse, state)

print("相机 %dx%d, 初始曝光 %.1f us" % (W, H, EXPOSURE_US))
print("操作: [鼠标左键点击 或 空格/s]保存帧  [+/-]调曝光  [q]退出")
print("提示: 先点击画面窗口使其获得焦点! 移动棋盘覆盖9区域+各种倾斜, 拍20~30张")
while True:
    bgr = grab_frame()
    if bgr is None:
        continue
    state["last_frame"] = bgr
    # 显示已保存数量
    disp = bgr.copy()
    cv2.putText(disp, "saved: %d  exp: %.0fus  [click/space]=save [q]=quit" % (count, EXPOSURE_US),
                (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    cv2.imshow("calib", disp)
    key = cv2.waitKey(30) & 0xFF
    if key == ord(' ') or key == ord('s') or key == ord('S'):
        fname = os.path.join(SAVE_DIR, "calib_%03d.png" % count)
        cv2.imwrite(fname, bgr)
        count += 1
        print("saved %s" % fname)
    elif key == ord('+') or key == ord('='):
        EXPOSURE_US = min(EXPOSURE_US * 1.5, MAX_EXP)
        cam.MV_CC_SetFloatValue("ExposureTime", EXPOSURE_US)
        print("exposure -> %.1f us" % EXPOSURE_US)
    elif key == ord('-') or key == ord('_'):
        EXPOSURE_US = max(EXPOSURE_US / 1.5, MIN_EXP)
        cam.MV_CC_SetFloatValue("ExposureTime", EXPOSURE_US)
        print("exposure -> %.1f us" % EXPOSURE_US)
    elif key == ord('q'):
        break

cam.MV_CC_StopGrabbing(); cam.MV_CC_CloseDevice(); cam.MV_CC_DestroyHandle()
MvCamera.MV_CC_Finalize()
print("采集完成, 共保存 %d 张到 %s" % (count, SAVE_DIR))
