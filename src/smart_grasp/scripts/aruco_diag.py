#!/usr/bin/env python3
import cv2
import numpy as np

img = cv2.imread("/tmp/frame_color.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
h, w = gray.shape
print(f"图像尺寸: {w}x{h}")
print(f"亮度: mean={gray.mean():.0f} min={gray.min()} max={gray.max()}")
print(f"清晰度(Laplacian方差): {cv2.Laplacian(gray, cv2.CV_64F).var():.0f} (<100偏模糊)")
print()

dicts = [d for d in dir(cv2.aruco) if d.startswith("DICT_")]
found_any = False
for dn in dicts:
    ad = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dn))
    det = cv2.aruco.ArucoDetector(ad, cv2.aruco.DetectorParameters())
    corners, ids, rejected = det.detectMarkers(gray)
    if ids is not None and len(ids) > 0:
        found_any = True
        for c, i in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2)
            side = np.linalg.norm(pts[0] - pts[1])
            print(f"[命中] 字典={dn} id={i} 中心={pts.mean(axis=0).astype(int)} 边长={side:.0f}px")

if not found_any:
    print("所有字典均未检测到 ArUco 码")
    ad = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    det = cv2.aruco.ArucoDetector(ad, cv2.aruco.DetectorParameters())
    _, _, rejected = det.detectMarkers(gray)
    print(f"疑似候选框(rejected)数量: {len(rejected)}")
    # 保存增强对比度图辅助判断
    clahe = cv2.createCLAHE(3.0, (8, 8)).apply(gray)
    _, _, rej2 = det.detectMarkers(clahe)
    corners2, ids2, _ = det.detectMarkers(clahe)
    print(f"CLAHE增强后: ids={None if ids2 is None else ids2.flatten()} rejected={len(rej2)}")
