#!/usr/bin/env python3
"""生成可被 aruco_grab.launch.py(dictionary_id 6, markerLength 0.05)识别的 ArUco 标记 PNG,
覆盖 models/aruco_marker/materials/textures/aruco_marker.png。

注意: aruco_ros 的 dictionary_id 是 aruco(Rafael Muñoz)库的 DICT_TYPES 序号, 与 cv2.aruco 的
预定义字典不一定逐一对应。本脚本用 cv2.aruco 生成常见 6x6 字典的标记, 若现场识别不到,
按你们 aruco_node 实际字典改 --dict / --id 重新生成即可。

用法:
  python3 make_aruco_marker.py            # 默认 DICT_6X6_250, id 0, 600px, 含白边
  python3 make_aruco_marker.py --id 23 --dict DICT_6X6_250 --size 600
"""
import argparse, os
import numpy as np
import cv2

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.normpath(os.path.join(
        here, "..", "models", "aruco_marker", "materials", "textures", "aruco_marker.png"))
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--dict", default="DICT_6X6_250")
    ap.add_argument("--size", type=int, default=600, help="标记像素边长(不含白边)")
    ap.add_argument("--border", type=int, default=60, help="四周白色 quiet-zone 像素")
    ap.add_argument("--out", default=out)
    a = ap.parse_args()

    aruco = cv2.aruco
    d = aruco.getPredefinedDictionary(getattr(aruco, a.dict))
    gen = getattr(aruco, "generateImageMarker", None) or aruco.drawMarker
    marker = gen(d, a.id, a.size)
    canvas = np.full((a.size + 2*a.border, a.size + 2*a.border), 255, np.uint8)
    canvas[a.border:a.border+a.size, a.border:a.border+a.size] = marker
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    cv2.imwrite(a.out, canvas)
    print(f"wrote {a.out}  (dict={a.dict} id={a.id})")

if __name__ == "__main__":
    main()
