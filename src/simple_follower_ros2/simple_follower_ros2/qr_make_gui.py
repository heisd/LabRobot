#!/usr/bin/env python3
# coding=utf-8
"""二维码可视化生成工具(GUI).

用 OpenCV 滑条实时调方向 / 角度 / 类型, 窗口里实时预览二维码,
按键保存到 QR 节点同级的 ``qr_codes/`` 目录. 与 ``adjust_hsv`` 同样基于 cv2,
不需要额外安装 GUI 库.

运行::

    ros2 run simple_follower_ros2 qr_make_gui
    python3 qr_make_gui.py

窗口 'QR Maker' 滑条:
    angle        转角(度), 0~180
    dir 0L/1R    0=左转, 1=右转
    mode         0=固定转角  1=转到发现线  2=直行  3=停止
    box          模块像素大小(影响图片清晰度)

按键:
    s  保存当前二维码到 qr_codes/
    r  随机一个角度(并随机方向)
    q / ESC  退出
"""

import os
import random

import cv2
import numpy as np

try:
    from simple_follower_ros2.qr_make import output_dir, render_qr, save_qr, turn_content
except ImportError:  # 直接 python3 运行时
    from qr_make import output_dir, render_qr, save_qr, turn_content

WIN = 'QR Maker'
MODE_NAMES = {0: 'fixed-angle', 1: 'seek-line', 2: 'straight', 3: 'stop'}


def _nothing(_):
    pass


def build_content(mode, direction, angle):
    """根据滑条状态生成 (文件名, 二维码内容)."""
    if mode == 0:                       # 固定转角
        return turn_content(direction, angle)
    if mode == 1:                       # 转到发现线
        return f'turn_{direction}', f'path:{direction}'
    if mode == 2:                       # 直行
        return 'go_straight', 'path:straight'
    return 'stop', 'path:stop'          # 停止


def compose(qr_bgr, content, last_saved):
    """把二维码和文字信息拼成一张展示画布."""
    qr = cv2.resize(qr_bgr, (320, 320), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((460, 360, 3), 255, np.uint8)
    canvas[10:330, 20:340] = qr
    cv2.putText(canvas, content, (20, 360),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    cv2.putText(canvas, 's=save  r=random  q=quit', (20, 392),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 120, 120), 1)
    if last_saved:
        cv2.putText(canvas, f'saved: {last_saved}', (20, 422),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 130, 0), 1)
    return canvas


def main():
    out = output_dir()
    os.makedirs(out, exist_ok=True)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar('angle', WIN, 30, 180, _nothing)
    cv2.createTrackbar('dir 0L/1R', WIN, 0, 1, _nothing)
    cv2.createTrackbar('mode', WIN, 0, 3, _nothing)
    cv2.createTrackbar('box', WIN, 10, 20, _nothing)

    last_content = None
    last_saved = ''
    qr_bgr = None

    while True:
        angle = cv2.getTrackbarPos('angle', WIN)
        direction = 'left' if cv2.getTrackbarPos('dir 0L/1R', WIN) == 0 else 'right'
        mode = cv2.getTrackbarPos('mode', WIN)
        box = max(2, cv2.getTrackbarPos('box', WIN))

        name, content = build_content(mode, direction, float(angle))

        # 内容变化时才重新生成, 省 CPU
        if content != last_content:
            try:
                qr_bgr = render_qr(content, box_size=box, border=4)
            except Exception as err:  # noqa: BLE001
                print('render failed:', err)
                qr_bgr = np.full((100, 100, 3), 200, np.uint8)
            last_content = content

        label = f'[{MODE_NAMES.get(mode, "?")}] {content}'
        cv2.imshow(WIN, compose(qr_bgr, label, last_saved))

        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):       # q / ESC
            break
        if key == ord('r'):             # 随机角度 + 随机方向
            cv2.setTrackbarPos('angle', WIN, random.randint(15, 90))
            cv2.setTrackbarPos('dir 0L/1R', WIN, random.randint(0, 1))
        if key == ord('s'):             # 保存
            path = os.path.join(out, f'{name}.png')
            try:
                backend = save_qr(content, path, box, 4)
                last_saved = os.path.basename(path)
                print(f'[{backend}] "{content}" -> {path}')
            except Exception as err:  # noqa: BLE001
                print('save failed:', err)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
