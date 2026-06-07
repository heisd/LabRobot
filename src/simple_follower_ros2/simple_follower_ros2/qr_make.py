#!/usr/bin/env python3
# coding=utf-8
"""二维码生成工具.

为巡线 + 二维码路径选择生成可打印的二维码图片.
生成的图片默认保存在 **本脚本(即 QR 节点)同级目录** 下的 ``qr_codes/`` 文件夹中,
也就是 ``simple_follower_ros2/simple_follower_ros2/qr_codes/``.

后端按以下顺序自动选择(哪个可用用哪个):
  1. ``qrcode`` (+Pillow)
  2. ``segno``               (纯 Python, 无需 Pillow)
  3. ``cv2.QRCodeEncoder``   (OpenCV contrib)

用法示例::

    # 生成一组默认的路径选择二维码(left/right/straight/stop)
    ros2 run simple_follower_ros2 qr_make --all
    python3 qr_make.py --all

    # 生成单个自定义二维码
    ros2 run simple_follower_ros2 qr_make --data "path:left" --name turn_left
    python3 qr_make.py --data "stop" --name stop_here --box-size 12
"""

import argparse
import os

# 默认的一组路径选择二维码: 文件名 -> 二维码内容
DEFAULT_CODES = {
    'turn_left': 'path:left',
    'turn_right': 'path:right',
    'go_straight': 'path:straight',
    'stop': 'path:stop',
}


def output_dir():
    """返回保存目录: 与本脚本(QR 节点)同级的 qr_codes/ 文件夹."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, 'qr_codes')


def _save_with_qrcode(data, path, box_size, border):
    import qrcode
    qr = qrcode.QRCode(
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=box_size,
        border=border,
    )
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color='black', back_color='white')
    img.save(path)


def _save_with_segno(data, path, box_size, border):
    import segno
    qr = segno.make(data, error='m')
    qr.save(path, scale=box_size, border=border)


def _save_with_cv2(data, path, box_size, border):
    import cv2
    import numpy as np
    encoder = cv2.QRCodeEncoder_create()
    qr = encoder.encode(data)  # 单通道 0/255 模块图
    # 放大每个模块并加白边
    qr = cv2.resize(qr, None, fx=box_size, fy=box_size, interpolation=cv2.INTER_NEAREST)
    pad = border * box_size
    qr = cv2.copyMakeBorder(qr, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(path, qr)


def save_qr(data, path, box_size=10, border=4):
    """尝试可用后端生成二维码, 全部失败则抛出最后一个异常."""
    backends = (_save_with_qrcode, _save_with_segno, _save_with_cv2)
    last_err = None
    for backend in backends:
        try:
            backend(data, path, box_size, border)
            return backend.__name__.replace('_save_with_', '')
        except Exception as err:  # noqa: BLE001 - 逐个后端兜底
            last_err = err
    raise RuntimeError(
        'No QR backend available. Install one of: '
        '`pip install qrcode[pil]` / `pip install segno` / opencv-contrib'
    ) from last_err


def main():
    parser = argparse.ArgumentParser(description='生成二维码图片(用于巡线路径选择)')
    parser.add_argument('--data', help='二维码内容(单个生成时必填)')
    parser.add_argument('--name', help='输出文件名(不含扩展名), 默认按内容生成')
    parser.add_argument('--all', action='store_true', help='生成一组默认的路径选择二维码')
    parser.add_argument('--out', default=None, help='输出目录, 默认 <节点同级>/qr_codes')
    parser.add_argument('--box-size', type=int, default=10, help='每个模块的像素大小')
    parser.add_argument('--border', type=int, default=4, help='白边模块数(标准为 4)')
    args = parser.parse_args()

    out = args.out or output_dir()
    os.makedirs(out, exist_ok=True)

    if args.all:
        items = DEFAULT_CODES.items()
    elif args.data:
        name = args.name or args.data.replace(':', '_').replace('/', '_').replace(' ', '_')
        items = [(name, args.data)]
    else:
        parser.error('请使用 --data 指定内容, 或使用 --all 生成默认集合')

    for name, data in items:
        path = os.path.join(out, f'{name}.png')
        backend = save_qr(data, path, args.box_size, args.border)
        print(f'[{backend}] "{data}" -> {path}')

    print(f'\n完成, 文件保存在: {out}')


if __name__ == '__main__':
    main()
