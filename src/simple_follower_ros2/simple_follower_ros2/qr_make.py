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

    # 生成任意/随机角度的固定转角二维码
    ros2 run simple_follower_ros2 qr_make --turn left --angle 45
    ros2 run simple_follower_ros2 qr_make --turn right --random --min-angle 20 --max-angle 90

    # 打开可视化生成工具(滑条调方向/角度, 实时预览, 按 s 保存)
    ros2 run simple_follower_ros2 qr_make_gui
    ros2 run simple_follower_ros2 qr_make --gui
"""

import argparse
import os
import random

# 默认的一组路径选择二维码: 文件名 -> 二维码内容
#   path:left / path:right        -> 原地转, 转到重新发现线
#   path:left30 / path:right30    -> 原地固定转 30 度
#   path:straight                 -> 直行
#   path:stop                     -> 停车
DEFAULT_CODES = {
    'turn_left': 'path:left',
    'turn_right': 'path:right',
    'turn_left_30': 'path:left30',
    'turn_right_30': 'path:right30',
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
    # 必须用 make_qr 强制标准二维码: segno.make 对短内容会生成 Micro QR(M1-M4),
    # 而微信 / 手机相机 / ZBar / OpenCV 都不支持 Micro QR, 会"扫不出来".
    qr = segno.make_qr(data, error='m')
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


def turn_content(direction, angle):
    """方向 + 角度 -> (文件名, 二维码内容).

    例如 turn_content('left', 30) -> ('turn_left_30', 'path:left30').
    """
    a = f'{angle:g}'
    content = f'path:{direction}{a}'
    name = f'turn_{direction}_{a.replace(".", "_")}'
    return name, content


def render_qr(data, box_size=10, border=4):
    """生成二维码并返回 OpenCV BGR 图(用于 GUI 预览). 全部后端失败则抛异常."""
    import numpy as np
    import cv2

    # qrcode (+Pillow)
    try:
        import qrcode
        qr = qrcode.QRCode(
            error_correction=qrcode.constants.ERROR_CORRECT_M,
            box_size=box_size, border=border)
        qr.add_data(data)
        qr.make(fit=True)
        img = qr.make_image(fill_color='black', back_color='white').convert('RGB')
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    except Exception:  # noqa: BLE001 - 逐后端兜底
        pass

    # segno (纯 Python) -> PNG 字节 -> 解码
    try:
        import io
        import segno
        buff = io.BytesIO()
        segno.make_qr(data, error='m').save(buff, kind='png', scale=box_size, border=border)
        arr = np.frombuffer(buff.getvalue(), dtype=np.uint8)
        gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    except Exception:  # noqa: BLE001
        pass

    # OpenCV 自带编码器
    enc = cv2.QRCodeEncoder_create()
    qr = enc.encode(data)
    qr = cv2.resize(qr, None, fx=box_size, fy=box_size, interpolation=cv2.INTER_NEAREST)
    pad = border * box_size
    qr = cv2.copyMakeBorder(qr, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)


def main():
    parser = argparse.ArgumentParser(description='生成二维码图片(用于巡线路径选择)')
    parser.add_argument('--data', help='二维码内容(单个生成时必填)')
    parser.add_argument('--name', help='输出文件名(不含扩展名), 默认按内容生成')
    parser.add_argument('--all', action='store_true', help='生成一组默认的路径选择二维码')
    parser.add_argument('--turn', choices=['left', 'right'], help='生成固定转角二维码的方向')
    parser.add_argument('--angle', type=float, help='固定转角角度(度), 配合 --turn')
    parser.add_argument('--random', action='store_true', help='随机角度(配 --min-angle/--max-angle)')
    parser.add_argument('--min-angle', type=float, default=15.0, help='随机角度下限(度)')
    parser.add_argument('--max-angle', type=float, default=90.0, help='随机角度上限(度)')
    parser.add_argument('--gui', action='store_true', help='打开可视化生成工具(等价于 qr_make_gui)')
    parser.add_argument('--out', default=None, help='输出目录, 默认 <节点同级>/qr_codes')
    parser.add_argument('--box-size', type=int, default=10, help='每个模块的像素大小')
    parser.add_argument('--border', type=int, default=4, help='白边模块数(标准为 4)')
    args = parser.parse_args()

    if args.gui:
        try:
            from simple_follower_ros2.qr_make_gui import main as gui_main
        except ImportError:
            from qr_make_gui import main as gui_main
        gui_main()
        return

    out = args.out or output_dir()
    os.makedirs(out, exist_ok=True)

    if args.all:
        items = DEFAULT_CODES.items()
    elif args.turn or args.random:
        direction = args.turn or random.choice(['left', 'right'])
        if args.angle is not None:
            angle = args.angle
        elif args.random:
            angle = float(random.randint(int(args.min_angle), int(args.max_angle)))
        else:
            angle = 30.0
        name, data = turn_content(direction, angle)
        items = [(args.name or name, data)]
    elif args.data:
        name = args.name or args.data.replace(':', '_').replace('/', '_').replace(' ', '_')
        items = [(name, args.data)]
    else:
        parser.error('请使用 --data / --all / --turn / --random / --gui 之一')

    for name, data in items:
        path = os.path.join(out, f'{name}.png')
        backend = save_qr(data, path, args.box_size, args.border)
        print(f'[{backend}] "{data}" -> {path}')

    print(f'\n完成, 文件保存在: {out}')


if __name__ == '__main__':
    main()
