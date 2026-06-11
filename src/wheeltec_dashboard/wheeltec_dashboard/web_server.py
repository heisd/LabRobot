"""Static file HTTP server that hosts the dashboard front-end.

It is a thin wrapper around ``http.server`` exposed as a ROS 2 node so that
it can be launched alongside ``rosbridge_websocket`` from a single launch
file. Parameters:

* ``port`` (int, default 8080) — TCP port to listen on.
* ``address`` (string, default "0.0.0.0") — interface to bind to.
* ``web_root`` (string, default "") — override directory containing
  ``index.html``. When empty, the installed share/wheeltec_dashboard/web
  directory is used.
"""

from __future__ import annotations

import os
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class _QuietHandler(SimpleHTTPRequestHandler):
    """Suppress the default per-request stderr logging.

    额外提供 ``/pkg/<package>/<相对路径>`` 路由: 把 ament share 目录里的文件
    (URDF 引用的 STL/DAE 网格等)开放给浏览器 —— 前端 3D 视图把
    ``package://rm_description/meshes/x.STL`` 映射成
    ``/pkg/rm_description/meshes/x.STL`` 即可加载机器人模型。
    """

    def log_message(self, format, *args):  # noqa: A002 - signature dictated by base
        return

    def translate_path(self, path):
        clean = unquote(urlsplit(path).path)
        if clean.startswith('/pkg/'):
            parts = clean[len('/pkg/'):].split('/', 1)
            if len(parts) == 2 and parts[0] and parts[1]:
                try:
                    share = os.path.abspath(get_package_share_directory(parts[0]))
                except Exception:  # noqa: BLE001  未知包名 -> 404
                    return os.path.join(self.directory, '__pkg_not_found__')
                target = os.path.normpath(os.path.join(share, parts[1]))
                # 防目录穿越: 解析后必须仍在该包的 share 目录内
                if target == share or target.startswith(share + os.sep):
                    return target
            return os.path.join(self.directory, '__forbidden__')
        return super().translate_path(path)


class DashboardWebServer(Node):
    def __init__(self) -> None:
        super().__init__('wheeltec_dashboard_web_server')

        self.declare_parameter('port', 8080)
        self.declare_parameter('address', '0.0.0.0')
        self.declare_parameter('web_root', '')

        port = int(self.get_parameter('port').value)
        address = str(self.get_parameter('address').value)
        web_root = str(self.get_parameter('web_root').value)

        if not web_root:
            web_root = os.path.join(
                get_package_share_directory('wheeltec_dashboard'), 'web'
            )

        if not os.path.isdir(web_root):
            raise RuntimeError(f'web_root directory does not exist: {web_root}')

        handler = partial(_QuietHandler, directory=web_root)
        self._httpd = ThreadingHTTPServer((address, port), handler)
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name='dashboard-http', daemon=True
        )
        self._thread.start()

        self.get_logger().info(
            f'Wheeltec dashboard available at http://{address}:{port}/ '
            f'(serving {web_root})'
        )

    def destroy_node(self) -> bool:
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:  # pragma: no cover - best-effort shutdown
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardWebServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
