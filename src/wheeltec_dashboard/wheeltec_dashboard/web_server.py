"""Static file HTTP server that hosts the dashboard front-end.

It is a thin wrapper around ``http.server`` exposed as a ROS 2 node so that
it can be launched alongside ``rosbridge_websocket`` from a single launch
file. Parameters:

* ``port`` (int, default 8000) — TCP port to listen on.
* ``address`` (string, default "0.0.0.0") — interface to bind to.
* ``web_root`` (string, default "") — override directory containing
  ``index.html``. When empty, the installed share/wheeltec_dashboard/web
  directory is used.
"""

from __future__ import annotations

import os
import threading
import urllib.request
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlencode, urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

# 取本地 web_video_server 时不走系统代理(环境里常设了 http_proxy, 否则会绕去代理失败)。
_NOPROXY_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


class _QuietHandler(SimpleHTTPRequestHandler):
    """Suppress the default per-request stderr logging.

    额外提供 ``/pkg/<package>/<相对路径>`` 路由: 把 ament share 目录里的文件
    (URDF 引用的 STL/DAE 网格等)开放给浏览器 —— 前端 3D 视图把
    ``package://rm_description/meshes/x.STL`` 映射成
    ``/pkg/rm_description/meshes/x.STL`` 即可加载机器人模型。
    """

    # web_video_server 的端口(由节点参数注入); 浏览器经 WSL2 localhost 转发直连
    # 8081 常不通(空响应), 故走本服务的同源代理 /video/snapshot, 内部本地取 8081。
    video_port = 8081

    def log_message(self, format, *args):  # noqa: A002 - signature dictated by base
        return

    def do_GET(self):
        if urlsplit(self.path).path == '/video/snapshot':
            return self._proxy_snapshot()
        return super().do_GET()

    def _proxy_snapshot(self):
        """把 /video/snapshot?topic=...&quality=... 在本地转发到
        web_video_server(127.0.0.1:<video_port>)/snapshot, 取回单帧 JPEG 回给浏览器。
        浏览器只连本服务(8000, 能过 WSL2 转发), 8081 仅在 WSL 内部本地访问。"""
        # parse_qs 会 url 解码(%2F -> /); 再用 urlencode(safe='/') 把斜杠原样发给
        # web_video_server —— 它不解码 %2F, 收到 %2F 会找不到话题(HTTP 000/空)。
        q = parse_qs(urlsplit(self.path).query)
        topic = (q.get('topic') or [''])[0]
        if not topic:
            self.send_error(400, 'missing topic')
            return
        params = {'topic': topic}
        if q.get('quality'):
            params['quality'] = q['quality'][0]
        upstream = (f'http://127.0.0.1:{self.video_port}/snapshot?'
                    + urlencode(params, safe='/'))
        try:
            with _NOPROXY_OPENER.open(upstream, timeout=5) as resp:
                data = resp.read()
                ctype = resp.headers.get('Content-Type', 'image/jpeg')
        except Exception:  # noqa: BLE001  上游没起/没帧 -> 502, 前端会重试
            try:
                self.send_error(502, 'web_video_server snapshot unavailable')
            except Exception:  # noqa: BLE001
                pass
            return
        try:
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)
        except Exception:  # noqa: BLE001  浏览器中途断开 -> 忽略
            pass

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

        self.declare_parameter('port', 8000)
        self.declare_parameter('address', '0.0.0.0')
        self.declare_parameter('web_root', '')
        self.declare_parameter('video_port', 8081)

        port = int(self.get_parameter('port').value)
        address = str(self.get_parameter('address').value)
        web_root = str(self.get_parameter('web_root').value)
        # 同源视频代理 /video/snapshot 的上游(本地 web_video_server)端口。
        _QuietHandler.video_port = int(self.get_parameter('video_port').value)

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
