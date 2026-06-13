# qt_ros_test

WheelTec 机器人 Qt5 上位机示例工程,基于 QtWidgets + rclcpp 提供一个图形界面客户端,显示相机图像、电池电压、里程计速度,并通过 GUI 控件发布 `cmd_vel` 控制机器人。

## 概述

包内是一个独立的 Qt 应用,启动后会:
- 订阅相机图像(普通或压缩 RGB / 深度)进行预览。
- 订阅 `/PowerVoltage` 与 `/odom` 显示电压与速度。
- 通过界面按钮 / 仪表盘发布 `cmd_vel`。
- 支持执行远程脚本(需要虚拟机 root 密码参数)。

## 目录结构

```
qt_ros_test/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
│   ├── main.cpp                 # Qt 应用入口
│   ├── main_window.cpp          # 主窗口逻辑
│   ├── qnode.cpp                # ROS2 后端线程
│   └── CCtrlDashBoard.cpp       # 自定义控制仪表盘控件
├── ui/                          # Qt Designer 文件
├── resources/                   # 图片 / 图标
├── script/                      # 辅助脚本
└── launch/
    ├── qt_ros_test.launch.py
    └── keyboard_camera.launch.py
```

## 依赖项

- buildtool: `ament_cmake`
- depend: `rclcpp`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`cv_bridge`、`Qt5`(`Qt5::Widgets`、`Qt5::Network`)、`OpenCV`

## 节点说明

### `qt_ros_test`(可执行)

订阅:
- `<rgbtopic>` (sensor_msgs/CompressedImage) — RGB 压缩图像(默认 `/camera/color/image_raw/compressed`)
- `<depthtopic>` (sensor_msgs/Image) — 深度图像(默认 `/camera/depth/image_raw`)
- `/PowerVoltage` (std_msgs/Float32) — 电池电压
- `/odom` (nav_msgs/Odometry) — 机器人里程计

发布:
- `cmd_vel` (geometry_msgs/Twist) — 通过界面按钮/仪表盘控制底盘

参数:
| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `user_passward` | `dongguan` | 虚拟机 root 密码(用于上位机执行 sudo 操作) |
| `rgbtopic` | `/camera/color/image_raw/compressed` | RGB 图像话题 |
| `depthtopic` | `/camera/depth/image_raw` | 深度图像话题 |

## 启动文件

- **`qt_ros_test.launch.py`** — 启动 Qt 上位机节点,传入 RGB/深度话题与 root 密码参数。
- **`keyboard_camera.launch.py`** — 启动键盘 + 相机的辅助 launch,便于在调试 Qt 界面时一起测试。

## 编译与运行

```bash
# 确保已安装 Qt5
sudo apt install qtbase5-dev qtdeclarative5-dev

colcon build --packages-up-to qt_ros_test
source install/setup.bash
ros2 launch qt_ros_test qt_ros_test.launch.py
```

## 使用示例

```bash
# 1. 启动底盘 + 相机
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py

# 2. 启动上位机
ros2 launch qt_ros_test qt_ros_test.launch.py
# 通过界面按钮控制小车、查看图像/电压/速度
```

## 注意事项

1. 必须在桌面环境中运行(需要 X11/Wayland),纯命令行机器人主板请用 ssh -X 转发。
2. `user_passward` 用于上位机调用 sudo,**部署到生产环境时务必修改默认值或改用密钥**。
3. RGB 话题默认是 compressed,若相机驱动只发布裸图像,需修改参数。
4. 与其他 cmd_vel 发布者同时使用会冲突。

## 在WSL Humble 编译如下
```bash
> colcon build --packages-select qt_ros_test
Starting >>> qt_ros_test
[Processing: qt_ros_test]                             
[Processing: qt_ros_test]                                     
--- stderr: qt_ros_test                                          
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In destructor ‘virtual qt_ros_test::MainWindow::~MainWindow()’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:86:22: warning: ‘static int QProcess::execute(const QString&)’ is deprecated: Use QProcess::execute(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
   86 |     QProcess::execute("killall -s INT -w ros2\n");
      |     ~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:265:16: note: declared here
  265 |     static int execute(const QString &command);
      |                ^~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::init_cmd()’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:114:16: warning: ‘void QProcess::start(const QString&, QIODevice::OpenMode)’ is deprecated: Use QProcess::start(const QString &program, const QStringList &arguments,OpenMode mode = ReadWrite) instead [-Wdeprecated-declarations]
  114 |     _cmd->start("bash");
      |     ~~~~~~~~~~~^~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:168:10: note: declared here
  168 |     void start(const QString &command, OpenMode mode = ReadWrite);
      |          ^~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::closeroslaunch()’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:143:19: warning: ‘void QProcess::start(const QString&, QIODevice::OpenMode)’ is deprecated: Use QProcess::start(const QString &program, const QStringList &arguments,OpenMode mode = ReadWrite) instead [-Wdeprecated-declarations]
  143 |     killall->start("sh /home/wheeltec/wheeltec_ros2/src/qt_ros_test/script/killall.sh\n");
      |     ~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:168:10: note: declared here
  168 |     void start(const QString &command, OpenMode mode = ReadWrite);
      |          ^~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::slot_cmd_read()’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:158:10: warning: ISO C++ forbids variable length array ‘buffer’ [-Wvla]
  158 |     char buffer[maxSize];
      |          ^~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::ReadSettings()’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:214:10: warning: unused variable ‘remember’ [-Wunused-variable]
  214 |     bool remember = settings.value("remember_settings", false).toBool();
      |          ^~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:215:10: warning: unused variable ‘checked’ [-Wunused-variable]
  215 |     bool checked = settings.value("use_environment_variables", false).toBool();
      |          ^~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::on_pushButton_2dmap_clicked(bool)’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:498:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  498 |         rviz_cmd->startDetached("rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:504:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  504 |         rviz_cmd->startDetached("killall -2 rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::on_pushButton_2dmap_2_clicked()’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:527:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  527 |         rviz_cmd->startDetached("rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:533:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  533 |         rviz_cmd->startDetached("killall -2 rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::on_pushButton_2dmap_3_clicked()’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:556:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  556 |         rviz_cmd->startDetached("rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:562:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  562 |         rviz_cmd->startDetached("killall -2 rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::on_pushButton_2dnav_clicked(bool)’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:600:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  600 |         rviz_cmd->startDetached("rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:606:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  606 |         rviz_cmd->startDetached("killall -2 rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::on_pushButton_3dmap_clicked(bool)’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:628:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  628 |         rviz_cmd->startDetached("rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:634:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  634 |         rviz_cmd->startDetached("killall -2 rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp: In member function ‘void qt_ros_test::MainWindow::on_pushButton_3dnav_clicked(bool)’:
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:662:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  662 |         rviz_cmd->startDetached("rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
/home/li/Lab/src/qt_ros_test/src/main_window.cpp:668:32: warning: ‘static bool QProcess::startDetached(const QString&)’ is deprecated: Use QProcess::startDetached(const QString &program, const QStringList &arguments) instead [-Wdeprecated-declarations]
  668 |         rviz_cmd->startDetached("killall -2 rviz2");
      |         ~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~
In file included from /usr/include/x86_64-linux-gnu/qt5/QtCore/QtCore:170,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGuiDepends:3,
                 from /usr/include/x86_64-linux-gnu/qt5/QtGui/QtGui:3,
                 from /home/li/Lab/src/qt_ros_test/src/main_window.cpp:12:
/usr/include/x86_64-linux-gnu/qt5/QtCore/qprocess.h:280:17: note: declared here
  280 |     static bool startDetached(const QString &command);
      |                 ^~~~~~~~~~~~~
---
Finished <<< qt_ros_test [1min 14s]

Summary: 1 package finished [1min 14s]
  1 package had stderr output: qt_ros_test
```
