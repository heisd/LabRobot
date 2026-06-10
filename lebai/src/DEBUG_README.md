# VSCode ROS 2 节点调试指南

## 配置说明

已为您配置了以下 ROS 2 节点的调试环境：

1. **hand_eye** - 手眼标定节点
2. **charuco_dectet_node** - ChArUco 标定板识别节点
3. **aruco_dectet** - ArUco 标记识别节点

## 使用方法

### 1. 确保 Debug 模式编译

在调试前，需要确保代码以 Debug 模式编译（包含调试符号）：

```bash
cd /home/dxf/wheeltec_lebai_humble_2024_7_17
colcon build --packages-select grab_demo --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

或者使用 VSCode 的任务：
- 按 `Ctrl+Shift+P` (或 `Cmd+Shift+P` on Mac)
- 输入 "Tasks: Run Task"
- 选择 "colcon build grab_demo"

### 2. 设置 ROS 2 环境

在调试前，确保终端中已设置 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/dxf/wheeltec_lebai_humble_2024_7_17/install/setup.bash
```

**注意**：VSCode 的调试器会自动设置基本环境变量，但某些 ROS 2 功能可能需要完整的环境。如果遇到问题，可以在终端中手动运行节点进行测试。

### 3. 启动调试

1. 打开 VSCode 的调试面板（按 `F5` 或点击左侧调试图标）
2. 在顶部下拉菜单中选择要调试的节点：
   - "调试 hand_eye 节点"
   - "调试 charuco_dectet_node 节点"
   - "调试 aruco_dectet 节点"
3. 点击绿色播放按钮或按 `F5` 开始调试

### 4. 设置断点

在源代码文件中点击行号左侧，设置断点。当程序执行到断点时会自动暂停。

### 5. 调试功能

- **继续执行** (`F5`): 继续执行直到下一个断点
- **单步跳过** (`F10`): 执行当前行，不进入函数内部
- **单步进入** (`F11`): 进入函数内部
- **单步跳出** (`Shift+F11`): 跳出当前函数
- **重启** (`Ctrl+Shift+F5`): 重新启动调试
- **停止** (`Shift+F5`): 停止调试

## 常见问题

### 问题 1: 找不到可执行文件

**解决方案**: 确保已经编译了包：
```bash
colcon build --packages-select grab_demo
```

### 问题 2: 节点启动失败，提示找不到 ROS 2 环境

**解决方案**: 
1. 确保在终端中 source 了 ROS 2 环境
2. 或者修改 launch.json，在 `environment` 中添加更多环境变量

### 问题 3: 断点不生效

**解决方案**: 
1. 确保以 Debug 模式编译（包含 `-g` 标志）
2. 检查 CMakeLists.txt 中是否设置了 `CMAKE_BUILD_TYPE=Debug`
3. 重新编译包

### 问题 4: 调试时看不到变量值

**解决方案**: 
1. 确保代码以 Debug 模式编译
2. 检查 gdb 是否正确安装：`which gdb`
3. 尝试在断点处使用调试控制台查看变量

## 调试多个节点

如果需要同时调试多个节点：

1. 在一个终端中手动启动其他节点（如相机节点）
2. 在 VSCode 中调试目标节点
3. 使用 `ros2 topic list` 检查话题连接

## 附加到运行中的进程

如果需要调试已经运行的节点：

1. 找到进程 ID: `ps aux | grep <节点名>`
2. 在 launch.json 中添加 "attach" 配置
3. 使用进程 ID 附加到进程

## 提示

- 调试时建议关闭优化，使用 `-O0` 编译选项
- 可以在 CMakeLists.txt 中添加：
  ```cmake
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
  ```
- 使用 `ros2 node list` 检查节点是否正常运行
- 使用 `ros2 topic echo <topic_name>` 检查话题数据

