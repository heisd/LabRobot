# 运行时修改参数指南

## 方法1: 使用 `ros2 param set` 命令（推荐）

在终端中运行以下命令，可以在运行时动态修改目标距离：

```bash
# 修改目标距离为 1.0 米
ros2 param set /image_converter targetDist_ 1.0

# 修改目标距离为 0.5 米
ros2 param set /image_converter targetDist_ 0.5

# 修改目标距离为 1.5 米
ros2 param set /image_converter targetDist_ 1.5
```

**说明：**
- 节点名称：`/image_converter`（在 run_tracker.h 中定义）
- 参数名称：`targetDist_`（注意末尾有下划线）
- 单位：米
- **实时生效**：由于代码在每次深度回调时都会读取参数，修改后会在下一次深度回调时生效

## 方法2: 查看当前参数值

```bash
# 查看所有参数
ros2 param list /image_converter

# 查看目标距离参数的当前值
ros2 param get /image_converter targetDist_

# 查看所有参数的详细信息
ros2 param describe /image_converter targetDist_
```

## 方法3: 在 Launch 时覆盖参数

如果使用 `ros2 launch` 启动，可以通过命令行覆盖参数：

```bash
ros2 launch wheeltec_robot_kcf wheeltec_robot_kcf.launch.py targetDist_:=1.2
```

## 方法4: 使用 `ros2 param load` 加载参数文件

创建参数文件 `params.yaml`：

```yaml
image_converter:
  ros__parameters:
    targetDist_: 1.0
    linear_KP_: 3.0
    linear_KI_: 0.0
    linear_KD_: 1.0
    angular_KP_: 0.5
    angular_KI_: 0.0
    angular_KD_: 2.0
```

然后加载：
```bash
ros2 param load /image_converter params.yaml
```

## 其他可修改的参数

```bash
# PID 控制参数
ros2 param set /image_converter linear_KP_ 3.0
ros2 param set /image_converter linear_KI_ 0.0
ros2 param set /image_converter linear_KD_ 1.0
ros2 param set /image_converter angular_KP_ 0.5
ros2 param set /image_converter angular_KI_ 0.0
ros2 param set /image_converter angular_KD_ 2.0
```

## 验证参数修改

修改参数后，查看节点的输出日志，应该能看到新的目标距离值：
```
targetDist(m): 1.0
current_dist(m): 0.85
```

