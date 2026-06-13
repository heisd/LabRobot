# ROS 2 Colcon 编译指南 (ROS 2 Humble)

本工作空间集成的 ROS 2 功能包（移动底盘 + 机械臂）统一通过 colcon 进行构建。

## 1. 编译前准备

### 1.1 拉取子模块
部分核心依赖（如 `yolo_ros`）以 Git 子模块形式存在：
```bash
git submodule update --init --recursive
```

### 1.2 安装系统依赖
在根目录下运行：
```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 2. 核心编译命令

### 2.1 推荐编译方式 (全量)
```bash
colcon build --symlink-install
```
*   使用 `--symlink-install` 后，修改脚本或资源文件无需重新编译。

### 2.2 内存优化编译 (防止卡死)
如果设备内存较小（如编译 MoveIt 或视觉算法包时），请限制并发数：
```bash
colcon build --symlink-install --parallel-workers 2
```

### 2.3 增量/失败后续编译
```bash
colcon build --packages-skip-build-finished --continue-on-error
```

## 3. 指定包编译

当功能包较多时，推荐只编译特定的子系统：

| 目标 | 编译命令 |
| --- | --- |
| **仅编译底盘** | `colcon build --packages-up-to turn_on_wheeltec_robot` |
| **仅编译机械臂** | `colcon build --packages-up-to lebai_driver` |
| **仅编译视觉抓取** | `colcon build --packages-up-to grab_demo` |
| **仅编译单个包** | `colcon build --packages-select <包名>` |

## 4. 构建顺序与依赖查看

`colcon` 会自动解析 `package.xml` 中的依赖关系并决定构建顺序。

### 4.1 查看构建顺序
列出所有包及其构建路径（按构建顺序排序）：
```bash
colcon list
```

### 4.2 可视化依赖图
在终端查看功能包之间的依赖逻辑：
```bash
colcon graph
```

### 4.3 依赖逻辑说明
*   **接口包 (Interfaces)** 总是最先编译。
*   **应用包 (App/Demo)** 总是最后编译。
*   **并行构建**: 无相互依赖的包会同时开始编译。

## 5. 环境配置

编译完成后，运行以下命令刷新环境变量：
```bash
source install/setup.bash
```

## 5. 常见问题排查

*   **编译报错找不到包**: 请确认是否已运行 `git submodule update`。
*   **权限问题**: 请勿使用 `sudo colcon build`，否则会导致后续权限错误。
*   **彻底清理**: 如遇不可恢复的构建冲突，可运行 `rm -rf build/ install/ log/` 后重新编译。
