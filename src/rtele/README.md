# RTele - Remote Teleoperation Application

## 设计
基于火山云veRTC设计，实现机器人远程遥控功能。

## 构建方式

本项目支持两种构建方式：
1. **独立CMake构建** - 适合开发和测试
2. **ROS2 colcon构建** - 适合与ROS2包集成使用

---

## 方式1：独立CMake构建

### 前置要求

- CMake 3.16+
- C++17 编译器 (GCC 7+ 或 Clang 5+)
- Git
- VolcEngineRTC SDK (Linux x86_64 或 ARM64)

### 构建步骤

```bash
# 创建构建目录
mkdir build && cd build

# 配置 CMake
cmake ..

# 编译（使用多核加速）
cmake --build . -j$(nproc)

# 运行程序
./bin/rtele \
  --room_id=room001 \
  --user_id=user001 \
  --app_id=your_app_id \
  --app_key=your_app_key \
  --enable_publish_video=true
```

### 第三方依赖

以下依赖库将在首次编译时自动下载并从源码编译：
- **Abseil C++** (20240116.2) - Google 的 C++ 基础库
- **fmt** (10.2.1) - 格式化库
- **spdlog** (v1.12.0) - 快速的 C++ 日志库
- **OpenCV** (4.8.1) - 计算机视觉库（仅编译 core, imgcodecs, imgproc 模块）

注意：首次编译会下载并编译这些依赖库，可能需要较长时间（10-30分钟）。后续编译会使用缓存，速度会快很多。

### 编译选项

```bash
# Debug 模式编译
cmake .. -DCMAKE_BUILD_TYPE=Debug

# Release 模式编译（默认）
cmake .. -DCMAKE_BUILD_TYPE=Release

# ARM64 平台
cmake .. -DUSE_ARM64=ON

# 清理并重新编译
rm -rf build && mkdir build && cd build && cmake .. && cmake --build . -j$(nproc)
```

---

## 方式2：ROS2 colcon构建

### 前置要求

- ROS2 (Humble/Iron/Rolling)
- 所有方式1中的依赖

### 工作空间设置

```bash
# 0. Source ROS2环境（重要！）
# 如果遇到错误，尝试在新终端中执行，或先unset所有ROS相关变量
# 如果用的是zsh就source对应的zsh文件
source /opt/ros/humble/setup.bash   # 或你的ROS2版本：iron, rolling等

# 验证ROS2环境已加载
echo $ROS_DISTRO  # 应该输出：humble

# 1. 创建ROS2工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. 链接rtele包
ln -s /path/to/rtele ./rtele

# 3. 链接third_party中的ROS2包（消息定义等）
ln -s /path/to/rtele/third_party/hdas_msg ./hdas_msg
ln -s /path/to/rtele/third_party/system_manager_msg ./system_manager_msg
ln -s /path/to/rtele/third_party/teleoperation_msg_ros2 ./teleoperation_msg_ros2
ln -s /path/to/rtele/third_party/galaxea_robot_tele ./galaxea_robot_tele

# 4. 编译
cd ~/ros2_ws
colcon build

# 5. Source环境
source install/setup.bash

# 6. 运行
ros2 run rtele rtele \
  --room_id=room001 \
  --user_id=user001 \
  --app_id=your_app_id \
  --app_key=your_app_key
```

### 只编译rtele及其依赖

```bash
cd ~/ros2_ws
colcon build --packages-up-to rtele
```

### 常见问题

**Q: 编译时提示找不到ament_cmake**
```bash
# 确保已经source ROS2环境
source /opt/ros/humble/setup.bash  # 根据你的ROS2版本调整
```

**Q: 每次打开终端都要source吗？**
```bash
# 可以添加到~/.bashrc或~/.zshrc自动加载
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### ROS2集成说明

当使用colcon构建时，rtele会：
- 自动启用ROS2支持（定义`USE_ROS2`宏）
- 使用FetchContent编译第三方依赖（Abseil、spdlog、OpenCV等）
- 链接ROS2消息包（hdas_msg、system_manager_msg等）
- 可以在代码中使用`#ifdef USE_ROS2`条件编译ROS2相关功能

---

## 平台支持

项目支持以下平台：
- **x86_64**: 使用 VolcEngineRTC_Linux_x86_64
- **aarch64 (ARM64)**: 使用 VolcEngineRTC_Linux_3.58.1.500_aarch64

CMake 会自动检测系统架构并选择对应的 SDK。

## 使用方法

### 临时测试
不同用户生成不同的临时token，在火山云veRTC控制台生成临时token，需要手动生成（在控制台指定UserID，RoomID，有效期等参数）

### 日产使用
不同用户通过调用相关服务API，申请Token使用，目前我们还不支持

## 基础概念
参考https://www.volcengine.com/docs/6348/70120?lang=zh
需要理解RoomID，UserID（不能重复），推流拉流，点对点，广播消息的概念

音视频流的限制参考https://www.volcengine.com/docs/6348/257549?lang=zh

## 运行前置条件
参考https://www.volcengine.com/docs/6348/141350?lang=zh，
源码里面的SDK版本，VolcEngineRTC_Linux_3.60.102.4700_x86_64_Release

## 命令行参数

### 必需参数

- `--room_id`: 房间 ID
- `--user_id`: 用户 ID
- `--app_id`: 应用 ID (从控制台获取)
- `--app_key`: 应用密钥 (从控制台获取)

### 可选参数

- `--enable_video`: 是否启用视频 (默认: true)

## 目录结构

- `deploy/` - K8s4r deployment configurations
- `third_party/` - Third-party libraries (VolcEngineRTC SDK)
- `main.cc` - Application entry point
- `BUILD.bazel` - Bazel build configuration
- `MODULE.bazel` - Bazel module configuration

## 部署

将应用部署到 K8s4r 集群：

```bash
kubectl apply -f deploy/
```
