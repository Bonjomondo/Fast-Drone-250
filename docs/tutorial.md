# Fast-Drone-250 自主空中机器人详细教程

## 目录

- [学习路线](#学习路线)
- [教程大纲](#教程大纲)
- [第一章：项目概述与准备工作](#第一章项目概述与准备工作)
- [第二章：ROS基础知识](#第二章ros基础知识)
- [第三章：硬件组装与配置](#第三章硬件组装与配置)
- [第四章：环境感知模块](#第四章环境感知模块)
- [第五章：视觉惯性里程计(VINS)](#第五章视觉惯性里程计vins)
- [第六章：栅格地图构建](#第六章栅格地图构建)
- [第七章：路径搜索(A*算法)](#第七章路径搜索a算法)
- [第八章：B样条轨迹优化](#第八章b样条轨迹优化)
- [第九章：Ego-Planner规划框架](#第九章ego-planner规划框架)
- [第十章：飞行控制模块(PX4Ctrl)](#第十章飞行控制模块px4ctrl)
- [第十一章：仿真实验](#第十一章仿真实验)
- [第十二章：实机飞行](#第十二章实机飞行)

---

## 学习路线

### 适用人群
本教程适合以下人员学习：
- 对自主无人机感兴趣的高校学生
- 机器人领域的研究人员
- 无人机行业从业者
- 具有一定编程基础的爱好者

### 学习路线图

```
基础阶段 (2-4周)
├── C++编程基础
├── ROS机器人操作系统基础
├── Linux操作系统基础
└── 基础数学知识（线性代数、概率论）

进阶阶段 (4-6周)
├── 机器人运动学与动力学
├── 状态估计与SLAM基础
├── 路径规划算法
└── 控制理论基础

实践阶段 (4-8周)
├── 硬件组装与调试
├── 软件环境配置
├── 仿真实验
└── 实机飞行测试
```

### 推荐学习顺序

1. **第一步：掌握基础知识**
   - 学习ROS入门（推荐古月居ROS教程）
   - 熟悉C++编程和CMake构建系统
   - 了解Ubuntu Linux系统操作

2. **第二步：理解理论基础**
   - 学习高飞老师的《移动机器人运动规划》课程
   - 理解四旋翼动力学模型
   - 学习状态估计基础（卡尔曼滤波、VIO）

3. **第三步：项目实践**
   - 完成硬件组装
   - 配置软件环境
   - 进行仿真测试
   - 实机飞行实验

---

## 教程大纲

| 章节 | 内容 | 难度 | 预计学时 |
|------|------|------|----------|
| 第一章 | 项目概述与准备工作 | ⭐ | 2小时 |
| 第二章 | ROS基础知识 | ⭐⭐ | 10小时 |
| 第三章 | 硬件组装与配置 | ⭐⭐ | 8小时 |
| 第四章 | 环境感知模块 | ⭐⭐⭐ | 6小时 |
| 第五章 | 视觉惯性里程计(VINS) | ⭐⭐⭐⭐ | 10小时 |
| 第六章 | 栅格地图构建 | ⭐⭐⭐ | 8小时 |
| 第七章 | 路径搜索(A*算法) | ⭐⭐⭐ | 6小时 |
| 第八章 | B样条轨迹优化 | ⭐⭐⭐⭐ | 12小时 |
| 第九章 | Ego-Planner规划框架 | ⭐⭐⭐⭐ | 10小时 |
| 第十章 | 飞行控制模块(PX4Ctrl) | ⭐⭐⭐ | 8小时 |
| 第十一章 | 仿真实验 | ⭐⭐ | 4小时 |
| 第十二章 | 实机飞行 | ⭐⭐⭐⭐ | 20小时 |

---

## 第一章：项目概述与准备工作

### 前置知识
- 基本的编程概念
- Linux命令行基础操作
- 了解什么是无人机

### 1.1 项目简介

Fast-Drone-250是浙江大学FAST-Lab实验室开源的自主无人机项目，该项目实现了一套完整的自主飞行系统，包括：

- **感知系统**：使用Intel RealSense D435i深度相机获取环境信息
- **定位系统**：基于VINS-Fusion的视觉惯性里程计
- **规划系统**：Ego-Planner轨迹规划算法
- **控制系统**：PX4飞控 + px4ctrl位置控制器

### 1.2 系统架构概览

```
┌─────────────────────────────────────────────────────────────────┐
│                        Fast-Drone-250 系统架构                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐        │
│  │  RealSense  │────▶│  VINS-Fusion │────▶│  Ego-Planner│        │
│  │  D435i      │     │  状态估计    │     │  轨迹规划   │        │
│  └─────────────┘     └─────────────┘     └─────────────┘        │
│         │                   │                   │                │
│         ▼                   ▼                   ▼                │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐        │
│  │  深度图像   │     │  位姿/速度  │     │  轨迹指令   │        │
│  └─────────────┘     └─────────────┘     └─────────────┘        │
│         │                                       │                │
│         ▼                                       ▼                │
│  ┌─────────────┐                        ┌─────────────┐         │
│  │  Grid Map   │                        │  px4ctrl    │         │
│  │  栅格地图   │                        │  姿态控制   │         │
│  └─────────────┘                        └─────────────┘         │
│                                               │                  │
│                                               ▼                  │
│                                        ┌─────────────┐          │
│                                        │  PX4 飞控   │          │
│                                        └─────────────┘          │
│                                               │                  │
│                                               ▼                  │
│                                        ┌─────────────┐          │
│                                        │  电机/电调  │          │
│                                        └─────────────┘          │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 硬件清单

详细硬件清单请参考项目根目录下的 `purchase_list.xlsx` 文件。

主要硬件包括：
- 机架：Q250碳纤维机架（250mm轴距）
- 飞控：Holybro Pixhawk 4 / Pixhawk V5+
- 机载电脑：Intel NUC
- 深度相机：Intel RealSense D435i
- 动力系统：2306电机 + 5寸螺旋桨
- 电池：4S 2300mAh航模电池
- 遥控器：乐迪AT9S

### 1.4 软件环境要求

- **操作系统**：Ubuntu 20.04 LTS
- **ROS版本**：ROS Noetic
- **依赖库**：
  - OpenCV 4.x
  - Eigen 3.x
  - Ceres Solver 2.x
  - PCL (Point Cloud Library)

### 1.5 代码结构说明

```
Fast-Drone-250/
├── src/
│   ├── planner/                    # 规划模块
│   │   ├── bspline_opt/           # B样条优化
│   │   ├── plan_env/              # 规划环境（栅格地图）
│   │   ├── plan_manage/           # 规划管理（Ego-Planner核心）
│   │   ├── path_searching/        # 路径搜索（A*算法）
│   │   └── traj_utils/            # 轨迹工具类
│   │
│   ├── realflight_modules/        # 实飞模块
│   │   ├── VINS-Fusion/           # 视觉惯性里程计
│   │   ├── px4ctrl/               # 位置控制器
│   │   └── realsense-ros/         # 相机驱动
│   │
│   ├── uav_simulator/             # 仿真模块
│   │   ├── so3_quadrotor_simulator/  # 四旋翼仿真器
│   │   ├── local_sensing/         # 局部感知仿真
│   │   └── map_generator/         # 地图生成
│   │
│   └── utils/                     # 工具模块
│       ├── quadrotor_msgs/        # 消息定义
│       └── uav_utils/             # 常用工具函数
│
├── shfiles/                       # Shell脚本
├── firmware/                      # PX4固件
└── carbon_board/                  # 碳板设计文件
```

---

## 第二章：ROS基础知识

### 前置知识
- C++编程基础
- Linux命令行操作
- 基本的面向对象编程概念

### 2.1 ROS核心概念

ROS（Robot Operating System）是一个用于编写机器人软件的框架。本项目基于ROS Noetic开发。

#### 2.1.1 节点（Node）

节点是ROS中执行计算的进程。在本项目中，主要节点包括：

```cpp
// 示例：ego_planner_node.cpp 中的节点初始化
int main(int argc, char **argv)
{
    // 初始化ROS节点，节点名称为"ego_planner_node"
    ros::init(argc, argv, "ego_planner_node");
    
    // 创建节点句柄
    ros::NodeHandle nh("~");
    
    // 创建规划器实例
    EGOReplanFSM rebo_replan;
    rebo_replan.init(nh);
    
    // 进入循环
    ros::spin();
    return 0;
}
```

#### 2.1.2 话题（Topic）

话题是节点之间通信的通道。发布者（Publisher）发布消息，订阅者（Subscriber）接收消息。

项目中的关键话题：

| 话题名称 | 消息类型 | 描述 |
|---------|----------|------|
| `/odom_world` | `nav_msgs/Odometry` | 里程计数据 |
| `/planning/bspline` | `traj_utils/Bspline` | B样条轨迹 |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | 位置控制指令 |
| `/mavros/imu/data_raw` | `sensor_msgs/Imu` | IMU数据 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图像 |

#### 2.1.3 服务（Service）

服务用于请求-响应式的通信。在本项目中主要用于：
- 切换飞控模式
- 解锁/上锁电机

```cpp
// px4ctrl中使用服务切换飞控模式
mavros_msgs::SetMode offb_set_mode;
offb_set_mode.request.custom_mode = "OFFBOARD";
set_FCU_mode_srv.call(offb_set_mode);
```

### 2.2 ROS编译系统

本项目使用`catkin_make`进行编译：

```bash
# 进入工作空间根目录
cd Fast-Drone-250

# 编译
catkin_make

# 设置环境变量
source devel/setup.bash
```

### 2.3 Launch文件

Launch文件用于批量启动多个节点。关键的launch文件：

**single_run_in_sim.launch** - 仿真环境启动：
```xml
<launch>
    <!-- 地图大小参数 -->
    <arg name="map_size_x" value="40.0"/>
    <arg name="map_size_y" value="40.0"/>
    <arg name="map_size_z" value="5.0"/>
    
    <!-- 启动Ego-Planner节点 -->
    <node pkg="ego_planner" type="ego_planner_node" name="ego_planner_node">
        <!-- 各种参数配置 -->
    </node>
    
    <!-- 启动轨迹服务器 -->
    <node pkg="ego_planner" type="traj_server" name="traj_server">
    </node>
</launch>
```

---

## 第三章：硬件组装与配置

### 前置知识
- 基本的焊接技能
- 电气安全常识
- 四旋翼基本结构知识

### 3.1 机架组装

1. **安装电机**：按照机架说明书安装四个电机
2. **安装电调**：将电调焊接到分电板
3. **安装飞控**：使用减震垫固定飞控

⚠️ **注意**：电机转向和信号线顺序必须正确！

```
         前
    1 ↺    ↻ 2
         ╲ ╱
          ╳
         ╱ ╲
    4 ↻    ↺ 3
         后

说明：
- 1、3号电机顺时针转（CCW螺旋桨）
- 2、4号电机逆时针转（CW螺旋桨）
```

### 3.2 飞控配置

使用QGroundControl进行飞控设置：

1. **烧录固件**
   ```bash
   # 使用项目提供的固件
   firmware/px4_fmu-v5_default.px4
   ```

2. **关键参数设置**
   ```
   机架类型: Generic 250 Racer
   dshot_config: dshot600
   CBRK_SUPPLY_CHK: 894281
   CBRK_USB_CHK: 197848
   CBRK_IO_SAFETY: 22027
   SER_TEL1_BAUD: 921600
   ```

3. **提高IMU发布频率**
   
   在SD卡创建 `/etc/extras.txt`：
   ```
   mavlink stream -d /dev/ttyACM0 -s ATTITUDE_QUATERNION -r 200
   mavlink stream -d /dev/ttyACM0 -s HIGHRES_IMU -r 200
   ```

### 3.3 机载电脑配置

1. **安装Ubuntu 20.04**
2. **安装ROS Noetic**
3. **安装依赖库**
4. **编译项目代码**

详细步骤请参考 `readme.md`。

---

## 第四章：环境感知模块

### 前置知识
- 计算机视觉基础
- 相机模型（针孔相机模型）
- 深度图像原理

### 4.1 Intel RealSense D435i

D435i是一款立体视觉深度相机，具有以下特点：

- **双目红外相机**：用于深度估计
- **RGB相机**：用于彩色图像采集
- **IMU**：用于运动估计（但精度不够，本项目使用飞控IMU）

### 4.2 RealSense ROS驱动

启动相机驱动：
```bash
roslaunch realsense2_camera rs_camera.launch
```

发布的主要话题：
- `/camera/depth/image_rect_raw`: 深度图像
- `/camera/infra1/image_rect_raw`: 左红外图像
- `/camera/infra2/image_rect_raw`: 右红外图像

### 4.3 深度图像处理

在`grid_map.cpp`中，深度图像被投影到3D空间：

```cpp
void GridMap::projectDepthImage()
{
    // 遍历深度图像的每个像素
    for (int v = 0; v < rows; v += skip_pix)
    {
        for (int u = 0; u < cols; u += skip_pix)
        {
            // 获取深度值
            depth = (*row_ptr) / mp_.k_depth_scaling_factor_;
            
            // 使用相机内参将像素坐标转换为3D坐标
            // pt_cur = K^(-1) * [u, v, 1]^T * depth
            pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
            pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
            pt_cur(2) = depth;
            
            // 将相机坐标系下的点转换到世界坐标系
            pt_world = camera_r * pt_cur + md_.camera_pos_;
        }
    }
}
```

**关键参数说明**：
- `fx`, `fy`: 相机焦距（像素单位）
- `cx`, `cy`: 主点坐标
- `k_depth_scaling_factor`: 深度缩放因子（D435通常为1000）

---

## 第五章：视觉惯性里程计(VINS)

### 前置知识
- 视觉SLAM基础
- 卡尔曼滤波
- 光流法
- IMU预积分

### 5.1 VINS-Fusion简介

VINS-Fusion是一种鲁棒的视觉惯性里程计系统，结合视觉和IMU信息进行状态估计。

**核心功能**：
- 实时位姿估计
- 在线外参标定
- 回环检测

### 5.2 系统流程

```
┌───────────────┐    ┌───────────────┐
│  双目图像     │    │  IMU数据      │
└───────┬───────┘    └───────┬───────┘
        │                    │
        ▼                    ▼
┌───────────────┐    ┌───────────────┐
│  特征提取     │    │  IMU预积分    │
│  与光流跟踪   │    │              │
└───────┬───────┘    └───────┬───────┘
        │                    │
        └────────┬───────────┘
                 ▼
        ┌───────────────┐
        │  非线性优化    │
        │  (Ceres)      │
        └───────┬───────┘
                 │
                 ▼
        ┌───────────────┐
        │  位姿输出      │
        │  速度输出      │
        └───────────────┘
```

### 5.3 参数配置

配置文件位于：`src/realflight_modules/VINS-Fusion/config/`

**fast-drone-250.yaml 关键参数**：

```yaml
# 相机内参
cam0_intrinsics: [fx, fy, cx, cy]
cam1_intrinsics: [fx, fy, cx, cy]

# 相机-IMU外参
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   data: [...]  # 4x4变换矩阵

# IMU参数
acc_n: 0.1          # 加速度计噪声
gyr_n: 0.01         # 陀螺仪噪声
acc_w: 0.001        # 加速度计随机游走
gyr_w: 0.0001       # 陀螺仪随机游走
```

### 5.4 外参标定

外参标定的目的是精确确定相机与IMU之间的相对位姿。

**标定步骤**：
1. 启动VINS和相机驱动
2. 缓慢移动无人机
3. 观察外参是否收敛
4. 将标定结果写入配置文件

---

## 第六章：栅格地图构建

### 前置知识
- 栅格地图概念
- 概率论基础
- 射线投射（Raycasting）

### 6.1 栅格地图原理

栅格地图将3D空间划分为规则的立方体（体素），每个体素存储占用概率。

**关键概念**：
- **占用概率**: 表示该体素被障碍物占用的概率
- **Log-odds表示**: 使用对数几率表示概率，便于计算

```cpp
// 概率与log-odds之间的转换
double logit(double p) {
    return log(p / (1 - p));
}
```

### 6.2 地图更新算法

在`grid_map.cpp`中实现的raycast算法：

```cpp
void GridMap::raycastProcess()
{
    // 遍历每个投影点
    for (int i = 0; i < md_.proj_points_cnt; ++i)
    {
        pt_w = md_.proj_points_[i];
        
        // 终点标记为占用
        vox_idx = setCacheOccupancy(pt_w, 1);
        
        // 射线投射：从相机中心到点的路径上的体素标记为空闲
        raycaster.setInput(pt_w / resolution, camera_pos / resolution);
        while (raycaster.step(ray_pt))
        {
            setCacheOccupancy(ray_pt * resolution, 0);
        }
    }
    
    // 更新占用概率
    while (!md_.cache_voxel_.empty())
    {
        // 使用贝叶斯更新规则
        // l(x|z) = l(x) + log(P(z|x)/P(z|~x))
        md_.occupancy_buffer_[idx] += log_odds_update;
    }
}
```

### 6.3 障碍物膨胀

为了安全飞行，需要对障碍物进行膨胀处理：

```cpp
void GridMap::clearAndInflateLocalMap()
{
    // 膨胀步数 = 膨胀半径 / 分辨率
    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    
    // 对每个占用的体素进行膨胀
    for (int x = min_x; x <= max_x; ++x)
        for (int y = min_y; y <= max_y; ++y)
            for (int z = min_z; z <= max_z; ++z)
            {
                if (md_.occupancy_buffer_[idx] > threshold)
                {
                    // 膨胀周围的体素
                    inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
                }
            }
}
```

### 6.4 重要参数说明

| 参数名 | 含义 | 推荐值 |
|--------|------|--------|
| `resolution` | 栅格分辨率(米) | 0.1 |
| `obstacles_inflation` | 膨胀半径(米) | 0.2 |
| `p_hit` | 观测到障碍物时的更新概率 | 0.70 |
| `p_miss` | 观测到空闲时的更新概率 | 0.35 |

---

## 第七章：路径搜索(A*算法)

### 前置知识
- 图搜索算法基础
- 启发式搜索
- 数据结构（优先队列）

### 7.1 A*算法原理

A*算法是一种启发式图搜索算法，用于找到从起点到终点的最短路径。

**代价函数**：
$$f(n) = g(n) + h(n)$$

其中：
- $g(n)$: 从起点到当前节点的实际代价
- $h(n)$: 从当前节点到终点的启发式估计

### 7.2 代码实现

在`dyn_a_star.cpp`中的实现：

```cpp
bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    // 初始化起点
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    openSet_.push(startPtr);
    
    while (!openSet_.empty())
    {
        // 取出f值最小的节点
        current = openSet_.top();
        openSet_.pop();
        
        // 到达终点
        if (current->index == endPtr->index)
        {
            gridPath_ = retrievePath(current);
            return true;
        }
        
        // 扩展相邻节点
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    // 计算邻居节点
                    neighborIdx = current->index + Vector3i(dx, dy, dz);
                    
                    // 检查是否被占用
                    if (checkOccupancy(Index2Coord(neighborIdx)))
                        continue;
                    
                    // 更新代价
                    tentative_gScore = current->gScore + sqrt(dx*dx + dy*dy + dz*dz);
                    
                    if (tentative_gScore < neighborPtr->gScore)
                    {
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr);
                    }
                }
    }
    return false;
}
```

### 7.3 启发式函数

项目中实现了三种启发式函数：

```cpp
// 1. 对角距离启发式
double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));
    
    int diag = min(min(dx, dy), dz);
    // 计算对角线距离 + 剩余直线距离
    return sqrt(3.0) * diag + remaining_distance;
}

// 2. 曼哈顿距离
double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    return abs(dx) + abs(dy) + abs(dz);
}

// 3. 欧氏距离
double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}
```

---

## 第八章：B样条轨迹优化

### 前置知识
- 样条曲线基础
- 数值优化方法
- 梯度下降算法
- Ceres Solver基础

### 8.1 B样条曲线基础

B样条曲线是由控制点定义的参数曲线，具有以下优点：
- 局部控制性：移动一个控制点只影响局部曲线
- 凸包性质：曲线在控制点的凸包内
- 连续性：可以保证速度、加速度的连续性

**三次B样条位置公式**：
$$\mathbf{p}(t) = \frac{1}{6}[(1-t)^3\mathbf{P}_{i-1} + (3t^3-6t^2+4)\mathbf{P}_i + (-3t^3+3t^2+3t+1)\mathbf{P}_{i+1} + t^3\mathbf{P}_{i+2}]$$

### 8.2 优化问题建模

在`bspline_optimizer.cpp`中，轨迹优化被建模为：

$$\min_{\mathbf{Q}} \lambda_1 J_{smooth} + \lambda_2 J_{collision} + \lambda_3 J_{feasibility} + \lambda_4 J_{fitness}$$

其中：
- $J_{smooth}$: 平滑性代价（最小化jerk）
- $J_{collision}$: 碰撞代价
- $J_{feasibility}$: 动力学可行性代价
- $J_{fitness}$: 与参考轨迹的拟合代价

### 8.3 平滑性代价

```cpp
void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, 
    double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    
    // 计算jerk
    for (int i = 0; i < q.cols() - 3; i++)
    {
        // jerk = P_{i+3} - 3*P_{i+2} + 3*P_{i+1} - P_i
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        
        // 代价 = jerk的范数平方
        cost += jerk.squaredNorm();
        
        // 梯度
        temp_j = 2.0 * jerk;
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
    }
}
```

### 8.4 碰撞代价

```cpp
void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, 
    double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    
    for (int i = order_; i < end_idx; ++i)
    {
        // 计算控制点到障碍物的距离
        double dist = (cps_.points.col(i) - base_point).dot(direction);
        double dist_err = clearance - dist;
        
        if (dist_err > 0)  // 太近
        {
            // 惩罚代价
            cost += pow(dist_err, 3);
            // 梯度指向远离障碍物的方向
            gradient.col(i) += -3.0 * dist_err * dist_err * direction;
        }
    }
}
```

### 8.5 动力学可行性代价

```cpp
void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, 
    double &cost, Eigen::MatrixXd &gradient)
{
    // 速度可行性
    for (int i = 0; i < q.cols() - 1; i++)
    {
        Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
        
        if (vi.norm() > max_vel_)
        {
            // 惩罚超速
            cost += pow(vi.norm() - max_vel_, 2);
        }
    }
    
    // 加速度可行性
    for (int i = 0; i < q.cols() - 2; i++)
    {
        Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) / (ts * ts);
        
        if (ai.norm() > max_acc_)
        {
            // 惩罚过大加速度
            cost += pow(ai.norm() - max_acc_, 2);
        }
    }
}
```

### 8.6 L-BFGS优化

使用L-BFGS算法求解优化问题：

```cpp
bool BsplineOptimizer::rebound_optimize(double &final_cost)
{
    // 配置L-BFGS参数
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    
    // 执行优化
    int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, 
        BsplineOptimizer::costFunctionRebound, NULL, 
        BsplineOptimizer::earlyExit, this, &lbfgs_params);
    
    return (result == lbfgs::LBFGS_CONVERGENCE);
}
```

---

## 第九章：Ego-Planner规划框架

### 前置知识
- 有限状态机(FSM)
- 实时系统概念
- 前面章节的内容

### 9.1 系统架构

Ego-Planner采用分层架构：

```
┌─────────────────────────────────────────┐
│            EGOReplanFSM                  │
│          (有限状态机管理)                 │
├─────────────────────────────────────────┤
│           EGOPlannerManager              │
│          (规划算法封装)                   │
├──────────────────┬──────────────────────┤
│    GridMap       │   BsplineOptimizer   │
│   (栅格地图)      │   (轨迹优化)          │
├──────────────────┼──────────────────────┤
│    AStar         │   UniformBspline     │
│   (路径搜索)      │   (B样条曲线)         │
└──────────────────┴──────────────────────┘
```

### 9.2 有限状态机(FSM)

在`ego_replan_fsm.cpp`中定义了规划器的状态机：

```cpp
enum FSM_EXEC_STATE
{
    INIT,           // 初始化状态
    WAIT_TARGET,    // 等待目标点
    GEN_NEW_TRAJ,   // 生成新轨迹
    REPLAN_TRAJ,    // 重规划轨迹
    EXEC_TRAJ,      // 执行轨迹
    EMERGENCY_STOP, // 紧急停止
    SEQUENTIAL_START // 顺序启动（多机协同）
};
```

**状态转换图**：

```
INIT ──────► WAIT_TARGET
                 │
                 ▼
         SEQUENTIAL_START
                 │
                 ▼
           GEN_NEW_TRAJ ◄───────┐
                 │               │
                 ▼               │
            EXEC_TRAJ ──► REPLAN_TRAJ
                 │
                 ▼
           EMERGENCY_STOP
```

### 9.3 核心回调函数

```cpp
void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
    switch (exec_state_)
    {
    case INIT:
        // 等待里程计数据
        if (!have_odom_) return;
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
        
    case WAIT_TARGET:
        // 等待目标点
        if (!have_target_) return;
        changeFSMExecState(SEQUENTIAL_START, "FSM");
        break;
        
    case GEN_NEW_TRAJ:
        // 生成新轨迹
        if (planFromGlobalTraj(10))
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        break;
        
    case EXEC_TRAJ:
        // 执行轨迹，检查是否需要重规划
        if (t_cur > replan_thresh_)
        {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        break;
        
    case REPLAN_TRAJ:
        // 从当前状态重规划
        if (planFromCurrentTraj())
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        break;
    }
}
```

### 9.4 安全检查机制

```cpp
void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
{
    // 检查深度数据是否超时
    if (map->getOdomDepthTimeout())
    {
        ROS_ERROR("Depth Lost! EMERGENCY_STOP");
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }
    
    // 检查轨迹是否会碰撞
    for (double t = t_cur; t < duration; t += time_step)
    {
        if (map->getInflateOccupancy(traj.evaluate(t)))
        {
            // 尝试重规划
            if (planFromCurrentTraj())
            {
                changeFSMExecState(EXEC_TRAJ, "SAFETY");
            }
            else
            {
                // 紧急停止
                changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
        }
    }
}
```

### 9.5 关键参数配置

在`single_run_in_exp.launch`中：

```xml
<!-- 地图参数 -->
<param name="grid_map/resolution" value="0.1"/>
<param name="grid_map/map_size_x" value="40.0"/>
<param name="grid_map/map_size_y" value="40.0"/>
<param name="grid_map/map_size_z" value="5.0"/>

<!-- 规划参数 -->
<param name="manager/max_vel" value="2.0"/>
<param name="manager/max_acc" value="6.0"/>
<param name="fsm/planning_horizon" value="7.5"/>

<!-- 优化参数 -->
<param name="optimization/lambda_smooth" value="1.0"/>
<param name="optimization/lambda_collision" value="0.5"/>
<param name="optimization/dist0" value="0.5"/>
```

---

## 第十章：飞行控制模块(PX4Ctrl)

### 前置知识
- 四旋翼动力学
- PID控制
- PX4飞控基础

### 10.1 控制架构

PX4Ctrl是连接规划器和飞控的桥梁，实现从轨迹命令到姿态/推力的转换。

```
┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│  Ego-Planner  │────▶│   PX4Ctrl     │────▶│   PX4飞控    │
│  (位置/速度)   │     │  (姿态/推力)   │     │  (电机控制)   │
└───────────────┘     └───────────────┘     └───────────────┘
```

### 10.2 状态机

PX4Ctrl内部也有状态机管理：

```cpp
// PX4CtrlFSM.h
enum State_t
{
    MANUAL_CTRL,    // 手动控制（遥控器）
    AUTO_HOVER,     // 自动悬停
    CMD_CTRL,       // 指令控制（接收规划器命令）
    AUTO_TAKEOFF,   // 自动起飞
    AUTO_LAND       // 自动降落
};
```

**状态转换**：

```
                        MANUAL_CTRL
                        /    |    \
                       /     |     \
              AUTO_TAKEOFF   |   (遥控器拨杆)
                    \        |
                     \       |
                      AUTO_HOVER ◄────── AUTO_LAND
                          |
                          |
                       CMD_CTRL
```

### 10.3 位置控制算法

在`controller.cpp`中实现的位置控制器：

```cpp
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
    // 计算期望加速度（PD控制）
    // a_des = a_ff + Kp*(p_des - p) + Kv*(v_des - v) + g
    Eigen::Vector3d des_acc;
    des_acc = des.a 
            + Kv.asDiagonal() * (des.v - odom.v) 
            + Kp.asDiagonal() * (des.p - odom.p);
    des_acc += Eigen::Vector3d(0, 0, param_.gra);  // 补偿重力
    
    // 计算推力
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
    
    // 计算期望姿态
    // 从期望加速度方向计算roll和pitch
    double yaw_odom = fromQuaternion2yaw(odom.q);
    roll = (des_acc(0) * sin(yaw_odom) - des_acc(1) * cos(yaw_odom)) / g;
    pitch = (des_acc(0) * cos(yaw_odom) + des_acc(1) * sin(yaw_odom)) / g;
    
    // 构造四元数
    u.q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    
    return debug_msg_;
}
```

### 10.4 推力模型估计

飞控需要的是油门百分比（0-1），而控制器输出的是加速度。需要在线估计推力模型：

```cpp
bool LinearControl::estimateThrustModel(const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
    // 使用递归最小二乘法估计 thr2acc 比例
    // 模型: a_z = thr2acc * thrust
    
    double thr = t_t.second;  // 当前油门值
    
    // RLS更新
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    
    return true;
}
```

### 10.5 关键参数配置

在`ctrl_param_fpv.yaml`中：

```yaml
# 质量
mass: 1.2

# 悬停油门百分比
hover_percent: 0.3

# 增益参数
gain:
    Kp0: 5.0
    Kp1: 5.0
    Kp2: 6.0
    Kv0: 3.0
    Kv1: 3.0
    Kv2: 4.0

# 遥控器通道反向
rc_reverse:
    roll: false
    pitch: false
    yaw: false
    throttle: false
```

---

## 第十一章：仿真实验

### 前置知识
- ROS话题与节点通信
- Rviz可视化工具
- 前面章节的内容

### 11.1 仿真环境启动

```bash
# 编译项目
cd Fast-Drone-250
catkin_make
source devel/setup.bash

# 启动仿真
roslaunch ego_planner single_run_in_sim.launch
```

### 11.2 仿真环境架构

```
┌─────────────────────────────────────────────────┐
│                  仿真环境                         │
├────────────────┬────────────────────────────────┤
│                │                                 │
│  map_generator │  so3_quadrotor_simulator       │
│  (生成随机地图)  │  (四旋翼动力学仿真)              │
│                │                                 │
├────────────────┼────────────────────────────────┤
│                │                                 │
│  local_sensing │  odom_visualization            │
│  (深度图仿真)   │  (里程计可视化)                  │
│                │                                 │
└────────────────┴────────────────────────────────┘
                        │
                        ▼
              ┌─────────────────┐
              │   Ego-Planner   │
              │   (轨迹规划)     │
              └─────────────────┘
```

### 11.3 使用Rviz进行交互

在Rviz中可以：
1. 按**G键**进入目标点选择模式
2. **左键点击**设置目标点
3. 观察无人机规划路径并飞行

### 11.4 调试工具

使用**PlotJuggler**实时查看数据：
```bash
rosrun plotjuggler plotjuggler
```

可监控的话题：
- `/odom_world`: 里程计
- `/planning/bspline`: 轨迹
- `/position_cmd`: 控制指令

---

## 第十二章：实机飞行

### 前置知识
- 所有前面章节的内容
- 遥控器操作技能
- 安全意识

### 12.1 飞行前检查清单

- [ ] 电池电量充足
- [ ] 螺旋桨安装正确且牢固
- [ ] 所有线缆连接正常
- [ ] 遥控器电量充足
- [ ] 飞行场地安全、无人

### 12.2 启动顺序

```bash
# Terminal 1: 启动realsense和mavros
sh shfiles/rspx4.sh

# Terminal 2: 检查VINS输出
rostopic echo /vins_fusion/imu_propagate

# Terminal 3: 启动控制器
roslaunch px4ctrl run_ctrl.launch

# Terminal 4: 起飞
sh shfiles/takeoff.sh

# Terminal 5: 启动规划器
roslaunch ego_planner single_run_in_exp.launch
```

### 12.3 遥控器模式切换

```
5通道：悬停模式开关
    - 内侧：进入OFFBOARD模式
    - 外侧：返回手动模式

6通道：指令模式开关
    - 下侧：接受规划器指令
    - 上侧：退出指令模式，进入悬停

7通道：紧急停桨（最后手段）
```

### 12.4 紧急情况处理

| 情况 | 处理方法 |
|------|----------|
| VINS定位正常但撞障碍物 | 拨6通道退出指令模式 |
| VINS定位飘移 | 拨5通道返回手动模式 |
| 撞障碍物但还在空中 | 先试6通道，不行就5通道 |
| 已经炸机 | 立即拨5通道上锁 |
| 飞向危险区域 | 7通道紧急停桨 |

### 12.5 常见问题排查

**Q: 无人机悬停不稳定**
- 检查`hover_percent`参数是否正确
- 检查VINS外参是否标定准确
- 降低飞行速度

**Q: 规划器生成碰撞轨迹**
- 增大`obstacles_inflation`参数
- 检查深度相机是否正常工作
- 增大`resolution`使地图更精确

**Q: 起飞后无法控制**
- 检查遥控器通道是否反向
- 检查`rc_reverse`参数设置

---

## 附录

### A. 常用命令

```bash
# 查看话题
rostopic list
rostopic echo /topic_name
rostopic hz /topic_name

# 查看节点
rosnode list
rosnode info /node_name

# 查看参数
rosparam list
rosparam get /param_name
```

### B. 参考资料

1. [ROS官方文档](http://wiki.ros.org/)
2. [PX4开发者文档](https://docs.px4.io/)
3. [Ego-Planner论文](https://arxiv.org/abs/2008.08835)
4. [VINS-Fusion论文](https://arxiv.org/abs/1901.03642)
5. [深蓝学院运动规划课程](https://www.shenlanxueyuan.com/course/385)

### C. 代码仓库

- GitHub: https://github.com/ZJU-FAST-Lab/Fast-Drone-250
- VINS-Fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion

---

*本教程由浙江大学FAST-Lab实验室提供技术支持，仅供学习使用，严禁商用。*
