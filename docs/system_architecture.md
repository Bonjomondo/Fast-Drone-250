# Fast-Drone-250 系统架构文档

## System Architecture Documentation

---

## 概述 (Overview)

Fast-Drone-250是一个完整的自主四旋翼无人机系统，由浙江大学FAST-Lab实验室开发。本文档详细描述了系统的各个模块、功能、优势及局限性。

---

## 系统整体架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Fast-Drone-250 系统架构                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌───────────────────────────────────────────────────────────────────┐     │
│   │                        感知层 (Perception Layer)                    │     │
│   │  ┌─────────────────┐    ┌─────────────────┐    ┌──────────────┐   │     │
│   │  │  RealSense D435i│    │  VINS-Fusion    │    │  Grid Map    │   │     │
│   │  │  (深度相机)      │───▶│  (视觉惯性里程计) │───▶│  (栅格地图)   │   │     │
│   │  └─────────────────┘    └─────────────────┘    └──────────────┘   │     │
│   └───────────────────────────────────────────────────────────────────┘     │
│                                    │                                         │
│                                    ▼                                         │
│   ┌───────────────────────────────────────────────────────────────────┐     │
│   │                        规划层 (Planning Layer)                      │     │
│   │  ┌─────────────────┐    ┌─────────────────┐    ┌──────────────┐   │     │
│   │  │  A* Path Search │───▶│  B-spline Opt   │───▶│  FSM Manager │   │     │
│   │  │  (路径搜索)      │    │  (轨迹优化)      │    │  (状态管理)   │   │     │
│   │  └─────────────────┘    └─────────────────┘    └──────────────┘   │     │
│   └───────────────────────────────────────────────────────────────────┘     │
│                                    │                                         │
│                                    ▼                                         │
│   ┌───────────────────────────────────────────────────────────────────┐     │
│   │                        控制层 (Control Layer)                       │     │
│   │  ┌─────────────────┐    ┌─────────────────┐    ┌──────────────┐   │     │
│   │  │  Traj Server    │───▶│  PX4Ctrl        │───▶│  PX4 FCU     │   │     │
│   │  │  (轨迹服务器)    │    │  (位置控制器)    │    │  (飞控)      │   │     │
│   │  └─────────────────┘    └─────────────────┘    └──────────────┘   │     │
│   └───────────────────────────────────────────────────────────────────┘     │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 模块详细说明

### 1. RealSense D435i (深度相机模块)

**位置**: `src/realflight_modules/realsense-ros/`

#### 功能
- 提供深度图像数据用于障碍物检测
- 提供双目红外图像用于VINS特征点提取
- 以30-90Hz的频率发布深度和图像数据

#### 输出话题
| 话题名称 | 消息类型 | 频率 | 描述 |
|---------|----------|------|------|
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 30Hz | 深度图像 |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 30Hz | 左红外图像 |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 30Hz | 右红外图像 |
| `/camera/infra1/camera_info` | `sensor_msgs/CameraInfo` | 30Hz | 相机内参 |

#### 优势
- 工业级设备，稳定可靠
- 室内环境深度测量精度高
- USB3.0接口，易于安装

#### 局限性
- 户外强光环境下性能下降
- 最大测量距离约10米
- 结构光红外投影会干扰VINS特征点检测（需要遮挡）

---

### 2. VINS-Fusion (视觉惯性里程计模块)

**位置**: `src/realflight_modules/VINS-Fusion/`

#### 功能
- 融合双目视觉和IMU数据进行状态估计
- 实时输出无人机位姿和速度
- 在线标定相机-IMU外参
- 可选的回环检测功能

#### 系统流程
```
┌────────────┐     ┌────────────┐     ┌────────────┐
│  Feature   │     │    IMU     │     │  Sliding   │
│  Tracking  │────▶│  Preinte-  │────▶│  Window    │
│            │     │  gration   │     │  Optimizer │
└────────────┘     └────────────┘     └────────────┘
      │                  │                   │
      │                  │                   ▼
      │                  │            ┌────────────┐
      └──────────────────┴───────────▶│   Pose &   │
                                      │  Velocity  │
                                      └────────────┘
```

#### 输出话题
| 话题名称 | 消息类型 | 频率 | 描述 |
|---------|----------|------|------|
| `/vins_fusion/imu_propagate` | `nav_msgs/Odometry` | 200Hz | IMU预积分位姿 |
| `/vins_fusion/odometry` | `nav_msgs/Odometry` | 30Hz | 优化后位姿 |
| `/vins_fusion/extrinsic` | `nav_msgs/Odometry` | 30Hz | 外参矩阵 |

#### 优势
- 不依赖GPS，适合室内环境
- 结合视觉和IMU，鲁棒性好
- 开源且社区活跃

#### 局限性
- 对光照变化敏感
- 需要足够的视觉特征点
- CPU计算量较大
- 长距离飞行会有累积误差

---

### 3. Grid Map (栅格地图模块)

**位置**: `src/planner/plan_env/`

#### 功能
- 维护3D栅格占用地图
- 使用深度图像更新地图
- 对障碍物进行膨胀处理
- 提供碰撞检测接口

#### 核心算法
```
深度图像 ──▶ 3D点投影 ──▶ Raycast ──▶ 概率更新 ──▶ 膨胀
```

#### 关键接口
```cpp
class GridMap {
public:
    // 初始化地图
    void initMap(ros::NodeHandle &nh);
    
    // 检查点是否被占用
    bool getInflateOccupancy(Eigen::Vector3d pos);
    
    // 获取地图分辨率
    double getResolution();
    
    // 检查深度数据是否超时
    bool getOdomDepthTimeout();
};
```

#### 配置参数
| 参数 | 默认值 | 描述 |
|------|--------|------|
| `resolution` | 0.1m | 栅格分辨率 |
| `map_size_x/y/z` | 40/40/5m | 地图尺寸 |
| `obstacles_inflation` | 0.2m | 障碍物膨胀半径 |
| `p_hit` | 0.70 | 观测到占用时的更新概率 |
| `p_miss` | 0.35 | 观测到空闲时的更新概率 |

#### 优势
- 实时更新，适应动态环境
- 概率表示，对噪声鲁棒
- 高效的内存使用

#### 局限性
- 内存占用随地图尺寸立方增长
- 无法处理动态障碍物的预测
- 深度相机盲区无法检测

---

### 4. A* Path Search (路径搜索模块)

**位置**: `src/planner/path_searching/`

#### 功能
- 在栅格地图中搜索无碰撞路径
- 为轨迹优化提供初始路径
- 使用启发式函数加速搜索

#### 算法实现
```cpp
bool AStar::AstarSearch(Vector3d start_pt, Vector3d end_pt) {
    // 1. 初始化起点
    // 2. 循环扩展节点
    //    - 取出f值最小的节点
    //    - 检查是否到达终点
    //    - 扩展相邻节点
    //    - 更新代价
    // 3. 回溯路径
}
```

#### 时间复杂度
- 最坏情况: O(V + E log V)
- V: 节点数
- E: 边数

#### 优势
- 保证找到最短路径
- 计算速度快
- 实现简单可靠

#### 局限性
- 只提供离散路径点
- 不考虑动力学约束
- 长距离搜索可能较慢

---

### 5. B-spline Optimization (B样条优化模块)

**位置**: `src/planner/bspline_opt/`

#### 功能
- 将离散路径转换为平滑轨迹
- 优化轨迹满足动力学约束
- 避免与障碍物碰撞
- 最小化能量消耗

#### 优化目标
$$\min_{\mathbf{Q}} J = \lambda_1 J_{smooth} + \lambda_2 J_{collision} + \lambda_3 J_{feasibility} + \lambda_4 J_{fitness}$$

#### 各项代价函数

| 代价项 | 公式 | 作用 |
|--------|------|------|
| $J_{smooth}$ | $\sum \|\mathbf{j}_i\|^2$ | 最小化jerk |
| $J_{collision}$ | $\sum \max(0, d_0 - d_i)^3$ | 避免碰撞 |
| $J_{feasibility}$ | $\sum \max(0, \|v_i\| - v_{max})^2$ | 速度约束 |
| $J_{fitness}$ | $\sum \|p_i - p_{ref}\|^2$ | 跟踪参考 |

#### 核心类
```cpp
class BsplineOptimizer {
public:
    // 设置环境
    void setEnvironment(const GridMap::Ptr &map);
    
    // 设置控制点
    void setControlPoints(const Eigen::MatrixXd &points);
    
    // 优化
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts);
    
    // 获取不同轨迹
    std::vector<ControlPoints> distinctiveTrajs(vector<pair<int, int>> segments);
};
```

#### 优势
- 生成动力学可行轨迹
- 保证安全间距
- 支持多轨迹选择

#### 局限性
- 优化可能陷入局部最优
- 参数调整需要经验
- 计算时间随轨迹长度增加

---

### 6. EGOReplanFSM (Ego-Planner 状态机模块)

**位置**: `src/planner/plan_manage/`

#### 功能
- 管理规划器的运行状态
- 处理用户输入和传感器数据
- 协调各模块工作
- 处理异常情况

#### 状态定义
```cpp
enum FSM_EXEC_STATE {
    INIT,            // 初始化
    WAIT_TARGET,     // 等待目标
    GEN_NEW_TRAJ,    // 生成新轨迹
    REPLAN_TRAJ,     // 重规划
    EXEC_TRAJ,       // 执行轨迹
    EMERGENCY_STOP,  // 紧急停止
    SEQUENTIAL_START // 顺序启动
};
```

#### 状态转换规则
```
INIT: have_odom -> WAIT_TARGET
WAIT_TARGET: have_target -> SEQUENTIAL_START
SEQUENTIAL_START: plan_success -> EXEC_TRAJ
EXEC_TRAJ: need_replan -> REPLAN_TRAJ
EXEC_TRAJ: collision_detected -> EMERGENCY_STOP
Any_State: !rc_is_hover -> MANUAL (退出程序控制)
```

#### 定时器
| 定时器 | 周期 | 功能 |
|--------|------|------|
| `exec_timer_` | 10ms | 执行状态机 |
| `safety_timer_` | 50ms | 安全检查 |

#### 优势
- 清晰的状态管理
- 完善的异常处理
- 支持多机协同

#### 局限性
- 状态切换有延迟
- 需要稳定的数据输入
- 复杂场景可能需要额外逻辑

---

### 7. Traj Server (轨迹服务器模块)

**位置**: `src/planner/plan_manage/src/traj_server.cpp`

#### 功能
- 接收B样条轨迹
- 按固定频率采样轨迹点
- 计算期望偏航角
- 发布位置控制指令

#### 工作流程
```
B-spline轨迹 ──▶ 时间采样 ──▶ 位置/速度/加速度 ──▶ Position Command
                    │
                    ▼
              偏航角计算 ──▶ yaw, yaw_dot
```

#### 偏航角控制策略
```cpp
// 偏航角朝向运动方向
Eigen::Vector3d dir = traj[0].evaluate(t + time_forward) - pos;
double yaw = atan2(dir(1), dir(0));

// 限制偏航角变化率
if (yaw_change > max_yaw_rate * dt) {
    yaw = last_yaw + max_yaw_rate * dt;
}
```

#### 输出话题
| 话题 | 消息类型 | 频率 |
|------|----------|------|
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | 100Hz |

#### 优势
- 高频率输出保证平滑控制
- 自动计算偏航角
- 轨迹结束后自动悬停

#### 局限性
- 偏航角策略相对简单
- 不处理yaw轨迹规划

---

### 8. PX4Ctrl (位置控制器模块)

**位置**: `src/realflight_modules/px4ctrl/`

#### 功能
- 接收位置/速度/加速度指令
- 计算期望姿态和推力
- 与PX4飞控通信
- 处理遥控器输入

#### 控制算法
```
期望加速度 = a_ff + Kp*(p_des - p) + Kv*(v_des - v) + g

期望推力 = |a_des| / thr2acc

期望姿态 = f(a_des, yaw_des)
```

#### 状态机
```cpp
enum State_t {
    MANUAL_CTRL,   // 遥控器控制
    AUTO_HOVER,    // 悬停
    CMD_CTRL,      // 指令控制
    AUTO_TAKEOFF,  // 自动起飞
    AUTO_LAND      // 自动降落
};
```

#### 安全机制
1. **遥控器优先**: 随时可通过遥控器接管
2. **超时保护**: 指令超时自动悬停
3. **速度检查**: 异常速度时拒绝切换
4. **推力估计**: 在线估计推力模型

#### 配置参数
| 参数 | 描述 |
|------|------|
| `mass` | 飞机质量(kg) |
| `hover_percent` | 悬停油门百分比 |
| `Kp0, Kp1, Kp2` | 位置增益 |
| `Kv0, Kv1, Kv2` | 速度增益 |

#### 优势
- 简单有效的控制算法
- 完善的安全机制
- 在线推力模型估计

#### 局限性
- 简化的姿态控制
- 不支持复杂机动动作
- 需要准确的质量参数

---

### 9. UAV Simulator (仿真模块)

**位置**: `src/uav_simulator/`

#### 组成部分

##### a. so3_quadrotor_simulator
- 四旋翼动力学仿真
- 接收姿态/推力指令
- 输出仿真里程计

##### b. local_sensing
- 仿真深度相机
- 根据地图生成深度图
- 支持多机仿真

##### c. map_generator
- 生成随机障碍物地图
- 可配置障碍物密度和尺寸

#### 仿真架构
```
┌───────────────┐     ┌───────────────┐
│ map_generator │────▶│ local_sensing │
└───────────────┘     └───────┬───────┘
                              │
                              ▼
┌───────────────┐     ┌───────────────┐
│ Ego-Planner   │◀───▶│ so3_quadrotor │
│               │     │  _simulator   │
└───────────────┘     └───────────────┘
```

#### 优势
- 无需硬件即可测试算法
- 安全的开发环境
- 可重复的测试条件

#### 局限性
- 与真实环境有差距
- 未模拟所有传感器噪声
- 无法完全代替实机测试

---

## 数据流图

### 实飞数据流
```
RealSense ─────┬────────────────────────────────────▶ Grid Map
               │                                          │
               ▼                                          │
           VINS-Fusion ──▶ Odom ──┬──────────────────────┤
               │                  │                       │
               ▼                  ▼                       ▼
             IMU ──────────▶ PX4Ctrl ◀─── Traj Server ◀── Ego-Planner
                                │
                                ▼
                           PX4 飞控
                                │
                                ▼
                             电机
```

### 话题连接关系
```
/camera/depth/image_rect_raw ──▶ [grid_map]
/camera/infra1/image_rect_raw ──▶ [vins_fusion]
/camera/infra2/image_rect_raw ──▶ [vins_fusion]
/mavros/imu/data_raw ──▶ [vins_fusion]
/vins_fusion/imu_propagate ──▶ [ego_planner, grid_map, px4ctrl]
/planning/bspline ──▶ [traj_server]
/position_cmd ──▶ [px4ctrl]
/mavros/setpoint_raw/attitude ◀── [px4ctrl]
```

---

## 性能指标

### 计算资源占用
| 模块 | CPU占用(单核) | 内存占用 |
|------|--------------|----------|
| VINS-Fusion | 50-80% | 500MB |
| Grid Map | 20-40% | 100-500MB |
| Ego-Planner | 30-50% | 200MB |
| PX4Ctrl | 5-10% | 50MB |

### 延迟性能
| 环节 | 延迟 |
|------|------|
| 深度图像获取 | 33ms (30Hz) |
| VINS状态估计 | 10-30ms |
| 轨迹规划 | 20-100ms |
| 控制指令生成 | 1ms |

### 飞行性能
| 指标 | 典型值 |
|------|--------|
| 最大速度 | 2.5 m/s |
| 最大加速度 | 6 m/s² |
| 定位精度 | 0.1-0.3 m |
| 续航时间 | 5分钟 |

---

## 扩展与改进方向

### 已知局限性及改进建议

1. **VINS累积误差**
   - 建议：增加回环检测
   - 建议：融合GPS/UWB等绝对定位

2. **动态障碍物**
   - 建议：增加运动目标检测
   - 建议：实现动态避障

3. **户外环境**
   - 建议：使用激光雷达替代深度相机
   - 建议：增加GPS融合

4. **续航时间**
   - 建议：使用更大容量电池
   - 建议：优化飞行轨迹以节省能量

5. **计算资源**
   - 建议：使用GPU加速VINS
   - 建议：使用嵌入式平台（如Jetson NX）

---

## 总结

Fast-Drone-250是一个功能完整、模块化设计的自主无人机系统。它展示了从感知、规划到控制的完整技术栈，适合作为研究和学习的平台。

**核心优势**：
- 完全开源
- 模块化设计
- 文档完善
- 社区支持

**适用场景**：
- 室内自主导航研究
- 运动规划算法验证
- 无人机课程教学
- 竞赛平台开发

---

*文档版本: 1.0*
*最后更新: 2024*
*作者: FAST-Lab, Zhejiang University*
