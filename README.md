# M1 NX 全栈 — FAST-LIO + WebUI + 依赖

Genesis M1 四足机器人 Jetson Orin NX 完整部署包。
包含 GPU 加速 FAST-LIO2、WebUI 控制面板、体素降采样、nvblox 建图、云端代理。

## 目录结构

```
m1-nx-stack/
├── fastlio_src/       # FAST-LIO GPU 源码 (Airy96 适配)
├── voxel_filter/      # CUDA 体素降采样 (m1_voxel_filter)
├── nvblox_bridge/     # nvblox TSDF 建图桥接
├── webui/             # WebUI 控制面板 + 云端代理
├── scripts/           # 启动脚本、配置文件
├── config/            # Nav2 参数
└── README.md
```

## 架构

```
rslidar_sdk (M1 本体)
  ├─ /front_lidar       (86K pts, 10Hz)
  └─ /front_lidar/imu   (Y-up, 100Hz)
         │ Zenoh DDS domain 66
         ▼
m1_voxel_filter (CUDA, 0.05m)
  └─ /front_lidar/filtered  (16K pts)
         │
         ▼
fastlio_mapping (GPU)
  │  IMU rotation R_x(90°) 内嵌 C++
  │  Static TF 内嵌 C++
  ├─ /Odometry            (odom→imu_link)
  ├─ /cloud_registered
  └─ /Laser_map
         │
         ▼
m1_nvblox_br (TSDF 建图)
```

## 一键部署

```bash
# 1. Clone
git clone https://github.com/xki2ng/m1-nx-stack.git ~/m1-nx-stack

# 2. 安装 WebUI 自启
bash ~/m1-nx-stack/webui/install_service.sh

# 3. 编译
cd ~/m1_ws
# 需要先链接源码到工作空间
ln -s ~/m1-nx-stack/fastlio_src ~/m1_ws/src/fast_lio
ln -s ~/m1-nx-stack/voxel_filter ~/m1_ws/src/m1_voxel_filter
ln -s ~/m1-nx-stack/nvblox_bridge ~/m1_ws/src/nvblox_fastlio_bridge

# 编译 FAST-LIO (GPU)
colcon build --packages-select fast_lio \
  --cmake-args -DFASTLIO_USE_CUDA=ON \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=87

# 编译 voxel + nvblox
colcon build --packages-select m1_voxel_filter nvblox_fastlio_bridge
```

## WebUI 功能

- **控制面板**: 进程状态实时监控，独立 START/STOP 按钮
- **地图管理**: ros2 bag 录制保存，自定义命名，列表/删除
- **轨迹管理**: GPS 路径记录、存档
- **云端访问**: SSH 隧道 → cloud_proxy (8443)
- **多客户端**: Ubuntu/Windows 浏览器均可

## FAST-LIO 关键修复

| 修复项 | 说明 |
|--------|------|
| Airy96 Point 布局 | `float x,y,z` (非 PCL_ADD_POINT4D) |
| Ring buffer | 256 (Airy96 96线) |
| cube_side_length | 1000 (200 导致 FOV 误删) |
| IMU 旋转 | R_x(90°) 内嵌 C++ |
| TF 发布 | 内嵌 C++ (无外部脚本) |

## 验证配置

`scripts/m1_airy96_rot.yaml`:
- lid: `/front_lidar/filtered`
- imu: `/front_lidar/imu`
- body_frame: `imu_link`
- extrinsic_R: R_y(90°)
- Gravity: Z ≈ -9.81 ✅
- Odometry: ~10 Hz ✅

## 文件清单

```
webui/
  index.html  control_server.py  cloud_proxy.py  save_map.py
  tunnel_guard.sh  install_service.sh  m1-webui.service  vendor/

scripts/
  m1_airy96_rot.yaml  m1_full_stack.sh  m1_nav2.launch.py
  m1_nav2_fastlio.yaml  activate.sh  start_nav2.sh
  m1-fastlio.service  zenoh_nx_client.json5
  costmap_zenoh_relay.py  m1_vel_sign.py

config/
  nav2_params.yaml  m1_bt_nav.xml

fastlio_src/    → FAST-LIO GPU 源码
voxel_filter/   → CUDA 体素降采样
nvblox_bridge/  → nvblox TSDF 桥接
```
