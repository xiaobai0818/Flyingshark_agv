# Flyingshark R10

香橙派 5 Max 上的两轮差速小车。底盘是塔克 XTARK R10，控制板 OPENCTR B20，激光雷达 LD06，深度相机 Astra Pro。系统是 Ubuntu 20.04、ROS Noetic。工作空间在 `/home/orangepi/Flyingshark_agv`。

坐标原点是 `base_footprint`，在两轮接地点的中心，贴地。x 朝前，y 朝左，z 朝上，单位米。下面的安装位置都相对这个原点。

## 目录

| 目录 | 内容 |
| --- | --- |
| `driver` | 底盘、LD06、Astra Pro |
| `slam` | Cartographer 建图 |
| `navigation` | move_base，局部规划默认 TEB |
| `utils` | 车体模型、EKF、总启动 |
| `ui` | 网页控制台 |
| `map` | 保存的地图 |
| `shfiles` | 启动脚本 |

改安装位置时，同时改 `utils/src/flyingshark_description/urdf/r10.urdf.xacro` 和 `utils/src/flyingshark_bringup/launch/bringup.launch`。外壳半径还要和 `navigation` 里的 `robot_radius` 一致。

## 使用

网页控制台开机自启，服务名 `flyingshark-webui`，端口 8080。浏览器打开香橙派的地址即可。

先点「初始化」。底盘、雷达、相机和麦克风会打开，画面、摇杆、实时点云和左右响度条就可以用。建图或导航再点对应按钮。关掉建图或导航时，传感器继续开着。再点一次「初始化」会把传感器也关掉。

已知地图导航才能重定位。重定位时车会慢转一圈，用雷达对齐地图。设置里的速度会保存在 `map/webui_settings.json`。地图保存在 `map/`，默认是 `r10.yaml`。

终端里也可以：

```bash
bash shfiles/init.sh          # 底盘、雷达、相机
bash shfiles/mapping.sh       # 建图
bash shfiles/save_map.sh      # 存到 map/r10.*
bash shfiles/blank_nav.sh     # 无地图导航
bash shfiles/navigation.sh    # 已存地图，AMCL
bash shfiles/webui.sh         # 手动开网页
```

`mapping.sh`、`blank_nav.sh`、`navigation.sh` 不会重复启动已经打开的驱动。

航向由 EKF 用陀螺仪 Z 轴融进 `/odom`，线速度用轮式编码器。起步时让车停稳约 2 秒，用来估陀螺仪零偏。建图用 Cartographer，激光加 IMU，回环是开的。深度相机只进代价地图做避障，不写进全局地图。已知地图导航用 AMCL，地图不再边走边改。

## 外形

外壳按圆处理，半径 0.108 m。车顶离地约 0.20 m。模型里的圆柱只表示底盘一圈，高 40 mm，不是整车高度。

| 项目 | 数值 |
| --- | --- |
| 外壳半径 | 0.108 m |
| 车顶离地 | 约 0.20 m |
| 轮胎直径 | 74 mm |
| 轮胎宽度 | 30 mm |
| 两轮中心距 | 164 mm |
| 轮心离地 | 37 mm |

左轮 y = +82 mm，右轮 y = −82 mm。

## 传感器

### LD06

装在车心前方，扫描面离地 105 mm，朝向与车头一致。后方约 120° 到 240° 被铝柱挡住，驱动里把这段裁掉。有效距离 0.08 m 到 8 m。

| 项目 | 数值 |
| --- | --- |
| x / y / z | 0.060 / 0 / 0.105 m |
| 坐标系 | `laser` |
| 接口 | 40 pin UART6，`/dev/ldlidar`（`ttyS6`），230400，3.3 V |
| 接线 | 板子 13 脚 TX 接雷达 RX，11 脚 RX 接雷达 TX，共地 |

`/scan_raw` 是原始扫描，`/scan` 是裁过角度之后的。

### Astra Pro

一根 USB 线，里面有集线器。彩色相机是 `2bc5:0501`，深度芯片是 `2bc5:0403`。必须插 USB 3 口，两路才会都出来。插板载 USB 2 口时，经常只有彩色、没有深度。

相机在车头前方 55 mm、离地 160 mm，朝前，没有俯仰。图像 640×480，15 Hz。避障只用离地 0.12 m 到 0.65 m、相机真正看到的表面。

| 项目 | 数值 |
| --- | --- |
| x / y / z | 0.055 / 0 / 0.160 m |
| 坐标系 | `camera_link` |
| 外形（模型） | 90 × 30 × 25 mm |

话题：`/camera/color/image_raw`，`/camera/depth/points`，`/camera/depth_registered/points`，`/camera/depth_obstacles`。

麦克风在深度芯片上，双声道，16000 Hz，声卡 `hw:Pro,0`。左右声道是 `/camera/audio/left` 和 `/camera/audio/right`。初始化之后，网页上有对应的响度条。它不是麦克风阵列，不能判断声源方向。

### IMU

在 OPENCTR B20 上，坐标系 `base_imu_link`。相对 `base_link` 高 20 mm，离地 57 mm，没有额外旋转。话题 `/imu`。Cartographer 的跟踪坐标系就是它。

### 底盘串口

OPENCTR B20 接 40 pin UART2，`/dev/openctr`（`ttyS2`），230400，3.3 V。板子 8 脚 TX 接控制板 RX，10 脚 RX 接控制板 TX，共地。轮式里程计是 `/wheel/odom`，融合后是 `/odom`。
