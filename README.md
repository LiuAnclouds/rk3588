# Mine Pedestrian Vision

![RK3588](https://img.shields.io/badge/SoC-RK3588-blue)
![Orange Pi 5 Pro](https://img.shields.io/badge/Board-Orange%20Pi%205%20Pro-orange)
![RKNN](https://img.shields.io/badge/Runtime-RKNN%20NPU-green)
![YOLOv8 Seg](https://img.shields.io/badge/Model-YOLOv8n--seg%20INT8-yellow)
![Hikrobot](https://img.shields.io/badge/Camera-Hikrobot%20GigE-lightgrey)

面向矿洞行人检测避障原型的边缘视觉工程。项目在 Orange Pi 5 Pro / RK3588 上接入海康威视 GigE 工业相机，使用 RKNN NPU 实时运行 YOLOv8 segmentation，输出行人框、分割 mask、脚底像素点、左右方向和近/中/远粗距离分区，并提供 MJPEG 实时可视化流。

支持 **USB转RS485行人检测数据传输**、**开机自启动**、**可配置IP与端口**的完整部署方案。

## Demo

| 近处重点目标 | 多目标矿洞场景 | 复杂背景场景 |
| --- | --- | --- |
| ![demo three](docs/images/mmexport1781891594276.jpg) | ![demo many](docs/images/mmexport1781891601272.jpg) | ![demo five](docs/images/mmexport1781891604099.jpg) |

## Features

- **实时行人检测与分割**：YOLOv8n-seg INT8 RKNN，RK3588 NPU 三核推理。
- **工程化可视化**：MJPEG 流直接叠加检测框、mask、方向线和目标卡片。
- **融合友好输出**：`web/live_targets.json` 输出 bbox、footpoint、bearing、range zone，可供后续雷达融合。
- **USB转RS485数据传输**：检测到人时自动通过串口发送结构化数据帧，支持模拟模式。
- **开机自启动**：systemd 服务，上电自动运行推理和 Web 服务。
- **CLI管理工具**：`mine_control` 命令行统一管理 IP、端口、串口、模型等参数。
- **可配置推流IP**：默认读取WiFi地址，支持手动设置，Web页面支持 `?ip=` 参数。

## Hardware

| Component | Value |
| --- | --- |
| Board | Orange Pi 5 Pro |
| SoC | Rockchip RK3588 |
| Camera | Hikrobot MV-CU013-A0GC GigE |
| Board Wi-Fi / SSH | `192.168.1.5` |
| Board Ethernet | `192.168.10.1/24` |
| Camera IP | `192.168.10.2/24` |
| RS485 | USB转RS485 (CH340/FT232等) |
| Project path | `/home/orangepi/moonxkj/mine_pedestrian` |

## Quick Start

```bash
cd /home/orangepi/moonxkj/mine_pedestrian

# 拉满 CPU/NPU/DDR 频率（可选，建议演示前执行）
echo orangepi | sudo -S ./scripts/performance_mode.sh

# 一键启动所有服务
./scripts/mine_control start
```

打开实时流页面（自动使用配置的IP）：

- Web页面: <http://192.168.1.5:8080/>
- MJPEG stream: <http://192.168.1.5:8090/stream>
- 指定IP访问: <http://192.168.1.5:8080/?ip=192.168.1.5&port=8090>

停止服务：

```bash
./scripts/mine_control stop
```

## CLI 管理工具 `mine_control`

安装到系统PATH后，可在任意目录使用：

```bash
sudo ln -sf /home/orangepi/moonxkj/mine_pedestrian/scripts/mine_control /usr/local/bin/mine_control
```

### 服务控制

```bash
mine_control status      # 查看全部运行状态
mine_control start       # 启动所有服务
mine_control stop        # 停止所有服务
mine_control restart     # 重启所有服务
```

### IP 管理

```bash
mine_control ip show           # 查看当前IP和WiFi IP
mine_control ip auto           # 自动检测WiFi IP并设置
mine_control ip set 192.168.1.100  # 手动设置推流IP
```

### 端口管理

```bash
mine_control port show                  # 查看当前端口
mine_control port set mjpeg 8091        # 修改MJPEG流端口
mine_control port set web 8081          # 修改Web页面端口
```

### 模型管理

```bash
mine_control model show        # 查看当前模型
mine_control model list        # 列出可用模型
mine_control model set /path/to/model.rknn   # 切换模型
```

### RS485 串口数据传输

```bash
mine_control serial show       # 查看串口配置
mine_control serial list       # 列出可用串口设备
mine_control serial set /dev/ttyUSB0 115200  # 设置串口设备和波特率
mine_control serial on         # 启用串口发送
mine_control serial start      # 启动RS485发送进程
mine_control serial stop       # 停止RS485发送
mine_control serial test       # 发送测试帧
mine_control serial off        # 禁用串口发送
```

### 查看配置

```bash
mine_control config            # 查看完整配置文件
```

## RS485 数据协议

检测到行人时，通过串口发送 NMEA 风格数据帧：

```
$PED,<frame_id>,<id>,<confidence>,<x1>,<y1>,<x2>,<y2>,<foot_x>,<foot_y>,<bearing>,<bearing_zone>,<range_zone>*<checksum>
```

示例：

```
$PED,134820,1,0.446,912,409,1277,1010,1094,1009,24.8,right,near*3A
```

无目标时发送空帧：

```
$PED,134821,0,0,,,,,,,*XX
```

校验和为 `$` 和 `*` 之间字符的 XOR 结果（十六进制大写）。

配置参数在 `config/mine_config.conf` 中：

```ini
SERIAL_DEV=/dev/ttyUSB0    # 串口设备路径
SERIAL_BAUD=115200          # 波特率
SERIAL_ENABLED=1            # 是否启用
```

如果没有硬件串口设备，程序自动进入**模拟模式**——只在日志中打印数据帧，不实际发送。

## 开机自启动

安装 systemd 服务：

```bash
cd /home/orangepi/moonxkj/mine_pedestrian
sudo ./scripts/setup_autostart.sh
```

脚本会自动安装并启用 `mine_pedestrian.service`，询问是否立即启动。

手动管理服务：

```bash
sudo systemctl start mine_pedestrian     # 启动
sudo systemctl stop mine_pedestrian      # 停止
sudo systemctl status mine_pedestrian    # 查看状态
sudo systemctl restart mine_pedestrian   # 重启
sudo systemctl disable mine_pedestrian   # 取消自启
```

服务日志：

```bash
tail -f /home/orangepi/moonxkj/mine_pedestrian/logs/service.log
```

## Output JSON

实时结果写入 `web/live_targets.json`，格式示例：

```json
{
  "frame_id": 89,
  "fps": 32.15,
  "infer_ms": 22.60,
  "targets": [
    {
      "id": 1,
      "type": "person",
      "confidence": 0.748,
      "bbox": [347, 495, 951, 1013],
      "footpoint_px": [664, 1012],
      "bearing_deg": 1.3,
      "bearing_zone": "front",
      "range_zone": "near",
      "mask_available": true
    }
  ]
}
```

## Scripts

| Script | Description |
| --- | --- |
| `scripts/mine_control` | **CLI管理工具** - 统一管理IP、端口、串口、模型、服务启停 |
| `scripts/run_live_web.sh` | 启动相机实时推理、MJPEG 流和 Web 页面 |
| `scripts/stop_live_web.sh` | 停止实时推理、Web 服务和 RS485 发送 |
| `scripts/mine_rs485.py` | RS485 行人检测数据串口发送程序 |
| `scripts/setup_autostart.sh` | 安装 systemd 开机自启动服务 |
| `scripts/mine_pedestrian.service` | systemd 服务单元文件 |
| `scripts/run_image.sh` | 对单张图片推理并保存可视化结果 |
| `scripts/bench_images.sh` | 批量跑 `test_images`，生成展示图 |
| `scripts/run_camera_once.sh` | 抓取一帧相机图并跑单图推理 |
| `scripts/run_camera_loop.sh` | 旧版循环抓图推理，调试用 |
| `scripts/run_smoke.sh` | 检查 RKNN 模型和 runtime 是否能加载 |
| `scripts/performance_mode.sh` | 设置 CPU/NPU/DDR 等 governor 为 performance |
| `scripts/setup_camera_net.sh` | 临时配置相机网口 IP |

## Build

```bash
cd /home/orangepi/moonxkj/mine_pedestrian
make all
```

单独编译实时程序：

```bash
make live
```

工程依赖：

- OpenCV 4
- Hikrobot MVS SDK: `/opt/MVS`
- RKNN runtime: `lib/librknnrt.so`
- RKNN C API header: `include/rknn_api.h`
- Python 3 (RS485脚本)
- pyserial (可选，RS485实际串口发送时需要)

## Project Layout

```text
mine_pedestrian/
├── README.md
├── Makefile
├── .gitignore
├── config/               # 运行配置文件
│   └── mine_config.conf  # IP、端口、串口等参数
├── src/                  # C++ source
├── scripts/              # 管理和运行脚本
├── models/               # RKNN models
├── include/              # RKNN API header
├── lib/                  # project-local RKNN runtime
├── web/                  # Web 页面和实时 JSON
├── docs/images/          # README 展示图
├── test_images/          # 测试图片
├── bin/                  # 编译产物
├── logs/                 # 运行日志
└── outputs/              # 推理输出
```

## Model

当前默认模型：

```text
models/current_seg.rknn -> models/yolov8n_seg_rk3588_i8.rknn
```

已测试权衡：

| Model | Notes |
| --- | --- |
| `yolov8n_seg_rk3588_i8.rknn` | 当前默认，速度和效果平衡最好 |
| `yolov8s_seg_rk3588_fp.rknn` | 精度略好但实时帧率低，保留作对比 |

实时相机流常见表现约 `28-32 FPS`。

## Generate Demo Images

```bash
./scripts/bench_images.sh test_images docs/images 1
```

## Notes

- 不要直接依赖系统 `/usr/lib/librknnrt.so`，它可能太旧。
- 相机网口建议固定为 `192.168.10.1/24`，相机固定为 `192.168.10.2/24`。
- `footpoint_px`、`bearing_deg`、`range_zone` 是视觉侧给雷达融合的粗定位信息。
- RS485 发送依赖 `web/live_targets.json`，确保推理服务先启动。
- 配置文件 `config/mine_config.conf` 首次运行 `mine_control` 时自动生成。
- `logs/`、`outputs/`、`MvSdkLog/` 是运行产物。
