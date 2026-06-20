# Mine Pedestrian Vision

![RK3588](https://img.shields.io/badge/SoC-RK3588-blue)
![Orange Pi 5 Pro](https://img.shields.io/badge/Board-Orange%20Pi%205%20Pro-orange)
![RKNN](https://img.shields.io/badge/Runtime-RKNN%20NPU-green)
![YOLOv8 Seg](https://img.shields.io/badge/Model-YOLOv8n--seg%20INT8-yellow)
![Hikrobot](https://img.shields.io/badge/Camera-Hikrobot%20GigE-lightgrey)

面向矿洞行人检测避障原型的边缘视觉工程。项目在 Orange Pi 5 Pro / RK3588 上接入海康威视 GigE 工业相机，使用 RKNN NPU 实时运行 YOLOv8 segmentation，输出行人框、分割 mask、脚底像素点、左右方向和近/中/远粗距离分区，并提供 MJPEG 实时可视化流。

## Demo

| 近处重点目标 | 多目标矿洞场景 | 复杂背景场景 |
| --- | --- | --- |
| ![demo three](docs/images/mmexport1781891594276.jpg) | ![demo many](docs/images/mmexport1781891601272.jpg) | ![demo five](docs/images/mmexport1781891604099.jpg) |

实时画面会把半透明信息卡贴在检测框旁边：重点目标显示置信度、方向、角度和距离分区；多人拥挤时，背景目标自动切换成小状态牌，减少遮挡。

## Features

- **实时行人检测与分割**：YOLOv8n-seg INT8 RKNN，RK3588 NPU 三核推理。
- **工程化可视化**：MJPEG 流直接叠加检测框、mask、方向线和目标卡片。
- **融合友好输出**：`web/live_targets.json` 输出 bbox、footpoint、bearing、range zone，可供后续雷达融合。
- **项目内 RKNN runtime**：使用 `lib/librknnrt.so`，避免系统旧版 runtime 报 `Invalid RKNN model version`。
- **海康 GigE 相机接入**：基于 Hikrobot MVS SDK `/opt/MVS`。

## Hardware

| Component | Value |
| --- | --- |
| Board | Orange Pi 5 Pro |
| SoC | Rockchip RK3588 |
| Camera | Hikrobot MV-CU013-A0GC GigE |
| Board Wi-Fi / SSH | `192.168.1.5` |
| Board Ethernet | `192.168.10.1/24` |
| Camera IP | `192.168.10.2/24` |
| Project path | `/home/orangepi/moonxkj/mine_pedestrian` |

## Quick Start

```bash
cd /home/orangepi/moonxkj/mine_pedestrian

# 可选：拉满 CPU/NPU/DDR 等频率，适合演示前执行
echo orangepi | sudo -S ./scripts/performance_mode.sh

# 启动实时推理和 MJPEG 服务
./scripts/run_live_web.sh
```

打开实时流：

- MJPEG stream: <http://192.168.1.5:8090/stream>
- 简洁全屏页面: <http://192.168.1.5:8080/>

停止服务：

```bash
./scripts/stop_live_web.sh
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

`range_zone` 是基于脚底点在图像里的纵向位置给出的粗分区，不是绝对距离；后续可以和雷达距离做融合。

## Scripts

| Script | Description |
| --- | --- |
| `scripts/run_live_web.sh` | 启动相机实时推理、MJPEG 流和 8080 全屏页面 |
| `scripts/stop_live_web.sh` | 停止实时推理和 8080 页面服务 |
| `scripts/run_image.sh` | 对单张图片推理并保存可视化结果 |
| `scripts/bench_images.sh` | 批量跑 `test_images`，生成 README 展示图 |
| `scripts/run_camera_once.sh` | 抓取一帧相机图并跑单图推理 |
| `scripts/run_camera_loop.sh` | 旧版循环抓图推理，调试用 |
| `scripts/run_smoke.sh` | 检查 RKNN 模型和 runtime 是否能加载 |
| `scripts/performance_mode.sh` | 设置 CPU/NPU/DDR 等 governor 为 performance |
| `scripts/setup_camera_net.sh` | 临时配置相机网口 IP，正常静态网络已配置后不需要每次执行 |

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

## Project Layout

```text
mine_pedestrian/
├── README.md
├── Makefile
├── .gitignore
├── src/                  # C++ source
├── scripts/              # run/build/test helpers
├── models/               # RKNN models, current_seg.rknn points to selected model
├── include/              # RKNN API header
├── lib/                  # project-local RKNN runtime
├── web/                  # minimal web entry and live JSON/snapshot
├── docs/images/          # README demo images generated from test_images
├── test_images/          # mine-like sample images for regression/demo
└── bin/                  # compiled binaries on the board
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

实时相机流常见表现约 `28-32 FPS`，具体取决于曝光、目标数量和可视化绘制开销。

## Generate Demo Images

```bash
cd /home/orangepi/moonxkj/mine_pedestrian
./scripts/bench_images.sh test_images docs/images 1
```

该命令会使用当前模型批量推理 `test_images`，并把带可视化叠加的结果写到 `docs/images/`。

## Notes

- 不要直接依赖系统 `/usr/lib/librknnrt.so`，它可能太旧。
- 相机网口建议固定为 `192.168.10.1/24`，相机固定为 `192.168.10.2/24`。
- `footpoint_px`、`bearing_deg`、`range_zone` 是视觉侧给雷达融合的粗定位信息。
- `logs/`、`outputs/`、`MvSdkLog/` 是运行产物，不纳入工程主内容。
