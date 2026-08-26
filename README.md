# 具身智能精密装配

通过「语音唤醒 → 语音识别派发任务 → 工业相机拍照 → 视觉识别 → 机械臂抓放」的闭环，完成两道赛题：

| 任务 | 目标 | 涉及能力 |
|------|------|----------|
| 任务一 | 识别任务卡一场景内容并语音播报 | 机械臂到位 + 海康 MVS 拍照 + Qwen 视觉大模型 + TTS 播报 |
| 任务二 | 阅读任务卡二装配指令，按序把六色方块抓放到对应颜色托盘 | 任务卡解析 + OpenCV 六色定位 + 九点标定 + AUBO 抓放 + 吸盘控制 |

> 更完整的架构与流程说明见 [docs/项目流程说明.md](docs/项目流程说明.md)；答辩提纲见 [docs/PPT大纲.md](docs/PPT大纲.md)。

---

# 食用手册（单机本地部署）

## 0. 前提条件

- **Windows 10/11**（海康 MVS 客户端、AUBO SDK 的 wheel 均为 Windows 版）。
- 已安装 **海康 MVS 客户端**（提供相机驱动与 SDK，见「第 4 步」）。
- 机器人侧需有 **AUBO 控制器** 在同网段可达。
- NVIDIA 显卡**可选**：没有也能跑（语音识别走 CPU），只是 PyTorch 的 CUDA 版包更大一些。

## 1. 创建 conda 环境（Python 3.10）

AUBO SDK 的 wheel 是 `cp310` 版本，**必须用 Python 3.10**：

```bash
conda create -n assembly python=3.10 -y
conda activate assembly
```

（环境名 `assembly` 可自行改，后文一致即可。）

## 2. 安装依赖

**必须在项目根目录执行**（`requirements.txt` 里引用了相对路径的本地 wheel）：

```bash
# 2.1 先装 CUDA 版 PyTorch（requirements 里固定了 +cu124，PyPI 上没有，需从官方 cu124 源装）
pip install torch==2.6.0+cu124 torchaudio==2.6.0+cu124 torchvision==0.21.0+cu124 --index-url https://download.pytorch.org/whl/cu124

# 2.2 再装其余全部依赖（含本地 AUBO SDK wheel）
pip install -r requirements.txt
```

- 若这台机器**没有 NVIDIA 显卡**，可把 2.1 换成 CPU 版（`pip install torch==2.6.0 torchaudio==2.6.0 torchvision==0.21.0`），语音识别仍能正常跑，只是不能用 GPU 加速。
- `requirements.txt` 中的 `pyaubo-sdk @ file:./resources/pyaubo_sdk-0.24.1-cp310-cp310-win_amd64.whl` 引用仓库自带的 wheel，无需额外下载。

## 3. 放置语音识别模型（FunASR / SenseVoiceSmall）

语音识别用的是本地模型 SenseVoiceSmall，模型体积较大**不进 git**（已 gitignore），需自行放置到：

```
resources/modelscope/hub/models/iic/SenseVoiceSmall/
```

两种方式任选其一：

- **从实验室机器拷贝**：直接把那台机器上 `resources/modelscope/` 整个目录拷到本机对应位置（最快）。
- **modelscope 下载**：
  ```bash
  modelscope download --model iic/SenseVoiceSmall --local_dir resources/modelscope/hub/models/iic/SenseVoiceSmall
  ```

放置完成后，`config.ASR_MODEL_DIR` 里对应的路径（已按项目根自动推导，无需改）就能找到模型。

## 4. 安装海康 MVS 客户端并配置 SDK 目录

1. 到海康官网下载并安装 **MVS（Machine Vision Software）客户端**，用于给相机配置 IP、验证取图。
2. 装好后，把 `config.MVS_SDK_DIR` 改成**这台机器**上的 MVS Python 例程目录：

   ```python
   MVS_SDK_DIR = r"E:\software\mvs\MVS\Development\Samples\Python"
   ```

   （默认是 `E:\software\mvs\...`，换机器按本机实际安装位置改；该目录下应有 `MvImport/MvCameraControl_class.py`。）
3. 运行时 DLL 使用 MVS 标准安装路径 `C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64`，装了 MVS 客户端即存在，无需改动。

## 5. 填写现场参数（config.py）

现场一般只需改 `config.py` 里这几处（其余已有合理默认值）：

| 参数 | 含义 | 说明 |
|------|------|------|
| `CAMERA_IP` | 海康相机 IP | 用 MVS 客户端查看/设置相机实际 IP |
| `MVS_SDK_DIR` | MVS Python 例程目录 | 见「第 4 步」 |
| `ROBOT_IP` | 遨博机器人 IP | 实机 IP；注释里另有虚拟机器人 IP 供调试 |
| `ROBOT_PORT` / `ROBOT_USER` / `ROBOT_PASSWORD` / `ROBOT_NAME` | 机器人连接参数 | 默认 `30004` / `aubo` / `123456` / `rob1`，按现场改 |
| `ROBOT_TARGET` | 任务卡一拍照位（六维位姿） | **需示教标定** |
| `TASK2_CARD_VIEW_POSE` / `TASK2_BLOCK_VIEW_POSE` / `TASK2_TRAY_VIEW_POSE` | 任务二三区拍照位 | **需示教标定** |
| `TASK2_CALIBRATION_ORIGIN_POSE` / `TASK2_BLOCK_CALIBRATION_ORIGIN_POSE` / `TASK2_TRAY_CALIBRATION_ORIGIN_POSE` | 三区标定原点位姿 | **需示教标定** |
| `ASR_MODEL_DIR` | 语音识别模型目录 | 已按项目根推导，一般无需改 |

> 位姿格式统一为 `[x, y, z, rx, ry, rz]`，XYZ 单位 mm、RX/RY/RZ 单位弧度。**仓库里的位姿值是实验室那台机器标出来的**，换机器/换工位后必须重新示教标定（见「第 7 步」）。

## 6. 配置密钥（secrets.json）

大模型密钥通过 **环境变量** 或 **本地 `secrets.json`** 提供（`secrets.json` 已 gitignore，不会提交）：

1. 复制模板并改名：

   ```bash
   copy secrets_example.json secrets.json
   ```

2. 编辑 `secrets.json`，把 `DASHSCOPE_API_KEY` 的值填成你的 DashScope API Key：

   ```json
   {
     "DASHSCOPE_API_KEY": "sk-你的key"
   }
   ```

   （也可以用环境变量 `set DASHSCOPE_API_KEY=sk-你的key`，优先级高于 `secrets.json`。）

## 7. 现场一次性标定（换机器必做）

换机器后，仓库里 `data/` 下的标定数据是**实验室那台机器的坐标，不可直接用于新机器抓放**。正式抓放前，按顺序重做下面三步，生成你本机的数据：

| 步骤 | 工具 | 产物 | 作用 |
|------|------|------|------|
| 1 | `python task2_tuner.py` | `data/task2_tuning.json`（gitignore） | 调六色 HSV 阈值、曝光/增益 |
| 2 | `python task2_offset_calibrate.py` | `data/task2_offsets_v2.json` | 方块/托盘 XY 偏移 + 抓放 Z（示教器对准） |
| 3 | `python task2_tray_verify.py` | `data/task2_verified_trays.json` | 人工验收托盘坐标 |

标定要点（现场经验）：

- **九点标定**：用 VisionMaster 生成标定文件，命名为 `visionmaster_task2_calibration.xml` 放入 `resources/`；标定原点用**基座坐标**（mm/rad）填进 `config.TASK2_*_CALIBRATION_ORIGIN_POSE`。**方块拍照点与托盘拍照点在基座坐标下的 Z 高度、rz（相机旋转轴）必须一致**，否则两块区域无法共用同一标定矩阵。
- **抓放 Z**：`TASK2_BLOCK_PICK_Z` 推荐 28mm——**偏高吸不住、偏低会撞**。`task2_offset_calibrate.py` 自动生成的 JSON 会覆盖 config 里的 Z；想手动定 Z 就把 `config.TASK2_MANUAL_Z_OVERRIDE = True`（XY 偏移仍自动读 JSON，无需手改 JSON）。
- **颜色识别**：HSV 阈值手调不友好，最终方案是「HSV + 加权联合评分」（HSV 覆盖率 + 色相距离 + Lab 距离）共同区分六色，由 `TASK2_COLOR_FALLBACK_ENABLED` 在 HSV 检测异常时自动兜底。
- **托盘验收**：`task2_tray_verify.py` 是为调试时提前人工验收固定托盘、避免后续意外而做的优化项。

> 建议的**安全调试顺序**：先把 `config.TASK2_EXECUTE_ROBOT = False` 只测视觉（不动机械臂），视觉通了再标定，最后改回 `True` 开真抓放。

## 8. 确认开关（已默认 True，代表正式运行模式）

`config.py` 里与「直接跑通」相关的开关目前**都已设成 True**，配完现场参数即可直接运行：

| 开关 | 值 | 作用 |
|------|----|------|
| `TASK2_USE_VERIFIED_TRAYS` | `True` | 任务二读人工验收托盘坐标（否则现场重新识别托盘） |
| `TASK2_REQUIRE_ALL_COLORS` | `True` | 六色必须齐全，缺色报错 |
| `TASK2_COLOR_FALLBACK_ENABLED` | `True` | HSV 检测异常时启用六色联合兜底 |
| `TASK2_REQUIRE_OFFSET_FILE` | `True` | 必须有 `data/task2_offsets_v2.json` 才允许真实抓放（防误用旧坐标） |
| `TASK2_MANUAL_Z_OVERRIDE` | `False` | 手动定抓放 Z（`True` 时用 config 的 Z 覆盖偏移文件里的 Z，XY 仍自动读 JSON） |
| `TASK2_EXECUTE_ROBOT` | `True` | 执行真实机械臂抓放（临时调视觉时可改 `False`） |
| `ROBOT_VACUUM_ENABLED` | `True` | 启用末端吸盘 Tool IO |

## 9. 运行

```bash
python main.py
```

- 先喊唤醒词「**小具同学**」，听到「我已就绪，请下达指令」后，再说「**任务一**」或「**任务二**」。
- 想跳过语音唤醒、直接测流程，可分别运行 `python task/task1.py`、`python task/task2.py`。
- 识别到「**退出系统**」时程序退出。

## 10. 排障：看 output/ 核对中间流程

所有运行时产物统一落在 **`output/`**（已 gitignore）。流程跑不动时，按链路逐段核对：

```
output/
├── hik_mvs_capture.jpg            # 任务一拍照结果
├── temp_voice_command.wav         # 唤醒后录到的语音（检查 ASR 输入）
└── task2/
    ├── task2_card.jpg             # 任务卡二拍照原图
    ├── task2_blocks.jpg           # 方块区拍照原图
    ├── task2_trays.jpg            # 托盘区拍照原图
    ├── blocks_detected.jpg        # 方块检测标注图（绿框 + 编号）
    ├── trays_detected.jpg         # 托盘检测标注图
    ├── *_mask.png                 # 六色 HSV 分割掩膜
    ├── task2_llm_*.txt            # 大模型原始返回（核对任务卡解析）
    ├── task2_plan_*.json          # 生成的装配计划
    ├── task2_error_*.log          # 错误日志
    ├── diagnostics/               # 视觉诊断图
    ├── offset_calibration/        # 偏移标定中间产物
    └── tray_verification/         # 托盘验收中间产物
```

| 症状 | 先看哪里 |
|------|----------|
| 语音没反应 / 识别错 | `output/temp_voice_command.wav` 听录音；`config.ASR_CORRECTION_DICT` 是否覆盖 |
| 任务一播报内容不对 | `output/hik_mvs_capture.jpg` 拍得是否清晰；`output/task2/` 同理看各区原图 |
| 任务二颜色识别错 / 缺色 | `output/task2/*_mask.png`、`blocks_detected.jpg`、`trays_detected.jpg`；HSV 阈值用 `task2_tuner.py` 调 |
| 任务卡解析失败 | `output/task2/task2_llm_*.txt` 看大模型原文返回 |
| 抓放位置偏 / 报错 | `data/task2_offsets_v2.json` 是否本机标定；`output/task2/offset_calibration/` 中间产物 |
| 程序异常退出 | `output/task2/task2_error_*.log` 最新一条 |

---

## 技术栈

- **海康 MVS**：工业相机采集、参数配置、实时取图
- **OpenCV**：图像预处理、HSV 分割、轮廓检测、中心点定位、视觉标定转换
- **大语言模型 API（DashScope Qwen）**：任务卡内容识别、装配指令解析、步骤规划
- **AUBO SDK（pyaubo-sdk）**：机器人状态读取、TCP 位姿、运动控制、末端吸盘 IO
- **FunASR / SenseVoiceSmall**：本地语音识别（唤醒 + 指令）
- **pyttsx3**：本地 TTS 播报

## 文档

- [项目流程说明](docs/项目流程说明.md) — 完整流程与架构说明
- [PPT 大纲](docs/PPT大纲.md) — 答辩演示提纲

## 致谢与模型选型

本项目开发得到了 Codex、ChatGPT、DeepSeek、Claude 等 AI 的辅助，特别鸣谢。大模型识别环节感谢通义千问（Qwen）提供的免费模型额度；本次任务对识别能力要求不强，早期模型（如 qwen_v1）已够用且耐用，出于成本考虑推荐使用早期大模型（可在 `config.MODEL` 按需调整）。
