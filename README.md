# 具身智能精密装配 · 国赛改造版

在已完赛的省赛工程上扩展九色识别、旋转对准、方块叠放及 AI 盒子语音接入。保留 Windows 上位机、海康 MVS、AUBO SDK、吸盘 IO 和标定数据体系。

本文是当前代码的使用与验收入口，更新于 **2026-09-19**。改动对照基线为省赛留档提交 `bd66985`（提交名 `save final competition version`）。

**当前状态：功能代码已接入，40 项离线回归通过，AI 盒子健康检查通过；国赛实物的颜色、角度、高度、语音和整机精度尚未完成验收。软件测试通过不等于整机已调通。**

## 1. 相较省赛，改了哪些

| 项目 | 省赛基础 | 当前国赛实现 | 验证边界 |
| --- | --- | --- | --- |
| 任务卡 JSON | `step / block_color / tray_color`，固定六步落盘 | `step / source_color / target_type / target_color`，区分托盘与方块目标 | 新协议回归通过；旧 JSON 不再直接兼容 |
| 返回内容校验 | 六色及六步覆盖检查，程序重新编号 | 严格字段、连续编号、来源不重复；前六步覆盖六块六盘；叠放目标须已放置且顶面未占用 | 不补全、不重排；最后阶段动作数待样卡收紧 |
| 颜色识别 | 六色方块、六色托盘 | 方块增加青/粉/棕，共九色；托盘仍六色 | 合成图通过；新增三色 HSV 是待调初值 |
| 联合颜色匹配 | HSV 覆盖率、色相和 Lab 评分，全排列分配 | 改用匈牙利匹配，增加匹配代价阈值及重复中心拒绝 | 避免九色全排列开销，不保证实物识别率 |
| 方向识别 | 有图像轮廓角度，未用于主动对准 | 将矩形边两端经九点矩阵变换，计算基座 XY 方向角 | 不直接拿图像角度当机械臂转角；须实测方向 |
| 旋转执行 | 固定抓放姿态搬运 | 正方形每 90° 等价，选最小转角；共同净空高度旋转再转移；处理 ±π 姿态误差边界 | 数学与调用顺序通过，实物待验收 |
| 放置目标及高度 | 方块落托盘，固定抓放 Z | 放到已放置方块上；来源高度补偿、目标顶面累计、多层计划 | 30/28 mm 为暂定高度，必须实测 |
| 执行状态 | 记录抓放计划及执行结果 | 抓放前预检完整计划；每步成功才提交 `placed_block_map`；失败停止后续并落日志 | “成功”是程序/控制器返回，不是视觉复核 |
| 任务二不运动模式 | 关闭抓放仍会走拍照位移动；未连接时可能只生成计划 | `TASK2_EXECUTE_ROBOT=False` 跳过任务二拍照位移动及抓放；开启时无机器人则报错 | 仅限任务二，不是全局禁动开关 |
| 主流程及任务一返回 | 缺少整轮成功状态区分 | 任务一返回成功/失败；只把成功任务计入完成；两项成功后最终播报并退出；失败清空本轮记录；启动播报异常也断开机器人连接 | 不等于增强任务一场景理解；旧逻辑限制见第 3 节 |
| 语音设备 | 电脑 PyAudio + FunASR + pyttsx3 | 默认盒子 HTTP 录音、ASR、Piper 离线播报；调用接口不变，保留 `local` 后端 | 仅实测了健康检查，声音未验收 |
| 唤醒及语音故障 | 本地文字识别后宽松匹配 | 盒子识别文字后纠错、匹配“小具同学”；故障明确抛错，不自动改用电脑音频 | 不是盒子原生关键词模型；不支持播报打断 |
| 调参与诊断 | 六色调参和分区视觉测试 | 方块 1～9、托盘 1～6；旧六色配置按颜色合并；保存一侧保留另一侧；诊断改为九块六盘 | 标定文件格式不变 |
| 回归与文档 | 原省赛测试/说明 | 新增计划模块、语音协议回归、盒子独立测试及说明 | 当前 39 项离线用例，不接真实硬件或外部 API |

主要代码位置：`config.py`、`main.py`、`modules/llm.py`、`modules/task2_planning.py`（新增）、`modules/task2_vision.py`、`modules/robot.py`、`modules/voice.py`、`task/task1.py`、`task/task2.py`、`task2_tuner.py`、`task2_vision_test.py`。

没有重写：相机采集封装、AUBO 连接/基础运动、吸盘电气控制、位姿记录、九点标定和 XY 偏移工具。原有数据保留，但换设备/工位/TCP 后不能直接认为可用。没有修改盒子的系统、IP、原生模型或服务配置，也未在工程中保存盒子登录密码。

## 2. 当前整体逻辑

主程序保持两段式交互：

```text
“小具同学” → “我已就绪，请下达指令” → “任务一”或“任务二”
任务成功 → 返回等待再次唤醒
两项均成功 → “两个任务均已完成” → 程序退出
任务失败 → 清空本轮完成记录 → 提示重新发卡（先检查/恢复实物场景）
```

任务一：到任务一拍照位 → 采集 → Qwen 理解 → 播报结果。

任务二沿用“三处各拍一次”，不是每抓一块都重拍：

```text
任务卡拍照并解析 → 校验/复述
→ 方块区拍照（九色） → 托盘区拍照（六色）
→ 预检全部抓放计划 → 六次落盘 → 按卡面叠放 → 记录完成
```

### 2.1 指令格式与目标位置

下面只是单项结构，不是一份完整可执行任务卡：

```json
{"step": 1, "source_color": "红色", "target_type": "tray", "target_color": "黄色"}
```

`tray` 目标使用当前识别的托盘位置和方向；`block` 目标使用该块前面放置后的预测位置、方向和顶面高度，不使用它仍在原料区时的位置。

前六步红橙黄绿蓝紫各使用一次，六色托盘各占一次。后续来源只能是青/粉/棕，各最多一次；目标须先放置且顶面未占用，可继续叠到前面已放好的新增色上。

`TASK2_EXPECTED_STACK_COUNT=None` 暂接受 1～3 次叠放，即总计 7～9 步；确定样卡后应设为精确的 1/2/3。0 仅用于六步回归，仍使用新 JSON。这是当前暂定校验范围，不是对最终赛规的额外解释，也不能防止 OCR 漏动作后仍恰好满足总数范围。

### 2.2 角度与高度

图像角 `angle_deg` 和基座角 `robot_angle_deg` 分开保存。正方形对准增量：

```text
delta = (目标基座角 - 来源基座角 + 45) % 90 - 45
放置 rz = 抓取 rz + radians(delta)，归一化到 [-π, π)
```

按当前竖直吸取方式保持抓取 rx/ry。须确认九点坐标与基座一致、TCP 在吸盘中心、吸起后方块不打滑；平面标定不能自动消除不同高度的透视误差。

```text
抓取 TCP Z = 参考抓取 Z + 来源块高度 - 参考高度
落盘 TCP Z = 参考放置 Z + 来源块高度 - 参考高度
叠放 TCP Z = 目标块已记录的顶面 TCP 接触 Z + 来源块高度
```

顶面 Z 是同一吸盘接触顶面时的等效 TCP 高度，包含工具偏置，不是裸物块物理表面高度。参考抓放 Z 的 173/180 mm 是旧工位值；参考高度 30 mm、各色 30/28 mm 须与实际 TCP 一起复核。

共同净空高度取当前 Z、`ROBOT_SAFE_Z`、抓取 Z+抬升量、放置 Z+抬升量的最大值。当前 `ROBOT_SAFE_Z=400` mm；这不是避障规划，不能替代路径、工作空间及机械限位检查。

## 3. 测试前必须知道

- **默认 `TASK2_EXECUTE_ROBOT=True`、`ROBOT_VACUUM_ENABLED=True`。不要一上来运行 `main.py` 或两个任务入口。**
- `TASK2_EXECUTE_ROBOT=False` 只约束任务二，不禁止任务一、标定/吸盘工具运动或输出，也不阻止独立任务入口创建机器人连接。
- `ROBOT_VACUUM_ENABLED=False` 不是禁动模式；不能靠关吸盘模拟真实装配成功。
- “退出系统”只在语音监听阶段处理，不是运动中的急停。使用现场急停及控制器安全措施，不能依赖语音停止机械臂。
- 拍照后不重定位，吸取/落点没有视觉复核；物体被挪动、滑落或未吸到但 IO 正常时，程序状态可能与实物不一致。失败后无断点续装，不要直接从旧计划续跑。
- **任务一仍有省赛逻辑：到位失败也尝试采集；根目录 `任务卡1.png` 存在时优先读本地图。** 返回成功主要代表识别/播报完成，不能证明机器人到位或使用了现场图。
- 真机测试前设 `DEBUG_IMAGE=None`、三项 `TASK2_*_DEBUG_IMAGE=None`；任务一还需把 `LOCAL_TEST_IMAGE_NAME` 指向不存在的测试文件，或将原样图移到其他位置保存。任务二指定输出文件名，不走任务一本地图兜底，但仍受调试图配置影响。
- 尚无真实单步抓放 CLI、原生“小具同学”模型适配、播报打断、备用文本交互、五分钟自动计时、任务一国赛场景语义增强、装配中重拍重定位。

## 4. 环境与配置

### 4.1 使用现有环境

下文在项目根的 PowerShell 执行。本机已验证 Windows 64 位、Python 3.10.20、NumPy 2.2.6、OpenCV 4.12.0、SciPy 1.15.3：

```powershell
Set-Location E:\project_test\project_all
conda activate env_aubo
python --version
```

`env_aubo` 是本机环境名，换机按实际名称调整。AUBO wheel 为 `cp310-win_amd64`，完整工程使用 Python 3.10。已有环境能运行时不用为盒子重装，盒子语音客户端只用标准库。

新电脑需另建 Python 3.10 环境、安装海康 MVS 及项目依赖。`requirements.txt` 仍为省赛完整依赖快照，含本地语音及固定 CUDA PyTorch；本次未做新环境重装验收。沿用完整依赖的安装命令：

```powershell
conda create -n assembly python=3.10 -y
conda activate assembly
python -m pip install torch==2.6.0+cu124 torchaudio==2.6.0+cu124 torchvision==0.21.0+cu124 --index-url https://download.pytorch.org/whl/cu124
python -m pip install -r requirements.txt
```

必须在项目根执行，以解析 `resources/pyaubo_sdk-0.24.1-cp310-cp310-win_amd64.whl`。盒子模式不需要电脑 CUDA/FunASR 模型；如另做精简或 CPU 环境，须同步处理 requirements 固定项，不能只换第一条命令便假定兼容。

### 4.2 网络与外部依赖

| 设备/服务 | 当前配置 | 说明 |
| --- | --- | --- |
| AI 盒子 | `http://192.168.1.11:8765` | Ubuntu；麦克风/扬声器接盒子，HTTP 不需要 SSH 密码，限可信内网 |
| AUBO 控制器 | `192.168.1.10:30004`，实例 `rob1` | 当前默认值，需核对实机 |
| 海康相机 | `192.168.1.20` | 用 MVS 验证取图，避免与其他软件同时占用 |
| 视觉大模型 | `config.API_URL / MODEL` | 上传卡面/场景图，需要外网和有效密钥，可能计费 |

`MVS_SDK_DIR` 指向本机 MVS 的 `Development/Samples/Python`，应含 `MvImport/MvCameraControl_class.py`。驱动和 DLL 沿用原安装。

模型密钥由 `DASHSCOPE_API_KEY` 环境变量或本地 `secrets.json` 提供。首次设置参考 `secrets_example.json`，不覆盖已有文件，不提交密钥。`test/api_test.py` 是旧版独立 API 样例，不能替代正式任务卡解析验收。

### 4.3 参数与数据来源

| 参数/文件 | 用途及注意事项 |
| --- | --- |
| `VOICE_BACKEND / AI_BOX_URL` | 默认 `ai_box` / 盒子地址，支持同名环境变量覆盖 |
| `AI_BOX_*TIMEOUT / AI_BOX_VAD_THRESHOLD` | HTTP 等待、录音、VAD；不同于省赛本地 RMS 阈值 |
| `TASK2_REQUIRE_ALL_COLORS=True` | 九块六盘完整性；关闭后仍校验指令涉及的目标 |
| `TASK2_EXPECTED_STACK_COUNT=None` | 叠放数量，样卡确认后收紧；0 仅六步回归 |
| `TASK2_ROTATION_ENABLED=True` | 基座方向角对准；关闭后不伪造已对准的方向 |
| `TASK2_ROTATE_AT_CLEARANCE=True` | 高位先单独旋转；关闭时可能在高位转移中改姿态 |
| `TASK2_REQUIRE_OFFSET_FILE=True` | 抓放前要求 XY 文件，不替代现场标定验收 |
| `TASK2_BLOCK_PICK_Z / TASK2_TRAY_PLACE_Z` | 参考 TCP 抓放高度，只取 config，不从偏移 JSON 读 Z |
| `TASK2_REFERENCE_BLOCK_HEIGHT_MM / TASK2_BLOCK_HEIGHT_MM` | 参考及每色高度，全部实测，不以平面边长代替 |
| `ROBOT_SPEED / ROBOT_ACCELERATION / ROBOT_SAFE_Z` | 当前 0.3 / 0.3 / 400；首次联调按现场要求降低速度，运动单位沿 SDK |
| `data/aubo_poses.json` | 四个拍照位唯一来源；XYZ 为 mm，姿态为 rad |
| `resources/visionmaster_task2_calibration.xml` | 像素到平面矩阵，中心与方向共用；换相机姿态/工位后复核 |
| `data/task2_offsets_v2.json` | 两区 XY 原点、偏移和对应拍照姿态，须与位姿和矩阵单位匹配 |
| `data/task2_tuning.json` | 颜色、曝光、增益等；存在时覆盖相关 config；不进 Git，换机另备份 |
| `data/task2_verified_trays.json` | 人工验收记录，仅调试，正式任务不读取 |

## 5. 整体测试流程

按阶段验收后再推进。**纯离线回归、静态图计划、真机实验不能互相替代。**

### 第 0 步：备份与安全确认

记录 config，备份位姿、XY、九点 XML、调参 JSON。位姿记录器会清理 `data/` 和 `output/aubo_pose_records/` 内旧的 `aubo_poses*.json/txt` 副本，备份应放在上述目录之外。确认 TCP、电压、工作空间、急停；不直接照搬旧工位数值抓放。

### 第 1 步：纯离线回归

```powershell
python -m unittest discover -s test -p "test*.py" -v
```

通过标准：39 项全部 `OK`。覆盖动作规则、九色合成图、角度/高度、运动顺序模拟、失败不提交、主流程完成判定、语音异常及调参窗口关闭；不连接盒子、相机、机器人或视觉 API，也不加载本地语音模型。

### 第 2 步：AI 盒子语音单测

先关闭其他占用盒子麦克风的客户端。下列命令不连接机器人：

```powershell
python test/ai_box_check.py
python test/ai_box_check.py --action tts
python test/ai_box_check.py --action listen
python test/ai_box_check.py --action dialogue
```

依次验收：

1. 默认仅 GET 健康检查，`ready=true`、资源存在；`capture_ready=lazy` 不证明音频设备正常。
2. `tts`：人工确认盒子确实出声、清晰且完整。
3. `listen`：提示后直接说“任务一”，终端显示“任务一”，不需先喊“小E同学”。
4. `dialogue`：单独说“小具同学”，等就绪播报后说“任务一/任务二”；只复述，不执行任务。
5. 重复测试距离、噪声、冷启动耗时及“退出系统”，记录失败样例。

盒子原生关键词是“小E同学”；当前用 `prewoken=true` 调 ASR，上位机纠错后匹配“小具同学”。不是更换原生模型，不支持唤醒词和任务词连说、播报中打断。若赛方要求原生关键词检测，需确认方案或向设备方取得模型。

断线、ASR 超时、播报跳过/打断明确报错，不自动重发或切电脑音频。客户端退出/超时不意味着服务端已停止，等本轮结束后再重试。

显式切换后端（仅当前 PowerShell 会话）：

```powershell
$env:VOICE_BACKEND = "local"   # 省赛电脑麦克风/扬声器，需原模型和依赖
$env:VOICE_BACKEND = "ai_box"  # 恢复盒子
```

本地模型仍在 `resources/modelscope/hub/models/iic/SenseVoiceSmall/`，由 `ASR_MODEL_DIR` 指定，不进 Git。独立 `ai_box_check.py` 固定测盒子，不受 `VOICE_BACKEND=local` 影响。

### 第 3 步：相机、九色、方向静态检查

先由操作员用示教器把相机放到相应拍照位。下列工具自身不移动机械臂，`--camera` 会取图，`--show`/调参打开窗口。清除 `DEBUG_IMAGE`，关闭占用相机的其他程序：

```powershell
python task2_vision_test.py --scene card --camera --show
python task2_vision_test.py --scene block --camera --show
python task2_vision_test.py --scene tray --camera --show
```

卡面模式只查清晰度，不调用模型。查看 `output/task2/diagnostics/<时间_区域>/` 的原图、标注、`detections.json`、`summary.txt`。方块九色、托盘六色，轮廓和中心不得重复指向同一物体；核对 `robot_angle_deg`，不只看颜色正确。

当前学校暂无青色方块，`config.py` 的 `TASK2_DISABLED_BLOCK_COLORS = ("青色",)` 临时跳过青色检测和完整性检查，方块诊断预期为其余八色；托盘仍为六色。任务卡若需要青色方块会明确报错并停止，不会跳过该抓放。青色到货并完成实物调参后，将该项改为 `()`，恢复九色检查。

已有图可替代相机，路径换成实际文件；颜色调参命令如下：

```powershell
python task2_vision_test.py --scene block --image output/task2/task2_blocks.jpg --show
python task2_tuner.py --camera --scene block
python task2_tuner.py --camera --scene tray
```

调参方块按 1～9、托盘按 1～6，`s` 保存、`c` 刷新、`q` 退出；保存一侧保留另一侧。重点实测青/粉/棕与旧颜色的区分，不以放宽完整性开关替代调参。

诊断脚本即使用静态图也读取四点位姿和现有 XY 文件，过期标定可能提前报错。新工位可先用调参工具只调图像，完成第 5 步标定后回来复验坐标/方向。

### 第 4 步：静态图生成完整计划，不实例化机器人

验证“真实卡面 → 新 JSON → 九色目标 → 全部抓放位姿”。需要三张区域图及匹配的位姿/XY/九点数据。**会发送任务卡图片到配置的视觉 API，可能计费，不是纯离线测试。**

不直接运行 `task/task2.py`（独立入口会创建 `Robot()`），而在 PowerShell 用下列片段。先改三项图片路径为实际文件；仅在当前进程修改配置，不改 config 文件：

```powershell
@'
from pathlib import Path
from types import SimpleNamespace
import config
from modules.vision import Vision
from modules.llm import LLM
from task.task2 import task2_run

config.TASK2_EXECUTE_ROBOT = False
config.TASK2_CARD_DEBUG_IMAGE = r"output/task2/task2_card.jpg"
config.TASK2_BLOCK_DEBUG_IMAGE = r"output/task2/task2_blocks.jpg"
config.TASK2_TRAY_DEBUG_IMAGE = r"output/task2/task2_trays.jpg"
for image in (config.TASK2_CARD_DEBUG_IMAGE, config.TASK2_BLOCK_DEBUG_IMAGE,
              config.TASK2_TRAY_DEBUG_IMAGE):
    if not Path(image).is_file():
        raise SystemExit("请先填写有效图片路径：" + image)
result = task2_run(SimpleNamespace(speak=print), Vision(), None, LLM())
if not result or result["status"] != "planned":
    raise SystemExit("计划生成失败，检查 output/task2 的错误日志")
print("计划生成成功；未连接机器人、未录音或播报、未执行运动。")
'@ | python -
```

验收最新 `task2_llm_*.txt` 和 `task2_*.json`：原文动作无遗漏，前六步及叠放关系正确，抓放 XY/Z 和 `rotation_delta_deg` 合理。`status=planned`，每步 `pending`，`placed_block_map` 为空；`placed_state` 只是预测，不是成功执行记录。

不要只放一个块跑完整任务来“测单步”：完整计划有动作数及目标完整性校验。缺标定时先完成第 5 步受控标定，再回来重测。

### 第 5 步：现场标定与吸盘检查

从这里开始连接机器人，部分工具会真实运动/写 IO，不受 `TASK2_EXECUTE_ROBOT` 统一保护。备份并确认现场安全，逐项执行，禁止成组复制运行。

| 顺序 | 入口/操作 | 影响和验收 |
| --- | --- | --- |
| 1 | `python aubo_pose_recorder.py` | 读取示教器手动摆好的四点，保存唯一 JSON；不自动走四点，保存会清理旧命名副本 |
| 2 | VisionMaster 或已有九点工具 | `task2_vm_auto_calibrate.py`、`task2_vm_multi_position_calibrate.py` 会自动运动；生成候选 XML/报告，不自动替换正式 XML；人工验收后采用 |
| 3 | `python task2_offset_calibrate.py` | 自动移到两区拍照位，结合人工对准记录 XY，不标定 Z；须人工检查路径，不能假定旧标定脚本每段都走新高位转移逻辑 |
| 4 | 示教/尺测，填写 config | 校准参考抓放 Z、参考及每色高度、TCP、净空；不直接采用 30/28 mm 初值 |
| 5 | `python test/aubo_suction_check.py` | 连接机器人，只读 TCP/IO，不移动、不写输出 |
| 6 | `python test/aubo_suction_check.py --run` | 原地写工具 IO，人工确认后启停吸盘；核对 `SUCTION_ENABLE_OUTPUT_TEST`、工具电压和引脚 |

当前工具电压为 12V，须匹配实际硬件。位姿、矩阵、XY 补偿是一组数据，不能混用新旧版本；修改拍照位后重新标定。候选 XML 未采用就不会影响正式工程。

可选托盘验收：`python task2_tray_verify.py --no-move-robot`。不加该选项且未提供图片时，原工具可能自动移动到托盘拍照位；保存结果不替代正式任务重新拍照。

### 第 6 步：先验收单块，再放行整套任务

低速、受控现场依次验证：高位姿态变化 → 单块落盘对准 → 单块叠放 → 不同方向/高度/画面位置的重复实验。检查实际旋转方向、吸盘中心、滑移、落点和层高误差。

**目前没有真实单步抓放 CLI，也没有 `--step`、`--dry-run` 等任务参数。** 单步模拟不等于单块实测。此阶段需调试人员使用经过人工审核的单步调用，或另行增加带确认的单步入口；不把完整任务入口冒充单步工具。未通过前不直接放行完整抓放。

### 第 7 步：独立任务与整机回归

只有前述检查通过、清除调试图片、核对真实工位/安全参数后才运行：

```powershell
python task/task1.py
```

绕过唤醒直接执行任务一，可能运动、采集、调用 API 和播报。人工确认确实到位、使用现场图、结果及播报正确，不只看返回成功。

重新布置场景，确认 `TASK2_EXECUTE_ROBOT=True` 后：

```powershell
python task/task2.py
```

绕过唤醒直接执行完整任务二，不是单步测试。检查每次落盘/叠放及最终朝向、位置、高度；日志应为 `completed`，各步 `robot_status=completed`，已放置记录覆盖成功动作。仍须人工验收实物。

最后测试完整派发：

```powershell
python main.py
```

“小具同学”→等就绪→“任务一”，完成后再次唤醒→“任务二”，也支持反序。两项成功后最终播报并退出；本轮重复请求已完成任务会提示执行另一项。失败清空本轮记录但不恢复实物，每次重试前检查现场、恢复物块并按流程重新发卡。

记录总耗时、各环节耗时及成功率，重复完整流程和已知失败场景。目前无五分钟自动截止，需人工计时。

## 6. 看结果与排障

产物集中在 `output/`，不进 Git；使用调试图时可能直接返回原文件，不一定另存 `task2_card.jpg` 副本。

| 文件/现象 | 检查内容 |
| --- | --- |
| `output/hik_mvs_capture.jpg` | 任务一实拍；若读本地样图，以终端打印的实际路径为准 |
| `output/task2/task2_card.jpg`、`task2_blocks.jpg`、`task2_trays.jpg` | 三区实拍，确认不是旧图/调试图 |
| `output/task2/*_detected.jpg`、`*_mask.png` | 轮廓、中心、颜色掩膜；缺色先查光照、曝光、HSV |
| `output/task2/diagnostics/` | 分区原图、标注、目标 JSON、摘要 |
| `output/task2/task2_llm_*.txt` | 模型原文和通过校验的步骤，重点查最后一句是否漏动作 |
| `output/task2/task2_*.json` | 动作、计划、旋转增量、完整位姿、每步状态、已放置记录 |
| `output/task2/task2_error_*.log` | 任务二错误及堆栈，失败先读日志，不直接续跑 |
| XY 偏移或边缘误差大 | 复核拍照位、矩阵范围、XY 补偿；不同高度有投影误差 |
| 旋转方向不对 | 区分图像/基座角，查标定坐标、TCP、滑移，不直接乱改正负号 |
| 叠放 Z 不对 | 查参考 TCP Z、来源高度、目标累计顶面、工具偏置 |
| 健康检查通过但无声/无识别 | 分项测试音频，查看错误码、设备连接；health 仅验证资源 |

盒子音频不回传电脑，`output/temp_voice_command.wav` 仅用于 `local`。更新位姿、标定或调参后，回到静态检查和计划验收，再运行实物任务。

## 7. 验收清单与资料

- [x] 协议、九色合成图、方向/高度、状态、调参窗口及语音离线回归：39 项通过。
- [x] AI 盒子 `/health` 实测可达，相关模型资源存在。
- [ ] 真实播报、录音、“小具同学”交互及噪声/延迟验收。
- [ ] 确认样卡叠放数量，验证 OCR 不漏最后阶段动作。
- [ ] 九色阈值、实际高度、TCP、九点及 XY 补偿复验。
- [ ] 高位旋转、单块对准、单块叠放、全区域精度实测。
- [ ] 任务一现场图及语义、任务二完整装配、两种任务顺序联调。
- [ ] 总时间、重复成功率、异常停机及恢复流程验收。

补充：`docs/国赛改造说明.md` 解释计划、方向和高度；`docs/AI盒子接入说明.md` 解释语音协议及状态；`AI盒子参考文件/` 为厂商资料。`docs/项目流程说明.md` 和原 PPT 属于省赛资料，不作为国赛参数及新指令格式的依据。

开发过程中使用了 Codex、ChatGPT、DeepSeek、Claude 等辅助工具，视觉大模型使用通义千问。实际模型由 `config.MODEL` 指定，切换后应重新验收任务卡及场景识别，不据旧演示推定新模型效果。
