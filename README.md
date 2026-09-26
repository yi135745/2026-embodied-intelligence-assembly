# AUBO 具身智能精密装配 · 国赛工程

本项目由省赛 AUBO 系统迭代而来，完成“语音唤醒 → 任务卡理解 → 九色物块/六色托盘定位 → 方向对准 → 抓取、放置与叠放”。国赛版本沿用原有设备接口，增加了完整计划预检、公共平面标定、单一物理锚点和可追溯质量门禁。

> **安全提示**：`python main.py` 默认连接 AI 盒子、海康相机和 AUBO，并允许真实运动。离线测试通过不代表机器人路径、Tool IO、吸盘、相机或现场精度已经验收。新机器、新工位、相机/TCP 改动后，应先执行第 7 节现场流程。

## 1. 一分钟理解工程

```text
语音命令
   ↓
main.py                         生命周期、任务派发、完成状态
   ↓
task/task1.py / task/task2.py  业务编排和整份计划预检
   ↓
modules/*                      语音、相机、理解、机器人、感知、规划、标定
   ↓
drivers/*                      AUBO SDK、MVS、AI盒子、模型 API

data/ + resources/             唯一正式现场状态
tools/                         点位、调参、标定、审计与诊断
output/                        可删除的图片、候选、日志和运行证据
```

日常正式运行：

```powershell
conda activate env_aubo
python main.py
```

首次部署不要直接运行整机任务。完整“如何食用”见第 7 节。

## 2. 工程框架

### 2.1 目录职责

| 路径 | 责任 | 不应承担 |
| --- | --- | --- |
| `main.py` | 初始化、两段式语音派发、本轮完成状态、最终断开设备 | 视觉算法、任务规则、坐标计算 |
| `task/` | 任务一/二编排、完整计划预检、成功后提交状态 | SDK 细节、HSV、标定拟合 |
| `modules/voice.py` | 唤醒、识别、播报的统一接口 | 业务派发 |
| `modules/camera.py` | 单帧采集、调试图入口、曝光和增益下发 | 颜色识别、机器人移动 |
| `modules/interpreter.py` | 图片上传、任务一文字识别、任务二合法 JSON 解析 | 坐标、动作规则、运动决策 |
| `modules/robot.py` | AUBO 连接、单位转换、安全运动、抓放、Tool IO | 颜色和任务语义 |
| `modules/task2_perception/` | 颜色/形状、中心、方向、公共矩阵坐标换算 | 标定拟合、任务规划 |
| `modules/task2_planning.py` | 动作协议校验、标称终态推演、执行补偿 | 图像处理、SDK 调用 |
| `modules/task2_calibration/` | 矩阵拟合、质量评估、版本启用、物理锚点组合 | 正式任务编排 |
| `contracts/` | 跨模块纯数据结构和数值契约 | 设备、文件和流程 |
| `runtime/` | 加载并验证唯一正式位姿、矩阵绑定和物理标定 | 生成第二套现场数据 |
| `drivers/` | 厂商 SDK、HTTP 和硬件协议边界 | 业务规则 |
| `tools/setup/` | 点位记录、Tool IO 等设备设置 | 正式任务执行 |
| `tools/task2/workflow/` | 调参、矩阵标定、物理锚定、审计和恢复 | 日常业务入口 |
| `tools/task2/diagnostics/` | 静态图、相机、高度等诊断 | 正式状态的隐式修改 |
| `test/` | 无硬件副作用的离线回归 | 实机验收 |

详细工具索引见 [tools/README.md](tools/README.md)。

### 2.2 依赖原则

- `main → task → modules → drivers` 单向依赖；任务和工具负责组合多个能力。
- 各功能模块彼此平行，不复制 SDK 调用，也不各自维护现场坐标。
- LLM 只提取语义，不能直接生成机械臂坐标或绕过确定性校验。
- 任务二在第一次抓放前检查整份计划；只有真实抓放成功才更新 `placed_block_map`。
- 正式矩阵、位姿、物理锚点和调参文件各有唯一来源，禁止在脚本中再维护一份有效副本。

## 3. 公共接口与数据契约

### 3.1 单位与位姿

业务层所有 TCP 位姿统一为：

```text
[x, y, z, rx, ry, rz]
```

- `x/y/z`：毫米；
- `rx/ry/rz`：弧度；
- 只有 AUBO SDK 边界把毫米转换成米；
- 像素中心统一为 `(u, v)`；
- 图像角不能直接当作机器人 `rz`。

### 3.2 主要模块接口

| 能力 | 主要接口 | 契约摘要 |
| --- | --- | --- |
| 语音 | `Voice.wake()`、`listen()`、`speak()` | 统一 AI 盒子/本地后端；后端失败不伪装成功 |
| 相机 | `Camera.capture(output_name, debug_image, exposure_time, gain)` | 只负责得到图片，返回 `Path` |
| 任务理解 | `Interpreter.identify_image()`、`parse_task2_card()` | 只提取文字或合法 JSON，不决定坐标和动作安全 |
| 机器人 | `get_current_pose()`、`move_to()`、`move_to_safe()` | 返回/接收 mm+rad；跨区优先使用安全移动 |
| 抓放 | `Robot.pick_and_place()`、`set_suction()` | 明确成功才返回成功；Tool IO 失败不能吞掉 |
| 感知 | `ColorObjectDetector.detect(image, kind, ...)` | 输出颜色、像素中心、面积、方向和可选机器人位姿 |
| 视觉完整性 | `validate_colors(targets, kind)` | 方块校验九色，托盘校验六色 |
| 任务协议 | `validate_actions(actions)` | 检查步号、颜色、目标类型、重复来源和叠放规则 |
| 规划 | `build_plan(actions, blocks, trays)` | 生成标称抓放位姿和预测终态 |
| 动作补偿 | `apply_motion_compensation(plan, compensation)` | 只改变实际命令，不污染标称终态 |
| 标定 | `fit_relative_homography()`、`write_vm_xml()`、`activate_calibration()` | 拟合候选、写 XML、质量门禁后原子启用 |
| 运行状态 | `load_task2_runtime_state()` | 组合当前矩阵和物理锚点，验证版本与拍照条件 |

### 3.3 任务二动作协议

模型必须返回动作数组，每项恰好包含：

```json
{
  "step": 1,
  "source_color": "红色",
  "target_type": "tray",
  "target_color": "黄色"
}
```

确定性校验要求：

- `step` 从 1 连续编号，程序不补号、不重排；
- 来源方块不能重复；
- 前六步是六个基础色方块到六个不同托盘；
- 后续来源只能是青、粉、棕，目标必须是已放置且顶部未被占用的方块；
- 字段缺失、颜色非法、动作数不符或目标不可达时，整份计划在运动前失败。

### 3.4 视觉目标与计划语义

`VisionTarget` 保存：`kind`、`color`、`pixel_center`、`area`、`angle_deg`、`robot_pose` 和 `robot_angle_deg`。

- `robot_pose`：由公共矩阵、区域拍照位、共享相机—吸盘偏移、配置 Z 和拍照姿态组合出的标称位姿；
- `robot_angle_deg`：把矩形边的两个像素端点都映射到机器人平面后计算，避免图像 Y 轴翻转和透视造成角度符号错误；
- `placed_state.pose`：物块理论终态；
- `command_robot_pose`：扣除预测动作残差后真正下发的 TCP；
- 叠放推演始终使用理论终态，不能把补偿重复计算两次。

## 4. 正式数据与唯一来源

| 文件 | 作用 |
| --- | --- |
| `data/aubo_poses.json` | 任务卡、方块参考平面、托盘和通用目标四个位姿的唯一来源 |
| `data/task2_tuning.json` | 三个场景的曝光/增益及方块、托盘 HSV 调参结果 |
| `data/task2_offsets_v2.json` | 一次人工物理锚点、物块高度、像素旋转观测和动作补偿 |
| `resources/visionmaster_task2_calibration.xml` | 方块区和托盘区共用的唯一像素→平面矩阵 |
| `resources/visionmaster_task2_template.xml` | 生成候选 XML 的独立模板 |
| `resources/visionmaster_task2_calibration_binding.json` | 正式 XML 指纹和质量摘要 |
| `resources/visionmaster_task2_calibration_report.json` | 正式矩阵的完整观测报告 |
| `resources/calibration_history/` | 正式矩阵历史备份 |
| `output/` | 原图、mask、标注图、候选、失败报告和任务日志；可删除，不是正式状态源 |

候选只有经过质量门禁、人工检查标注图并明确输入 `yes` 后，才会备份旧版并覆盖正式文件。

## 5. 底层算法

### 5.1 三个拍照场景

任务卡、物块区和托盘区使用独立曝光/增益：任务卡只调曝光和增益；物块区检测九色；托盘区检测六色。三个调参工具都会先安全移动到对应拍照位，除非显式使用图片模式或 `--no-move`。

物块视觉以 HSV 多区间掩膜、开闭运算、轮廓面积和近方形门禁为基础。颜色重叠或低饱和时，联合使用 HSV 覆盖率、色相和 Lab 颜色距离做一对一分配；红色/粉色通过现场原型避免同一轮廓被双占。白板区域门禁失败时，`fallback` 会留下警告并退回全图识别，但不会跳过尺寸、形状、任务协议和坐标门禁。

托盘视觉沿用物块区“颜色分割 + HSV/Lab 联合判别”的经验，但把身份和中心解耦：HSV/Lab 负责六色身份，嵌套方框几何负责最终中心。调参预览、正式任务和九点自动标定共用这套中心实现：

1. 灰度、模糊、Canny 和闭运算提取方框；
2. 聚合同一托盘的多层边缘；
3. HSV/Lab 色块中心用于匹配颜色身份，并在方框漏检时提供补充候选；
4. 所有中心必须通过两行三列的间距、行列一致性和边界检查；
5. 槽位只按 `top/bottom + left/center/right` 空间拓扑建立，不按颜色身份建立标定对应。

### 5.2 九点公共平面矩阵

相机安装在末端。托盘保持不动，机械臂在托盘拍照位附近走 3×3 网格；当前相邻位移为 `8 mm`。每帧最多观察六个托盘中心，因此一次最多得到 54 组观测。

对同一个静止托盘中心 `i`，在相机位姿 `j` 的观测满足：

```text
H(pixel_ij) + camera_xy_j ≈ fixed_xy_i
```

`H` 是像素到机器人平面增量的 3×3 单应矩阵。拟合不需要六个托盘的绝对机器人坐标：同一托盘在九次拍照中的预测基座坐标应保持不变，程序据此最小化组内残差。

实现细节：

- 线性仿射解初始化，再用 `soft_l1` 鲁棒最小二乘优化透视项；
- 各托盘组按样本数加权，避免观测更多的托盘支配结果；
- 至少需要 3 个托盘区域、每区至少 5 张有效图，并覆盖上下两行和至少两列；
- 九个相机位姿各自至少保留 3 个有效中心；
- 有 4 个以上区域时，对每个托盘做一次“整组留出”交叉验证。

矩阵只描述尺度、轴向、旋转和轻微透视，不负责相机光心到吸盘 TCP 的绝对平移。

### 5.3 单一物理锚点

人工粗校只在物块区执行一次：记录物块像素中心 `p_anchor`，再把吸盘 TCP 人工对准该物块中心并读取真实 TCP。当前矩阵启用后，共享平移量按下式派生：

```text
shared_offset_xy = aligned_tcp_xy
                 - block_view_origin_xy
                 - H(p_anchor)
```

运行时两区共用该偏移：

```text
block_robot_xy = block_view_xy + H(block_pixel) + shared_offset_xy
tray_robot_xy  = tray_view_xy  + H(tray_pixel)  + shared_offset_xy
```

矩阵标定和物理锚点采集相互解耦：

| 流程 | 观测对象 | 产物 | 是否依赖另一流程 |
| --- | --- | --- | --- |
| 托盘九点矩阵 | 6 个托盘中心随相机 3×3 移动的像素变化 | 公共 XML、质量绑定和报告 | 不依赖物理锚点 |
| 物块人工锚点 | 一个物块像素中心与人工对准 TCP | `task2_offsets_v2.json` 原始物理观测 | 采集时不加载公共矩阵 |

两者可按任意顺序采集，正式任务启动时才组合。启用新矩阵后，只要相机、TCP 和方块参考拍照位没变，原始锚点可以用新矩阵重新派生毫米偏移；不要求重做人工粗校，但必须重新做落点验收。

### 5.4 物块高度、方向和旋转偏心

`data/aubo_poses.json` 中的方块拍照位是参考平面拍照位：

```text
实际方块拍照Z = 参考平面拍照Z + TASK2_BLOCK_HEIGHT_MM
托盘拍照Z     = aubo_poses中的托盘拍照Z
```

这样相机到方块顶面和托盘平面的有效距离保持一致。全部物块只有一个真实高度 `TASK2_BLOCK_HEIGHT_MM`；它同时用于方块拍照 Z 增量和叠放层高。`TASK2_BLOCK_PICK_Z` 与 `TASK2_TRAY_PLACE_Z` 是在该统一高度下实测的 TCP 值，不再按颜色修正。

正方形方向每 90° 等价。程序把边向量通过公共矩阵转换到机器人 XY，再把角度归一化到 `[-45°, 45°)`；`TASK2_ROTATION_DIRECTION` 可选择最短、全正或全负等价旋转。

吸盘未抓在真实中心时，物块原地旋转后像素中心会移动。标定工具保存旋转前后像素，运行时用当前矩阵转换中心残差，并拟合参考姿态下的二维偏心 `b`：

```text
center_residual(θ) ≈ (I - R(θ)) · b
```

矩阵更新后可以从原始像素重新派生毫米偏心，避免沿用旧矩阵下的派生值。

### 5.5 标称计划与动作补偿

坐标标定解决“这个像素理论上对应哪里”；释放滑动、柔性吸盘和位置相关残差属于执行误差，不能回写物理锚点。

动作补偿接口支持：

```text
predicted_residual = constant_bias
                   + spatial_matrix · (target_xy - reference_xy)
                   + (I - R(rotation)) · rotation_center_bias

command_xy = nominal_xy - predicted_residual
```

当前默认物理标定流程只做一次人工锚点和 1～2 次原地旋转。人工粗校完成后会先原子写入可独立使用的零补偿基线；后续旋转失败不会丢失人工结果。正式规划保留统一动作补偿接口，但不再执行容易误导的托盘自动验证流程。

## 6. 标定误差怎么理解

### 6.1 报告指标

| 指标 | 计算含义 | 能证明什么 | 不能证明什么 |
| --- | --- | --- | --- |
| `world_rms_mm` | `sqrt(mean(||H(pixel)-world||²))` | 训练样本在机器人平面的拟合一致性 | 吸盘最终落点精度 |
| `pixel_rms` | 用 `H⁻¹` 把世界点投回图像后的像素 RMS | 矩阵双向重投影一致性 | 毫米级机械重复性 |
| `region_relative_rms_mm` | 每个托盘在九次相机运动中的组内 RMS | 某槽位是否抖动或错配 | 未观测位置的绝对误差 |
| `leave_one_region_out_relative_rms_mm` | 留出一个完整托盘后再评估 | 矩阵跨区域泛化能力 | TCP、吸盘和释放误差 |
| `geometry_match_rms_px` | 后续帧几何中心相对首帧槽位的跟踪误差 | 托盘跟踪稳定性 | 公共矩阵毫米误差 |
| `rotation_center_fit_rms_mm` | 偏心模型对旋转中心残差的拟合 RMS | 固定偏心是否一致 | 放置释放的随机漂移 |

当前矩阵启用门禁：

```text
world RMS ≤ 1.0 mm
pixel RMS ≤ 10.0 px
```

这些是拒绝明显错误候选的上限，不是项目承诺的落点精度。当前正式值必须通过审计工具读取，不在 README 中硬编码。

### 6.2 共用矩阵前提

两区可以有不同 XY 拍照位，但相机安装、镜头、RX/RY 和观测平面关系不能改变。当前程序明确检查：

```text
有效拍摄高度差 ≤ 2.0 mm
RZ差 ≤ 0.02 rad
```

高度未知时应现场测量物块并设置 `TASK2_BLOCK_HEIGHT_MM`。高度误差会改变像素到毫米比例，且离图像中心越远通常越明显，不能靠固定 XY 偏移完全消除。

### 6.3 实际误差预算

| 来源 | 典型表现 | 处理方式 |
| --- | --- | --- |
| 曝光、HSV/Lab、轮廓中心 | 同一物体像素中心抖动、边缘漏检 | 分场景调曝光；检查原图、mask、标注图 |
| 矩阵拟合 | 位置越远误差方向/尺度越明显 | 检查 world/pixel RMS、整组留出 RMS、对应关系 |
| 高度/姿态不一致 | 中心附近尚可，边缘系统性偏差 | 测量高度，重建同有效 Z/RZ 的拍照位 |
| 物理锚点 | 全画面近似固定偏移 | 重新人工对准并确认 TCP 读取值 |
| 抓取旋转偏心 | 不旋转正常，旋转后中心画圆 | 增加正反角观测，检查偏心拟合 RMS |
| AUBO 到位重复性 | 同命令多次落点分散 | 检查速度、到位误差、负载和机械状态 |
| 吸盘压缩/释放滑动 | 坐标准确但实物落点随机漂移 | 调气路、泄压、慢抬；独立统计重复抓放 |

诊断顺序应为“图像中心 → 矩阵 → 高度/姿态 → 物理锚点 → 旋转偏心 → 抓放重复性”，不要把所有误差都塞进一个固定偏移。

## 7. 操作手册：如何食用

### 7.1 环境与开机准备

本项目使用 Windows、Python 3.10 和 AUBO/MVS 本地驱动：

```powershell
conda activate env_aubo
pip install -r requirements.txt
```

根据 `secrets_example.json` 配置本机 `secrets.json`。不要提交 API 密钥、机器人密码或 AI 盒子账号。

开始前确认：AUBO 和 Tool IO 正确；MVS/VisionMaster 没有占用相机；机械臂周围无人和障碍；安全 Z、TCP、速度、抓放 Z 已低速验证；已用游标卡尺测量本次物块高度。

### 7.2 新工位完整流程

以下命令默认可能移动机器人，应依次执行，不要并行。

#### 1）填写高度和基础配置

先在 `config.py` 核对 `TASK2_BLOCK_HEIGHT_MM`、`TASK2_BLOCK_PICK_Z`、`TASK2_TRAY_PLACE_Z` 和 `TASK2_ROTATION_DIRECTION`。完整可选项见第 8 节。

#### 2）记录四个正式点位

```powershell
python tools/setup/aubo_pose_recorder.py
```

依次记录 `TASK2_CARD_VIEW_POSE`、`TASK2_BLOCK_VIEW_POSE`、`TASK2_TRAY_VIEW_POSE`、`ROBOT_TARGET`。方块位记录参考平面拍照位，不要手工再抬高物块高度。

#### 3）分别调整三个场景

```powershell
# 默认：物块区九色
python tools/task2/workflow/task2_tuner.py

# 任务卡只调曝光/增益
python tools/task2/workflow/task2_card_capture_tuner.py

# 托盘区六色
python tools/task2/workflow/task2_tuner.py --scene tray
```

工具默认自动移动到对应位置。相机已经在位且不希望运动时用 `--no-move`；离线图片用 `--image <路径>`。

调色键：数字键切颜色，`[`/`]` 切同色 HSV 区间，`n` 新增，`x` 删除，`s` 显式保存，`v` 在完整性通过后保存人工确认。按 `q`、`ESC` 或关闭窗口直接退出且不保存，防止试调参数误写；正式运行只读取合并后的 `data/task2_tuning.json`。正式检测框统一显示为黄色，便于和白色托盘底图区分。

#### 4、5）采集矩阵与物理锚点

两个步骤没有先后依赖，可按现场方便顺序执行：

```powershell
# 托盘区自动走 3×3，生成公共矩阵候选
python tools/task2/workflow/task2_tray_auto_calibrate.py

# 物块区人工粗校，随后只做原地旋转像素采样
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py
```

矩阵工具自动走九点并保存原图/标注图；质量通过且输入 `yes` 后才启用。物理锚点工具只在物块区工作：人工对准后立即写入零补偿基线，随后默认做 1～2 次原地旋转；不进入托盘区、不做平移采样、不修改公共矩阵。

只保留人工粗校：

```powershell
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --manual-only
```

离线重算已有托盘报告：

```powershell
python tools/task2/workflow/task2_tray_auto_calibrate.py --from-report <报告路径>
```

#### 6）审计正式状态

```powershell
python tools/task2/workflow/task2_calibration_audit.py
```

必须确认矩阵结构和指纹有效、质量报告可追溯、两区有效高度/RZ 兼容、物理锚点能与当前矩阵组合。不得删除 binding、放宽阈值或把 `UNKNOWN` 当 `PASS`。

#### 7）分级验收

1. 静态图：九色、六色、中心和方向；
2. 设备单项：相机、机器人读取、Tool IO；
3. 低速单块吸取和释放；
4. 单托盘不旋转落点；
5. 正反旋转中心和方向；
6. 叠放高度；
7. 完整任务成功率、耗时和落点残差。

#### 8）运行正式任务

```powershell
python main.py
```

### 7.3 日常检查与重标定触发条件

日常至少确认相机/工具/工作台未被碰动、物块高度正确、三个场景曝光未串用、标定审计通过、单块低速吸放正常。

| 变化 | 至少需要重做 |
| --- | --- |
| 只改变光照、曝光或背景 | 对应 tuner + 静态图检查 |
| 更换物块高度 | 更新高度参数，检查有效拍摄距离、抓放 Z、叠放 |
| 改变方块参考拍照位 XY | 重新记录点位并重做物理锚点 |
| 改变相机高度、RZ、镜头或安装 | 点位、公共矩阵、组合审计和落点验收 |
| 改变 TCP、吸盘或工具安装 | 物理锚点、抓放 Z、旋转偏心、低速验收 |
| 只启用新矩阵且物理关系未变 | 原始锚点可重组；必须审计和重新落点验收 |
| 修改泄压、慢抬、速度或负载 | 单块与重复抓放验收，不混入坐标偏移 |

### 7.4 可复制运行命令

所有命令都在项目根目录执行。忘记参数时可直接查看帮助：

```powershell
python tools/task2/workflow/task2_tuner.py --help
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --help
```

#### 点位与 Tool IO

```powershell
# 用示教器依次移动到四个位姿，程序只读取当前TCP并写aubo_poses.json
python tools/setup/aubo_pose_recorder.py

# 控制末端泵/泄压阀，不移动机械臂；会设置Tool电压
python tools/setup/vacuum_io_test.py

# 外部已经提供Tool电压时，跳过公共电压设置
python tools/setup/vacuum_io_test.py --skip-voltage
```

`vacuum_io_test.py` 会真实切换 Tool IO。`--skip-voltage` 只跳过电压配置，不会把 IO 测试变成只读。

#### 物块/托盘 HSV 调参

```powershell
# 默认：自动移动到物块拍照位，调整物块九色
python tools/task2/workflow/task2_tuner.py

# 自动移动到托盘拍照位，调整托盘六色
python tools/task2/workflow/task2_tuner.py --scene tray

# 使用本地图片调物块；不连接相机、不移动机器人
python tools/task2/workflow/task2_tuner.py --image "E:\images\blocks.jpg"

# 使用本地图片调托盘
python tools/task2/workflow/task2_tuner.py --scene tray --image "E:\images\trays.jpg"

# 使用相机，但保持机器人当前位姿
python tools/task2/workflow/task2_tuner.py --scene block --no-move
python tools/task2/workflow/task2_tuner.py --scene tray --no-move

# --camera只是显式声明默认相机模式，通常不需要填写
python tools/task2/workflow/task2_tuner.py --camera --scene block
```

调参会更新 `data/task2_tuning.json`；图片模式仍允许保存参数，因此它不是纯只读命令。

#### 任务卡曝光调参

```powershell
# 自动移动到任务卡拍照位，只调曝光/增益
python tools/task2/workflow/task2_card_capture_tuner.py

# 相机已经在位，不移动机器人
python tools/task2/workflow/task2_card_capture_tuner.py --no-move

# 本地图片预览，仍可把滑条值保存到task2_tuning.json
python tools/task2/workflow/task2_card_capture_tuner.py --image "E:\images\task_card.jpg"
```

#### 托盘九点公共矩阵

```powershell
# 正式采集：连接机器人和相机，自动走九点，生成候选并等待yes启用
python tools/task2/workflow/task2_tray_auto_calibrate.py

# 指定另一份只读VisionMaster模板；仍属于实机九点流程
python tools/task2/workflow/task2_tray_auto_calibrate.py --template "E:\calibration\template.xml"

# 用已有报告离线重算；不连接机器人、不自动激活正式XML
python tools/task2/workflow/task2_tray_auto_calibrate.py --from-report "E:\reports\tray_calibration_report.json"
```

#### 单一物理锚点与旋转偏心

```powershell
# 推荐默认流程：人工粗校落盘后，按config中的quick/robust做原地旋转
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py

# 只建立人工物理锚点和零补偿基线，不执行自动旋转抓放
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --manual-only

# 白板边界失败立即停止
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --board-guard strict

# 白板边界失败时警告并退回HSV单目标审计；这是默认模式
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --board-guard fallback

# 完全关闭标定专用白板边界检查；背景必须足够干净
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --board-guard off

# 参数可以组合
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --manual-only --board-guard strict
```

脚本仍接受 `--compensation-model adaptive` 和 `--compensation-model constant`，用于保留同一 CLI/报告字段。当前收敛后的正式采集都只执行“人工锚点 + 原地旋转像素观测”，不会因为该参数恢复平移、托盘修正或托盘验证；日常直接使用无参数默认流程即可。

#### 审计、列出版本和恢复矩阵

```powershell
# 只读审计正式XML、质量、两区位姿条件和物理锚点链路
python tools/task2/workflow/task2_calibration_audit.py

# 只读列出当前正式矩阵和历史备份
python tools/task2/workflow/task2_calibration_versions.py

# 恢复指定候选/历史XML；检查质量、等待yes、备份当前正式版后才覆盖
python tools/task2/workflow/task2_calibration_versions.py --activate "E:\project_test\project_all\resources\calibration_history\example.xml"
```

恢复矩阵后必须再次运行审计和落点验收。物理关系未改变时原始锚点可与新矩阵重新组合，但不能把旧的毫米派生值当作验收结果。

#### 正式视觉诊断

```powershell
# 默认：物块场景，相机拍照并自动移动到物块拍照位
python tools/task2/diagnostics/task2_vision_test.py

# 明确选择三个场景
python tools/task2/diagnostics/task2_vision_test.py --scene card --camera
python tools/task2/diagnostics/task2_vision_test.py --scene block --camera
python tools/task2/diagnostics/task2_vision_test.py --scene tray --camera

# 相机拍照但不移动机器人，并弹窗查看原图/标注图
python tools/task2/diagnostics/task2_vision_test.py --scene tray --camera --no-move --show

# 对已有图片执行正式检测，不移动机器人
python tools/task2/diagnostics/task2_vision_test.py --scene block --image "E:\images\blocks.jpg" --show
python tools/task2/diagnostics/task2_vision_test.py --scene tray --image "E:\images\trays.jpg" --show
python tools/task2/diagnostics/task2_vision_test.py --scene card --image "E:\images\task_card.jpg"
```

`task2_vision_test.py` 保存诊断输出，不修改正式矩阵或物理锚点。该工具的任务卡场景只检查原图清晰度，不调用外部视觉模型；正式任务中的任务卡解析才需要网络、密钥并可能产生 API 成本。

#### 托盘六色人工确认

```powershell
# 自动移动到托盘拍照位、拍照、显示结果，输入yes后保存调试记录
python tools/task2/diagnostics/task2_tray_verify.py

# 相机拍照但保持机器人当前位姿
python tools/task2/diagnostics/task2_tray_verify.py --no-move-robot

# 使用已有图片，不移动机器人
python tools/task2/diagnostics/task2_tray_verify.py --image "E:\images\trays.jpg"

# 不弹出OpenCV窗口，仍在终端要求人工确认
python tools/task2/diagnostics/task2_tray_verify.py --image "E:\images\trays.jpg" --no-show
```

确认结果写入 `data/task2_verified_trays.json`，只供人工调试，正式任务不读取它。

#### 拍照高度对照

```powershell
# 方块区：用示教器手动切换Z，程序不发送运动命令
python tools/task2/diagnostics/task2_height_probe.py --scene block

# 托盘区：默认对比相对当前基准0/20/40 mm
python tools/task2/diagnostics/task2_height_probe.py --scene tray

# 自定义非负高度序列；第一项必须为0
python tools/task2/diagnostics/task2_height_probe.py --scene block --deltas 0 10 20 30

# 本轮固定曝光/增益，只用于对照，不写入config或tuning文件
python tools/task2/diagnostics/task2_height_probe.py --scene tray --deltas 0 15 30 --exposure-us 6000 --gain 2
```

高度对照要求连接 AUBO 以读取 TCP，但脚本不会发送机械臂运动命令；每个高度都由操作者使用示教器调整。

## 8. Ctrl+F 配置索引

下面列出现场最常改的 [config.py](config.py) 变量。推荐直接在本节复制变量名，用 `Ctrl+F` 跳到定义处；不要另建第二份配置文件。

### 8.1 执行和完整性

| 变量 | 可选值/单位 | 定义 |
| --- | --- | --- |
| `TASK2_EXECUTE_ROBOT` | `True` / `False` | 正式任务是否执行机器人；`False` 仍会拍照、识别并生成计划，但不移动机器人，也不是所有工具的急停 |
| `TASK2_REQUIRE_ALL_COLORS` | `True` / `False` | 是否要求画面完整九色/六色；任务卡引用的目标始终必须存在 |
| `TASK2_REQUIRE_OFFSET_FILE` | `True` / `False` | 是否强制存在正式 V2 物理标定；实机建议保持 `True` |
| `TASK2_EXPECTED_STACK_COUNT` | `None` / 整数 | `None` 按协议接受；整数时限制预期叠放数量 |
| `TASK2_DISABLED_BLOCK_COLORS` | 元组 | 临时禁用方块颜色；比赛正式流程通常保持空元组 |

### 8.2 语音、相机和离线图片

| 变量 | 可选值/单位 | 定义 |
| --- | --- | --- |
| `VOICE_BACKEND` | `"ai_box"` / `"local"` | 语音后端；AI 盒子失败不会静默切换 |
| `MVS_DEVICE_INDEX` | 非负整数 | 海康相机设备序号 |
| `MVS_TIMEOUT_MS` | ms | 单帧采集超时 |
| `MVS_EXPOSURE_TIME`、`MVS_GAIN` | 数值 | 全局默认；曝光 `<=0`、增益 `<0` 表示不主动设置 |
| `TASK2_CARD_EXPOSURE_TIME`、`TASK2_CARD_GAIN` | 数值 / `None` | 任务卡专用；`None` 沿用 MVS 默认 |
| `TASK2_BLOCK_EXPOSURE_TIME`、`TASK2_BLOCK_GAIN` | 数值 / `None` | 物块区专用 |
| `TASK2_TRAY_EXPOSURE_TIME`、`TASK2_TRAY_GAIN` | 数值 / `None` | 托盘区专用 |
| `TASK2_CARD_DEBUG_IMAGE` | 路径 / `None` | 用本地任务卡图替代实拍 |
| `TASK2_BLOCK_DEBUG_IMAGE` | 路径 / `None` | 用本地物块图替代实拍 |
| `TASK2_TRAY_DEBUG_IMAGE` | 路径 / `None` | 用本地托盘图替代实拍 |
| `TASK2_CARD_IMAGE_MAX_EDGE` | px | 上传任务卡前允许的最长边 |
| `TASK2_CARD_JPEG_QUALITY` | `0..100` | 任务卡 JPEG 质量 |

### 8.3 视觉模式与阈值

| 变量 | 可选值/单位 | 定义 |
| --- | --- | --- |
| `TASK2_BLOCK_BOARD_GUARD_MODE` | `"strict"` / `"fallback"` / `"off"` | 白板失败即停 / 警告后全图检测 / 不检测白板 |
| `TASK2_BLOCK_BOARD_REGION_SCALE` | `1.0`～`1.30`，默认 `1.10` | 将检测到的白板有效框从中心等比放大；`1.10`相当于每边约增加5%，补偿有高度物块的透视外扩，未经静态图验收不建议超过`1.15` |
| `TASK2_MIN_CONTOUR_AREA`、`TASK2_MAX_CONTOUR_AREA` | px² | 物块/托盘轮廓面积范围 |
| `TASK2_MORPH_KERNEL` | 正奇数优先 | HSV mask 开闭运算核尺寸 |
| `TASK2_COLOR_FALLBACK_ENABLED` | `True` / `False` | 是否启用 HSV 覆盖率 + 色相 + Lab 联合分配 |
| `TASK2_COLOR_MASK_WEIGHT` | 浮点 | 候选像素落入目标 HSV 区间的权重 |
| `TASK2_COLOR_HUE_WEIGHT` | 浮点 | HSV 色相距离权重 |
| `TASK2_COLOR_LAB_WEIGHT` | 浮点 | Lab 感知颜色距离权重 |
| `TASK2_COLOR_FALLBACK_MIN_S`、`TASK2_COLOR_FALLBACK_MIN_V` | `0..255` | 联合候选宽掩膜最低饱和度/亮度 |
| `TASK2_COLOR_MAX_ASPECT_RATIO` | 比值 | 候选最大长宽比 |
| `TASK2_COLOR_MIN_RECT_FILL` | `0..1` | 轮廓面积/最小外接矩形面积下限 |
| `TASK2_COLOR_MAX_ASSIGNMENT_COST` | 浮点 | 颜色全局分配最大代价；不要为通过而盲目放宽 |

HSV 正式值优先来自 `data/task2_tuning.json`，它会按颜色合并覆盖 `TASK2_BLOCK_HSV_RANGES` 和 `TASK2_TRAY_HSV_RANGES`。建议通过 tuner 修改，不直接手写大量范围。

### 8.4 矩阵、物理锚点与高度

| 变量 | 可选值/单位 | 定义 |
| --- | --- | --- |
| `TASK2_TRAY_CALIBRATION_STEP_MM` | mm | 托盘 3×3 相邻相机位移 |
| `TASK2_CALIBRATION_WORLD_SCALE_MM` | 倍率 | XML 世界坐标到毫米的倍率；当前矩阵直接输出 mm 时为 `1.0` |
| `TASK2_SHARED_CALIBRATION_MAX_Z_DIFF_MM` | mm | 两区有效拍摄高度差上限 |
| `TASK2_SHARED_CALIBRATION_MAX_RZ_DIFF_RAD` | rad | 两区拍照 RZ 差上限 |
| `TASK2_OFFSET_CALIBRATION_PROFILE` | `"quick"` / `"robust"` | 原地旋转 1 次 / 正反旋转 2 次；写入相同正式接口 |
| `TASK2_BLOCK_HEIGHT_MM` | mm | 全部物块唯一实测高度；同时决定方块拍照 Z 增量和叠放层高 |
| `TASK2_BLOCK_PICK_Z` | mm | 当前统一高度物块的吸取 TCP Z |
| `TASK2_TRAY_PLACE_Z` | mm | 当前统一高度物块的放置 TCP Z；名称按动作语义，不按区域语义 |

### 8.5 旋转、路径和到位

| 变量 | 可选值/单位 | 定义 |
| --- | --- | --- |
| `TASK2_ROTATION_ENABLED` | `True` / `False` | 是否执行方向对准 |
| `TASK2_ROTATION_DIRECTION` | `"shortest"` / `"positive"` / `"negative"` | 最短等价角 / 只正转 / 只负转 |
| `TASK2_ROTATE_AT_CLEARANCE` | `True` / `False` | 是否在净空高度完成旋转 |
| `TASK2_LIFT_DISTANCE_MM` | mm | 抓取后的默认抬升距离 |
| `TASK2_SETTLE_SECONDS` | s | 到拍照位后等待稳定时间 |
| `ROBOT_SAFE_Z` | mm | 跨区安全高度；必须现场确认可达且无碰撞 |
| `ROBOT_SPEED`、`ROBOT_ACCELERATION` | SDK 比例值 | 常规运动速度/加速度 |
| `ROBOT_WAIT_TIMEOUT` | s | 等待运动完成超时 |
| `ROBOT_POSITION_TOLERANCE` | mm | 到位位置容差 |
| `ROBOT_ORIENTATION_TOLERANCE` | deg | 到位姿态容差 |

### 8.6 吸盘、泄压与慢抬

| 变量 | 可选值/单位 | 定义 |
| --- | --- | --- |
| `ROBOT_VACUUM_ENABLED` | `True` / `False` | 是否实际控制吸盘 |
| `TOOL_IO_VOLTAGE` | V | 末端工具电压，当前硬件通常为 `12` |
| `TOOL_IO_CONFIGURE_VOLTAGE` | `True` / `False` | 是否由程序设置公共 Tool 电压 |
| `TOOL_IO_VENT_INDEX`、`TOOL_IO_PUMP_INDEX` | IO 序号 | 泄压阀和气泵对应的末端 IO |
| `TOOL_IO_VENT_OPEN_LEVEL`、`TOOL_IO_PUMP_ON_LEVEL` | `True` / `False` | 对应硬件的有效电平 |
| `TOOL_IO_SUCTION_WAIT_SEC` | s | 开泵后等待吸附时间 |
| `TOOL_IO_VENT_BEFORE_PUMP_OFF_SEC` | s | 释放时开阀与泵运行重叠时间 |
| `TOOL_IO_RELEASE_WAIT_SEC` | s | 停泵后继续泄压等待时间 |
| `TASK2_RELEASE_SLOW_LIFT_MM` | mm | 保持泄压时低速竖直脱离距离 |
| `TASK2_RELEASE_SLOW_SPEED`、`TASK2_RELEASE_SLOW_ACCELERATION` | SDK 比例值 | 释放慢抬速度/加速度 |

修改 Tool IO 电压、电平、泄压时序或慢抬参数后，必须重新做单块受控验收。

## 9. 运行输出与排障

```text
output/task2/task2_<时间>.json
output/task2/task2_error_<时间>.log
output/task2/*_detected.jpg
output/task2/*_mask.png
output/task2/tray_auto_calibration/<时间>/
output/task2/closed_loop_offset_calibration/<时间>/
```

| 现象 | 优先检查 |
| --- | --- |
| 托盘只识别 4～5 个 | 托盘曝光、原图边缘、颜色标注图；颜色可补几何漏检，但仍要求 2×3 拓扑 |
| 方块识别到白板外 | 白板门禁、轮廓面积、标注图、HSV/Lab 分配 |
| 中心处准确、边缘偏差大 | 物块高度、有效 Z/RZ、矩阵留出 RMS |
| 全画面固定偏移 | 人工物理锚点、TCP、拍照位 XY |
| 旋转后偏、平移不偏 | 旋转像素观测、角度方向、偏心拟合 |
| 同一命令落点分散 | AUBO 重复定位、吸盘压缩、释放滑动、气路、速度 |
| 标定质量 `UNKNOWN` | 正式 binding/report 和 XML 指纹；不要依赖 `output/` 拼状态 |
| 相机打开失败 | 关闭 MVS、VisionMaster 或其他占用进程 |

不要把一次坏落点直接写成新的固定偏移；先确定误差属于视觉、矩阵、高度、物理锚点还是执行重复性。

## 10. 离线验证与实机边界

代码变更后运行：

```powershell
python -m compileall -q main.py task modules drivers contracts runtime tools test
python -m unittest discover -s test -q
```

输出 `OK` 只证明静态接口和离线算法回归通过，不证明相机曝光、AI 盒子/API、机器人路径、Tool IO、吸盘或当前工位精度。正式验收必须按“离线 → 静态图 → 设备单项 → 受控运动 → 完整任务”逐级进行。
