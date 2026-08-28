# 答辩 PPT 填稿（17 页 · V1）

> 骨架已冻结为 5 章 17 页。本文件是**逐页填稿**：每页给出可照抄的要点 + 实际引用位置，照着去对应文件拿资料即可。
> 引用统一为「模块 · 位置」，括号内是相对路径 + 行号，可直接点击跳转。

---

# 01 赛题与总体方案

## P1 · 封面
- 标题：具身智能精密装配赛
- 副标题：语音交互 · 视觉识别 · 机械臂抓放 一体化方案
- 团队名 / 成员
- 本页不讲技术。

---

## P2 · 赛题任务与需求分析
**内容**
- 任务一：获取任务卡一 → 识别场景内容 → 语音播报识别结果
- 任务二：获取并理解任务卡二 → 识别六色方块与托盘 → 按指令关系完成六次抓放
- 抽象出系统需求：**人机交互｜环境感知｜任务理解｜空间定位｜运动执行**

**📍 引用**
- 两条任务链定义：[docs/项目流程说明.md:7-12](docs/项目流程说明.md#L7-L12)、[README.md:5-8](README.md#L5-L8)

---

## P3 · 总体解决方案
**内容**
- 一条概念链：
  `语音交互 → 任务理解 → 视觉感知 → 空间定位 → 运动执行 → 结果反馈`
- 底部对应真实技术：
  `SenseVoice / Qwen / MVS+OpenCV / VisionMaster / AUBO / pyttsx3 TTS`
- 本页不讲软件架构、不讲函数、不讲模块划分，只讲"准备怎么把机器人从接收任务连接到完成操作"。

**📍 引用**
- 技术栈一览：[README.md:196-203](README.md#L196-L203)

---

# 02 系统工程设计

## P4 · 业务逻辑与流程编排
**内容**
- 真实入口结构：
  ```
  main.py（系统入口 + 语音唤醒 + 任务派发）
     ↓ 派发
  任务一 task1.py    任务二 task2.py
  ```
- `main.py` 只做「唤醒 → 识别任务提示词 → 派发」，不含任何任务逻辑。
- `task1.py` / `task2.py` 各自编排一条业务流程。
- 核心句：**业务流程描述「做什么、以什么顺序做」，不直接处理底层硬件实现。**

**📍 引用**
- 入口职责说明：[main.py:1-8](main.py#L1-L8)
- 初始化四个能力实例、派发两任务：[main.py:26-66](main.py#L26-L66)
- 「不直接碰底层库」原文：[task/task1.py:3-6](task/task1.py#L3-L6)

---

## P5 · 功能分析与模块划分
**内容**
- 从流程反推所需能力，拆出五个模块（都在 `modules/` 目录下）：

| 能力 | 模块 | 对外接口（真实） |
|------|------|------------------|
| 语音 | voice.py | `wake() / listen() / speak() / is_exit()` |
| 视觉采集 | vision.py | `capture()` |
| 任务理解 | llm.py | `identify_image() / parse_task2_card()` |
| 视觉处理 | task2_vision.py | `ColorObjectDetector.detect() / CoordinateTransformer.pixel_to_robot()` |
| 机器人控制 | robot.py | `move_to() / move_to_safe() / get_current_pose() / set_suction() / pick_and_place()` |

- 核心句：**模块按独立能力划分，而不是按任务划分** —— 所以同一个 `robot.py` / `vision.py` 被任务一、任务二共同调用。

**📍 引用**
- 目录结构与模块职责表：[docs/项目流程说明.md:18-55](docs/项目流程说明.md#L18-L55)
- 五个能力实例同时传给两个任务：[main.py:26-30](main.py#L26-L30)

---

## P6 · 接口设计与分层封装
**内容**
- 设计方法：定义职责 → 定义输入 → 定义输出 → 封装接口。
- 三个真实接口示例：
  ```
  Vision.capture()         输入拍摄配置 → 输出图像路径
  LLM.parse_task2_card()   输入任务卡图像 → 输出 6 步结构化 JSON
  Robot.move_to()          输入目标位姿 → 返回是否到位(bool)
  ```
- `robot.py` 的层次展示：
  ```
  基础能力  move_to() / get_current_pose() / set_suction()
        ↓ 组合
  复合能力  move_to_safe() / pick_and_place()
  ```
- 核心句：**基础能力独立封装，复杂能力通过基础能力组合实现。**

**📍 引用**
- `Vision.capture()` 三级兜底与返回值：[modules/vision.py:210-232](modules/vision.py#L210-L232)
- `LLM.parse_task2_card()`：[modules/llm.py:155-207](modules/llm.py#L155-L207)
- `Robot.move_to()` / `move_to_safe()` / `pick_and_place()`：[modules/robot.py:80-111](modules/robot.py#L80-L111)、[modules/robot.py:113-145](modules/robot.py#L113-L145)、[modules/robot.py:303-322](modules/robot.py#L303-L322)

---

## P7 · 底层实现与技术选型
**内容**

| 能力 | 实现 |
|------|------|
| 语音识别 | FunASR / SenseVoiceSmall（本地 CPU） |
| 语音播报 | pyttsx3 |
| 图像采集 | 海康 MVS SDK |
| 图像处理 | OpenCV |
| 任务理解 | DashScope Qwen-VL（qwen3-vl-flash） |
| 标定 | VisionMaster |
| 机器人控制 | pyaubo-sdk（AUBO ARCS） |

- 逻辑句：**不是为了显得高级才用这些技术，而是为了实现前面定义好的接口。**

**📍 引用**
- 模型名 `qwen3-vl-flash`：[config.py:142](config.py#L142)
- AUBO SDK 引用：[modules/robot.py:14-15](modules/robot.py#L14-L15)
- 语音模型目录：[config.py:61](config.py#L61)

---

## P8 · 系统软件架构
**内容**
- 四层架构图（总结页，不再重讲前面）：
  ```
          业务流程层   main / task1 / task2
               ↓
          能力模块层   voice / vision / llm / task2_vision / robot
               ↓
          基础/复合能力  move_to / move_to_safe / pick_and_place ...
               ↓
          底层实现层   ASR / MVS / OpenCV / Qwen / AUBO / TTS
  ```
- 补充贯穿全局的 `config.py`（集中配置）与 `data/`（标定数据）、`output/`（运行时产物）。
- 收尾句：**形成业务逻辑与底层实现解耦、能力模块可独立调用的分层软件架构。**

**📍 引用**
- 分离式架构说明：[docs/项目流程说明.md:14-16](docs/项目流程说明.md#L14-L16)

---

# 03 核心技术实现

## P9 · 大模型任务理解与可信输出
**页面主标题建议**
- **从“能看懂”到“可执行”：受约束的多模态任务解析**

**页面内容（建议做成一条横向链路）**
```
任务卡高清图像
   → 多模态大模型直接理解
   → 提示词约束六步JSON
   → 程序二次逻辑校验
   → 生成确定性装配计划
```

**创新点 1｜OCR-free 多模态直读**
- 任务卡高清图像直接交给 Qwen-VL 理解，不再经过「图像预处理 → OCR文字 → 本地模型理解」的串联流程。
- 减少中间文字识别误差和模块数量；任务二保留最长边 2560、JPEG 95，优先保证小字可读性。

**创新点 2｜提示词工程约束输出接口**
- Prompt 明确限定：只返回 JSON 数组、恰好 6 项、固定字段、颜色来自六色集合。
- `temperature=0` 降低同一任务卡多次调用时的随机性。
- 大模型输出不作为自然语言直接执行，而是作为程序可解析的结构化接口。

**创新点 3｜模型输出的二次确定性校验**
- 语法校验：必须是合法 JSON，且顶层必须为 6 项数组。
- 字段校验：每项必须包含合法的 `block_color` 与 `tray_color`。
- 业务校验：六步必须完整覆盖六种方块颜色和六种托盘颜色。
- 任一条件不满足即停止任务，不把不完整结果传给机械臂。

**创新点 4｜不确定语义与确定执行解耦**
- 大模型只回答「什么颜色方块放到什么颜色托盘」，不生成机器人坐标、运动轨迹或 IO 指令。
- 坐标由 OpenCV、九点标定和偏移标定确定；运动由固定程序调用 AUBO SDK 执行。
- 原始模型返回与校验后的步骤同时写入日志，支持现场复核和故障追踪。

**页底核心句**
- **大模型负责理解开放语义，确定性程序负责验证边界与控制执行。**

**答辩口述（约 25 秒）**
- “我们的创新点不是让大模型直接控制机械臂，而是把它作为受约束的任务语义解析器。任务卡图像由多模态模型直接读取，提示词限定六步 JSON；程序再检查 JSON 格式、颜色合法性和六色覆盖关系。只有通过二次校验的数据才会进入视觉定位和机械臂执行，从而兼顾大模型的理解能力与机器人系统的确定性。”

**📍 引用**
- 任务一 prompt：[config.py](config.py)
- `identify_image()` 与图片压缩：[modules/llm.py:96-153](modules/llm.py#L96-L153)
- `parse_task2_card()` 温度 0 与强校验：[modules/llm.py:167](modules/llm.py#L167)、[modules/llm.py:190-201](modules/llm.py#L190-L201)
- LLM 原始返回留痕：[modules/llm.py:176-183](modules/llm.py#L176-L183)
- 校验后步骤留痕：[modules/llm.py:202-206](modules/llm.py#L202-L206)
- 任务二 prompt 与上传分辨率：[config.py:230-234](config.py#L230-L234)、[config.py:217-218](config.py#L217-L218)

---

## P10 · 六色目标视觉识别
**内容**
- 基础路径：图像 → HSV 分割 → 形态学开闭 → `findContours` → `minAreaRect` → 中心坐标。
- 真实问题：方块与托盘颜色表现不同 → **方块/托盘分区 HSV 配置**（方块紫 H≈120~145，托盘印刷紫 H≈165~179）。
- 红色环回端：方块红拆两段区间 `(0,80,25)-(8,..)` + `(175,..)-(179,..)`。
- fallback 兜底：
  1. 宽掩膜（S≥30、V≥20）找候选；
  2. 长宽比≤1.8、矩形填充≥0.55 过滤杂物；
  3. 三证据加权 **mask 覆盖率 0.45 + 色相距离 0.35 + Lab 距离 0.20** 联合评分；
  4. 六色全局一对一分配（排除台外杂物）。
- `validate_six_colors()` 校验六色齐全。

**📍 引用**
- HSV 分割 + minAreaRect：[modules/task2_vision.py:156-177](modules/task2_vision.py#L156-L177)
- 方块/托盘分区 HSV：[config.py:184-192](config.py#L184-L192)
- 联合评分权重：[config.py:197-202](config.py#L197-L202)
- fallback 全局分配：[modules/task2_vision.py:200-290](modules/task2_vision.py#L200-L290)
- 六色校验：[modules/task2_vision.py:293-297](modules/task2_vision.py#L293-L297)

---

## P11 · 视觉—机器人坐标转换
**内容**
- 两步走：
  1. **九点标定**：读 VisionMaster `CalibMatrix`（3×3 单应），像素 → 标定平面世界坐标 mm（注意 `TASK2_CALIBRATION_WORLD_SCALE_MM=10.0`，VM 世界坐标是 cm 要乘 10）；
  2. **偏移补偿**：示教器对准参考色读真实 TCP，与预测值做差得 XY 偏移 + 抓放 Z，写入 `task2_offsets_v2.json`。
- 空间流程：像素 `(u,v)` → 九点标定 → 世界坐标 `(x,y)` → 基座标定原点 → 实机 XY Offset → 抓放 Z + 姿态 → AUBO 基座目标位姿。
- 最终公式：`机器人位姿 = 标定原点 + 世界坐标(mm) + XY偏移(mm) + 目标Z(mm) + 姿态`。
- 关键约束：方块/托盘拍照点在基座坐标下 **Z、rz 必须一致**，否则两块区域无法共用同一标定矩阵。
- 不要写"±0.1mm"这类未验证精度。

**📍 引用**
- `pixel_to_world()` / `pixel_to_robot()`：[modules/task2_vision.py:108-131](modules/task2_vision.py#L108-L131)
- 标定矩阵读取：[modules/task2_vision.py:91-99](modules/task2_vision.py#L91-L99)
- 标定原点/XY 偏移/抓放 Z 配置：[config.py:225-231](config.py#L225-L231)
- 九点标定约束与抓放 Z 说明：[docs/项目流程说明.md:121](docs/项目流程说明.md#L121)、[docs/项目流程说明.md:132](docs/项目流程说明.md#L132)

---

## P12 · 机械臂抓放与末端执行
**内容**
- 运动：`moveLine` 直线运动 + `_wait_until_arrives` 到位轮询（位置容差 2mm / 姿态 1° / 超时 120s）。
- 抓放序列：上方接近 → 下降 → 吸取 → 抬升 80mm → 平移 → 下降 → 释放 → 抬升。
- 吸盘：**末端 Tool IO**（非控制柜 Standard DO，已实机验证）、**12V**、IO0 泄压阀 / IO1 真空泵。
  - 吸取 = 泄压阀关 + 真空泵开；释放 = 泵停 + 泄压阀短暂开启后关闭。
  - 写 IO 后回读校验。
- 安全运动 `move_to_safe()`：升安全 Z 500mm → 平移转 → 下降，三阶段任一失败即终止。
- 连接失败自动降级「无臂模式」。

**📍 引用**
- `move_to` / 到位轮询：[modules/robot.py:80-111](modules/robot.py#L80-L111)、[modules/robot.py:147-167](modules/robot.py#L147-L167)
- `pick_and_place` 动作序列：[modules/robot.py:303-322](modules/robot.py#L303-L322)
- 吸盘 IO 配置与回读：[modules/robot.py:204-293](modules/robot.py#L204-L293)
- IO 电压/通道/容差：[config.py:264-287](config.py#L264-L287)
- 无臂模式：[modules/robot.py:44-48](modules/robot.py#L44-L48)
- 吸盘实机说明：[docs/项目流程说明.md:148-157](docs/项目流程说明.md#L148-L157)

---

# 04 系统集成与现场调试

## P13 · 任务集成与完整运行链路
**内容**
- 任务一完整链：`move_to_safe` → `capture` → `identify_image` → `speak`
- 任务二完整链：`parse_task2_card` → `detect`（方块/托盘）→ `_build_plan` → 六步 `pick_and_place` 循环
- 本页意义：展示前面设计的独立能力如何被组合成两条完整业务链。

**📍 引用**
- 任务一流程：[task/task1.py:43-63](task/task1.py#L43-L63)
- 任务二流程：[task/task2.py:67-121](task/task2.py#L67-L121)
- 装配计划构建：[task/task2.py:42-45](task/task2.py#L42-L45)

---

## P14 · 现场标定与调试工具链
**内容**
- 三个标定/调参工具：
  - `task2_tuner.py`：HSV/曝光/增益 滑条调参 → `task2_tuning.json`
  - `task2_offset_calibrate.py`：XY 偏移 + 抓放 Z → `task2_offsets_v2.json`
  - `task2_tray_verify.py`：托盘坐标人工验收 → `task2_verified_trays.json`
- 两个独立测试入口：`task2_vision_test.py`（视觉离线）、`vacuum_io_test.py`（吸盘 Tool IO 单测）。
- 无臂模式（机器人连接失败自动降级，不影响视觉联调）。
- 核心句：**针对比赛现场建立独立调试/标定/验证工具，而不是所有问题都靠改主程序解决。**

**📍 引用**
- 工具清单与产物：[docs/项目流程说明.md:31-39](docs/项目流程说明.md#L31-L39)、[README.md:123-127](README.md#L123-L127)
- 无臂模式：[modules/robot.py:44-48](modules/robot.py#L44-L48)

---

## P15 · 系统可靠性与排障设计
**内容**
- LLM → JSON 强校验（恰好 6 步 + 六色覆盖）
- 视觉 → 六色完整性检查 + 联合评分 fallback
- 机器人 → 到位轮询（2mm/1°）+ IO 写后回读 + 抓放每步 `robot_status` 留痕（running/completed/failed）
- 安全开关 → `TASK2_REQUIRE_OFFSET_FILE`（无偏移文件禁止真抓放）、`TASK2_EXECUTE_ROBOT`（只测视觉不动臂）
- 配置 → 全部集中 `config.py`
- 记录 → 原图/mask/标注图/JSON/LLM 原文/error log 统一落 `output/`
- 标题用「系统可靠性设计」。

**📍 引用**
- 抓放留痕 `robot_status`：[task/task2.py:111-116](task/task2.py#L111-L116)
- 安全开关：[config.py:210-212](config.py#L210-L212)
- 运行时产物目录：[docs/项目流程说明.md:190](docs/项目流程说明.md#L190)、[README.md:164-193](README.md#L164-L193)

---

# 05 成果展示与总结展望

## P16 · 系统成果展示
**内容**
- 少字多实物，可直接取用 `output/` 里已有的真实素材：
  - `blocks_detected.jpg` / `trays_detected.jpg`（检测标注图）
  - 六色 `*_mask.png`（HSV 分割掩膜）
  - `diagnostics/` 下 `block_detected.jpg` / `tray_detected.jpg`（视觉诊断图）
  - `offset_calibration/` 下标定中间图
- 文字只说已验证事实：完成语音交互、视觉识别、任务解析、坐标转换与机械臂抓放的完整链路。
- 不写未测试的"99.5%"。

**📍 引用**
- 实际产出物位置：[output/](output/)、[output/task2/](output/task2/)、[output/task2/diagnostics/](output/task2/diagnostics/)、[output/task2/offset_calibration/](output/task2/offset_calibration/)

---

## P17 · 总结与展望
**内容**
- 左「当前成果」：
  - 完整闭环：语音 → 感知 → 理解 → 定位 → 执行 → 反馈
  - 模块化工程：业务流程与能力模块分离
  - 现场工具链：调参 → 标定 → 验证 → 排障
- 右「后续方向」（明确标"未来工作"）：
  - 视觉反馈闭环 / 参数自动适应 / 更复杂环境路径规划 / 力觉柔顺操作 / 本地多模态模型 / 更复杂任务泛化

---

## 附：骨架需注意的细节（写 PPT 时核对）
1. 旧提纲是 13 页（[docs/PPT大纲.md](docs/PPT大纲.md)），非 22 页；13 → 17 基本是拆分 + 重排。
2. 五个能力模块都在 `modules/` 子目录下，另有贯穿全局的 `config.py`。
3. 任务理解写死 `qwen3-vl-flash`（[config.py:142](config.py#L142)）。
4. P9 补 `temperature=0` + 图片压缩（[config.py:146-147](config.py#L146-L147)、[config.py:217](config.py#L217)）。
5. 吸盘电压 12V 非 24V（[config.py:266](config.py#L266)）。
6. 方块抓取 Z 默认 28mm、`TASK2_MANUAL_Z_OVERRIDE=True` 手动覆盖（[config.py:230-234](config.py#L230-L234)）。
