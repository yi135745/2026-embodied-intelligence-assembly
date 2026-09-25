# AUBO 具身智能精密装配 · 国赛工程

本工程由已完成省赛的 AUBO 项目迭代而来，国赛版本增加九色物块、六色托盘、方向对准、叠放、AI 盒子语音、公共平面标定和抓放动作补偿。

当前代码提供离线回归。离线测试不代表相机、吸盘、机械臂路径或现场精度已经验收；更换机器人、TCP、相机高度、工位或作业板后必须重新走现场流程。

## 1. 快速入口

项目根目录执行：

```powershell
conda activate env_aubo
python main.py
```

正式任务默认连接 AI 盒子、海康相机和 AUBO，并执行机器人动作。首次接入新工位不要直接运行整机任务，先完成第5节的现场流程。

纯离线回归：

```powershell
python -m compileall -q main.py task modules drivers contracts runtime tools test
python -m unittest discover -s test -q
```

通过标准：测试输出 `OK`；具体数量以当次运行结果为准。

## 2. 系统结构

```text
main.py
  ├─ 语音唤醒与任务派发
  ├─ task/task1.py：任务一编排
  └─ task/task2.py：任务二拍照、完整计划预检、逐步执行与日志

modules/
  ├─ voice.py / camera.py / interpreter.py / robot.py：四个平行设备能力
  ├─ task2_perception/：颜色、形状、中心、方向和坐标换算
  ├─ task2_planning.py：协议校验、标称规划和命令补偿
  └─ task2_calibration/：拟合、质量、版本和XML，正式任务不导入

drivers/：AUBO、MVS、DashScope、AI盒子底层适配
contracts/：平行功能库共享的纯数据与数值契约
runtime/：唯一正式现场位姿、偏移、绑定和补偿状态加载

tools/
  ├─ setup/：点位和通用硬件设置
  ├─ task2/workflow/：正式调参、标定、偏移、审计和恢复
  └─ task2/diagnostics/：图片、相机、高度等诊断
```

详细工具索引见 [tools/README.md](tools/README.md)。

### 2.1 架构设计原则

- `main.py` 只管理生命周期、语音派发和整轮完成状态；不实现视觉、坐标或机械臂细节。
- `task/` 负责编排；`modules/` 提供彼此平行的功能库；`drivers/` 封装厂商 SDK/HTTP；`tools/` 承载现场调试与标定流程。
- LLM只提取任务语义，不能直接生成机械臂坐标或绕过确定性校验。
- 相机采集、颜色识别、协议校验、标称规划、动作补偿和机器人执行彼此分层，便于分别使用静态图、纯数学和Fake设备测试。
- 功能库之间不横向调用，只共同依赖 `contracts/`；由 `task/` 或 `tools/` 负责组合。
- 位姿、公共矩阵、XY锚点和调参数据各有唯一正式来源，禁止在脚本或配置中维护平行副本。
- 任务二先预检整份计划，再逐步执行；只有真实抓放成功才提交状态。
- 所有TCP位姿在业务层统一使用 `[x,y,z,rx,ry,rz]`，XYZ为毫米、姿态为弧度；只有AUBO SDK边界转换成米。

## 3. 任务二如何工作

### 3.1 任务卡协议

模型只负责把任务卡提取成动作：

```json
{
  "step": 1,
  "source_color": "红色",
  "target_type": "tray",
  "target_color": "黄色"
}
```

确定性程序负责校验：

- 步号连续、来源颜色不重复；
- 前六步为六色物块分别放到六色托盘；
- 后续1～3步只允许青、粉、棕叠放；
- 叠放目标必须已放置且顶面未被占用；
- 模型输出不能直接绕过坐标、方向、高度和安全检查。

### 3.2 三次拍照，一次完整规划

正式流程依次拍摄任务卡、物块区和托盘区。程序在第一次抓放前生成并检查整份计划；只有某一步真实抓放成功，才把该物块提交到 `placed_block_map`。

当前不是运行中视觉闭环：正式任务不会在每次放置后复拍，也不会依据实物落点自动重试。

### 3.3 坐标、方向和高度

稳定坐标模型：

```text
公共单应矩阵：pixel -> 平面 world delta（mm）
robot XY = 分区拍照位XY + world delta + 分区XY offset
robot Z  = config中的抓取/放置参考Z + 物块高度差
robot姿态 = 当前拍照位姿态 + 正方形等价旋转
```

方块区与托盘区在相机 Z、RZ相同的前提下共用唯一矩阵：

```text
resources/visionmaster_task2_calibration.xml
```

两区绝对XY锚点、矩阵指纹和动作补偿共同保存在：

```text
data/task2_offsets_v2.json
```

`block_xy_offset`、`tray_xy_offset` 只表达坐标锚点；抓取偏心、旋转偏心、释放滑动等进入可选的 `motion_compensation`，不再反向污染坐标偏移。

规划中：

- `robot_pose` 是物块理论终态；
- `command_robot_pose` 是实际下发的补偿后 TCP；
- 叠放推演始终使用理论终态，避免重复补偿。

## 4. 正式数据与唯一来源

| 文件 | 作用 |
| --- | --- |
| `data/aubo_poses.json` | 四个正式点位唯一来源，XYZ=mm、姿态=rad |
| `data/task2_tuning.json` | 当前相机和光照下的HSV、曝光、增益 |
| `data/task2_offsets_v2.json` | 两区XY锚点、矩阵绑定和动作补偿 |
| `resources/visionmaster_task2_calibration.xml` | 方块/托盘唯一公共平面矩阵 |
| `resources/visionmaster_task2_template.xml` | 生成候选XML的独立结构模板 |
| `resources/visionmaster_task2_calibration_binding.json` | 正式矩阵指纹和质量摘要 |
| `resources/visionmaster_task2_calibration_report.json` | 正式矩阵完整观测报告 |
| `resources/calibration_history/` | 正式矩阵历史备份 |
| `output/` | 图片、掩膜、报告、候选和运行日志；可清理，不是正式状态源 |

启用新公共矩阵后，旧偏移失效是正常行为，必须重新运行闭环偏移工具。不要通过删除绑定或复制旧指纹绕过检查。

## 5. 新工位正式调试流程

依次运行，不要并行占用相机或机器人。

### 5.1 记录点位

```powershell
python tools/setup/aubo_pose_recorder.py
```

记录任务卡、物块、托盘拍照位及通用目标位。抓取Z和放置Z仍在 `config.py` 中维护。

### 5.2 调整物块和托盘视觉

```powershell
python tools/task2/workflow/task2_tuner.py
python tools/task2/workflow/task2_tuner.py --scene tray
```

默认自动移动到对应拍照位。图片模式可使用 `--image`；相机已经在位且不希望运动时使用 `--no-move`。保存后仍需运行视觉诊断检查联合识别结果：

```powershell
python tools/task2/diagnostics/task2_vision_test.py
python tools/task2/diagnostics/task2_vision_test.py --scene tray
```

### 5.3 生成唯一公共矩阵

```powershell
python tools/task2/workflow/task2_tray_auto_calibrate.py
```

托盘区自动走3×3相机位置，每帧最多采集六个托盘中心，一次生成最多54组对应点。候选通过点数、矩阵结构、world RMS和pixel RMS后，人工核对标注图并输入 `yes` 才会备份旧版并启用。

方块和托盘拍照位允许XY不同，但要求：

```text
Z差 ≤ 2 mm
RZ差 ≤ 0.02 rad
```

### 5.4 建立双区锚点和动作补偿

```powershell
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py
```

程序先要求依次人工粗对准物块区和托盘区，随后自动完成平移、旋转、跨区放置和独立验证。

每个人工阶段都会立即落盘。需要跳过高级自动采样时可加 `--manual-only`：程序读取两次人工中心对应的 TCP，生成固定 XY 偏移候选，并将综合动作补偿置零；仍需明确输入 `yes` 才会原子启用。

未加 `--manual-only` 时，双人工锚点完成后也会暂停选择：回车继续所选自动补偿模型，输入 `manual` 立即以人工固定 XY + 零补偿候选结束并进入启用确认，输入 `q` 只保留候选退出。现场不必为了切换备用方案重新运行人工步骤。

需要与综合模型进行实机对比时，可加 `--compensation-model constant`。该模式沿用同一采样、候选和启用接口，只取跨区落点残差的二维中位数作为固定偏心，空间补偿矩阵与旋转偏心均强制为零；验证抓放会实际使用这份固定补偿。

采样模式由 `config.py` 控制：

```python
TASK2_OFFSET_CALIBRATION_PROFILE = "quick"   # 少样本、现场快速建立
TASK2_OFFSET_CALIBRATION_PROFILE = "robust"  # 多位置、多角度，拟合空间补偿
```

当前配置为 `robust`。验证残差超过门限时只拒绝启用候选，不会修改正式偏移。

### 5.5 审计当前正式版本

```powershell
python tools/task2/workflow/task2_calibration_audit.py
```

### 5.6 低速验收后运行整机

先验收单块吸取、释放、RZ方向、托盘落点和叠放，再运行：

```powershell
python main.py
```

## 6. 比赛与学校的策略开关

### 6.1 白板区域门禁

```python
TASK2_BLOCK_BOARD_GUARD_MODE = "fallback"
```

| 值 | 行为 |
| --- | --- |
| `strict` | 白板四边不可靠立即停止，适合保守调试 |
| `fallback` | 优先使用白板区域；边界失败时警告并退回全图HSV＋形状＋面积识别 |
| `off` | 不检测白板，背景必须足够干净 |

当前比赛策略为 `fallback`。它只放宽白板定位，不会跳过任务卡规则、所需颜色、轮廓尺寸、坐标版本或机器人到位检查。

### 6.2 旋转方向

```python
TASK2_ROTATION_DIRECTION = "negative"
```

| 值 | 行为 |
| --- | --- |
| `shortest` | 在正负方向中选择最短等价角 |
| `positive` | 所有非零RZ增量只走正向 |
| `negative` | 所有非零RZ增量只走负向 |

正方形每90°方向等价，因此 `+20°` 在全负模式下可转换为 `-70°`。当前学校工位使用 `negative` 避免正向旋转碰撞；赛场确认空间后可改回 `shortest`。

### 6.3 颜色完整性与执行

```python
TASK2_REQUIRE_ALL_COLORS = True
TASK2_EXECUTE_ROBOT = True
TASK2_REQUIRE_OFFSET_FILE = True
```

- `TASK2_REQUIRE_ALL_COLORS=False` 只是不要求无关颜色全部出现；任务卡实际引用的来源和目标仍必须识别。
- 当前没有“识别几步就执行几步”的部分任务模式。
- `TASK2_EXECUTE_ROBOT=False` 只使正式任务生成离线计划，不是所有工具的全局急停开关。

## 7. 执行边界

正式运行会硬停止于位姿/单位错误、标定与偏移版本不匹配、机器人未连接或未到位、Tool IO失败、任务卡规则无效，以及计划所需物块、托盘、方向、坐标或高度缺失。这些门禁保护数据和动作的一致性，不建议为了现场通过而取消。

视觉策略允许按第6节降级。放宽白板或颜色限制会提高继续执行的概率，也会提高背景误识别风险，必须结合保存的原图、mask和标注图判断。

跨区运动采用“原地升高→高位平移/旋转→垂直下降”，抓放在共同净空高度完成水平转移。它不是三维避障规划：系统没有学校机架、线缆、关节和末端工具模型，也没有真空压力、掉块检测或放置后复拍。`ROBOT_SAFE_Z`、旋转方向、速度、TCP和抓放Z必须在当前工位实测。

## 8. 运行输出与排障入口

正式任务输出：

```text
output/task2/task2_<时间>.json
output/task2/task2_error_<时间>.log
output/task2/*_detected.jpg
output/task2/*_mask.png
```

闭环标定输出：

```text
output/task2/closed_loop_offset_calibration/<时间>/
```

常见处理：

- 白板边界警告：`fallback`会继续全图识别，检查标注图是否出现板外候选。
- 颜色不完整：检查原图、各色mask、轮廓尺寸和任务所需颜色。
- 偏移属于旧矩阵：重新运行闭环偏移，不复制旧指纹。
- 纯平移残差超限：检查误检、点位、单位和矩阵。
- 托盘验证失败：候选不会覆盖正式偏移，查看该轮完整报告。
- 移动或吸盘失败：检查SDK返回、最终TCP误差、目标可达性、Tool IO回读和实物气路。

候选标定和偏移只有在质量检查后明确输入 `yes` 才会备份旧版并覆盖正式数据。

## 9. AI盒子与外部依赖

默认语音后端：

```python
VOICE_BACKEND = "ai_box"
```

AI盒子负责录音、ASR和播报适配；业务层仍只调用统一的 `Voice` 接口。语音异常不会静默切换成本地后端。

任务卡视觉模型需要有效API配置。密钥、设备密码和现场账号不得写入README、源码或运行报告。

项目的AI维护约束、接口不变量和设计历史位于 `.agents/skills/aubo-competition-project/`；README只介绍工程架构和执行使用方法。
