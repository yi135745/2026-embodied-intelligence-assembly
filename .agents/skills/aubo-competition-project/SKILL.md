---
name: aubo-competition-project
description: 维护本仓库的 AUBO 具身智能竞赛工程；在修改或审查架构、模块接口、任务流程、视觉标定、位姿偏移、现场工具或硬件行为时使用。
---

# AUBO 竞赛工程约束

本工程是用户在省赛实机迭代中逐步形成、再扩展到国赛的系统，不是可任意重构的通用机器人样例。优先理解既有设计，再改变实现。

## 开始工作

1. 先判断请求属于讨论、诊断、实现还是实机操作。用户说“讨论”“先回答”“暂不修改”时，只读检查并说明结论，不修改文件。
2. 查看 `git status --short`，把已有改动视为用户资产；不得用回退或批量覆盖清理现场数据。
3. 根据任务读取下列最少必要参考：
   - 涉及分层、目录或职责：读 [references/architecture.md](references/architecture.md)。
   - 涉及方法签名、JSON、坐标、标定或数据文件：读 [references/interfaces-and-data.md](references/interfaces-and-data.md)。
   - 涉及调试、标定、测试、相机或机器人执行：读 [references/workflows-and-safety.md](references/workflows-and-safety.md)。
   - 涉及“为什么这样设计”、省赛兼容或准备重构：读 [references/design-history.md](references/design-history.md)。
4. 在结构性修改前，向用户简述当前接口、拟保留的不变量和拟改变的边界。若修改会新增第二套正式数据、平行入口、隐式回退或跨层职责，而用户没有明确要求，先停下讨论。

## 核心不变量

- `main.py` 只负责生命周期、语音派发和任务完成状态；`task/` 只编排；`modules/` 是彼此平行的功能库；`drivers/` 是 SDK/HTTP 边界；`tools/` 是现场工作流和诊断入口。
- 业务流程只依赖模块接口，不直接调用 MVS、AUBO、DashScope、FunASR 或盒子 HTTP 底层。
- `voice.py`、`camera.py`、`interpreter.py`、`robot.py`、`task2_perception/`、`task2_planning.py`、`task2_calibration/` 之间不得横向导入；只可依赖 `contracts/`、各自内部私有实现或对应 `drivers/`，由 `task/` 或 `tools/` 组合。
- `runtime/` 只加载唯一正式现场数据；功能库不得通过另一个功能库间接读取现场状态。可复用实现下沉到对应功能库，工具只编排。
- `Interpreter` 只提取任务语义；字段校验、顺序、坐标、方向、高度、状态提交和安全门禁由确定性程序负责。禁止用模型输出直接驱动机械臂。
- 所有 TCP 位姿统一为 `[x,y,z,rx,ry,rz]`，XYZ 为毫米，姿态为弧度；AUBO SDK 边界才把 XYZ 转成米。
- `data/aubo_poses.json` 是四个正式拍照位的唯一运行时来源。不要同时在 `config.py` 维护另一套有效位姿。
- 方块区与托盘区在相机 Z、RZ 相同的前提下共用唯一平面矩阵 `resources/visionmaster_task2_calibration.xml`；XY 拍照位可以不同，两区各自偏移保存在同一个 `data/task2_offsets_v2.json`。
- Z 从 `config.py` 读取，抓放姿态从当前拍照位读取；XY 偏移文件不拥有 Z，也不能替代位姿文件。
- 正式矩阵、质量绑定、完整观测报告和偏移指纹是一条版本链。启用新矩阵后旧偏移变为 STALE 是正确行为，必须重做闭环偏移，不能绕过检查。
- `output/` 是可删除的运行证据，不是正式状态源。运行所必需的模板、正式矩阵、绑定和完整报告必须在 `resources/`，正式现场状态在 `data/`。
- 任务二在执行前预检完整计划；只有一次抓放明确成功后才能提交 `placed_block_map`。预测终态不是执行成功。
- 不得把离线单测通过表述为相机、语音、API、吸盘或整机精度已经验收。

## 修改原则

- 优先扩展现有接口；不要因为局部脚本方便就增加长期并存的正式接口或数据源。
- 兼容层只能映射到同一真源，不能形成可独立变化的第二实现。
- 错误必须在边界处显式暴露。不要静默换矩阵、沿用旧偏移、猜测缺失任务、切换语音后端或把机器人失败当成功。
- 现场候选数据先写候选和报告，经质量门禁、人工核对、明确确认、备份后再原子启用。
- 硬件动作、Tool IO、相机占用、外部 API 与正式数据覆盖分别是不同权限；用户批准代码修改不等于批准实机执行。
- 结构改变后同步更新本 Skill 的相关 reference、根 README、工具说明和回归测试。不要在普通 README 与 Skill 中复制整段内容；README 面向人使用，Skill 记录 AI 决策约束。

## 完成标准

说明保留了哪些契约、改变了哪些边界、验证覆盖到哪里，以及仍需哪类实机验收。若当前正式矩阵或偏移处于 STALE/UNKNOWN/FAIL，明确报告，不以降低阈值或删除绑定消除告警。
