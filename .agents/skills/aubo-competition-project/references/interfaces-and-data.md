# 接口与数据契约

## 公共模块接口

### Voice

- `Voice.wake(wake_word) -> bool`：命中唤醒词返回 `True`，退出命令返回 `False`。
- `Voice.listen() -> str`：返回纠错、归一化后的单次识别文本。
- `Voice.speak(text) -> None`：同步播报；AI 盒子失败应抛出服务错误，不静默切换电脑音频。
- `Voice.is_exit(text) -> bool`：供命令阶段判断全局退出。

改变盒子协议应尽量只改 `drivers/ai_box.py`，保持 `modules/voice.py` 和任务层调用不变。盒子原生唤醒、上位机文字匹配和本地 FunASR 是不同语义，不能混写成同一种能力。

### Camera

- `Camera.capture(output_name=None, debug_image=None, exposure_time=None, gain=None) -> Path`。
- 调试图优先于实机；未指定任务输出且根目录本地测试图存在时可读取本地图；否则调用 MVS。
- 相机采集只负责得到图片，不负责颜色检测、任务理解或机器人移动。

### Robot

- `Robot.available` 表示是否成功连接。
- `get_current_pose() -> [x,y,z,rx,ry,rz]`：返回基座 TCP，XYZ mm、姿态 rad。
- `move_to(pose) -> bool`：SDK 边界将 mm 转 m，等待位置与姿态均到位。
- `move_to_safe(pose, safe_z=None) -> bool`：用于跨区域，顺序为原地升高、在安全高度平移/旋转、下降。
- `pick_and_place(pick_pose, place_pose, lift_mm=None) -> bool`：负责共同净空、吸取、旋转、转移、释放；不决定目标语义。
- `set_suction(enabled) -> bool`：末端 Tool IO，不是控制柜 Standard DO；配置和回读失败不得视为成功。
- `disconnect()` 必须在集成入口或独立硬件脚本的 `finally` 中调用。

连接失败可以形成 `available=False` 供显式离线流程使用，但当 `TASK2_EXECUTE_ROBOT=True` 时任务二必须失败，不能把“无臂计划生成”冒充执行完成。

### Interpreter 与任务

- `Interpreter.identify_image(image) -> str`：任务一可播报文字。
- `Interpreter.parse_task2_card(image, output_dir=None) -> list | dict`：只负责API调用和合法JSON解析；任务层再调用动作协议校验，设备适配层不依赖业务规则。
- `task1_run(...) -> bool`：只有完整识别成功才返回 `True`。
- `task2_run(...) -> dict | None`：真实执行完整成功时 `record["status"] == "completed"`；离线计划不是完成任务。
- `main.run_tasks(...)` 只在上述成功契约满足时提交本轮完成状态；任一任务失败会清空本轮状态。

## 任务二动作协议

每项必须恰好包含：

```json
{
  "step": 1,
  "source_color": "红色",
  "target_type": "tray",
  "target_color": "黄色"
}
```

- `step` 从 1 连续编号；程序不补号、不重排。
- 来源方块不能重复。
- 前六步为六个基础色方块到六个不同托盘，完整覆盖六色。
- 后续来源只能是青/粉/棕，目标类型为 `block`，且目标方块必须已放置、顶面未被占用。
- 模型输出不完整、字段多余、颜色非法或动作数不符时拒绝，不猜测。

`modules/task2_planning.py` 是动作协议、标称装配计划和命令补偿的唯一功能入口；`contracts/task2.py` 只保存跨功能库的数据与规范角契约。提示词只是帮助模型输出，不是验证器。

## 视觉目标协议

`VisionTarget` 字段：

- `kind`：`方块` 或 `托盘`；
- `color`：标准中文颜色；
- `pixel_center`、`area`、`angle_deg`：图像观测；
- `robot_pose`：公共矩阵 + 分区原点/偏移 + config Z + 当前拍照姿态形成的六维位姿；
- `robot_angle_deg`：矩形边的两点分别经过公共矩阵后，在基座 XY 平面计算出的方向。

不要把 OpenCV 图像角直接当 AUBO RZ，也不要把颜色身份用于托盘九点几何对应。

## 坐标与标定模型

稳定模型是：

```text
公共单应矩阵: pixel -> planar world delta (mm)
robot XY = zone view origin XY + world delta + zone XY offset
robot Z  = config 中的抓取/放置参考 Z
robot 姿态 = 对应当前拍照位姿态，放置 RZ 再叠加最小等价旋转
```

约束：

- 方块和托盘必须通过同一个 `CoordinateTransformer` 和同一正式 XML。
- 两区拍照位 XY 可以不同；相机 Z 与 RZ 必须在配置容差内。当前实现不校验 RX/RY 同一性，因此改变 RX/RY 仍应视为重新标定风险，不能据此宣称矩阵可复用。
- 正方形按 90° 等价，旋转差归一化到 `[-45°, 45°)`。
- `TASK2_ROTATION_DIRECTION` 可在等价角中选择最短双向、全正向或全负向；主任务和闭环工具必须调用同一选择函数，日志同时记录请求角与实际命令角。
- 高度补偿使用同一 TCP 接触定义；`top_tcp_z` 是等效 TCP 顶面高度，不是裸物块物理坐标。

## 正式文件契约

### `data/aubo_poses.json`

- 必须声明 `position_unit=mm`、`orientation_unit=rad`。
- 必须且只能包含 `TASK2_CARD_VIEW_POSE`、`TASK2_BLOCK_VIEW_POSE`、`TASK2_TRAY_VIEW_POSE`、`ROBOT_TARGET`。
- `apply_aubo_pose_records()` 在运行时覆盖 config 中的这四项。

### `resources/visionmaster_task2_calibration.xml`

- 唯一公共像素到平面矩阵；正式接口不区分 block/tray XML。
- 模板 `visionmaster_task2_template.xml` 独立存在，不能从正在使用的正式文件临时复制后再覆盖原件。
- 绑定 JSON 保存正式 XML 指纹与质量摘要；完整报告 JSON 保存闭环偏移所需观测。
- 历史 XML 存入 `resources/calibration_history/`，恢复也必须经过校验和备份。

### `data/task2_offsets_v2.json`

- 一个文件同时保存 block/tray 的原点 XY、XY offset 和拍照姿态快照。
- 只接受一个 `calibration_xml_sha256`，必须与当前正式公共 XML 一致。
- `block_xy_offset`、`tray_xy_offset` 只表达公共矩阵到两区基座坐标的静态锚定，不得吸收抓取偏心、旋转偏心或释放滑动。
- 可选 `motion_compensation` 与坐标字段保存在同一正式文件并共享版本链；旧文件没有该字段时等价于零补偿。
- 规划中的 `robot_pose`/`placed_state.pose` 是标称物体终态；`command_robot_pose` 是扣除预测动作残差后实际下发的 TCP 位姿。叠放推演必须使用标称终态，不能使用已补偿命令重复扣减。
- 保存 `calibration_world_scale_mm`，阻止单位不明的旧文件被误用。
- 旧版 `tray_calibration_xml_sha256` 仅可忽略兼容，不能重新成为第二指纹。
- Z 不从该文件加载。历史配置名 `TASK2_BLOCK_PICK_Z` / `TASK2_TRAY_PLACE_Z` 的语义是抓取 Z / 放置 Z，而不是物块区域 Z / 托盘区域 Z；跨区反向取回同样按动作选择 Z。

### `data/task2_tuning.json`

- 由调参工具生成，本机光照/相机相关；加载时按颜色合并，不得用旧六色文件抹掉国赛新增颜色。

## 兼容规则

- 保留被 `main.py`、`task/`、工具和 Fake 测试使用的公共方法签名；确需改变时同时迁移调用者和测试，并向用户说明。
- 别名可以暂时指向同一真源，但不得让别名拥有独立文件、独立指纹或不同运行结果。
- 失败消息应指出应该运行的当前工具路径，不要把用户导回根目录旧脚本或 legacy 流程。
