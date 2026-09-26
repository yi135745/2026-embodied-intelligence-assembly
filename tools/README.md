# 调试工具入口

所有命令均可在项目根目录运行；脚本也会自行定位项目根目录，因此切换到脚本所在目录后直接运行同样有效。

## 任务二现场主流程

依次执行，不要并行运行：

```powershell
python tools/setup/aubo_pose_recorder.py
python tools/task2/workflow/task2_tuner.py
python tools/task2/workflow/task2_card_capture_tuner.py
python tools/task2/workflow/task2_tuner.py --scene tray
python tools/task2/workflow/task2_tray_auto_calibrate.py
python tools/task2/workflow/task2_closed_loop_offset_calibrate.py
```

调参入口均默认自动安全移动到对应拍照位。`task2_tuner.py` 不带参数时调物块9色；`--scene tray` 调托盘6色；任务卡独立工具只调曝光/增益。物块调参位的Z按现场物块高度自动抬高。

托盘九点标定的首帧和后续帧都调用当前托盘颜色检测器：方框几何中心优先，HSV/Lab 结果只补几何漏检中心；六槽位身份仍由 2×3 空间拓扑确定，不按颜色建立对应。

闭环标定在方块人工锚定后立即写检查点和可验证的物理锚点候选。现场需要保底时运行
`python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --manual-only`：只记录一次方块区人工物理锚点（像素、TCP、参考拍照位和物块高度），综合动作补偿置零，不执行自动抓放采样。托盘区不再建立第二个人工偏移。默认 `--board-guard fallback` 在白板边界不可靠时改做全画面 HSV 单方块审计，仍保留唯一目标、面积、形状、填充率和复拍位移检查。

物理锚点与托盘九点矩阵可按任意顺序采集，正式运行时由标定库组合。矩阵把像素转换为平面毫米增量；物理锚点提供唯一相机—吸盘绝对平移。旋转试验正式保存旋转前后像素中心，毫米偏心由当前矩阵派生。方块实际拍照Z由参考平面拍照位加现场物块高度得到，托盘拍照位不变。

HSV 调参窗口直接显示正式检测器最终结果，滑条与结果保持在同一窗口；单色掩膜仍在内部参与计算，但不再占用显示区域。数字键切换颜色，`[`/`]` 切换同色多段 HSV，`n` 新增区间，`x` 删除区间，`s` 只保存参数；确认正式检测的颜色和中心均正确后按 `v`，完整性门禁通过才会保存人工肉眼审计，并从每个已确认目标内部提取现场 HSV 原型用于易混颜色分类。审计图片和 JSON 位于 `output/task2/tuning_review/`，正式运行仍只读取 `data/task2_tuning.json`。

红色与粉色允许 HSV 色相范围重叠，但正式检测不会再按两个掩膜各自独立占用轮廓；程序对红/粉完整方块候选按现场 HSV 原型的色相、饱和度和亮度执行一对一分配，右侧以 `RP:R`、`RP:P` 标出最终结果。
红粉标记在当前正式检测窗口中放大显示。若 `RP:R` 与 `RP:P` 正好颠倒，按 `r` 对调红/粉分类原型；核对正确后按 `v`，工具会从当前两个实物重新采样并固化正确原型。

正式放置释放顺序为：先开泄压阀并短暂保持气泵运行，再停泵、继续泄压，保持阀开启低速竖直抬升 10 mm，随后关阀并恢复正常抬升。重叠时间、停泵后泄压时间、慢抬距离、速度和加速度均由 `config.py` 的 `TOOL_IO_VENT_BEFORE_PUMP_OFF_SEC`、`TOOL_IO_RELEASE_WAIT_SEC` 和 `TASK2_RELEASE_SLOW_*` 参数控制；抓取流程不使用该慢抬段。

`TASK2_BLOCK_PICK_Z` 与 `TASK2_TRAY_PLACE_Z` 的历史名称按动作语义解释为抓取 Z 和放置 Z，不按物块区/托盘区解释。闭环正向搬运和反向取回都必须在吸取时使用前者、释放时使用后者。

若要对比纯固定偏心与综合补偿，运行 `python tools/task2/workflow/task2_closed_loop_offset_calibrate.py --compensation-model constant`。固定模式对落点残差逐轴取中位数，输出与正式接口相同的 `motion_compensation`，但 `spatial_residual_matrix` 和 `rotation_center_bias_mm` 必须为零；独立验证样本仍不参与计算。

无论选择 `constant` 还是 `adaptive`，双人工锚点完成并落盘后都会暂停：回车继续自动采样，输入 `manual` 以人工固定 XY + 零补偿候选结束并进入启用确认，输入 `q` 保留候选后退出且不修改正式文件。

新机首次部署不要求存在历史版本，但要求当前矩阵具有可追溯质量结果。新生成矩阵的 RMS 会随正式绑定文件持久保存；历史版本只用于回退，不是质量校验的前提。相机、安装高度或姿态改变后，应重新生成相应矩阵，不能仅凭旧机器上的 PASS 继续使用。

## 目录说明

- `setup/`：点位记录、吸盘 IO 等通用设备设置。
- `task2/workflow/`：现场正式使用的四步流程、质量审计和版本恢复。
- `task2/diagnostics/`：视觉、拍照高度及托盘识别诊断，不属于必跑流程。

需要只读检查当前版本时运行：

```powershell
python tools/task2/workflow/task2_calibration_audit.py
```
