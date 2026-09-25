"""调试程序按场景自动到记录的拍照位；不修改位姿文件。"""

import config
from runtime.site_data import load_aubo_pose_records
from modules.robot import Robot


SCENE_POSES = {
    "card": "TASK2_CARD_VIEW_POSE",
    "block": "TASK2_BLOCK_VIEW_POSE",
    "tray": "TASK2_TRAY_VIEW_POSE",
}


def move_to_scene_view(scene):
    name = SCENE_POSES[scene]
    pose = load_aubo_pose_records().get(name)
    if pose is None:
        raise RuntimeError("AUBO位姿记录缺少%s。" % name)
    print("调试自动前往%s：%s；过渡安全Z：%.3f mm" %
          (name, pose, float(config.ROBOT_SAFE_Z)))
    robot = Robot()
    try:
        if not robot.available:
            raise RuntimeError("AUBO未连接，无法到达%s。" % name)
        if not robot.move_to_safe(pose):
            raise RuntimeError("到达%s失败；不启动相机调试。" % name)
    finally:
        robot.disconnect()
