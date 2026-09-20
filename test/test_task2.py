"""国赛离线回归，不连接硬件/API，不改现场数据。"""
import io
import json
import math
import sys
import tempfile
import unittest
from contextlib import ExitStack, redirect_stdout
from copy import deepcopy
from pathlib import Path
from unittest.mock import Mock, patch, call
from xml.etree import ElementTree

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config
import main
import modules.task2_vision as vm
from modules.llm import LLM
from modules.robot import Robot
from modules.task2_planning import build_plan, normalize_square_angle, validate_actions
from modules.task2_vision import ColorObjectDetector, CoordinateTransformer, VisionTarget, validate_colors
from task.task2 import task2_run
from task2_tuner import save_tuning


def actions(count=3):
    colors = config.TASK2_TRAY_COLORS
    result = [{"step": i+1, "source_color": c, "target_type": "tray",
               "target_color": colors[(i+1) % 6]} for i, c in enumerate(colors)]
    for i, color in enumerate(config.TASK2_EXTRA_BLOCK_COLORS[:count]):
        result.append({"step": 7+i, "source_color": color, "target_type": "block", "target_color": colors[i]})
    return result


def targets(kind):
    colors = config.TASK2_BLOCK_COLORS if kind == "方块" else config.TASK2_TRAY_COLORS
    return [VisionTarget(kind, c, (float(50+i*100), 100.), 6400, 20,
                         [float(50+i*100), 100. if kind == "方块" else -100.,
                          173 if kind == "方块" else 180, math.pi, 0., 1.57],
                         20. if kind == "方块" else 0.) for i, c in enumerate(colors)]


def make_image(path, kind):
    ranges = config.TASK2_BLOCK_HSV_RANGES if kind == "方块" else config.TASK2_TRAY_HSV_RANGES
    image = np.full((450, 1300, 3), 245, np.uint8)
    for i, values in enumerate(ranges.values()):
        lo, hi = values[0]
        hsv = np.uint8([[[sum(pair)//2 for pair in zip(lo, hi)]]])
        bgr = tuple(map(int, cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]))
        corners = cv2.boxPoints(((70+i*140, 200), (70, 70), 20)).astype(np.int32)
        cv2.fillConvexPoly(image, corners, bgr)
    assert cv2.imwrite(str(path), image)
    return path


class Task2Test(unittest.TestCase):
    def setUp(self):
        self.scope = ExitStack()
        self.addCleanup(self.scope.close)
        self.scope.enter_context(redirect_stdout(io.StringIO()))
        for name, value in {
            "TASK2_EXPECTED_STACK_COUNT": None, "TASK2_REFERENCE_BLOCK_HEIGHT_MM": 30.,
            "TASK2_DISABLED_BLOCK_COLORS": (),
            "TASK2_BLOCK_HEIGHT_MM": {**{c: 30. for c in config.TASK2_TRAY_COLORS},
                                      **{c: 28. for c in config.TASK2_EXTRA_BLOCK_COLORS}},
            "TASK2_ROTATION_ENABLED": True, "TASK2_ROTATE_AT_CLEARANCE": True,
            "ROBOT_SAFE_Z": 400.,
        }.items():
            self.scope.enter_context(patch.object(config, name, value))

    def test_actions_card_expansion_and_explicit_count(self):
        for count in (1, 2, 3):
            self.assertEqual(actions(count), validate_actions(actions(count)))
        with patch.object(config, "TASK2_EXPECTED_STACK_COUNT", 3):
            with self.assertRaises(ValueError):
                validate_actions(actions(1))
        with patch.object(config, "TASK2_EXPECTED_STACK_COUNT", 0):
            self.assertEqual(6, len(validate_actions(actions(0))))

    def test_invalid_actions(self):
        for field, value in (("step", 8), ("step", True), ("source_color", "红色"),
                             ("target_type", "tray"), ("target_color", "棕色"), ("target_color", [])):
            invalid = actions()
            invalid[6][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                validate_actions(invalid)
        for value in (None, [], actions(0), [None]*9):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_actions(value)
        for index, target in ((1, "橙色"), (7, "红色")):
            invalid = actions()
            invalid[index]["target_color"] = target
            with self.assertRaises(ValueError):
                validate_actions(invalid)

    def test_stack_final_xy_source_height_and_rotation(self):
        blocks, trays = targets("方块"), targets("托盘")
        originals = deepcopy(blocks)
        plan = build_plan(actions(), blocks, trays)
        cyan = plan[6]
        self.assertEqual(trays[1].robot_pose[:2], cyan["place"]["robot_pose"][:2])
        self.assertNotEqual(blocks[0].robot_pose[:2], cyan["place"]["robot_pose"][:2])
        self.assertEqual(180, plan[0]["place"]["robot_pose"][2])
        self.assertEqual(208, cyan["place"]["robot_pose"][2])
        self.assertEqual(171, cyan["pick"]["robot_pose"][2])
        self.assertEqual(-20, cyan["rotation_delta_deg"])
        self.assertAlmostEqual(1.57-math.radians(20), cyan["place"]["robot_pose"][5])
        self.assertEqual(originals, blocks)

    def test_multi_layer_with_measured_heights(self):
        steps = actions()
        steps[7]["target_color"] = "青色"
        steps[8]["target_color"] = "粉色"
        with patch.dict(config.TASK2_BLOCK_HEIGHT_MM, {"青色": 27.5, "粉色": 26., "棕色": 29.}):
            plan = build_plan(steps, targets("方块"), targets("托盘"))
        self.assertEqual([207.5, 233.5, 262.5], [p["place"]["robot_pose"][2] for p in plan[6:]])

    def test_rotation_disabled_records_actual_angle(self):
        with patch.object(config, "TASK2_ROTATION_ENABLED", False):
            plan = build_plan(actions(), targets("方块"), targets("托盘"))
        self.assertEqual(20, plan[0]["placed_state"]["angle_deg"])
        self.assertEqual(0, plan[0]["rotation_delta_deg"])

    def test_missing_targets_and_invalid_geometry(self):
        blocks = targets("方块")
        for value in (None, [0, 1, math.nan, 0, 0, 0]):
            blocks[-1].robot_pose = value
            with self.assertRaises(ValueError):
                build_plan(actions(), blocks, targets("托盘"))
        with self.assertRaises(ValueError):
            build_plan(actions(), targets("方块")[:-1], targets("托盘"))
        with patch.dict(config.TASK2_BLOCK_HEIGHT_MM, {"青色": -1}):
            with self.assertRaises(ValueError):
                build_plan(actions(), targets("方块"), targets("托盘"))

    def test_square_wrap_and_image_y_flip(self):
        self.assertEqual(2, normalize_square_angle(-88))
        self.assertEqual(-2, normalize_square_angle(88))
        transformer = CoordinateTransformer()
        rect = ((100., 100.), (40., 40.), 20.)
        for matrix in (np.diag([1., -1., 1.]), np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., 1.]])):
            transformer.matrix = matrix
            self.assertAlmostEqual(-20, transformer.rectangle_angle_to_robot(rect), places=4)

    def test_vm_matrix_saved_correspondences(self):
        root = ElementTree.parse(config.TASK2_CALIBRATION_FILE).getroot()
        def points(name):
            return [(float(p.findtext("X")), float(p.findtext("Y")))
                    for p in root.find(".//CalibPointFListParam[@ParamName='%s']" % name)]
        transformer = CoordinateTransformer(config.TASK2_CALIBRATION_FILE)
        images, worlds = points("ImagePointLst"), points("WorldPointLst")
        recorded_count = int(root.findtext(".//CalibParam[@ParamName='TransNum']/ParamValue"))
        self.assertGreaterEqual(recorded_count, 9)
        self.assertEqual(recorded_count, len(images))
        self.assertEqual(recorded_count, len(worlds))
        errors = [math.dist(transformer.pixel_to_world(*image), world)
                  for image, world in zip(images, worlds)]
        self.assertTrue(all(math.isfinite(error) for error in errors))
        recorded_rms = float(root.findtext(
            ".//CalibParam[@ParamName='TransWorldError']/ParamValue"))
        computed_rms = math.sqrt(sum(error ** 2 for error in errors) / len(errors))
        self.assertAlmostEqual(recorded_rms, computed_rms, places=4)

    def test_nine_blocks_six_trays_with_angles(self):
        with tempfile.TemporaryDirectory() as folder:
            transformer = CoordinateTransformer()
            transformer.matrix = np.eye(3)
            detector = ColorObjectDetector(transformer)
            for kind, count in (("方块", 9), ("托盘", 6)):
                path = make_image(Path(folder)/("%d.png" % count), kind)
                found, _ = detector.detect(path, kind, include_robot_pose=False)
                validate_colors(found, kind)
                self.assertEqual(count, len(found))
                self.assertEqual(count, len(set(t.pixel_center for t in found)))
                for target in found:
                    self.assertAlmostEqual(20, target.robot_angle_deg, delta=1)
                    colors = config.TASK2_BLOCK_COLORS if kind == "方块" else config.TASK2_TRAY_COLORS
                    self.assertAlmostEqual(70 + colors.index(target.color)*140, target.pixel_center[0], delta=2)

    def test_missing_cyan_can_be_ignored_for_vision_but_not_execution(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(
                config, "TASK2_DISABLED_BLOCK_COLORS", ("青色",)):
            transformer = CoordinateTransformer()
            transformer.matrix = np.eye(3)
            image = cv2.imread(str(make_image(Path(folder) / "blocks.png", "方块")))
            cyan_index = config.TASK2_BLOCK_COLORS.index("青色")
            cv2.rectangle(image, (cyan_index * 140 + 30, 150),
                          (cyan_index * 140 + 110, 250), (245, 245, 245), -1)
            path = Path(folder) / "without_cyan.png"
            self.assertTrue(cv2.imwrite(str(path), image))
            found, _ = ColorObjectDetector(transformer).detect(
                path, "方块", include_robot_pose=False)
            validate_colors(found, "方块")
            self.assertEqual(set(config.TASK2_BLOCK_COLORS) - {"青色"},
                             {target.color for target in found})
            with self.assertRaisesRegex(ValueError, "已屏蔽的青色方块"):
                validate_actions(actions())

    def test_missing_block_is_not_silently_filled(self):
        with tempfile.TemporaryDirectory() as folder:
            path = make_image(Path(folder)/"missing.png", "方块")
            image = cv2.imread(str(path))
            image[:, 1120:] = 245  # 删除最后的棕色物块
            cv2.imwrite(str(path), image)
            with self.assertRaises(ValueError):
                found, _ = ColorObjectDetector().detect(path, "方块", include_robot_pose=False)
                validate_colors(found, "方块")

    def test_llm_bad_card_is_logged_and_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            response = Mock()
            response.__enter__ = Mock(return_value=response)
            response.__exit__ = Mock(return_value=False)
            response.read.return_value = json.dumps({"choices": [{"message": {"content": "[]"}}]}).encode()
            with patch("modules.llm.request.urlopen", return_value=response), patch(
                    "modules.llm._image_file_to_data_url", return_value="data:image/jpeg;base64,AA"), patch.object(
                    config, "DASHSCOPE_API_KEY", "offline-test"), self.assertRaises(ValueError):
                LLM().parse_task2_card("unused.png", folder)
            text = next(Path(folder).glob("task2_llm_*.txt")).read_text(encoding="utf-8")
            self.assertIn("[RAW_OUTPUT]\n[]", text)
            self.assertNotIn("[VALIDATED_STEPS]", text)

    def test_old_tuning_keeps_new_colors_and_other_scene(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"tuning.json"
            with patch.object(config, "TASK2_TUNING_FILE", str(path)), patch.object(
                    config, "TASK2_BLOCK_HSV_RANGES", deepcopy(config.TASK2_BLOCK_HSV_RANGES)), patch.multiple(
                    config, TASK2_BLOCK_GAIN=None, TASK2_TRAY_GAIN=None):
                save_tuning({"block_hsv_ranges": {"红色": [[[0,90,50], [8,255,200]]]}, "capture": {"block_gain": 1}})
                save_tuning({"tray_hsv_ranges": {}, "capture": {"tray_gain": 2}})
                vm.load_task2_tuning(path)
                self.assertEqual(9, len(config.TASK2_BLOCK_HSV_RANGES))
                data = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual({"block_gain": 1, "tray_gain": 2}, data["capture"])

    def test_pick_and_place_rotates_at_clearance(self):
        robot = Robot.__new__(Robot)
        events = []
        robot.get_current_pose = lambda: [500,600,250,math.pi,0,0]
        robot.move_to = lambda p: events.append(("move", list(p))) or True
        robot.vacuum_on = lambda: events.append(("on", None)) or True
        robot.vacuum_off = lambda: events.append(("off", None)) or True
        pick, place = [10,20,173,math.pi,0,0], [100,200,208,math.pi,0,0.4]
        self.assertTrue(robot.pick_and_place(pick, place, 80))
        moves = [p for name,p in events if name == "move"]
        self.assertEqual([500,600,400], moves[0][:3])
        self.assertEqual(pick, moves[2])
        self.assertEqual([10,20,400,math.pi,0,0.4], moves[4])
        self.assertEqual([100,200,400], moves[5][:3])
        self.assertEqual(place, moves[6])
        self.assertEqual("on", events[3][0])
        self.assertEqual("off", events[-2][0])

    def test_motion_stops_on_rotation_failure(self):
        robot = Robot.__new__(Robot)
        robot.get_current_pose = lambda: [0,0,400,math.pi,0,0]
        robot.move_to = Mock(side_effect=[True,True,True,True,False])
        robot.vacuum_on = Mock(return_value=True)
        robot.vacuum_off = Mock(return_value=True)
        self.assertFalse(robot.pick_and_place([0,0,173,math.pi,0,0], [100,100,208,math.pi,0,1]))
        self.assertEqual(5, robot.move_to.call_count)

    def test_orientation_error_wrap(self):
        self.assertAlmostEqual(math.degrees(0.02), Robot._orientation_error_degrees(
            [0,0,0,math.pi,0,math.pi-0.01], [0,0,0,-math.pi,0,-math.pi+0.01]))

    def test_llm_parse_and_log_without_network(self):
        with tempfile.TemporaryDirectory() as folder:
            payload = {"choices": [{"message": {"content": json.dumps(actions(), ensure_ascii=False)}}]}
            response = Mock()
            response.__enter__ = Mock(return_value=response)
            response.__exit__ = Mock(return_value=False)
            response.read.return_value = json.dumps(payload).encode()
            with patch("modules.llm.request.urlopen", return_value=response), patch(
                    "modules.llm._image_file_to_data_url", return_value="data:image/jpeg;base64,AA"), patch.object(
                    config, "DASHSCOPE_API_KEY", "offline-test"):
                self.assertEqual(actions(), LLM().parse_task2_card("unused.png", folder))
            text = next(Path(folder).glob("task2_llm_*.txt")).read_text(encoding="utf-8")
            self.assertIn("timestamp:", text)
            self.assertIn("[VALIDATED_STEPS]", text)

    def run_flow(self, folder, execute=True, fail_at=None):
        voice, vision, robot, llm = Mock(), Mock(), Mock(), Mock()
        robot.available = True
        robot.move_to_safe.return_value = True
        robot.pick_and_place.side_effect = [True]*(fail_at-1)+[False] if fail_at else [True]*9
        vision.capture.return_value = Path(folder)/"unused.png"
        llm.parse_task2_card.return_value = actions()
        detector = Mock()
        debug = np.zeros((20,20,3), np.uint8)
        detector.detect.side_effect = [(targets("方块"), debug), (targets("托盘"), debug)]
        with patch("task.task2.ColorObjectDetector", return_value=detector), patch(
                "task.task2.apply_aubo_pose_records"), patch("task.task2.load_task2_tuning"), patch(
                "task.task2.load_task2_offsets"), patch.object(config, "TASK2_OUTPUT_DIR", folder), patch.object(
                config, "TASK2_SETTLE_SECONDS", 0), patch.object(config, "TASK2_EXECUTE_ROBOT", execute), patch.multiple(
                config, TASK2_CARD_VIEW_POSE=[0]*6, TASK2_BLOCK_VIEW_POSE=[0]*6, TASK2_TRAY_VIEW_POSE=[0]*6):
            result = task2_run(voice, vision, robot, llm)
        self.assertEqual(3, vision.capture.call_count)
        return result, robot, voice

    def test_flow_three_photos_nine_committed_states(self):
        with tempfile.TemporaryDirectory() as folder:
            result, robot, _ = self.run_flow(folder)
            self.assertEqual("completed", result["status"])
            self.assertEqual(9, len(result["placed_block_map"]))
            self.assertEqual(9, robot.pick_and_place.call_count)
            self.assertEqual(3, robot.move_to_safe.call_count)

    def test_failure_does_not_commit_or_continue(self):
        with tempfile.TemporaryDirectory() as folder:
            result, robot, voice = self.run_flow(folder, fail_at=7)
            self.assertIsNone(result)
            record = json.loads(next(Path(folder).glob("task2_*.json")).read_text(encoding="utf-8"))
            self.assertEqual("failed", record["status"])
            self.assertEqual(6, len(record["placed_block_map"]))
            self.assertEqual("failed", record["plan"][6]["robot_status"])
            self.assertEqual(7, robot.pick_and_place.call_count)
            self.assertNotIn(call("任务已完成"), voice.speak.call_args_list)

    def test_dry_run_no_motion_no_completion(self):
        with tempfile.TemporaryDirectory() as folder:
            result, robot, _ = self.run_flow(folder, execute=False)
            self.assertEqual("planned", result["status"])
            self.assertEqual({}, result["placed_block_map"])
            robot.move_to_safe.assert_not_called()
            robot.pick_and_place.assert_not_called()

    def test_startup_speech_failure_disconnects_robot(self):
        from modules.voice import VoiceServiceError

        voice, robot = Mock(), Mock()
        voice.speak.side_effect = VoiceServiceError("tts failed")
        with patch("modules.voice.Voice", return_value=voice), \
                patch("main.apply_aubo_pose_records"), patch("main.Vision"), \
                patch("main.Robot", return_value=robot), patch("main.LLM"), \
                patch("main.run_tasks") as run:
            with self.assertRaises(VoiceServiceError):
                main.main()
        robot.disconnect.assert_called_once()
        run.assert_not_called()

    def test_dispatch_success_and_failure(self):
        for order in (("任务一", "任务二"), ("任务二", "任务一")):
            voice = Mock()
            voice.listen.side_effect = order
            voice.wake.return_value = True
            voice.is_exit.return_value = False
            with patch("main.task1_run", return_value=True), patch("main.task2_run", return_value={"status": "completed"}):
                main.run_tasks(voice, None, None, None)
            voice.speak.assert_any_call("两个任务均已完成")
        voice = Mock()
        voice.listen.return_value = "任务二"
        voice.wake.side_effect = [True,False]
        voice.is_exit.return_value = False
        with patch("main.task2_run", return_value=None):
            main.run_tasks(voice, None, None, None)
        self.assertNotIn(call(config.RETURN_REPLY), voice.speak.call_args_list)


if __name__ == "__main__":
    unittest.main()
