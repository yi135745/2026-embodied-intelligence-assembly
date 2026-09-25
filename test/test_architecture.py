"""架构依赖离线守卫；不导入业务模块，不连接任何设备。"""

import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULES = ROOT / "modules"
PUBLIC_FEATURES = {
    "voice", "camera", "interpreter", "robot",
    "task2_perception", "task2_planning", "task2_calibration",
}


def feature_owner(path):
    relative = path.relative_to(MODULES)
    first = relative.parts[0]
    return first[:-3] if first.endswith(".py") else first


def imported_roots(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                yield alias.name
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            yield node.module


class ArchitectureDependencyTests(unittest.TestCase):
    def test_public_features_do_not_import_each_other(self):
        problems = []
        for path in MODULES.rglob("*.py"):
            owner = feature_owner(path)
            if owner not in PUBLIC_FEATURES:
                continue
            for imported in imported_roots(path):
                if not imported.startswith("modules."):
                    continue
                target = imported.split(".", 2)[1]
                if target in PUBLIC_FEATURES and target != owner:
                    problems.append("%s: %s -> %s" %
                                    (path.relative_to(ROOT), owner, target))
        self.assertEqual(problems, [], "功能库存在横向依赖：\n" + "\n".join(problems))

    def test_features_do_not_depend_on_orchestration_or_runtime_loader(self):
        problems = []
        for path in MODULES.rglob("*.py"):
            owner = feature_owner(path)
            if owner not in PUBLIC_FEATURES:
                continue
            for imported in imported_roots(path):
                root = imported.split(".", 1)[0]
                if root in {"task", "tools", "runtime"}:
                    problems.append("%s: %s" % (path.relative_to(ROOT), imported))
        self.assertEqual(problems, [], "功能库反向依赖了组合层：\n" + "\n".join(problems))


if __name__ == "__main__":
    unittest.main()
