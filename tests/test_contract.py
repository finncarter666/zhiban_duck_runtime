"""Release invariants that do not require a second checkout or physical hardware."""

import ast
import hashlib
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


# 平台展示文字的精确替换；不改变已存储的算法与时序基线。
PLATFORM_LABELS = {
    "zhiban_runtime/application.py": {
        "Zhiban Duck Linux Runtime": "Zhiban Duck RDK X3",
    },
    "zhiban_runtime/cli.py": {
        "Zhiban Duck Linux runtime": "Zhiban Duck native RDK X3 runtime",
    },
    "zhiban_runtime/control/motion_control.py": {
        "Linux": "D-Robotics RDK X3",
    },
    "zhiban_runtime/hardware/device_runtime.py": {
        "zhiban-hardware100": "rdk-hardware100",
    },
}


class _CanonicalLabels(ast.NodeTransformer):
    def __init__(self, labels):
        self.labels = labels

    def visit_Constant(self, node):
        if isinstance(node.value, str):
            node.value = self.labels.get(node.value, node.value)
        return node


class _WithoutDocstrings(ast.NodeTransformer):
    def generic_visit(self, node):
        node = super().generic_visit(node)
        if isinstance(
            node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)
        ):
            if node.body and isinstance(node.body[0], ast.Expr):
                value = node.body[0].value
                if isinstance(value, ast.Constant) and isinstance(value.value, str):
                    node.body.pop(0)
        return node


class _CanonicalNames(ast.NodeTransformer):
    """仅还原迁移清单中的标识符，不改动常量、分支或调用结构。"""

    def __init__(self, names):
        self.names = names

    def generic_visit(self, node):
        node = super().generic_visit(node)
        fields = {
            ast.FunctionDef: "name",
            ast.AsyncFunctionDef: "name",
            ast.Name: "id",
            ast.Attribute: "attr",
        }
        field = fields.get(type(node))
        if field is not None:
            value = getattr(node, field)
            setattr(node, field, self.names.get(value, value))
        return node


def _stable_ast_dump(value):
    """Use one AST schema across Python 3.8+ for formatting-only comparisons."""
    if isinstance(value, ast.AST):
        # Python 3.8 wraps subscript values; newer interpreters do not.
        if type(value).__name__ == "Index":
            return _stable_ast_dump(value.value)
        if type(value).__name__ == "ExtSlice":
            return _stable_ast_dump(ast.Tuple(elts=value.dims, ctx=ast.Load()))
        fields = []
        for name, field in ast.iter_fields(value):
            if field is None and getattr(type(value), name, ...) is None:
                continue
            # Python 3.12 adds this empty field to ordinary classes/functions.
            if name == "type_params" and not field:
                continue
            fields.append(f"{name}={_stable_ast_dump(field)}")
        return f"{type(value).__name__}({', '.join(fields)})"
    if isinstance(value, list):
        return "[" + ", ".join(_stable_ast_dump(item) for item in value) + "]"
    return repr(value)


def executable_fingerprint(path, canonical_names=None, canonical_labels=None):
    """Ignore documentation and relocated top-level imports, not control branches."""
    tree = _WithoutDocstrings().visit(ast.parse(path.read_text()))
    tree = _CanonicalNames(canonical_names or {}).visit(tree)
    tree = _CanonicalLabels(canonical_labels or {}).visit(tree)
    tree.body = [
        node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))
    ]
    return hashlib.sha256(_stable_ast_dump(tree).encode()).hexdigest()


class DeploymentContractTest(unittest.TestCase):
    def test_executable_structure_matches_declared_baselines(self):
        baseline = json.loads(
            (ROOT / "tests/fixtures/runtime_contract.json").read_text()
        )
        migration = json.loads(
            (ROOT / "tests/fixtures/naming_migration.json").read_text()
        )
        timing_changes = json.loads(
            (ROOT / "tests/fixtures/realtime_contract.json").read_text()
        )["modules"]
        calibration_changes = json.loads(
            (ROOT / "tests/fixtures/calibration_contract.json").read_text()
        )["modules"]
        self.assertEqual(
            set(calibration_changes),
            {
                "zhiban_runtime/hardware/device_runtime.py",
                "zhiban_runtime/control/motion_control.py",
            },
        )
        # 行为修订按顺序校验前后指纹，不覆盖原算法与时序基线。
        self.assertEqual(
            set(timing_changes),
            {"zhiban_runtime/hardware/backend.py", "zhiban_runtime/application.py"},
        )
        canonical_names = {new: old for old, new in migration["symbols"].items()}
        for relative, expected in baseline["modules"].items():
            current = ROOT / migration["modules"].get(relative, relative)
            if relative in timing_changes:
                change = timing_changes[relative]
                self.assertEqual(change["before_sha256"], expected)
                self.assertEqual(change["current_path"], str(current.relative_to(ROOT)))
                self.assertTrue(change["reason"])
                expected = change["after_sha256"]
            relative_current = str(current.relative_to(ROOT))
            if relative_current in calibration_changes:
                change = calibration_changes[relative_current]
                self.assertEqual(change["before_sha256"], expected)
                self.assertTrue(change["reason"])
                expected = change["after_sha256"]
            with self.subTest(module=relative_current):
                self.assertEqual(
                    executable_fingerprint(
                        current,
                        canonical_names,
                        PLATFORM_LABELS.get(str(current.relative_to(ROOT))),
                    ),
                    expected,
                )

    def test_platform_labels_are_exact_and_not_behavior_wildcards(self):
        for relative, labels in PLATFORM_LABELS.items():
            tree = ast.parse((ROOT / relative).read_text())
            strings = [
                node.value
                for node in ast.walk(tree)
                if isinstance(node, ast.Constant) and isinstance(node.value, str)
            ]
            for current, previous in labels.items():
                with self.subTest(module=relative, label=current):
                    self.assertEqual(strings.count(current), 1)
                    self.assertNotIn(previous, strings)

    def test_naming_migration_is_unambiguous_and_complete(self):
        migration = json.loads(
            (ROOT / "tests/fixtures/naming_migration.json").read_text()
        )
        for category in ("modules", "symbols"):
            entries = migration[category]
            self.assertEqual(len(entries), len(set(entries.values())))
            self.assertFalse(set(entries) & set(entries.values()))
        for previous, current in migration["modules"].items():
            self.assertFalse((ROOT / previous).exists(), previous)
            self.assertTrue((ROOT / current).is_file(), current)

        identifiers = set()
        for path in (ROOT / "zhiban_runtime").rglob("*.py"):
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    identifiers.add(node.name)
                elif isinstance(node, ast.Name):
                    identifiers.add(node.id)
                elif isinstance(node, ast.Attribute):
                    identifiers.add(node.attr)
        self.assertFalse(set(migration["symbols"]) & identifiers)
        self.assertTrue(set(migration["symbols"].values()) <= identifiers)

    def test_bundled_model_checksums(self):
        manifest = json.loads((ROOT / "models/manifest.json").read_text())["models"]
        self.assertEqual(
            set(manifest),
            {
                f"zhiban_{name}.onnx"
                for name in ("walk", "roller", "getup", "pick", "roulade")
            },
        )
        self.assertEqual(
            set(manifest), {p.name for p in (ROOT / "models").glob("*.onnx")}
        )
        for name, entry in manifest.items():
            with self.subTest(model=name):
                self.assertEqual(
                    hashlib.sha256((ROOT / "models" / name).read_bytes()).hexdigest(),
                    entry["sha256"],
                )
                if name != "zhiban_roller.onnx":
                    self.assertEqual(
                        entry["sha256"], entry["before_reorganization_sha256"]
                    )
                else:
                    self.assertEqual(
                        entry["renamed_metadata_keys"], ["run_path", "task_id"]
                    )

    def test_python_sources_parse_with_python38_grammar(self):
        for path in [
            ROOT / "main.py",
            *ROOT.joinpath("zhiban_runtime").rglob("*.py"),
            *ROOT.joinpath("tools").rglob("*.py"),
        ]:
            with self.subTest(module=path.name):
                ast.parse(path.read_text(), feature_version=(3, 8))

    def test_protocol_and_model_dimensions(self):
        from zhiban_runtime.inference.policy_math import (
            MOTOR_IDS,
            OBS_DIM,
            ACTION_DIM,
            POLICY_IDS,
            DEFAULT_DEG,
        )

        self.assertEqual((OBS_DIM, ACTION_DIM), (61, 14))
        self.assertEqual(len(POLICY_IDS), 14)
        self.assertEqual(len(DEFAULT_DEG), 15)
        self.assertEqual(
            MOTOR_IDS, [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
        )

    def test_resources_are_resolved_from_checkout(self):
        from zhiban_runtime.paths import PROJECT_ROOT, MODEL_DIR, WEB_DIR

        self.assertEqual(PROJECT_ROOT, ROOT)
        self.assertTrue((MODEL_DIR / "zhiban_walk.onnx").is_file())
        self.assertTrue((WEB_DIR / "index.html").is_file())


if __name__ == "__main__":
    unittest.main()
