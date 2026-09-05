"""Regression tests for composed YAML configuration files."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import yaml

from config import load_config_file


class ConfigCompositionTests(unittest.TestCase):
    def test_default_memory_section_matches_external_source(self):
        root = Path(__file__).resolve().parents[1]
        default_path = root / "config" / "default.yaml"
        memory_path = root / "config" / "functions" / "memory.yaml"

        raw_default = yaml.safe_load(default_path.read_text(encoding="utf-8"))
        raw_memory = yaml.safe_load(memory_path.read_text(encoding="utf-8"))
        resolved = load_config_file(str(default_path))

        self.assertNotIn("MEMORY", raw_default["FUNCTIONS"])
        self.assertNotIn("INCLUDES", resolved)
        self.assertEqual(
            resolved["FUNCTIONS"]["MEMORY"],
            raw_memory["FUNCTIONS"]["MEMORY"],
        )

    def test_includes_are_relative_and_current_file_wins_recursively(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "base.yaml").write_text(
                "SECTION:\n  KEEP: 1\n  OVERRIDE: base\n",
                encoding="utf-8",
            )
            (root / "main.yaml").write_text(
                "INCLUDES: base.yaml\nSECTION:\n  OVERRIDE: main\nEXTRA: true\n",
                encoding="utf-8",
            )

            resolved = load_config_file(str(root / "main.yaml"))

        self.assertEqual(
            resolved,
            {"SECTION": {"KEEP": 1, "OVERRIDE": "main"}, "EXTRA": True},
        )

    def test_include_cycles_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "a.yaml").write_text("INCLUDES: b.yaml\n", encoding="utf-8")
            (root / "b.yaml").write_text("INCLUDES: a.yaml\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "cyclic config include"):
                load_config_file(str(root / "a.yaml"))


if __name__ == "__main__":
    unittest.main()
