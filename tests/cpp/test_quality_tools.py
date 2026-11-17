"""Regression checks for the strict, complete Report Card acceptance gate."""

import copy
import subprocess
import tempfile
import unittest
from pathlib import Path

from check_report_complete import validate
from cppcheck_report import command_database


class ReportAcceptanceTests(unittest.TestCase):
    def test_framework_entrypoints_do_not_hide_dead_helpers(self):
        source = """
class Runtime {
public:
    virtual void on_configure();
    static void SetUpTestSuite();
};
void Runtime::on_configure() {}
void Runtime::SetUpTestSuite() {}
static void never_called() {}
int main() { return 0; }
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "callbacks.cpp"
            path.write_text(source)
            result = subprocess.run(
                [
                    "cppcheck",
                    "--enable=all",
                    "--library=" + str(Path(__file__).with_name("ros-framework.cfg")),
                    str(path),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("The function 'never_called' is never used", result.stderr)
        self.assertNotIn("The function 'on_configure' is never used", result.stderr)
        self.assertNotIn("The function 'SetUpTestSuite' is never used", result.stderr)

    def test_cppcheck_preserves_escaped_cmake_definitions_without_mutation(self):
        entry = {
            "directory": "/build",
            "file": "/source/driver.cpp",
            "command": 'c++ -DNAME=\\"driver\\" -c /source/driver.cpp',
            "arguments": ["c++", '-DNAME="driver"', "-c", "/source/driver.cpp"],
        }
        prepared = command_database([entry])
        self.assertEqual(prepared[0]["command"], entry["command"])
        self.assertNotIn("arguments", prepared[0])
        self.assertIn("arguments", entry)
        ros_entry = dict(entry, command=entry["command"] + " -isystem /opt/ros/include")
        self.assertEqual(
            command_database([ros_entry])[0]["command"],
            entry["command"] + " -isystem /opt/ros/include",
        )
        self.assertIn("-isystem", ros_entry["command"])
        with self.assertRaises(ValueError):
            command_database([{"file": "/source/driver.cpp", "arguments": []}])

    def setUp(self):
        self.report = {
            "mode": "full",
            "analysis_complete": True,
            "grade": "A+",
            "score": 100,
            "status": "passed",
            "checks": [
                {
                    "tool": tool,
                    "status": "passed",
                    "analyzed": 9,
                    "total": 9,
                    "diagnostics": [],
                }
                for tool in ("clang-format", "cppcheck", "clang-tidy")
            ],
        }

    def test_accepts_only_complete_clean_aplus(self):
        self.assertEqual(validate(self.report, require_aplus=True), 0)
        for key, value in (
            ("mode", "quick"),
            ("analysis_complete", False),
            ("grade", "A"),
            ("score", 99),
            ("status", "findings"),
        ):
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate(dict(self.report, **{key: value}), require_aplus=True)

    def test_rejects_warning_and_information_diagnostics(self):
        for severity in ("warning", "information", "error"):
            invalid = copy.deepcopy(self.report)
            invalid["checks"][1]["diagnostics"].append({"severity": severity})
            with self.subTest(severity=severity), self.assertRaises(ValueError):
                validate(invalid, require_aplus=True)

    def test_rejects_missing_duplicate_skipped_and_incomplete_tools(self):
        variants = []
        missing = copy.deepcopy(self.report)
        missing["checks"].pop()
        variants.append(missing)
        duplicate = copy.deepcopy(self.report)
        duplicate["checks"][0]["tool"] = "cppcheck"
        variants.append(duplicate)
        for key, value in (
            ("status", "skipped"),
            ("status", "error"),
            ("analyzed", 8),
            ("total", 0),
            ("analyzed", True),
        ):
            invalid = copy.deepcopy(self.report)
            invalid["checks"][0][key] = value
            variants.append(invalid)
        for invalid in variants:
            with self.subTest(report=invalid), self.assertRaises(ValueError):
                validate(invalid, require_aplus=True)


if __name__ == "__main__":
    unittest.main()
