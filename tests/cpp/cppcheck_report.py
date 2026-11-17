"""Preserve CMake's escaped definitions when Cppcheck imports the ROS database."""

import json
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path


def command_database(entries):
    result = []
    for original in entries:
        entry = dict(original)
        if not entry.get("command"):
            raise ValueError("Cppcheck report requires shell-escaped command entries")
        entry.pop("arguments", None)
        result.append(entry)
    return result


def compiler_definitions(entries):
    compiler = shlex.split(entries[0]["command"])[0]
    result = subprocess.run(
        [compiler, "-std=c++17", "-dM", "-E", "-x", "c++", "-include", "cstddef", "-"],
        input="",
        capture_output=True,
        text=True,
        check=True,
    )
    needed = {
        "__GNUC__",
        "__GNUC_MINOR__",
        "__GLIBCXX__",
        "_GLIBCXX_RELEASE",
        "_LIBCPP_VERSION",
        "__clang__",
        "__clang_major__",
    }
    definitions = []
    for line in result.stdout.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3 and parts[1] in needed:
            definitions.append("-D" + parts[1] + "=" + parts[2])
    return definitions


def main():
    args = sys.argv[1:]
    definitions = []
    with tempfile.TemporaryDirectory(prefix="ros-cppcheck-") as temporary:
        for index, argument in enumerate(args):
            if argument.startswith("--project="):
                source = Path(argument.split("=", 1)[1])
                destination = Path(temporary) / "compile_commands.json"
                entries = command_database(json.loads(source.read_text()))
                definitions = compiler_definitions(entries)
                destination.write_text(json.dumps(entries))
                args[index] = "--project=" + str(destination)
        return subprocess.run(
            [
                "cppcheck",
                "--platform=unix64",
                "-D__unix__=1",
                "-D__linux__=1",
                "-U_WIN32",
                "-U__APPLE__",
                "--library=std",
                "--library=" + str(Path(__file__).with_name("ros-framework.cfg")),
                "--suppress=missingIncludeSystem",
                *definitions,
                *args,
            ],
            check=False,
        ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
