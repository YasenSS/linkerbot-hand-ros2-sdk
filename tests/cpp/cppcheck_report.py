"""Preserve CMake's escaped definitions when Cppcheck imports the ROS database."""

import json
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


def main():
    args = sys.argv[1:]
    with tempfile.TemporaryDirectory(prefix="ros-cppcheck-") as temporary:
        for index, argument in enumerate(args):
            if argument.startswith("--project="):
                source = Path(argument.split("=", 1)[1])
                destination = Path(temporary) / "compile_commands.json"
                destination.write_text(
                    json.dumps(command_database(json.loads(source.read_text())))
                )
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
                "--library=googletest",
                "--suppress=missingIncludeSystem",
                *args,
            ],
            check=False,
        ).returncode


if __name__ == "__main__":
    raise SystemExit(main())
