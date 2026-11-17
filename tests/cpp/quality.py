"""Audit every package translation unit using the real ROS compile database."""

import argparse
import concurrent.futures
import json
import re
import shlex
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "linkerbot_hand_ros_cpp"


def owned(path):
    return Path(path).resolve().is_relative_to(PACKAGE)


def run(command, log, *, cwd=ROOT):
    with log.open("w") as stream:
        result = subprocess.run(
            command, cwd=cwd, stdout=stream, stderr=subprocess.STDOUT, check=False
        )
    return result.returncode


def executable(name, version=None):
    path = shutil.which(name)
    if not path:
        raise RuntimeError(f"Required tool not found: {name}")
    if version:
        result = subprocess.run(
            [path, "--version"], capture_output=True, text=True, check=True
        )
        if version not in result.stdout:
            raise RuntimeError(f"{name} requires version {version}: {result.stdout}")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    database = json.loads((args.build_dir / "compile_commands.json").read_text())
    entries = {}
    for entry in database:
        path = Path(entry["file"])
        if not path.is_absolute():
            path = Path(entry["directory"]) / path
        if owned(path):
            entries[path.resolve()] = dict(entry, file=str(path.resolve()))
    sources = set((PACKAGE / "src").rglob("*.cpp")) | set(
        (PACKAGE / "test").rglob("*.cpp")
    )
    if set(entries) != sources:
        raise RuntimeError(
            f"Incomplete compile database: missing {sources - entries.keys()}"
        )
    selected = output / "compile_commands.json"
    selected.write_text(json.dumps(list(entries.values()), indent=2) + "\n")
    formatter = executable("clang-format", "18.1.8")
    tidy = executable("clang-tidy", "18.1.8")
    cppcheck = executable("cppcheck")
    files = sorted(p for p in PACKAGE.rglob("*") if p.suffix in {".cpp", ".hpp", ".h"})
    results = {}
    results["format"] = run(
        [formatter, "--dry-run", "--Werror", *map(str, files)], output / "format.log"
    )

    def check_source(path):
        log = output / (
            path.relative_to(PACKAGE).as_posix().replace("/", "__") + ".tidy.log"
        )
        command = [tidy, str(path), "-p", str(args.build_dir.resolve())]
        command += ["--warnings-as-errors=*"]
        return str(path.relative_to(ROOT)), run(command, log)

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for path, code in pool.map(check_source, sorted(entries)):
            results[path] = code
    xml = output / "cppcheck.xml"
    with xml.open("w") as errors, (output / "cppcheck.log").open("w") as progress:
        checked = subprocess.run(
            [
                cppcheck,
                "--project=" + str(selected),
                "--enable=all",
                "--library=std",
                "--library=googletest",
                "-D__unix__=1",
                "-D__linux__=1",
                "-U_WIN32",
                "-U__APPLE__",
                "--platform=unix64",
                "--std=c++17",
                "--inline-suppr",
                "--suppress=missingIncludeSystem",
                "--xml",
                "--xml-version=2",
            ],
            stdout=progress,
            stderr=errors,
            check=False,
        )
    findings = []
    metadata = []
    tree = ET.parse(xml)
    for error in tree.findall("./errors/error"):
        if error.get("id") == "checkersReport":
            # Cppcheck emits its active-checker count as an information record,
            # even for clean input. Keep telemetry without treating it as a bug.
            metadata.append(dict(error.attrib))
            continue
        locations = error.findall("location")
        # Cppcheck's first location owns the diagnostic. Later locations can
        # point into a caller while the actual warning belongs to a dependency.
        if not locations or owned(locations[0].get("file", "")):
            findings.append(
                {
                    "id": error.get("id"),
                    "severity": error.get("severity"),
                    "message": error.get("msg"),
                    "locations": [dict(location.attrib) for location in locations],
                }
            )
    results["cppcheck"] = checked.returncode or int(bool(findings))
    # A malformed/missing report is fatal; dependency diagnostics remain in XML.
    if tree.getroot().find("errors") is None:
        raise RuntimeError("Cppcheck did not produce an errors element")
    (output / "cppcheck-owned.json").write_text(json.dumps(findings, indent=2) + "\n")
    headers = sorted(
        p for p in (PACKAGE / "include").rglob("*") if p.suffix in {".hpp", ".h"}
    )
    template = next(e for p, e in entries.items() if p.name == "commands.cpp")
    arguments = template.get("arguments") or shlex.split(template["command"])
    flags = []
    skip = False
    for item in arguments[1:]:
        if skip:
            skip = False
        elif item in {"-o", "-c"}:
            skip = True
        else:
            flags.append(item)
    for header in headers:
        probe = output / (header.stem + ".header.cpp")
        probe.write_text(f'#include "{header}"\n')
        results["header/" + header.name] = run(
            [
                arguments[0],
                *flags,
                "-fsyntax-only",
                "-Wall",
                "-Wextra",
                "-Wpedantic",
                "-Werror",
                str(probe),
            ],
            output / (header.stem + ".header.log"),
            cwd=template["directory"],
        )
    diagnostics = {}
    for log in output.glob("*.tidy.log"):
        for line in log.read_text().splitlines():
            if str(PACKAGE) in line and re.search(r": (?:warning|error):", line):
                match = re.match(
                    r"(.+):(\d+):(\d+): (warning|error): (.+) \[([^\]]+)\]", line
                )
                if match:
                    path, number, column, severity, message, checks = match.groups()
                    checks = checks.replace(",-warnings-as-errors", "")
                    diagnostics[(path, number, column, checks)] = {
                        "file": str(Path(path).relative_to(ROOT)),
                        "line": int(number),
                        "column": int(column),
                        "severity": severity,
                        "checks": checks,
                        "message": message,
                    }
    inventory = sorted(
        diagnostics.values(), key=lambda finding: (finding["file"], finding["line"])
    )
    (output / "tidy-owned.json").write_text(json.dumps(inventory, indent=2) + "\n")
    summary = {
        "mode": "full",
        "sources": len(entries),
        "headers": len(headers),
        "results": results,
        "tidy_diagnostics": dict(Counter(f["checks"] for f in inventory)),
        "cppcheck_diagnostics": dict(Counter(f["id"] for f in findings)),
        "cppcheck_metadata": metadata,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return int(any(results.values()))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(2)
