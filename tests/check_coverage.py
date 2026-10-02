#!/usr/bin/env python3
"""Require 100% LLVM line, function, and branch coverage for every reported file."""

import json
import sys
from pathlib import Path


METRICS = ("lines", "functions", "branches")


def main() -> int:
    if len(sys.argv) != 2:
        print(f"Usage: {Path(sys.argv[0]).name} <llvm-cov-export.json>", file=sys.stderr)
        return 2

    report_path = Path(sys.argv[1])
    try:
        report = json.loads(report_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"Unable to read LLVM coverage report: {error}", file=sys.stderr)
        return 2

    if not isinstance(report, dict) or report.get("type") != "llvm.coverage.json.export":
        print("Unsupported LLVM coverage export format", file=sys.stderr)
        return 2

    data = report.get("data")
    if not isinstance(data, list):
        print("LLVM coverage export has no data array", file=sys.stderr)
        return 2

    files = []
    for entry in data:
        if not isinstance(entry, dict) or not isinstance(entry.get("files"), list):
            print("Malformed LLVM coverage export data", file=sys.stderr)
            return 2
        files.extend(entry["files"])

    if not files:
        print("LLVM coverage report contains no in-scope source files", file=sys.stderr)
        return 1

    failures = []
    for file in files:
        if not isinstance(file, dict):
            failures.append("malformed source-file entry")
            continue
        filename = file.get("filename", "<unknown source>")
        summary = file.get("summary", {})
        if not isinstance(summary, dict):
            failures.append(f"{filename}: malformed coverage summary")
            continue
        for metric in METRICS:
            values = summary.get(metric)
            if not isinstance(values, dict) or "count" not in values or "covered" not in values:
                failures.append(f"{filename}: missing {metric} summary")
                continue

            count = values["count"]
            covered = values["covered"]
            if covered != count:
                failures.append(f"{filename}: {metric} coverage {covered}/{count} (required 100%)")

    if failures:
        print("LLVM coverage gate failed:")
        for failure in failures:
            print(f"  {failure}")
        return 1

    print(f"LLVM coverage gate passed: {len(files)} source files have 100% lines, functions, and branches.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
