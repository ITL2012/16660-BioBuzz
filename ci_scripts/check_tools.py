#!/usr/bin/env python3
"""
Checks that the Python tools can still read a Black Box session.

A session is one SQLite file. The robot writes it, the JVM tests write it,
and the tools in `tools/` read it — so the contract between the two sides is
worth a CI check of its own.

    python3 ci_scripts/check_tools.py
"""
import importlib.util
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "tools")
FIXTURE = os.path.join(ROOT, "blackbox_fixtures", "auto_test_session", "session.db")
SAMPLE = os.path.join(ROOT, "TeamCode", "build", "sample-session.db")


def _load_bbdb():
    """Loads tools/bbdb.py without needing it on the import path."""
    spec = importlib.util.spec_from_file_location("bbdb", os.path.join(TOOLS, "bbdb.py"))
    if spec is None or spec.loader is None:
        raise ImportError("could not load tools/bbdb.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bbdb = _load_bbdb()


def fail(message):
    print("FAIL: {}".format(message))
    sys.exit(1)


def check_fixture():
    if not os.path.exists(FIXTURE):
        fail("fixture missing: {} (run ci_scripts/generate_fixtures.py)".format(FIXTURE))

    records = bbdb.load(FIXTURE)
    if not records:
        fail("the fixture produced no records")

    kinds = {r.get("record") or r.get("event") or r.get("frame") for r in records}
    for wanted in ("state", "planner_decision"):
        if wanted not in kinds:
            fail("the fixture has no '{}' record (got {})".format(wanted, sorted(kinds)))

    events = [r for r in records if r.get("event") == "phase"]
    if not events or events[0].get("name") != "1-collect":
        fail("the fixture's event payload did not survive the round trip")

    sessions = list(bbdb.iter_sessions([FIXTURE]))
    if not sessions:
        fail("iter_sessions found nothing")
    print("ok: fixture readable ({} records, {} session(s))".format(len(records), len(sessions)))


def check_builder():
    out = os.path.join(ROOT, "build", "ci-tool-check")
    result = subprocess.run(
        [sys.executable, os.path.join(TOOLS, "blackbox_builder.py"), FIXTURE, "--out", out],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        fail("blackbox_builder.py failed:\n{}".format(result.stdout + result.stderr))
    for name in ("state.csv", "events.csv", "frames.csv", "report.txt"):
        if not os.path.exists(os.path.join(out, name)):
            fail("blackbox_builder.py did not write {}".format(name))
    print("ok: blackbox_builder.py wrote the dataset")


def check_sample():
    if not os.path.exists(SAMPLE):
        print("skip: no sample session (run the JVM tests first)")
        return

    records = bbdb.load(SAMPLE)
    frames = [r for r in records if r.get("frame") == "FRONT"]
    if not frames:
        fail("the sample session has no FRONT frames")

    data = bbdb.frame_bytes(SAMPLE, frames[0]["frame_id"])
    if not data or data[:2] != b"\xff\xd8":
        fail("the sample session's frame is not a JPEG")
    print("ok: sample session readable ({} records, {} FRONT frames)".format(
        len(records), len(frames)))


def main():
    check_fixture()
    check_builder()
    check_sample()
    print("all tool checks passed")


if __name__ == "__main__":
    main()
