"""Black Box session reader.

A session is ONE SQLite file recorded by the Robot Controller:

    blackbox/<label>_<timestamp>.db

The robot state, the world-model tracks, the planner decisions, the events
and the camera frames (as BLOBs) all live in typed tables inside that one
file. This module reads it with Python's stdlib sqlite3 — no pip install,
no extra dependency.

The `load()` function rebuilds the flat record dictionaries the analysis
tools have always consumed, so a tool that used to read `state.jsonl` now
reads the database with the same field names:

    {"t": wall-clock ms, "tn": monotonic ns, "record": "wm", ...}
    {"t": ..., "tn": ..., "event": "phase", "name": "1-collect"}
    {"t": ..., "tn": ..., "event": "planner_decision", "action": "GOTO", ...}
    {"t": ..., "tn": ..., "frame": "FRONT", "frame_id": 12}
"""

import os
import sqlite3

__all__ = [
    "iter_sessions",
    "load",
    "frames",
    "frame_bytes",
    "frame_index",
    "session_ids",
]


def _is_session(path):
    if not os.path.isfile(path):
        return False
    if not path.endswith(".db"):
        return False
    try:
        conn = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
    except sqlite3.Error:
        return False
    try:
        row = conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name='session'"
        ).fetchone()
        return row is not None
    except sqlite3.Error:
        return False
    finally:
        conn.close()


def iter_sessions(paths):
    """Yield every session database in the given files and folders."""
    for p in paths:
        if _is_session(p):
            yield p
        elif os.path.isdir(p):
            for name in sorted(os.listdir(p)):
                sub = os.path.join(p, name)
                if _is_session(sub):
                    yield sub
                elif os.path.isdir(sub):
                    for inner in sorted(os.listdir(sub)):
                        deep = os.path.join(sub, inner)
                        if _is_session(deep):
                            yield deep


def _connect(path):
    conn = sqlite3.connect("file:{}?mode=ro".format(path), uri=True)
    conn.row_factory = sqlite3.Row
    return conn


def session_ids(path):
    """The session ids stored in a database."""
    conn = _connect(path)
    try:
        return [r["session_id"] for r in conn.execute("SELECT session_id FROM session")]
    finally:
        conn.close()


def _events(conn, session_ref):
    """Every event of a session as a flat record dict."""
    rows = conn.execute(
        "SELECT * FROM event WHERE session_ref = ? ORDER BY seq, id", (session_ref,)
    ).fetchall()
    if not rows:
        return []
    fields = conn.execute("SELECT * FROM event_field").fetchall()
    by_event = {}
    for f in fields:
        by_event.setdefault(f["event_ref"], []).append(f)

    out = []
    for ev in rows:
        rec = {
            "t": ev["t_ms"],
            "tn": ev["t_nanos"],
            "event": ev["type"],
        }
        for f in by_event.get(ev["id"], []):
            value = f["value_int"]
            if value is None:
                value = f["value_num"]
            if value is None:
                value = f["value_text"]
            rec[f["key"]] = value
        out.append(rec)
    return out


def _decisions(conn, session_ref):
    out = []
    for d in conn.execute(
        "SELECT * FROM decision WHERE session_ref = ? ORDER BY t_nanos, id", (session_ref,)
    ):
        out.append({
            "t": d["t_ms"],
            "tn": d["t_nanos"],
            "event": "planner_decision",
            "source": d["source"],
            "action": d["action"],
            "targetId": d["target_id"],
            "targetLabel": d["target_label"],
            "value": d["expected_value"],
            "costSec": d["est_cost_sec"],
            "ballsHeld": d["balls_held"],
            "timeRemainingSec": d["time_remaining_sec"],
            "why": d["rationale"],
        })
    return out


def _states(conn, session_ref):
    """The robot state rows, plus the tracks seen at the same instant."""
    states = conn.execute(
        "SELECT * FROM state WHERE session_ref = ? ORDER BY t_nanos, id", (session_ref,)
    ).fetchall()
    tracks = conn.execute(
        "SELECT * FROM track WHERE session_ref = ? ORDER BY t_nanos, id", (session_ref,)
    ).fetchall()

    by_time = {}
    for tr in tracks:
        by_time.setdefault(tr["t_nanos"], []).append(tr)

    out = []
    for s in states:
        rec = {
            "t": s["t_ms"],
            "tn": s["t_nanos"],
            "record": "state",
            "poseXcm": s["pose_x_cm"],
            "poseYcm": s["pose_y_cm"],
            "headingRad": s["heading_rad"],
            "hiveX": s["hive_x_cm"],
            "hiveY": s["hive_y_cm"],
            "hiveConf": s["hive_conf"],
            "ballsHeld": s["balls_held"],
            "flyL": s["fly_l_rpm"],
            "flyR": s["fly_r_rpm"],
            "batteryV": s["battery_v"],
        }
        out.append(rec)

        groups = {"ball": [], "opponent": [], "flower": []}
        for tr in by_time.get(s["t_nanos"], []):
            groups.setdefault(tr["kind"], []).append({
                "id": tr["track_id"],
                "x": tr["x_cm"],
                "y": tr["y_cm"],
                "lbl": tr["label"],
                "conf": tr["confidence"],
                "vx": tr["vel_x"],
                "vy": tr["vel_y"],
                "intent": tr["intent"],
                "intentConf": tr["intent_conf"],
            })
        if any(groups.values()):
            wm = dict(rec)
            wm["record"] = "wm"
            wm["balls"] = groups.get("ball", [])
            wm["opp"] = groups.get("opponent", [])
            wm["flowers"] = groups.get("flower", [])
            out.append(wm)
    return out


def load(path, session_id=None):
    """Every record of a session, in time order."""
    conn = _connect(path)
    try:
        sessions = conn.execute("SELECT * FROM session ORDER BY created_at_ms DESC, id DESC").fetchall()
        if not sessions:
            return []
        header = None
        if session_id is not None:
            for s in sessions:
                if s["session_id"] == session_id:
                    header = s
                    break
        if header is None:
            header = sessions[0]
        ref = header["id"]

        records = []
        records.extend(_states(conn, ref))
        records.extend(_events(conn, ref))
        records.extend(_decisions(conn, ref))

        for f in conn.execute(
            "SELECT id, capture_nanos, camera, t_ms FROM frame WHERE session_ref = ? "
            "ORDER BY capture_nanos, id",
            (ref,),
        ):
            records.append({
                "t": f["t_ms"],
                "tn": f["capture_nanos"],
                "frame": f["camera"],
                "frame_id": f["id"],
            })

        records.sort(key=lambda r: (r.get("tn") or 0, r.get("record") or r.get("event") or ""))
        return records
    finally:
        conn.close()


def frame_index(path, camera="FRONT", session_id=None):
    """(wall-clock ms, frame id) for every frame of one camera."""
    conn = _connect(path)
    try:
        sessions = conn.execute("SELECT * FROM session ORDER BY created_at_ms DESC").fetchall()
        if not sessions:
            return []
        header = sessions[0]
        if session_id is not None:
            for s in sessions:
                if s["session_id"] == session_id:
                    header = s
                    break
        rows = conn.execute(
            "SELECT id, capture_nanos, t_ms FROM frame WHERE session_ref = ? AND camera = ? "
            "ORDER BY capture_nanos, id",
            (header["id"], camera),
        ).fetchall()
        return [(r["t_ms"], r["id"]) for r in rows]
    finally:
        conn.close()


def frame_bytes(path, frame_id):
    """The JPEG bytes of one frame."""
    conn = _connect(path)
    try:
        row = conn.execute("SELECT data FROM frame WHERE id = ?", (frame_id,)).fetchone()
        if row is None:
            return None
        return row["data"]
    finally:
        conn.close()


def frames(path, camera="FRONT", session_id=None):
    """Every frame of one camera as (wall-clock ms, jpeg bytes)."""
    conn = _connect(path)
    try:
        sessions = conn.execute("SELECT * FROM session ORDER BY created_at_ms DESC").fetchall()
        if not sessions:
            return []
        header = sessions[0]
        if session_id is not None:
            for s in sessions:
                if s["session_id"] == session_id:
                    header = s
                    break
        rows = conn.execute(
            "SELECT capture_nanos, t_ms, data FROM frame WHERE session_ref = ? AND camera = ? "
            "ORDER BY capture_nanos, id",
            (header["id"], camera),
        ).fetchall()
        return [(r["t_ms"], r["data"]) for r in rows]
    finally:
        conn.close()
