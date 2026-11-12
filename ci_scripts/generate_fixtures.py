#!/usr/bin/env python3
"""
Generates a minimal BlackBox test fixture for CI regression testing.

A Black Box session is ONE SQLite file, so the fixture is one .db with the
same tables the robot writes (see TeamCode/src/main/java/.../sessiondb.kt).

The DDL below must stay in step with SessionSchema.DDL — the JVM test
SessionSchemaTest compares the two and fails if they drift apart.

    python3 ci_scripts/generate_fixtures.py
    -> blackbox_fixtures/auto_test_session/session.db
"""
import os
import sqlite3

DDL = [
    """
    CREATE TABLE IF NOT EXISTS session (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        session_id    TEXT    NOT NULL UNIQUE,
        label         TEXT    NOT NULL DEFAULT '',
        schema_version INTEGER NOT NULL DEFAULT 1,
        created_at_ms INTEGER NOT NULL DEFAULT 0,
        started_nanos INTEGER NOT NULL DEFAULT 0,
        ended_nanos   INTEGER NOT NULL DEFAULT 0,
        notes         TEXT    NOT NULL DEFAULT ''
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS state (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        session_ref   INTEGER NOT NULL,
        t_nanos       INTEGER NOT NULL,
        t_ms          INTEGER NOT NULL,
        pose_x_cm     REAL    NOT NULL DEFAULT 0,
        pose_y_cm     REAL    NOT NULL DEFAULT 0,
        heading_rad   REAL    NOT NULL DEFAULT 0,
        hive_x_cm     REAL,
        hive_y_cm     REAL,
        hive_conf     REAL    NOT NULL DEFAULT 0,
        balls_held    INTEGER NOT NULL DEFAULT -1,
        fly_l_rpm     REAL    NOT NULL DEFAULT -1,
        fly_r_rpm     REAL    NOT NULL DEFAULT -1,
        battery_v     REAL    NOT NULL DEFAULT -1
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS track (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        session_ref   INTEGER NOT NULL,
        t_nanos       INTEGER NOT NULL,
        kind          TEXT    NOT NULL,
        track_id      INTEGER NOT NULL,
        x_cm          REAL    NOT NULL DEFAULT 0,
        y_cm          REAL    NOT NULL DEFAULT 0,
        label         TEXT    NOT NULL DEFAULT '',
        confidence    REAL    NOT NULL DEFAULT 0,
        vel_x         REAL    NOT NULL DEFAULT 0,
        vel_y         REAL    NOT NULL DEFAULT 0,
        intent        TEXT    NOT NULL DEFAULT '',
        intent_conf   REAL    NOT NULL DEFAULT 0
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS decision (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        session_ref   INTEGER NOT NULL,
        t_nanos       INTEGER NOT NULL,
        t_ms          INTEGER NOT NULL,
        sim_time_sec  REAL    NOT NULL DEFAULT 0,
        action        TEXT    NOT NULL DEFAULT '',
        target_id     INTEGER NOT NULL DEFAULT -1,
        target_label  TEXT    NOT NULL DEFAULT '',
        expected_value REAL   NOT NULL DEFAULT 0,
        est_cost_sec  REAL    NOT NULL DEFAULT 0,
        balls_held    INTEGER NOT NULL DEFAULT 0,
        time_remaining_sec REAL NOT NULL DEFAULT 0,
        robot_x_cm    REAL    NOT NULL DEFAULT 0,
        robot_y_cm    REAL    NOT NULL DEFAULT 0,
        rationale     TEXT    NOT NULL DEFAULT '',
        source        TEXT    NOT NULL DEFAULT ''
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS event (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        session_ref   INTEGER NOT NULL,
        seq           INTEGER NOT NULL DEFAULT 0,
        t_nanos       INTEGER NOT NULL,
        t_ms          INTEGER NOT NULL,
        type          TEXT    NOT NULL
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS event_field (
        event_ref     INTEGER NOT NULL,
        key           TEXT    NOT NULL,
        value_text    TEXT,
        value_num     REAL,
        value_int     INTEGER,
        PRIMARY KEY (event_ref, key)
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS frame (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        session_ref   INTEGER NOT NULL,
        capture_nanos INTEGER NOT NULL,
        t_ms          INTEGER NOT NULL DEFAULT 0,
        camera        TEXT    NOT NULL,
        bytes         INTEGER NOT NULL DEFAULT 0,
        width_px      INTEGER NOT NULL DEFAULT 0,
        height_px     INTEGER NOT NULL DEFAULT 0,
        data          BLOB
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS golden (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        name          TEXT    NOT NULL UNIQUE,
        session_id    TEXT    NOT NULL DEFAULT '',
        created_at_ms INTEGER NOT NULL DEFAULT 0,
        decision_count INTEGER NOT NULL DEFAULT 0,
        tolerance_sec REAL    NOT NULL DEFAULT 0.5,
        notes         TEXT    NOT NULL DEFAULT ''
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS golden_decision (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        golden_ref    INTEGER NOT NULL,
        seq           INTEGER NOT NULL,
        sim_time_sec  REAL    NOT NULL DEFAULT 0,
        action        TEXT    NOT NULL DEFAULT '',
        target_id     INTEGER NOT NULL DEFAULT -1,
        target_label  TEXT    NOT NULL DEFAULT '',
        expected_value REAL   NOT NULL DEFAULT 0,
        est_cost_sec  REAL    NOT NULL DEFAULT 0,
        balls_held    INTEGER NOT NULL DEFAULT 0,
        time_remaining_sec REAL NOT NULL DEFAULT 0,
        robot_x_cm    REAL    NOT NULL DEFAULT 0,
        robot_y_cm    REAL    NOT NULL DEFAULT 0,
        rationale     TEXT    NOT NULL DEFAULT ''
    )
    """,
]

INDEXES = [
    "CREATE INDEX IF NOT EXISTS idx_state_session ON state(session_ref, t_nanos)",
    "CREATE INDEX IF NOT EXISTS idx_track_session ON track(session_ref, t_nanos)",
    "CREATE INDEX IF NOT EXISTS idx_decision_session ON decision(session_ref, t_nanos)",
    "CREATE INDEX IF NOT EXISTS idx_event_session ON event(session_ref, seq)",
    "CREATE INDEX IF NOT EXISTS idx_frame_session ON frame(session_ref, capture_nanos)",
    "CREATE INDEX IF NOT EXISTS idx_golden_decision ON golden_decision(golden_ref, seq)",
]

BASE_MS = 1700000000000
NANOS_0 = 1000000000


def main():
    out_dir = os.path.join("blackbox_fixtures", "auto_test_session")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "session.db")
    if os.path.exists(path):
        os.remove(path)

    conn = sqlite3.connect(path)
    try:
        for statement in DDL + INDEXES:
            conn.execute(statement)

        cur = conn.execute(
            "INSERT INTO session (session_id, label, schema_version, created_at_ms, started_nanos, notes) "
            "VALUES (?, ?, ?, ?, ?, ?)",
            ("auto_test_session", "auto", 1, BASE_MS, NANOS_0, "CI fixture"),
        )
        ref = cur.lastrowid

        poses = [
            (NANOS_0, 0.0, 0.0, 0.0, 150.0, 0.0, 0.8, 0),
            (NANOS_0 + 1000000000, 10.0, 0.0, 0.0, 150.0, 0.0, 0.8, 0),
            (NANOS_0 + 2000000000, 20.0, 5.0, 0.1, 150.0, 0.0, 0.8, 1),
        ]
        for i, (tn, px, py, hd, hx, hy, hc, held) in enumerate(poses):
            conn.execute(
                "INSERT INTO state (session_ref, t_nanos, t_ms, pose_x_cm, pose_y_cm, heading_rad, "
                "hive_x_cm, hive_y_cm, hive_conf, balls_held, fly_l_rpm, fly_r_rpm, battery_v) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (ref, tn, BASE_MS + i * 1000, px, py, hd, hx, hy, hc, held, -1.0, -1.0, 12.8),
            )

        # One world-model track: a pollen ball on the field.
        conn.execute(
            "INSERT INTO track (session_ref, t_nanos, kind, track_id, x_cm, y_cm, label, confidence) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (ref, NANOS_0 + 1000000000, "ball", 1, 50.0, 10.0, "Pollen", 0.8),
        )

        # One planner decision.
        conn.execute(
            "INSERT INTO decision (session_ref, t_nanos, t_ms, sim_time_sec, action, target_id, "
            "target_label, expected_value, est_cost_sec, balls_held, time_remaining_sec, robot_x_cm, "
            "robot_y_cm, rationale, source) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (ref, NANOS_0 + 2000000000, BASE_MS + 2000, 1.5, "COLLECT_BALL", 1, "Pollen",
             0.8, 2.5, 0, 28.0, 20.0, 5.0, "best density", "auto"),
        )

        # One event with a typed payload.
        ev = conn.execute(
            "INSERT INTO event (session_ref, seq, t_nanos, t_ms, type) VALUES (?, ?, ?, ?, ?)",
            (ref, 0, NANOS_0, BASE_MS, "phase"),
        ).lastrowid
        conn.execute(
            "INSERT INTO event_field (event_ref, key, value_text) VALUES (?, ?, ?)",
            (ev, "name", "1-collect"),
        )

        conn.commit()
    finally:
        conn.close()

    print("Test fixture written to {}".format(path))


if __name__ == "__main__":
    main()
