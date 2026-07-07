# Rewrite `stages/run_maze_v12.py` — clean slate

You're working in an EV3 maze-solving robot repo. One file — `stages/run_maze_v12.py` —
has grown to ~1300 lines through many incremental AI edits and is now hard to change.
Everything else in the repo is fine. **Your job is to rewrite that one file from scratch:
same behavior, radically simpler code.** You are the reset.

## Source of truth

- **`run_maze_v12_notes.md`** (Korean) — what the robot must do. Authoritative for behavior.
- **`정리.md`** (Korean) — pinmap, color codes, reflectance thresholds, tuning defaults.
- **`stages/run_maze_v12.py`** (current) — read it to understand behavior, then throw the
  *structure* away. It's the bloat you're replacing; do not port its shape or its helpers.

## The platform is not negotiable

- Runs on a LEGO EV3 brick under **ev3dev2, Python 3.5**. No f-strings — use `.format()`.
- `import ev3dev2` **only inside `run()`**, never at module top. A PC without ev3dev2 must
  still `python3 -m py_compile stages/run_maze_v12.py lib/*.py` cleanly.
- No BACK button input. Stop = network stop / Ctrl-C. Restart = network `reset` action.
- Run entry point stays: `python3 stages/run_maze_v12.py`.

## Reuse the infra, don't rebuild it

These libs work — call them, keep the maze/driving/PD logic in the one file:

- `lib.hardware.Ev3Hardware` — `drive`/`drive_raw`/`stop`, `reset_encoders`/`read_encoders`/
  `enc_avg`, `read_left_reflect`/`read_right_reflect`, `read_center_color_now`/
  `read_center_color`, `beep_ok`. The robot also needs a **gripper (outC MediumMotor)** and
  an **ultrasonic sensor (in4)**; the current file bolts these on at runtime with a
  monkeypatch (`attach_v12_hardware`). Don't. Add them to `lib/hardware.py` properly instead.
- `lib.shared_params.SharedParams` — live-tunable params (operator tunes on the real robot).
- `lib.telemetry.Telemetry`, `lib.decision_log.DecisionLog`, `lib.tuning_server.TuningServer`
  — the dashboard, telemetry frames, and decision log. Keep them wired; they're how the robot
  is debugged in the field. Trim the param set to what the code actually uses.

## Design bar

- Simple, linear, obvious. Prefer flat control flow over clever abstraction. Someone changing
  a threshold, a turn angle, or a marker rule should find exactly one place to do it.
- Every function and every param earns its place. No dead code, no speculative knobs, no
  defensive shims for cases that can't happen.
- Keep it in the ballpark of the behavior notes — don't invent new features, don't drop
  documented ones (immediate-stop color markers, PD-off slow re-confirm at suspected nodes,
  the reset session loop, the explore→return state machine).

## Done means

- `stages/run_maze_v12.py` rewritten and much shorter, `lib/hardware.py` cleanly extended for
  gripper + ultrasonic, monkeypatch gone.
- `python3 -m py_compile stages/run_maze_v12.py lib/*.py` passes.
- Behavior matches `run_maze_v12_notes.md`.

Read the two Korean docs and the current file first, then design the structure before writing.
