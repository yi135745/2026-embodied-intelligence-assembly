# Project agent rules

## Mandatory project skill

- Use `$aubo-competition-project` before changing architecture, public module interfaces, task flow, calibration/pose/offset data, hardware behavior, or the official debugging workflow.
- Also use it when diagnosing cross-module behavior or when a proposed simplification could create a second source of truth.
- A request marked as discussion, review, or “暂不修改” authorizes inspection only. Do not edit runtime or formal calibration data until the user approves implementation.

## Verification

- Preserve unrelated working-tree changes; this repository commonly contains live site data and unfinished tuning work.
- For Python runtime changes, run `python -m compileall -q modules tools test` and `python -m unittest discover -s test -q` unless the user limits validation.
- Offline tests do not authorize or prove robot motion, Tool IO, camera capture, paid API calls, or voice hardware operation.
