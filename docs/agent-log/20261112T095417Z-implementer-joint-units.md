ID: 20261112T095417Z-implementer-joint-units

## Problem description

Provide degrees and radians in the shared web joint controls while preserving physical values, the degree-based HTTP API and unchanged-plan readiness.

## How did you manage to tackle the problem?

Read AGENTS.md, architecture/software/safety/roadmap documents, arm_web package metadata, current page and web_viewer/joint_planning API conversion paths. Changed src/arm_web/web/index.html to add a joint-only selector and unit-aware positions, gauges, target bounds/accessibility labels and jog speed. Cached canonical degree values prevent repeated conversion drift and leave blank/nonfinite input invalid. Only user target edits/manual commands invalidate plans; display switches preserve readiness and existing response-order safeguards. Updated README.md and docs/06-ros2-software.md, preserving their CRLF line endings. Added test/test_web_units.py and test/web_units_check.js under src/arm_web to exercise the actual page script in a bounded Node VM/DOM fixture. No backend, TCP semantics, hardware, existing contribution records or protected files changed.

## Did it solve the problem? If not what was wrong?

Node script checks pass: 100-roundtrip conversions, zero/negative/exact/near limits, blank/nonfinite/out-of-range rejection, current copying, API plan/direct/jog payloads in both units, speed preservation, unchanged-plan readiness, edits, stale state and response races. With ROS Lyrical sourced, direct pytest (PYTHONPATH including src/arm_web) passed 12 tests; the first invocation without that path had collection import failures. colcon build --symlink-install --packages-up-to arm_web passed with the existing setuptools tests_require warning. colcon test --packages-select arm_web exited 5 with NO TESTS RAN; current packaging does not collect these pytest tests. colcon test-result --verbose reports 0 tests/0 errors/0 failures, so it supplies no test evidence. git -c core.whitespace=cr-at-eol diff --check passes; ordinary diff --check flags the preserved CRLF documentation lines. Independent reviewer reported no behavior defect after actual-script checks. Browser rendering/native input sanitization, live ROS transport, simulation dynamics and physical safety were not validated; no browser launch or hardware action was attempted.
