# Base Planner

This branch contains a minimal executable AirSim trajectory planner. It keeps
only the task-parser API, Qwen trajectory-planner API, camera geometry, and
AirSim vehicle control. Detection, target binding, memory, semantic completion,
avoidance, recovery, candidate scoring, pending queues, Web UI, and the old
fast/slow runtime are intentionally absent.

## Runtime

```text
instruction -> MissionPlan -> navigation workflow
            -> RGB + aligned depth observation
            -> 5 x [dx, dy, dz, dyaw_deg]
            -> absolute world poses -> AirSim execution
```

Translations are relative to each preceding predicted pose. Yaw is also an
increment relative to that pose. AirSim executes position and yaw together
with `MaxDegreeOfFreedom`, so flight direction and camera direction remain
independent.

The camera adapter reads each frame's actual AirSim camera pose and FOV. It
does not assume the configured camera is horizontal or forward-facing.

## Run

Edit `config/base.yaml`, then:

```bash
pip install -r requirements.txt
python run_airsim_cli.py
python run_airsim_cli.py --instruction "Follow the road"
python run_airsim_cli.py --no-takeoff
```

The base planner has no semantic completion detector. `Mission executed`
means that every validated trajectory stage was executed without a reported
collision or API error; it does not claim that a visual target was reached.

## Structure

- `planner/domain`: immutable mission, observation, pose, trajectory, and result types.
- `planner/application`: mission validation, routing, and stage scheduling.
- `planner/workflows`: task-specific state machines; only `navigation` exists now.
- `planner/services`: trajectory conversion, validation, motion policy, and execution.
- `planner/ports`: interfaces owned by the core.
- `planner/adapters`: OpenAI-compatible APIs and AirSim implementations.
- `planner/capabilities`: contracts for optional future capabilities.
- `planner/extensions`: explicit allow-list registry for future modules.
- `config`: typed schema, loader, and the only runtime YAML.
- `tests`: unit, contract, integration, and fake-component test areas.

Future detection or memory implementations belong under dedicated adapter or
capability packages and are injected into a workflow. They must not call
AirSim directly or be dynamically imported from model output.

## Test

```bash
pytest tests
```
