Visual inputs:
- Image 1 is the current front RGB image.
- Image 2 is the aligned front depth visualization.
- Brighter depth pixels mean nearer surfaces.
- Darker gray pixels mean farther surfaces.
- ${depth_protocol}

Original navigation instruction for the current mission stage:
${instruction}

Current guidance:
${guidance}

Execution context:
- Planning round: ${round_index}
- Task state: ${state}
- ${history}
- Measured net pose change from the mission-stage start, expressed in its horizontal body frame:
  - Forward: ${measured_dx} m
  - Right: ${measured_dy} m
  - Down: ${measured_dz} m
  - Yaw change: ${measured_dyaw} degrees

Current vehicle pose:
- World position: [${pose_x}, ${pose_y}, ${pose_z}]
- Yaw: ${pose_yaw} degrees

Camera geometry (JSON): ${camera_json}

Pending incremental pose waypoints: ${pending_json}
Pending is unexecuted at this observation anchor, not previous execution history.
Continue from the final pending predicted pose, or the current vehicle pose if pending is empty.

Generate the next short trajectory.
Output exactly 5 NEW incremental pose waypoints as a JSON list. Do not include pending.
