# Graph Report - fast_slow  (2026-09-02)

## Corpus Check
- Corpus is ~25,204 words - fits in a single context window. You may not need a graph.

## Summary
- 279 nodes · 747 edges · 11 communities (10 shown, 1 thin omitted)
- Extraction: 95% EXTRACTED · 5% INFERRED · 0% AMBIGUOUS · INFERRED: 36 edges (avg confidence: 0.91)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Trajectory Guards
- Completion Evaluation
- Async Completion Pipeline
- Runtime Orchestration
- Above Navigation
- Planning Controller
- Path Streaming
- Target Memory Binding
- Watchdog Synchronization
- AirSim Action Quantization
- Package Metadata

## God Nodes (most connected - your core abstractions)
1. `RuntimeObjects` - 41 edges
2. `CompletionPipeline` - 40 edges
3. `run_fast_slow_loop()` - 40 edges
4. `_handle_distance_completion_trigger()` - 25 edges
5. `_update_target_pose_from_bundle()` - 21 edges
6. `_debug_print()` - 20 edges
7. `_is_above_stage()` - 20 edges
8. `FastSlowController` - 19 edges
9. `_handle_background_target_lost()` - 19 edges
10. `_above_stage_state()` - 18 edges

## Surprising Connections (you probably didn't know these)
- `_prebind_target_from_bundle()` --uses--> `DetectionDepthBundle`  [INFERRED]
  runtime.py → completion_pipeline.py
- `_record_locked_target_snapshot()` --uses--> `DetectionDepthBundle`  [INFERRED]
  runtime.py → completion_pipeline.py
- `_update_target_pose_from_bundle()` --uses--> `DetectionDepthBundle`  [INFERRED]
  runtime.py → completion_pipeline.py
- `_above_stage_state()` --uses--> `CompletionPipeline`  [INFERRED]
  runtime.py → completion_pipeline.py
- `_cached_target_distance()` --uses--> `CompletionPipeline`  [INFERRED]
  runtime.py → completion_pipeline.py

## Import Cycles
- None detected.

## Communities (11 total, 1 thin omitted)

### Community 0 - "Trajectory Guards"
Cohesion: 0.09
Nodes (43): _active_queue_overshoots_locked_surface(), _apply_above_roof_acquisition_path_guard(), _apply_bearing_path_guard(), _apply_memory_path_guard(), _apply_obstacle_path_guard(), _bearing_only_active(), _candidate_trace_text(), _clamp() (+35 more)

### Community 1 - "Completion Evaluation"
Cohesion: 0.09
Nodes (41): _best_detection_from_list(), _capture_completion_depth(), _capture_fresh_completion_frames(), _capture_fresh_rgb_frames(), _complete_stage_from_memory(), _complete_stage_from_vlm(), _confirm_completion_now(), _debug_logs_enabled() (+33 more)

### Community 2 - "Async Completion Pipeline"
Cohesion: 0.14
Nodes (11): CompletionPipeline, CompletionPipelineEvent, CompletionPipelineJob, DetectionDepthBundle, Any, ThreadPoolExecutor, Asynchronous detector/depth observation pipeline for the fast-slow runtime. In…, Pick the view whose depth drives stop/confirm timing. This is intentionally… (+3 more)

### Community 3 - "Runtime Orchestration"
Cohesion: 0.12
Nodes (28): CameraRecordingOptions, _active_relocalization_session_id(), _build_locked_relocalization_validator(), _build_runtime_objects(), _bump_lock_generation(), _bump_stage_generation(), _bundle_from_relocalization_result(), _completion_retry_waiting() (+20 more)

### Community 4 - "Above Navigation"
Cohesion: 0.14
Nodes (28): _above_max_allowed_world_z(), _above_pre_roof_facade_clearance_context(), _above_queue_violation_reason(), _above_roof_candidate_ready(), _above_stage_state(), AboveStageRuntimeState, _apply_above_altitude_path_guard(), _capture_and_submit_plan() (+20 more)

### Community 5 - "Planning Controller"
Cohesion: 0.11
Nodes (12): ContinuousPlanningDecision, ExecutionDecision, FastSlowController, PlanningJob, Any, Fast-slow controller for asynchronous sliding-window planning. The controller…, Submit a slow planning job when the pending queue is valid., Estimate sequential waypoint travel times using commanded velocity. (+4 more)

### Community 6 - "Path Streaming"
Cohesion: 0.14
Nodes (13): Drop any pending slow-planner result without touching executed history., ContinuousPathStream, _distance(), _passed_target(), PathStreamPoll, _point3(), Non-blocking AirSim path streaming for sliding-window execution., Keep one AirSim path active and rewrite it when the local tail grows. (+5 more)

### Community 7 - "Target Memory Binding"
Cohesion: 0.13
Nodes (22): _attach_depth_to_detection_lists(), _bind_view_relative_stage(), _bootstrap_mission_memory(), _caption_for_stage(), _detect_future_memory_snapshot(), FutureMemoryObservation, FutureMemoryScanJob, _maybe_submit_future_memory_scan() (+14 more)

### Community 8 - "Watchdog Synchronization"
Cohesion: 0.10
Nodes (15): Whether a submitted plan still needs to be polled or discarded., _cached_target_distance(), _CompletionRadiusWatchdog, _continuous_path_velocity(), _drop_path_points_behind_vehicle(), Turn a very-near collision into stage-specific facade contact evidence., Read the cheap cached target distance independently of waypoint events., Turn toward a locked target before a ForwardOnly path is planned/issued. (+7 more)

### Community 9 - "AirSim Action Quantization"
Cohesion: 0.25
Nodes (11): _apply_airsim_vertical_path_quantization(), _execute_action_stage(), _minimum_airsim_climb_command_m(), _minimum_airsim_vertical_command_m(), _normalize_direct_vertical_action_m(), _quantized_climb_body_z(), Return a vertical displacement that is strictly greater than 5 m., Return the configured climb minimum without weakening AirSim's limit. (+3 more)

## Knowledge Gaps
- **1 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `CompletionPipeline` connect `Async Completion Pipeline` to `Trajectory Guards`, `Completion Evaluation`, `Runtime Orchestration`, `Above Navigation`, `Target Memory Binding`, `Watchdog Synchronization`?**
  _High betweenness centrality (0.180) - this node is a cross-community bridge._
- **Why does `FastSlowController` connect `Planning Controller` to `Watchdog Synchronization`, `Trajectory Guards`, `Runtime Orchestration`, `Path Streaming`?**
  _High betweenness centrality (0.166) - this node is a cross-community bridge._
- **Why does `run_fast_slow_loop()` connect `Runtime Orchestration` to `Trajectory Guards`, `Completion Evaluation`, `Async Completion Pipeline`, `Above Navigation`, `Path Streaming`, `Target Memory Binding`, `Watchdog Synchronization`, `AirSim Action Quantization`?**
  _High betweenness centrality (0.148) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `RuntimeObjects` (e.g. with `CompletionPipeline` and `FastSlowController`) actually correct?**
  _`RuntimeObjects` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 17 inferred relationships involving `CompletionPipeline` (e.g. with `_above_stage_state()` and `_cached_target_distance()`) actually correct?**
  _`CompletionPipeline` has 17 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `run_fast_slow_loop()` (e.g. with `CompletionPipeline` and `ContinuousPathStream`) actually correct?**
  _`run_fast_slow_loop()` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `_handle_distance_completion_trigger()` (e.g. with `CompletionPipeline` and `_capture_completion_depth()`) actually correct?**
  _`_handle_distance_completion_trigger()` has 2 INFERRED edges - model-reasoned connections that need verification._