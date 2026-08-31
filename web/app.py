"""Flask application factory and route definitions."""
import json, time, os
from flask import Flask, Response, request, send_from_directory

_frontend_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend")


def create_app(state, camera_recorder=None):
    app = Flask(__name__)

    @app.route("/")
    def index():
        return send_from_directory(_frontend_dir, "index.html")

    @app.route("/<path:path>")
    def static_files(path):
        return send_from_directory(_frontend_dir, path)

    @app.route("/task", methods=["POST"])
    def update_task():
        from flask import request
        data = request.get_json(force=True)
        if data and "task" in data:
            state.update(
                task=data["task"],
                step=0,
                task_done=False,
                reasoning="",
                reasoning_summary="",
                scene_analysis="",
                trajectory_queue=[],
                qwen_waypoints=[],
                trajectory_candidates=[],
                selected_trajectory={},
                candidates=[],
                selected_actions=[],
                error="",
            )
            return {"status": "ok", "task": data["task"]}
        return {"status": "error"}, 400

    @app.route("/depth_frame")
    def depth_frame():
        png = state.get_depth_frame()
        if not png:
            return Response("depth frame not ready", 503,
                            headers={"Cache-Control": "no-cache"})
        return Response(png, mimetype="image/png",
                        headers={"Cache-Control": "no-cache"})

    @app.route("/debug_state")
    def debug_state():
        return state.get_state()

    @app.route("/frame")
    def frame():
        png = state.get_frame()
        if not png:
            return "", 204
        return Response(png, mimetype="image/png",
                        headers={"Cache-Control": "no-cache"})

    @app.route("/down_frame")
    def down_frame():
        png = state.get_down_frame()
        if not png:
            return "", 204
        return Response(png, mimetype="image/png",
                        headers={"Cache-Control": "no-cache"})

    @app.route("/camera_api/list")
    def camera_api_list():
        return {"cameras": state.get_camera_list()}

    @app.route("/camera_api/frame/<camera_id>")
    def camera_api_frame(camera_id):
        png = state.get_camera_frame(camera_id)
        if not png:
            return Response("camera frame not ready", 503,
                            headers={"Cache-Control": "no-cache"})
        return Response(png, mimetype="image/png",
                        headers={"Cache-Control": "no-cache"})

    @app.route("/camera_api/metadata/<camera_id>")
    def camera_api_metadata(camera_id):
        metadata = state.get_camera_metadata(camera_id)
        if not metadata:
            return {"status": "not_ready", "camera_id": camera_id}, 503
        return metadata

    @app.route("/camera_api/recording/status")
    def camera_api_recording_status():
        if camera_recorder is None:
            return state.get_camera_recording_status()
        return camera_recorder.status()

    @app.route("/camera_api/recording/start", methods=["POST"])
    def camera_api_recording_start():
        if camera_recorder is None:
            return {"status": "unavailable", "error": "camera recorder not configured"}, 503
        data = request.get_json(silent=True) or {}
        try:
            status = camera_recorder.start(
                mode=data.get("mode"),
                camera_ids=data.get("camera_ids"),
            )
            return {"status": "ok", "recording": status}
        except (TypeError, ValueError, RuntimeError) as exc:
            return {"status": "error", "error": str(exc)}, 400

    @app.route("/camera_api/recording/stop", methods=["POST"])
    def camera_api_recording_stop():
        if camera_recorder is None:
            return {"status": "unavailable", "error": "camera recorder not configured"}, 503
        return {"status": "ok", "recording": camera_recorder.stop()}

    @app.route("/camera_api/recording/event", methods=["POST"])
    def camera_api_recording_event():
        if camera_recorder is None:
            return {"status": "unavailable", "error": "camera recorder not configured"}, 503
        data = request.get_json(silent=True) or {}
        return {"status": "ok", "recording": camera_recorder.trigger_event(data.get("name", "web"))}

    @app.route("/events")
    def events():
        def gen():
            last_ver = -1
            while True:
                st = state.get_state()
                if st["version"] != last_ver:
                    last_ver = st["version"]
                    yield f"data: {json.dumps(st)}\n\n"
                time.sleep(0.15)
        return Response(gen(), mimetype="text/event-stream",
                        headers={"Cache-Control": "no-cache",
                                 "Connection": "keep-alive"})

    return app
