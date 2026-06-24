"""Flask application factory and route definitions."""
import json, time, os
from flask import Flask, Response, send_from_directory

_frontend_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "frontend")


def create_app(state):
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
            state.update(task=data["task"])
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
