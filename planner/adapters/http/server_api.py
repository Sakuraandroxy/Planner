"""Four frontend routes. Flight and model calls stay outside HTTP request handlers."""
from contextlib import asynccontextmanager
from dataclasses import asdict

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, ConfigDict, field_validator
from starlette.concurrency import run_in_threadpool

from planner.domain.server import BackendUnavailable, ServerState


class TaskRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    instruction: str

    @field_validator("instruction")
    @classmethod
    def nonempty(cls, value):
        if not value.strip():
            raise ValueError("instruction must be non-empty text")
        return value.strip()


def create_api(controller, settings) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app):
        controller.start()
        try:
            yield
        finally:
            await run_in_threadpool(controller.close)

    app = FastAPI(title="Planner API", lifespan=lifespan)
    app.add_middleware(CORSMiddleware, allow_origins=list(settings.allowed_origins),
                       allow_methods=["GET", "POST"], allow_headers=["Content-Type"], allow_credentials=False)

    @app.post("/api/tasks", status_code=202)
    def submit(request: TaskRequest):
        try:
            task = controller.submit(request.instruction)
        except BackendUnavailable as exc:
            code = 503 if exc.state is ServerState.CONNECTING else 409
            raise HTTPException(code, detail={"state": exc.state.value, "message": str(exc)}) from exc
        return {"task_id": task.task_id, "state": ServerState.RUNNING.value}

    @app.get("/api/status")
    def status():
        return asdict(controller.status)

    @app.get("/api/logs")
    def logs(cursor: int = Query(default=0, ge=0)):
        return asdict(controller.logs.read(cursor))

    @app.post("/api/stop")
    def stop():
        from fastapi.responses import JSONResponse
        state = controller.stop()
        return JSONResponse({"state": state.value}, status_code=202 if state is ServerState.STOPPING else 200)

    return app
