"""Dependency assembly and error mapping for the console routes.

``server_api`` keeps the route table; this module builds the FastAPI application
around the controller and holds the error shapes every endpoint raises, so the
status codes are decided in one place.
"""

from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from starlette.concurrency import run_in_threadpool

from planner.assembly import BackendUnavailable, ServerState

CONSOLE_DIR = Path(__file__).resolve().parents[3] / "console"


def unavailable(exc: BackendUnavailable) -> HTTPException:
    """The controller refused a command because of the state it is in."""
    code = (
        503
        if exc.state in (ServerState.CONNECTING, ServerState.DISCONNECTED)
        else 409
    )
    return HTTPException(
        code, detail={"state": exc.state.value, "message": str(exc)}
    )


def invalid(exc: ValueError, code: int = 400) -> HTTPException:
    """The request reached the controller and was rejected as unusable."""
    return HTTPException(code, detail={"message": str(exc)})


def invoke(action, *args, code: int = 400):
    """Run a controller command, mapping its refusal onto an HTTP error."""
    try:
        return action(*args)
    except BackendUnavailable as exc:
        raise unavailable(exc) from exc
    except ValueError as exc:
        raise invalid(exc, code) from exc


def missing_catalog(name: str) -> HTTPException:
    """The controller carries no catalog of the named kind."""
    return HTTPException(
        503, detail={"message": f"{name} catalog is unavailable"}
    )


def missing_hub(name: str) -> HTTPException:
    """The service was built without the hub this route reads.

    A route must never answer with an empty payload for a hub that was never
    attached: the console would show an empty panel and no error.
    """
    return HTTPException(
        503,
        detail={
            "reason": f"{name}_hub_missing",
            "message": f"the {name} hub is not attached to this service",
        },
    )


def unsupported(action: str, simulator: str, reason: str, message: str) -> HTTPException:
    """501 for an action this deployment cannot serve, with a stated reason.

    Two causes share this shape: the active simulator has no port for the
    action (``capability_missing``), or the service was built without the
    component that carries it (``*_hub_missing``). Clients branch on the
    status code alone.
    """
    return HTTPException(
        501,
        detail={
            "unsupported": action,
            "simulator": simulator,
            "reason": reason,
            "message": message,
        },
    )


def capabilities(controller):
    """Flags for the active map's simulator, as the controller reports them."""
    return controller.capabilities(), controller.active_simulator


def capability_gate(controller):
    """``require(action)``: refuse an action the active simulator cannot do."""

    def require(action):
        caps, simulator = capabilities(controller)
        if caps is not None and not caps.supports(action):
            raise unsupported(
                action,
                simulator,
                "capability_missing",
                (
                    f"simulator '{simulator}' does not implement the port "
                    f"behind {action}"
                ),
            )

    return require


def build_app(controller, settings, frames=None) -> FastAPI:
    """The application object: lifespan, CORS and cache headers."""

    @asynccontextmanager
    async def lifespan(app):
        controller.start()
        try:
            yield
        finally:
            await run_in_threadpool(controller.close)

    app = FastAPI(title="ProjectAirSim VLN Planner API", lifespan=lifespan)
    app.state.frames = frames
    app.add_middleware(
        CORSMiddleware,
        allow_origins=list(settings.allowed_origins),
        allow_methods=["GET", "POST"],
        allow_headers=["Content-Type"],
        allow_credentials=False,
    )

    @app.middleware("http")
    async def cache_headers(request, call_next):
        response = await call_next(request)
        path = request.url.path
        if path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-store"
        elif path == "/" or path.startswith("/console/"):
            # Revalidate so an updated console never runs from a stale cache.
            response.headers["Cache-Control"] = "no-cache"
        return response

    return app


def mount_console(app) -> None:
    """Serve the console bundle next to the API when the build is present."""
    if not CONSOLE_DIR.is_dir():
        return

    app.mount(
        "/console",
        StaticFiles(directory=str(CONSOLE_DIR)),
        name="console",
    )

    @app.get("/", include_in_schema=False)
    def index():
        return FileResponse(CONSOLE_DIR / "index.html")
