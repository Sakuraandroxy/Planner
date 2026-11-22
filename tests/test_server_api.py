import pytest
from fastapi.testclient import TestClient

from planner.adapters.http.server_api import create_api
from planner.config import (
    MapCatalog,
    MapEntry,
    MapScene,
    ProfileInfo,
    ServerSettings,
    VehicleModelCatalog,
    VehicleModelGroup,
    VehicleModelOption,
)
from planner.domain import WorldPose
from planner.domain import BackendUnavailable, ServerState, ServerStatus, TaskSubmission
from planner.domain import QueuedWaypoint
from planner.domain import MODE_CONTINUOUS
from planner.domain import WEATHER_CATALOG, WEATHER_FIELDS, merge_weather
from planner.runtime import FrameHub
from planner.runtime import LogBuffer
from planner.runtime import TrajectoryHub
from planner.simulator import registry


class FakeController:
    def __init__(self, state: ServerState = ServerState.IDLE):
        self.state_value = state
        self.logs = LogBuffer(capacity=10)
        self.submitted = []
        self.started = False
        self.closed = False
        self.home_calls = []
        self.vehicles = ("rover", "drone")
        self.vehicle = self.vehicles[0]
        self.maps = MapCatalog(
            default_map="cityenviron",
            maps=(
                MapEntry(
                    id="cityenviron",
                    label="城市",
                    launch_map="cityenviron",
                    simulator="projectairsim",
                    scene=MapScene(
                        file="scene_city_dual.jsonc", config_path="config/scenes/city"
                    ),
                ),
                MapEntry(
                    id="neighborhood",
                    label="郊区",
                    launch_map="neighborhood",
                    simulator="projectairsim",
                    scene=MapScene(
                        file="scene_neighborhood_dual.jsonc",
                        config_path="config/scenes/neighborhood",
                    ),
                ),
            ),
        )
        self.weather_catalog = WEATHER_CATALOG
        self.vehicle_models = VehicleModelCatalog(
            groups=(
                VehicleModelGroup(
                    "rover",
                    "offroad",
                    (VehicleModelOption("offroad", "越野车"),),
                ),
                VehicleModelGroup(
                    "drone",
                    "quadrotor",
                    (
                        VehicleModelOption("quadrotor", "四旋翼"),
                        VehicleModelOption("hexarotor", "六旋翼"),
                    ),
                ),
            )
        )
        self.models = {"rover": "offroad", "drone": "quadrotor"}
        self.models_switching = False
        self.model_calls = []
        self.profiles = (
            ProfileInfo("default", "世界模型规划器", "实机调校参数", "deepseek-flash"),
            ProfileInfo("vlm", "普通 VLM 基线", "通用导航参数", "deepseek-flash"),
        )
        self.profile = "default"
        self.profile_switching = False
        self.profile_calls = []
        self.map = "cityenviron"
        self.view_mode = "rover_chase"
        self.view_pending = False
        self.weather = None
        self.map_switching = False
        self.sim_error = None
        self.simulator_owned = True
        self.world_calls = []
        self.world_stop_calls = 0
        self.manages_simulator = True
        self.weather_calls = []
        self.view_calls = []
        self.activate_calls = []

    def capabilities(self):
        """Mirror ``ServerController.capabilities``: flags for the active map."""
        entry = self.maps.entry(self.map)
        return None if entry is None else registry.capabilities_for(entry.simulator)

    @property
    def active_simulator(self):
        entry = self.maps.entry(self.map)
        return entry.simulator if entry is not None else ""

    @property
    def simulator_labels(self):
        return registry.labels()

    def start(self):
        self.started = True

    def close(self):
        self.closed = True

    def submit(self, instruction, vehicle=None):
        if self.state_value is not ServerState.IDLE:
            raise BackendUnavailable(self.state_value)
        requested = self.vehicle if vehicle is None else vehicle
        if requested not in self.vehicles:
            raise ValueError(f"unknown vehicle '{requested}'")
        task = TaskSubmission(
            f"task_{len(self.submitted) + 1}",
            instruction,
            requested,
        )
        self.submitted.append(task)
        self.vehicle = requested
        return task

    def stop(self):
        return self.state_value

    def home(self, vehicle=None):
        if self.state_value is not ServerState.IDLE:
            raise BackendUnavailable(self.state_value)
        requested = self.vehicle if vehicle is None else vehicle.strip()
        if requested not in self.vehicles:
            raise ValueError(f"unknown vehicle '{requested}'")
        self.home_calls.append(requested)
        self.vehicle = requested
        return requested

    def request_world(self, map_id):
        entry = self.maps.entry(str(map_id))
        if entry is None:
            raise ValueError(
                f"unknown map '{map_id}'; expected one of "
                f"{list(self.maps.ids)}"
            )
        if self.state_value not in (
            ServerState.IDLE,
            ServerState.DISCONNECTED,
        ):
            raise BackendUnavailable(self.state_value)
        self.world_calls.append(entry.id)
        if entry.id == self.map and self.state_value is ServerState.IDLE:
            return ServerState.IDLE
        self.map = entry.id
        self.state_value = ServerState.CONNECTING
        self.map_switching = True
        return ServerState.CONNECTING

    def stop_world(self):
        if not self.manages_simulator:
            raise ValueError("this deployment does not manage the simulator")
        if self.state_value is not ServerState.IDLE:
            raise BackendUnavailable(self.state_value)
        self.world_stop_calls += 1
        self.state_value = ServerState.CONNECTING
        return ServerState.CONNECTING

    def request_weather(self, values):
        if self.state_value is not ServerState.IDLE:
            raise BackendUnavailable(self.state_value)
        merged = merge_weather(self.weather, values)
        self.weather = merged
        self.weather_calls.append(dict(values))
        return merged

    def activate(self, vehicle):
        if self.state_value is not ServerState.IDLE:
            raise BackendUnavailable(self.state_value)
        requested = vehicle.strip()
        if requested not in self.vehicles:
            raise ValueError(f"unknown vehicle '{requested}'")
        self.activate_calls.append(requested)
        self.vehicle = requested
        self.view_mode = f"{requested}_chase"
        return requested

    def request_models(self, updates):
        if self.vehicle_models is None:
            raise ValueError("this deployment does not configure vehicle models")
        if self.state_value is not ServerState.IDLE:
            raise BackendUnavailable(self.state_value)
        merged = dict(self.models)
        for vehicle, model_id in updates.items():
            group = self.vehicle_models.group(vehicle)
            if group is None:
                raise ValueError(f"unknown vehicle '{vehicle}'")
            if group.option(model_id) is None:
                raise ValueError(f"unknown {vehicle} model '{model_id}'")
            merged[vehicle] = model_id
        if merged == self.models:
            return ServerState.IDLE
        self.models = merged
        self.model_calls.append(dict(updates))
        self.state_value = ServerState.CONNECTING
        self.models_switching = True
        return ServerState.CONNECTING

    def request_profile(self, profile):
        if not self.profiles:
            raise ValueError("this deployment does not list any navigation profiles")
        chosen = str(profile).strip()
        if chosen not in [info.id for info in self.profiles]:
            raise ValueError(
                f"unknown profile '{chosen}'; expected one of "
                f"{[info.id for info in self.profiles]}"
            )
        if chosen == self.profile:
            return self.state_value
        if self.state_value is ServerState.DISCONNECTED:
            # No session to rebuild: the choice waits for the next scene start.
            self.profile = chosen
            self.profile_calls.append(chosen)
            return ServerState.DISCONNECTED
        if self.state_value is not ServerState.IDLE:
            raise BackendUnavailable(self.state_value)
        self.profile = chosen
        self.profile_calls.append(chosen)
        self.state_value = ServerState.CONNECTING
        self.profile_switching = True
        return ServerState.CONNECTING

    def request_view(self, mode):
        if self.state_value in (
            ServerState.CONNECTING,
            ServerState.DISCONNECTED,
        ):
            raise BackendUnavailable(self.state_value)
        if self.state_value in (ServerState.RUNNING, ServerState.STOPPING):
            self.view_mode = mode
            return mode, True
        self.view_mode = mode
        self.view_calls.append(mode)
        return mode, False

    @property
    def status(self):
        return ServerStatus(
            self.state_value,
            self.submitted[-1].instruction if self.submitted else "",
            vehicle=self.vehicle,
            vehicles=self.vehicles,
            map=self.map,
            maps=self.maps.ids,
            view_mode=self.view_mode,
            view_pending=self.view_pending,
            weather=dict(self.weather) if self.weather is not None else None,
            map_switching=self.map_switching,
            sim_error=self.sim_error,
            models=dict(self.models) if self.vehicle_models is not None else None,
            models_switching=self.models_switching,
            profile=self.profile,
            profiles=tuple(info.id for info in self.profiles),
            profile_switching=self.profile_switching,
        )


def _client(controller, trajectory_hub=None):
    app = create_api(
        controller,
        ServerSettings(log_buffer_size=10),
        trajectory_hub=trajectory_hub,
    )
    return TestClient(app)


def test_submit_returns_202_and_task_id():
    controller = FakeController()
    with _client(controller) as client:
        assert controller.started
        response = client.post(
            "/api/tasks",
            json={"instruction": "  fly to the red car  "},
        )
        assert response.status_code == 202
        assert response.json() == {
            "task_id": "task_1",
            "state": "running",
            "vehicle": "rover",
        }
    assert controller.submitted[0].instruction == "fly to the red car"
    assert controller.closed


def test_submit_routes_to_the_requested_vehicle():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post(
            "/api/tasks",
            json={"instruction": "fly to the target", "vehicle": " drone "},
        )
        assert response.status_code == 202
        assert response.json()["vehicle"] == "drone"
        assert client.get("/api/status").json()["vehicle"] == "drone"
    assert controller.submitted[0].vehicle == "drone"


def test_submit_rejects_an_unknown_vehicle():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post(
            "/api/tasks", json={"instruction": "fly", "vehicle": "boat"}
        )
    assert response.status_code == 400
    assert "unknown vehicle" in response.json()["detail"]["message"]
    assert controller.submitted == []


def test_submit_validates_the_body():
    controller = FakeController()
    with _client(controller) as client:
        assert client.post("/api/tasks", json={"instruction": "  "}).status_code == 422
        assert (
            client.post(
                "/api/tasks", json={"instruction": "fly", "extra": 1}
            ).status_code
            == 422
        )
        assert client.post("/api/tasks", json={}).status_code == 422


@pytest.mark.parametrize(
    ("state", "code"),
    [(ServerState.CONNECTING, 503), (ServerState.RUNNING, 409)],
)
def test_submit_unavailable_states(state, code):
    controller = FakeController(state)
    with _client(controller) as client:
        response = client.post("/api/tasks", json={"instruction": "fly"})
    assert response.status_code == code
    assert response.json()["detail"]["state"] == state.value


def test_status_and_logs():
    controller = FakeController()
    controller.logs.append("planner ready")
    with _client(controller) as client:
        status = client.get("/api/status")
        assert status.status_code == 200
        payload = status.json()
        # Port-derived feature flags ride along with the status payload. They
        # are asserted separately so the lifecycle fields below stay readable.
        simulator = payload.pop("simulator")
        caps = payload.pop("capabilities")
        assert payload == {
            "state": "idle",
            "instruction": "",
            "last_result": None,
            "vehicle": "rover",
            "vehicles": ["rover", "drone"],
            "map": "cityenviron",
            "maps": ["cityenviron", "neighborhood"],
            "view_mode": "rover_chase",
            "view_pending": False,
            "weather": None,
            "map_switching": False,
            "sim_error": None,
            "models": {"rover": "offroad", "drone": "quadrotor"},
            "models_switching": False,
            "profile": "default",
            "profiles": ["default", "vlm"],
            "profile_switching": False,
        }
        assert simulator == "projectairsim"
        assert caps["vehicles"] == ["rover", "drone"]
        assert caps["actions"] == ["home", "models", "video", "weather"]
        assert caps["task_kinds"] == ["basic_motion", "visual_goal_navigation", "free_navigation"]
        assert status.headers["cache-control"] == "no-store"

        first = client.get("/api/logs").json()
        assert first["next_cursor"] == 1
        assert first["items"][0]["message"] == "planner ready"
        assert first["truncated"] is False

        second = client.get("/api/logs", params={"cursor": 1}).json()
        assert second["items"] == []
        assert second["next_cursor"] == 1

        assert (
            client.get("/api/logs", params={"cursor": -1}).status_code == 422
        )


def test_status_reports_a_pending_view_switch():
    controller = FakeController()
    controller.view_pending = True
    with _client(controller) as client:
        status = client.get("/api/status").json()
    assert status["view_pending"] is True


def test_stop_codes():
    idle = FakeController(ServerState.IDLE)
    with _client(idle) as client:
        response = client.post("/api/stop")
        assert response.status_code == 200
        assert response.json() == {"state": "idle"}

    stopping = FakeController(ServerState.STOPPING)
    with _client(stopping) as client:
        response = client.post("/api/stop")
        assert response.status_code == 202
        assert response.json() == {"state": "stopping"}


def test_home_is_accepted_while_idle():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/home")
    assert response.status_code == 202
    assert response.json() == {"state": "idle", "vehicle": "rover"}
    assert controller.home_calls == ["rover"]


def test_home_accepts_the_drone():
    controller = FakeController()
    controller.vehicle = "drone"
    with _client(controller) as client:
        response = client.post("/api/home")
    assert response.status_code == 202
    assert response.json() == {"state": "idle", "vehicle": "drone"}
    assert controller.home_calls == ["drone"]


def test_home_routes_to_the_requested_vehicle():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/home", json={"vehicle": " drone "})
    assert response.status_code == 202
    assert response.json() == {"state": "idle", "vehicle": "drone"}
    assert controller.home_calls == ["drone"]


def test_home_rejects_an_unknown_vehicle():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/home", json={"vehicle": "boat"})
    assert response.status_code == 400
    assert "unknown vehicle" in response.json()["detail"]["message"]
    assert controller.home_calls == []


@pytest.mark.parametrize(
    ("state", "code"),
    [
        (ServerState.CONNECTING, 503),
        (ServerState.RUNNING, 409),
        (ServerState.STOPPING, 409),
    ],
)
def test_home_is_rejected_outside_idle(state, code):
    controller = FakeController(state)
    with _client(controller) as client:
        response = client.post("/api/home")
    assert response.status_code == code
    assert response.json()["detail"]["state"] == state.value
    assert controller.home_calls == []


def test_vehicle_activation_returns_the_chase_view():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/vehicle", json={"vehicle": " drone "})
        assert response.status_code == 202
        assert response.json() == {
            "state": "idle",
            "vehicle": "drone",
            "view_mode": "drone_chase",
        }
        assert client.get("/api/status").json()["vehicle"] == "drone"
    assert controller.activate_calls == ["drone"]


def test_vehicle_activation_rejects_an_unknown_vehicle():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/vehicle", json={"vehicle": "boat"})
    assert response.status_code == 400
    assert "unknown vehicle" in response.json()["detail"]["message"]
    assert controller.activate_calls == []


@pytest.mark.parametrize(
    "payload", [{}, {"vehicle": "  "}, {"vehicle": "drone", "extra": 1}]
)
def test_vehicle_activation_validates_the_body(payload):
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/vehicle", json=payload)
    assert response.status_code == 422
    assert controller.activate_calls == []


@pytest.mark.parametrize(
    ("state", "code"),
    [
        (ServerState.CONNECTING, 503),
        (ServerState.RUNNING, 409),
        (ServerState.STOPPING, 409),
    ],
)
def test_vehicle_activation_requires_idle(state, code):
    controller = FakeController(state)
    with _client(controller) as client:
        response = client.post("/api/vehicle", json={"vehicle": "drone"})
    assert response.status_code == code
    assert response.json()["detail"]["state"] == state.value
    assert controller.activate_calls == []


def test_world_options_returns_the_catalog():
    controller = FakeController()
    with _client(controller) as client:
        response = client.get("/api/world/options")
    assert response.status_code == 200
    body = response.json()
    assert body["maps"] == ["cityenviron", "neighborhood"]
    assert body["map_labels"]["neighborhood"] == "郊区"
    assert body["default_map"] == "cityenviron"
    assert body["selection"] == {"map": "cityenviron"}
    assert body["simulator_owned"] is True


def test_world_options_unavailable_without_a_catalog():
    controller = FakeController()
    controller.maps = None
    with _client(controller) as client:
        response = client.get("/api/world/options")
    assert response.status_code == 503


def test_world_switch_returns_202_and_changes_the_selection():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/world", json={"map": "neighborhood"})
        assert response.status_code == 202
        assert response.json() == {
            "state": "connecting",
            "map": "neighborhood",
        }
        status = client.get("/api/status").json()
    assert status["map"] == "neighborhood"
    assert status["map_switching"] is True
    assert controller.world_calls == ["neighborhood"]


def test_world_switch_to_the_same_map_returns_200():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/world", json={"map": "cityenviron"})
    assert response.status_code == 200
    assert response.json() == {"state": "idle", "map": "cityenviron"}


def test_world_switch_rejects_an_unknown_map():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/world", json={"map": "moon"})
    assert response.status_code == 400
    assert "unknown map" in response.json()["detail"]["message"]
    assert controller.world_calls == []


@pytest.mark.parametrize(
    ("state", "code"),
    [(ServerState.CONNECTING, 503), (ServerState.RUNNING, 409)],
)
def test_world_switch_requires_idle(state, code):
    controller = FakeController(state)
    with _client(controller) as client:
        response = client.post("/api/world", json={"map": "neighborhood"})
    assert response.status_code == code
    assert response.json()["detail"]["state"] == state.value


def test_world_switch_starts_a_scene_while_disconnected():
    controller = FakeController(ServerState.DISCONNECTED)
    with _client(controller) as client:
        response = client.post("/api/world", json={"map": "neighborhood"})
        assert response.status_code == 202
        assert response.json() == {
            "state": "connecting",
            "map": "neighborhood",
        }
    assert controller.world_calls == ["neighborhood"]


def test_stop_world_returns_202_and_leaves_the_console_disconnected():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/world/stop")
    assert response.status_code == 202
    assert response.json() == {"state": "connecting"}
    assert controller.world_stop_calls == 1


@pytest.mark.parametrize(
    ("state", "code"),
    [(ServerState.CONNECTING, 503), (ServerState.RUNNING, 409)],
)
def test_stop_world_requires_idle(state, code):
    controller = FakeController(state)
    with _client(controller) as client:
        response = client.post("/api/world/stop")
    assert response.status_code == code
    assert controller.world_stop_calls == 0


def test_stop_world_without_a_managed_simulator_returns_400():
    controller = FakeController()
    controller.manages_simulator = False
    with _client(controller) as client:
        response = client.post("/api/world/stop")
    assert response.status_code == 400
    assert "does not manage" in response.json()["detail"]["message"]


def test_disconnected_commands_return_503():
    controller = FakeController(ServerState.DISCONNECTED)
    with _client(controller) as client:
        task = client.post("/api/tasks", json={"instruction": "fly"})
        assert task.status_code == 503
        assert task.json()["detail"]["state"] == "disconnected"
        assert (
            client.post("/api/home", json={"vehicle": "rover"}).status_code
            == 503
        )
        assert (
            client.post(
                "/api/vehicle", json={"vehicle": "drone"}
            ).status_code
            == 503
        )
        assert (
            client.post("/api/weather", json={"rain": 0.5}).status_code
            == 503
        )
        assert (
            client.post(
                "/api/view", json={"mode": "rover_front"}
            ).status_code
            == 503
        )
        assert client.post("/api/stop").status_code == 200


def test_world_request_validates_the_body():
    controller = FakeController()
    with _client(controller) as client:
        assert client.post("/api/world", json={"map": "  "}).status_code == 422
        assert (
            client.post(
                "/api/world", json={"map": "cityenviron", "extra": 1}
            ).status_code
            == 422
        )


def test_profile_options_list_what_a_switch_would_change():
    controller = FakeController()
    with _client(controller) as client:
        body = client.get("/api/profile").json()
    assert body["current"] == "default"
    assert [profile["id"] for profile in body["profiles"]] == ["default", "vlm"]
    assert body["profiles"][0]["label"] == "世界模型规划器"
    assert body["profiles"][1]["note"] == "通用导航参数"
    assert body["profiles"][0]["model"] == "deepseek-flash"
    assert body["switching"] is False


def test_a_profile_can_be_chosen_before_a_scene_starts():
    controller = FakeController(ServerState.DISCONNECTED)
    with _client(controller) as client:
        response = client.post("/api/profile", json={"profile": "vlm"})
        assert response.status_code == 200
        assert response.json() == {"state": "disconnected", "profile": "vlm"}
        assert client.get("/api/profile").json()["current"] == "vlm"
    assert controller.profile_calls == ["vlm"]
    assert controller.profile_switching is False


def test_profile_switch_returns_202_and_shows_the_new_profile():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/profile", json={"profile": "vlm"})
        assert response.status_code == 202
        assert response.json() == {"state": "connecting", "profile": "vlm"}
        status = client.get("/api/status").json()
        options = client.get("/api/profile").json()
    assert status["profile"] == "vlm"
    assert status["profiles"] == ["default", "vlm"]
    assert status["profile_switching"] is True
    assert options["switching"] is True
    assert controller.profile_calls == ["vlm"]


def test_selecting_the_running_profile_is_a_no_op():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/profile", json={"profile": "default"})
        assert response.status_code == 200
        assert response.json() == {"state": "idle", "profile": "default"}
    assert controller.profile_calls == []


def test_profile_switch_rejects_an_unknown_profile():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/profile", json={"profile": "nope"})
    assert response.status_code == 400
    assert "expected one of" in response.json()["detail"]["message"]


def test_profile_switch_is_refused_while_a_task_is_running():
    controller = FakeController(ServerState.RUNNING)
    with _client(controller) as client:
        response = client.post("/api/profile", json={"profile": "vlm"})
    assert response.status_code == 409
    assert controller.profile_calls == []


def test_model_options_return_the_catalog_and_selection():
    controller = FakeController()
    with _client(controller) as client:
        response = client.get("/api/models")
    assert response.status_code == 200
    body = response.json()
    assert body["models"] == {"rover": "offroad", "drone": "quadrotor"}
    assert [option["id"] for option in body["options"]["drone"]] == [
        "quadrotor",
        "hexarotor",
    ]
    assert body["options"]["drone"][1]["label"] == "六旋翼"
    assert body["options"]["rover"][0]["id"] == "offroad"
    assert body["switching"] is False


def test_model_options_unavailable_without_a_catalog():
    controller = FakeController()
    controller.vehicle_models = None
    with _client(controller) as client:
        response = client.get("/api/models")
    assert response.status_code == 503


def test_model_switch_returns_202_and_updates_the_selection():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post(
            "/api/models", json={"models": {"drone": "hexarotor"}}
        )
        assert response.status_code == 202
        assert response.json() == {
            "state": "connecting",
            "models": {"rover": "offroad", "drone": "hexarotor"},
        }
        status = client.get("/api/status").json()
    assert status["models"]["drone"] == "hexarotor"
    assert status["models_switching"] is True
    assert controller.model_calls == [{"drone": "hexarotor"}]


def test_model_switch_to_the_same_selection_returns_200():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post(
            "/api/models",
            json={"models": {"rover": "offroad", "drone": "quadrotor"}},
        )
    assert response.status_code == 200
    assert response.json() == {
        "state": "idle",
        "models": {"rover": "offroad", "drone": "quadrotor"},
    }
    assert controller.model_calls == []


@pytest.mark.parametrize(
    "payload",
    [
        {"models": {"drone": "trirotor"}},
        {"models": {"boat": "offroad"}},
    ],
)
def test_model_switch_rejects_unknown_selections(payload):
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/models", json=payload)
    assert response.status_code == 400
    assert controller.model_calls == []


@pytest.mark.parametrize(
    "payload",
    [{}, {"models": {}}, {"models": {"drone": "  "}}, {"models": {"drone": "quadrotor"}, "extra": 1}],
)
def test_model_switch_validates_the_body(payload):
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/models", json=payload)
    assert response.status_code == 422
    assert controller.model_calls == []


@pytest.mark.parametrize(
    ("state", "code"),
    [(ServerState.CONNECTING, 503), (ServerState.RUNNING, 409)],
)
def test_model_switch_requires_idle(state, code):
    controller = FakeController(state)
    with _client(controller) as client:
        response = client.post(
            "/api/models", json={"models": {"drone": "hexarotor"}}
        )
    assert response.status_code == code
    assert response.json()["detail"]["state"] == state.value


def test_weather_options_return_fields_and_presets():
    controller = FakeController()
    with _client(controller) as client:
        response = client.get("/api/weather")
    assert response.status_code == 200
    body = response.json()
    assert [field["id"] for field in body["fields"]] == list(WEATHER_FIELDS)
    preset_ids = [preset["id"] for preset in body["presets"]]
    assert "clear" in preset_ids
    assert "heavy_rain" in preset_ids
    heavy = next(
        preset for preset in body["presets"] if preset["id"] == "heavy_rain"
    )
    assert set(heavy["values"]) == set(WEATHER_FIELDS)


def test_weather_options_unavailable_without_a_catalog():
    controller = FakeController()
    controller.weather_catalog = None
    with _client(controller) as client:
        response = client.get("/api/weather")
    assert response.status_code == 503


def test_weather_update_returns_the_merged_state():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/weather", json={"rain": 0.5})
        assert response.status_code == 202
        body = response.json()
        assert body["state"] == "idle"
        assert body["weather"]["rain"] == 0.5
        assert body["weather"]["fog"] == 0.0
        assert client.get("/api/status").json()["weather"]["rain"] == 0.5
    assert controller.weather_calls == [{"rain": 0.5}]


@pytest.mark.parametrize(
    "payload",
    [
        {"rain": 1.5},
        {"rain": -0.1},
        {"rain": "0.5"},
        {"unknown": 0.5},
        {},
    ],
)
def test_weather_update_rejects_invalid_values(payload):
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/weather", json=payload)
    assert response.status_code == 400
    assert controller.weather_calls == []


@pytest.mark.parametrize(
    ("state", "code"),
    [(ServerState.CONNECTING, 503), (ServerState.RUNNING, 409)],
)
def test_weather_update_requires_idle(state, code):
    controller = FakeController(state)
    with _client(controller) as client:
        response = client.post("/api/weather", json={"rain": 0.5})
    assert response.status_code == code


def test_view_switch_returns_the_applied_mode():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/view", json={"mode": "drone_chase"})
    assert response.status_code == 200
    assert response.json() == {"view_mode": "drone_chase", "deferred": False}
    assert controller.view_calls == ["drone_chase"]


def test_view_switch_during_a_task_is_deferred():
    controller = FakeController(ServerState.RUNNING)
    with _client(controller) as client:
        response = client.post("/api/view", json={"mode": "drone_front"})
    assert response.status_code == 200
    assert response.json() == {"view_mode": "drone_front", "deferred": True}


def test_view_switch_rejects_an_unknown_mode():
    controller = FakeController()
    with _client(controller) as client:
        response = client.post("/api/view", json={"mode": "moon"})
    assert response.status_code == 422
    assert controller.view_calls == []


def test_view_switch_unavailable_while_connecting():
    controller = FakeController(ServerState.CONNECTING)
    with _client(controller) as client:
        response = client.post("/api/view", json={"mode": "drone_front"})
    assert response.status_code == 503


def test_trajectory_is_empty_without_a_hub():
    with _client(FakeController()) as client:
        body = client.get("/api/trajectory").json()
    assert body["active"] is False
    assert body["mode"] == "idle"
    assert body["points"] == []
    assert body["queue"] == {"count": 0, "capacity": 0}


def test_trajectory_returns_the_published_queue():
    hub = TrajectoryHub()
    hub.publish_queue(
        instruction="follow the road",
        stage_id="stage_1",
        mode=MODE_CONTINUOUS,
        capacity=10,
        pose=WorldPose(0.0, 0.0, 0.0, 0.0),
        items=(
            QueuedWaypoint(
                7,
                WorldPose(0.0, 0.0, 0.0, 0.0),
                WorldPose(1.0, 0.0, 0.0, 0.0),
                0.0,
            ),
        ),
        planning_calls=2,
        path_length_m=3.5,
    )
    with _client(FakeController(), hub) as client:
        body = client.get("/api/trajectory").json()
    assert body["active"] is True
    assert body["mode"] == "continuous_world_queue"
    assert body["queue"] == {"count": 1, "capacity": 10}
    assert body["points"][0]["waypoint_id"] == 7
    assert body["points"][0]["x"] == 1.0
    assert body["points"][0]["distance_m"] == pytest.approx(1.0)
    assert body["path_length_m"] == 3.5
    assert body["revision"] == 1


def test_cors_preflight():
    controller = FakeController()
    with _client(controller) as client:
        response = client.options(
            "/api/tasks",
            headers={
                "Origin": "http://localhost:5173",
                "Access-Control-Request-Method": "POST",
                "Access-Control-Request-Headers": "content-type",
            },
        )
    assert response.status_code == 200
    assert response.headers["access-control-allow-origin"] == "*"
    assert "POST" in response.headers["access-control-allow-methods"]


def test_console_files_are_served():
    controller = FakeController()
    with _client(controller) as client:
        index = client.get("/")
        assert index.status_code == 200
        assert "视觉语言导航在线控制台" in index.text
        assert 'id="xpra-frame"' in index.text
        assert 'id="vehicle-modes"' in index.text
        assert 'data-vehicle="rover"' in index.text
        assert 'data-vehicle="drone"' in index.text
        assert 'id="current-vehicle"' in index.text
        assert 'id="home"' in index.text
        assert 'id="world-map"' in index.text
        assert 'id="world-apply"' in index.text
        assert 'id="world-reload"' in index.text
        assert 'id="model-rover"' in index.text
        assert 'id="model-drone"' in index.text
        assert 'id="models-apply"' in index.text
        assert 'id="models-feedback"' in index.text
        assert 'id="view-modes"' in index.text
        assert 'data-perspective="first"' in index.text
        assert 'data-perspective="third"' in index.text
        assert 'id="weather-apply"' in index.text
        assert 'id="weather-preset"' in index.text
        assert 'id="weather-fields"' in index.text
        assert 'id="world-color"' not in index.text
        assert 'id="trajectory-list"' in index.text
        assert 'id="trajectory-empty"' in index.text
        assert 'id="trajectory-toggle"' in index.text
        assert 'id="view-shield"' in index.text
        assert index.headers["cache-control"] == "no-cache"

        # The console is split into classic scripts sharing one global scope.
        # Checking their concatenation keeps this test about behaviour, not about
        # which file a function happens to live in.
        tokens = (
            "pollStatus", "pollTrajectory", "renderTrajectory", "/api/trajectory",
            "/api/tasks", "/api/home", "/api/vehicle", "activateVehicle",
            "drone_front", "xpra-frame", "10010", "view-toggle", "vehicle-modes",
            "selectedVehicle", "VEHICLE_LABELS", "world-apply", "/api/world/options",
            "/api/world", "/api/models", "loadModelOptions", "applyModels",
            "model-drone", "/api/view", "/api/weather", "applyWeather", "applyWorld",
            "disconnected", "启动场景", "requestView", "renderViewButtons",
            "view_pending", "renderViewShield", "view-shield", "resetXpra",
            "world-map", "weather-preset", "trajectory-toggle", "/api/video",
        )
        source = ""
        for name in ("core.js", "world.js", "settings.js", "flight.js", "app.js"):
            part = client.get(f"/console/src/{name}")
            assert part.status_code == 200, name
            assert part.headers["cache-control"] == "no-cache"
            source += part.text
        for token in tokens:
            assert token in source, token
        assert client.get("/console/src/scene3d.js").status_code == 200

        styles = client.get("/console/styles.css")
        assert styles.status_code == 200
        assert ".shield" in styles.text
        assert styles.headers["cache-control"] == "no-cache"


def test_video_route_exists_only_with_frames():
    controller = FakeController()
    settings = ServerSettings(log_buffer_size=10)
    without = create_api(controller, settings)
    assert "/api/video" not in {
        getattr(route, "path", "") for route in without.routes
    }

    hub = FrameHub(max_fps=10)
    with_video = create_api(controller, settings, frames=hub)
    assert "/api/video" in {
        getattr(route, "path", "") for route in with_video.routes
    }