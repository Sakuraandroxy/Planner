"""Which family's driver gets which call.

The router is the only object that knows both families exist. Two bugs came from
handing a whole switch to the target's driver: the UE4 driver does not listen on UE5's
ports, so a live UE5 stack looked like an idle machine, the stop was skipped, and the
launch died on "display :104 is still held by" the other family's X server. Measured
switching UE5 -> UE4 and back through the HTTP service.
"""

from types import SimpleNamespace

from planner.simulator.marker import write_map_marker
from planner.simulator.router import FamilyRouter


class _RecordingDriver:
    def __init__(self, family_id):
        self.family_id = family_id
        self.calls: list[str] = []

    def running(self):
        return False

    def preflight(self):
        self.calls.append("preflight")

    def stop(self):
        self.calls.append("stop")

    def start(self, entry):
        self.calls.append(f"start:{entry.id}")

    def restart(self, entry):
        self.calls.append(f"restart:{entry.id}")


MAPS = SimpleNamespace()


def _router(tmp_path, marker_map=None):
    from planner.config import MapCatalog, MapEntry

    maps = MapCatalog(
        default_map="blocks",
        maps=(
            MapEntry(id="blocks", label="Blocks", simulator="airsim",
                     binary="/data/airsim/Blocks/Binaries/Linux/Blocks",
                     map_name="Blocks"),
            MapEntry(id="cityenviron", label="City", simulator="projectairsim",
                     launch_map="cityenviron", scene_file="scene_city_dual.jsonc",
                     sim_config_path="config/scenes/city"),
        ),
    )
    if marker_map:
        marker = tmp_path / "marker"
        write_map_marker(marker, marker_map)
    else:
        marker = tmp_path / "no-marker-yet"
    settings = SimpleNamespace(map_marker_file=str(marker), default_family="projectairsim")
    families = SimpleNamespace(
        ids=("projectairsim", "airsim"),
        default_family="projectairsim",
        entry=lambda family_id: SimpleNamespace(id=family_id),
    )
    drivers = {}

    def factory(family):
        drivers[family.id] = _RecordingDriver(family.id)
        return drivers[family.id]

    router = FamilyRouter(families, settings, tmp_path, maps, driver_factory=factory)
    router._test_drivers = drivers
    return router


def test_switching_families_stops_the_family_that_is_live(tmp_path):
    router = _router(tmp_path, marker_map="cityenviron")
    router.restart(router._maps.entry("blocks"))
    order = [(d.family_id, d.calls) for d in router._test_drivers.values()]
    assert router._test_drivers["projectairsim"].calls == ["stop"], order
    assert router._test_drivers["airsim"].calls == ["restart:blocks"], order


def test_a_same_family_switch_leaves_the_other_driver_alone(tmp_path):
    router = _router(tmp_path, marker_map="blocks")
    router.restart(router._maps.entry("blocks"))
    assert router._test_drivers["airsim"].calls == ["restart:blocks"]
    assert "projectairsim" not in router._test_drivers


def test_an_unreadable_marker_falls_back_to_the_default_family(tmp_path):
    """Boot and switch paths read the same record; a missing one must not crash."""
    router = _router(tmp_path, marker_map=None)
    router.stop()
    assert router._test_drivers["projectairsim"].calls == ["stop"]
