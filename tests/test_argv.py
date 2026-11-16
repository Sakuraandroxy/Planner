"""One argv renderer for every simulator, with no silent literal leakage."""

import pytest

from planner.config import MapEntry
from planner.simulator.argv import argv_values, render, render_argv
from planner.domain.runtime import SimulatorFamily


class Settings:
    display = ":104"
    xpra_port = 10010
    topics_port = 8991
    services_port = 8992


def values(entry=None, simulator=None):
    return argv_values(
        entry=entry or MapEntry(
            id="city", label="城市", simulator="projectairsim",
            launch_map="cityenviron", scene_file="scene_city_dual.jsonc",
            sim_config_path="config/scenes/city",
            binary="/sim/CityEnviron/Binaries/Linux/CityEnviron-Linux-Shipping",
            map_name="CityEnviron",
        ),
        simulator=simulator or SimulatorFamily(
            id="projectairsim", label="ProjectAirSim", driver="xpra_start",
            argv=("xpra", "start"), stop_argv=("xpra", "stop"),
            ports={"xpra": 10010, "topics": 8991, "services": 8992},
        ),
        settings=Settings(),
        runtime_dir="output/xpra",
    )


def test_projectairsim_ports_come_from_the_family_block_not_the_settings():
    """One family's port list cannot make another one resolve a wrong port."""
    got = render_argv(
        ("{map.map_name}", "{display}", "{ports.xpra}", "{ports.topics}", "{ports.services}"),
        values(),
    )
    assert got == ["CityEnviron", ":104", "10010", "8991", "8992"]


def test_airsim_template_carries_settings_and_the_binary_map_name():
    entry = MapEntry("blocks", "Blocks", simulator="airsim",
                     binary="/sim/Blocks/Binaries/Linux/Blocks", map_name="Blocks")
    simulator = SimulatorFamily(
        id="airsim", label="AirSim", driver="xpra_start",
        ports={"xpra": 10010, "rpc": 41451}, settings_file="/sim/settings.json",
        res_x=1280, res_y=720, argv=("xpra", "start"),
        stop_argv=("xpra", "stop"),
    )
    got = render_argv(
        ("{map.binary}", "{map.map_name}", "-settings={settings_file}", "-abslog={abslog}"),
        values(entry, simulator),
    )
    assert got == [
        "/sim/Blocks/Binaries/Linux/Blocks",
        "Blocks",
        "-settings=/sim/settings.json",
        "-abslog=output/xpra/logs/blocks.log",
    ]


def test_unresolved_placeholder_raises_instead_of_leaking_the_token():
    with pytest.raises(ValueError, match="does not provide"):
        render("--start-child={nope}", values())


def test_empty_value_raises_because_xpra_would_get_a_bare_flag():
    with pytest.raises(ValueError, match="resolved to nothing"):
        render("-settings={settings_file}", {"settings_file": "  "})


def test_display_number_strips_the_colon_for_socket_paths():
    assert values()["display_number"] == "104"
