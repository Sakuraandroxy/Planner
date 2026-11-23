"""Capability gating may not hide the panel the operator needs first.

The models markup once sat *inside* the world card, so ``applyCapabilities`` hid the
whole world card on AirSim -- every map selector vanished a second after page load and
there was no way to start a scene. The three setup panels now share one card behind a
tab strip, which makes that mistake easy to repeat: gating has to hide a tab, and the
map picker has to keep its own panel.
"""

import re
from pathlib import Path

CONSOLE = Path(__file__).resolve().parents[1] / "console"
SRC = CONSOLE / "src"
#: classic scripts in load order; app.js binds events last, so it must come last.
SCRIPTS = ("core.js", "world.js", "settings.js", "flight.js", "app.js")


def _sources() -> str:
    """All console scripts joined, so moving a function between files is not an edit here."""
    return "\n".join((SRC / name).read_text(encoding="utf-8") for name in SCRIPTS)


def _html() -> str:
    return (CONSOLE / "index.html").read_text(encoding="utf-8")


def test_gated_ids_exist_in_the_markup():
    """A gate that names a missing element silently does nothing."""
    app = _sources()
    body = re.search(r"function applyCapabilities\(\).*?\n}", app, flags=re.DOTALL).group(0)
    gated = set(re.findall(r'\$\("([a-z0-9-]+)"\)', body))
    assert gated, "applyCapabilities references no ids at all"
    have = set(re.findall(r'id="([^"]+)"', _html()))
    assert gated <= have, f"gating references unknown ids: {sorted(gated - have)}"


def test_capability_gating_hides_a_tab_not_the_settings_card():
    app = _code(_sources())
    body = re.search(r"function applyCapabilities\(\).*?\n}", app, flags=re.DOTALL).group(0)
    assert 'show($("tab-models")' in body, body
    assert "models-card" not in body, "the card is gone; hide the tab instead"
    assert "settings-card" not in body, "the settings card must stay visible"


def test_each_panel_owns_its_own_controls():
    """A gate on one panel must not be able to take another panel's controls."""
    html = _html()
    world = html[html.index('id="panel-world"') : html.index('id="panel-weather"')]
    weather = html[html.index('id="panel-weather"') : html.index('id="panel-models"')]
    models = html[html.index('id="panel-models"') :]

    assert 'id="world-map"' in world
    assert 'id="weather-preset"' in weather
    assert 'id="model-rover"' in models
    for foreign in ('id="model-rover"', 'id="weather-preset"'):
        assert foreign not in world


def _code(text: str) -> str:
    """Drop line comments, so a check on code cannot match prose about the code."""
    return "\n".join(line.split("//")[0] for line in text.splitlines())


def test_gating_uses_explicit_card_ids_not_dom_nesting():
    app = _code(_sources())
    assert 'closest("section")' not in app, (
        "capability gating must name the card it hides; resolving it by DOM nesting "
        "made moving a control between cards hide the wrong panel"
    )


def test_service_level_features_are_not_gated_by_the_simulator():
    """Remaining trajectory is a planner feature, so no family may lose it.

    ``/api/trajectory`` used to sit behind a port flag, and the port it checked
    was the service's own hub handed back by the family -- so UE4 answered 501
    for data its shared workflows were already writing, and the card was hidden
    for it. A capability gate may only hide something the engine cannot do.
    """
    app = _code(_sources())
    assert 'hasAction("trajectory")' not in app, (
        "the remaining-trajectory card must not depend on a simulator capability"
    )
    assert 'id="trajectory-card"' in _html()
    api = Path(__file__).resolve().parents[1] / "planner/adapters/http/server_api.py"
    body = api.read_text(encoding="utf-8")
    route = re.search(
        r'@app\.get\("/api/trajectory"\).*?return asdict\(trajectory_hub\.read\(\)\)',
        body,
        flags=re.DOTALL,
    ).group(0)
    assert 'require("trajectory")' not in route, route


def test_the_navigation_profile_switch_is_not_gated_either():
    """Choosing a profile decides which planner and navigation settings are used.

    Nothing in it depends on what the engine can do, so a port flag would hide the
    switch on a family that works exactly as well: the profile tab and its two
    endpoints stay ungated.
    """
    app = _code(_sources())
    assert 'hasAction("profile")' not in app, (
        "the profile tab must not depend on a simulator capability"
    )
    assert 'id="tab-profile"' in _html()
    api = Path(__file__).resolve().parents[1] / "planner/adapters/http/server_api.py"
    body = api.read_text(encoding="utf-8")
    routes = re.search(
        r'@app\.get\("/api/profile"\).*?status_code=code\n        \)',
        body,
        flags=re.DOTALL,
    ).group(0)
    assert 'require("profile")' not in routes, routes


def test_the_profile_panel_shows_only_which_profile_is_running():
    """The switch is a demo prop: the name of the profile, nothing else.

    The panel used to render each profile's note and ``模型：<name>`` straight out of
    the config, and to paste the backend's English error onto the page. That put a
    vendor name and an endpoint string in front of an audience and turned a short
    switch into a wall of text.
    """
    app = _code(_sources())
    html = _html()
    assert "deepseek" not in (app + html).lower()
    assert "info.note" not in app, "the note is operator information, not screen text"
    assert "info.model" not in app, "the model name belongs in the log, not the panel"
    switch = re.search(
        r"async function applyProfile\(\).*?\n}", app, flags=re.DOTALL
    ).group(0)
    assert "error.message" not in switch, (
        "the profile panel must not quote the backend's message"
    )
    assert 'id="profile-choice"' in html, "the panel uses the same select as the other tabs"
    enabled = re.search(
        r"function renderProfiles\(\).*?const enabled =(.*?);\n", app, flags=re.DOTALL
    ).group(1)
    assert '"disconnected"' in enabled, (
        "a profile can be chosen before any scene is loaded"
    )
    assert '"disconnected"' in switch, "applying must work in that state too"
    assert 'setProfileFeedback("已切换", "ok")' in switch, (
        "a choice made before a scene starts must not stay on 切换中…"
    )


def test_the_scripts_load_in_dependency_order():
    """Classic scripts share one global scope, so order is the only wiring.

    ``app.js`` calls ``init()`` at the end and touches every panel, so it has to be
    the last tag; ``core.js`` owns the shared state the others read.
    """
    html = _html()
    tags = re.findall(r'<script src="/console/src/([a-z0-9.]+)"></script>', html)
    assert tags[: len(SCRIPTS)] == list(SCRIPTS), tags
    assert '<script type="module" src="/console/src/scene3d.js"></script>' in html


def test_the_3d_panel_is_optional_and_shares_one_snapshot():
    """The waypoint list owns the snapshot; the 3D panel only receives it."""
    scene = (SRC / "scene3d.js").read_text(encoding="utf-8")
    assert "Promise.race([import(CDN), timeout])" in scene, (
        "a library that never arrives must not hang the module"
    )
    assert "window.Trajectory3D" in scene
    flight = _code((SRC / "flight.js").read_text(encoding="utf-8"))
    assert "window.Trajectory3D.update(snapshot)" in flight
    assert '"/api/trajectory"' not in _code(scene), (
        "the panel must not poll a second copy of the same snapshot"
    )

