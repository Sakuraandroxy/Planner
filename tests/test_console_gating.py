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


def _html() -> str:
    return (CONSOLE / "index.html").read_text(encoding="utf-8")


def test_gated_ids_exist_in_the_markup():
    """A gate that names a missing element silently does nothing."""
    app = (CONSOLE / "app.js").read_text(encoding="utf-8")
    body = re.search(r"function applyCapabilities\(\).*?\n}", app, flags=re.S).group(0)
    gated = set(re.findall(r'\$\("([a-z0-9-]+)"\)', body))
    assert gated, "applyCapabilities references no ids at all"
    have = set(re.findall(r'id="([^"]+)"', _html()))
    assert gated <= have, f"gating references unknown ids: {sorted(gated - have)}"


def test_capability_gating_hides_a_tab_not_the_settings_card():
    app = _code((CONSOLE / "app.js").read_text(encoding="utf-8"))
    body = re.search(r"function applyCapabilities\(\).*?\n}", app, flags=re.S).group(0)
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
    app = _code((CONSOLE / "app.js").read_text(encoding="utf-8"))
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
    app = _code((CONSOLE / "app.js").read_text(encoding="utf-8"))
    assert 'hasAction("trajectory")' not in app, (
        "the remaining-trajectory card must not depend on a simulator capability"
    )
    assert 'id="trajectory-card"' in _html()
    api = Path(__file__).resolve().parents[1] / "planner/adapters/http/server_api.py"
    body = api.read_text(encoding="utf-8")
    route = re.search(
        r'@app\.get\("/api/trajectory"\).*?return asdict\(snapshot\)', body, flags=re.S
    ).group(0)
    assert 'require("trajectory")' not in route, route


def test_the_navigation_profile_switch_is_not_gated_either():
    """Choosing a profile decides which planner and navigation settings are used.

    Nothing in it depends on what the engine can do, so a port flag would hide the
    switch on a family that works exactly as well: the profile tab and its two
    endpoints stay ungated.
    """
    app = _code((CONSOLE / "app.js").read_text(encoding="utf-8"))
    assert 'hasAction("profile")' not in app, (
        "the profile tab must not depend on a simulator capability"
    )
    assert 'id="tab-profile"' in _html()
    api = Path(__file__).resolve().parents[1] / "planner/adapters/http/server_api.py"
    body = api.read_text(encoding="utf-8")
    routes = re.search(
        r'@app\.get\("/api/profile"\).*?status_code=code\n        \)',
        body,
        flags=re.S,
    ).group(0)
    assert 'require("profile")' not in routes, routes


def test_the_profile_panel_shows_only_which_profile_is_running():
    """The switch is a demo prop: the name of the profile, nothing else.

    The panel used to render each profile's note and ``模型：<name>`` straight out of
    the config, and to paste the backend's English error onto the page. That put a
    vendor name and an endpoint string in front of an audience and turned a short
    switch into a wall of text.
    """
    app = _code((CONSOLE / "app.js").read_text(encoding="utf-8"))
    html = _html()
    assert "deepseek" not in (app + html).lower()
    assert "info.note" not in app, "the note is operator information, not screen text"
    assert "info.model" not in app, "the model name belongs in the log, not the panel"
    switch = re.search(
        r"async function applyProfile\(\).*?\n}", app, flags=re.S
    ).group(0)
    assert "error.message" not in switch, (
        "the profile panel must not quote the backend's message"
    )
    assert 'id="profile-choice"' in html, "the panel uses the same select as the other tabs"
    assert 'id="profile-current"' in html, "the panel keeps the status line the other tabs have"
    status = re.search(
        r'\$\("profile-current"\)\.textContent =.*?;', app, flags=re.S
    ).group(0)
    assert "profileName(" in status, "the status line shows the profile name"
    assert "model" not in status and "note" not in status, (
        "the status line carries the name, or one word while switching"
    )

