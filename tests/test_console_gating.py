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
