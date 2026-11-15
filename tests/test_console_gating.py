"""Capability gating may not hide the card the operator needs first.

``applyCapabilities`` used to resolve its target with ``node.closest("section")``, and
the models markup sat *inside* the world card. AirSim advertises
``actions: ["weather"]`` with no model control, so hiding "models" hid the whole world
card -- every map selector vanished a second after page load and there was no way to
start a scene. ProjectAirSim offered model control, which is why this only broke on
UE4 and survived every earlier look at the console.
"""

import re
from pathlib import Path

CONSOLE = Path(__file__).resolve().parents[1] / "console"


def _html() -> str:
    return (CONSOLE / "index.html").read_text(encoding="utf-8")


def _sections() -> list[str]:
    return re.findall(r"<section\b.*?</section>", _html(), flags=re.S)


def test_gated_ids_exist_in_the_markup():
    """A gate that names a missing element silently does nothing."""
    app = (CONSOLE / "app.js").read_text(encoding="utf-8")
    body = re.search(r"function applyCapabilities\(\).*?\n}", app, flags=re.S).group(0)
    gated = set(re.findall(r'\$\("([a-z0-9-]+)"\)', body))
    assert gated, "applyCapabilities references no ids at all"
    have = set(re.findall(r'id="([^"]+)"', _html()))
    assert gated <= have, f"gating references unknown ids: {sorted(gated - have)}"


def test_the_world_card_does_not_own_other_capabilities():
    for section in _sections():
        if 'id="world-map"' not in section:
            continue
        for foreign in ("models-apply", "models-card", "weather-apply", "weather-card"):
            assert foreign not in section, (
                f"the world card contains {foreign!r}; a capability gate that hides it "
                "would hide the map selector with it"
            )
        return
    raise AssertionError("no card contains id=\"world-map\"")


def _code(text: str) -> str:
    """Drop line comments, so a check on code cannot match prose about the code."""
    return "\n".join(line.split("//")[0] for line in text.splitlines())


def test_gating_uses_explicit_card_ids_not_dom_nesting():
    app = _code((CONSOLE / "app.js").read_text(encoding="utf-8"))
    assert 'closest("section")' not in app, (
        "capability gating must name the card it hides; resolving it by DOM nesting "
        "made moving a control between cards hide the wrong panel"
    )
