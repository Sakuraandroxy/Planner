"""The stop-scene control has to reach the endpoint that takes the stack down.

A button that posts nowhere is invisible in tests and obvious to the operator, so
these two checks keep the markup, the listener and the route name together.
"""

from pathlib import Path

CONSOLE = Path(__file__).resolve().parents[1] / "console"
SRC = CONSOLE / "src"
SCRIPTS = ("core.js", "world.js", "settings.js", "flight.js", "app.js")


def test_the_console_offers_a_stop_scene_button():
    html = (CONSOLE / "index.html").read_text(encoding="utf-8")
    assert 'id="world-stop"' in html


def test_the_button_posts_to_the_stop_endpoint():
    app = "\n".join((SRC / name).read_text(encoding="utf-8") for name in SCRIPTS)
    assert '"/api/world/stop"' in app
    assert '$("world-stop").addEventListener' in app
