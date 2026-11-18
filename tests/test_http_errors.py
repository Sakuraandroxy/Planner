"""A rejected instruction has to name its own cause.

Started the service without ``$VLN_API_KEY`` and the console answered every instruction
with "task parser request failed (HTTPStatusError)". The status, the URL and the reason
were all in the exception and none of them reached the operator.
"""

import httpx
import pytest

from planner.adapters.http_errors import describe_http_error
from planner.errors import ProtocolError


def _status(code: int, text: str = "") -> httpx.HTTPStatusError:
    request = httpx.Request("POST", "https://api.deepseek.com/anthropic/v1/messages")
    response = httpx.Response(code, request=request, text=text)
    return httpx.HTTPStatusError(f"client error {code}", request=request, response=response)


def test_a_401_says_which_knob_to_turn():
    line = describe_http_error(_status(401, '{"error":{"type":"authentication_error"}}'))
    assert "HTTP 401" in line
    assert "api_key" in line and "VLN_API_KEY" in line
    assert "authentication_error" in line          # the provider's own reason survives


def test_a_404_points_at_the_url_and_model():
    line = describe_http_error(_status(404))
    assert "HTTP 404" in line and "model" in line


def test_server_side_failure_is_called_provider_side():
    line = describe_http_error(_status(503, "upstream busy"))
    assert "HTTP 503" in line and "provider" in line and "upstream busy" in line


def test_an_error_without_a_request_attached_still_describes_itself():
    """``HTTPError.request`` raises when unset; the reporter must not join in."""
    assert "cannot connect" in describe_http_error(httpx.ConnectError("down"))


def test_the_key_never_appears_in_the_message():
    line = describe_http_error(_status(401, "nope"))
    assert "sk-" not in line and "Bearer" not in line


def test_the_task_parser_carries_the_reason_into_the_rejection(monkeypatch):
    from planner.adapters.task_parser import openai_task_parser as module

    def fake_post(url, **kwargs):
        raise _status(401, '{"error":{"type":"authentication_error"}}')

    monkeypatch.setattr(module.httpx, "post", fake_post)
    parser = module.OpenAITaskParser(
        url="https://api.deepseek.com/anthropic/v1/messages",
        model="deepseek-flash",
        api_key="",
        timeout_s=5.0,
    )
    with pytest.raises(ProtocolError) as raised:
        parser.parse("向前飞两米")
    message = str(raised.value)
    assert "HTTP 401" in message and "VLN_API_KEY" in message
    # The cause is chained, not flattened away.
    assert isinstance(raised.value.__cause__, httpx.HTTPStatusError)


def test_a_configured_endpoint_without_a_credential_warns_at_build_time(caplog):
    import logging
    import os

    from planner.application.factories import build_task_parser
    from planner.config import default_config_path, load_config

    os.environ.pop("VLN_API_KEY", None)
    config = load_config(default_config_path())
    with caplog.at_level(logging.WARNING, logger="planner.application.factories"):
        build_task_parser(config, kinds=None, aerial=True)
    text = caplog.text
    assert "task_parser" in text and "no credential was found" in text
