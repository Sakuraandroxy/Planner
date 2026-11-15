"""Describe a failed model-endpoint request in terms an operator can act on.

Every model call here goes through httpx, and each one used to report the same useless
line: "task parser request failed (HTTPStatusError)". Measured on this machine -- the
service was started without ``$VLN_API_KEY``, so the provider answered 401 and every
instruction in the console was rejected with a message that hid the status, the URL and
the cause completely.
"""

from __future__ import annotations

import httpx

#: What a status means for a configuration this project owns. Keys are the field names
#: in ``config/base.yaml``, so the reader can go fix the file instead of guessing.
_HINTS: dict[int, str] = {
    401: "no credential or an invalid one: set api_key, or export the variable named "
         "by api_key_env (VLN_API_KEY by default)",
    403: "the credential was refused: wrong key for this endpoint or model, or the "
         "account has no access to it",
    404: "no such endpoint or model: check url and model (url must include the "
         "provider's path prefix)",
    422: "the provider rejected the request body: check request_format and the "
         "generation options",
    429: "rate limited or out of quota",
}


def _request_of(exc: httpx.HTTPError):
    """``HTTPError.request`` is a property that raises when no request was attached.

    Unit tests and some transport failures build the error by hand, and an error
    reporter that itself throws would replace a readable rejection with a crash.
    """
    try:
        return exc.request
    except RuntimeError:
        return None


def describe_http_error(exc: httpx.HTTPError) -> str:
    """One line: status, endpoint, what to change. Never the key itself."""
    request = _request_of(exc)
    where = f" {request.url}" if request is not None else ""

    if isinstance(exc, httpx.HTTPStatusError):
        response = exc.response
        code = response.status_code
        parts = [f"HTTP {code} {response.reason_phrase}".rstrip() + (f" from{where}" if where else "")]
        hint = _HINTS.get(code)
        if hint is None and code >= 500:
            hint = "the provider itself failed; retry, then check its status page"
        if hint:
            parts.append(hint)
        # The provider's own body usually carries the real reason; it is the provider's
        # text, not ours, and it never echoes the Authorization header back.
        detail = " ".join((response.text or "").split())[:200]
        if detail:
            parts.append(f"provider said: {detail}")
        return "; ".join(parts)

    if isinstance(exc, httpx.TimeoutException):
        return f"timed out{where or ' the endpoint'}"
    if isinstance(exc, httpx.ConnectError):
        return f"cannot connect{where}; check url and that the endpoint is serving"
    return f"{type(exc).__name__}{where}: {exc}"
