"""Regression tests for serialization of AirSim msgpackrpc futures."""

from __future__ import annotations

import threading

from sim.airsim_client import _SynchronizedAirSimRpcClient


class _BlockingFuture:
    def __init__(self, entered: threading.Event, release: threading.Event):
        self.entered = entered
        self.release = release

    def join(self):
        self.entered.set()
        assert self.release.wait(timeout=1.0)
        return "joined"


class _RawRpcClient:
    def __init__(self):
        self.join_entered = threading.Event()
        self.release_join = threading.Event()
        self.pose_called = threading.Event()

    def hoverAsync(self):
        return _BlockingFuture(self.join_entered, self.release_join)

    def simGetVehiclePose(self):
        self.pose_called.set()
        return "pose"


def test_waited_async_rpc_holds_lock_until_future_join_finishes():
    raw = _RawRpcClient()
    client = _SynchronizedAirSimRpcClient(raw)
    results = []

    wait_thread = threading.Thread(
        target=lambda: results.append(client.call_async_and_wait("hoverAsync")),
    )
    wait_thread.start()
    assert raw.join_entered.wait(timeout=1.0)

    pose_thread = threading.Thread(target=lambda: results.append(client.simGetVehiclePose()))
    pose_thread.start()

    # The pose RPC must remain outside the raw msgpackrpc client while the
    # first Future owns its IOLoop under the shared lock.
    assert not raw.pose_called.wait(timeout=0.1)

    raw.release_join.set()
    wait_thread.join(timeout=1.0)
    pose_thread.join(timeout=1.0)

    assert not wait_thread.is_alive()
    assert not pose_thread.is_alive()
    assert raw.pose_called.is_set()
    assert sorted(results) == ["joined", "pose"]


def test_nonblocking_async_submission_still_returns_future_without_joining():
    raw = _RawRpcClient()
    client = _SynchronizedAirSimRpcClient(raw)

    future = client.hoverAsync()

    assert isinstance(future, _BlockingFuture)
    assert not raw.join_entered.is_set()
