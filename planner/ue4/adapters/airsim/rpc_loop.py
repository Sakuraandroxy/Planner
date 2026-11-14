"""Owner-side compatibility with msgpack-rpc's restartable Tornado loop.

msgpack-rpc 0.4.1 restarts its loop inside a timeout callback. Tornado 4.5
also leaves its running flag set if a callback raises KeyboardInterrupt.
Keep both behaviours local to each AirSim client; do not patch the SDK.
"""


class OwnerRpcLoop:
    def __init__(self, loop):
        self._delegate = loop
        # msgpack-rpc's TCP transport uses this loop to create its IOStream.
        self._ioloop = loop._ioloop
        self._running = False

    def start(self):
        if self._running:
            # The timeout callback has already requested stop. Let the outer
            # Future.get() pump restart after that loop invocation unwinds.
            return
        self._running = True
        try:
            self._delegate.start()
        except BaseException:
            # Tornado 4.5's finally does not reset its running flag on interrupt.
            # The loop has unwound here, so the next cleanup RPC may restart it.
            self._delegate.stop()
            raise
        finally:
            self._running = False

    def stop(self):
        self._delegate.stop()

    def close(self):
        self._delegate.dettach_periodic_callback()
        self._ioloop.close()


def configure_rpc_loop(vehicle_client):
    """Adapt the SDK-owned loop before any requests create their Futures."""
    rpc = getattr(vehicle_client, "client", None)
    loop = getattr(rpc, "_loop", None)
    if loop is not None:
        rpc._loop = OwnerRpcLoop(loop)


def close_rpc_client(vehicle_client):
    rpc = vehicle_client.client
    try:
        rpc.close()
    finally:
        if isinstance(getattr(rpc, "_loop", None), OwnerRpcLoop):
            rpc._loop.close()
