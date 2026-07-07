"""Background thread: continuously capture frames for the frontend."""
import time, threading, io
import airsim
from PIL import Image
import numpy as np


class FrameCapturer:
    """Daemon thread that captures color + depth frames at a fixed interval."""

    def __init__(self, capture_interval: float = 0.1, include_depth: bool = True,
                 depth_interval: float = 2.0):
        self._interval = capture_interval
        self._include_depth = include_depth
        self._depth_interval = depth_interval
        self._last_depth_time = 0.0
        self._client = None
        self._thread = None
        self._running = False
        self._last_error = ""

    def start(self, state):
        """Start background frame capture into SharedState."""
        self._client = airsim.MultirotorClient()
        self._client.confirmConnection()
        self._running = True
        self._thread = threading.Thread(target=self._loop, args=(state,), daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _loop(self, state):
        while self._running:
            try:
                responses = self._client.simGetImages([
                    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, True)
                ])
                if responses and responses[0].image_data_uint8:
                    img = Image.open(io.BytesIO(bytes(responses[0].image_data_uint8)))
                    buf = io.BytesIO()
                    img.convert("RGB").save(buf, format="JPEG", quality=85)
                    state.set_frame(buf.getvalue())
            except Exception as exc:
                msg = f"rgb: {exc}"
                if msg != self._last_error:
                    self._last_error = msg
                    print(f"[FrameCapturer] capture error: {msg}")

            now = time.monotonic()
            if self._include_depth and now - self._last_depth_time >= self._depth_interval:
                self._last_depth_time = now
                try:
                    depth_resp = self._client.simGetImages([
                        airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)
                    ])
                    if depth_resp and depth_resp[0].image_data_float:
                        d = np.array(depth_resp[0].image_data_float, dtype=np.float32)
                        d = d.reshape(depth_resp[0].height, depth_resp[0].width)
                        d = np.clip(d, 0, 100.0)
                        d8 = (d / 100.0 * 255).astype(np.uint8)
                        img = Image.fromarray(d8, mode="L")
                        if img.width > 256:
                            preview_height = max(1, int(img.height * 256 / img.width))
                            img = img.resize((256, preview_height), Image.Resampling.BILINEAR)
                        img = img.convert("RGB")
                        bd = io.BytesIO()
                        img.save(bd, format="PNG")
                        state.set_depth_frame(bd.getvalue())
                    else:
                        depth_info = "missing response"
                        if depth_resp:
                            depth_info = f"width={depth_resp[0].width} height={depth_resp[0].height} floats={len(depth_resp[0].image_data_float)}"
                        msg = f"empty_depth_response: {depth_info}"
                        if self._last_error != msg:
                            self._last_error = msg
                            print(f"[FrameCapturer] {msg}")
                except Exception as exc:
                    msg = f"depth: {exc}"
                    if msg != self._last_error:
                        self._last_error = msg
                        print(f"[FrameCapturer] capture error: {exc}")
            if self._interval > 0:
                time.sleep(self._interval)

