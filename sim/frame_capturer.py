"""后台帧抓取器 —— daemon 线程持续从 AirSim 取帧推送给前端。

一次 simGetImages 同时取 RGB + DepthPerspective，主循环零等待读缓存。
解决周报 7-1 中提到的 "AirSim 取图 ~7s" 问题。
"""

import threading
import time as _time
from io import BytesIO
from typing import Optional, Tuple


class FrameCapturer:
    """后台 daemon 线程：持续抓取 RGB + 深度图。

    - 前端预览：RGB 实时推送 SharedState
    - 主循环：直接读 latest_frame / latest_depth（零等待）
    - 独立 AirSim 连接，不与主线程共享 IOLoop
    """

    def __init__(self, state, interval=0.1):
        import airsim
        from config import cfg
        port = int(cfg.get("SIM", {}).get("AIRSIM_PORT", 41451))
        self.client = airsim.MultirotorClient(port=port)
        self.state = state
        self.interval = interval
        self._stop = threading.Event()
        self._paused = threading.Event()
        self._busy = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

        # 线程安全的帧缓存（主循环直接读）
        self._latest_rgb: Optional[bytes] = None    # JPEG 字节（给 VLM）
        self._latest_depth: Optional['numpy.ndarray'] = None  # float32 [H,W] 米
        self._latest_rgb_png: Optional[bytes] = None  # PNG 字节（给前端）

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def pause(self, wait: bool = True, timeout: float = 2.0):
        """Pause background RPCs to avoid competing with the main control loop."""
        self._paused.set()
        if not wait:
            return
        deadline = _time.perf_counter() + max(0.0, timeout)
        while self._busy.is_set() and _time.perf_counter() < deadline:
            _time.sleep(0.01)

    def resume(self):
        self._paused.clear()

    def get_latest_frame(self) -> Tuple[Optional[bytes], Optional['numpy.ndarray']]:
        """主循环调用：获取最近一次抓取的 (RGB_JPEG_bytes, depth_meters)。

        返回两个都是 None 表示还没抓到第一帧。
        """
        with self._lock:
            return self._latest_rgb, self._latest_depth

    def get_latest_rgb_png(self) -> Optional[bytes]:
        """获取最近一次 RGB PNG（前端 /frame 接口用）。"""
        with self._lock:
            return self._latest_rgb_png

    def _loop(self):
        """一次 simGetImages 同时取 RGB + DepthPerspective，原子写入缓存。"""
        import airsim
        from PIL import Image
        import io as _io
        import numpy as np

        while not self._stop.is_set():
            if self._paused.is_set():
                _time.sleep(min(self.interval, 0.05))
                continue
            try:
                self._busy.set()
                responses = self.client.simGetImages([
                    airsim.ImageRequest("front_center", airsim.ImageType.Scene,
                                        False, True),  # RGB, compressed
                    airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective,
                                        True, False),  # Depth, float32
                ])

                rgb_bytes = None
                depth_meters = None
                rgb_png = None

                if responses and len(responses) >= 2:
                    # RGB
                    if responses[0].image_data_uint8:
                        rgb_img = Image.open(_io.BytesIO(bytes(responses[0].image_data_uint8)))
                        rgb_img = rgb_img.convert("RGB")  # AirSim 返回 RGBA，JPEG 不支持
                        jpeg_buf = BytesIO()
                        rgb_img.save(jpeg_buf, format="JPEG", quality=85)
                        rgb_bytes = jpeg_buf.getvalue()

                        png_buf = BytesIO()
                        rgb_img.save(png_buf, format="PNG")
                        rgb_png = png_buf.getvalue()

                    # Depth
                    if responses[1].image_data_float:
                        r = responses[1]
                        depth_meters = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)

                # 原子写入缓存（主循环读）
                with self._lock:
                    self._latest_rgb = rgb_bytes
                    self._latest_depth = depth_meters
                    self._latest_rgb_png = rgb_png

            except Exception as exc:
                # 首 3 次错误打印日志，之后静默（避免刷屏）
                if not hasattr(self, '_error_count'):
                    self._error_count = 0
                self._error_count += 1
                if self._error_count <= 3:
                    print(f"  [FrameCapturer] error #{self._error_count}: {exc}")
                elif self._error_count == 4:
                    print(f"  [FrameCapturer] suppressing further errors...")
            finally:
                self._busy.clear()
            _time.sleep(self.interval)
