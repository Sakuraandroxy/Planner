"""图像编码工具 —— 一次编码 JPEG/base64，供 detector/planner 等模块复用。

用法（入口层）:
    from agent.common.image_encoder import ImageEncoder
    encoder = ImageEncoder()
    encoder.encode_front(frame)         # 编码并缓存
    encoder.encode_down(down_frame)
    # 后续 detector.detect(), planner.plan() 内部可选调 get_cached_*()

模块可选消费（不强制，不知道此模块就自己编码）:
    from agent.common.image_encoder import get_cached_front_b64
    b64 = get_cached_front_b64()
    if b64:
        ...  # 复用
    else:
        ...  # 自己编码
"""

import base64
import io
from io import BytesIO
from typing import Optional


# 模块级缓存（单例）
_front_jpeg_b64: Optional[str] = None
_down_jpeg_b64: Optional[str] = None


class ImageEncoder:
    """PIL Image → JPEG base64 编码器，结果写入模块级缓存。"""

    @staticmethod
    def encode_front(frame) -> Optional[str]:
        """编码前视图为 JPEG base64，写入全局缓存。"""
        global _front_jpeg_b64
        if frame is None:
            return None
        if frame.mode == "RGBA":
            frame = frame.convert("RGB")
        buf = BytesIO()
        frame.save(buf, format="JPEG", quality=85)
        _front_jpeg_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")
        return _front_jpeg_b64

    @staticmethod
    def encode_down(down_frame) -> Optional[str]:
        """编码下视图为 JPEG base64，写入全局缓存。"""
        global _down_jpeg_b64
        if down_frame is None:
            return None
        buf = BytesIO()
        down_frame.save(buf, format="JPEG", quality=85)
        _down_jpeg_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")
        return _down_jpeg_b64

    @staticmethod
    def clear():
        """清空缓存（通常不需要，下一步会覆盖）。"""
        global _front_jpeg_b64, _down_jpeg_b64
        _front_jpeg_b64 = None
        _down_jpeg_b64 = None


# ─── 模块级 getter（供 detector/planner 等可选消费） ───

def get_cached_front_b64() -> Optional[str]:
    """获取最近一次编码的前视图 JPEG base64（不消费，可重复读取）。"""
    return _front_jpeg_b64


def get_cached_down_b64() -> Optional[str]:
    """获取最近一次编码的下视图 JPEG base64。"""
    return _down_jpeg_b64


def has_cached_front() -> bool:
    return _front_jpeg_b64 is not None
