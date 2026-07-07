"""通用模块——任务管理、预热、图像编码等。"""
from agent.common.task_manager import TaskManager
from agent.common.warmup import warmup_from_config
from agent.common.image_encoder import ImageEncoder, get_cached_front_b64, get_cached_down_b64
