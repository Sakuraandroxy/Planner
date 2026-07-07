"""配置系统 —— 读取 YAML 配置文件，全局可访问。

用法:
    from config import cfg
    detector_name = cfg["AGENT"]["DETECTOR"]  # "groundingdino"
"""
import os, yaml

_CFG = None

def get_cfg(path=None):
    global _CFG
    if _CFG is not None:
        return _CFG
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "default.yaml")
    with open(path, "r", encoding="utf-8") as f:
        _CFG = yaml.safe_load(f)
    return _CFG

# 导入时自动加载
cfg = get_cfg()
