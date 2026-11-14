"""Simulator infrastructure shared by every simulator: process stack and worker.

Nothing in here may import a simulator client library. Simulator-specific code
lives in ``planner.simulators.<name>``.
"""
