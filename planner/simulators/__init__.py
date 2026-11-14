"""Simulator-specific implementations. One package per simulator, peers.

Only code inside this package may import a simulator client library. The
architecture test in ``tests/test_architecture.py`` enforces that, because it is
the rule that keeps the shared task layer free of simulator names.
"""
