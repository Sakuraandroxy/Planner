"""Model backends.

Model modules know how to call a specific backend. They should not decide when
or why a backend is used in the closed loop; that belongs under
``agent.functions``.
"""
