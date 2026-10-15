from .registry import ExtensionRegistry


def build_extension_registry() -> ExtensionRegistry:
    """Return an empty allow-list for explicit assembly; never auto-discover code."""
    return ExtensionRegistry()

