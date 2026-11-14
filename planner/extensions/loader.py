from .registry import ExtensionRegistry


def build_extension_registry() -> ExtensionRegistry:
    """Return the explicit extension registry; the base build has no plugins."""
    return ExtensionRegistry()

