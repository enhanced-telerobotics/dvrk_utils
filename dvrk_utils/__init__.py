"""dVRK utilities with optional TF dependencies loaded on demand."""

from importlib import import_module

__all__ = ['stamped_to_se3', 'frame_to_se3', 'se3_to_frame']


def __getattr__(name):
    if name in __all__:
        value = getattr(import_module('.utils', __name__), name)
        globals()[name] = value
        return value
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
