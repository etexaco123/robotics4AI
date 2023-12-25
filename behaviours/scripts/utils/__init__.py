import os

__all__ = []
dir_path = os.path.dirname(os.path.realpath(__file__))

for f in os.listdir(dir_path):
     if f != "__init__.py" and f[-3:] == ".py":
        __all__.append(f[:-3])