import os

__all__ = []
dir_path = os.path.dirname(os.path.realpath(__file__))

for root, dirs, files in os.walk(dir_path):
    for d in dirs:
        for f in os.listdir(os.path.join(root, d)):
            if f != "__init__.py" and f[-3:] == ".py":
                __all__.append(f[:-3])
