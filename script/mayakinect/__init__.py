"""MayaKinect Python package: scene setup, UI and tools around the C++ plugin."""

import importlib
import sys


def reload_all():
    """Reload every mayakinect module (Maya caches modules after the first import).

    Deepest modules are reloaded first, the package itself last.
    """
    names = [n for n in sys.modules if n == __name__ or n.startswith(__name__ + ".")]
    names.sort(key=lambda n: n.count("."), reverse=True)
    for name in names:
        module = sys.modules.get(name)
        if module is not None:
            importlib.reload(module)
    print("mayakinect: reloaded {} module(s)".format(len(names)))
