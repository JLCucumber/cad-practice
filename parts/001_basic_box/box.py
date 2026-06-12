"""001_basic_box — minimal CadQuery demo part.

Builds a simple parametric box and exports it as STL next to this file.
Run: python box.py
"""

from pathlib import Path

import cadquery as cq

# Box dimensions in millimetres.
LENGTH = 40.0
WIDTH = 30.0
HEIGHT = 20.0


def make_box(length: float = LENGTH, width: float = WIDTH, height: float = HEIGHT) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height)


if __name__ == "__main__":
    result = make_box()
    out = Path(__file__).with_name("box.stl")
    cq.exporters.export(result, str(out))
    print(f"Wrote {out}")
