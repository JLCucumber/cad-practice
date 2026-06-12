# cad-practice

A personal collection of CAD practice parts modelled with [CadQuery](https://cadquery.readthedocs.io/).
Each part is a small, self-contained exercise — a Python source file plus its exported STL.

## Layout

```
cad-practice/
├── parts/
│   └── 001_basic_box/
│       ├── box.py      # CadQuery source
│       └── box.stl     # exported mesh
├── requirements.txt    # pinned cadquery version
├── .gitignore
└── README.md
```

Parts are numbered (`NNN_name`). To add a new one, create a new folder under `parts/`
with the next number.

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

CadQuery pulls in OCP / OpenCascade, so the first install is large and may take a while.

## Run a part

Each `*.py` exports its STL next to the source when run directly:

```bash
cd parts/001_basic_box
python box.py        # writes box.stl
```

Open the resulting `.stl` in any mesh viewer (e.g. `f3d`, MeshLab, PrusaSlicer) to inspect it.

## License

MIT — see [LICENSE](LICENSE).
