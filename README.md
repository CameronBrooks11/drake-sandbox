# drake-sandbox

Some playing around with the [Drake](https://drake.mit.edu) toolbox in Python. Parked; kept to
come back to.

## What's here

- **`test_drake_meshcat.py`:** a 4-DoF arm built as a Drake `MultibodyPlant` and shown in
  MeshCat, with a slider per joint. One yaw joint at the base, then three pitch joints. Its
  link lengths (300, 265, 265 and 150 mm) are the optimised design in Saoud et al.,
  "Design Optimization and Trajectory Planning of a Strawberry Harvesting Manipulator",
  vol. 13 no. 6 (2024), pp. 3948–3959,
  [doi:10.11591/eei.v13i6.7957](https://doi.org/10.11591/eei.v13i6.7957), open access under
  CC BY-SA.
- **`test_meshcat.py`:** the smallest MeshCat check: opens a viewer and draws a box.

## Running it

Drake's pip wheels are for Linux and macOS. The scripts were written against Drake 1.46.0,
MeshCat 0.3.2 and Python 3.11; `requirements.txt` pins the first two.

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
python test_drake_meshcat.py
```

The script prints a MeshCat URL. Open it, move the sliders `j1` to `j4`, and press Ctrl+C to stop.

## Licence

MIT; see [LICENSE](LICENSE).
