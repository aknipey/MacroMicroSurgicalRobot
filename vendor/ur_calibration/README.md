# Vendored from Universal_Robots_ROS2_Driver / ur_calibration

Why it's here: docs/UR_CALIBRATION_FIX.md. In short, our driver checkout (3.8.0) has a `ur_calibration`
that never connects and writes wrong link rotations, and we don't edit the driver repo.

- Source: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver, branch `jazzy`, commit
  `67578e0e8fe3f8366e046f5c96b8ce6675c4e00d` (3.10.0)
- Files, **byte-identical, do not edit** (fix things in `src/ur_calibration_extract.cpp` instead):

  | File | sha256 |
  |---|---|
  | `include/ur_calibration/calibration.hpp` | `1d9ec962d3b26b5d40c507fa60e88595e6112156eccb150c4ba20a3704f1d42e` |
  | `src/calibration.cpp` | `bd8809f356235509a0cec0fb9d11d951f8dd96c3cd5a48bd924d33289e6f0c72` |

- `calibration.cpp` here contains upstream fix `81df755` (URDF rpy convention). Compared with our 3.8.0
  checkout, that is the only difference in these two files.
- Licence: BSD-3-Clause, `LICENSE` (copied from the driver repo). Copyright 2019 FZI Forschungszentrum
  Informatik, on behalf of Universal Robots A/S (see the file headers).
- Not vendored: `calibration_consumer.*` and `calibration_correction.cpp`, replaced by
  `src/ur_calibration_extract.cpp`.
