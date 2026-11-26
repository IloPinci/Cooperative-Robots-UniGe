# Bimanual exercise setup

## Start the exercise

Open a terminal in this folder and run:

```bash
./run_simulator.sh
```

Keep that terminal open. The launcher selects this exercise's `.venv` automatically and ignores inherited
`PYTHONPATH` settings from ROS workspaces.
In MATLAB, set Current Folder to this `bimanual/` folder, then run:

```matlab
start_bimanual
```

Alternatively, set MATLAB's Current Folder to
`matlab/` and run `BimanualMain`.
The supplied run lasts 20 simulated seconds and opens result plots afterward.
Start only one Python viewer at a time: all viewers use the same UDP port.
Stop the Python process from its terminal with Ctrl+C; the supplied receiver thread
may require a second Ctrl+C to exit. Closing only the viewer window may leave that
thread running.

Viewer mouse controls from the course guide:

- Rotate: Ctrl + left mouse drag.
- Pan: Ctrl + Alt + left mouse drag.

## Environment

- Ubuntu 22.04, Python 3.10.12; isolated environment in `.venv`.
- NumPy 2.2.6 and PyBullet 3.2.7, pinned in `requirements-linux.txt`.
- MATLAB R2025b Update 1; Robotics System Toolbox and DSP System Toolbox.
- Robot URDFs and meshes are already supplied in `simulator/panda_robot`.
- Plane assets are included in the PyBullet package through `pybullet_data`.
- Communication: localhost `127.0.0.1`, UDP **1505**, fourteen double values per
  packet (left seven joints, then right seven joints), 112 bytes total.

The supplied PDF's UDP test uses port 1500. Use **1505** for this version,
matching both `UDPInterface.m` and `franka_panda_simulation.py`.
MATLAB and Python run on this same Ubuntu machine, so no VM, port forwarding,
or firewall changes are needed.

The original simulator `simulator/requirements.txt` is preserved. It pins a broad set of
2021 packages, including plotting/data libraries that this simulator does not
import. Use the root `requirements-linux.txt` for the tested Python 3.10 setup.
Package sources: [NumPy on PyPI](https://pypi.org/project/numpy/2.2.6/),
[PyBullet on PyPI](https://pypi.org/project/pybullet/3.2.7/).

## Recreate or check the environment

```bash
./setup.sh
```

This creates/reuses `.venv`, installs the pinned runtime dependencies, checks
package consistency, and loads both robots in a headless simulation.
To check an existing installation without downloading anything:

```bash
.venv/bin/python -E -m pip check
.venv/bin/python -E check_setup.py
```

To also open and close a test OpenGL window:

```bash
.venv/bin/python -E check_setup.py --gui
```

## Exercise status

Verified on 20 November 2026:

- Pinned packages installed; downloaded wheels matched official PyPI SHA256 hashes.
- Package consistency check and both robot models passed, including all meshes.
- GUI/OpenGL test passed on the Intel/Mesa graphics driver.
- Full supplied MATLAB controller completed 20 simulated seconds and emitted
  4,000 valid UDP packets, each containing fourteen finite doubles.
- Supplied GUI simulator launched from another working directory through
  `run_simulator.sh` and stayed running after 100 test UDP packets.
- `setup.sh` reran successfully against the installed dependencies without downloads.

The environment setup is separate from completing the assignment. The supplied
`matlab/BimanualMain.m` still has TODOs for the right-arm world/base transform and
distinct grasp/tool goals. In the Python scene, the right base is at
`[1.05, 0, 0]` with a rotation of pi about world Z; the MATLAB template currently
initializes both base transforms to identity. These exercise choices have been
left for your implementation. A successful run verifies execution and
communication, not completion of the cooperative manipulation tasks.

## Local working copy

The MATLAB template is in `matlab/`, the Python viewer and robot meshes are in
`simulator/`, and course PDFs are in `handouts/`. The original course download
remains outside this Git repository.

This machine reuses the existing Python environment through an ignored `.venv`
symlink to the original setup. Do not delete that environment while using the
link. On another machine, run `./setup.sh` to create a normal local `.venv`.

Read `matlab/BimanualMain.m`, then `PandaArm.m`, `tpik/TaskTool.m`,
`ActionManager.m`, `BimanualSim.m`, and `UDPInterface.m` (all under `matlab/`).
The assignment's frames, grasp goals, safety tasks and cooperative phases still
need implementing; this branch initially preserves the supplied controller.
