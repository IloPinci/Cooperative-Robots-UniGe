# Bimanual manipulation

Two Panda arms controlled as one 14-joint system in MATLAB, with a PyBullet viewer.
The assignment and setup PDFs are in `handouts/`.

## First-time setup on each Ubuntu machine

Tested on Ubuntu 22.04 (x86-64), Python 3.10 and MATLAB R2025b.
Install **Robotics System Toolbox** and **DSP System Toolbox** in MATLAB.
Each teammate creates their own `.venv`; it is not included in Git.

On each machine:

```bash
sudo apt update
sudo apt install git curl ca-certificates libgl1 libx11-6
git clone https://github.com/IloPinci/Cooperative-Robots-UniGe.git
cd Cooperative-Robots-UniGe
```

If already cloned, open your existing repository instead. Before this branch is
merged, select it with `git switch charbel/bimanual`. Then `cd bimanual` and use
the appropriate setup below. Internet is needed for installation.

**Ubuntu 22.04 (tested):**

```bash
sudo apt install python3 python3-venv
./setup.sh
```

**Ubuntu 24.04 or newer:** use a separate Python 3.10 environment. The pinned
PyBullet version has no Python 3.12+ wheel, so skip `./setup.sh` on those machines.
From the cloned repository's `bimanual/` folder, before creating `.venv`:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
"$HOME/.local/bin/uv" venv --python 3.10 --seed .venv
.venv/bin/python -E -m pip install --only-binary=:all: -r requirements-linux.txt
.venv/bin/python -E -m pip check
.venv/bin/python -E check_setup.py
```

`uv` downloads Python 3.10 if needed, leaving system Python unchanged.
Both routes use the same pinned NumPy 2.2.6 and PyBullet 3.2.7 packages.
Expect `No broken requirements found` and `PASS` after checking both models and
meshes. The Ubuntu 24.04+ route still needs testing on teammates' machines.
Use `requirements-linux.txt`, rather than the old `simulator/requirements.txt`.
Sources: [uv installation](https://docs.astral.sh/uv/getting-started/installation/),
[Python environments](https://docs.astral.sh/uv/pip/environments/),
[PyBullet wheels](https://pypi.org/project/pybullet/3.2.7/#files).

## Run

From `bimanual/`, in a desktop terminal:

```bash
./run_simulator.sh
```

In a fresh MATLAB session, open this folder and run:

```matlab
start_bimanual
```

MATLAB runs for 20 simulated seconds and opens plots. The viewer holds the last
joint targets afterward; close its window or press Ctrl+C in its terminal to exit.
Run one viewer at a time. Rotate with Ctrl + left drag, pan with Ctrl + middle
drag (hold the wheel), and zoom with the wheel.

MATLAB and Python run on the same machine: no IP changes are needed between
teammates. MATLAB sends 14 doubles over localhost UDP port 1505, left joints
first, then right joints. Use a fresh MATLAB session when switching from UVMS.
If setup passes but the viewer cannot open, run `.venv/bin/python -E check_setup.py --gui`
to check that machine's OpenGL/display separately.

## What we implemented

- **Frames and goals:** separate arm bases, matching right-base placement in
  MATLAB and Python, tool offsets, and two grasp poses at the object ends.
- **Reach:** joint limits → minimum end-effector height → manipulability → tool
  pose tasks, with left then right tasks at each level. Height minimum is 0.15 m.
- **Capture:** once both tools are within 1 mm and 0.01 rad of their goals, store
  each fixed tool-to-object transform: `tTo = wTt \ wTo`.
- **Carry:** put the rigid-grasp task first, keep the safety tasks, and replace
  tool tracking with object motion toward `[0.65; -0.35; 0.28]` m and identity
  orientation. Each arm infers the object pose using `wTo = wTt*tTo`.
- **Stop:** when both inferred object poses reach the same tolerances, keep only
  the altitude tasks. The solver returns zero velocity while height is safe.
- **Viewer:** receive packets without a background thread, ignore invalid data,
  hold the last targets when MATLAB stops sending, and release resources on exit.

Added tasks in `matlab/tpik/`:

- `TaskJointLimits`: one activation per joint near either position limit.
- `TaskAltitude`: protects end-effector height above the plane at world z = 0,
  with a 0.10 m activation buffer above the 0.15 m minimum.
- `TaskRigidGrasp`: matches the two inferred object velocities and corrects pose
  disagreement using the captured transforms; fully active during carrying.
- `TaskObjectMotion`: uses the left arm's object estimate to track the object
  goal; the higher-priority rigid constraint couples the right arm to that motion.

The existing `TaskTool` and `TaskManipulability` are reused. Grasp capture assumes
rigid attachment; finger closing, contact sensing and a dynamic cube are not simulated.

## Smoothing options for review (not implemented)

Action switches are currently immediate. After capture, tool tracking was nearly
stopped, but the distant object goal immediately requests motion. This can cause
a sudden joint-velocity change. Existing inequality activation bells do not smooth
that action switch. Two options for later:

1. **Small change:** ramp the object-motion velocity reference from zero to its
   normal value after capture. This addresses the start of carrying only.
2. **General solution:** adapt the UVMS action-transition manager to blend task
   activations across actions, preserving task identity and priority changes.
   This needs adapting to the bimanual stacks rather than copying unchanged.

In either approach, activate the rigid-grasp constraint immediately at capture;
do not fade it. Review the resulting joint commands before choosing an option.
