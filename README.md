# Cooperative Robotics UniGe

Two independent exercises, each with its own MATLAB task classes and solver helpers:

- **`uvms/`**: underwater vehicle navigation and landing; MATLAB + Unity.
- **`bimanual/`**: two Panda arms manipulating one object; MATLAB + PyBullet.

## Run UVMS

Start the Unity UVMS viewer. In MATLAB:

```matlab
cd('/path/to/Cooperative-Robots-UniGe/uvms')
RobustMain
```

See [the UVMS guide](uvms/README.md) for task lists and transitions.

## Run bimanual

For first-time Ubuntu dependencies and Python compatibility, follow the
[bimanual setup guide](bimanual/README.md#first-time-setup-on-each-ubuntu-machine).

From the repository root, start the viewer in a terminal:

```bash
cd bimanual
./run_simulator.sh
```

In MATLAB:

```matlab
cd('/path/to/Cooperative-Robots-UniGe/bimanual')
start_bimanual
```

See [the bimanual guide](bimanual/README.md) and the assignment in
`bimanual/handouts/` for the reach, grasp, carry and stop implementation,
its assumptions and the proposed smoothing options.

## Working rules

- Work on one exercise at a time. The projects contain identically named classes
  such as `Task` and `ActionManager` --->  do not add the whole repository with `genpath`.
  Use a fresh MATLAB session when switching projects.
- Keep `.venv`, Python caches and generated files out of Git. Keep `panda.mat` and
  robot meshes: the bimanual model needs them.
- Keep solver copies separate for now: `RegPseudoInverse.m` differs between the
  supplied templates. Share helpers only after checking their behavior.
- Use small commits on an exercise branch, then a pull request to `main`.
