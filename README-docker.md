# xasv-simu Docker files (fixed v2)

This version applies the practical fix for the `catkin init` problem:

- **do not persist `/ws/.catkin_tools` as a separate Docker volume**
- initialize/configure/build the catkin workspace inside `scripts/docker_prepare_sim.sh`
- keep only `build`, `devel` and `logs` as persistent Docker volumes

## First-time workflow

### 1) Build the images

```bash
sudo docker compose build sim sitl tools
```

### 2) Prepare the simulation workspace

```bash
bash scripts/docker_prepare_sim.sh
```

This runs:

- `catkin init`
- `catkin config --skiplist ardupilot_hitl`
- `catkin build`

inside the `sim` container.

### 3) Start the simulator

```bash
bash scripts/docker_run_sim.sh
```

### 4) Start SITL

In another terminal:

```bash
bash scripts/docker_run_sitl.sh
```

## Notes

- `sim` still runs as `root` to avoid volume-permission issues
- `ardupilot_hitl` is intentionally skipped in the initial simulation workspace build
- the external dependencies normally handled by `install_deps.sh` (`livox_laser_simulation` and `ping360_gazebo`) are baked into the `sim` image
