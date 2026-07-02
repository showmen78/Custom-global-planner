# Custom_global_planner

This project finds shortest driving route from a raw start object position to a raw goal object position using the AD-map stack. CARLA is only used to read the `start_1` and `goal_1` object positions from the world and to draw the final route back into the simulator.

The routing logic, lane matching, lane-direction handling, centerline sampling, and adjacent-lane lookup are done outside CARLA with the AD-map libraries.

## 1. What this project uses

This project uses the CARLA AD-map repository:

- [carla-simulator/map](https://github.com/carla-simulator/map)

In this repo, that dependency is expected inside the `map_repo/` folder, and its Python bindings must already be built so that this file exists:

```bash
map_repo/install/setup.bash
```

The `map_repo/` folder in this project contains that repository:

- [carla-simulator/map](https://github.com/carla-simulator/map)

## 2. Project architecture

The architecture used in this project is shown below.

![Project architecture](image/project_architecture.png)

## 3. Short project flow

1. CARLA finds the raw positions of `start_1` and `goal_1`.
2. The OpenDRIVE map is loaded from `maps/Town05.xodr`.
3. AD-map matches the raw positions to nearby drivable lanes.
4. AD-map finds the  shortest route.
5. The route centerline is sampled into points.
6. Adjacent lane points can also be queried for one route sample.
7. CARLA draws the final route for visualization.

## 4. Repository layout

```text
.
├── activate.sh
├── maps/
│   └── Town05.xodr
├── map_repo/
├── scripts/
│   ├── admap_route.py
│   ├── carla_bridge.py
│   ├── carla_global_planner.py
│   ├── common.py
│   ├── object_finder.py
│   └── run_map_test.py
└── temporary/
```

## 5. Main files

- `scripts/run_map_test.py`: main entry point for the routing demo
- `scripts/admap_route.py`: CARLA-independent AD-map route logic
- `scripts/carla_bridge.py`: reads object positions from CARLA and draws saved route points
- `scripts/carla_global_planner.py`: optional CARLA global planner comparison path
- `activate.sh`: activates the AD-map environment and loads the built `map_repo` install

## 6. Requirements

The code uses only the Python standard library plus these two runtime dependencies:

- `ad_map_access`
- `carla`

They are listed in [requirement.txt](requirement.txt).

Notes:

- `ad_map_access` comes from the built [carla-simulator/map](https://github.com/carla-simulator/map) repo.
- `carla` comes from the local CARLA Python API / egg in your CARLA installation.

## 7. Before you run

Make sure these items are ready first:

1. CARLA is installed and the server can run.
2. The town loaded in CARLA matches the OpenDRIVE map you want to use.
3. In your map two objects (cube) named `start_1` and `goal_1` are available.
3. The `map_repo` dependency is built, and `map_repo/install/setup.bash` exists.
4. The CARLA Python environment exists.

This project currently uses local hard-coded paths, so update them if your machine is different:

- `scripts/run_map_test.py`
  - `XODR_PATH`
  - `CARLA_PYTHON`
- `scripts/carla_bridge.py`
  - `CARLA_ROOT`


## 8. How to run this project

### Step 1: Start CARLA

Start the CARLA server, load the correct map, and press **Play** in the simulator.

### Step 2: Activate the project environment

From the project root:

```bash
source activate.sh
```

This script:

- activates the `admap` environment
- sources `map_repo/install/setup.bash`
- adds `scripts/` to `PYTHONPATH`

### Step 3: Run the main route demo

```bash
python scripts/run_map_test.py
```

### Step 4: Check the result in CARLA

The script will:

1. read the raw `start_1` and `goal_1` object positions
2. load the OpenDRIVE map
3. compute the route with AD-map
4. sample route points
5. optionally query adjacent lane points
6. draw the AD-map route in CARLA

## 9. Optional route comparison

If you want to compare the AD-map result against the CARLA global planner, enable the booleans in `scripts/run_map_test.py`:

```python
show_carla_route = True
show_ad_map_route = True
```

When enabled:

- AD-map route is drawn in yellow
- CARLA global planner route is drawn in blue

## 10. Example commands

```bash
cd ~/Desktop/Custom_global_planner
source activate.sh
python scripts/run_map_test.py
```

Or in one command:

```bash
bash -lc '
source activate.sh
python scripts/run_map_test.py
'
```

## 11. Output

During a normal run, the project prints information such as:

- CARLA start and goal object positions
- number of parsed lanes
- matched start and goal lane ids
- route length
- number of sampled route points
- adjacent lane points for the selected route sample

Temporary JSON outputs are written into the `temporary/` folder.

![Project output](image/output_screenshot.png)

## 12. Current issues

There are still some known issues in the current version:

1. Sometimes the project does not return a path even when a valid path is available.
2. Sometimes the route goes in the wrong lane direction or chooses the wrong lane.
