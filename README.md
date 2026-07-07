# Custom_global_planner

This project finds the shortest driving route from a raw start object position to a raw goal object position using the AD-map stack. CARLA is only used to read the `start_1` and `goal_1` object positions from the world and to draw the final route back into the simulator.

The route search, lane matching, lane-direction handling, centerline sampling, and adjacent-lane lookup are done outside CARLA with the AD-map libraries and the custom routing code in this repo.

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
4. The matched positions are snapped to lane-center routing points.
5. A custom shortest-path search uses lane connectivity from `map_repo` to build a legal lane-by-lane route.
6. Only forward lane continuation and same-direction lane changes are allowed while building the route.
7. The final lane-consistent route centerline is sampled into points.
8. Adjacent lane points can also be queried for one route sample.
9. CARLA draws the final route for visualization.

## 4. Route cost model

The custom global planner uses a lane-level graph search with a simple custom cost design.

- Forward movement cost = length of the next lane
- Lane-change cost = fixed `LANE_CHANGE_PENALTY`
- Illegal moves are ignored from the graph
- Lane changes inside intersection lanes are blocked

The current movement-cost summary is shown below.

![Route cost table](image/table_image.png)

In practice, this means the planner prefers shorter legal forward progress, while lane changes are only taken when their fixed penalty still leads to a lower total route cost.

## 5. Recent changes and fixes

These are the main routing changes that were added to fix the wrong-lane and opposite-direction issues:

- Replaced the old final route reconstruction with a custom lane-by-lane search built on top of `map_repo` lane connectivity.
- Stopped depending on `ad.map.route.planRoute()` for the final shortest-path result.
- Filtered lane changes so the route can move only to adjacent lanes with the same travel direction.
- Used the lane direction to decide what “forward” means on each lane, so the route does not travel backward on negative-direction lanes.
- Snapped routing to lane-center positions so the final path follows the road centerline more like a waypoint-based planner.



## 6. Repository layout

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
│   └── run_map_test.py
└── temporary/
```

## 7. Main files

- `scripts/run_map_test.py`: main entry point for the routing demo
- `scripts/admap_route.py`: CARLA-independent AD-map route logic and custom lane-path search
- `scripts/carla_bridge.py`: reads object positions from CARLA and draws saved route points
- `scripts/carla_global_planner.py`: optional CARLA global planner comparison path
- `activate.sh`: activates the AD-map environment and loads the built `map_repo` install

## 8. Requirements

The code uses only the Python standard library plus these two runtime dependencies:

- `ad_map_access`
- `carla`

They are listed in [requirement.txt](requirement.txt).

Notes:

- `ad_map_access` comes from the built [carla-simulator/map](https://github.com/carla-simulator/map) repo.
- `carla` comes from the local CARLA Python API / egg in your CARLA installation.

## 9. Before you run

Make sure these items are ready first:

1. CARLA is installed and the server can run.
2. The town loaded in CARLA matches the OpenDRIVE map you want to use.
3. In your map two objects (cube) named `start_1` and `goal_1` are available.
4. The `map_repo` dependency is built, and `map_repo/install/setup.bash` exists.
5. The CARLA Python environment exists.

This project currently uses local hard-coded paths, so update them if your machine is different:

- `scripts/run_map_test.py`
  - `XODR_PATH`
  - `CARLA_PYTHON`
- `scripts/carla_bridge.py`
  - `CARLA_ROOT`

## 10. How to run this project

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
3. compute the route with the custom AD-map-based lane search
4. sample route points
5. optionally query adjacent lane points
6. draw the AD-map route in CARLA

## 11. Optional route comparison

If you want to compare the AD-map result against the CARLA global planner, enable the booleans in `scripts/run_map_test.py`:

```python
show_carla_route = True
show_ad_map_route = True
```

When enabled:

- AD-map route is drawn in yellow
- CARLA global planner route is drawn in blue

## 12. Example commands

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

## 13. Output

During a normal run, the project prints information such as:

- CARLA start and goal object positions
- number of parsed lanes
- matched start and goal lane ids
- snapped start and goal lane information
- route length
- number of sampled route points
- adjacent lane points for the selected route sample

Temporary JSON outputs are written into the `temporary/` folder.

![Project output](image/output_screenshot.png)



## 14. Important note about shortest-path routing

This project still uses `map_repo` for:

- loading the OpenDRIVE map
- lane matching
- lane geometry
- lane direction
- lane-to-lane connectivity
- adjacent lane lookup

But the final shortest-path search is custom in this repo. It does **not** rely on `ad.map.route.planRoute()` for the final path anymore.
