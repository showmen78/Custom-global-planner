#!/usr/bin/env python3

"""Run the demo that compares raw CARLA object positions with AD-map routing."""
import subprocess
from pathlib import Path

from admap_route import close_map, create_enu_point, describe_enu_position, find_adjacent_points, find_shortest_route, get_match_lane_id, load_xodr_map, sample_route_centerline
from common import clean_number, get_path_length, point_to_tuple, read_json, write_json

PROJECT_ROOT = Path(__file__).resolve().parents[1]
XODR_PATH = PROJECT_ROOT / 'maps' / 'Town05.xodr'
CARLA_BRIDGE = PROJECT_ROOT / 'scripts' / 'carla_bridge.py'
CARLA_PYTHON = Path('/home/umd-user/miniconda3/envs/carla_env/bin/python')
TEMP_DIRECTORY = PROJECT_ROOT / 'temporary'
LANDMARKS_FILE = TEMP_DIRECTORY / 'carla_landmarks.json'
ADMAP_ROUTE_FILE = TEMP_DIRECTORY / 'admap_route_points.json'
CARLA_ROUTE_FILE = TEMP_DIRECTORY / 'carla_global_route_points.json'

SEARCH_RADIUS_M = 8.0
ROUTE_POINT_SPACING_M = 1.0
ROUTE_DRAW_POINT_SIZE = 0.24
ADMAP_ROUTE_DRAW_Z_OFFSET_M = 0.8
CARLA_ROUTE_DRAW_Z_OFFSET_M = 0.55
ROUTE_DRAW_LIFE_TIME_S = 600.0
ROUTE_DRAW_LINE_THICKNESS = 0.14
ROUTE_DRAW_FLUSH_INTERVAL = 100
ROUTE_DRAW_SETTLE_TIME_S = 0.5
ADJACENT_ROUTE_POINT_INDEX = 5
CARLA_TO_ENU_Y_SIGN = -1.0

show_carla_route = False
show_ad_map_route = True

def run_bridge(arguments):
    """Run the small CARLA bridge script with the given command line args.

    input: `arguments` (`list[str]`)
    output: none (`None`)
    """
    command = [str(CARLA_PYTHON), str(CARLA_BRIDGE)] + arguments
    subprocess.run(command, check=True)

def get_start_and_goal():
    """Read the raw `start_1` and `goal_1` object positions from CARLA.

    input: none (`None`)
    output: start and goal data (`dict[str, dict[str, float | str]]`)
    """
    run_bridge(['get-objects', '--start-name', 'start_1', '--goal-name', 'goal_1', '--output', str(LANDMARKS_FILE)])
    return read_json(LANDMARKS_FILE)

def to_enu_point(location):
    """Convert one CARLA position into an AD-map ENU point.

    input: `location` (`dict[str, float]`) with `x`, `y`, `z`
    output: ENU point (`ad.map.point.ENUPoint`)
    """
    return create_enu_point(x=location['x'], y=CARLA_TO_ENU_Y_SIGN * location['y'], z=location['z'])

def to_carla_point(point):
    """Convert one ENU point or point-like value back into CARLA coordinates.

    input: `point` (`tuple[float, float, float] | dict`)
    output: CARLA point (`dict[str, float]`)
    """
    x, y, z = point_to_tuple(point)
    return {'x': float(x), 'y': float(CARLA_TO_ENU_Y_SIGN * y), 'z': float(z)}

def to_carla_points(points):
    """Convert a whole list of points into CARLA point dictionaries.

    input: `points` (`list[tuple | dict]`)
    output: converted points (`list[dict[str, float]]`)
    """
    return [to_carla_point(point) for point in points]

def save_route(path, route_points, extra_points=None):
    """Save a route file that the CARLA bridge can draw.

    input: `path` (`str | Path`), `route_points` (`list[dict]`), `extra_points` (`list[dict] | None`)
    output: none (`None`)
    """
    write_json(path, {'points': route_points, 'highlight_points': extra_points or []})

def draw_route(path, color='yellow', z_offset=ADMAP_ROUTE_DRAW_Z_OFFSET_M, extra_color=None):
    """Ask the CARLA bridge to draw one saved route file.

    input: `path` (`str | Path`), `color` (`str`), `z_offset` (`float`), `extra_color` (`str | None`)
    output: none (`None`)
    """
    arguments = ['draw-route', '--input', str(path), '--point-size', str(ROUTE_DRAW_POINT_SIZE), '--z-offset', str(z_offset), '--life-time', str(ROUTE_DRAW_LIFE_TIME_S), '--line-thickness', str(ROUTE_DRAW_LINE_THICKNESS), '--flush-interval', str(ROUTE_DRAW_FLUSH_INTERVAL), '--settle-time', str(ROUTE_DRAW_SETTLE_TIME_S), '--color', str(color)]
    if extra_color is not None:
        arguments.extend(['--highlight-color', str(extra_color)])
    run_bridge(arguments)

def get_carla_route(start_location, goal_location):
    """Get the optional CARLA global-planner route for the same start and goal.

    input: `start_location` (`dict[str, float]`), `goal_location` (`dict[str, float]`)
    output: route points or no result (`list[dict[str, float]] | None`)
    """
    try:
        from carla_global_planner import find_carla_shortest_path
    except Exception as error:
        print(f'\nSkipping CARLA global route: {error}')
        return None
    try:
        return find_carla_shortest_path(start_location=start_location, goal_location=goal_location, sampling_resolution=ROUTE_POINT_SPACING_M, python_executable=CARLA_PYTHON)
    except Exception as error:
        print(f'\nSkipping CARLA global route: {error}')
        return None

def print_map_point(label, point_info):
    """Print one AD-map point in a short readable line.

    input: `label` (`str`), `point_info` (`dict`)
    output: none (`None`)
    """
    x, y, z = (clean_number(value) for value in point_info['position'])
    print(f"{label} [ENU]: position=({x:.3f}, {y:.3f}, {z:.3f}) road_id={point_info['road_id']} lane_id={point_info['lane_id']}")

def print_carla_point(label, point):
    """Print one CARLA point in a short readable line.

    input: `label` (`str`), `point` (`dict`)
    output: none (`None`)
    """
    x = clean_number(point['x'])
    y = clean_number(point['y'])
    z = clean_number(point['z'])
  
    message = f'{label} [CARLA]: position=({x:.3f}, {y:.3f}, {z:.3f})'
    road_id = point.get('road_id')
    lane_id = point.get('lane_id')
    if road_id is not None and lane_id is not None:
        message += f' road_id={road_id} lane_id={lane_id}'
    print(message)

def main():
    """Run the full test flow from CARLA objects to route drawing.

    input: none (`None`)
    output: none (`None`)
    """
    TEMP_DIRECTORY.mkdir(parents=True, exist_ok=True)

    landmarks = get_start_and_goal()
    start_location = landmarks['start']
    goal_location = landmarks['goal']

    print_carla_point('Loaded start object', start_location)
    print_carla_point('Loaded goal object', goal_location)

    if show_carla_route:
        carla_route = get_carla_route(start_location, goal_location)
        if carla_route:
            save_route(CARLA_ROUTE_FILE, carla_route)
            draw_route(CARLA_ROUTE_FILE, color='blue', z_offset=CARLA_ROUTE_DRAW_Z_OFFSET_M)
            print('CARLA route points:', len(carla_route))
            print('CARLA route length:', f'{get_path_length(carla_route):.2f} m')

    print('\nLoading OpenDRIVE map...')
    lanes = load_xodr_map(XODR_PATH)
    print('Total parsed lanes:', len(lanes))

    start_point = to_enu_point(start_location)
    goal_point = to_enu_point(goal_location)

    try:
        route, start_match, goal_match, route_length_m = find_shortest_route(start_point=start_point, goal_point=goal_point, search_radius=SEARCH_RADIUS_M)
        route_points = sample_route_centerline(route, start_match=start_match, goal_match=goal_match, spacing=ROUTE_POINT_SPACING_M, start_point=start_point, goal_point=goal_point)
        side_point_data = find_adjacent_points(route, start_match, goal_match, route_points, route_point_index=ADJACENT_ROUTE_POINT_INDEX, spacing=ROUTE_POINT_SPACING_M)
        side_points = side_point_data['adjacent_points']

        save_route(ADMAP_ROUTE_FILE, to_carla_points(route_points), extra_points=to_carla_points(side_points))

        if show_ad_map_route:
            draw_route(ADMAP_ROUTE_FILE, color='yellow', z_offset=ADMAP_ROUTE_DRAW_Z_OFFSET_M, extra_color='yellow')
            print('AD-map route points:', len(route_points))
            print('AD-map route length:', f'{get_path_length(route_points):.2f} m')

        print('\nRoute completed')
        print('Matched start lane:', get_match_lane_id(start_match))
        print('Matched goal lane:', get_match_lane_id(goal_match))
        print('Route length:', f'{route_length_m:.2f} m')
        print('Yellow points:', len(route_points))
        print('Adjacent route index used:', side_point_data['selected_route_index'])
        print('Adjacent lane points:', len(side_points))
        print_map_point('First path point', describe_enu_position(route_points[0], search_radius=SEARCH_RADIUS_M))
        print_map_point('Last path point', describe_enu_position(route_points[-1], search_radius=SEARCH_RADIUS_M))
        for point_number, side_point in enumerate(side_points, start=1):
            print_map_point(f'Adjacent lane point {point_number}', side_point)
    finally:
        close_map()

if __name__ == '__main__':
    main()
