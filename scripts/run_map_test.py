#!/usr/bin/env python3

"""Run the demo that compares raw CARLA object positions with AD-map routing."""
import subprocess
from pathlib import Path

from admap_route import close_map, create_enu_point, describe_enu_position, enu_point_to_tuple, find_adjacent_points, find_shortest_route, load_xodr_map, sample_route_centerline
from common import clean_number, get_path_length, point_to_tuple, read_json, write_json

PROJECT_ROOT = Path(__file__).resolve().parents[1]
XODR_PATH = PROJECT_ROOT / 'maps' / 'Town10HD_Opt.xodr'
CARLA_BRIDGE = PROJECT_ROOT / 'scripts' / 'carla_bridge.py'
CARLA_PYTHON = Path('/home/umd-user/miniconda3/envs/carla_env/bin/python')
TEMP_DIRECTORY = PROJECT_ROOT / 'temporary'
LANDMARKS_FILE = TEMP_DIRECTORY / 'carla_landmarks.json'
ADMAP_ROUTE_FILE = TEMP_DIRECTORY / 'admap_route_points.json'
CARLA_ROUTE_FILE = TEMP_DIRECTORY / 'carla_global_route_points.json'
ADMAP_SNAP_POINTS_FILE = TEMP_DIRECTORY / 'admap_snap_points.json'

SEARCH_RADIUS_M = 8.0
ROUTE_SAMPLING_RESOLUTION_M = 3.0
ROUTE_DRAW_POINT_SIZE = 0.24
ADMAP_ROUTE_DRAW_Z_OFFSET_M = 0.8
CARLA_ROUTE_DRAW_Z_OFFSET_M = 0.55
SNAP_DRAW_POINT_SIZE = 0.60
SNAP_DRAW_Z_OFFSET_M = 1.05
SNAP_DRAW_LIFE_TIME_S = 600.0
ROUTE_DRAW_LIFE_TIME_S = 600.0
ROUTE_DRAW_LINE_THICKNESS = 0.14
ROUTE_DRAW_FLUSH_INTERVAL = 100
ROUTE_DRAW_SETTLE_TIME_S = 0.5
ADJACENT_ROUTE_POINT_INDEX = 5
CARLA_TO_ENU_Y_SIGN = -1.0
LANE_CHANGE_PENALTY = 2
GOAL_APPEND_DISTANCE_THRESHOLD_M = 1.0

show_carla_route = True
show_ad_map_route = True

def run_bridge(arguments, quiet=True):
    """Run the small CARLA bridge script with the given command line args.

    input: `arguments` (`list[str]`), `quiet` (`bool`)
    output: none (`None`)
    """
    command = [str(CARLA_PYTHON), str(CARLA_BRIDGE)] + arguments
    try:
        subprocess.run(command, check=True, capture_output=quiet, text=True)
    except subprocess.CalledProcessError as error:
        message_lines = ['CARLA bridge failed.']
        if error.stdout:
            message_lines.append(error.stdout.strip())
        if error.stderr:
            message_lines.append(error.stderr.strip())
        raise RuntimeError('\n'.join((line for line in message_lines if line))) from error

def get_start_and_goal():
    """Read the raw `start_1` and `goal_1` object positions from CARLA.

    input: none (`None`)
    output: start and goal data (`dict[str, dict[str, float | str]]`)
    """
    
    #print('===========================get stat and goal ====================================')
    run_bridge(['get-objects', '--start-name', 'start_1', '--goal-name', 'goal_1', '--output', str(LANDMARKS_FILE)])
    print('='*10)
    print(LANDMARKS_FILE)
    print('='*10)
    return read_json(LANDMARKS_FILE)

def to_enu_point(location):
    """Convert one CARLA position into an AD-map ENU point.

    input: `location` (`dict[str, float]`) with `x`, `y`, `z`
    output: ENU point (`ad.map.point.ENUPoint`)
    """
    loc = tuple((location['x'], CARLA_TO_ENU_Y_SIGN * location['y'], location['z']))

    #return create_enu_point(x=location['x'], y=CARLA_TO_ENU_Y_SIGN * location['y'], z=location['z'])
    return create_enu_point(loc)

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

def draw_route(path, color='yellow', z_offset=ADMAP_ROUTE_DRAW_Z_OFFSET_M, extra_color=None, point_size=ROUTE_DRAW_POINT_SIZE, life_time=ROUTE_DRAW_LIFE_TIME_S):
    """Ask the CARLA bridge to draw one saved route file.

    input: `path` (`str | Path`), `color` (`str`), `z_offset` (`float`), `extra_color` (`str | None`), `point_size` (`float`), `life_time` (`float`)
    output: none (`None`)
    """
    arguments = ['draw-route', '--input', str(path), '--point-size', str(point_size), '--z-offset', str(z_offset), '--life-time', str(life_time), '--line-thickness', str(ROUTE_DRAW_LINE_THICKNESS), '--flush-interval', str(ROUTE_DRAW_FLUSH_INTERVAL), '--settle-time', str(ROUTE_DRAW_SETTLE_TIME_S), '--color', str(color)]
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
        return find_carla_shortest_path(
            start_location=start_location,
            goal_location=goal_location,
            sampling_resolution=ROUTE_SAMPLING_RESOLUTION_M,
            python_executable=CARLA_PYTHON,
            xodr_path=XODR_PATH,
        )
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

def format_position(point):
    """Format one CARLA or AD-map point position for compact logging.

    input: `point` (`dict | tuple`)
    output: formatted position (`str`)
    """
    x, y, z = (clean_number(value) for value in point_to_tuple(point))
    return f'({x:.3f}, {y:.3f}, {z:.3f})'

def format_point_summary(point):
    """Format one point with position and lane identifiers.

    input: `point` (`dict`)
    output: summary text (`str`)
    """
    return f'pos={format_position(point)}, road_id={point.get("road_id")}, lane_id={point.get("lane_id")}'

def print_route_debug_summary(start_carla, start_ad_map, goal_carla, goal_ad_map, carla_route_length_m, ad_map_route_length_m):
    """Print the route debug summary in the requested order.

    input: summary points (`dict`), route lengths (`float | None`)
    output: none (`None`)
    """
    carla_length_text = 'unavailable'
    if carla_route_length_m is not None:
        carla_length_text = f'{carla_route_length_m:.2f} m'
    print(f'start: carla: {format_point_summary(start_carla)}, ad_map: {format_point_summary(start_ad_map)}')
    print(f'goal: carla: {format_point_summary(goal_carla)}, ad_map: {format_point_summary(goal_ad_map)}')
    print(f'route_length: carla: {carla_length_text}, ad_map: {ad_map_route_length_m:.2f} m')

def main():
    """Run the full test flow from CARLA objects to route drawing.

    input: none (`None`)
    output: none (`None`)
    """
    TEMP_DIRECTORY.mkdir(parents=True, exist_ok=True)

    landmarks = get_start_and_goal()
    start_location = landmarks['start']
    goal_location = landmarks['goal']

    carla_route_length_m = None
    if show_carla_route:
        carla_route = get_carla_route(start_location, goal_location)
        if carla_route:
            save_route(CARLA_ROUTE_FILE, carla_route)
            draw_route(CARLA_ROUTE_FILE, color='blue', z_offset=CARLA_ROUTE_DRAW_Z_OFFSET_M)
            carla_route_length_m = get_path_length(carla_route)
            print('CARLA route points:', len(carla_route))

    print('\nLoading OpenDRIVE map...')
    print('Total parsed lanes:', len(load_xodr_map(XODR_PATH)))

    start_point = to_enu_point(start_location)
    goal_point = to_enu_point(goal_location)

    try:
        route, route_length_m, snapped_start_point, snapped_goal_point = find_shortest_route(
            start_point=start_point,
            goal_point=goal_point,
            search_radius=SEARCH_RADIUS_M,
            lane_change_penalty_m=LANE_CHANGE_PENALTY,
        )
        
        route_points = sample_route_centerline(
            route,
            spacing=ROUTE_SAMPLING_RESOLUTION_M,
            start_point=snapped_start_point,
            goal_point=snapped_goal_point,
            goal_append_distance_threshold_m=GOAL_APPEND_DISTANCE_THRESHOLD_M,
        )
        start_ad_map_info = describe_enu_position(snapped_start_point, search_radius=SEARCH_RADIUS_M)
        goal_ad_map_info = describe_enu_position(snapped_goal_point, search_radius=SEARCH_RADIUS_M)
        ad_map_route_length_m = get_path_length(route_points)
        print_route_debug_summary(start_location, start_ad_map_info, goal_location, goal_ad_map_info, carla_route_length_m, ad_map_route_length_m)
        side_point_data = find_adjacent_points(route, route_points, route_point_index=ADJACENT_ROUTE_POINT_INDEX)
        side_points = side_point_data['adjacent_points']
        
       
        
        save_route(ADMAP_ROUTE_FILE, to_carla_points(route_points), extra_points=to_carla_points(side_points))
        save_route(ADMAP_SNAP_POINTS_FILE, [], extra_points=to_carla_points([enu_point_to_tuple(snapped_start_point), enu_point_to_tuple(snapped_goal_point)]))
        
        if show_ad_map_route:
            #this project requires carla to draw
            draw_route(ADMAP_ROUTE_FILE, color='yellow', z_offset=ADMAP_ROUTE_DRAW_Z_OFFSET_M, extra_color='yellow')
            draw_route(ADMAP_SNAP_POINTS_FILE, color='green', z_offset=SNAP_DRAW_Z_OFFSET_M, extra_color='green', point_size=SNAP_DRAW_POINT_SIZE, life_time=SNAP_DRAW_LIFE_TIME_S)
            
            print('AD-map route points:', len(route_points))
            

        print('\nRoute completed')
        print('AD-map analytic route length:', f'{route_length_m:.2f} m')
        print_map_point('First path point', describe_enu_position(route_points[0], search_radius=SEARCH_RADIUS_M))
        print_map_point('Last path point', describe_enu_position(route_points[-1], search_radius=SEARCH_RADIUS_M))
    finally:
        close_map()

if __name__ == '__main__':
    main()
