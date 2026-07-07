#!/usr/bin/env python3

"""Read simple object positions from CARLA and draw saved route files back into the world."""
import argparse
import re
import sys
import time
from pathlib import Path

from common import clean_number, read_json, write_json

CARLA_ROOT = Path('/home/umd-user/carla_source/carla')

def add_import_path(path):
    """Add one folder or egg path to Python imports if it is not there yet.

    input: `path` (`str | Path`)
    output: none (`None`)
    """
    path = str(path)
    if path not in sys.path:
        sys.path.insert(0, path)

def load_carla():
    """Load the local CARLA Python API from the configured install folder.

    input: none (`None`)
    output: imported CARLA module (`module`)
    """
    python_tag = f'py{sys.version_info.major}.{sys.version_info.minor}'
    dist_folder = CARLA_ROOT / 'PythonAPI' / 'carla' / 'dist'
    eggs = list(dist_folder.glob(f'carla-*{python_tag}*.egg'))
    if not eggs:
        raise RuntimeError(f'No CARLA egg found for {python_tag}')
    add_import_path(eggs[0])
    import carla
    return carla

def connect_to_carla(carla, host='127.0.0.1', port=2000, timeout=20.0):
    """Connect to the running CARLA server and return the world handle.

    input: `carla` (`module`), `host` (`str`), `port` (`int`), `timeout` (`float`)
    output: client and world (`tuple[carla.Client, carla.World]`)
    """
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    return (client, world)

def clean_name(name):
    """Make CARLA object names easier to match.

    input: `name` (`str | None`)
    output: cleaned name (`str`)
    """
    if not name:
        return ''
    return re.sub('_sm_\\d+$', '', str(name).strip().casefold())

def get_actor_names(actor):
    """Collect the possible lookup names for one CARLA actor.

    input: `actor` (`carla.Actor`)
    output: possible names (`set[str]`)
    """
    names = set()
    attributes = dict(getattr(actor, 'attributes', {}))
    for key in ('id', 'role_name', 'name'):
        value = clean_name(attributes.get(key))
        if value:
            names.add(value)
    type_id = clean_name(getattr(actor, 'type_id', ''))
    if type_id:
        names.add(type_id)
    return names

def find_actor(world, wanted_name):
    """Find one actor by name when the environment-object lookup is not enough.

    input: `world` (`carla.World`), `wanted_name` (`str`)
    output: matching actor or nothing (`carla.Actor | None`)
    """
    matches = []
    for actor in world.get_actors():
        if wanted_name in get_actor_names(actor):
            matches.append(actor)
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise RuntimeError(f"Expected one actor named '{wanted_name}', found {len(matches)}: {[actor.id for actor in matches]}")
    return None

def find_object(world, wanted_name):
    """Find one named object in the CARLA world.

    input: `world` (`carla.World`), `wanted_name` (`str`)
    output: one object handle (`carla.EnvironmentObject | carla.Actor`)
    """
    wanted_name = clean_name(wanted_name)
    objects = list(world.get_environment_objects())
    exact_matches = [obj for obj in objects if clean_name(obj.name) == wanted_name]
   
    if len(exact_matches) == 1:
        return exact_matches[0]
    if len(exact_matches) > 1:
        raise RuntimeError(f"Expected one environment object named '{wanted_name}', found {len(exact_matches)}: {[obj.name for obj in exact_matches]}")
  
    prefix_matches = [obj for obj in objects if clean_name(obj.name).startswith(wanted_name)]
  
    if len(prefix_matches) == 1:
        return prefix_matches[0]
 
    actor = find_actor(world, wanted_name)
    if actor is not None:
        return actor
    raise RuntimeError(f"Could not find any object matching '{wanted_name}'.")

def get_location(item):
    """Read the location of one CARLA object or actor.

    input: `item` (`carla.EnvironmentObject | carla.Actor`)
    output: location (`carla.Location`)
    """
    if hasattr(item, 'get_transform'):
        return item.get_transform().location
    return item.transform.location

def get_name(item):
    """Read the best available name for one CARLA object or actor.

    input: `item` (`carla.EnvironmentObject | carla.Actor`)
    output: readable name (`str`)
    """
    if hasattr(item, 'get_transform'):
        attributes = dict(getattr(item, 'attributes', {}))
        return attributes.get('role_name') or attributes.get('name') or getattr(item, 'type_id', str(item.id))
    return getattr(item, 'name', str(item.id))

def object_to_dict(item, world_map=None):
    """Turn one CARLA object into a plain position dictionary.

    input: `item` (`carla.EnvironmentObject | carla.Actor`), `world_map` (`carla.Map | None`)
    output: object data (`dict[str, str | float]`)
    """
    location = get_location(item)
    point = {'name': str(get_name(item)), 'x': float(location.x), 'y': float(location.y), 'z': float(location.z)}
    if world_map is not None:
        waypoint = world_map.get_waypoint(location, project_to_road=True)
        if waypoint is not None:
            point['road_id'] = int(waypoint.road_id)
            point['lane_id'] = int(waypoint.lane_id)
    return point

def get_start_and_goal(world, start_name, goal_name):
    """Read the raw positions of the chosen start and goal objects.

    input: `world` (`carla.World`), `start_name` (`str`), `goal_name` (`str`)
    output: start and goal data (`dict[str, dict[str, float | str]]`)
    """
    world_map = world.get_map()
    return {
        'start': object_to_dict(find_object(world, start_name), world_map=world_map),
        'goal': object_to_dict(find_object(world, goal_name), world_map=world_map),
    }

def get_color(carla, color_name):
    """Map a simple color name to a CARLA debug color.

    input: `carla` (`module`), `color_name` (`str`)
    output: debug color (`carla.Color`)
    """
    colors = {'yellow': carla.Color(r=255, g=255, b=0), 'red': carla.Color(r=255, g=0, b=0), 'blue': carla.Color(r=0, g=120, b=255), 'green': carla.Color(r=0, g=255, b=0)}
    try:
        return colors[color_name]
    except KeyError as error:
        raise ValueError(f'Unsupported debug color: {color_name}') from error

def draw_route(carla, world, points, point_size=0.24, z_offset=0.8, life_time=600.0, line_thickness=0.14, color='yellow', flush_interval=100, settle_time=0.5):
    """Draw one route as dots plus short lines in the CARLA world.

    input: `carla` (`module`), `world` (`carla.World`), `points` (`list[dict]`), drawing settings (`float | int | str`)
    output: number of route points drawn (`int`)
    """
    route_color = get_color(carla, color)
    locations = [carla.Location(x=float(point['x']), y=float(point['y']), z=float(point['z']) + z_offset) for point in points]
    draw_calls = 0
  
    for index, location in enumerate(locations):
        world.debug.draw_point(location, size=point_size, color=route_color, life_time=life_time, persistent_lines=True)
        draw_calls += 1
      
        if index + 1 < len(locations):
            world.debug.draw_line(location, locations[index + 1], thickness=line_thickness, color=route_color, life_time=life_time, persistent_lines=True)
            draw_calls += 1
     
        if flush_interval > 0 and draw_calls % flush_interval == 0:
            time.sleep(0.02)
    if settle_time > 0.0:
        time.sleep(settle_time)
    return len(points)

def draw_points(carla, world, points, color, point_size=0.18, z_offset=0.35, life_time=0.0):
    """Draw stand-alone helper dots in the CARLA world.

    input: `carla` (`module`), `world` (`carla.World`), `points` (`list[dict]`), `color` (`carla.Color`), drawing settings (`float`)
    output: none (`None`)
    """
    for point in points:
        location = carla.Location(x=float(point['x']), y=float(point['y']), z=float(point['z']) + z_offset)
        world.debug.draw_point(location, size=point_size, color=color, life_time=life_time, persistent_lines=True)

def print_carla_point(label, point):
    """Print one CARLA point in a clean one-line format.

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

def handle_get_objects(args):
    """Handle the CLI command that reads start and goal objects.

    input: `args` (`argparse.Namespace`)
    output: none (`None`)
    """
    carla = load_carla()
    _, world = connect_to_carla(carla, host=args.host, port=args.port)
  
    landmarks = get_start_and_goal(world, args.start_name, args.goal_name)
    draw_points(carla=carla, world=world, points=[landmarks['start'], landmarks['goal']], color=carla.Color(r=255, g=0, b=0))
    write_json(args.output, landmarks)
    print('CARLA map:', world.get_map().name)
    print_carla_point('Start object', landmarks['start'])
    print_carla_point('Goal object', landmarks['goal'])

def handle_draw_route(args):
    """Handle the CLI command that draws one saved route file.

    input: `args` (`argparse.Namespace`)
    output: none (`None`)
    """
    carla = load_carla()
    _, world = connect_to_carla(carla, host=args.host, port=args.port)
    route_data = read_json(args.input)
    route_color = args.color
    extra_color = args.highlight_color or route_color
    point_count = draw_route(carla=carla, world=world, points=route_data['points'], point_size=args.point_size, z_offset=args.z_offset, life_time=args.life_time, line_thickness=args.line_thickness, color=route_color, flush_interval=args.flush_interval, settle_time=args.settle_time)
    extra_points = route_data.get('highlight_points', [])
   
    if extra_points:
        draw_points(carla=carla, world=world, points=extra_points, color=get_color(carla, extra_color), point_size=args.point_size, z_offset=args.z_offset, life_time=args.life_time)
    print(f'{route_color.title()} route points drawn:', point_count)
    if extra_points:
        print(f'Adjacent {extra_color} dots drawn:', len(extra_points))

def make_parser():
    """Build the command-line parser for this bridge script.

    input: none (`None`)
    output: parser (`argparse.ArgumentParser`)
    """
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)

    get_parser = subparsers.add_parser('get-objects')
    get_parser.add_argument('--start-name', default='start_1')
    get_parser.add_argument('--goal-name', default='goal_1')
    get_parser.add_argument('--output', required=True)
    get_parser.add_argument('--host', default='127.0.0.1')
    get_parser.add_argument('--port', type=int, default=2000)
    get_parser.set_defaults(handler=handle_get_objects)

    draw_parser = subparsers.add_parser('draw-route')
    draw_parser.add_argument('--input', required=True)
    draw_parser.add_argument('--host', default='127.0.0.1')
    draw_parser.add_argument('--port', type=int, default=2000)
    draw_parser.add_argument('--point-size', type=float, default=0.24)
    draw_parser.add_argument('--z-offset', type=float, default=0.8)
    draw_parser.add_argument('--life-time', type=float, default=600.0)
    draw_parser.add_argument('--line-thickness', type=float, default=0.14)
    draw_parser.add_argument('--flush-interval', type=int, default=100)
    draw_parser.add_argument('--settle-time', type=float, default=0.5)
    draw_parser.add_argument('--color', default='yellow')
    draw_parser.add_argument('--highlight-color')
    draw_parser.set_defaults(handler=handle_draw_route)
    return parser

def main():
    """Parse the command line and run the requested bridge command.

    input: none (`None`)
    output: none (`None`)
    """
    parser = make_parser()
    args = parser.parse_args()
    args.handler(args)

if __name__ == '__main__':
    main()
