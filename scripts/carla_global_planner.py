#!/usr/bin/env python3

"""Run CARLA's global route planner as a separate optional path source."""
import argparse
import subprocess
import tempfile
from pathlib import Path

from carla_bridge import CARLA_ROOT, add_import_path, connect_to_carla, load_carla
from common import point_to_tuple, read_json, write_json

def add_agent_path():
    """Add the CARLA agent package folder so the global planner can be imported.

    input: none (`None`)
    output: none (`None`)
    """
    add_import_path(CARLA_ROOT / 'PythonAPI' / 'carla')

def to_location(carla, point):
    """Turn one plain point dictionary into a CARLA location.

    input: `carla` (`module`), `point` (`dict[str, float]`)
    output: location (`carla.Location`)
    """
    return carla.Location(x=float(point['x']), y=float(point['y']), z=float(point['z']))

def waypoint_to_dict(waypoint):
    """Turn one CARLA waypoint into a simple point dictionary.

    input: `waypoint` (`carla.Waypoint`)
    output: point data (`dict[str, float]`)
    """
    location = waypoint.transform.location
    return {'x': float(location.x), 'y': float(location.y), 'z': float(location.z)}

def drop_repeat_points(points, tolerance=0.05):
    """Remove consecutive route points that are basically the same.

    input: `points` (`list[dict[str, float]]`), `tolerance` (`float`)
    output: cleaned point list (`list[dict[str, float]]`)
    """
    cleaned_points = []
    for point in points:
        if not cleaned_points:
            cleaned_points.append(point)
            continue
        first_point = point_to_tuple(cleaned_points[-1])
        second_point = point_to_tuple(point)
        if ((first_point[0] - second_point[0]) ** 2 + (first_point[1] - second_point[1]) ** 2 + (first_point[2] - second_point[2]) ** 2) ** 0.5 > tolerance:
            cleaned_points.append(point)
    return cleaned_points

def load_route_map(carla, xodr_path=None, host='127.0.0.1', port=2000):
    """Load the planner map either from a local OpenDRIVE file or from CARLA.

    input: `carla` (`module`), `xodr_path` (`str | Path | None`), `host` (`str`), `port` (`int`)
    output: planner map (`carla.Map`)
    """
    if xodr_path is not None:
        xodr_path = Path(xodr_path)
        map_content = xodr_path.read_text(encoding='utf-8')
        return carla.Map(xodr_path.stem, map_content)

    _, world = connect_to_carla(carla=carla, host=host, port=port)
    try:
        return world.get_map()
    except RuntimeError as error:
        raise RuntimeError('CARLA could not provide its OpenDRIVE map. Pass a local xodr_path to use the planner with a repo map file.') from error

def build_route(start_point, goal_point, host='127.0.0.1', port=2000, spacing=1.0, xodr_path=None):
    """Run CARLA's global planner and return the sampled route points.

    input: `start_point` (`dict[str, float]`), `goal_point` (`dict[str, float]`), `host` (`str`), `port` (`int`), `spacing` (`float`), `xodr_path` (`str | Path | None`)
    output: route points (`list[dict[str, float]]`)
    """
    carla = load_carla()
    add_agent_path()
    from agents.navigation.global_route_planner import GlobalRoutePlanner

    planner_map = load_route_map(carla=carla, xodr_path=xodr_path, host=host, port=port)
    planner = GlobalRoutePlanner(planner_map, spacing)
    route_trace = planner.trace_route(to_location(carla, start_point), to_location(carla, goal_point))
    if not route_trace:
        raise RuntimeError('CARLA global route planner did not return a route.')
    route_points = [waypoint_to_dict(waypoint) for waypoint, _ in route_trace]
    route_points = drop_repeat_points(route_points)
    if not route_points:
        raise RuntimeError('CARLA global route planner returned only duplicate route points.')
    return route_points

def run_in_carla_python(start_point, goal_point, python_path, host='127.0.0.1', port=2000, spacing=1.0, xodr_path=None):
    """Run this file inside the CARLA Python environment and read the result back.

    input: `start_point` (`dict[str, float]`), `goal_point` (`dict[str, float]`), `python_path` (`str | Path`), `host` (`str`), `port` (`int`), `spacing` (`float`), `xodr_path` (`str | Path | None`)
    output: route points (`list[dict[str, float]]`)
    """
    with tempfile.TemporaryDirectory(prefix='carla_global_planner_') as temp_folder:
        temp_folder = Path(temp_folder)
        input_file = temp_folder / 'planner_input.json'
        output_file = temp_folder / 'planner_output.json'
        write_json(input_file, {'start': start_point, 'goal': goal_point})
        command = [str(python_path), str(Path(__file__).resolve()), '--input', str(input_file), '--output', str(output_file), '--host', str(host), '--port', str(port), '--spacing', str(spacing)]
        if xodr_path is not None:
            command.extend(['--xodr-path', str(xodr_path)])
        subprocess.run(command, check=True)
        return read_json(output_file)['points']

def find_carla_shortest_path(start_location, goal_location, host='127.0.0.1', port=2000, sampling_resolution=1.0, python_executable=None, xodr_path=None):
    """Return the CARLA global-planner route for the given start and goal.

    input: `start_location` (`dict[str, float]`), `goal_location` (`dict[str, float]`), `host` (`str`), `port` (`int`), `sampling_resolution` (`float`), `python_executable` (`str | Path | None`), `xodr_path` (`str | Path | None`)
    output: route points (`list[dict[str, float]]`)
    """
    if python_executable is not None:
        return run_in_carla_python(start_point=start_location, goal_point=goal_location, python_path=python_executable, host=host, port=port, spacing=sampling_resolution, xodr_path=xodr_path)
    return build_route(start_point=start_location, goal_point=goal_location, host=host, port=port, spacing=sampling_resolution, xodr_path=xodr_path)

def make_parser():
    """Build the command-line parser used by the CARLA subprocess wrapper.

    input: none (`None`)
    output: parser (`argparse.ArgumentParser`)
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--spacing', type=float, default=1.0)
    parser.add_argument('--xodr-path')
    return parser

def main():
    """Run the planner wrapper from the command line.

    input: none (`None`)
    output: none (`None`)
    """
    parser = make_parser()
    args = parser.parse_args()
    request = read_json(args.input)
    route_points = build_route(start_point=request['start'], goal_point=request['goal'], host=args.host, port=args.port, spacing=args.spacing, xodr_path=args.xodr_path)
    write_json(args.output, {'points': route_points})

if __name__ == '__main__':
    main()
