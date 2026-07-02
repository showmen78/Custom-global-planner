#!/usr/bin/env python3

"""Small shared helpers used by the scripts in this project."""
import json
import math
from pathlib import Path

def read_json(path):
    """Read one JSON file from disk.

    input: `path` (`str | Path`) to a JSON file
    output: parsed JSON data (`dict | list`)
    """
    with Path(path).open('r', encoding='utf-8') as file:
        return json.load(file)

def write_json(path, data):
    """Write one JSON file and create the parent folder if needed.

    input: `path` (`str | Path`), `data` (`dict | list`)
    output: none (`None`)
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as file:
        json.dump(data, file, indent=2)

def clean_number(value, tolerance=0.0005):
    """Round tiny floating-point noise down to zero before printing.

    input: `value` (`float | int`), `tolerance` (`float`)
    output: cleaned number (`float`)
    """
    value = float(value)
    if abs(value) < tolerance:
        return 0.0
    return value

def point_to_tuple(point):
    """Turn one point into a plain `(x, y, z)` tuple.

    input: `point` (`tuple[float, float, float] | list[float] | dict`)
    output: point as a tuple (`tuple[float, float, float]`)
    """
    if isinstance(point, dict):
        if 'position' in point:
            return point_to_tuple(point['position'])
        return (float(point['x']), float(point['y']), float(point['z']))
    return (float(point[0]), float(point[1]), float(point[2]))

def get_path_length(points):
    """Add up the full length of a point-by-point path.

    input: `points` (`list[tuple | dict]`)
    output: path length in meters (`float`)
    """
    if len(points) < 2:
        return 0.0
    total_length = 0.0
    for first_point, second_point in zip(points, points[1:]):
        total_length += math.dist(point_to_tuple(first_point), point_to_tuple(second_point))
    return total_length
