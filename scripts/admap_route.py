"""Provide the CARLA-independent routing.

This module works on top of `ad_map_access` and the OpenDRIVE file in
`maps/`. It loads the map, matches raw positions onto drivable lanes, finds a
legal route, chooses a continuous lane path, samples centerline points, and
can also query adjacent parallel lane points. It does not import CARLA or
talk to a CARLA server.
"""
from pathlib import Path
import math
import ad_map_access as ad

def load_xodr_map(xodr_path, overlap_margin=0.05):
    """Load the OpenDRIVE file and hand it to AD-map.

    input: `xodr_path` (`str | Path`), `overlap_margin` (`float`)
    output: parsed lane list (`list`)
    """
    xodr_path = Path(xodr_path)
    if not xodr_path.exists():
        raise FileNotFoundError(xodr_path)
    map_content = xodr_path.read_text(encoding='utf-8')
    loaded = ad.map.access.initFromOpenDriveContent(map_content, overlap_margin)
  
    if not loaded:
        raise RuntimeError(f'Failed to load map: {xodr_path}')
    return list(ad.map.lane.getLanes())

def create_enu_point(x, y, z=0.0):
    """Build one AD-map point from plain x, y, z values.

    input: `x` (`float`), `y` (`float`), `z` (`float`)
    output: ENU point (`ad.map.point.ENUPoint`)
    """
    point = ad.map.point.ENUPoint()
    point.x = ad.map.point.ENUCoordinate(float(x))
    point.y = ad.map.point.ENUCoordinate(float(y))
    point.z = ad.map.point.ENUCoordinate(float(z))
    return point

def to_base_value(value):
    """Turn one AD-map wrapped value into a normal Python value.

    input: `value` (`object`) which can be a plain Python value or an AD-map wrapper
    output: unwrapped base value (`object`)
    """
    base_value = getattr(value, 'toBaseType', value)
   
    if callable(base_value):
        base_value = base_value()
    return base_value

def distance_to_float(distance):
    """Convert one AD-map distance value into a normal Python float.

    input: `distance` (AD-map distance-like object)
    output: distance in meters (`float`)
    """
    return float(to_base_value(distance))

def coordinate_to_float(coordinate):
    """Convert one AD-map coordinate into a normal Python float.

    input: `coordinate` (AD-map coordinate-like object)
    output: coordinate value (`float`)
    """
    return float(to_base_value(coordinate))

def lane_id_to_int(lane_id):
    """Convert one AD-map lane id into a normal Python integer.

    input: `lane_id` (AD-map lane-id-like value)
    output: lane id (`int`)
    """
    return int(to_base_value(lane_id))

def route_length(route):
    """Get the full length of one AD-map route.

    input: `route` (AD-map route object)
    output: route length in meters (`float`)
    """
    length = ad.map.route.calcLength(route)
    return distance_to_float(length)

def get_map_matches(enu_point, search_radius=8.0):
    """Find nearby drivable lane matches for one raw map position.

    input: `enu_point` (`ad.map.point.ENUPoint`), `search_radius` (`float`)
    output: nearby routable matches (`list`)
    """
    matcher = ad.map.match.AdMapMatching()
    matches = matcher.getMapMatchedPositions(enu_point, ad.physics.Distance(search_radius), ad.physics.Probability(0.0))
    return [match for match in matches if is_routable_match(match)]

def is_routable_match(map_match):
    """Check if one matched lane can actually be used for routing.

    input: `map_match` (AD-map match object)
    output: whether the lane is routable (`bool`)
    """
    lane_id = map_match.lane_point.para_point.lane_id
    lane = ad.map.lane.getLane(lane_id)
    return ad.map.lane.isRouteable(lane)

def point_distance(first_point, second_point):
    """Measure the straight-line distance between two ENU points.

    input: `first_point` (`ad.map.point.ENUPoint`), `second_point` (`ad.map.point.ENUPoint`)
    output: distance in meters (`float`)
    """
    first_coordinates = enu_point_to_tuple(first_point)
    second_coordinates = enu_point_to_tuple(second_point)
    return math.dist(first_coordinates, second_coordinates)

def get_nominal_routing_directions(lane_id):
    """Get the legal travel directions for one lane.

    input: `lane_id` (AD-map lane-id-like value)
    output: allowed routing directions (`list`)
    """
    lane = ad.map.lane.getLane(lane_id)
    directions = []
  
    if ad.map.lane.isLaneDirectionPositive(lane):
        directions.append(ad.map.route.RoutingDirection.POSITIVE)
   
    if ad.map.lane.isLaneDirectionNegative(lane):
        directions.append(ad.map.route.RoutingDirection.NEGATIVE)
    if not directions:
        raise RuntimeError(f'Lane {lane_id_to_int(lane_id)} has no legal routing direction.')

    return directions

def create_routing_points(map_match):
    """Build all possible routing start or goal points from one lane match.

    input: `map_match` (AD-map match object)
    output: direction and routing-point pairs (`list[tuple]`)
    """
    para_point = map_match.lane_point.para_point
    routing_points = []
  
    for direction in get_nominal_routing_directions(para_point.lane_id):
        routing_points.append((direction, ad.map.route.createRoutingPoint(para_point, direction)))
  
    return routing_points

def get_match_distance(map_match):
    """Get how far the raw point is from one matched lane.

    input: `map_match` (AD-map match object)
    output: match distance in meters (`float`)
    """
    return distance_to_float(map_match.matched_point_distance)

def get_match_probability(map_match):
    """Get the match confidence for one lane match.

    input: `map_match` (AD-map match object)
    output: match probability (`float`)
    """
    return distance_to_float(map_match.probability)

def find_shortest_route(start_point, goal_point, search_radius=2.0):
    """Find the best legal route between the given start and goal points.

    input: `start_point` (`ad.map.point.ENUPoint`), `goal_point` (`ad.map.point.ENUPoint`), `search_radius` (`float`)
    output: route, start match, goal match, and route length (`tuple`)
    """
    start_matches = get_map_matches(start_point, search_radius)
    goal_matches = get_map_matches(goal_point, search_radius)
  
    if not start_matches:
        raise RuntimeError('No drivable lane found near the start point.')
    if not goal_matches:
        raise RuntimeError('No drivable lane found near the goal point.')
 
    best_result = None
    best_score = None
    straight_line_distance = point_distance(start_point, goal_point)
    # Reject routes that are unrealistically shorter than the direct object-to-object distance.
    minimum_plausible_route_length = max(0.0, straight_line_distance - 2.0)
  
    for start_match in start_matches:
        start_lane_id = get_match_lane_id(start_match)
       
        for goal_match in goal_matches:
            goal_lane_id = get_match_lane_id(goal_match)
            for _, start_routing_point in create_routing_points(start_match):
            
                for _, goal_routing_point in create_routing_points(goal_match):
                    route = ad.map.route.planRoute(start_routing_point, goal_routing_point)
                 
                    if len(route.road_segments) == 0:
                        continue
                    route_length_m = route_length(route)
                    if route_length_m < minimum_plausible_route_length:
                        continue
                    try:
                        build_lane_path(route, start_lane_id, goal_lane_id)
                    except RuntimeError:
                        continue
                    score = (not ad.map.match.isActualWithinLaneMatch(start_match), not ad.map.match.isActualWithinLaneMatch(goal_match), get_match_distance(start_match) + get_match_distance(goal_match), route_length_m, -(get_match_probability(start_match) + get_match_probability(goal_match)))
                    if best_score is None or score < best_score:
                        best_score = score
                        best_result = (route, start_match, goal_match, route_length_m)
  
    if best_result is None:
        raise RuntimeError('No legal route exists between the start and goal positions. All candidate routes were inconsistent with the object positions.')
   
    return best_result

def enu_point_to_tuple(point):
    """Convert one AD-map ENU point into a normal Python tuple.

    input: `point` (`ad.map.point.ENUPoint`)
    output: point as `(x, y, z)` (`tuple[float, float, float]`)
    """
    return (coordinate_to_float(point.x), coordinate_to_float(point.y), coordinate_to_float(point.z))

def get_match_lane_id(map_match):
    """Read the lane id from one AD-map match result.

    input: `map_match` (AD-map match object)
    output: lane id (`int`)
    """
    return lane_id_to_int(map_match.lane_point.para_point.lane_id)

def decode_lane_identifier(ad_lane_id):
    """Split one encoded AD-map lane id into road, section, and lane numbers.

    input: `ad_lane_id` (`int` or AD-map lane-id-like value)
    output: decoded lane details (`dict[str, int | None]`)
    """
    ad_lane_id = lane_id_to_int(ad_lane_id)
    if ad_lane_id <= 10000:
        return {'ad_lane_id': ad_lane_id, 'road_id': None, 'section_id': None, 'lane_id': None}
  
    return {'ad_lane_id': ad_lane_id, 'road_id': ad_lane_id // 10000, 'section_id': ad_lane_id % 10000 // 100, 'lane_id': ad_lane_id % 100 - 50}

def tuple_to_enu_point(point):
    """Convert a plain Python point tuple into an AD-map ENU point.

    input: `point` (`tuple[float, float, float]`)
    output: ENU point (`ad.map.point.ENUPoint`)
    """
    return create_enu_point(x=point[0], y=point[1], z=point[2])

def select_best_match(matches):
    """Pick the best lane match from a list of nearby candidates.

    input: `matches` (`list`)
    output: best match or nothing (`object | None`)
    """
    if not matches:
        return None
 
    return min(matches, key=lambda match: (not ad.map.match.isActualWithinLaneMatch(match), get_match_distance(match), -get_match_probability(match)))

def describe_enu_position(enu_position, search_radius=2.0):
    """Describe one point with its road and lane information.

    input: `enu_position` (`ad.map.point.ENUPoint | tuple[float, float, float]`), `search_radius` (`float`)
    output: point summary (`dict`)
    """
    if hasattr(enu_position, 'x'):
        enu_point = enu_position
    else:
        enu_point = tuple_to_enu_point(enu_position)
    matches = get_map_matches(enu_point, search_radius)
 
    if not matches:
        raise RuntimeError('No drivable lane found near the requested path point.')
    best_match = select_best_match(matches)
    lane_details = decode_lane_identifier(best_match.lane_point.para_point.lane_id)
 
    return {'position': enu_point_to_tuple(enu_point), **lane_details}

def lane_segments_are_connected(current_lane_segment, next_lane_segment):
    """Check if two route lane segments connect in a legal driving order.

    input: `current_lane_segment` (AD-map route lane segment), `next_lane_segment` (AD-map route lane segment)
    output: whether they connect legally (`bool`)
    """
    current_lane_id = lane_id_to_int(current_lane_segment.lane_interval.lane_id)
    next_lane_id = lane_id_to_int(next_lane_segment.lane_interval.lane_id)
    current_successors = {lane_id_to_int(lane_id) for lane_id in current_lane_segment.successors}
    next_predecessors = {lane_id_to_int(lane_id) for lane_id in next_lane_segment.predecessors}
    return next_lane_id in current_successors or current_lane_id in next_predecessors

def lane_segment_is_wrong_way(lane_segment):
    """Check if one route lane segment goes against the lane direction.

    input: `lane_segment` (AD-map route lane segment)
    output: whether the segment is wrong-way (`bool`)
    """
    return bool(to_base_value(lane_segment.lane_interval.wrong_way))

def lane_segment_offset(lane_segment):
    """Get the lane offset value used while comparing lane changes.

    input: `lane_segment` (AD-map route lane segment)
    output: lane offset (`int`)
    """
    return int(lane_segment.route_lane_offset)

def get_best_transition(current_lane_id, current_lane_segment, current_cost, current_path, previous_lane_segment_map, next_lane_id, next_lane_segment):
    """Find the cheapest legal way to move from the current lane into the next lane.

    input: current lane info (`int`, AD-map lane segment, `int`, `list[int]`, `dict[int, object]`) and next lane info (`int`, AD-map lane segment)
    output: best next cost and lane path (`tuple[int, list[int]] | None`)
    """
    best_transition = None
    for connector_lane_id, connector_lane_segment in previous_lane_segment_map.items():
        if lane_segment_is_wrong_way(connector_lane_segment):
            continue
        if not lane_segments_are_connected(connector_lane_segment, next_lane_segment):
            continue
     
        lateral_shift_cost = abs(lane_segment_offset(connector_lane_segment) - lane_segment_offset(current_lane_segment))
        retroactive_lane_switch_cost = 0 if connector_lane_id == current_lane_id else 100
        candidate_cost = current_cost + lateral_shift_cost + retroactive_lane_switch_cost
    
        if connector_lane_id == current_lane_id:
            candidate_path = current_path + [next_lane_id]
        else:
            candidate_path = current_path[:-1] + [connector_lane_id, next_lane_id]
        candidate = (candidate_cost, candidate_path)
        if best_transition is None or candidate[0] < best_transition[0]:
            best_transition = candidate
  
    return best_transition

def build_lane_segment_maps(route):
    """Build quick lane lookup tables for every road segment in the route.

    input: `route` (AD-map route object)
    output: road segments and lane maps (`tuple[list, list[dict[int, object]]]`)
    """
    road_segments = list(route.road_segments)
    if not road_segments:
        raise RuntimeError('Route contains no road segments.')
  
    lane_segment_maps = []
   
    for road_segment in road_segments:
        lane_segment_map = {lane_id_to_int(lane_segment.lane_interval.lane_id): lane_segment for lane_segment in road_segment.drivable_lane_segments if not lane_segment_is_wrong_way(lane_segment)}
        if not lane_segment_map:
            raise RuntimeError('Route contains only wrong-way lane segments in one road segment.')
        lane_segment_maps.append(lane_segment_map)
  
    return (road_segments, lane_segment_maps)

def build_lane_path(route, start_lane_id, goal_lane_id):
    """Pick one continuous lane-by-lane path from the start lane to the goal lane.

    input: `route` (AD-map route object), `start_lane_id` (`int`), `goal_lane_id` (`int`)
    output: selected lane id for each route segment (`dict[int, int]`)
    """
    road_segments, lane_segment_maps = build_lane_segment_maps(route)
  
    if start_lane_id not in lane_segment_maps[0]:
   
        raise RuntimeError(f'Matched start lane {start_lane_id} is not in the routed first segment.')
  
    path_costs = {start_lane_id: (0, [start_lane_id])}
    previous_lane_segment_map = lane_segment_maps[0]
   
    for lane_segment_map in lane_segment_maps[1:]:
        next_costs = {}
        # Keep only the cheapest legal way to reach each lane in the next road segment.
      
        for next_lane_id, next_lane_segment in lane_segment_map.items():
            best_candidate = None
        
            for current_lane_id, (current_cost, current_path) in path_costs.items():
                current_lane_segment = previous_lane_segment_map[current_lane_id]
                candidate = get_best_transition(current_lane_id, current_lane_segment, current_cost, current_path, previous_lane_segment_map, next_lane_id, next_lane_segment)
            
                if candidate is not None and (best_candidate is None or candidate[0] < best_candidate[0]):
                    best_candidate = candidate
          
            if best_candidate is not None:
                next_costs[next_lane_id] = best_candidate
        if not next_costs:
            raise RuntimeError('Could not build a continuous lane path through the routed road segments.')
     
        path_costs = next_costs
        previous_lane_segment_map = lane_segment_map
  
    if goal_lane_id not in path_costs:
        raise RuntimeError(f'Matched goal lane {goal_lane_id} is not reachable in the routed last segment.')
    selected_lane_ids = path_costs[goal_lane_id][1]
  
    return {int(road_segment.segment_count_from_destination): lane_id for road_segment, lane_id in zip(road_segments, selected_lane_ids)}

def get_route_position_at_distance(route, origin_route_position, distance_from_start):
    """Get the route position at one distance measured from the matched start.

    input: `route` (AD-map route object), `origin_route_position` (AD-map route para-point), `distance_from_start` (`float`)
    output: sampled route position (`object | None`)
    """
    route_position = ad.map.route.RouteParaPoint()
    found = ad.map.route.calculateRouteParaPointAtDistance(route, origin_route_position, ad.physics.Distance(distance_from_start), route_position)
    if not found:
        return None
    return route_position

def get_selected_lane_para_point(route_position, route, lane_ids_by_segment):
    """Project one sampled route position onto the lane chosen for that segment.

    input: `route_position` (AD-map route para-point), `route` (AD-map route object), `lane_ids_by_segment` (`dict[int, int]`)
    output: lane para-point on the selected lane (`object`)
    """
    segment_counter = int(route_position.segment_count_from_destination)
    expected_lane_id = lane_ids_by_segment[segment_counter]
 
    for para_point in ad.map.route.getLaneParaPoints(route_position, route):
        if lane_id_to_int(para_point.lane_id) == expected_lane_id:
            return para_point
 
    raise RuntimeError('Failed to project the sampled route position onto the selected lane path.')

def sample_lane_center(para_point):
    """Sample one point from the center of a lane.

    input: `para_point` (AD-map lane para-point)
    output: center point in ENU coordinates (`tuple[float, float, float]`)
    """
    center_point = ad.map.lane.getENULanePoint(para_point, ad.physics.ParametricValue(0.5))
    return enu_point_to_tuple(center_point)

def get_parallel_lane_ids(lane_segment_map, center_lane_id):
    """Collect all usable adjacent lane ids beside one center lane.

    input: `lane_segment_map` (`dict[int, object]`), `center_lane_id` (`int`)
    output: adjacent lane ids (`list[int]`)
    """
    if center_lane_id not in lane_segment_map:
        raise RuntimeError(f'Lane {center_lane_id} is not available in the current road segment.')
    left_lane_ids = []
    visited_lane_ids = {center_lane_id}
    current_lane_id = center_lane_id
  
    while True:
        left_lane_id = lane_id_to_int(lane_segment_map[current_lane_id].left_neighbor)
        if left_lane_id <= 0 or left_lane_id not in lane_segment_map or left_lane_id in visited_lane_ids:
            break
        left_lane_ids.append(left_lane_id)
        visited_lane_ids.add(left_lane_id)
        current_lane_id = left_lane_id
    right_lane_ids = []
    current_lane_id = center_lane_id
  
    while True:
        right_lane_id = lane_id_to_int(lane_segment_map[current_lane_id].right_neighbor)
        if right_lane_id <= 0 or right_lane_id not in lane_segment_map or right_lane_id in visited_lane_ids:
            break
        right_lane_ids.append(right_lane_id)
        visited_lane_ids.add(right_lane_id)
        current_lane_id = right_lane_id
  
    return left_lane_ids + right_lane_ids

def points_are_close(first_point, second_point, tolerance=0.05):
    """Check if two sampled ENU points are basically the same.

    input: `first_point` (`tuple[float, float, float]`), `second_point` (`tuple[float, float, float]`), `tolerance` (`float`)
    output: whether the points are close (`bool`)
    """
    return all((abs(first_value - second_value) <= tolerance for first_value, second_value in zip(first_point, second_point)))

def find_adjacent_points(route, start_match, goal_match, route_points, route_point_index=5, spacing=1.0):
    """Find nearby side-lane points around one route sample.

    input: `route` (AD-map route object), `start_match` (AD-map match object), `goal_match` (AD-map match object), `route_points` (`list[tuple[float, float, float]]`), `route_point_index` (`int`), `spacing` (`float`)
    output: selected route point and adjacent lane points (`dict`)
    """
    if not route_points:
        raise RuntimeError('The route point list is empty.')
    if spacing <= 0.0:
        raise ValueError('Route-point spacing must be greater than zero.')
  
    selected_route_index = min(max(int(route_point_index), 0), len(route_points) - 1)
    target_route_point = route_points[selected_route_index]
    start_lane_id = get_match_lane_id(start_match)
    goal_lane_id = get_match_lane_id(goal_match)
    lane_ids_by_segment = build_lane_path(route, start_lane_id, goal_lane_id)
    road_segments, lane_segment_maps = build_lane_segment_maps(route)
    lane_segment_maps_by_counter = {int(road_segment.segment_count_from_destination): lane_segment_map for road_segment, lane_segment_map in zip(road_segments, lane_segment_maps)}
    origin_route_position = ad.map.route.RouteParaPoint()
    found_origin = ad.map.route.getRouteParaPointFromParaPoint(start_match.lane_point.para_point, route, origin_route_position)
    
    if not found_origin:
        raise RuntimeError('Could not locate the matched start point on the computed route.')
 
    total_length = route_length(route)
    best_candidate = None
    distance_from_start = 0.0
  
    while distance_from_start <= total_length:
        route_position = get_route_position_at_distance(route, origin_route_position, distance_from_start)
     
        if route_position is None:
            raise RuntimeError(f'Could not locate the route position at {distance_from_start:.2f} m while searching adjacent lanes.')
     
        segment_counter = int(route_position.segment_count_from_destination)
        selected_lane_id = lane_ids_by_segment[segment_counter]
        para_points_by_lane_id = {lane_id_to_int(para_point.lane_id): para_point for para_point in ad.map.route.getLaneParaPoints(route_position, route)}
      
        if selected_lane_id not in para_points_by_lane_id:
            raise RuntimeError(f'Selected lane {selected_lane_id} is missing from the sampled parallel lane set.')
      
        selected_lane_point = sample_lane_center(para_points_by_lane_id[selected_lane_id])
        candidate_distance = math.dist(target_route_point, selected_lane_point)
     
        if best_candidate is None or candidate_distance < best_candidate['distance_to_target']:
            best_candidate = {'distance_to_target': candidate_distance, 'segment_counter': segment_counter, 'selected_lane_id': selected_lane_id, 'route_position': route_position, 'para_points_by_lane_id': para_points_by_lane_id}
      
        distance_from_start += spacing
    
    if best_candidate is None:
        raise RuntimeError('Could not find a sampled route point for the requested adjacent-lane query.')
   
    adjacent_points = []
    adjacent_lane_ids = get_parallel_lane_ids(lane_segment_maps_by_counter[best_candidate['segment_counter']], best_candidate['selected_lane_id'])
  
    for adjacent_lane_id in adjacent_lane_ids:
        para_point = best_candidate['para_points_by_lane_id'].get(adjacent_lane_id)
 
        if para_point is None:
            continue
   
        adjacent_points.append({'position': sample_lane_center(para_point), **decode_lane_identifier(adjacent_lane_id)})
  
    return {'selected_route_index': selected_route_index, 'selected_route_point': target_route_point, 'selected_lane_id': best_candidate['selected_lane_id'], 'adjacent_points': adjacent_points}

def sample_route_centerline(route, start_match, goal_match, spacing=1.0, start_point=None, goal_point=None):
    """Sample the final route into evenly spaced center points.

    input: `route` (AD-map route object), `start_match` (AD-map match object), `goal_match` (AD-map match object), `spacing` (`float`), `start_point` (`ad.map.point.ENUPoint | None`), `goal_point` (`ad.map.point.ENUPoint | None`)
    output: sampled route points (`list[tuple[float, float, float]]`)
    """
    if spacing <= 0.0:
        raise ValueError('Route-point spacing must be greater than zero.')
 
    start_lane_id = get_match_lane_id(start_match)
    goal_lane_id = get_match_lane_id(goal_match)
    lane_ids_by_segment = build_lane_path(route, start_lane_id, goal_lane_id)
    origin_route_position = ad.map.route.RouteParaPoint()
    found_origin = ad.map.route.getRouteParaPointFromParaPoint(start_match.lane_point.para_point, route, origin_route_position)
  
    if not found_origin:
        raise RuntimeError('Could not locate the matched start point on the computed route.')
    total_length = route_length(route)
    sampled_points = []
    distance_from_start = 0.0
   
    while distance_from_start < total_length:
        # Sample along the selected lane path, not just the broader road segment route.
        route_position = get_route_position_at_distance(route, origin_route_position, distance_from_start)
        if route_position is None:
            raise RuntimeError(f'Route sampling stopped before the destination at {distance_from_start:.2f} m of {total_length:.2f} m.')
        para_point = get_selected_lane_para_point(route_position, route, lane_ids_by_segment)
        sampled_points.append(sample_lane_center(para_point))
        distance_from_start += spacing
   
    final_route_position = get_route_position_at_distance(route, origin_route_position, total_length)
  
    if final_route_position is None:
        raise RuntimeError('Could not sample the final route position.')
  
    final_para_point = get_selected_lane_para_point(final_route_position, route, lane_ids_by_segment)
    final_center_point = sample_lane_center(final_para_point)
   
    if not sampled_points or not points_are_close(sampled_points[-1], final_center_point):
        sampled_points.append(final_center_point)
    if start_point is not None:
        start_tuple = enu_point_to_tuple(start_point)
        if not sampled_points or not points_are_close(sampled_points[0], start_tuple):
            sampled_points.insert(0, start_tuple)
    if goal_point is not None:
        goal_tuple = enu_point_to_tuple(goal_point)
        if not sampled_points or not points_are_close(sampled_points[-1], goal_tuple):
            sampled_points.append(goal_tuple)
 
    print('Route length:', f'{total_length:.2f} m')
    print('Expected points:', int(total_length / spacing) + 1)
    print('Generated points:', len(sampled_points))
    return sampled_points

def close_map():
    """Clear the loaded AD-map data after the route work is done.

    input: none (`None`)
    output: none (`None`)
    """
    ad.map.access.cleanup()
