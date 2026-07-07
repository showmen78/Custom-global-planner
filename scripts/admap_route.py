"""Provide the CARLA-independent routing.

This module works on top of `ad_map_access` and the OpenDRIVE file in
`maps/`. It loads the map, matches raw positions onto drivable lanes, finds a
legal route, chooses a continuous lane path, samples centerline points, and
can also query adjacent parallel lane points. 
"""
from pathlib import Path
import heapq
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

def create_enu_point(p):
    """Build one AD-map point from plain x, y, z values.

    input:  (`tuple[float, float, float]`)
    output: ENU point (`ad.map.point.ENUPoint`)
    """
    point = ad.map.point.ENUPoint()
    point.x = ad.map.point.ENUCoordinate(float(p[0]))
    point.y = ad.map.point.ENUCoordinate(float(p[1]))
    point.z = ad.map.point.ENUCoordinate(float(p[2]))
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


def get_match_lane_id(map_match):
    """Read the lane id from one AD-map match result.

    input: `map_match` (AD-map match object)
    output: lane id (`int`)
    """
    return int(to_base_value(map_match.lane_point.para_point.lane_id))


def get_match_center_point(map_match):
    """Project one matched point onto the center of its lane.

    input: `map_match` (AD-map match object)
    output: snapped center point as `(x, y, z)` (`tuple[float, float, float]`)
    """
    return sample_lane_center(map_match.lane_point.para_point)


def get_centerline_match_distance(raw_point, map_match):
    """Measure how far one raw point is from the center of one matched lane.

    input: `raw_point` (`ad.map.point.ENUPoint`), `map_match` (AD-map match object)
    output: centerline distance in meters (`float`)
    """
    return math.dist(enu_point_to_tuple(raw_point), get_match_center_point(map_match))


def build_route_candidates(enu_point, search_radius=8.0):
    """Turn nearby lane matches into snapped routing candidates.

    input: `enu_point` (`ad.map.point.ENUPoint`), `search_radius` (`float`)
    output: routing candidates (`list[dict[str, object]]`)
    """
    matches = get_map_matches(enu_point, search_radius)
    raw_point = enu_point_to_tuple(enu_point)
    candidates = []

    for match in matches:
        center_point = get_match_center_point(match)
        snap_distance = math.dist(raw_point, center_point)
        candidates.append({'match': match, 'lane_id': get_match_lane_id(match), 'center_point': center_point, 
                           'center_point_enu': create_enu_point(center_point), 
                           'snap_distance': snap_distance, 'is_in_lane': ad.map.match.isActualWithinLaneMatch(match), 'probability': get_match_probability(match)})



    """
    the lane whose center is closest to the start/goal point
    a lane where the point is actually inside the lane
    a lane with higher match confidence
    the smaller lane id if everything else is tied
    are prefered
    """
    

    candidates.sort(key=lambda candidate: (candidate['snap_distance'], not candidate['is_in_lane'], -candidate['probability'], candidate['lane_id']))
    return candidates


def parametric_value_to_float(value):
    """Convert one AD-map parametric value into a normal Python float.

    input: `value` (AD-map parametric-value-like object)
    output: parametric value (`float`)
    """
    return float(to_base_value(value))


def get_lane(lane_id):
    """Load one lane object from AD-map using its lane id.

    input: `lane_id` (`int` or AD-map lane-id-like value)
    output: lane object (`ad.map.lane.Lane`)
    """
    return ad.map.lane.getLane(lane_id)


def lane_is_positive(lane_id):
    """Check whether one lane follows the positive parametric direction.

    input: `lane_id` (`int` or AD-map lane-id-like value)
    output: whether the lane direction is positive (`bool`)
    """
    return get_lane(lane_id).direction == ad.map.lane.LaneDirection.POSITIVE


def lanes_have_same_direction(first_lane_id, second_lane_id):
    """Check whether two lanes move in the same driving direction.

    input: `first_lane_id` (`int`), `second_lane_id` (`int`)
    output: whether both lanes have the same direction (`bool`)
    """
    return lane_is_positive(first_lane_id) == lane_is_positive(second_lane_id)


def get_lane_length_m(lane_id):
    """Read one lane length in meters.

    input: `lane_id` (`int` or AD-map lane-id-like value)
    output: lane length in meters (`float`)
    """
    return distance_to_float(get_lane(lane_id).length)


def get_travel_start_offset(lane_id):
    """Return the parametric offset where driving starts on one lane.

    input: `lane_id` (`int`)
    output: travel-start offset (`float`)
    """
    return 0.0 if lane_is_positive(lane_id) else 1.0


def get_travel_end_offset(lane_id):
    """Return the parametric offset where driving leaves one lane.
    in AD-map, a lane position is stored with a parametric offset from 0.0 to 1.0
    some lanes are driven from 0 -> 1, some lanes are driven from 1 -> 0
    if lane A is positive, driving starts at offset 0.0->1, else from 1->0

    input: `lane_id` (`int`)
    output: travel-end offset (`float`)
    """
    return 1.0 if lane_is_positive(lane_id) else 0.0


def lane_offsets_move_forward(lane_id, start_offset, end_offset, tolerance=1e-6):
    """Check whether one offset change follows the lane direction. 
    checks whether moving from start_offset to end_offset goes in the legal driving direction of that lane.
    The offset here is the position along a lane, usually from:
    0.0 = one end of the lane
    1.0 = the other end of the lane
    positive lane: 0.2 -> 0.8 is forward
    negative lane: 0.8 -> 0.2 is forward

    input: `lane_id` (`int`), `start_offset` (`float`), `end_offset` (`float`), `tolerance` (`float`)
    output: whether the move is forward on that lane (`bool`)
    """
    if lane_is_positive(lane_id):
        return end_offset >= start_offset - tolerance
    return end_offset <= start_offset + tolerance


def get_forward_contact_location(lane_id):
    """tells the code which lane connection means “go forward” for that lane.
    for a positive lane, driving forward means going toward its successor lane
    for a negative lane, driving forward means going toward its predecessor lane

    input: `lane_id` (`int`)
    output: forward contact location (`ad.map.lane.ContactLocation`)
    """
    return ad.map.lane.ContactLocation.SUCCESSOR if lane_is_positive(lane_id) else ad.map.lane.ContactLocation.PREDECESSOR


def lane_is_intersection(lane_id):
    """Check whether one lane is marked as an intersection lane by AD-map.

    input: `lane_id` (`int`)
    output: whether the lane is an intersection lane (`bool`)
    """
    return get_lane(lane_id).type == ad.map.lane.LaneType.INTERSECTION


def sample_lane_center_at_offset(lane_id, parametric_offset):
    """Sample one lane-center point at a chosen offset along the lane.

    input: `lane_id` (`int`), `parametric_offset` (`float`)
    output: center point in ENU coordinates (`tuple[float, float, float]`)
    """
    ## creates a para point with info which lane you are on and where in that lane you are.
    para_point = ad.map.point.ParaPoint()
    para_point.lane_id = lane_id
    para_point.parametric_offset = ad.physics.ParametricValue(float(parametric_offset))

    return sample_lane_center(para_point= para_point)


def get_allowed_lane_transitions(lane_id, lane_change_penalty_m=15.0):
    """List the legal next-lane moves from one lane. takes one lane and asks: “from this lane, where am I legally allowed to go next?”
    
    looks at all connected lanes of the current lane
    keeps the forward road connection in the legal driving direction
    keeps left/right lane changes only if the other lane has the same driving direction
    ignores non-drivable lanes
    ignores opposite-direction lane changes

    input: `lane_id` (`int`), `lane_change_penalty_m` (`float`)
    output: lane transitions (`list[dict[str, object]]`)
    """
    transitions = []
    forward_location = get_forward_contact_location(lane_id)

    for contact_lane in get_lane(lane_id).contact_lanes:
        next_lane_id = int(to_base_value(contact_lane.to_lane))
        next_lane = get_lane(next_lane_id)
        if not ad.map.lane.isRouteable(next_lane):
            continue
        if contact_lane.location == forward_location:
            transitions.append({'to_lane_id': next_lane_id, 'type': 'forward', 'cost_m': get_lane_length_m(next_lane_id)})
            continue
        if not lane_is_intersection(lane_id) and not lane_is_intersection(next_lane_id) and contact_lane.location in (ad.map.lane.ContactLocation.LEFT, ad.map.lane.ContactLocation.RIGHT) and lanes_have_same_direction(lane_id, next_lane_id):
            transitions.append({'to_lane_id': next_lane_id, 'type': 'lane_change', 'cost_m': lane_change_penalty_m})

    return transitions


def find_lane_path(start_lane_id, goal_lane_id, start_offset, goal_offset, lane_change_penalty_m=15.0):
    """Find a legal lane-by-lane path without jumping into opposite-direction lanes.
    It is a shortest-path search over lanes. It tries to find a legal sequence of lanes from the start lane to the goal lane, 
    while respecting direction and allowed lane changes.

    input: `start_lane_id` (`int`), `goal_lane_id` (`int`), `start_offset` (`float`), `goal_offset` (`float`), `lane_change_penalty_m` (`float`)
    output: lane path and transition types (`tuple[list[int] | None, list[str] | None]`)
    """
    
    
    #if start and goal are already on the same lane, and the goal is ahead in the legal driving direction, then the route can finish directly on that lane.
    direct_finish_allowed = start_lane_id == goal_lane_id and lane_offsets_move_forward(start_lane_id, start_offset, goal_offset)
    
    """
    frontier = [(0.0, start_lane_id, [start_lane_id], [], False)], Creates the search queue.
    current cost = 0.0
    current lane = start_lane_id
    current lane path = [start_lane_id]
    transition types so far = []
    has_left_start = False means we have not moved to another lane yet
    
    """
    
    frontier = [(0.0, start_lane_id, [start_lane_id], [], False)]
    
    #Stores the cheapest known cost for each search state.
    #At the beginning, the cheapest way to be at the start lane without leaving it is 0.0.
    best_costs = {(start_lane_id, False): 0.0}

    while frontier:
        #Pops the cheapest state from the priority queue. So the search always explores the current best option first.
        current_cost, lane_id, lane_path, transition_types, has_left_start = heapq.heappop(frontier)
        state_key = (lane_id, has_left_start)
        if current_cost != best_costs.get(state_key):
            continue
        if lane_id == goal_lane_id and (has_left_start or direct_finish_allowed):
            return (lane_path, transition_types)

        for transition in get_allowed_lane_transitions(lane_id, lane_change_penalty_m):
            next_lane_id = transition['to_lane_id']
            next_cost = current_cost + transition['cost_m']
            next_state_key = (next_lane_id, True)
            if next_cost >= best_costs.get(next_state_key, float('inf')):
                continue
            best_costs[next_state_key] = next_cost
            heapq.heappush(frontier, (next_cost, next_lane_id, lane_path + [next_lane_id], transition_types + [transition['type']], True))

    return (None, None)


def get_lane_segment_length_m(lane_id, start_offset, end_offset):
    """Measure how much distance is traveled along one lane segment.

    input: `lane_id` (`int`), `start_offset` (`float`), `end_offset` (`float`)
    output: traveled lane distance in meters (`float`)
    """
    return abs(end_offset - start_offset) * get_lane_length_m(lane_id)


def advance_offset_forward(lane_id, start_offset, distance_m):
    """Move one parametric offset forward along the lane by a physical distance.

    input: `lane_id` (`int`), `start_offset` (`float`), `distance_m` (`float`)
    output: forward-shifted offset on the same lane (`float`)
    """
    offset_delta = max(distance_m, 0.0) / max(get_lane_length_m(lane_id), 1e-6)
    if lane_is_positive(lane_id):
        return min(1.0, start_offset + offset_delta)
    return max(0.0, start_offset - offset_delta)


def choose_lane_change_offset(lane_id, start_offset, goal_offset=None, lane_change_distance_m=8.0):
    """Choose one forward point on the current lane where the lane change happens.

    input: `lane_id` (`int`), `start_offset` (`float`), `goal_offset` (`float | None`), `lane_change_distance_m` (`float`)
    output: chosen lane-change offset (`float`)
    """
    offset_delta = lane_change_distance_m / max(get_lane_length_m(lane_id), lane_change_distance_m)
    if lane_is_positive(lane_id):
        lane_change_offset = min(0.95, start_offset + max(offset_delta, (1.0 - start_offset) * 0.35))
        if goal_offset is not None:
            lane_change_offset = min(lane_change_offset, max(start_offset + 0.02, goal_offset - 0.02))
        return lane_change_offset

    lane_change_offset = max(0.05, start_offset - max(offset_delta, start_offset * 0.35))
    if goal_offset is not None:
        lane_change_offset = max(lane_change_offset, min(start_offset - 0.02, goal_offset + 0.02))
    return lane_change_offset


def choose_lane_change_offsets(lane_id, next_lane_id, start_offset, goal_offset=None, lane_change_distance_m=8.0, use_lateral=False):
    """Choose where the lane change starts and where it lands on the adjacent lane.

    Most lane changes stay diagonal with a short forward landing shift. For the
    final approach, the route builder can request a lateral lane change so the
    connector stays at the same along-lane position.

    input: `lane_id` (`int`), `next_lane_id` (`int`), `start_offset` (`float`), `goal_offset` (`float | None`), `lane_change_distance_m` (`float`), `use_lateral` (`bool`)
    output: current-lane and target-lane offsets (`tuple[float, float]`)
    """
    from_offset = choose_lane_change_offset(lane_id, start_offset, goal_offset, lane_change_distance_m)
    if use_lateral:
        return (from_offset, from_offset)

    landing_distance_m = lane_change_distance_m * 0.45
    to_offset = advance_offset_forward(next_lane_id, from_offset, landing_distance_m)

    if goal_offset is not None:
        if lane_is_positive(next_lane_id):
            to_offset = min(to_offset, goal_offset)
        else:
            to_offset = max(to_offset, goal_offset)

    return (from_offset, to_offset)


def build_custom_route(start_match, goal_match, lane_path, transition_types, lane_change_distance_m=8.0):
    """Turn one lane-id path into a route description that can be sampled later.
    
    [
    {'type': 'lane_segment', 'lane_id': 160149, 'start_offset': 0.73, 'end_offset': 1.0},
    {'type': 'lane_segment', 'lane_id': 250149, 'start_offset': 0.0, 'end_offset': 0.45}, #for lane segment 
    {'type': 'lane_change', 'from_lane_id': 250149, 'to_lane_id': 250148, 'from_offset': 0.45, 'to_offset': 0.51}, # for lane change
    ...
]

    the lane change happens at one chosen position along the road
    usually it lands a short distance ahead on the adjacent lane
    near the end of the route, the last two lane changes stay lateral instead
    
    input: `start_match` (AD-map match object), `goal_match` (AD-map match object), `lane_path` (`list[int]`), `transition_types` (`list[str]`), `lane_change_distance_m` (`float`)
    output: custom route data (`dict[str, object]`)
    """
    start_lane_id = get_match_lane_id(start_match)
    goal_lane_id = get_match_lane_id(goal_match)
    start_offset = parametric_value_to_float(start_match.lane_point.para_point.parametric_offset)
    goal_offset = parametric_value_to_float(goal_match.lane_point.para_point.parametric_offset)
    route_steps = []
    current_offset = start_offset
    lane_change_indices = [index for index, transition_type in enumerate(transition_types) if transition_type == 'lane_change']
    final_lateral_lane_changes = set(lane_change_indices[-2:])

    for index, lane_id in enumerate(lane_path):
        is_last_lane = index == len(lane_path) - 1
        if is_last_lane:
            if not lane_offsets_move_forward(lane_id, current_offset, goal_offset):
                raise RuntimeError('The goal point is behind the selected lane path direction.')
            route_steps.append({'type': 'lane_segment', 'lane_id': lane_id, 'start_offset': current_offset, 'end_offset': goal_offset})
            break

        next_lane_id = lane_path[index + 1]
        transition_type = transition_types[index]
        
        #if moving forward
        if transition_type == 'forward':
            end_offset = get_travel_end_offset(lane_id) # 1 for positive 0 for negative direction
            
            if not lane_offsets_move_forward(lane_id, current_offset, end_offset): #a bool that indicate whether the offset is moving forward or not
                raise RuntimeError('The selected route would move backward on the current lane.')
            
            route_steps.append({'type': 'lane_segment', 'lane_id': lane_id, 'start_offset': current_offset, 'end_offset': end_offset})
            current_offset = get_travel_start_offset(next_lane_id)
            continue

        #if not lane change and not move forward
        if transition_type != 'lane_change':
            raise RuntimeError(f'Unsupported lane transition type: {transition_type}')
        
        #otherwise if lane change 
        is_goal_lane = index + 1 == len(lane_path) - 1 and next_lane_id == goal_lane_id
        use_lateral_lane_change = index in final_lateral_lane_changes
        lane_change_from_offset, lane_change_to_offset = choose_lane_change_offsets(
            lane_id,
            next_lane_id,
            current_offset,
            goal_offset if is_goal_lane else None,
            lane_change_distance_m,
            use_lateral=use_lateral_lane_change,
        )

        # the lane isn't moving the forward direction
        if not lane_offsets_move_forward(lane_id, current_offset, lane_change_from_offset):
            raise RuntimeError('The selected lane change would move backward on the lane.')
        
        route_steps.append({'type': 'lane_segment', 'lane_id': lane_id, 'start_offset': current_offset, 'end_offset': lane_change_from_offset})
        route_steps.append({'type': 'lane_change', 'from_lane_id': lane_id, 'to_lane_id': next_lane_id, 'from_offset': lane_change_from_offset, 'to_offset': lane_change_to_offset})
        current_offset = lane_change_to_offset

    route_data = {'lane_path': lane_path, 'transition_types': transition_types, 'steps': route_steps, 'start_lane_id': start_lane_id, 'goal_lane_id': goal_lane_id, 'start_offset': start_offset, 'goal_offset': goal_offset}
    route_data['route_length_m'] = get_custom_route_length(route_data)
    return route_data


def get_custom_route_length(route_data):
    """Measure the full length of one custom lane route.

    input: `route_data` (`dict[str, object]`)
    output: route length in meters (`float`)
    """
    total_length_m = 0.0
    for step in route_data['steps']:
        if step['type'] == 'lane_segment': #if following a lane 
            total_length_m += get_lane_segment_length_m(step['lane_id'], step['start_offset'], step['end_offset'])
            continue
        
        #otherwise in case of lane change
        start_point = sample_lane_center_at_offset(step['from_lane_id'], step['from_offset'])
        end_point = sample_lane_center_at_offset(step['to_lane_id'], step['to_offset'])
        total_length_m += math.dist(start_point, end_point)
        
    return total_length_m


def find_route_from_snapped_matches(start_match, goal_match, lane_change_penalty_m=15.0, lane_change_distance_m=8.0):
    """Build one legal route between already-snapped start and goal lane matches.

    input: snapped start and goal matches (AD-map match objects), `lane_change_penalty_m` (`float`), `lane_change_distance_m` (`float`)
    output: route description (`dict[str, object]`)
    """
    start_lane_id = get_match_lane_id(start_match)
    goal_lane_id = get_match_lane_id(goal_match)
    start_offset = parametric_value_to_float(start_match.lane_point.para_point.parametric_offset)
    goal_offset = parametric_value_to_float(goal_match.lane_point.para_point.parametric_offset)
    lane_path, transition_types = find_lane_path(start_lane_id, goal_lane_id, start_offset, goal_offset, lane_change_penalty_m)
    if lane_path is None or transition_types is None:
        raise RuntimeError('No legal route exists between the snapped start and goal lanes.')

    return build_custom_route(start_match, goal_match, lane_path, transition_types, lane_change_distance_m)


def find_shortest_route(start_point, goal_point, search_radius=2.0, lane_change_penalty_m=15.0, lane_change_distance_m=8.0):
    """Route between legal start/goal lane candidates while keeping fixed main snap points.

    input: `start_point` (`ad.map.point.ENUPoint`), `goal_point` (`ad.map.point.ENUPoint`), `search_radius` (`float`), `lane_change_penalty_m` (`float`), `lane_change_distance_m` (`float`)
    output: route, route length, snapped start point, and snapped goal point (`tuple`)
    """
    start_candidates = build_route_candidates(start_point, search_radius)
    goal_candidates = build_route_candidates(goal_point, search_radius)
    if not start_candidates:
        raise RuntimeError('No drivable lane found near the start point.')
    if not goal_candidates:
        raise RuntimeError('No drivable lane found near the goal point.')

    snapped_start_point = start_candidates[0]['center_point_enu']
    snapped_goal_point = goal_candidates[0]['center_point_enu']
    best_route = None
    best_score = None

    for start_candidate in start_candidates:
        for goal_candidate in goal_candidates:
            try:
                route_data = find_route_from_snapped_matches(
                    start_candidate['match'],
                    goal_candidate['match'],
                    lane_change_penalty_m=lane_change_penalty_m,
                    lane_change_distance_m=lane_change_distance_m,
                )
            except RuntimeError:
                continue

            score = (
                start_candidate['snap_distance'] + goal_candidate['snap_distance'],
                route_data['route_length_m'],
                not start_candidate['is_in_lane'],
                not goal_candidate['is_in_lane'],
                -(start_candidate['probability'] + goal_candidate['probability']),
            )
            if best_score is None or score < best_score:
                best_score = score
                best_route = route_data

    if best_route is None:
        raise RuntimeError('No legal route exists between any nearby start and goal lane candidates.')

    return (best_route, best_route['route_length_m'], snapped_start_point, snapped_goal_point)

def enu_point_to_tuple(point):
    """Convert one AD-map ENU point into a normal Python tuple.

    input: `point` (`ad.map.point.ENUPoint`)
    output: point as `(x, y, z)` (`tuple[float, float, float]`)
    """
    #return (coordinate_to_float(point.x), coordinate_to_float(point.y), coordinate_to_float(point.z))
    return (float(to_base_value(point.x)), float(to_base_value(point.y)), float(to_base_value(point.z)))


def get_opendrive_lane_info(ad_lane_id):
    """Split one AD-map lane id back into the OpenDRIVE ids used to create it.

    AD-map does not store separate `road_id` or `section_id` fields on the lane
    object. During OpenDRIVE import, `map_repo` builds the AD lane id with the
    official helper `opendrive::geometry::laneId(roadId, laneSectionIndex, laneIndex)`.
    This function reverses that exact format so the printed ids match the
    original OpenDRIVE lane information.

    input: `ad_lane_id` (`int` or AD-map lane-id-like value)
    output: OpenDRIVE lane details (`dict[str, int | None]`)
    """
    ad_lane_id = int(to_base_value(ad_lane_id))
    if ad_lane_id <= 10000:
        return {'ad_lane_id': ad_lane_id, 'road_id': None, 'lane_section_index': None, 'section_id': None, 'lane_id': None}

    road_id = ad_lane_id // 10000
    lane_section_index = ad_lane_id % 10000 // 100
    lane_id = ad_lane_id % 100 - 50
    return {'ad_lane_id': ad_lane_id, 'road_id': road_id, 'lane_section_index': lane_section_index, 'section_id': lane_section_index, 'lane_id': lane_id}




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
    
    {
    'position': (28.6, 42.7, 0.6),
    'ad_lane_id': 160149,
    'road_id': 16,
    'lane_section_index': 1,
    'section_id': 1,
    'lane_id': -1,
}

    input: `enu_position` (`ad.map.point.ENUPoint | tuple[float, float, float]`), `search_radius` (`float`)
    output: point summary (`dict`)
    """
    if hasattr(enu_position, 'x'):
        enu_point = enu_position
    else:
        enu_point = create_enu_point(enu_position)
    matches = get_map_matches(enu_point, search_radius)
 
    if not matches:
        raise RuntimeError('No drivable lane found near the requested path point.')
    best_match = select_best_match(matches)
    lane_details = get_opendrive_lane_info(best_match.lane_point.para_point.lane_id)
 
    return {'position': enu_point_to_tuple(enu_point), **lane_details}



def sample_lane_center(para_point):
    """Sample one point from the center of a lane.

    input: `para_point` (AD-map lane para-point)
    output: center point in ENU coordinates (`tuple[float, float, float]`)
    """
    center_point = ad.map.lane.getENULanePoint(para_point, ad.physics.ParametricValue(0.5))
    return enu_point_to_tuple(center_point)

def get_adjacent_lane_id(lane_id, side):
    """Return one same-direction adjacent lane on the requested side.

    input: `lane_id` (`int`), `side` (`ad.map.lane.ContactLocation`)
    output: adjacent lane id or nothing (`int | None`)
    """
    for contact_lane in get_lane(lane_id).contact_lanes:
        if contact_lane.location != side:
            continue
        next_lane_id = int(to_base_value(contact_lane.to_lane))
        if next_lane_id <= 0:
            continue
        if not lanes_have_same_direction(lane_id, next_lane_id):
            continue
        if not ad.map.lane.isRouteable(get_lane(next_lane_id)):
            continue
        return next_lane_id
    return None


def get_parallel_lane_ids(center_lane_id):
    """Collect same-direction adjacent lane ids beside one lane.

    input: `center_lane_id` (`int`)
    output: adjacent lane ids (`list[int]`)
    """
    left_lane_ids = []
    visited_lane_ids = {center_lane_id}
    current_lane_id = center_lane_id

    while True:
        next_left_lane_id = get_adjacent_lane_id(current_lane_id, ad.map.lane.ContactLocation.LEFT)
        if next_left_lane_id is None or next_left_lane_id in visited_lane_ids:
            break
        left_lane_ids.append(next_left_lane_id)
        visited_lane_ids.add(next_left_lane_id)
        current_lane_id = next_left_lane_id

    right_lane_ids = []
    current_lane_id = center_lane_id
    while True:
        next_right_lane_id = get_adjacent_lane_id(current_lane_id, ad.map.lane.ContactLocation.RIGHT)
        if next_right_lane_id is None or next_right_lane_id in visited_lane_ids:
            break
        right_lane_ids.append(next_right_lane_id)
        visited_lane_ids.add(next_right_lane_id)
        current_lane_id = next_right_lane_id

    return left_lane_ids + right_lane_ids


def points_are_close(first_point, second_point, tolerance=0.05):
    """Check if two sampled ENU points are basically the same.

    input: `first_point` (`tuple[float, float, float]`), `second_point` (`tuple[float, float, float]`), `tolerance` (`float`)
    output: whether the points are close (`bool`)
    """
    return all((abs(first_value - second_value) <= tolerance for first_value, second_value in zip(first_point, second_point)))


def sample_lane_offsets(lane_id, start_offset, end_offset, spacing):
    """Create evenly spaced offsets between two points on one lane.

    input: `lane_id` (`int`), `start_offset` (`float`), `end_offset` (`float`), `spacing` (`float`)
    output: sampled parametric offsets (`list[float]`)
    """
    lane_length_m = max(get_lane_length_m(lane_id), spacing)
    offset_step = spacing / lane_length_m
    sampled_offsets = [start_offset]

    if lane_is_positive(lane_id):
        current_offset = start_offset + offset_step
        while current_offset < end_offset - 1e-6:
            sampled_offsets.append(current_offset)
            current_offset += offset_step
    else:
        current_offset = start_offset - offset_step
        while current_offset > end_offset + 1e-6:
            sampled_offsets.append(current_offset)
            current_offset -= offset_step

    if abs(sampled_offsets[-1] - end_offset) > 1e-6:
        sampled_offsets.append(end_offset)
    return sampled_offsets


def interpolate_points(first_point, second_point, spacing):
    """Create short straight connector points between two sampled route points.

    input: `first_point` (`tuple[float, float, float]`), `second_point` (`tuple[float, float, float]`), `spacing` (`float`)
    output: interpolated points without the first point (`list[tuple[float, float, float]]`)
    """
    connector_length_m = math.dist(first_point, second_point)
    connector_count = max(2, int(connector_length_m / spacing) + 1)
    return [tuple((first_value + (second_value - first_value) * (step / connector_count) for first_value, second_value in zip(first_point, second_point))) for step in range(1, connector_count + 1)]


def append_sampled_point(sampled_points, sampled_point_info, point, lane_id, parametric_offset):
    """Append one sampled route point while avoiding duplicates.

    input: `sampled_points` (`list[tuple[float, float, float]]`), `sampled_point_info` (`list[dict[str, object]]`), `point` (`tuple[float, float, float]`), `lane_id` (`int`), `parametric_offset` (`float`)
    output: none (`None`)
    """
    if sampled_points and points_are_close(sampled_points[-1], point):
        return
    sampled_points.append(point)
    sampled_point_info.append({'position': point, 'lane_id': lane_id, 'parametric_offset': parametric_offset})


def find_adjacent_points(route, route_points, route_point_index=5):
    """Find nearby same-direction side-lane points around one route sample.

    input: `route` (`dict[str, object]`), `route_points` (`list[tuple[float, float, float]]`), `route_point_index` (`int`)
    output: selected route point and adjacent lane points (`dict`)
    """
    if not route_points:
        raise RuntimeError('The route point list is empty.')
    if 'sampled_point_info' not in route or len(route['sampled_point_info']) != len(route_points):
        raise RuntimeError('Route points must be sampled before adjacent lanes can be queried.')

    selected_route_index = min(max(int(route_point_index), 0), len(route_points) - 1)
    selected_point_info = route['sampled_point_info'][selected_route_index]
    selected_lane_id = selected_point_info['lane_id']
    selected_offset = selected_point_info['parametric_offset']
    adjacent_points = []

    for adjacent_lane_id in get_parallel_lane_ids(selected_lane_id):
        adjacent_points.append({'position': sample_lane_center_at_offset(adjacent_lane_id, selected_offset), **get_opendrive_lane_info(adjacent_lane_id)})

    return {'selected_route_index': selected_route_index, 'selected_route_point': route_points[selected_route_index], 'selected_lane_id': selected_lane_id, 'adjacent_points': adjacent_points}


def sample_route_centerline(route, spacing=1.0, start_point=None, goal_point=None, goal_append_distance_threshold_m=1.0):
    """Sample the final custom route into evenly spaced center points.

    input: `route` (`dict[str, object]`), `spacing` (`float`), `start_point` (`ad.map.point.ENUPoint | None`), `goal_point` (`ad.map.point.ENUPoint | None`), `goal_append_distance_threshold_m` (`float`)
    output: sampled route points (`list[tuple[float, float, float]]`)
    """
    if spacing <= 0.0:
        raise ValueError('Route-point spacing must be greater than zero.')
    if goal_append_distance_threshold_m < 0.0:
        raise ValueError('Goal append distance threshold must be zero or greater.')

    sampled_points = []
    sampled_point_info = []
    for step in route['steps']:
        if step['type'] == 'lane_segment':
            for parametric_offset in sample_lane_offsets(step['lane_id'], step['start_offset'], step['end_offset'], spacing):
                append_sampled_point(sampled_points, sampled_point_info, sample_lane_center_at_offset(step['lane_id'], parametric_offset), step['lane_id'], parametric_offset)
            continue

        connector_start_point = sample_lane_center_at_offset(step['from_lane_id'], step['from_offset'])
        connector_end_point = sample_lane_center_at_offset(step['to_lane_id'], step['to_offset'])
        for connector_point in interpolate_points(connector_start_point, connector_end_point, spacing):
            append_sampled_point(sampled_points, sampled_point_info, connector_point, step['to_lane_id'], step['to_offset'])

    if start_point is not None:
        start_tuple = enu_point_to_tuple(start_point)
        if not sampled_points or not points_are_close(sampled_points[0], start_tuple):
            sampled_points.insert(0, start_tuple)
            sampled_point_info.insert(0, {'position': start_tuple, 'lane_id': route['start_lane_id'], 'parametric_offset': route['start_offset']})
    if goal_point is not None:
        goal_tuple = enu_point_to_tuple(goal_point)
        if not sampled_points or math.dist(sampled_points[-1], goal_tuple) > goal_append_distance_threshold_m:
            sampled_points.append(goal_tuple)
            sampled_point_info.append({'position': goal_tuple, 'lane_id': route['goal_lane_id'], 'parametric_offset': route['goal_offset']})

    route['sampled_point_info'] = sampled_point_info
    print('Route length:', f"{route['route_length_m']:.2f} m")
    print('Expected points:', int(route['route_length_m'] / spacing) + 1)
    print('Generated points:', len(sampled_points))
    return sampled_points

def close_map():
    """Clear the loaded AD-map data after the route work is done.

    input: none (`None`)
    output: none (`None`)
    """
    ad.map.access.cleanup()
