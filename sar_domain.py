import sys, cv2, imageio, pddlgym, random, time, numpy as np, re

from GTPyhop import gtpyhop

expansions = 0

domain_name = "searchandrescue"
domain = gtpyhop.Domain(gtpyhop)

rigid = gtpyhop.State('rigid relations')
# add types for problem-specific objects in problem file
rigid.types = {
    'direction' : ['up', 'down', 'left', 'right'],
    'person' : [],
    'hospital' : [],
    'wall' : [],
    'robot' : [],
    'location' : [],
    'chicken' : []
}

rigid.relations = {
    'connected' : {},
    'hospital-at' : {},
    'wall-at' : {},
    'chicken-at' : {},
}

initial = gtpyhop.State('state0')
initial.person_at = {}
initial.robot_at = {}
initial.carrying = {}
initial.clear = {}

# Helper from GTPyhop examples. Credit: Dana S. Nau
def is_a(variable,type):
    """
    In most classical planners, one would declare data-types for the parameters
    of each action, and the data-type checks would be done by the planner.
    GTPyhop doesn't have a way to do that, so the 'is_a' function gives us a
    way to do it in the preconditions of each action, command, and method.
    
    'is_a' doesn't implement subtypes (e.g., if rigid.type[x] = y and
    rigid.type[x] = z, it doesn't infer that rigid.type[x] = z. It wouldn't be
    hard to implement this, but it isn't needed in the simple-travel domain.
    """
    return variable in rigid.types[type]

################################################################################
# Actions
################################################################################

# compound names are combined to make connection to PDDLGym easier
def move(state, rob, start, dest, dir):
    if is_a(rob, 'robot') and is_a(start, 'location') and is_a(dest, 'location') and \
    is_a(dir, 'direction'):
        if rigid.relations['connected'][start].get(dir) == dest and \
        state.robot_at[rob] == start and state.clear[dest]:
            state.clear[start] = True
            state.clear[dest] = False
            state.robot_at[rob] = dest
            return state

def pickup(state, rob, person, loc):
    if is_a(rob, 'robot') and is_a(person, 'person') and is_a(loc, 'location'):
        if state.robot_at[rob] == loc and state.person_at[person] == loc and \
        state.carrying[rob] is None:
            state.carrying[rob] = person
            return state

def dropoff(state, rob, person, loc):
    if is_a(rob, 'robot') and is_a(person, 'person') and is_a(loc, 'location'):
        if state.robot_at[rob] == loc and state.carrying[rob] == person:
            state.person_at[person] = loc
            state.carrying[rob] = None
            return state
        
gtpyhop.declare_actions(move, pickup, dropoff)

################################################################################
# Commands (doing same thing as actions)
################################################################################

def c_move(state, rob, start, dest, dir):
    if is_a(rob, 'robot') and is_a(start, 'location') and is_a(dest, 'location') and \
    is_a(dir, 'direction'):
        if rigid.relations['connected'][start].get(dir) == dest and \
        state.robot_at[rob] == start and state.clear[dest]:
            state.clear[start] = True
            state.clear[dest] = False
            state.robot_at[rob] = dest
            return state

def c_pickup(state, rob, person, loc):
    if is_a(rob, 'robot') and is_a(person, 'person') and is_a(loc, 'location'):
        if state.robot_at[rob] == loc and state.person_at[person] == loc and \
        state.carrying[rob] is None:
            state.carrying[rob] = person
            return state

def c_dropoff(state, rob, person, loc):
    if is_a(rob, 'robot') and is_a(person, 'person') and is_a(loc, 'location'):
        if state.robot_at[rob] == loc and state.carrying[rob] == person:
            state.person_at[person] = loc
            state.carrying[rob] = None
            return state
        
gtpyhop.declare_commands(c_move, c_pickup, c_dropoff)

################################################################################
# Methods
################################################################################

# helper method to determine direction of adjacency between locations
def adjacent(curr_loc, dest):
    if rigid.relations['connected'][curr_loc].get('up') == dest:
        return 'up'
    elif rigid.relations['connected'][curr_loc].get('down') == dest:
        return 'down'
    elif rigid.relations['connected'][curr_loc].get('left') == dest:
        return 'left'
    elif rigid.relations['connected'][curr_loc].get('right') == dest:
        return 'right'
    
    return None

loc_reg = re.compile("f(\d+)-(\d+)f")

def best_dir(state, curr, dest):
    lowest_dist = np.inf
    best_dirs = []

    for dir in rigid.types['direction']:
        if rigid.relations['connected'][curr].get(dir) is None \
            or not state.clear[rigid.relations['connected'][curr].get(dir)]:
            continue

        new_loc = rigid.relations['connected'][curr][dir]

        search1 = loc_reg.search(new_loc)
        search2 = loc_reg.search(dest)

        # Euclidean distance heuristic
        dist = np.sqrt((int(search1.group(1)) - int(search2.group(1))) ** 2 + \
                    (int(search1.group(2)) - int(search2.group(2))) ** 2)
        
        if dist < lowest_dist:
            lowest_dist = dist
            best_dirs = [dir]
        elif dist == lowest_dist:
            best_dirs.append(dir)
        
    return best_dirs


def navigate(state, rob, dest):
    if is_a(rob, 'robot') and is_a(dest, 'location') and state.clear[dest]:
        curr_loc = state.robot_at[rob]
        adj = adjacent(curr_loc, dest)

        if curr_loc == dest:
            return []
        elif not adj is None:
            return [('move', rob, curr_loc, dest, adj)]
        else:
            result = []

            while adj is None:
                dirs = best_dir(state, curr_loc, dest)

                if len(dirs) == 1:
                    dir = dirs[0]
                else:
                    dir = random.choice(dirs)

                new_loc = rigid.relations['connected'][curr_loc][dir]
                result.append(('move', rob, curr_loc, new_loc, dir))

                curr_loc = new_loc
                adj = adjacent(curr_loc, dest)

            result.append(('move', rob, curr_loc, dest, adj))

            return result
        
def pickup_person(state, rob, person):
    if is_a(rob, 'robot') and is_a(person, 'person'):
        person_loc = state.person_at[person]
        rob_loc = state.robot_at[rob]

        if state.carrying[rob] == person:
            return []
        elif state.carrying[rob] == None and person_loc == rob_loc:
            return [('pickup', rob, person, person_loc)]
        elif state.carrying[rob] == None:
            nav_plan = navigate(state, rob, person_loc)
            return nav_plan + [('pickup', rob, person, person_loc)]
        
def put_at_loc(state, rob, person, loc):
    if is_a(rob, 'robot') and is_a(person, 'person') and is_a(loc, 'location'):
        rob_loc = state.robot_at[rob]
        person_loc = state.person_at[person]

        if person_loc == loc:
            return []
        elif state.carrying[rob] == person:
            nav_plan = navigate(state, rob, loc)
            return nav_plan + [('dropoff', rob, person, loc)]
        
def take_to_loc(state, person, loc):
    if is_a(person, 'person') and is_a(loc, 'location'):
        for rob in rigid.types['robot']:
            rob_loc = state.robot_at[rob]
            person_loc = state.person_at[person]

            if person_loc == loc: 
                return []
            elif state.carrying[rob] == person:
                return put_at_loc(state, rob, person, loc)
            else:
                pickup_plan = pickup_person(state, rob, person)

                at_person_state = state.copy()
            
                at_person_state.robot_at[rob] = person_loc
                at_person_state.clear[rob_loc] = True
                at_person_state.clear[person_loc] = False
                at_person_state.carrying[rob] = person

                put_plan = put_at_loc(at_person_state, rob, person, 
                                      loc)

                return pickup_plan + put_plan
        
gtpyhop.declare_unigoal_methods('person_at', take_to_loc)
gtpyhop.declare_unigoal_methods('carrying', pickup_person)
gtpyhop.declare_unigoal_methods('robot_at', navigate)

# split multi-goal. Order of person-at does not matter, but carrying 
# must be after all person-at goals and robot-at must be last
def sar_split_multigoal(multigoal):
    robot_at = None
    carrying = None

    for i, goal in enumerate(multigoal):
        if 'robot_at' in goal:
            if robot_at is None:
                robot_at = i
            else:
                raise ValueError("Cannot have multiple robot-at goals")
        elif 'carrying' in goal:
            if carrying is None:
                carrying = i
            else:
                raise ValueError("Cannot have multiple carrying goals")
            
    if robot_at != None:
        temp = multigoal[-1]
        multigoal[-1] = multigoal[robot_at]
        multigoal[robot_at] = temp
            
        if carrying != None:
            temp = multigoal[-2]
            multigoal[-2] = multigoal[carrying]
            multigoal[carrying] = temp

    elif carrying != None:
        temp = multigoal[-1]
        multigoal[-1] = multigoal[carrying]
        multigoal[carrying] = temp

gtpyhop.declare_multigoal_methods(sar_split_multigoal)

def coord_to_loc(coord):
    x, y = coord
    return f'f{x}-{y}f'

def create_plan(initial, goals):
    gtpyhop.verbose = 0
    plan = gtpyhop.find_plan(initial, goals)
    return len(plan), len(plan)

def run_lazy_lookahead(initial, goals, prob_idx=0, 
                       render=False, result_print=False):
    gtpyhop.verbose = 0
    start = time.time_ns()
    plan_length = 0
    recurse_errors = 0

    plan = None
    while plan is None:
        try:
            plan = gtpyhop.find_plan(initial, goals)
        except RecursionError:
            recurse_errors += 1

    # set up PDDLGym environment and observation
    env = pddlgym.make('SearchAndRescueLevel7-v0')
    env.fix_problem_index(prob_idx)
    obs, debuginfo = env.reset()

    new_state = initial.copy()

    # lazy lookahead, knowing actions can't fail
    while plan != []:
        act = plan.pop(0)
        pred = None
        # PDDLGym uses custom predicate structs we need to match
        for predicate in env.action_predicates:
            if predicate.name == act[0]:
                pred = predicate
                break

        variables = list(act[1:])
        if pred.name == 'move':
            variables = [act[-1]]
        elif pred.name == 'pickup':
            variables = [act[2]]
        elif pred.name == 'droppoff':
            variables = [act[2]]

        # PDDLGym expects grounded actions to be Literal structs
        gym_act = pddlgym.structs.Literal(pred, variables)
        obs, reward, done, truncated, debug_info = env.step(gym_act)

        if render:
            img = env.render()

            cv2.imshow("Search And Rescue Render", img)
            cv2.waitKey(1)

        plan_length += 1

    end = time.time_ns()

    cv2.destroyAllWindows()

    if result_print:
        print("Final obs: ", obs)
        print("Reward: ", reward)
        print("Done: ", done)


    return (end - start) // 1000000, plan_length, recurse_errors

def run_lookahead(initial, goals, rigid, prob_idx=0, 
                  render=False, result_print=False):
    gtpyhop.verbose = 0
    start = time.time_ns()
    plan_length = 0
    recurse_errors = 0

    plan = None
    while plan is None:
        try:
            plan = gtpyhop.find_plan(initial, goals)
        except RecursionError:
            recurse_errors += 1

    # set up PDDLGym environment and observation
    env = pddlgym.make('SearchAndRescueLevel7-v0')
    env.fix_problem_index(prob_idx)
    obs, debuginfo = env.reset()
    img = env.render()

    new_state = initial.copy()

    imageio.imwrite("start_state.png", img)

    last_plan = 0

    while plan != []:
        act = plan.pop(0)
        last_plan += 1
        pred = None
        # PDDLGym uses custom predicate structs we need to match
        for predicate in env.action_predicates:
            if predicate.name == act[0]:
                pred = predicate
                break

        variables = list(act[1:])
        if pred.name == 'move':
            variables = [act[-1]]
        elif pred.name == 'pickup':
            variables = [act[2]]
        elif pred.name == 'droppoff':
            variables = [act[2]]

        # PDDLGym expects grounded actions to be Literal structs
        gym_act = pddlgym.structs.Literal(pred, variables)
        obs, reward, done, truncated, debug_info = env.step(gym_act)

        if render:
            img = env.render()

            cv2.imshow("Search And Rescue Render", img)
            cv2.waitKey(1)
        
        for literal in obs:
            if literal[0] == 'carrying':
                if new_state.carrying['robot0'] == None:
                    imageio.imwrite("pickup_state.png", img)
                new_state.carrying['robot0'] = literal[1]
            if literal[0] in rigid.types['person']:
                new_state.person_at[literal[0]] = coord_to_loc(literal[1])
            if literal[0] in rigid.types['robot']:
                new_state.robot_at[literal[0]] = coord_to_loc(literal[1])

        if not done:
            new_plan = None
            while new_plan is None:
                try:
                    new_plan = gtpyhop.find_plan(new_state, goals)
                except RecursionError:
                    recurse_errors += 1

            if len(new_plan) < len(plan):
                plan = new_plan
        
        plan_length += 1

    end = time.time_ns()

    cv2.destroyAllWindows()

    if result_print:
        print("Final obs: ", obs)
        print("Reward: ", reward)
        print("Done: ", done)

    return (end - start) // 1000000, plan_length, recurse_errors