import sys

sys.path.append('../')
import gtpyhop
import pddlgym

path_to_gym = "../../pddlgym"

domain_name = "easyblocks"
the_domain = gtpyhop.Domain(domain_name)

rigid = gtpyhop.State('rigid relations')
rigid.types = {
    'block' : ['a', 'b', 'c', 'd'],
    'robot' : ['robot']
}

initial = gtpyhop.State('state0')
initial.on_table = {
    'a': True,
    'b': True,
    'c': True,
    'd': True
}

initial.clear = {
    'a': True,
    'b': True,
    'c': True,
    'd': True
}

initial.on = {
    'a': None,
    'b': None,
    'c': None,
    'd': None
}

initial.holding = {
    'robot': None
}

# Helper
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

# Actions
def pickup(state, x, robot):
    if is_a(x, 'block') and is_a(robot, 'robot') and  \
    state.clear[x] and state.on_table[x] and state.holding[robot] is None:
        state.on_table[x] = False
        state.clear[x] = False
        state.holding[robot] = x
        return state

def putdown(state, x, robot):
    if is_a(x, 'block') and is_a(robot, 'robot') and \
    state.holding[robot] == x:
        state.holding[robot] = None
        state.clear[x] = True
        state.on_table[x] = True
        return state

def stack(state, x, y, robot):
    if is_a(x, 'block') and is_a(y, 'block') and is_a(robot, 'robot') and \
    state.holding[robot] == x and state.clear[y]:
        state.clear[y] = False
        state.clear[x] = True
        state.on[x] = y
        state.holding[robot] = None
        return state

def unstack(state, x, y, robot):
    if is_a(x, 'block') and is_a(y, 'block') and is_a(robot, 'robot') and \
    state.holding[robot] is None and state.on[x] == y and state.clear[x]:
        state.clear[y] = True
        state.clear[x] = False
        state.on[x] = None
        state.holding[robot] = x
        return state

gtpyhop.declare_actions(pickup, putdown, stack, unstack)

# Commands
def c_pickup(state, x, robot):
    if is_a(x, 'block') and is_a(robot, 'robot') and  \
    state.clear[x] and state.on_table[x] and state.holding[robot] is None:
        state.on_table[x] = False
        state.clear[x] = False
        state.holding[robot] = x
        return state

def c_putdown(state, x, robot):
    if is_a(x, 'block') and is_a(robot, 'robot') and \
    state.holding[robot] == x:
        state.holding[robot] = None
        state.clear[x] = True
        state.on_table[x] = True
        return state

def c_stack(state, x, y, robot):
    if is_a(x, 'block') and is_a(y, 'block') and is_a(robot, 'robot') and \
    state.holding[robot] == x and state.clear[y]:
        state.clear[y] = False
        state.clear[x] = True
        state.on[x] = y
        state.holding[robot] = None
        return state

def c_unstack(state, x, y, robot):
    if is_a(x, 'block') and is_a(y, 'block') and is_a(robot, 'robot') and \
    state.holding[robot] is None and state.on[x] == y and state.clear[x]:
        state.clear[y] = True
        state.clear[x] = False
        state.on[x] = None
        state.holding[robot] = x
        return state

gtpyhop.declare_commands(c_pickup, c_putdown, c_stack, c_unstack)

# Methods
def move_to_table(state, x, robot):
    if is_a(x, 'block') and is_a(robot, 'robot'):
        if state.on_table[x]:
            return []
        elif state.on[x] != None and state.clear[x] and state.holding[robot] is None:
            return [('unstack', x, state.on[x], robot), ('putdown', x, robot)]
        
def move_to_block(state, x, y):
    if is_a(x, 'block') and is_a(y, 'block'):
        for robot in rigid.types['robot']:
            if state.on[x] == y:
                return []
            elif state.clear[x] and state.holding[robot] is None:
                if state.on_table[x]:
                    return [('pickup', x, robot), ('stack', x, y, robot)]
                elif state.on[x] != None:
                    return[('unstack', x, state.on[x], robot), ('stack', x, y, robot)]
            
gtpyhop.declare_unigoal_methods('on', move_to_block)
gtpyhop.declare_unigoal_methods('on_table', move_to_table)

gtpyhop.declare_multigoal_methods(gtpyhop.m_split_multigoal)

# Implementing a lazy lookahead using GTPyhop
def main():
    print("Domain: ", domain_name)

    # The goals must be in this order due to the lazy multigoal splitting
    goals = [('on', 'b', 'a'), ('on', 'c', 'b'), ('on', 'd', 'c')]
    print("Goals: ", str(goals).replace('\'', ""))

    # Change to 1-3 if you want more detailed HTN-planning output
    gtpyhop.verbose = 0
    result = gtpyhop.find_plan(initial, goals)
    print("Plan: ", str(result).replace('\'', ""))

    # set up PDDLGym environment and observation
    env = pddlgym.core.PDDLEnv(path_to_gym + "/pddlgym/pddl/easyblocks.pddl", 
                               path_to_gym + "/pddlgym/pddl/easyblocks/")
    env.fix_problem_index(0)
    obs, debuginfo = env.reset()

    # going through actions in GTPyhop plan
    for act in result:
        pred = None
        # PDDLGym uses custom predicate structs we need to match
        # This works here because the action methods above are named to match
        # the PDDL predicates
        for predicate in env.action_predicates:
            if predicate.name == act[0]:
                pred = predicate
                break
        
        # PDDLGym expects grounded actions to be Literal structs
        act = pddlgym.structs.Literal(pred, list(act[1:]))
        
        obs, reward, done, truncated, debug_info = env.step(act)

        # To replan based on the returned observation, need to convert from a 
        # PDDLGym State object (domain-dependent implementation)
        # Will need a default state to edit, since obs defines the whole state, 
        # not just the changes from the action. 
        # See the _state_to_internal method in custom/searchandrescue.py

    print("Final obs: ", obs)
    print("Reward: ", reward)
    print("Done: ", done)


if __name__ == '__main__':
    main()