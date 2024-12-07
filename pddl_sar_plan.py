# Must install bison and flex for ff.FF to run
import matplotlib; matplotlib.use('agg')
import pddlgym, cv2, time, numpy as np
from pddlgym_planners import ff, fd

def create_plan_ff(prob_idx):
    env = pddlgym.make('SearchAndRescueLevel7-v0')
    env.fix_problem_index(prob_idx)

    _, debuginfo = env.reset()

    obs = env.get_state()
    # the constants are also defined in the state, but the planner will complain
    env.domain.constants = []

    planner = ff.FF()
    plan = planner(env.domain, obs)

    stats = planner.get_statistics()

    return len(plan), stats["num_node_expansions"]

def create_plan_fd(prob_idx, final_flags):
    env = pddlgym.make('SearchAndRescueLevel7-v0')
    env.fix_problem_index(prob_idx)

    _, debuginfo = env.reset()

    obs = env.get_state()
    # the constants are also defined in the state, but the planner will complain
    env.domain.constants = []

    planner = fd.FD(alias_flag='', final_flags=final_flags)
    plan = planner(env.domain, obs)

    stats = planner.get_statistics()

    return len(plan), stats["num_node_expansions"]

def run_lazy_lookahead_fd(prob_idx, final_flags, render=False, 
                          result_print=False):
    # must use level 7 since that is the only ones with chickens present
    # FastForward will only work if all predicates have at least one 
    # instantiation
    start = time.time_ns()
    plan_length = 0

    env = pddlgym.make('SearchAndRescueLevel7-v0')
    env.fix_problem_index(prob_idx)
    _, debuginfo = env.reset()

    # will automatically install planner if not already installed
    planner = fd.FD(alias_flag='', final_flags=final_flags)
    # the default state is a tuple, but we need a State object
    obs = env.get_state()
    new_domain = env.domain
    # the constants are also defined in the state, but the planner will complain
    new_domain.constants = []
    plan = planner(env.domain, obs)

    while plan != []:
        act = plan.pop(0)
        tup_obs, reward, done, truncated, debug_info = env.step(act)
        obs = env.get_state()

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


    return (end - start) // 1000000, plan_length

def run_lazy_lookahead_ff(prob_idx, render=False, result_print=False):
    # must use level 7 since that is the only ones with chickens present
    # FastForward will only work if all predicates have at least one 
    # instantiation
    start = time.time_ns()
    plan_length = 0

    env = pddlgym.make('SearchAndRescueLevel7-v0')
    env.fix_problem_index(prob_idx)
    _, debuginfo = env.reset()

    # will automatically install planner if not already installed
    planner = ff.FF()
    # the default state is a tuple, but we need a State object
    obs = env.get_state()
    new_domain = env.domain
    # the constants are also defined in the state, but the planner will complain
    new_domain.constants = []
    plan = planner(env.domain, obs)

    while plan != []:
        act = plan.pop(0)
        tup_obs, reward, done, truncated, debug_info = env.step(act)
        obs = env.get_state()

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


    return (end - start) // 1000000, plan_length

def run_lookahead_fd(prob_idx, final_flags, render=False, result_print=False):
    # must use level 7 since that is the only ones with chickens present
    # FastForward will only work if all predicates have at least one 
    # instantiation
    start = time.time_ns()
    plan_length = 0

    env = pddlgym.make('SearchAndRescueLevel7-v0')
    env.fix_problem_index(prob_idx)
    _, debuginfo = env.reset()

    # will automatically install planner if not already installed
    planner = fd.FD(alias_flag='', final_flags=final_flags)
    # the default state is a tuple, but we need a State object
    
    obs = env.get_state()
    # the constants are also defined in the state, but the planner will complain
    env.domain.constants = []
    plan = planner(env.domain, obs)

    while plan != []:
        act = plan.pop(0)
        tup_obs, reward, done, truncated, debug_info = env.step(act)
        obs = env.get_state()

        if not done:
            plan = planner(env.domain, obs)

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


    return (end - start) // 1000000, plan_length

def run_lookahead_ff(prob_idx, render=False, result_print=False):
    # must use level 7 since that is the only ones with chickens present
    # FastForward will only work if all predicates have at least one 
    # instantiation
    start = time.time_ns()
    plan_length = 0

    env = pddlgym.make('SearchAndRescueLevel7-v0')
    env.fix_problem_index(prob_idx)
    _, debuginfo = env.reset()

    # will automatically install planner if not already installed
    planner = ff.FF()
    # the default state is a tuple, but we need a State object
    obs = env.get_state()
    new_domain = env.domain
    # the constants are also defined in the state, but the planner will complain
    env.domain.constants = []
    plan = planner(env.domain, obs)

    while plan != []:
        act = plan.pop(0)
        tup_obs, reward, done, truncated, debug_info = env.step(act)
        obs = env.get_state()

        if not done:
            plan = planner(env.domain, obs)

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


    return (end - start) // 1000000, plan_length

if __name__ == '__main__':
    run_lookahead_fd(46, '--search "astar(blind())"', render=True)