import sar_problem1, sar_problem2, sar_problem3, sar_domain, pddl_sar_plan
import matplotlib.pyplot as plt, threading, json, gc
from pddlgym_planners.planner import PlanningFailure, PlanningTimeout

def create_plans_gt(problem_num=0):
    results = {}

    if problem_num == 0:
        gt_initial = sar_problem1.initial
        gt_rigid = sar_problem1.rigid
        sar_domain.rigid = gt_rigid
        goal = sar_problem1.goal
        gym_prob = 0
    elif problem_num == 1:
        gt_initial = sar_problem2.initial
        gt_rigid = sar_problem2.rigid
        sar_domain.rigid = gt_rigid
        goal = sar_problem2.goal
        gym_prob = 40
    elif problem_num == 2:
        gt_initial = sar_problem3.initial
        gt_rigid = sar_problem3.rigid
        sar_domain.rigid = gt_rigid
        goal = sar_problem3.goal
        gym_prob = 46
    
    gt_errors = 0

    while results.get('gt') is None:
        try:
            gt_plan_len, gt_nodes_exp = sar_domain.create_plan(gt_initial, goal,)
            results['gt'] = (gt_plan_len, gt_nodes_exp)
        except RecursionError:
            print("GTPyhop Recursion Error")
            gt_errors += 1

    return results, gt_errors

def create_plans_ff(problem_num=0):
    results = {}

    if problem_num == 0:
        gym_prob = 0
    elif problem_num == 1:
        gym_prob = 40
    elif problem_num == 2:
        gym_prob = 46

    ff_plan_len, ff_nodes_exp = pddl_sar_plan.create_plan_ff(gym_prob)
    results['ff'] = (ff_plan_len, ff_nodes_exp)

    return results

def create_plans_fd(problem_num=0):
    results = {}

    if problem_num == 0:
        gym_prob = 0
    elif problem_num == 1:
        gym_prob = 40
    elif problem_num == 2:
        gym_prob = 46

    fd_blind_plan_len, fd_blind_nodes_exp = pddl_sar_plan.create_plan_fd(
        gym_prob, '--search "astar(blind())"')
    results['fd_blind'] = (fd_blind_plan_len, fd_blind_nodes_exp)
    
    fd_lmcut_plan_len, fd_lmcut_nodes_exp = pddl_sar_plan.create_plan_fd(
        gym_prob, '--search "astar(lmcut())"')
    results['fd_lmcut'] = (fd_lmcut_plan_len, fd_lmcut_nodes_exp)

    fd_ff_plan_len, fd_ff_nodes_exp = pddl_sar_plan.create_plan_fd(
        gym_prob, '--evaluator "hff=ff()" --search "lazy_greedy([hff])"')
    results['fd_ff'] = (fd_ff_plan_len, fd_ff_nodes_exp)
    
    fd_add_plan_len, fd_add_nodes_exp = pddl_sar_plan.create_plan_fd(
        gym_prob, '--evaluator "hcea=cea()" --search "lazy_greedy([hcea])"')
    results['fd_cea'] = (fd_add_plan_len, fd_add_nodes_exp)

    return results

def run_acting_gt(problem_num=0):
    results = {}

    if problem_num == 0:
        gt_initial = sar_problem1.initial
        gt_rigid = sar_problem1.rigid
        sar_domain.rigid = gt_rigid
        goal = sar_problem1.goal
        gym_prob = 0
    elif problem_num == 1:
        gt_initial = sar_problem2.initial
        gt_rigid = sar_problem2.rigid
        sar_domain.rigid = gt_rigid
        goal = sar_problem2.goal
        gym_prob = 40
    elif problem_num == 2:
        gt_initial = sar_problem3.initial
        gt_rigid = sar_problem3.rigid
        sar_domain.rigid = gt_rigid
        goal = sar_problem3.goal
        gym_prob = 46

    lazy_gt_results = sar_domain.run_lazy_lookahead(gt_initial, goal, prob_idx=gym_prob)
    results['lazy_gt'] = lazy_gt_results[:2]
    run_gt_results = sar_domain.run_lookahead(gt_initial, goal, prob_idx=gym_prob)
    results['run_gt'] = run_gt_results[:2]
    gt_recurse_errors = lazy_gt_results[2] + run_gt_results[2]

    return results, gt_recurse_errors

def run_acting_ff(problem_num=0):
    results = {}

    if problem_num == 0:
        gym_prob = 0
    elif problem_num == 1:
        gym_prob = 40
    elif problem_num == 2:
        gym_prob = 46

    results['lazy_ff'] = pddl_sar_plan.run_lazy_lookahead_ff(gym_prob)
    results['run_ff'] = pddl_sar_plan.run_lookahead_ff(gym_prob)

    return results

def run_acting_fd(problem_num=0):
    results = {}

    if problem_num == 0:
        gym_prob = 0
    elif problem_num == 1:
        gym_prob = 40
    elif problem_num == 2:
        gym_prob = 46

    fd_failures = 0

    try:
        results['lazy_fd_blind'] = pddl_sar_plan.run_lazy_lookahead_fd(
            gym_prob, '--search "astar(blind())"')
    except (PlanningTimeout, PlanningFailure):
        print("FD Failure")
        fd_failures += 1
    
    try:
        results['run_fd_blind'] = pddl_sar_plan.run_lookahead_fd(
            gym_prob, '--search "astar(blind())"')
    except (PlanningTimeout, PlanningFailure):
        print("FD Failure")
        fd_failures += 1
    
    try:
        results['lazy_fd_lmcut'] = pddl_sar_plan.run_lazy_lookahead_fd(
            gym_prob, '--search "astar(lmcut())"')
    except (PlanningTimeout, PlanningFailure):
        print("FD Failure")
        fd_failures += 1

    try:
        results['run_fd_lmcut'] = pddl_sar_plan.run_lookahead_fd(
            gym_prob, '--search "astar(lmcut())"')
    except (PlanningTimeout, PlanningFailure):
        print("FD Failure")
        fd_failures += 1
    
    try:
        results['lazy_fd_ff'] = pddl_sar_plan.run_lazy_lookahead_fd(
            gym_prob, 
            '--evaluator "hff=ff()" --search "lazy_greedy([hff])"')
    except (PlanningTimeout, PlanningFailure):
        print("FD Failure")
        fd_failures += 1

    try:
        results['run_fd_ff'] = pddl_sar_plan.run_lookahead_fd(
            gym_prob, 
            '--evaluator "hff=ff()" --search "lazy_greedy([hff])"')
    except (PlanningTimeout, PlanningFailure):
        print("FD Failure")
        fd_failures += 1
    
    try:
        results['lazy_fd_cea'] = pddl_sar_plan.run_lazy_lookahead_fd(
            gym_prob, 
            '--evaluator "hcea=cea()" --search "lazy_greedy([hcea])"')
    except (PlanningTimeout, PlanningFailure):
        print("FD Failure")
        fd_failures += 1

    try:
        results['run_fd_cea'] = pddl_sar_plan.run_lookahead_fd(
            gym_prob, 
            '--evaluator "hcea=cea()" --search "lazy_greedy([hcea])"')
    except (PlanningTimeout, PlanningFailure):
        print("FD Failure")
        fd_failures += 1

    return results, fd_failures

def eval_gt(num_tests):
    problem_results = {}

    for problem in range(3):
        results = {}
        results['expansions'] = {}
        results['plan_lengths'] = {}
        results['act_lengths'] = {}
        results['act_times'] = {}
        results['total_gt_errors'] = 0

        for _ in range(num_tests):
            gc.collect()
            
            plan_results, gt_errors = create_plans_gt(problem_num=problem)

            results['total_gt_errors'] += gt_errors
            
            for key in plan_results.keys():
                results['plan_lengths'][key] = results['plan_lengths'].get(key, []) + [plan_results[key][0]]
                results['expansions'][key] = results['expansions'].get(key, []) + [plan_results[key][1]]

            act_results, gt_errors = run_acting_gt(problem_num=problem)
            results['total_gt_errors'] += gt_errors
            for key in act_results.keys():
                results['act_lengths'][key] = results['act_lengths'].get(key, []) + [act_results[key][1]]
                results['act_times'][key] = results['act_times'].get(key, []) + [act_results[key][0]]

        problem_results[problem] = results
        print(f"GTPythop Done Problem {problem + 1}/3")

    with open('gtpyhop_eval_results.json', 'w') as results_file:
        json.dump(problem_results, results_file)

    gc.collect()

def eval_ff(num_tests):
    problem_results = {}

    for problem in range(3):
        results = {}
        results['expansions'] = {}
        results['plan_lengths'] = {}
        results['act_lengths'] = {}
        results['act_times'] = {}

        for _ in range(num_tests):
            gc.collect()
            
            plan_results = create_plans_ff(problem_num=problem)
            for key in plan_results.keys():
                results['plan_lengths'][key] = results['plan_lengths'].get(key, []) + [plan_results[key][0]]
                results['expansions'][key] = results['expansions'].get(key, []) + [plan_results[key][1]]

            act_results = run_acting_ff(problem_num=problem)
            for key in act_results.keys():
                results['act_lengths'][key] = results['act_lengths'].get(key, []) + [act_results[key][1]]
                results['act_times'][key] = results['act_times'].get(key, []) + [act_results[key][0]]

        problem_results[problem] = results
        print(f"FF Done Problem {problem + 1}/3")

    with open('ff_eval_results.json', 'w') as results_file:
        json.dump(problem_results, results_file)

    gc.collect()

def eval_fd(num_tests):
    problem_results = {}

    for problem in range(3):
        results = {}
        results['expansions'] = {}
        results['plan_lengths'] = {}
        results['act_lengths'] = {}
        results['act_times'] = {}
        results['total_fd_errors'] = 0

        for _ in range(num_tests):
            gc.collect()
            
            plan_results = create_plans_fd(problem_num=problem)
            
            for key in plan_results.keys():
                results['plan_lengths'][key] = results['plan_lengths'].get(key, []) + [plan_results[key][0]]
                results['expansions'][key] = results['expansions'].get(key, []) + [plan_results[key][1]]

            act_results, fd_failures = run_acting_fd(problem_num=problem)
            for key in act_results.keys():
                results['act_lengths'][key] = results['act_lengths'].get(key, []) + [act_results[key][1]]
                results['act_times'][key] = results['act_times'].get(key, []) + [act_results[key][0]]

            results['total_fd_errors'] += fd_failures

        problem_results[problem] = results
        print(f"FD Done Problem {problem + 1}/3")

    with open('fd_eval_results.json', 'w') as results_file:
        json.dump(problem_results, results_file)

    gc.collect()

def main():
    num_tests = 5

    gt_thread = threading.Thread(target=eval_gt, args=(num_tests,))  
    ff_thread = threading.Thread(target=eval_ff, args=(num_tests,))
    fd_thread = threading.Thread(target=eval_fd, args=(num_tests,))

    threads = [gt_thread, ff_thread, fd_thread]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

    gc.collect()

if __name__ == '__main__':
    main()