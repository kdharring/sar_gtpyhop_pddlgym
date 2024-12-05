import sar_problem1, sar_problem2, sar_domain, pddl_sar_plan
import matplotlib.pyplot as plt

def create_plans(problem_num=0):
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
        gym_prob = 45
    elif problem_num == 1:
        gt_initial = sar_problem2.initial
        gt_rigid = sar_problem2.rigid
        sar_domain.rigid = gt_rigid
        goal = sar_problem2.goal
        gym_prob = 50
    
    gt_errors = 0

    while results.get('gt') is None:
        try:
            gt_plan_len, gt_nodes_exp = sar_domain.create_plan(gt_initial, goal)
            results['gt'] = (gt_plan_len, gt_nodes_exp)
        except RecursionError:
            print("GTPyhop Recursion Error")
            gt_errors += 1

    ff_plan_len, ff_nodes_exp = pddl_sar_plan.create_plan_ff(gym_prob)
    results['ff'] = (ff_plan_len, ff_nodes_exp)

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

    return results, gt_errors

def run_acting(problem_num=0):
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
        gym_prob = 45

    lazy_gt_results = sar_domain.run_lazy_lookahead(gt_initial, goal, gym_prob)
    results['lazy_gt'] = lazy_gt_results[:2]
    run_gt_results = sar_domain.run_lookahead(gt_initial, goal, gym_prob)
    results['run_gt'] = run_gt_results[:2]
    gt_recurse_errors = lazy_gt_results[2] + run_gt_results[2]

    results['lazy_ff'] = pddl_sar_plan.run_lazy_lookahead_ff(gym_prob)
    results['run_ff'] = pddl_sar_plan.run_lookahead_ff(gym_prob)

    results['lazy_fd_blind'] = pddl_sar_plan.run_lazy_lookahead_fd(
        gym_prob, '--search "astar(blind())"')
    results['run_fd_blind'] = pddl_sar_plan.run_lookahead_fd(
        gym_prob, '--search "astar(blind())"')
    
    results['lazy_fd_lmcut'] = pddl_sar_plan.run_lazy_lookahead_fd(
        gym_prob, '--search "astar(lmcut())"')
    results['run_fd_lmcut'] = pddl_sar_plan.run_lookahead_fd(
        gym_prob, '--search "astar(lmcut())"')
    
    results['lazy_fd_ff'] = pddl_sar_plan.run_lazy_lookahead_fd(
        gym_prob, 
        '--evaluator "hff=ff()" --search "lazy_greedy([hff])"')
    results['run_fd_ff'] = pddl_sar_plan.run_lookahead_fd(
        gym_prob, 
        '--evaluator "hff=ff()" --search "lazy_greedy([hff])"')
    
    results['lazy_fd_cea'] = pddl_sar_plan.run_lazy_lookahead_fd(
        gym_prob, 
        '--evaluator "hcea=cea()" --search "lazy_greedy([hcea])"')
    results['run_fd_cea'] = pddl_sar_plan.run_lookahead_fd(
        gym_prob, 
        '--evaluator "hcea=cea()" --search "lazy_greedy([hcea])"',)

    return results, gt_recurse_errors
    

def main():
    num_tests = 3
    expansions = {}
    plan_lengths = {}
    act_lengths = {}
    act_times = {}

    total_gt_errors = 0

    for _ in range(num_tests):
        plan_results, gt_errors = create_plans()

        total_gt_errors += gt_errors
        
        for key in plan_results.keys():
            plan_lengths[key] = plan_lengths.get(key, []) + [plan_results[key][0]]
            expansions[key] = expansions.get(key, []) + [plan_results[key][1]]

        act_results, gt_errors = run_acting()
        total_gt_errors += gt_errors
        for key in act_results.keys():
            act_lengths[key] = act_lengths.get(key, []) + [act_results[key][1]]
            act_times[key] = act_times.get(key, []) + [act_results[key][0]]
        

    fig, axes = plt.subplots(2, 2, figsize=(8, 6), tight_layout=True)

    print("Total GT recursions errors: ", total_gt_errors)
    
    axes[0, 0].set_ylabel('Node expansions')
    axes[0, 0].set_title(f'Node expansions over Methods (N={num_tests})')
    axes[0, 0].boxplot(expansions.values(), tick_labels=expansions.keys())
    axes[0, 0].set_yscale('log')
    axes[0, 0].tick_params(axis='y', which='minor')

    axes[0, 1].set_ylabel('Plan Path Lengths')
    axes[0, 1].set_title(f'Plan Path Lengths over Methods (N={num_tests})')
    axes[0, 1].boxplot(plan_lengths.values(), tick_labels=plan_lengths.keys())
    axes[0, 1].set_yscale('log')
    axes[0, 1].tick_params(axis='y', which='minor')

    axes[1, 0].set_ylabel('Acting Path Lengths')
    axes[1, 0].set_title(f'Acting Path Lengths over Methods (N={num_tests})')
    axes[1, 0].boxplot(act_lengths.values(), tick_labels=act_lengths.keys())
    axes[1, 0].set_yscale('log')
    axes[1, 0].tick_params(axis='x', labelrotation=90)
    axes[1, 0].tick_params(axis='y', which='minor')

    axes[1, 1].set_ylabel('Acting times (msec)')
    axes[1, 1].set_title(f'Acting Times over Methods (N={num_tests})')
    axes[1, 1].boxplot(act_times.values(), tick_labels=act_times.keys())
    axes[1, 1].set_yscale('log')
    axes[1, 1].tick_params(axis='x', labelrotation=90)
    axes[1, 1].tick_params(axis='y', which='minor')

    fig.suptitle("SearchAndRescue-7 Problem 0 Evaluation")

    plt.show()

if __name__ == '__main__':
    main()