import matplotlib.pyplot as plt, json
def main():
    with open('gtpyhop_eval_results.json', 'r') as file:
        gt_results = json.load(file)
    with open('ff_eval_results.json', 'r') as file:
        ff_results = json.load(file)
    with open('fd_eval_results.json', 'r') as file:
        fd_results = json.load(file)

    num_tests = 5

    for res_num in gt_results.keys():

        fig, axes = plt.subplots(2, 2, figsize=(8, 6), tight_layout=True)

        total_gt_errors = gt_results[res_num]['total_gt_errors']
        total_fd_errors = fd_results[res_num]['total_fd_errors']

        expansions = gt_results[res_num]['expansions']
        expansions.update(ff_results[res_num]['expansions'])
        expansions.update(fd_results[res_num]['expansions'])

        plan_lengths = gt_results[res_num]['plan_lengths']
        plan_lengths.update(ff_results[res_num]['plan_lengths'])
        plan_lengths.update(fd_results[res_num]['plan_lengths'])

        act_lengths = gt_results[res_num]['act_lengths']
        act_lengths.update(ff_results[res_num]['act_lengths'])
        act_lengths.update(fd_results[res_num]['act_lengths'])

        act_times = gt_results[res_num]['act_times']
        act_times.update(ff_results[res_num]['act_times'])
        act_times.update(fd_results[res_num]['act_times'])

        print(res_num, " Total GT recursions errors: ", total_gt_errors)
        print(res_num, " Total FD errors: ", total_fd_errors)
        
        axes[0, 0].set_ylabel('Node expansions')
        axes[0, 0].set_title(f'Node expansions over Methods (N={num_tests})')
        axes[0, 0].boxplot(expansions.values(), tick_labels=expansions.keys())
        axes[0, 0].set_yscale('log')
        axes[0, 0].tick_params(axis='y', which='minor')

        axes[0, 1].set_ylabel('Plan Path Lengths')
        axes[0, 1].set_title(f'Plan Path Lengths over Methods (N={num_tests})')
        axes[0, 1].boxplot(plan_lengths.values(), tick_labels=plan_lengths.keys())
        axes[0, 1].tick_params(axis='y', which='minor')

        axes[1, 0].set_ylabel('Acting Path Lengths')
        axes[1, 0].set_title(f'Acting Path Lengths over Methods (N={num_tests})')
        axes[1, 0].boxplot(act_lengths.values(), tick_labels=act_lengths.keys())
        axes[1, 0].tick_params(axis='x', labelrotation=90)
        axes[1, 0].tick_params(axis='y', which='minor')

        axes[1, 1].set_ylabel('Acting times (msec)')
        axes[1, 1].set_title(f'Acting Times over Methods (N={num_tests})')
        axes[1, 1].boxplot(act_times.values(), tick_labels=act_times.keys())
        axes[1, 1].tick_params(axis='x', labelrotation=90)
        axes[1, 1].tick_params(axis='y', which='minor')

        fig.suptitle(f"SearchAndRescue-7 Problem {res_num} Evaluation")

        plt.show()

if __name__ == '__main__':
    main()