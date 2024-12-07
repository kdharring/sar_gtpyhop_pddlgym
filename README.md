# sar_gtpyhop_pddlgym

A project designed to evaluate GTPyhop and PDDL planners in a search and rescue gym environment. Contains GTPyhop translations of the SAR domain and three SAR problem files to experiment with. The gym environment and planners are included as submodules, including a fork of PDDLGym to include an extra problem file.

## Files

### sar_domain.py

Contains the domain definition of SearchAndRescue in GTPyhop. This includes all of the actions and methods GTPyhop uses, and the `run_lookahead` and `run_lazy_lookahead` implementations.

### sar_problem_\<x>.py

Contains a definition of the initial state and goals for problems found in PDDLGym. `sar_problem1.py` implements the `problem0.pddl` from SearchAndRescue-Level7 and `sar_problem1.py` implements `problem40.pddl` from SearchAndRescue-Level7. `sar_problem1.py` implements the new `problem50.pddl`.

Each problem file has a main block so that when the file is run an example acting run will execute. When not using this main block, be sure to use to correct problem file index in the function parameter. Be careful, since the indices are based on the sorted filenames, which means the indices are out of order (problem 50 is index 46 for example).

### pddl_sar_plan.py

This file contains methods to plan and act with FastForward and FastDownward. FastDownward has many options that also need to be specified when calling its methods. Examples of these options can be found here: <https://www.fast-downward.org/PlannerUsage>

### eval_planners_actors.py

This script conducts the tests used in the report. It will run multiple trials of planning and acting for each planning approach. Multiprocessing speeds this up a bit, but it still does take time to run. It cannot be sped up much more though, since FastDownward is the main bottleneck and it uses an external file for intermediate calculations, which would overwrite itself if FD is parallelized.

To help save progress, each method writes its results to its own results JSON file.

## Fixing PDDLGym_Planners

When running an example with FastForward search, you may encounter an AssertionError for a path existing. This is a downstream error of FastForward not compiling do to linkage issues in the C code. To fix this, navigate to the FF folder (`pddlgym_planners/pddlgym_planners/FF-v2.3`) and edit the following files:

* `main.c` - add `extern` to the definition of `gbracket_count`
* `search.c` - add `extern` to the definition of `lcurrent_goals`

Then run `make` to compile FF so it is not overwritten when the python file is run again.
