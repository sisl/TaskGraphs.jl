using TaskGraphs

# initialize loader
base_dir = joinpath("/scratch/task_graphs_experiments","replanning3")
loader, planner, base_results_dir, base_problem_dir = setup_replanning_experiments(base_dir)
# warm up to precompile replanning code
warmup(planner,loader)
# profile
profile_replanner!(loader,planner,base_problem_dir,base_results_dir)
