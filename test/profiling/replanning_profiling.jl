using TaskGraphs

# initialize loader
base_dir = joinpath("/scratch/task_graphs_experiments","replanning3")
loader, planners, base_results_dir, base_problem_dir = setup_replanning_experiments(base_dir)

for planner in planners
    # warm up to precompile replanning code
    warmup(planner,loader)
    # profile
    # planner_name = string(typeof(planner.primary_planner.replanner))
    # results_path = joinpath(base_results_dir,planner_name)
    # profile_replanner!(loader,planner,base_problem_dir,results_path)
end
