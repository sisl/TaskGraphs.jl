using TaskGraphs

# initialize loader
base_dir = joinpath("/scratch/task_graphs_experiments","replanning3")
loader, planners, base_results_dir, base_problem_dir = setup_replanning_experiments(base_dir)

for planner in planners
    # warm up to precompile replanning code
    warmup(planner,loader)
    # profile
    planner_name = string(typeof(planner.primary_planner.replanner))
    results_path = joinpath(base_results_dir,planner_name)
    profile_replanner!(loader,planner,base_problem_dir,results_path)
end
# if results show ''"CRCBS.Feature" = val', use the following line to convert:
# sed -i 's/\"CRCBS\.\([A-Za-z]*\)\"/\1/g' **/**/*.toml
using TOML

results_file = joinpath(base_results_dir,"MergeAndBalance","problem0001")
TaskGraphs.load_replanning_results(loader,planners[1],results_file)
