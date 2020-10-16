using TaskGraphs, CRCBS, TOML

# initialize loader
base_dir = joinpath("/scratch/task_graphs_experiments","replanning3")
base_problem_dir    = joinpath(base_dir,"problem_instances")
base_results_dir    = joinpath(base_dir,"results")

loader = ReplanningProblemLoader()
add_env!(loader,"env_2",init_env_2())

reset_task_id_counter!()
reset_operation_id_counter!()
# # get probem config
# problem_configs = replanning_config_3()
# # write problems
# Random.seed!(0)
# write_problems!(loader,problem_configs,base_problem_dir)

feats = [
    RunTime(),IterationCount(),LowLevelIterationCount(),
    TimeOutStatus(),IterationMaxOutStatus(),
    SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag(),NumConflicts(),
    ObjectPathSummaries(),
    ]
final_feats = [SolutionCost(),NumConflicts(),RobotPaths()]
planner_configs = []
for (primary_replanner, backup_replanner) in [
        (MergeAndBalance(),MergeAndBalance()),
        (ReassignFreeRobots(),ReassignFreeRobots()),
        (DeferUntilCompletion(),DeferUntilCompletion()),
    ]
    # Primary planner
    path_finder = DefaultAStarSC()
    set_iteration_limit!(path_finder,5000)
    primary_route_planner = CBSSolver(ISPS(path_finder))
    set_iteration_limit!(primary_route_planner,1000)
    primary_planner = FullReplanner(
        solver = NBSSolver(path_planner=primary_route_planner),
        replanner = primary_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    # Backup planner
    backup_planner = FullReplanner(
        solver = NBSSolver(
            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
            path_planner = PIBTPlanner{NTuple{3,Float64}}()
            ),
        replanner = backup_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_iteration_limit!(backup_planner,1)
    set_iteration_limit!(route_planner(backup_planner.solver),5000)
    # Full solver
    planner = ReplannerWithBackup(primary_planner,backup_planner)
    planner_config = (
        planner=planner,
        results_path=joinpath(base_results_dir,string(typeof(primary_replanner))),
        objective = SumOfMakeSpans()
    )
    push!(planner_configs,planner_config)
end

# for planner in planners
#     # warm up to precompile replanning code
#     warmup(planner,loader)
#     # profile
#     planner_name = string(typeof(planner.primary_planner.replanner))
#     results_path = joinpath(base_results_dir,planner_name)
#     profile_replanner!(loader,planner,base_problem_dir,results_path)
# end

# if results show ''"CRCBS.Feature" = val', use the following line to convert:
# sed -i 's/\"CRCBS\.\([A-Za-z]*\)\"/\1/g' **/**/*.toml

prob_name = "problem0001"
prob_file = joinpath(base_problem_dir,prob_name)
results_file = joinpath(base_results_dir,"MergeAndBalance",prob_name)
results = TaskGraphs.load_replanning_results(loader,planner_configs[1].planner,results_file)
config = CRCBS.load_config(loader,prob_file)

TaskGraphs.post_process_replanning_results!(results,config)
dummy_path = "dummy.toml"
results["primary_planner"][1]
