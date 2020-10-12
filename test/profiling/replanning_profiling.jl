using TaskGraphs
using CRCBS
using Random

# initialize loader
loader = ReplanningProblemLoader()
add_env!(loader,"env_2",init_env_2())
# get probem config
problem_configs = replanning_config_1()
# define paths to problems and results
base_dir            = joinpath("/scratch/task_graphs_experiments","replanning2")
base_problem_dir    = joinpath(base_dir,"problem_instances")
base_results_dir    = joinpath(base_dir,"results")
# write problems
Random.seed!(0)
# Problem generation and profiling
reset_task_id_counter!()
reset_operation_id_counter!()
write_repeated_pctapf_problems!(loader,problem_configs,base_problem_dir)
# set up planner and profiling features
feats = [
    RunTime(),IterationCount(),TimeOutStatus(),IterationMaxOutStatus(),
    SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag(),
    RobotPaths(),NumConflicts(),
    ]
final_feats = [
    RunTime(),IterationCount(),TimeOutStatus(),IterationMaxOutStatus(),
    SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag(),
    RobotPaths(),NumConflicts(),
    ]
# Primary planner
path_finder = DefaultAStarSC()
set_iteration_limit!(path_finder,5000)
primary_route_planner = CBSSolver(ISPS(path_finder))
set_iteration_limit!(primary_route_planner,1000)
primary_planner = FullReplanner(
    solver = NBSSolver(path_planner=primary_route_planner),
    replanner = MergeAndBalance(),
    cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
    )
set_verbosity!(primary_planner.solver,0)
# set_iteration_limit!(low_level(low_level(route_planner(primary_planner.solver))),5000)
# set_iteration_limit!(route_planner(primary_planner.solver),1000)
# Backup planner
backup_planner = FullReplanner(
    solver = NBSSolver(
        assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
        path_planner = PIBTPlanner{NTuple{3,Float64}}()
        ),
    replanner = MergeAndBalance(),
    cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
    )
set_iteration_limit!(backup_planner,1)
set_iteration_limit!(route_planner(backup_planner.solver),5000)
set_verbosity!(backup_planner.solver,0)
# Full solver
planner = ReplannerWithBackup(primary_planner,backup_planner)
# warm up so that the planner doesn't fail because of slow compilation
warmup(planner,loader)
# profile
# set_real_time_flag!(planner,false)
profile_replanner!(loader,planner,base_problem_dir,base_results_dir)
