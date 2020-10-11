using TaskGraphs
using CRCBS
# using LightGraphs, MetaGraphs, GraphUtils
# using JuMP, MathOptInterface
# using Gurobi
# using Random
# using TOML
#
# using Test
# using Logging

# Problem generation and profiling
reset_task_id_counter!()
reset_operation_id_counter!()
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
primary_planner = FullReplanner(
    solver = NBSSolver(),
    replanner = MergeAndBalance(),
    cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
    )
set_real_time_flag!(primary_planner.replanner,false)
set_verbosity!(primary_planner.solver,0)

# Backup planner
backup_planner = FullReplanner(
    solver = NBSSolver(
        assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
        path_planner = PIBTPlanner{NTuple{3,Float64}}()
        ),
    replanner = MergeAndBalance(),
    cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
    )
set_real_time_flag!(backup_planner.replanner,false)
set_iteration_limit!(backup_planner.solver,1)
set_iteration_limit!(route_planner(backup_planner.solver),500)
set_verbosity!(backup_planner.solver,0)

# Full solver
planner = ReplannerWithBackup(primary_planner,backup_planner)

###

# get probem config
problem_configs = replanning_config_2()
# define paths to problems and results
base_dir            = joinpath("/scratch/kylebrown/task_graphs_experiments","dummy")
base_problem_dir    = joinpath(base_dir,"problem_instances")
base_results_dir    = joinpath(base_dir,"results")
# initialize loader
loader = ReplanningProblemLoader()
# add_env!(loader,"env_2",init_env_2())
prob = pctapf_problem_1(planner.primary_planner.solver)
add_env!(loader,"env_2",prob.env.env_graph)
# write problems
write_repeated_pctapf_problems!(loader,problem_configs,base_problem_dir)

simple_prob_def = read_simple_repeated_problem_def(joinpath(base_problem_dir,"problem0001"))
prob = RepeatedPC_TAPF(simple_prob_def,planner.primary_planner.solver,loader)
# cache = ReplanningProfilerCache(features=feats,final_features=final_feats)

# construct_search_env(
#     planner.backup_planner.solver,
#     prob.env.schedule,
#     prob.env,
#     prob.env.cache,
# ).route_plan.paths[1].cost
# reset_solver!(solver)
# search_env, cache = profile_replanner!(solver,replan_model,prob,cache)
search_env, planner = profile_replanner!(planner,prob)
