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

solver = NBSSolver()
loader = ReplanningProblemLoader()
# add_env!(loader,"env_2",init_env_2())
prob = pctapf_problem_1(solver)
add_env!(loader,"env_2",prob.env.env_graph)

base_dir            = joinpath("/scratch/kylebrown/task_graphs_experiments","dummy")
base_problem_dir    = joinpath(base_dir,"problem_instances")
base_results_dir    = joinpath(base_dir,"results")

problem_configs = replanning_config_2()
write_repeated_pctapf_problems!(loader,problem_configs,base_problem_dir)

simple_prob_def = read_simple_repeated_problem_def(joinpath(base_problem_dir,"problem0001"))
prob = RepeatedPC_TAPF(simple_prob_def,solver,loader)

replan_model = MergeAndBalance()
set_real_time_flag!(replan_model,false)
set_verbosity!(solver,2)

reset_solver!(solver)
search_env, cache = profile_replanner!(solver,replan_model,prob)
cache.final_results
