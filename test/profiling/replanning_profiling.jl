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
solver = NBSSolver()
loader = ReplanningProblemLoader()
add_env!(loader,"env_2",init_env_2())

problem_configs = replanning_config_1()

base_dir            = joinpath("/scratch/task_graphs_experiments","replanning2")
base_problem_dir    = joinpath(base_dir,"problem_instances")
base_results_dir    = joinpath(base_dir,"results")

write_repeated_pctapf_problems!(loader,problem_configs,base_problem_dir)

# simple_prob_def = SimpleRepeatedProblemDef(r0 = [1,2,3],env_id = "env_1",)
#
# for (i,def) in enumerate([
#     (tasks=[1=>4,2=>5,3=>6],
#         ops=[
#             (inputs=[1,2],outputs=[3],Δt_op=0),
#             (inputs=[3],outputs=[],Δt_op=0)]),
#     (tasks=[7=>8,9=>10,11=>12],
#         ops=[
#             (inputs=[1,2],outputs=[3],Δt_op=0),
#             (inputs=[3],outputs=[],Δt_op=0)]),
#     ])
#     t = i*10
#     push!(simple_prob_def.requests,
#         SimpleReplanningRequest(pctapf_problem(Int[],def),t,t))
# end
# write_simple_repeated_problem_def("problem001",simple_prob_def)
simple_prob_def = read_simple_repeated_problem_def(joinpath(base_problem_dir,"problem0001"))
prob = RepeatedPC_TAPF(simple_prob_def,solver,loader)

prob.env.env_graph.dist_function

replan_model = MergeAndBalance()
set_real_time_flag!(replan_model,false)
profile_replanner!(solver,replan_model,prob)
