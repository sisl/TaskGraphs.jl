using TaskGraphs, CRCBS, GraphUtils, LightGraphs, SparseArrays
using Test

vtx_grid = initialize_dense_vtx_grid(4,4)
#  1   2   3   4
#  5   6   7   8
#  9  10  11  12
# 13  14  15  16
env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
# vtxs = [6,7,10,11]
# d = OilSpill(env_graph,vtxs)
#
# TaskGraphs.apply_disturbance!(env_graph,d)
# TaskGraphs.remove_disturbance!(env_graph,d)

solver = NBSSolver()
prob = pctapf_problem_1(solver)
s_prob = TaskGraphs.stochastic_problem(
    PC_TAPF,
    solver,
    prob,
    [CUB_AT(3,6)],
    DisturbanceSequence([2=>DroppedObject(1)])
)
env,cost = solve!(solver,prob)


handle_disturbance!(solver,prob,env,DroppedObject(1),2)

sched = OperatingSchedule()
add_to_schedule!(sched,ROBOT_AT(1,1),RobotID(1))
id1 = get_unique_action_id()
add_to_schedule!(sched,GO(1,1,2),id1)
add_edge!(sched,RobotID(1),id1)
id = get_unique_action_id()
add_to_schedule!(sched,GO(1,2,-1),id)
add_edge!(sched,id1,id)

add_to_schedule!(sched,CUB_AT(2,2),CleanUpBotID(2))
id2 = get_unique_action_id()
add_to_schedule!(sched,CUB_GO(2,2,3),id2)
add_edge!(sched,CleanUpBotID(2),id2)
id = get_unique_action_id()
add_to_schedule!(sched,CUB_GO(1,3,-1),id)
add_edge!(sched,id2,id)

search_env = construct_search_env(solver,sched,ProblemSpec(N=2,D=get_dist_matrix(env_graph)),env_graph)
TaskGraphs.regenerate_path_specs!(solver,search_env)
pcmapf = PC_MAPF(search_env)
node = initialize_root_node(search_env)

schedule_node = get_node_from_id(sched,CleanUpBotID(2))
v = get_vtx(sched,CleanUpBotID(2))
plan_path!(low_level(low_level(route_planner(solver))),
    pcmapf,search_env,node,schedule_node,v)

schedule_node = get_node_from_id(sched,id2)
v = get_vtx(sched,id2)
plan_path!(low_level(low_level(route_planner(solver))),
    pcmapf,search_env,node,schedule_node,v)

schedule_node = get_node_from_id(sched,RobotID(1))
v = get_vtx(sched,RobotID(1))
plan_path!(low_level(low_level(route_planner(solver))),
    pcmapf,search_env,node,schedule_node,v)

schedule_node = get_node_from_id(sched,id1)
v = get_vtx(sched,id1)
cbs_env = build_env(solver, pcmapf, search_env, node, VtxID(v))
remove_edges!(get_graph(cbs_env),Set([Edge(1,2)]))
TaskGraphs.regenerate_path_specs!(solver,search_env)
@show get_path_spec(sched, v).min_path_duration

set_verbosity!(solver,6)
set_iteration_limit!(low_level(low_level(route_planner(solver))),50)
plan_path!(low_level(low_level(route_planner(solver))),
    pcmapf,search_env,node,schedule_node,v)
