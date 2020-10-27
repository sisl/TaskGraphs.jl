using TaskGraphs, CRCBS, GraphUtils, LightGraphs, SparseArrays
using Test

vtx_grid = initialize_dense_vtx_grid(4,4)
#  1   2   3   4
#  5   6   7   8
#  9  10  11  12
# 13  14  15  16
env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
vtxs = [6,7,10,11]
d = OilSpill(env_graph,vtxs)

TaskGraphs.apply_disturbance!(env_graph,d)
TaskGraphs.remove_disturbance!(env_graph,d)

solver = NBSSolver()
pctapf = pctapf_problem_1(solver)

search_env = pctapf.env
add_to_schedule!(search_env.schedule,CUB_AT(1,2),CleanUpBotID(1))
id = get_unique_action_id()
add_to_schedule!(search_env.schedule,CUB_GO(1,2,-1),id)
add_edge!(search_env.schedule,CleanUpBotID(1),id)

get_node_from_id(search_env.schedule,CleanUpBotID(1))
