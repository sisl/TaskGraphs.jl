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
m = env_graph.dist_function.dist_mtxs[(1,2)]
@show m(1,7)
GraphUtils.remove_edges!(m,d.edges)
@show m(1,7)
GraphUtils.add_edges!(m,d.edges)
@show m(1,7)

TaskGraphs.account_for_disturbance!(env_graph,d)


env_graph.graph

env_graph.dist_function.dist_mtxs[(1,2)]
