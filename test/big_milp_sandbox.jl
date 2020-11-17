using TaskGraphs, CRCBS, GraphUtils, LightGraphs, JuMP
Revise.includet("/home/kylebrown/.julia/dev/TaskGraphs/src/milp_formulation.jl")

vtx_grid = initialize_dense_vtx_grid(3,3)
env = construct_factory_env_from_vtx_grid(vtx_grid)
# convert to graph to ensure undirected edges
env_graph = convert_env_graph_to_undirected(env.graph)

env_graph = Graph(4)
add_edge!(env_graph,1,2)
add_edge!(env_graph,3,4)
ne(env_graph)
collect(edges(env_graph))
time_extended_var_to_env_edge(env_graph,10)
