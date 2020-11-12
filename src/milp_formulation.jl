""" The number of edges in each "gadget" per original edge """
const GADGET_EDGE_MULTIPLIER = 5

""" convert from variable index to edge
 1   2  3   4
 ------------
 1   2  6   7
   3      8
 4   5  9   10
"""
function time_extended_var_to_env_edge(env_graph,v,t)
    time_index = rem(v-1, ne(env_graph)*GADGET_EDGE_MULTIPLIER*t)+1
    # src =
end
function convert_env_graph_to_undirected(G)
    env_graph = Graph(G)
    for v in vertices(env_graph)
        rem_edge!(env_graph,v,v)
    end
    return env_graph
end
function formulate_big_milp(prob::PC_TAPF,T_MAX)
    env_graph = Graph(get_env(prob).graph)
    G = ne(get_env(prob)) * GADGET_EDGE_MULTIPLIER * T_MAX
    edge_list = sort(collect(edges(get_env(G))))
end
