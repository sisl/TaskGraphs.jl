# Just dumping partially written ideas here


# function reverse_propagate_delay!(solver,cache,schedule,delay_vec)
#     buffer = zeros(nv(schedule))
#     for v in reverse(topological_sort_by_dfs(get_graph(schedule)))
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         buffer[v] = (cache.tF[v] - (cache.t0[v] + Δt_min))
#         for v2 in outneighbors(schedule,v)
#             delay_vec[v] = max(delay_vec[v], delay_vec[v2] - buffer[v2])
#             delay_vec[v] = max(0, delay_vec[v] - (cache.t0[v2] - cache.tF[v]))
#         end
#     end
#     delay_vec
# end
# function get_delay_vec(solver,cache,schedule,v0)
#     delay = zeros(nv(schedule))
#     for v in reverse(topological_sort_by_dfs(get_graph(schedule)))
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         delay[v] = (cache.tF[v] - (cache.t0[v] + Δt_min + minimum(cache.local_slack[v])))
#     end
#     # zero out all vtxs that are not predecessors of v
#     delay_vec = zeros(Int,nv(schedule))
#     for v in map(e->e.dst,collect(edges(bfs_tree(get_graph(schedule),v0;dir=:in))))
#         if cache.max_deadline[v] > 0
#             delay_vec[v] = min(delay[v],typemax(Int))
#         end
#     end
#     delay_vec
# end
# function backtrack_deadlines(solver,cache,schedule,v)
#     frontier = Set{Int}([v])
#     delay_cut = Set{Int}()
#     while length(frontier) > 0
#         v = pop!(frontier)
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         buffer = (cache.tF[v] - (cache.t0[v] + Δt_min))
#         if get_path_spec(schedule,v).fixed
#             continue
#         elseif cache.max_deadline[v] > 0 # still has room for some delay
#             push!(delay_cut,v)
#         elseif indegree(schedule,v) == 0
#             # if v is a root_node, the deadlines cannot be tightened anymore
#             @log_info(-1,solver.l3_verbosity,"ISPS: deadlines cannot be tightened any more.")
#             return Set{Int}()
#         else
#             for v2 in inneighbors(schedule,v)
#                 push!(frontier,v2)
#             end
#         end
#     end
#     return delay_cut
# end
# function tighten_deadline!(solver,env,route_plan,v,dt=1)
#     cache = env.cache
#     schedule = get_schedule(env)
#     active_set = cache.active_set
#     closed_set = cache.closed_set
#     node_queue = cache.node_queue
#     graph = get_graph(schedule)
#     # adjust allowable_slack
#     cache.max_deadline[v] = max(0, cache.max_deadline[v]-1)
#     # remove from closed set
#     push!(active_set, v)
#     setdiff!(closed_set,v)
#     # Trim schedule at v.t0
#     agent_id = get_path_spec(schedule,v).agent_id
#     if agent_id != -1
#         cbs_env = typeof(env.env)(
#             graph = env.env.graph,
#             agent_idx = agent_id,
#             cost_model = get_cost_model(env.env),
#             heuristic = get_heuristic_model(env.env)
#         )
#         new_path = trim_path(cbs_env,get_paths(route_plan)[agent_id],cache.t0[v])
#         set_solution_path!(route_plan,new_path,agent_id)
#         set_path_cost!(route_plan,new_path.cost,agent_id)
#     end
#     # Update node queue
#     for v2 in map(e->e.dst,collect(edges(bfs_tree(graph,v;dir=:out))))
#         setdiff!(closed_set,v2)
#         setdiff!(active_set,v2)
#     end
#     empty!(node_queue)
#     for v2 in active_set
#         node_queue[v2] = isps_queue_cost(schedule,cache,v2)
#     end
#     env,route_plan
# end
