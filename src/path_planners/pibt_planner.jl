function construct_heuristic_model(solver::PIBTPlanner,env_graph;
        ph = PerfectHeuristic(get_dist_matrix(env_graph)),
        kwargs...)
    construct_composite_heuristic(ph,ph,NullHeuristic())
end
function construct_cost_model(solver::PIBTPlanner,
        schedule, cache, problem_spec, env_graph, primary_objective=SumOfMakeSpans(); kwargs...)
    cost_model = construct_composite_cost_model(
        typeof(primary_objective)(schedule,cache),
        FullCostModel(maximum,TravelTime()),
        FullCostModel(maximum,TravelDistance())
        )
    heuristic_model = construct_heuristic_model(solver,env_graph;kwargs...)
    cost_model, heuristic_model
end

function CRCBS.is_consistent(cache::PIBTCache,pc_mapf::PC_MAPF)
    search_env = CRCBS.get_solution(cache)
    n_jobless = 0
    # for e in CRCBS.get_envs(cache)
    #     if !CRCBS.is_valid(get_goal(e))
    #         n_jobless += 1
    #     end
    # end
    if length(search_env.cache.closed_set) + n_jobless >= nv(search_env.schedule)
        return true
    elseif !any(CRCBS.is_valid, map(e->CRCBS.get_goal(e),CRCBS.get_envs(cache)))
        # all robots are done with their jobs
        return true
    end
    return false
end

"""
    CRCBS.pibt_update_solution!(solver,pc_mapf::PC_MAPF,solution::SearchEnv,cache)

Overridden to update the SearchEnv
"""
function CRCBS.pibt_update_solution!(solver,solution::SearchEnv,cache)
    CRCBS.pibt_update_solution!(solver,solution.route_plan,cache)
    for (p,env) in zip(get_paths(solution),CRCBS.get_envs(cache))
        sp = get_final_state(p)
        # update the env if the goal has been reached
        if is_goal(env,sp) && CRCBS.is_valid(env,get_goal(env))
            v = get_vtx(solution.schedule,env.node_id)
            update_env!(solver,solution,v,p)
            update_planning_cache!(solver,solution)
        end
    end
    solution
end
function CRCBS.pibt_update_env!(solver,mapf::PC_MAPF,cache,agent_id)
    node = initialize_root_node(solver,mapf)
    CRCBS.get_envs(cache)[agent_id] = build_env(solver,CRCBS.get_solution(cache),node,agent_id)
end
