let
    solver = NBSSolver()
    prob = pctapf_problem_1(solver)
    env = get_env(prob)

    # reserved = Set{Int}()
    # reserved = Set{Int}()
    # for (v,vp) in [(9,14)]
    #     add_edge!(get_graph(get_schedule(env)),v,vp)
    # end

    cache = compute_lower_bound(env)
    maximum(get_cache(env).tF)
    maximum(cache.tF)
end
