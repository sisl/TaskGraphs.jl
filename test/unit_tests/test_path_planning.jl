# test SearchEnv
let
    solver = NBSSolver()
    pc_tapf = pctapf_problem_4(solver)
    pc_mapf = PC_MAPF(pc_tapf.env)

    let
        env = deepcopy(pc_mapf.env)
        cost_type(env)
        state_type(env)
        action_type(env)
        get_cost(env)
        set_solution_path!(env,get_paths(env)[1],1)
        set_path_cost!(env,get_path_costs(env)[1],1)
        set_cost!(env,typemin(cost_type(env)))
        for v in 1:nv(env.schedule)
            get_start(env,v)
        end
        @test_throws BoundsError get_start(env,nv(env.schedule)+1)

        # test that changes will not affect copies of a SearchEnv
        env2 = copy(env)
        set_cost!(env,typemax(cost_type(env)))
        @test get_cost(env2) != get_cost(env)

        path = path_type(env)()
        extend_path!(path,4)
        set_solution_path!(env,path,1)
        @test length(get_paths(env)[1]) != length(get_paths(env2)[1])

        set_path_cost!(env,typemin(cost_type(env)),1)
        set_path_cost!(env2,typemax(cost_type(env)),1)
        @test get_path_costs(env)[1] != get_path_costs(env2)[1]

        env.cache.t0[1] = env2.cache.t0[1] + 1
        env.cache.tF[1] = env2.cache.tF[1] + 1
        @test env.cache.t0[1] != env2.cache.t0[1]
        @test env.cache.tF[1] != env2.cache.tF[1]
    end
    let
        node = initialize_root_node(solver,pc_mapf)
        @test solution_type(node) <: SearchEnv
        node2 = initialize_child_search_node(solver,pc_mapf,node)
        @test solution_type(node2) <: SearchEnv

        set_cost!(node,typemax(cost_type(node)))
        @test get_cost(node2) != get_cost(node)

        set_path_cost!(node,typemin(cost_type(node)),1)
        set_path_cost!(node2,typemax(cost_type(node)),1)
        @test get_path_costs(node)[1] != get_path_costs(node2)[1]

        path = path_type(node)()
        extend_path!(path,4)
        set_solution_path!(node,path,1)
        @test length(get_paths(node)[1]) != length(get_paths(node2)[1])

        # now reset the route plan and check that everything matches up again
        reset_route_plan!(node,node2.solution.route_plan)
        @test length(get_paths(node)[1]) == length(get_paths(node2)[1])
        @test get_path_costs(node)[1] == get_path_costs(node2)[1]
        @test get_cost(node2) == get_cost(node)

        CRCBS.cbs_update_conflict_table!(solver,pc_mapf,node2,nothing)

        node.solution.cache.t0[1] = node2.solution.cache.t0[1] + 1
        node.solution.cache.tF[1] = node2.solution.cache.tF[1] + 1
        @test node.solution.cache.t0[1] != node2.solution.cache.t0[1]
        @test node.solution.cache.tF[1] != node2.solution.cache.tF[1]
    end

end
