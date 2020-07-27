let
    f = pctapf_problem_1
    cost_model = SumOfMakeSpans()
    project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model,verbose=false)
    solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
    project_schedule = construct_partial_project_schedule(
        project_spec,
        problem_spec,
        robot_ICs,
        )
    base_search_env = construct_search_env(
        solver,
        project_schedule,
        problem_spec,
        env_graph
        )
    prob = formulate_assignment_problem(solver.assignment_model,base_search_env;
        optimizer=Gurobi.Optimizer,
    )
    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,base_search_env)
    @test validate(sched)

    solver = PIBTPlanner{NTuple{3,Float64}}()
    search_env = construct_search_env(planner,deepcopy(sched),base_search_env)
    @show get_cost_model(search_env)
    pc_mapf = PC_MAPF(search_env)

    num_agents(pc_mapf)
    node  = initialize_root_node(solver,pc_mapf)
    build_env(solver,pc_mapf,node,1)

    cache = CRCBS.pibt_init_cache(solver,pc_mapf)
    is_consistent(cache,pc_mapf)
    # pibt_step!(solver,pc_mapf,cache,1)
    set_iteration_limit!(solver,10)
    set_verbosity!(solver,4)
    reset_solver!(solver)
    # for i in 1:2
    #     a = CRCBS.wait(cache.envs[i],cache.states[i])
    #     CRCBS.set_action!(cache,i,a)
    # end
    solution, valid_flag = pibt!(solver,pc_mapf)
    # @show get_cost(solution)
    @show convert_to_vertex_lists(solution.route_plan)
    @test valid_flag
end
