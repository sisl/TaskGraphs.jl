let
    f = pctapf_problem_5
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

    goals = map(i->Vector{Int}(), 1:num_agents(base_search_env))
    for v in topological_sort_by_dfs(get_graph(sched))
        a = get_node_from_vtx(sched,v)
        if isa(a,AbstractRobotAction)
            i = get_id(get_robot_id(a))
            x = get_id(get_destination_location_id(a))
            if i != -1 && x != -1
                push!(goals[i],get_id(get_destination_location_id(a)))
            end
        end
    end
    goals

    solver = PIBTPlanner{NTuple{3,Float64}}()
    search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
    @show get_cost_model(search_env)
    pc_mapf = PC_MAPF(search_env)

    num_agents(pc_mapf)
    node  = initialize_root_node(solver,pc_mapf)
    CRCBS.build_env(solver,pc_mapf,node,AgentID(1))

    cache = CRCBS.pibt_init_cache(solver,pc_mapf)
    is_consistent(cache,pc_mapf)
    # pibt_step!(solver,pc_mapf,cache,1)
    set_iteration_limit!(solver,40)
    set_verbosity!(solver,4)
    reset_solver!(solver)
    solution, valid_flag = pibt!(solver,pc_mapf)

    @show vtx_lists = convert_to_vertex_lists(solution)
    grid_vtxs = map(v->[v[1],v[2]], get_graph(pc_mapf.env).vtxs)
    paths = map(p->map(v->grid_vtxs[v],p),vtx_lists)

    # using FactoryRendering
    # include("/home/kylebrown/.julia/dev/TaskGraphs/test/notebooks/render_tools.jl")
    # FactoryRendering.record_video(string(f,".webm"),vtx_lists,grid_vtxs;goals=goals,res=(400,400))
    # print_project_schedule(string(f),search_env;mode=:leaf_aligned)
    # run(`inkscape -z $(string(f)).svg -e $(string(f)).png`)

    # @show get_cost(solution)
    @show convert_to_vertex_lists(solution.route_plan)
    @test valid_flag
end
