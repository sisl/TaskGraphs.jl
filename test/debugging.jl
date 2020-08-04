let
    f = pctapf_problem_10
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
    set_iteration_limit!(solver,20)
    set_verbosity!(solver,4)
    reset_solver!(solver)
    solution, valid_flag = pibt!(solver,pc_mapf)

    @show vtx_lists = convert_to_vertex_lists(solution)
    grid_vtxs = map(v->[v[1],v[2]], get_graph(pc_mapf.env).vtxs)
    paths = map(p->map(v->grid_vtxs[v],p),vtx_lists)

    using FactoryRendering
    include("/home/kylebrown/.julia/dev/TaskGraphs/test/notebooks/render_tools.jl")
    base_path = joinpath(dirname(pathof(TaskGraphs)),"..","test",string(f))
    mkpath(base_path)
    vid_file = joinpath(base_path,"animation.webm")
    sched_file = joinpath(base_path,"schedule")
    FactoryRendering.record_video(vid_file,vtx_lists,grid_vtxs;goals=goals,res=(400,400))
    print_project_schedule(sched_file,search_env;mode=:leaf_aligned)
    run(`inkscape -z $sched_file.svg -e $sched_file.png`)
    run(`rm $sched_file.svg`)

    # @show get_cost(solution)
    @show convert_to_vertex_lists(solution.route_plan)
    @test valid_flag
end
let
    solver = NBSSolver(path_planner = PIBTPlanner{NTuple{3,Float64}}())
    set_verbosity!(solver,0)
    set_iteration_limit!(solver,1)
    set_iteration_limit!(route_planner(solver),50)

    replan_model = MergeAndBalance()
    commit_threshold = 5
    f = replanning_problem_1
    prob = f(solver)
    # env, cost = solve!(solver,PC_TAPF(prob.env);optimizer=Gurobi.Optimizer)
    # base_env = replan!(solver,MergeAndBalance(),env,prob.requests[1])
    base_env = prob.env
    env = prob.env

    request = prob.requests[1]
    remap_object_ids!(request.schedule,env.schedule)
    base_env = replan!(solver,replan_model,env,request)
    reset_solver!(solver)
    env, cost = solve!(solver,base_env;optimizer=Gurobi.Optimizer)

    request = prob.requests[2]
    remap_object_ids!(request.schedule,env.schedule)
    base_env = replan!(solver,replan_model,env,request)
    reset_solver!(solver)
    env, cost = solve!(solver,base_env;optimizer=Gurobi.Optimizer)

    request = prob.requests[3]
    remap_object_ids!(request.schedule,env.schedule) # NOTE Why is this causing a "key ObjectID(3) not found error?"
    base_env = replan!(solver,replan_model,env,request)

    reset_solver!(solver)
    # assignment_problem = formulate_assignment_problem(assignment_solver(solver), base_env;optimizer=Gurobi.Optimizer)
    # update_assignment_problem!(assignment_solver(solver), assignment_problem, base_env)
    # schedule, l_bound = solve_assignment_problem!(assignment_solver(solver),assignment_problem,base_env)
    # env, cost = plan_route!(route_planner(solver),schedule,base_env)

    env, cost = solve!(solver,base_env;optimizer=Gurobi.Optimizer)

    request = prob.requests[4]
    remap_object_ids!(request.schedule,env.schedule) # NOTE Why is this causing a "key ObjectID(3) not found error?"
    base_env = replan!(solver,replan_model,env,request)
    reset_solver!(solver)
    env, cost = solve!(solver,base_env;optimizer=Gurobi.Optimizer)

    goals = map(i->Vector{Int}(), 1:num_agents(env))
    for v in topological_sort_by_dfs(get_graph(env.schedule))
        a = get_node_from_vtx(env.schedule,v)
        if isa(a,AbstractRobotAction)
            i = get_id(get_robot_id(a))
            x = get_id(get_destination_location_id(a))
            if i != -1 && x != -1
                push!(goals[i],get_id(get_destination_location_id(a)))
            end
        end
    end
    goals

    @show vtx_lists = convert_to_vertex_lists(env.route_plan)
    grid_vtxs = map(v->[v[1],v[2]], get_graph(env).vtxs)
    paths = map(p->map(v->grid_vtxs[v],p),vtx_lists)

    using FactoryRendering
    include("/home/kylebrown/.julia/dev/TaskGraphs/test/notebooks/render_tools.jl")
    base_path = joinpath(dirname(pathof(TaskGraphs)),"..","test",string(f))
    mkpath(base_path)
    vid_file = joinpath(base_path,"animation.webm")
    sched_file = joinpath(base_path,"schedule")
    FactoryRendering.record_video(vid_file,vtx_lists,grid_vtxs;goals=goals,res=(400,400))
    print_project_schedule(sched_file,env;mode=:leaf_aligned)
    run(`inkscape -z $sched_file.svg -e $sched_file.png`)
    run(`rm $sched_file.svg`)

end
