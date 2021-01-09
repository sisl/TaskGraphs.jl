using TaskGraphs, CRCBS, GraphUtils, LightGraphs, JuMP, Gurobi
Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

# debugging replanning
let
    solver = NBSSolver()
    prob = pctapf_problem_2(solver)
    spec = get_problem_spec(get_env(prob))
    graph = get_graph(get_env(prob))
    sched = OperatingSchedule()
    reset_action_id_counter!()
    add_to_schedule!(sched,spec,ROBOT_AT(1,1),RobotID(1))
    add_to_schedule!(sched,spec,ROBOT_AT(2,2),RobotID(2))
    add_to_schedule!(sched,spec,OBJECT_AT(1,13),ObjectID(1))
    add_to_schedule!(sched,spec,OBJECT_AT(2,14),ObjectID(2))
    op = Operation(pre=Set([OBJECT_AT(1,1),OBJECT_AT(2,2)]))
    op_id = get_operation_id(op)
    add_to_schedule!(sched,spec,op,op_id)

    action_id = get_unique_action_id()
    add_to_schedule!(sched,spec,GO(1,1,-1),action_id)
    add_edge!(sched,RobotID(1),action_id)
    add_single_robot_delivery_task!(sched,spec,RobotID(1),ObjectID(1),op_id,action_id)

    action_id = get_unique_action_id()
    add_to_schedule!(sched,spec,GO(2,2,-1),action_id)
    add_edge!(sched,RobotID(2),action_id)
    add_single_robot_delivery_task!(sched,spec,RobotID(2),ObjectID(2),op_id,action_id)

    plot_project_schedule(sched,initialize_planning_cache(sched);verbose=false)

    search_env = construct_search_env(solver,sched,spec,graph)

    t_commit = 7
    t_split = t_commit
    replan_model = MergeAndBalance()
    new_sched, new_cache = prune_schedule(replan_model,search_env,t_commit)
    plot_project_schedule(new_sched,new_cache;verbose=false)

    robot_positions=get_env_snapshot(search_env,t_split)
    @test sanity_check(new_sched)
    new_sched, new_cache = split_active_vtxs!(replan_model,new_sched,problem_spec,new_cache,t_split;robot_positions=robot_positions)
    plot_project_schedule(new_sched,new_cache;verbose=false)

    # freeze nodes that terminate before cutoff time
    new_sched, new_cache = fix_precutoff_nodes!(replan_model,new_sched,problem_spec,new_cache,t_split)
    plot_project_schedule(new_sched,new_cache;verbose=false,
        color_function = (G,v,x,y,r)->get_path_spec(new_sched,v).fixed ? "gray" : get_prop(G,v,:color),
    )

    # Remove all "assignments" from schedule
    break_assignments!(replan_model,new_sched,problem_spec)
    @assert sanity_check(new_sched," after break_assignments!()")
    new_cache = initialize_planning_cache(new_sched,new_cache.t0,min.(new_cache.tF,t_commit))
    plot_project_schedule(new_sched,new_cache;verbose=true,
        color_function = (G,v,x,y,r)->get_path_spec(new_sched,v).fixed ? "gray" : get_prop(G,v,:color),
    )

end
let
    solver = NBSSolver()
    prob = pctapf_problem_2(solver)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    search_env, cost = solve!(solver,prob)
    sched               = get_schedule(search_env)
    cache               = get_cache(search_env)
    route_plan          = get_route_plan(search_env)
    problem_spec        = get_problem_spec(search_env)
    env_graph           = get_graph(search_env)

    t_request = 0
    next_sched = OperatingSchedule()

    replan_model = MergeAndBalance()

    t_commit = 5
    t_final = minimum(map(length, get_paths(get_route_plan(search_env))))
    t_split = min(t_commit,t_final)
    robot_positions=get_env_snapshot(search_env,t_split)
    reset_solver!(solver)

    plot_project_schedule(search_env;verbose=false)

    new_sched, new_cache = prune_schedule(replan_model,search_env,t_split)
    @assert sanity_check(new_sched," after prune_schedule()")
    plot_project_schedule(new_sched,new_cache;verbose=false)

    # split active nodes
    new_sched, new_cache = split_active_vtxs!(replan_model,new_sched,problem_spec,new_cache,t_split;robot_positions=robot_positions)
    @assert sanity_check(new_sched," after split_active_vtxs!()")
    plot_project_schedule(new_sched,new_cache;verbose=false)

    # freeze nodes that terminate before cutoff time
    new_sched, new_cache = fix_precutoff_nodes!(replan_model,new_sched,problem_spec,new_cache,t_split)
    plot_project_schedule(new_sched,new_cache;verbose=false,
        color_function = (G,v,x,y,r)->get_path_spec(new_sched,v).fixed ? "gray" : get_prop(G,v,:color),
    )

    # Remove all "assignments" from schedule
    break_assignments!(replan_model,new_sched,problem_spec)
    @assert sanity_check(new_sched," after break_assignments!()")
    new_cache = initialize_planning_cache(new_sched,new_cache.t0,min.(new_cache.tF,t_commit))
    plot_project_schedule(new_sched,new_cache;verbose=true,
        color_function = (G,v,x,y,r)->get_path_spec(new_sched,v).fixed ? "gray" : get_prop(G,v,:color),
    )

    # splice projects together!
    splice_schedules!(new_sched,next_sched)
    @assert sanity_check(new_sched," after splice_schedules!()")
    # NOTE: better performance is obtained when t_commit is the default t0 (tighter constraint on milp)
    t0 = map(v->get(new_cache.t0, v, t_commit), vertices(get_graph(new_sched)))
    tF = map(v->get(new_cache.tF, v, t_commit), vertices(get_graph(new_sched)))
    base_search_env = construct_search_env(
        solver,
        new_sched,
        search_env,
        initialize_planning_cache(new_sched,t0,tF)
        )
    base_route_plan = initialize_route_plan(search_env,get_cost_model(base_search_env))
    # @log_info(3,verbosity(solver),"Previous route plan: \n",sprint_route_plan(route_plan))
    trimmed_route_plan = trim_route_plan(base_search_env, base_route_plan, t_commit)
    # @log_info(3,verbosity(solver),"Trimmed route plan: \n",sprint_route_plan(trimmed_route_plan))
    SearchEnv(base_search_env, route_plan=trimmed_route_plan)

end
