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

    t_commit = 3
    t_split = t_commit
    replan_model = MergeAndBalance()
    new_sched, new_cache = prune_schedule(replan_model,search_env,t_commit)
    plot_project_schedule(new_sched,new_cache;verbose=false)


    robot_positions=get_env_snapshot(search_env,t_split)
    @test sanity_check(new_sched)
    new_sched, new_cache = split_active_vtxs!(replan_model,,problem_spec,new_cache,t_split;robot_positions=robot_positions)

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
    set_commit_threshold!(replan_model,1)
    commit_threshold=get_commit_threshold(replan_model)

    @test sanity_check(sched)

    t_commit = get_commit_time(replan_model, search_env, t_request, commit_threshold)
    t_final = minimum(map(length, get_paths(get_route_plan(search_env))))
    t_split = min(t_commit,t_final)
    robot_positions=get_env_snapshot(search_env,t_split)
    reset_solver!(solver)
    # TaskGraphs.set_time_limits!(replan_model,solver,t_request,t_commit)
    # Update operating schedule
    new_sched, new_cache = prune_schedule(replan_model,search_env,t_split)


end
