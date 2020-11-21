# Replanner Interface
let
    for model in [
            ReplannerConfig(),
            DeferUntilCompletion(),
            ReassignFreeRobots(),
            MergeAndBalance(),
            Oracle(),
            NullReplanner(),
        ]
        set_commit_threshold!(model,10)
        @test get_commit_threshold(model) == 10
        set_timeout_buffer!(model,10)
        @test get_timeout_buffer(model) == 10
        set_route_planning_buffer!(model,10)
        @test get_route_planning_buffer(model) == 10
    end
    FullReplanner()
    ProjectRequest(OperatingSchedule(),10,10)
end
let
    planner = FullReplanner(
        solver = NBSSolver(),
        replanner = MergeAndBalance()
    )
    set_real_time_flag!(planner,false) # turn off real-time op constraints
    # set_commit_threshold!(replan_model,40) # setting high commit threshold to allow for warmup
    prob = replanning_problem_1(planner.solver)
    env = prob.env
    stage = 0

    stage += 1
    request = prob.requests[stage]
    remap_object_ids!(request.schedule,env.schedule)
    base_env = replan!(planner,env,request)

    env, cost = solve!(planner.solver,PC_TAPF(base_env))

end
let
    cache = features=[
        RunTime(),SolutionCost(),OptimalFlag(),FeasibleFlag(),OptimalityGap(),
        IterationCount(),TimeOutStatus(),IterationMaxOutStatus(),
        RobotPaths()
        ]
    replan_model = MergeAndBalance()
    set_real_time_flag!(replan_model,false) # turn off real time constraints
    solvers = [
        NBSSolver(),
        NBSSolver(path_planner = PIBTPlanner{NTuple{3,Float64}}()),
    ]
    for solver in solvers
        set_verbosity!(solver,0)
        set_iteration_limit!(solver,1)
        set_iteration_limit!(route_planner(solver),5000)
    end
    problem_generators = replanning_test_problems()

    for f in problem_generators
        for solver in solvers
            # @show f
            cache = ReplanningProfilerCache(features=features)
            prob = f(solver)
            cache = profile_replanner!(solver,replan_model,prob)
            reset_solver!(solver)
        end
    end
end

# test each step of replanning pipeline
let
    solver = NBSSolver()
    prob = pctapf_problem_2(solver)
    spec = get_problem_spec(get_env(prob))
    graph = get_graph(get_env(prob))
    sched = OperatingSchedule()
    add_to_schedule!(sched,spec,ROBOT_AT(1,1),RobotID(1))
    add_to_schedule!(sched,spec,OBJECT_AT(1,13),ObjectID(1))
    add_to_schedule!(sched,spec,ROBOT_AT(2,2),RobotID(2))
    add_to_schedule!(sched,spec,OBJECT_AT(2,14),ObjectID(2))
    add_single_robot_delivery_task!(sched,spec,RobotID(1),1,1,13,1)
    for (node,id) in [
        (ROBOT_AT(1,1),RobotID(1)),
        (GO(1,1,13),ActionID(1)),
        (OBJECT_AT(1,13),ObjectID(1)),
        (COLLECT(1,1,13),ActionID(2)),
        (CARRY(1,1,13,1),ActionID(3)),
        (DEPOSIT(1,1,1),ActionID(4)),

        (ROBOT_AT(2,2),RobotID(2)),
        (GO(2,2,14),ActionID(2)),
        (OBJECT_AT(2,14),ObjectID(2)),
        (COLLECT(2,2,14),ActionID(4)),
        (CARRY(2,2,14,2),ActionID(6)),
        (DEPOSIT(2,2,2),ActionID(6)),
        ]
        add_to_schedule!(sched,spec,node,id)
    end

    env = construct_search_env(solver,sched,spec,graph)
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
