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
    @test remap_object_id(ObjectID(1),2) == ObjectID(3)
    @test remap_object_id(OBJECT_AT(1,1),2) == OBJECT_AT(3,1)
    remap_object_id(ScheduleNode(
        ObjectID(1),
        OBJECT_AT(1,1),
        PathSpec()
        ),2)
    prob = pctapf_problem_1(NBSSolver())
    sched = get_schedule(prob.env)
    remap_object_ids!(sched,10)
    for (k,v) in get_object_ICs(sched)
        @test get_id(k) > 10
        @test get_id(get_object_id(v)) > 10
    end
end
# Test break_assignments(...)
let
    sched = OperatingSchedule()
    n1 = add_node!(sched,make_node(sched,GO(1,1,1)))
    n2 = add_child!(sched,n1.id,make_node(sched,GO(1,1,2)))
    n3 = add_child!(sched,n2.id,make_node(sched,COLLECT(1,1,2)))
    # Break assignments should only remove BOT_GO-->BOT_COLLECT edges
    break_assignments!(sched,ProblemSpec())
    @test has_edge(sched,n1,n2)
    @test !has_edge(sched,n2,n3)
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
    for v in vertices(request.schedule)
        node = get_node(request.schedule,v)
        @test get_t0(node) >= request.t_request
        @test get_t0(base_env,node) >= get_commit_time(planner,env,request)
    end
    # env, cost = solve!(planner.solver,PC_TAPF(base_env))
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
        NBSSolver(assignment_model=TaskGraphsMILPSolver(ExtendedAssignmentMILP())),
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
let
    planner = FullReplanner(
        solver = NBSSolver(),
        replanner = ConstrainedMergeAndBalance(max_problem_size=20)
    )
    planner2 = FullReplanner(
        solver = NBSSolver(),
        replanner = MergeAndBalance()
    )
    set_commit_threshold!(planner,2)
    set_commit_threshold!(planner2,2)
    set_real_time_flag!(planner,false) # turn off real-time op constraints
    set_real_time_flag!(planner2,false) # turn off real-time op constraints
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
