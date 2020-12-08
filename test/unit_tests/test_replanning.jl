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
# Test break_assignments(...)
let
    sched = OperatingSchedule()
    prev_id = ActionID()
    for (id,n) in [
            (ActionID(1),GO(1,1,1)),
            (ActionID(2),GO(1,1,2)),
            (ActionID(3),COLLECT(1,1,2))
            ]
        add_to_schedule!(sched,n,id)
        add_edge!(sched,prev_id,id)
        prev_id = id
    end
    # Break assignments should only remove BOT_GO-->BOT_COLLECT edges
    break_assignments!(sched,ProblemSpec())
    @test has_edge(sched,ActionID(1),ActionID(2))
    @test !has_edge(sched,ActionID(2),ActionID(3))
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
