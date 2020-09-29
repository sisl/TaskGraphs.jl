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
    # redirect_to_files("out.log","err.log") do
        features=[
            RunTime(),SolutionCost(),OptimalFlag(),FeasibleFlag(),OptimalityGap(),
            IterationCount(),TimeOutStatus(),IterationMaxOutStatus(),
            RobotPaths()
            ]
        cache = ReplanningProfilerCache(features=features)
        # solver = NBSSolver(path_planner = PIBTPlanner{NTuple{3,Float64}}())
        solver = NBSSolver()
        # set_verbosity!(solver,5)
        # set_verbosity!(low_level(route_planner(solver)),4)
        # set_verbosity!(low_level(low_level(route_planner(solver))),5)
        set_iteration_limit!(solver,1)
        set_iteration_limit!(route_planner(solver),10)
        set_runtime_limit!(route_planner(solver),500)
        # set_debug!(solver,true)

        replan_model = MergeAndBalance()
        set_real_time_flag!(replan_model,false) # turn off real-time op constraints
        # set_commit_threshold!(replan_model,40) # setting high commit threshold to allow for warmup
        prob = replanning_problem_1(solver)
        # length(prob.requests)
        # pop!(prob.requests)
        # deleteat!(prob.requests,length(prob.requests))

        env, cache = profile_replanner!(solver,replan_model,prob,cache)
    # end

end
let
    cache = features=[
        RunTime(),SolutionCost(),OptimalFlag(),FeasibleFlag(),OptimalityGap(),
        IterationCount(),TimeOutStatus(),IterationMaxOutStatus(),
        RobotPaths()
        ]
    replan_model = MergeAndBalance()
    set_real_time_flag!(replan_model,false) # turn off real time constraints
    cost_model = SumOfMakeSpans()
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
            prob = f(solver;cost_function=cost_model)
            cache = profile_replanner!(solver,replan_model,prob)
            reset_solver!(solver)
        end
    end
end
# Problem generation and profiling
let
    solver = NBSSolver()
    loader = ReplanningProblemLoader()
    add_env!(loader,"env_1",init_env_1())

    simple_prob_def = SimpleRepeatedProblemDef(r0 = [1,2,3],env_id = "env_1",)

    for (i,def) in enumerate([
        (tasks=[1=>4,2=>5,3=>6],
            ops=[
                (inputs=[1,2],outputs=[3],Δt_op=0),
                (inputs=[3],outputs=[],Δt_op=0)]),
        (tasks=[7=>8,9=>10,11=>12],
            ops=[
                (inputs=[1,2],outputs=[3],Δt_op=0),
                (inputs=[3],outputs=[],Δt_op=0)]),
        ])
        t = i*10
        push!(simple_prob_def.requests,
            SimpleReplanningRequest(pctapf_problem(Int[],def),t,t))
    end
    write_simple_repeated_problem_def("problem001",simple_prob_def)
    simple_prob_def = read_simple_repeated_problem_def("problem001")
    prob = RepeatedPC_TAPF(simple_prob_def,solver,loader)

    search_env = replan!(solver,replan_model,prob.env,prob.requests[1])
    is_cyclic(search_env.schedule.graph)
    node_strings = [string(v," → ",string(get_node_from_vtx(search_env.schedule,v))) for v in vertices(search_env.schedule)]
    edgelist = collect(edges(search_env.schedule))
    node_strings[1:25],edgelist

    #
    # is_cyclic(prob.env.schedule.graph)
    # is_cyclic(prob.requests[1].schedule.graph)
    # is_cyclic(prob.requests[2].schedule.graph)

    set_verbosity!(solver,5)
    replan_model = MergeAndBalance()
    set_real_time_flag!(replan_model,false)
    profile_replanner!(solver,replan_model,prob)

end
