let
    ReplannerConfig()
    DeferUntilCompletion()
    ReassignFreeRobots()
    MergeAndBalance()
    Oracle()
    NullReplanner()
    ReplanningProfilerCache()
    FullReplanner()

    ProjectRequest(OperatingSchedule(),10,10)

    solver = NBSSolver()
    replan_model = MergeAndBalance()
    commit_threshold = 5
    prob = replanning_problem_1(solver)
    # env, cost = solve!(solver,PC_TAPF(prob.env);optimizer=Gurobi.Optimizer)
    # base_env = replan!(solver,MergeAndBalance(),env,prob.requests[1])
    base_env = prob.env
    env = prob.env

    request = prob.requests[1]
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
    env, cost = solve!(solver,base_env;optimizer=Gurobi.Optimizer)


    base_env = replan!(solver,replan_model,env,request)
    reset_solver!(solver)
    set_iteration_limit!(route_planner(solver),10)
    env, cost = solve!(solver,PC_TAPF(base_env);optimizer=Gurobi.Optimizer)

    prob.env.schedule
    convert_to_vertex_lists(prob.env.route_plan)
    solver = PIBTPlanner{NTuple{3,Float64}}()
    prob = replanning_problem_1(solver)
    env, cost = solve!(solver,PC_MAPF(prob.env))
    convert_to_vertex_lists(env.route_plan)

end
let
    replan_model = MergeAndBalance()
    cost_model = SumOfMakeSpans()
    solvers = [
        NBSSolver(),
        NBSSolver(path_planner = PIBTPlanner{NTuple{3,Float64}}()),
    ]
    for solver in solvers
        set_verbosity!(solver,0)
        set_iteration_limit!(solver,1)
        set_iteration_limit!(route_planner(solver),50)
    end
    commit_threshold = 5
    problem_generators = replanning_test_problems()

    for f in problem_generators
        for solver in solvers
            prob = f(solver;cost_function=cost_model)
            base_env = prob.env
            env = prob.env
            for (stage,request) in enumerate(prob.requests)
                if stage == 1
                #     base_env = construct_search_env(solver,request.schedule,env.problem_spec,get_graph(env))
                    # base_env = deepcopy(env)
                else
                    remap_object_ids!(request.schedule,env.schedule)
                    base_env = replan!(solver,replan_model,env,request;commit_threshold=commit_threshold)
                end
                reset_solver!(solver)
                env, cost = solve!(solver,base_env;optimizer=Gurobi.Optimizer)
                log_info(-1,solver,
                    "Problem: ",f,"\n",
                    "Solver: ",typeof(solver),"\n",
                    "Stage: ",stage,"\n",
                    "route planner iterations: ", iterations(route_planner(solver)),
                )
            end
        end
    end
end
