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
    prob = replanning_problem_1(solver)

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
                    base_env = construct_search_env(solver,request.schedule,env.problem_spec,get_graph(env))
                    # base_env = deepcopy(env)
                else
                    remap_object_ids!(request.schedule,base_env.schedule)
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
