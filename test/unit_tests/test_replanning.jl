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
        project_requests, problem_spec, robot_ICs, env_graph = f(;cost_function=cost_model)
        for solver in solvers
            base_env = nothing
            env = nothing
            for (stage,request) in enumerate(project_requests)
                if stage == 1
                    base_env = construct_search_env(solver,request.schedule,problem_spec,env_graph)
                else
                    remap_object_ids!(request.schedule,base_env.schedule)
                    base_env = replan!(solver,replan_model,env,request;commit_threshold=commit_threshold)
                end
                reset_solver!(solver)
                env, cost = solve!(solver,base_env;optimizer=Gurobi.Optimizer)
                # log_info(-1,solver,
                #     "Problem: ",f,"\n",
                #     "Solver: ",typeof(solver),"\n",
                #     "Stage: ",stage,"\n",
                #     "route planner iterations: ", iterations(route_planner(solver)),
                # )
            end
        end
    end
end
