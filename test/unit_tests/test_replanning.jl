# Freeze assignments
let
    replan_model = MergeAndBalance()
    cost_model = SumOfMakeSpans()
    f =  pctapf_problem_6
    project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model);
    project_schedule = construct_partial_project_schedule(
        project_spec,
        problem_spec,
        robot_ICs,
        )
    for solver in [
        NBSSolver(),
        NBSSolver(path_planner = PIBTPlanner{NTuple{3,Float64}}()),
        ]
        # solver = NBSSolver()
        # solver = NBSSolver(path_planner = PIBTPlanner{NTuple{3,Float64}}())
        base_search_env = construct_search_env(
            solver,
            project_schedule,
            problem_spec,
            env_graph,
            )

        @show iteration_limit(solver)
        @show runtime_limit(solver)
        @show deadline(solver)
        env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)

        new_proj_spec, _, _, _, _ = pctapf_problem_1(;verbose=false)
        new_schedule = construct_partial_project_schedule(new_proj_spec,problem_spec)
        request = ProjectRequest(new_schedule,2,2)
        remap_object_ids!(request.schedule,env.schedule)

        commit_threshold = 5
        base_search_env = replan!(solver,replan_model,env,request;commit_threshold=commit_threshold)

        reset_solver!(solver)
        env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
    end

end
# Test partial project schedule
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = pctapf_problem_6(;
        verbose=false);
    # construct a partial project schedule
    construct_partial_project_schedule(project_spec, problem_spec, robot_ICs)
end
