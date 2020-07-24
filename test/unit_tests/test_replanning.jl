# Freeze assignments
let
    project_spec, problem_spec, robot_ICs, _, env_graph = pctapf_problem_6(;
        verbose=false);
    cost_model = SumOfMakeSpans()
    solver = NBSSolver()
    project_schedule = construct_partial_project_schedule(
        project_spec,
        problem_spec,
        robot_ICs,
        )
    base_search_env = construct_search_env(
        solver,
        project_schedule,
        problem_spec,
        env_graph,
        initialize_planning_cache(project_schedule),
        cost_model,
        )
    env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)

    replan_model = MergeAndBalance()

    new_proj_spec, _, _, _, _ = pctapf_problem_1(;verbose=false)


    request = ProjectRequest(
        construct_partial_project_schedule(new_proj_spec,problem_spec),
        2,
        2
    )

    remap_object_ids!(request.schedule,env.schedule)
    replan!(solver,replan_model,env,request;commit_threshold=1)

    # let
    #     t = 0
    #     assignment_dict = freeze_assignments!(search_env.schedule, search_env.cache, t)
    #     @test length(assignment_dict) == 0
    # end
    # let
    #     t = 1
    #     assignment_dict = freeze_assignments!(search_env.schedule, search_env.cache, t)
    #     @test length(assignment_dict) == 1
    # end
    # let
    #     t = 2
    #     assignment_dict = freeze_assignments!(search_env.schedule, search_env.cache, t)
    #     @test length(assignment_dict) == 2
    # end
    # let
    #     t = 8
    #     assignment_dict = freeze_assignments!(search_env.schedule, search_env.cache, t)
    #     @test length(assignment_dict) == 3
    # end
end
# Test partial project schedule
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = pctapf_problem_6(;
        verbose=false);
    # construct a partial project schedule
    construct_partial_project_schedule(project_spec, problem_spec, robot_ICs)
end
