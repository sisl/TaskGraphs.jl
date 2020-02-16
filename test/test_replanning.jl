# Freeze assignments
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_6(;
        verbose=false);
    env = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph)
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver(verbosity=0)

    cost_model = SumOfMakeSpans
    solution, assignment, cost, search_env = high_level_search!(
        SparseAdjacencyMILP(),
        solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;
        cost_model=cost_model);
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
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_6(;
        verbose=false);
    # construct a partial project schedule
    construct_project_schedule(project_spec, problem_spec, robot_ICs)
end
