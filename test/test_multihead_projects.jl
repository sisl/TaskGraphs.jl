let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_8(;
        verbose=false);
    mtx = zeros(Int,problem_spec.N+problem_spec.M, problem_spec.M)
    for (j,i) in enumerate(assignments)
        mtx[i,j] = 1
    end
    assignment_dict, assignments = get_assignment_dict(mtx,problem_spec.N,problem_spec.M)
    env = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph)
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver(verbosity=2)

    cost_model = :SumOfMakeSpans
    milp_model = AssignmentMILP()
    solution, assignment, cost, search_env = high_level_search!(
        milp_model, solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;
        cost_model=cost_model);
    path1 = convert_to_vertex_lists(solution.paths[1])
    path2 = convert_to_vertex_lists(solution.paths[2])
    @test path1 == [1, 5, 6, 7, 8, 12, 11, 10, 9]
    @test path2 == [29, 25, 26, 27, 28, 24, 23, 22, 21]
    @test assignments == assignment
    if cost_model == :SumOfMakeSpans
        @test cost[1] == 16
    elseif cost_model == :MakeSpan
        @test cost[1] == 9
    end
end
