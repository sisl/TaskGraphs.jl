let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_1(;verbose=false);

    env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph)
    pc_mapf = PC_MAPF(env,mapf)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver()

    low_level_search!(solver,env,mapf,node)
    path1 = convert_to_vertex_lists(node.solution.paths[1])
    path2 = convert_to_vertex_lists(node.solution.paths[2])
    @show path1
    @show path2
end
# Test if the solver can ensure that robots sit and wait when necessary
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_2(;verbose=false);

    env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph)
    pc_mapf = PC_MAPF(env,mapf)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver()

    low_level_search!(solver,env,mapf,node)
    path1 = convert_to_vertex_lists(node.solution.paths[2])
    path2 = convert_to_vertex_lists(node.solution.paths[2])
    @show path1
    @show path2
    # @test path1[4] == path1[5] == path1[6] == path1[7] == path1[8] == 13
end
# Test if the solver will ensure that the lower priority robot (robot 1) yields
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_3(;verbose=false);

    env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph);
    pc_mapf = PC_MAPF(env,mapf)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver()

    low_level_search!(solver,env,mapf,node);
    path1 = convert_to_vertex_lists(node.solution.paths[1])
    path2 = convert_to_vertex_lists(node.solution.paths[2])
    @test path1[2] == 5 # test that robot one will indeed wait for robot 2
    @show path1
    @show path2
    # @test path1 == [5, 5, 6, 7, 8, 12, 12, 12, 12, 12, 16]
    # @test path2 == [2, 6, 10, 14, 18, 22, 26, 30, 31, 32]
    # @show get_cost(node.solution.paths[1])
    # @show get_cost(node.solution.paths[2])
    # @show get_cost(node.solution)
    @test get_cost(node) == aggregate_costs(get_cost_model(mapf.env),get_path_costs(node.solution))
    @test get_cost(node)[1] == maximum(map(path->length(path),get_paths(node.solution)))
    # @test path1[4] == path1[5] == path1[6] == path1[7] == path1[8] == 13
end
let
    # Test CBS wrapper!
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;
        verbose=false);

    env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph);
    pc_mapf = PC_MAPF(env,mapf)
    solver = PC_TAPF_Solver()

    solution, cost = solve!(solver,pc_mapf)
    path1 = convert_to_vertex_lists(solution.paths[1])
    path2 = convert_to_vertex_lists(solution.paths[2])
    # @test (length(path1) == 3) || (length(path2) == 3)
    @show problem_spec.root_nodes
    @show get_cost_model(pc_mapf.env).cost_models[1]
    @test get_cost(solution)[1] == 3
    @show path1
    @show path2
    # @show get_cost(solution.paths[1])
    # @show get_cost(solution.paths[2])
    # @show get_cost(solution)
end
let
    # Test Edge conflict
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_5(;verbose=false);

    env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph);
    pc_mapf = PC_MAPF(env,mapf)
    solver = PC_TAPF_Solver()

    solution, cost = solve!(solver,pc_mapf)
    path1 = convert_to_vertex_lists(solution.paths[1])
    path2 = convert_to_vertex_lists(solution.paths[2])
    # @test (length(path1) == 3) || (length(path2) == 3)
    @show path1
    @show path2
    # @show get_cost(solution.paths[1])
    # @show get_cost(solution.paths[2])
    # @show get_cost(solution)
end
let
    # Test the full loop
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;verbose=false);
    solver = PC_TAPF_Solver(verbosity=1)
    high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
end
let
    N = 8
    M = 16
    Random.seed!(0);
    project_spec, problem_spec, robot_ICs, env_graph = initialize_test_problem(N,M);

    # Task Assignment
    model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);
    optimize!(model);
    optimal = (termination_status(model) == MathOptInterface.OPTIMAL);
    @show optimal;
    assignment_matrix = get_assignment_matrix(model);
    assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:M);
    optimal_TA_cost = maximum(Int.(round.(value.(model[:tof])))); # lower bound on cost (from task assignment module)
    @show optimal_TA_cost;

    # Routing
    project_schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments);
    env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph);
    pc_mapf = PC_MAPF(env,mapf)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver(verbosity=1);
    low_level_search!(solver,env,mapf,node);
    # @show get_cost(node);
    paths = convert_to_vertex_lists(node.solution)
    # @show paths
    repair_solution!(solver,env,mapf,node);
    paths = convert_to_vertex_lists(node.solution)
    # @show paths
    # @show get_cost(node);
end
let
    N = 8
    M = 16
    Random.seed!(0);
    project_spec, problem_spec, robot_ICs, env_graph = initialize_test_problem(N,M)
    solver = PC_TAPF_Solver(verbosity=2);
    high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
end
# let
#     # try a bigger problem!
#     N = 20
#     M = 30
#     Random.seed!(1);
#     project_spec, problem_spec, robot_ICs, env_graph = initialize_test_problem(N,M)
#     solver = PC_TAPF_Solver(verbosity=0,max_A_star_iterations=10*nv(env_graph));
#     solution, assignment, cost, env = high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
#     conflict_table = detect_conflicts(get_paths(solution))
#     @test !CRCBS.is_valid(get_next_conflict(conflict_table))
# end
# let
#     # try a bigger problem!
#     N = 32
#     M = 64
#     Random.seed!(1);
#     project_spec, problem_spec, robot_ICs, env_graph = initialize_test_problem(N,M)
#     solver = PC_TAPF_Solver(verbosity=0,max_A_star_iterations=10*nv(env_graph));
#     solution, assignment, cost, env = high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
#     conflict_table = detect_conflicts(get_paths(solution))
#     @test !CRCBS.is_valid(get_next_conflict(conflict_table))
# end
let
    # try a bigger problem!
    N = 8
    M = 16
    Random.seed!(0);

    filename = string(ENVIRONMENT_DIR,"/env_2.toml");
    factory_env = read_env(filename);
    env_graph = factory_env.graph;
    dist_matrix = get_dist_matrix(env_graph);

    r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(factory_env),get_dropoff_zones(factory_env),
            get_free_zones(factory_env))
    project_spec = construct_random_project_spec(M,s0,sF)

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);
    # Solve the problem
    solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=5*nv(env_graph));
    solution, assignment, cost, search_env = high_level_search!(
        solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);

    robot_paths = convert_to_vertex_lists(solution)

    start_vtxs, start_headings, instructions = convert_solution_to_gridworld_instructions(factory_env,robot_paths)
    initialize_webots_world_file(factory_env, start_vtxs, start_headings, instructions)
end
