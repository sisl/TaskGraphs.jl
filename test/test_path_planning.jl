let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_1(;verbose=false);

    env = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph)
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver()

    low_level_search!(solver,env,node)
    path1 = convert_to_vertex_lists(node.solution.paths[1])
    path2 = convert_to_vertex_lists(node.solution.paths[2])
    @show path1
    @show path2
end
# Test if the solver can ensure that robots sit and wait when necessary
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_2(;verbose=false);

    env = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph)
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver()

    low_level_search!(solver,env,node)
    path1 = convert_to_vertex_lists(node.solution.paths[2])
    path2 = convert_to_vertex_lists(node.solution.paths[2])
    @show path1
    @show path2
    # @test path1[4] == path1[5] == path1[6] == path1[7] == path1[8] == 13
end
# Test if the solver will ensure that the lower priority robot (robot 1) yields
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_3(;verbose=false);

    env = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph);
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver()

    low_level_search!(solver,env,node);
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
    @test get_cost(node) == aggregate_costs(get_cost_model(pc_mapf.env),get_path_costs(node.solution))
    @test get_cost(node)[1] == maximum(map(path->length(path),get_paths(node.solution)))
    # @test path1[4] == path1[5] == path1[6] == path1[7] == path1[8] == 13
end
let
    # Test CBS wrapper!
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;
        verbose=false);

    env = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph);
    pc_mapf = PC_MAPF(env)
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

    env = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph);
    pc_mapf = PC_MAPF(env)
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
    # high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
    high_level_search!(AssignmentMILP(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer);
    high_level_search!(SparseAdjacencyMILP(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer);
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
    # assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:M);
    assignment_dict, assignments = get_assignment_dict(assignment_matrix,N,M)
    optimal_TA_cost = Int(round(value(objective_function(model))))
    @show optimal_TA_cost;

    # Routing
    project_schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments);
    env = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph);
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(pc_mapf)
    solver = PC_TAPF_Solver(verbosity=1);
    low_level_search!(solver,env,node);
    # @show get_cost(node);
    # paths = convert_to_vertex_lists(node.solution)
    # # @show paths
end
let
    N = 8
    M = 16
    Random.seed!(0);
    project_spec, problem_spec, robot_ICs, env_graph = initialize_test_problem(N,M)
    solver = PC_TAPF_Solver(verbosity=2);
    # high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
    high_level_search!(AssignmentMILP(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer);
    high_level_search!(SparseAdjacencyMILP(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer);
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
    env_graph = factory_env;
    dist_matrix = get_dist_matrix(env_graph);

    r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(factory_env),get_dropoff_zones(factory_env),
            get_free_zones(factory_env))
    project_spec = construct_random_project_spec(M,s0,sF)

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);
    # Solve the problem
    solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=5*nv(env_graph));
    # solution, assignment, cost, search_env = high_level_search!(
    #     solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);

    solution, assignment, cost, search_env = high_level_search!(AssignmentMILP(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer);
    # high_level_search!(SparseAdjacencyMILP(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer);

    robot_paths = convert_to_vertex_lists(solution)
end
# Test prioritized dfs planner
let
    cost_model = MakeSpan
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_1(;verbose=false);

    # milp_model = SparseAdjacencyMILP()
    milp_model = GreedyAssignment()
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    model = formulate_milp(milp_model,project_schedule,problem_spec;cost_model=cost_model)
    optimize!(model)
    adj_matrix = get_assignment_matrix(model)
    update_project_schedule!(milp_model,project_schedule,problem_spec,adj_matrix)

    solver = PC_TAPF_Solver(
        cbs_model = PrioritizedDFSPlanner(),
        astar_model = DFS_PathFinder(),
        l2_verbosity=5
    )
    search_env = construct_search_env(solver, project_schedule, problem_spec, env_graph;primary_objective=cost_model)

    env_graph = search_env.env.graph
    project_schedule = search_env.schedule
    N = search_env.num_agents

    route_plan = deepcopy(search_env.base_solution)
    paths = get_paths(route_plan)
    envs = Vector{PCCBS.LowLevelEnv}([PCCBS.LowLevelEnv() for i in 1:N])
    cbs_node = initialize_root_node(search_env)
    for i in 1:N
        node_id = get_next_node_matching_agent_id(project_schedule,cache,i)
        envs[i], _ = build_env(solver,search_env,cbs_node,get_vtx(project_schedule,node_id))
    end

    for k in 1:2
        if k == 1
            # test with paths of equal length
        elseif k == 2
            # test with paths of different lengths
            update_envs!(solver,search_env,envs,paths)
            push!(paths[1].path_nodes,PathNode(
                PCCBS.State(1,0),
                PCCBS.Action(Edge(1,5),1),
                PCCBS.State(5,1),
            ))
        end
        let
            envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
            @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
            if k == 1
                @test all(map(env->isa(env.schedule_node,GO),envs))
            elseif k == 2
                @test isa(envs[1].schedule_node,CARRY)
                @test isa(envs[2].schedule_node,GO)
            end
            @test get_sp(get_path_node(paths[1],1)) == PCCBS.State(vtx=5,t=1)
            @test get_sp(get_path_node(paths[2],1)) == PCCBS.State(vtx=8,t=1)
            @test get_vtx(search_env.schedule,ObjectID(1)) in search_env.cache.closed_set
            @test get_vtx(search_env.schedule,ObjectID(2)) in search_env.cache.closed_set
            @test get_vtx(search_env.schedule,RobotID(1)) in search_env.cache.closed_set
            @test get_vtx(search_env.schedule,RobotID(2)) in search_env.cache.closed_set
            @test status == false
            map(v->get_node_from_vtx(search_env.schedule,v),collect(search_env.cache.active_set))

            envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
            @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
            @test all(map(env->isa(env.schedule_node,CARRY),envs))
            @test status == false

            envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
            @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
            @test isa(envs[1].schedule_node,CARRY)
            @test isa(envs[2].schedule_node,GO)
            @test status == false

            envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
            @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
            @test isa(envs[1].schedule_node,GO)
            @test isa(envs[2].schedule_node,GO)
            @test status == false

            envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
            @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
            @test isa(envs[1].schedule_node,CARRY)
            @test isa(envs[2].schedule_node,GO)
            @test status == false

            envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
            @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
            @test isa(envs[1].schedule_node,GO)
            @test isa(envs[2].schedule_node,GO)
            @test status == true
        end
    end
end
# Still testing DFS planner
let

    cost_model = MakeSpan
    for (i, f) in enumerate([
                initialize_toy_problem_1,
                initialize_toy_problem_2,
                initialize_toy_problem_3,
                initialize_toy_problem_5,
                initialize_toy_problem_5,
                initialize_toy_problem_6,
                initialize_toy_problem_7,
                initialize_toy_problem_8,
            ])
        project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
        milp_model = GreedyAssignment()
        project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
        model = formulate_milp(milp_model,project_schedule,problem_spec;cost_model=cost_model)
        optimize!(model)
        adj_matrix = get_assignment_matrix(model)
        update_project_schedule!(milp_model,project_schedule,problem_spec,adj_matrix)

        solver = PC_TAPF_Solver(
            cbs_model = PrioritizedDFSPlanner(),
            astar_model = DFS_PathFinder()
        )
        env = construct_search_env(solver, project_schedule, problem_spec, env_graph;primary_objective=cost_model)
        pc_mapf = PC_MAPF(env)

        route_plan, cache, cost = CRCBS.solve!(solver,pc_mapf)
        @test length(cache.active_set) == 0
        @test length(cache.closed_set) == nv(project_schedule)
    end
end
