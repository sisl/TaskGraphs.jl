let
    id = ActionID(1)
    id += 1
end
let
    M = 3
    object_ICs = Dict{Int,OBJECT_AT}(
        1=>OBJECT_AT(1,1),
        2=>OBJECT_AT(2,2),
        3=>OBJECT_AT(3,3)
        )
    object_FCs = Dict{Int,OBJECT_AT}(
        1=>OBJECT_AT(1,4),
        2=>OBJECT_AT(2,5),
        3=>OBJECT_AT(3,6)
        )
    robot_ICs = Dict{Int,ROBOT_AT}(
        1=>ROBOT_AT(1,7),
        2=>ROBOT_AT(2,8),
        3=>ROBOT_AT(3,9)
        )
    project_spec = ProjectSpec(initial_conditions=object_ICs,final_conditions=object_FCs)
    add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [3], 1.0))
    add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
    delivery_graph = construct_delivery_graph(project_spec,M)
    problem_spec = TaskGraphProblemSpec()
end
let
    G = DiGraph(3)
    add_edge!(G,1,2)
    @test get_all_root_nodes(G) == Set([2,3])
end
# Simple Hand Crafted Problem
let
    project_spec, problem_spec, robot_ICs, optimal_assignments, env_graph = initialize_toy_problem_1()
    N = problem_spec.N
    M = problem_spec.M

    model = formulate_JuMP_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);
    optimize!(model)
    optimal = (termination_status(model) == MathOptInterface.TerminationStatusCode(1))
    @show optimal;
    cache = SearchCache(problem_spec)
    cache.x .= Matrix{Int}(value.(model[:x]))
    cache = process_solution(model,cache,problem_spec)
    assignments = map(j->findfirst(cache.x[:,j] .== 1),1:M)
    @test assignments == optimal_assignments
    @test cache.to0 == [0,0,3]
    @test cache.tof == [3,2,5]
    @test cache.slack == [0,1,0]
    # @show assignments
    # @show cache.to0
    # @show cache.tof
    # @show cache.tr0
    # @show cache.slack
    # @show cache.local_slack
    project_schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments);
    # @show project_schedule.path_id_to_vtx_map
    t0,tF,slack,local_slack = process_schedule(project_schedule)
    @test t0[get_vtx(project_schedule,RobotID(1))] == 0
    @test t0[get_vtx(project_schedule,RobotID(2))] == 0
    @test t0[get_vtx(project_schedule,RobotID(3))] == 3

    # try with perturbed start times
    t0[get_vtx(project_schedule,RobotID(2))] = 1
    t0,tF,slack,local_slack = process_schedule(project_schedule;t0=t0)
    @test t0[get_vtx(project_schedule,RobotID(1))] == 0
    @test t0[get_vtx(project_schedule,RobotID(2))] == 1
    @test t0[get_vtx(project_schedule,RobotID(3))] == 3

    @test length(get_vtx_ids(project_schedule)) == nv(get_graph(project_schedule))
    for (v,id) in enumerate(project_schedule.vtx_ids)
        @test get_vtx(project_schedule, id) == v
    end
end

let
    N = 4                  # num robots
    M = 6                  # num delivery tasks
    # env_graph, vtx_grid = initialize_grid_graph_with_obstacles([10,10]);
    env_graph = initialize_grid_graph_from_vtx_grid(initialize_dense_vtx_grid(4,4))
    # N = 40                  # num robots
    # M = 60                  # num delivery tasks
    # env_graph, vtx_grid = initialize_grid_graph_with_obstacles([50,50]);
    pickup_zones = collect(1:M)
    dropoff_zones = collect(M+1:2*M)
    free_zones = collect(2*M+1:nv(env_graph))
    dist_matrix = get_dist_matrix(env_graph)

    r0 = free_zones[1:N]
    s0 = pickup_zones[1:M]
    sF = dropoff_zones[1:M]
    # r0,s0,sF = get_random_problem_instantiation(
    #     N,M,pickup_zones,dropoff_zones,free_zones)

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    # Drs, Dss = cached_pickup_and_delivery_distances(pts[r0],pts[s0],pts[sF])
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    object_ICs1 = Dict{Int,OBJECT_AT}(o => object_ICs[o] for o in 1:Int(M/2))
    object_FCs1 = Dict{Int,OBJECT_AT}(o => object_FCs[o] for o in 1:Int(M/2))
    project_spec1 = construct_random_project_spec(Int(M/2),object_ICs1,object_FCs1;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    object_ICs2 = Dict{Int,OBJECT_AT}(o => object_ICs[o] for o in Int(M/2):M)
    object_FCs2 = Dict{Int,OBJECT_AT}(o => object_FCs[o] for o in Int(M/2):M)
    project_spec2 = construct_random_project_spec(Int(M/2),object_ICs2,object_FCs2;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    project_spec = combine_project_specs([project_spec1, project_spec2])

    filename = "project_spec.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(project_spec))
    end
    project_spec = read_project_spec(filename)

    delivery_graph = construct_delivery_graph(project_spec,M)
    G = delivery_graph.graph
    # initialize vector of operation times
    Δt = get_duration_vector(project_spec)
    # set initial conditions
    to0_ = Dict{Int,Float64}()
    for v in vertices(G)
        if is_leaf_node(G,v)
            to0_[v] = 0.0
        end
    end
    tr0_ = Dict{Int,Float64}()
    for i in 1:N
        tr0_[i] = 0.0
    end
    problem_spec = TaskGraphProblemSpec(N,M,G,dist_matrix,Drs,Dss,Δt,tr0_,to0_)
    # model = formulate_JuMP_optimization_problem(G,Drs,Dss,Δt,to0_,tr0_,Gurobi.Optimizer;OutputFlag=0);
    model = formulate_JuMP_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);

    optimize!(model)
    optimal = (termination_status(model) == MathOptInterface.TerminationStatusCode(1))
    @show optimal;
    assignment = Matrix{Int}(value.(model[:x]));

    cache = SearchCache(N,M,to0_,tr0_)
    for j in 1:M
        i = findfirst(assignment[:,j] .== 1)
        cache.x[i,j] = 1
    end
    solution_graph = construct_solution_graph(delivery_graph.graph,assignment)
    cache = process_solution(model,cache,problem_spec);

    assignments = map(j->findfirst(cache.x[:,j] .== 1),1:M)

    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    project_schedule = construct_project_schedule(project_spec, problem_spec, object_ICs, object_FCs, robot_ICs, assignments);

    o_keys = Set(collect(keys(get_object_ICs(project_schedule))))
    input_ids = union([get_input_ids(op) for (k,op) in get_operations(project_schedule)]...)
    @test o_keys == input_ids

    rg = get_display_metagraph(project_schedule)
end

let
    N = 4                  # num robots
    M = 6                  # num delivery tasks
    env_graph = initialize_grid_graph_from_vtx_grid(initialize_dense_vtx_grid(4,4))
    dist_matrix = get_dist_matrix(env_graph)
    pickup_zones = collect(1:M)
    dropoff_zones = collect(M+1:2*M)
    free_zones = collect(2*M+1:nv(env_graph))

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_random_task_graphs_problem(
        N,M,pickup_zones,dropoff_zones,free_zones,dist_matrix)

    model = formulate_JuMP_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);

    optimize!(model)
    optimal = (termination_status(model) == MathOptInterface.TerminationStatusCode(1))
    @show optimal;
    assignment_matrix = Matrix{Int}(value.(model[:x]));
    assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:M)
    project_schedule = construct_project_schedule(project_spec, problem_spec, object_ICs, object_FCs, robot_ICs, assignments);

    o_keys = Set(collect(keys(get_object_ICs(project_schedule))))
    input_ids = union([get_input_ids(op) for (k,op) in get_operations(project_schedule)]...)
    @test o_keys == input_ids
end
