let
    id = ActionID(1)
    id += 1
end
let
    Random.seed!(0);
    # Define Environment
    vtx_grid = initialize_dense_vtx_grid(8,8);
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid);
    dist_matrix = get_dist_matrix(env_graph);
    # Define project
    N = 5; M = 10;
    object_ICs = [OBJECT_AT(j,j) for j in 1:M];
    object_FCs = [OBJECT_AT(j,j+M) for j in 1:M];
    robot_ICs = [ROBOT_AT(i,i) for i in 1:N];
    spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3);
    operations = spec.operations;
    problem_spec = TaskGraphProblemSpec(N=N,M=M,D=dist_matrix);
    # Construct Partial Project Schedule
    project_schedule = ProjectSchedule();
    for pred in object_ICs
       add_to_schedule!(project_schedule, problem_spec, pred, get_object_id(pred))
    end
    for pred in robot_ICs
       add_to_schedule!(project_schedule, problem_spec, pred, get_robot_id(pred))
    end
    for op in operations
       operation_id = op.id
       add_to_schedule!(project_schedule, problem_spec, op, operation_id)
    end
    # Fill in gaps in project schedule (except for GO assignments)
    for op in operations
        operation_id = op.id
        for object_id in get_input_ids(op)
            # add action sequence
            object_ic = get_object_ICs(project_schedule)[object_id]
            pickup_station_id = get_id(get_location_id(object_ic))
            object_fc = object_FCs[object_id]
            dropoff_station_id = get_id(get_location_id(object_fc))

            # TODO Handle collaborative tasks
            # if is_single_robot_task(project_spec, object_id)
            robot_id = -1
            add_single_robot_delivery_task!(project_schedule,problem_spec,robot_id,
                object_id,pickup_station_id,dropoff_station_id)
            # elseif is_collaborative_robot_task(project_spec, object_id)
            # end

            action_id = ActionID(get_num_actions(project_schedule))
            add_edge!(project_schedule, action_id, operation_id)
        end
        for object_id in get_output_ids(op)
            add_edge!(project_schedule, operation_id, ObjectID(object_id))
        end
    end
    [v for v in vertices(get_graph(project_schedule)) if typeof(get_node_from_id(project_schedule,get_vtx_id(project_schedule,v)))==CARRY]
    # Formulate MILP problem
    tr0_ = Dict{Int,Float64}()
    to0_ = Dict{Int,Float64}()
    assignments = []
    model = Model(with_optimizer(optimizer,
        TimeLimit=100,
        OutputFlag=0
        ))
    NV = nv(get_graph(project_schedule))
    @variable(t0[1:NV] >= 0.0) # initial times for all nodes
    @variable(tF[1:NV] >= 0.0) # final times for all nodes
    Δt = map(v->get_path_spec(project_schedule, v).min_path_duration, vertices(get_graph(project_schedule)))
    # @variable(model, to0[1:M] >= 0.0) # object availability time
    # @variable(model, tor[1:M] >= 0.0) # object robot arrival time
    # @variable(model, toc[1:M] >= 0.0) # object collection complete time
    # @variable(model, tod[1:M] >= 0.0) # object deliver begin time
    # @variable(model, tof[1:M] >= 0.0) # object termination time
    # @variable(model, tr0[1:N+M] >= 0.0) # robot availability time

    # Assignment matrix x
    @variable(model, x[1:N+M,1:M], binary = true) # x[i,j] ∈ {0,1}
    @constraint(model, x * ones(M) .<= 1)         # each robot may have no more than 1 task
    @constraint(model, x' * ones(N+M) .== nR)     # each task must have exactly 1 assignment
    # for (i,t) in tr0_
    #     # start time for robot i
    #     @constraint(model, tr0[i] == t)
    # end
    # for (j,t) in to0_
    #     # start time for task j (applies only to tasks with no prereqs)
    #     @constraint(model, to0[j] == t)
    # end
    # for (i,j) in assignments
    #     # start time for task j (applies only to tasks with no prereqs)
    #     @constraint(model, x[i,j] == 1)
    # end
    # constraints
    Mm = 10000 # for big-M constraints
    for v in vertices(get_graph(project_schedule))
        @constraint(model, tF[v] >= t0[v] + Δt[v])
        for v2 in inneighbors(get_graph(project_schedule),v)
            @constraint(model, t0[v] >= tof[v])
        end
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        if typeof(node) <: GO && (get_path_spec(project_schedule, v).dummy_id == -1)
            # TODO add big M assignment constraints
            j = # How to easily get j?
            for (i,pred) in get_robot_ICs(project_schedule)
                v2 = get_vtx(project_schedule, get_robot_id(pred))
                start_idx = get_location_id(pred)
                end_idx = get_destination_location_id(node)
                dt_min = problem_spec.D[start_idx,end_idx]
                # @constraint(model, tor[j] - (t0[v2] + dt_min) >= -Mm*(1 - x[i,j]))
            end
        end
    end
    for j in 1:M
        # constraint on task start time
        if !is_root_node(G,j)
            for v in inneighbors(G,j)
                @constraint(model, to0[j] >= tof[v] + Δt[j])
            end
        end
        # constraint on dummy robot start time (corresponds to moment of object delivery)
        @constraint(model, tr0[j+N] == tof[j])
        # dummy robots can't do upstream jobs
        upstream_jobs = [j, map(e->e.dst,collect(edges(bfs_tree(G,j;dir=:in))))...]
        for v in upstream_jobs
            @constraint(model, x[j+N,v] == 0)
        end
        # lower bound on task completion time (task can't start until it's available).
        # tof[j] = to0[j] + Dss[j,j] + slack[j]
        @constraint(model, tor[j] >= to0[j])
        # @constraint(model, tof[j] >= tor[j] + Dss[j,j] + Δt_collect[j] + Δt_deliver[j])
        # bound on task completion time (assigned robot must first complete delivery)
        # Big M constraint (thanks Oriana!): When x[i,j] == 1, this constrains the final time
        # to be no less than the time it takes for the delivery to be completed by robot i.
        # When x[i,j] == 0, this constrains the final time to be greater than a large negative
        # number (meaning that this is a trivial constraint)
        for i in 1:N+M
            @constraint(model, tor[j] - (tr0[i] + Drs[i,j]) >= -Mm*(1 - x[i,j]))
        end
        @constraint(model, toc[j] == tor[j] + Δt_collect[j])
        @constraint(model, tod[j] == toc[j] + Dss[j,j])
        @constraint(model, tof[j] == tod[j] + Δt_deliver[j])
        # "Job-shop" constraints specifying that no station may be double-booked. A station
        # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
        # the windows for these operations cannot overlap. In the constraints below, t1 and t2
        # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
        # respectively. If eny of the operations for these two tasks require use of the same
        # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
        # j must occur before the operation for task j2. The opposite is true for y == [0,1].
        # We use the big M method here as well to tightly enforce the binary constraints.
        for j2 in j+1:M
            if (s0[j] == s0[j2]) || (s0[j] == sF[j2]) || (sF[j] == s0[j2]) || (sF[j] == sF[j2])
                # @show j, j2
                if s0[j] == s0[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                tmax = @variable(model)
                tmin = @variable(model)
                y = @variable(model, binary=true)
                @constraint(model, tmax >= t1[1])
                @constraint(model, tmax >= t2[1])
                @constraint(model, tmin <= t1[2])
                @constraint(model, tmin <= t2[2])

                @constraint(model, tmax - t2[1] <= (1 - y)*Mm)
                @constraint(model, tmax - t1[1] <= y*Mm)
                @constraint(model, tmin - t1[2] >= (1 - y)*-Mm)
                @constraint(model, tmin - t2[2] >= y*-Mm)
                @constraint(model, tmin + 1 <= tmax)
            end
        end
    end
    # cost depends only on root node(s)
    if cost_model == :SumOfMakeSpans
        @variable(model, T[1:length(root_nodes)])
        for (i,project_head) in enumerate(root_nodes)
            for v in project_head
                @constraint(model, T[i] >= tof[v])
            end
        end
        @objective(model, Min, sum(map(i->T[i]*get(weights,i,0.0), 1:length(root_nodes))))
        # @objective(model, Min, sum(map(v->tof[v]*get(weights,v,0.0), root_nodes)))
    elseif cost_model == :MakeSpan
        @variable(model, T)
        @constraint(model, T .>= tof)
        @objective(model, Min, T)
    end
    model;
end
let
    M = 3
    object_ICs = Vector{OBJECT_AT}([
        OBJECT_AT(1,1),
        OBJECT_AT(2,2),
        OBJECT_AT(3,3)
        ])
    object_FCs = Vector{OBJECT_AT}([
        OBJECT_AT(1,4),
        OBJECT_AT(2,5),
        OBJECT_AT(3,6)
        ])
    robot_ICs = Dict{Int,ROBOT_AT}(
        1=>ROBOT_AT(1,7),
        2=>ROBOT_AT(2,8),
        3=>ROBOT_AT(3,9)
        )
    # Testing root nodes
    let
        project_spec = ProjectSpec(initial_conditions=object_ICs,final_conditions=object_FCs)
        add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [3], 1.0))
        @test project_spec.root_nodes == Set{Int}([1])
        add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
        @test project_spec.root_nodes == Set{Int}([2])
    end
    let
        project_spec = ProjectSpec(initial_conditions=object_ICs,final_conditions=object_FCs)
        add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [], 1.0))
        @test project_spec.root_nodes == Set{Int}([1])
        add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
        @test project_spec.root_nodes == Set{Int}([1,2])
    end
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

    model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);
    optimize!(model)
    @test termination_status(model) == MathOptInterface.OPTIMAL

    assignment_matrix = get_assignment_matrix(model)
    assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:M)
    @test assignments == optimal_assignments

    project_schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments);

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
# combining two project specs
let
    Random.seed!(0)
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

    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial_conditions
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    # Drs, Dss = cached_pickup_and_delivery_distances(pts[r0],pts[s0],pts[sF])
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    object_ICs1 = Vector{OBJECT_AT}([object_ICs[o] for o in 1:Int(M/2)])
    object_FCs1 = Vector{OBJECT_AT}([object_FCs[o] for o in 1:Int(M/2)])
    project_spec1 = construct_random_project_spec(Int(M/2),object_ICs1,object_FCs1;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    object_FCs2 = Vector{OBJECT_AT}([object_FCs[o] for o in Int(M/2)+1:M])
    object_ICs2 = Vector{OBJECT_AT}([object_ICs[o] for o in Int(M/2)+1:M])
    project_spec2 = construct_random_project_spec(Int(M/2),object_ICs2,object_FCs2;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    project_spec = combine_project_specs([project_spec1, project_spec2])

    delivery_graph = construct_delivery_graph(project_spec,M)

    filename = "project_spec.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(project_spec))
    end
    project_spec_mod = read_project_spec(filename)
    @test project_spec_mod == project_spec
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

    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial_conditions
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    # Drs, Dss = cached_pickup_and_delivery_distances(pts[r0],pts[s0],pts[sF])
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    object_ICs1 = Vector{OBJECT_AT}([object_ICs[o] for o in 1:Int(M/2)])
    object_FCs1 = Vector{OBJECT_AT}([object_FCs[o] for o in 1:Int(M/2)])
    project_spec1 = construct_random_project_spec(Int(M/2),object_ICs1,object_FCs1;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    object_FCs2 = Vector{OBJECT_AT}([object_FCs[o] for o in Int(M/2)+1:M])
    object_ICs2 = Vector{OBJECT_AT}([object_ICs[o] for o in Int(M/2)+1:M])
    project_spec2 = construct_random_project_spec(Int(M/2),object_ICs2,object_FCs2;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
    project_spec = combine_project_specs([project_spec1, project_spec2])

    filename = "project_spec.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(project_spec))
    end
    project_spec = read_project_spec(filename)

    problem_def = SimpleProblemDef(project_spec,r0,s0,sF)
    filename = "problem_def.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(problem_def))
    end
    problem_def = read_problem_def(filename)

    for spec in [project_spec1, project_spec2, project_spec]
        let
            project_spec = spec
            delivery_graph = construct_delivery_graph(project_spec,M)
            project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix)
            model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);
            optimize!(model)
            @test termination_status(model) == MathOptInterface.OPTIMAL
            assignment = get_assignment_matrix(model);
            assignments = map(j->findfirst(assignment[:,j] .== 1),1:M)
            for r in N+1:N+M
                robot_ICs[r] = ROBOT_AT(r,sF[r-N])
            end
            project_schedule = construct_project_schedule(project_spec, problem_spec, object_ICs, object_FCs, robot_ICs, assignments);
            o_keys = Set(collect(keys(get_object_ICs(project_schedule))))
            input_ids = union([get_input_ids(op) for (k,op) in get_operations(project_schedule)]...)
            @test o_keys == input_ids
            rg = get_display_metagraph(project_schedule)
        end
    end
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

    model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;OutputFlag=0);

    optimize!(model)
    @test termination_status(model) == MathOptInterface.OPTIMAL

    assignment_matrix = get_assignment_matrix(model);
    assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:M)
    project_schedule = construct_project_schedule(project_spec, problem_spec, object_ICs, object_FCs, robot_ICs, assignments);

    o_keys = Set(collect(keys(get_object_ICs(project_schedule))))
    input_ids = union([get_input_ids(op) for (k,op) in get_operations(project_schedule)]...)
    @test o_keys == input_ids
end
