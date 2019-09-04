export
    get_root_node,
    get_bfs_node_traversal,
    get_all_root_nodes,
    get_bfs_traversal,
    initialize_random_2D_task_graph_env,
    cached_pickup_and_delivery_distances,
    formulate_optimization_problem,
    formulate_JuMP_optimization_problem,
    construct_factory_distance_matrix,
    construct_random_project_spec,
    combine_project_specs,
    compute_lower_time_bound

"""
    `get_root_node(G,v=1)`
"""
function get_root_node(G,v=1)
    node_list = map(e->e.dst, collect(edges(dfs_tree(G,v))))
    root_node = get(node_list, length(node_list), v)
end

"""
    `get_bfs_node_traversal(G,root_node=-1)`

    Gets a BFS traversal beginning from a particular root node.
"""
function get_bfs_node_traversal(G,root_node=-1)
    # root_node = map(e->e.dst, collect(edges(dfs_tree(G,1))))[end]
    if root_node < 0
        root_node = get_root_node(G)
    end
    bfs_traversal = map(e->e.dst, collect(edges(bfs_tree(G,root_node;dir=:in))))
    append!(bfs_traversal, root_node)
    bfs_traversal
end

"""
    `get_all_root_nodes(G)`
"""
function get_all_root_nodes(G)
    frontier = Set{Int}(collect(vertices(G)))
    root_nodes = Set{Int}()
    while length(frontier) > 0
        v = pop!(frontier)
        root_node = get_root_node(G,v)
        for v2 in get_bfs_node_traversal(G,root_node)
            setdiff!(frontier, v2)
        end
        push!(root_nodes, root_node)
    end
    return root_nodes
end

"""
    `get_bfs_traversal(G)`

    Returns a full bfs traversal, even if the graph is disjoint
"""
function get_bfs_traversal(G)
    traversal = Vector{Int}()
    for v in get_all_root_nodes(G)
        traversal = [traversal..., get_bfs_node_traversal(G,v)...]
    end
    traversal
end

"""
    initialize_random_2D_task_graph_env(G,N;d=[20,20])

    Inputs:
        `N` - number of agents
        `M` - number of tasks
        `vtxs` - a list of vertex coordinates
        `d` = [20,20] - dimensions of factor floor

    Outputs:
        `r₀` - indices of initial robot positions
        `s₀` - indices of initial object locations
        `sₜ` - indices of destination object locations
"""
function initialize_random_2D_task_graph_env(N,M;vtxs=nothing,d=[20,20])
    if vtxs == nothing
        k = 1
        x₀ = Vector{Vector{Int}}() # all possible grid locations
        for i in 1:d[1]
            for j in 1:d[2]
                push!(x₀,Vector{Int}([i,j]))
            end
        end
    else
        x₀ = vtxs
    end
    ##### Random Problem Initialization #####
    # x₀ = x₀[sortperm(rand(length(x₀)))]
    vtxs = sortperm(rand(length(x₀)))
    # initial robot locations
    r₀ = Vector{Int}(undef, N)
    for i in 1:N
        # r₀[i] = pop!(x₀)
        r₀[i] = pop!(vtxs)
    end
    # initial object locations - somewhere in factory
    s₀ = Vector{Int}(undef, M)
    for i in 1:M
        # s₀[i] = pop!(x₀)
        s₀[i] = pop!(vtxs)
    end
    # final object locations - depends on where the child objects "appear"
    sₜ = Vector{Int}(undef, M)
    for i in 1:M
        # if length(outneighbors(G,v)) > 0
        #     v2 = outneighbors(G,v)[1]
        #     sₜ[v] = s₀[v2]
        # else
        # sₜ[i] = pop!(x₀)
        sₜ[i] = pop!(vtxs)
        # end
    end
    r₀,s₀,sₜ,x₀
end

"""
    `cached_pickup_and_delivery_distances(r₀,oₒ,sₒ,dist=(x1,x2)->norm(x2-x1,1))`

    Inputs:
        `r₀` - vector of initial robot locations.
        `sₒ` - vector of initial object locations.
        `sₜ` - vector of station locations (object i must be brough to station i
            from its initial location)

    Outputs:
        `Drs` - distance from initial robot locations (including dummies) to
            object pickup locations
        `Dss` - distance from pickup stations to delivery stations (only the
            diagonal) is relevant for our problem
"""
function cached_pickup_and_delivery_distances(r₀,s₀,sₜ,dist=(x1,x2)->norm(x2-x1,1))
    N = size(r₀,1)
    M = size(s₀,1)
    # augment r₀ to include "dummy" robots that appear after dropoff
    r₀ = [r₀;s₀]
    # Construct distance matrix
    Drs = zeros(N+M,M) # distance robot to pickup station
    for i in 1:N+M
        for j in 1:M
            Drs[i,j] = dist(r₀[i],s₀[j])
        end
    end
    Dss = zeros(M,M) # distance robot to delivery station
    for i in 1:M
        for j in 1:M
            # distance from dummy robot to object + object to station
            Dss[i,j] = dist(s₀[i],sₜ[j])
        end
    end
    return Drs, Dss
end

"""
    `formulate_optimization_problem(G,Drs,Dss,Δt)`

    *** TODO: This function is still partially pseudocode. ***

    Inputs:
        `G` - graph with inverted tree structure that encodes dependencies
            between tasks
        `Drs` - Drs[i,j] is distance from initial robot position i to pickup
            station j
        `Dss` - Dss[j,j] is distance from start station j to final station j (we
            only care about the diagonal)
        `Δt` - Δt[j] is the duraction of time that must elapse after all prereqs
            of task j have been satisfied before task j becomes available
        `to0_` - a `Dict`, where `to0_[j]` gives the start time for task j
            (applies to leaf tasks only)
        `tr0_` - a `Dict`, where `tr0_[i]` gives the start time for robot i
            (applies to non-dummy robots only)

    Outputs:
        `model` - an Optimization problem
"""
function formulate_optimization_problem(G,Drs,Dss,Δt,to0_,tr0_)
    #TODO need to provide known start times for leaf tasks and non-dummy robots
    M = size(Dss,1)
    N = size(Drs,1)-M
    # Optimization variables
    x = zeros(Int, M)  # x[j] = i means that agent i is assigned to task j
    # Helper variables
    to0 = zeros(M)     # to0[j] = available start time for task j (already known for leaf nodes)
    tof = zeros(M)     # tof[j] = completion time for task j
    tr0 = zeros(N+M)   # tr0[i] = available start time for robot i
    # initial conditions
    for (i,t) in tr0_
        # start time for robot i
        tr0[i] = t
    end
    for (j,t) in to0_
        # start time for task j (applies only to tasks with no prereqs)
        to0[j] = t
    end
    # constraints
    for j in 1:M
        # constraint on task start time
        if !is_leaf_node(G,j)
            to0[j] == maximum(tof[inneighbors(G,j)]) + Δt[j]
        end
        # constraint on dummy robot start time
        tr0[j+N] == tof[j]
        for i in 1:N
            if x[i] == j
                # constraint on task completion time
                tof[j] == max(to0[j], tr0[i] + Drs[i,j]) + Dss[j,j]
            end
        end
    end
end

"""
    Express the TaskGraphs assignment problem as a MILP using the JuMP optimization
    framework.

    `formulate_JuMP_optimization_problem(G,Drs,Dss,Δt,to0_,tr0_,optimizer)`

    Inputs:
        `G` - graph with inverted tree structure that encodes dependencies
            between tasks
        `Drs` - Drs[i,j] is distance from initial robot position i to pickup
            station j
        `Dss` - Dss[j,j] is distance from start station j to final station j (we
            only care about the diagonal)
        `Δt` - Δt[j] is the duraction of time that must elapse after all prereqs
            of task j have been satisfied before task j becomes available
        `to0_` - a `Dict`, where `to0_[j]` gives the start time for task j
            (applies to leaf tasks only)
        `tr0_` - a `Dict`, where `tr0_[i]` gives the start time for robot i
            (applies to non-dummy robots only)
        `optimizer` - a JuMP optimizer (e.g., Gurobi.optimizer)

    Outputs:
        `model` - the optimization model
"""
function formulate_JuMP_optimization_problem(G,Drs,Dss,Δt,to0_,tr0_,optimizer;
    TimeLimit=50,
    OutputFlag=0
    )

    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag
        ))
    M = size(Dss,1)
    N = size(Drs,1)-M
    @variable(model, to0[1:M] >= 0.0)
    @variable(model, tof[1:M] >= 0.0)
    @variable(model, tr0[1:N+M] >= 0.0)

    # Assignment matrix x
    @variable(model, x[1:N+M,1:M], binary = true) # x[i,j] ∈ {0,1}
    @constraint(model, x * ones(M) .<= 1)         # each robot may have no more than 1 task
    @constraint(model, x' * ones(N+M) .== 1)      # each task must have exactly 1 assignment
    for (i,t) in tr0_
        # start time for robot i
        @constraint(model, tr0[i] == t)
    end
    for (j,t) in to0_
        # start time for task j (applies only to tasks with no prereqs)
        @constraint(model, to0[j] == t)
    end
    # constraints
    Mm = Matrix{Float64}(I,N+M,N+M) * (sum(Drs) + sum(Dss)) # for big-M constraints
    for j in 1:M
        # constraint on task start time
        if !is_leaf_node(G,j)
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
        @constraint(model, tof[j] >= to0[j] + Dss[j,j])
        # bound on task completion time (assigned robot must first complete delivery)
        # Big M constraint (thanks Oriana!): When x[i,j] == 1, this constrains the final time
        # to be no less than the time it takes for the delivery to be completed by robot i.
        # When x[i,j] == 0, this constrains the final time to be greater than a large negative
        # number (meaning that this is a trivial constraint)
        @constraint(model, tof[j] .- (tr0 + Drs[:,j] .+ Dss[j,j]) .>= -Mm*(1 .- x[:,j]))
    end
    # cost depends only on root node(s)
    # @objective(model, Min, tof[M])
    @objective(model, Min, sum(map(v->tof[v], collect(get_all_root_nodes(G)))))
    model;
end

"""
    `construct_factory_distance_matrix(r₀,oₒ,sₒ;dist::Function=(x1,x2)->norm(x2-x1,1))`

    Inputs: `r₀` - vector of initial robot locations. `sₒ` - vector of initial
    object locations. `sₜ` - vector of station locations (object i must be
    brough to station i from its initial location)
"""
function construct_factory_distance_matrix(r₀,s₀,sₜ;dist::Function=(x1,x2)->norm(x2-x1,1))
    N = size(r₀,1); M = size(s₀,1)
    # Construct distance matrix
    Drr = zeros(N,N) # distance robot to robot (not relevant)
    for i in 1:N
        for j in 1:N
            Drr[i,j] = dist(r₀[i],r₀[j])
        end
    end
    Drs = zeros(N,M) # distance robot to delivery completion
    for i in 1:N
        for j in 1:M
            # distance from robot to object + object to station
            Drs[i,j] = dist(r₀[i],s₀[j]) + dist(s₀[j],sₜ[j])
        end
    end
    Dss = zeros(M,M) # distance dummy robot to delivery completion
    for i in 1:M
        for j in 1:M
            # distance from dummy robot to object + object to station
            Dss[i,j] = dist(sₜ[i],s₀[j]) + dist(s₀[j],sₜ[j])
        end
    end
    D = [Drr Drs; Drs' Dss]
end

"""
    construct_random_project_spec(M::Int;max_children=1)

    Inputs:
        `M` - number of objects involved in the operation
        `max_parents` - determines the max number of inputs to any operation
        `depth_bias` ∈ [0,1] - hyperparameter for tuning depth.
            If `depth_bias` == 1.0, the project_spec graph will always be depth
            balanced (all paths through the tree will be of the same length).
            For `depth_bias` == 0.0, the graph will be as "strung out" as
            possible.
"""
function construct_random_project_spec(M::Int,object_ICs::Dict{Int,OBJECT_AT}=Dict{Int,OBJECT_AT}();
    max_parents=1,depth_bias=1.0,Δt_min::Int=0,Δt_max::Int=0)
    project_spec = ProjectSpec(
        M=M,
        initial_conditions=object_ICs
        )
    # fill with random operations going backwards
    i = M-1
    frontier = PriorityQueue{Int,Int}([M=>1])
    while i > 0
        depth = 1
        while true
            if (rand() > depth_bias) && (depth < length(frontier))
                depth += 1
            else
                break
            end
        end
        pairs = Vector{Pair{Int,Int}}()
        # pair = Pair{Int,Int}(0,0)
        for d in 1:depth
            push!(pairs, peek(frontier))
            dequeue!(frontier)
        end
        for p in pairs[1:end-1]
            enqueue!(frontier,p)
        end
        output_id = pairs[end].first
        station_id = output_id
        input_ids = collect(max(1,1+i-rand(1:max_parents)):i)
        i = i - length(input_ids)
        # Δt = Δt_min + (Δt_max-Δt_min)*rand()
        Δt=rand(Δt_min:Δt_max)
        # add_operation!(project_spec,construct_operation(station_id, input_ids, [output_id], Δt))
        add_operation!(project_spec,construct_operation(project_spec, station_id, input_ids, [output_id], Δt))
        for id in input_ids
            enqueue!(frontier, id=>M-i)
        end
    end
    project_spec
end
function construct_random_project_spec(M::Int,object_ICs::Vector{OBJECT_AT};
    max_parents=1,depth_bias=1.0,Δt_min::Int=0,Δt_max::Int=0)
    object_IC_dict = Dict{Int,OBJECT_AT}(get_id(get_o(pred))=>pred for pred in object_ICs)
    construct_random_project_spec(M,object_IC_dict;
        max_parents=max_parents,depth_bias=depth_bias,Δt_min=Δt_min,Δt_max=Δt_max)
end

"""
    `randomize_project_spec_stations(spec::P, n_stations::Int)  where P <: ProjectSpec`

    Change the object and station ids
"""
function randomize_project_spec_stations(spec::P, n_stations::Int)  where P <: ProjectSpec
    new_spec = ProjectSpec()
    station_idxs = sortperm(rand(n_pickup_zones))
    for op in spec.operations
        new_op = Operation(Δt = op.Δt)
        for pred in preconditions(op)
            push!(new_op.pre, OBJECT_AT(get_o(pred), station_idxs[get_s(pred)]))
        end
        for pred in postconditions(op)
            push!(new_op.post, OBJECT_AT(get_o(pred), station_idxs[get_s(pred)]))
        end
        add_operation!(new_spec, new_op)
    end
    new_spec
end

"""
    `combine_project_specs(specs::Vector{ProjectSpec})`

    A helper for combining multiple `ProjectSpec`s into a single
    ProjectSpec.
"""
function combine_project_specs(specs::Vector{P} where P <: ProjectSpec)
    M = 0
    new_spec = ProjectSpec(
        M=sum(map(spec->spec.M, specs)),
        initial_conditions=merge(map(spec->spec.initial_conditions, specs)...)
        )
    for spec in specs
        for op in spec.operations
            new_op = Operation(station_id=op.station_id, Δt = op.Δt)
            for pred in preconditions(op)
                push!(new_op.pre, OBJECT_AT(get_o(pred)+M, get_s(pred)+M))
            end
            for pred in postconditions(op)
                push!(new_op.post, OBJECT_AT(get_o(pred)+M, get_s(pred)+M))
            end
            add_operation!(new_spec, new_op)
        end
        M = M + spec.M
    end
    new_spec
end

"""
    compute_lower_time_bound(G,D,Δt)

    Given a task graph planning problem, computes a lower bound on completion
    time for each task.

    Inputs:
        `G` - The task graph. Encodes dependencies between the `M` atomic tasks
        `D` - The distance matrix. D[i,j] specifies the time required for agent
            i to accomplish task j
        `Δt`- The process time vector. Δt[j] encodes the amount of time required
            for task j to become available after all prereqs of j are satisfied

    Outputs:
        `t_low` ∈ Rᴹ - lower bound on completion time for each task
"""
function compute_lower_time_bound(G,D,Δt)
    M = nv(G)
    N = size(D,1) - M
    # split distance matrix into appropriate blocks
    Drr = D[1:N,1:N]            # distance robot to robot
    Drs = D[1:N,N+1:N+M]        # distance robot to delivery completion
    Dsr = Drs'                  # transpose of above
    Dss = D[N+1:N+M,N+1:N+M]    # distance dummy robot to delivery completion
    # compute lower bound
    t_low = zeros(nv(G))
    root_node = map(e->e.dst, collect(edges(dfs_tree(G,1))))[end]
    bfs_traversal = bfs_tree(G,root_node;dir=:in)
    ids = map(e->e.dst, collect(edges(bfs_traversal)))
    append!(ids, root_node)
    for v in ids
        t_low[v] = Δt[v] + minimum(Drs[:,v])
        for v2 in inneighbors(G,v)
            t_low[v] = max(t_low[v], t_low[v2] + Dss[v2,v] + Δt[v]) # + minimum(Dss[inneighbors(G,v),v]))
        end
    end
    t_low
end

function compute_lower_time_bound_and_slack(G,D,Δt)
    M = nv(G)
    N = size(D,1) - M
    # split distance matrix into appropriate blocks
    Drr = D[1:N,1:N]            # distance robot to robot
    Drs = D[1:N,N+1:N+M]        # distance robot to delivery completion
    Dsr = Drs'                  # transpose of above
    Dss = D[N+1:N+M,N+1:N+M]    # distance dummy robot to delivery completion
    # compute lower bound
    t_low = zeros(nv(G))
    slack = zeros(nv(G))
    root_node = map(e->e.dst, collect(edges(dfs_tree(G,1))))[end]
    bfs_traversal = bfs_tree(G,root_node;dir=:in)
    ids = map(e->e.dst, collect(edges(bfs_traversal)))
    append!(ids, root_node)
    for v in ids
        t_low[v] = Δt[v] + minimum(Drs[:,v])
        for v2 in inneighbors(G,v)
            t_low[v] = max(t_low[v], t_low[v2] + Δt[v] + minimum(Dss[inneighbors(G,v),v]))
        end
    end
    t_low
end

################################################################################
################################### Rendering ##################################
################################################################################

export
    get_display_metagraph

title_string(a::GO)        = "go"
title_string(a::COLLECT)   = "collect"
title_string(a::CARRY)     = "carry"
title_string(a::DEPOSIT)   = "deposit"

function get_display_metagraph(project_schedule::ProjectSchedule;
    object_color="orange",
    robot_color="lime",
    action_color="cyan",
    operation_color="red"
    )
    graph = MetaDiGraph(project_schedule.graph)
    for (id,pred) in get_object_ICs(project_schedule)
        v = get_vtx(project_schedule, get_o(pred))
        set_prop!(graph, v, :vtype, :object_ic)
        set_prop!(graph, v, :text, string("O",id))
        set_prop!(graph, v, :color, object_color)
    end
    for (id,pred) in get_robot_ICs(project_schedule)
        v = get_vtx(project_schedule, get_r(pred))
        set_prop!(graph, v, :vtype, :robot_ic)
        set_prop!(graph, v, :text, string("R",id))
        set_prop!(graph, v, :color, robot_color)
    end
    for (id,op) in get_operations(project_schedule)
        v = get_vtx(project_schedule, OperationID(id))
        set_prop!(graph, v, :vtype, :operation)
        set_prop!(graph, v, :text, string("M",id))
        set_prop!(graph, v, :color, operation_color)
    end
    for (id,a) in get_actions(project_schedule)
        v = get_vtx(project_schedule, ActionID(id))
        set_prop!(graph, v, :vtype, :action)
        set_prop!(graph, v, :text, string(title_string(a),"-",get_id(get_r(a))))
        set_prop!(graph, v, :color, action_color)
    end
    graph
end

###### Custom Solver Tools
export
    FeasibleAssignmentTable,
    get_feasible_assignments,
    add_constraint!,
    SearchCache,
    get_branch_id,
    is_feasible,
    TaskGraphProblemSpec,
    construct_solution_graph,
    process_solution,
    process_solution_fast,
    solve_task_graph,
    solve_task_graphs_problem

"""
    `FeasibleAssignmentTable`

    Maps task id to the ids of LEGAL robot assignments
"""
struct FeasibleAssignmentTable
    arr::Matrix{Bool}
end
function FeasibleAssignmentTable(M::Int,N::Int)
    FeasibleAssignmentTable(
        fill!(Matrix{Bool}(undef,N+M,M),true)
    )
end
function get_feasible_assignments(table::FeasibleAssignmentTable, v::Int)
    [i for (i, flag) in enumerate(table.arr[:,v]) if flag==true]
end

"""
    `add_constraint!(table::FeasibleAssignmentTable, i::Int, j::Int)`

    specify that robot `i` is not allowed to perform task `j`
"""
function add_constraint!(table::FeasibleAssignmentTable, i::Int, j::Int)
    table.arr[i,j] = false
end

struct SearchCache
    x           ::Matrix{Int}
    to0         ::Vector{Float64}
    tof         ::Vector{Float64}
    tr0         ::Vector{Float64}
    slack       ::Vector{Float64}
    local_slack ::Vector{Float64}
    FT          ::FeasibleAssignmentTable
end
function SearchCache(N::Int,M::Int,to0_::Dict{Int,Float64},tr0_::Dict{Int,Float64})
    cache = SearchCache(
        zeros(Int,N+M,M),
        zeros(M),
        zeros(M),
        Inf*ones(N+M),
        zeros(M),
        zeros(M),
        FeasibleAssignmentTable(M,N)
    )
    # initial conditions
    for (i,t) in tr0_
        cache.tr0[i] = t # start time for robot i
    end
    for (j,t) in to0_
        cache.to0[j] = t # start time for task j (leaf tasks only)
    end
    cache
end
function get_branch_id(cache::SearchCache,mode=1)
    if mode == 1
        M = length(cache.to0)
        i = argmax(cache.x * ones(M))
    elseif mode == 2
        M = length(cache.to0)
        for j in 1:M
            if sum(cache.x[j,:]) > 1
                i = argmin(cache.x[j,:] .* cache.slack)
                return i
            end
        end
    end
    i
end
function is_feasible(cache::SearchCache)
    M = length(cache.to0)
    rank(cache.x) == M
end

struct TaskGraphProblemSpec{G}
    N::Int
    M::Int
    graph::G
    Drs::Matrix{Float64}
    Dss::Matrix{Float64}
    Δt::Vector{Float64}
    tr0_::Dict{Int,Float64}
    to0_::Dict{Int,Float64}
end

# solution graph encodes dependencies between robots and tasks too
"""
    `construct_solution_graph(G,assignment)`

    Constructs a graph wherein all robots and tasks are represented by vertices,
    and edges go from prereqs to their children.If robot i is responsible
    for task j, there is an edge from robot i to task j. If task j must be
    completed before task k, there is an edge from task j to task k)
"""
function construct_solution_graph(G,assignment)
    M = nv(G)
    N = size(assignment,1) - M
    solution_graph = deepcopy(G)
    for j in vertices(solution_graph)
        set_prop!(solution_graph,j,:vtype,:task)
        set_prop!(solution_graph,j,:text,string("T",j))
        set_prop!(solution_graph,j,:color,"cyan")
    end
    for i in 1:N+M
        add_vertex!(solution_graph)
        set_prop!(solution_graph,nv(solution_graph),:vtype,:robot)
        set_prop!(solution_graph,nv(solution_graph),:text,string("R",i))
        set_prop!(solution_graph,nv(solution_graph),:color,"orange")
    end
    for i in 1:N+M
        for j in 1:M
            if assignment[i,j] == 1
                add_edge!(solution_graph,M+i,j) # robot i -> task j
                add_edge!(solution_graph,j,M+j+N) # task j -> dummy robot j+N
            end
        end
    end
    solution_graph
end

"""
    `process_solution(cache::SearchCache,spec::TaskGraphProblemSpec,bfs_traversal)`

    given a cache whose assignment matrix is feasible, compute the start and end times
    of the robots and tasks, as well as the slack.
"""
function process_solution(model,cache::SearchCache,spec::TaskGraphProblemSpec)
    Drs, Dss, Δt, N, G = spec.Drs, spec.Dss, spec.Δt, spec.N, spec.graph
    solution_graph = construct_solution_graph(G,cache.x)
    traversal = topological_sort_by_dfs(solution_graph)
    # Compute Lower Bounds Via Forward Dynamic Programming pass
    for v in traversal
        if get_prop(solution_graph,v,:vtype) == :task
            for v2 in inneighbors(G,v)
                cache.to0[v] = max(cache.to0[v], cache.tof[v2] + Δt[v])
            end
            xi = findfirst(cache.x[:,v] .== 1)
            tro0 = cache.tr0[xi] + Drs[xi,v]
            cache.tof[v] = max(cache.to0[v], tro0) + Dss[v,v]
            cache.tr0[v+N] = cache.tof[v]
        elseif get_prop(solution_graph,v,:vtype) == :robot
        end
    end
    # Compute Slack Via Backward Dynamic Programming pass
    for v in reverse(traversal)
        if get_prop(solution_graph,v,:vtype) == :task
            for v2 in inneighbors(G,v)
                if get_prop(solution_graph,v2,:vtype) == :task
                    cache.local_slack[v2] = cache.to0[v] - cache.tof[v2]
                    cache.slack[v2]       = cache.slack[v] + cache.local_slack[v2]
                end
            end
        end
    end
    for v in reverse(topological_sort_by_dfs(G))
        for v2 in inneighbors(G,v)
            cache.local_slack[v2] = cache.to0[v] - cache.tof[v2]
            cache.slack[v2]       = cache.slack[v] + cache.local_slack[v2]
        end
    end
    cache
end

"""
    `process_solution_fast(cache::SearchCache,spec::TaskGraphProblemSpec,bfs_traversal)`

    given a cache whose assignment matrix is feasible, compute the start and end times
    of the robots and tasks, as well as the slack. Identical to `process_solution`,
    except that `process_solution_fast` pulls `to0`, `tof` and `tr0` directly
    from the optimization model.
"""
function process_solution_fast(model,cache::SearchCache,spec::TaskGraphProblemSpec)
    Drs, Dss, Δt, N, G = spec.Drs, spec.Dss, spec.Δt, spec.N, spec.graph
    cache.tr0[:] = value.(model[:tr0])
    cache.to0[:] = value.(model[:to0])
    cache.tof[:] = value.(model[:tof])
    for v in reverse(topological_sort_by_dfs(G))
        for v2 in inneighbors(G,v)
            cache.local_slack[v2] = cache.to0[v] - cache.tof[v2]
            cache.slack[v2]       = cache.slack[v] + cache.local_slack[v2]
        end
    end
    cache
end

function solve_task_graph(cache::SearchCache,spec::TaskGraphProblemSpec,bfs_traversal)
    Drs, Dss, Δt, N, G = spec.Drs, spec.Dss, spec.Δt, spec.N, spec.graph
    # Compute Lower Bounds Via Forward Dynamic Programming pass
    for v in bfs_traversal
        cache.x[:,v] .= 0
        for v2 in inneighbors(G,v)
            cache.to0[v] = max(cache.to0[v], cache.tof[v2] + Δt[v])
        end
        r_idxs = get_feasible_assignments(cache.FT,v) # valid robot indices for this task
        tro0 = Inf
        xi = 0
        for i in r_idxs
            if cache.tr0[i] + Drs[i,v] < tro0
                xi = i
                tro0 = cache.tr0[i] + Drs[i,v]
            end
        end
        cache.x[xi,v] = 1
        cache.tof[v] = max(cache.to0[v], tro0) + Dss[v,v]
        cache.tr0[v+N] = cache.tof[v]
    end
    # Compute Slack Via Backward Dynamic Programming pass
    for v in reverse(bfs_traversal)
        for v2 in inneighbors(G,v)
            cache.local_slack[v2] = cache.to0[v] - cache.tof[v2]
            cache.slack[v2]       = cache.slack[v] + cache.local_slack[v2]
        end
    end
    cache
end

"""
    `solve_task_graphs_problem(G,Drs,Dss,Δt)`

    Inputs:
        `G` - graph with inverted tree structure that encodes dependencies
            between tasks
        `Drs` - Drs[i,j] is distance from initial robot position i to pickup
            station j
        `Dss` - Dss[j,j] is distance from start station j to final station j (we
            only care about the diagonal)
        `Δt` - Δt[j] is the duration of time that must elapse after all prereqs
            of task j have been satisfied before task j becomes available
        `to0_` - a `Dict`, where `to0_[j]` gives the start time for task j
            (applies to leaf tasks only)
        `tr0_` - a `Dict`, where `tr0_[i]` gives the start time for robot i
            (applies to non-dummy robots only)

    Outputs:
        `x` - the optimal assignment vector
"""
function solve_task_graphs_problem(G,Drs,Dss,Δt,to0_,tr0_;mode=1,MAX_ITERS=10000)
    M = size(Dss,1)
    N = size(Drs,1) - M
    bfs_traversal = Vector{Int}()
    for root_node in get_all_root_nodes(G)
        bfs_traversal = [bfs_traversal, get_bfs_node_traversal(G,root_node)]
    end
    bfs_traversal = get_bfs_node_traversal(G);
    cache0 = SearchCache(N,M,to0_,tr0_)
    for j in 1:M
        upstream_jobs = [j, map(e->e.dst,collect(edges(bfs_tree(G,j;dir=:in))))...]
        for v in upstream_jobs
            add_constraint!(cache0.FT, j+N, v)
        end
    end
    spec = TaskGraphProblemSpec(N,M,G,Drs,Dss,Δt,tr0_,to0_)

    cache0 = solve_task_graph(cache0,spec,bfs_traversal)
    @show cache0.tof
    @show cache0.slack
    @show cache0.local_slack;

    P = PriorityQueue{SearchCache,Float64}()
    enqueue!(P, cache0, cache0.tof[end])
    FEASIBLE = false
    iteration = 1
    while length(P) > 0
        cache, tf = dequeue_pair!(P)
        # check if solution is feasible
        if is_feasible(cache)
            FEASIBLE = true
            @show iteration
            @show FEASIBLE
            return cache, FEASIBLE
        end

        # BRANCH
        cache1 = deepcopy(cache); cache2 = deepcopy(cache)
        # select robot id to branch on ... HOW TO DECIDE?
        i = get_branch_id(cache,mode)
        # Which assignment to try keeping?
        idxs = findall(cache.x[i,:] .== 1)
        ordering = (cache.slack .* cache.x[i,:])[idxs]
        sort!(idxs, by=j->(cache.slack .* cache.x[i,:])[j])
        # add constraints
        # assignment must be maintained in child 1
        for j in bfs_traversal
            if j != idxs[1]
                add_constraint!(cache1.FT, i, j)
            end
        end
        # assignment must be swapped in child 2
        add_constraint!(cache2.FT, i, idxs[1])

        # for j in idxs[1:end-1]
        #     add_constraint!(cache1.FT, i, j)
        # end
        # add_constraint!(cache2.FT, i, idxs[end])

        cache1 = solve_task_graph(cache1,spec,bfs_traversal)
        enqueue!(P, cache1, cache1.tof[end])
        cache2 = solve_task_graph(cache2,spec,bfs_traversal)
        enqueue!(P, cache2, cache2.tof[end])
        iteration += 1
        if iteration > MAX_ITERS
            @show iteration
            @show FEASIBLE
            return cache, FEASIBLE
        end
    end
end

# Greedy Search with Correction
# for task in critical_path
#     dummy_robots_queue = []
#     for robot in sort(robots, by = lower_bound_on_task_completion_time)
#         if is_capable(robot, task)
#             if lower_bound(robot,task) < current_completion_time(task)
#                 if is_not_dummy(robot)
#                     partial_new_assignment = [robot => task]
#                     new_assignment = greedy_search(partial_new_assignment)
#                 else
#                 end
#             end
#         end
#     end
# end

# """
#     TODO: HungarianMethod
#
#     Not done yet...
# """
# function hungarian_method(Cp)
#     C = copy(Cp)
#     assignment = collect(1:size(C,1))
#     solved = false
#     # idxs = map(i->(mod(i-1,size(C,1))+1,div(i-1,size(C,2))+1),sortperm(C[:]))
#     idxs = Set{Tuple{Int}}()
#     idxs_i = Set{Int}()
#     idxs_j = Set{Int}()
#     for i in 1:size(C,1)
#         for j in sortperm(C[i,:])
#             if C[i,j] > 0
#                 C[i,:] .-= C[i,j]
#                 break
#             end
#         end
#     end
#     for j in 1:size(C,2)
#         for i in sortperm(C[:,j])
#             if C[i,j] > 0
#                 C[:,j] .-= C[i,j]
#                 break
#             end
#         end
#     end
#     if (rank(C .== 0) == size(C,1))
#         solved = true
#     end
#     while !solved
#         k = argmin(C[:])
#         i = mod(k-1,size(C,1))+1
#         j = div(i-1,size(C,2))+1
#     end
#     return assignment, solved
# end
