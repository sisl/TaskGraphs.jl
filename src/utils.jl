module TaskGraphsUtils

using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using LinearAlgebra
using DataStructures
using JuMP, Gurobi

using ..TaskGraphs
# using ..FactoryWorlds
# using ..PlanningPredicates
# using ..TaskGraphsCore

export
    formulate_optimization_problem,
    get_assignment_matrix,
    exclude_solutions!,
    cached_pickup_and_delivery_distances,
    construct_task_graphs_problem

"""
    Express the TaskGraphs assignment problem as a MILP using the JuMP optimization
    framework.

    `formulate_optimization_problem(G,Drs,Dss,Δt,to0_,tr0_,optimizer)`

    Inputs:
        `G` - graph with inverted tree structure that encodes dependencies
            between tasks
        `Drs` - Drs[i,j] is distance from initial robot position i to pickup
            station j
        `Dss` - Dss[j,j] is distance from start station j to final station j (we
            only care about the diagonal)
        `Δt` - Δt[j] is the duraction of time that must elapse after all prereqs
            of task j have been satisfied before task j becomes available
        `Δt_collect` - Δt_collect[j] is the time required for a robot to pick up
            object j
        `Δt_deliver` - Δt_deliver[j] is the time required for a robot to set
            down object j
        `to0_` - a `Dict`, where `to0_[j]` gives the start time for task j
            (applies to leaf tasks only)
        `tr0_` - a `Dict`, where `tr0_[i]` gives the start time for robot i
            (applies to non-dummy robots only)
        `root_nodes` - a vector of integers specfying the graph vertices that
            are roots of the project
        `weights` - a vector of weights that determines the contribution of each
            root_node to the objective
        `s0` - pickup stations for the tasks
        `sF` - dropoff stations for the tasks
        `optimizer` - a JuMP optimizer (e.g., Gurobi.optimizer)
    Keyword Args:
        `TimeLimit=100`
        `OutputFlag=0`
        `assignments=Dict{Int64,Int64}()` - maps robot id to assignment that must be
            enforced
        `cost_model=:MakeSpan` - optimization objective, either `:MakeSpan` or
            `:SumOfMakeSpans`

    Outputs:
        `model` - the optimization model
"""
function formulate_optimization_problem(G,Drs,Dss,Δt,Δt_collect,Δt_deliver,to0_,tr0_,root_nodes,weights,s0,sF,nR,optimizer;
    TimeLimit=100,
    OutputFlag=0,
    assignments=Dict{Int64,Int64}(),
    cost_model=:MakeSpan
    )

    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag
        ))
    M = size(Dss,1)
    N = size(Drs,1)-M
    @variable(model, to0[1:M] >= 0.0) # object availability time
    @variable(model, tor[1:M] >= 0.0) # object robot arrival time
    @variable(model, toc[1:M] >= 0.0) # object collection complete time
    @variable(model, tod[1:M] >= 0.0) # object deliver begin time
    @variable(model, tof[1:M] >= 0.0) # object termination time
    @variable(model, tr0[1:N+M] >= 0.0) # robot availability time

    # Assignment matrix x
    @variable(model, X[1:N+M,1:M], binary = true) # X[i,j] ∈ {0,1}
    @constraint(model, X * ones(M) .<= 1)         # each robot may have no more than 1 task
    @constraint(model, X' * ones(N+M) .== nR)     # each task must have exactly 1 assignment
    for (i,t) in tr0_
        # start time for robot i
        @constraint(model, tr0[i] == t)
    end
    for (j,t) in to0_
        # start time for task j (applies only to tasks with no prereqs)
        @constraint(model, to0[j] == t)
    end
    for (i,j) in assignments
        # start time for task j (applies only to tasks with no prereqs)
        @constraint(model, X[i,j] == 1)
    end
    # constraints
    Mm = sum(Drs) + sum(Dss) # for big-M constraints
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
            @constraint(model, X[j+N,v] == 0)
        end
        # lower bound on task completion time (task can't start until it's available).
        # tof[j] = to0[j] + Dss[j,j] + slack[j]
        @constraint(model, tor[j] >= to0[j])
        # @constraint(model, tof[j] >= tor[j] + Dss[j,j] + Δt_collect[j] + Δt_deliver[j])
        # bound on task completion time (assigned robot must first complete delivery)
        # Big M constraint (thanks Oriana!): When X[i,j] == 1, this constrains the final time
        # to be no less than the time it takes for the delivery to be completed by robot i.
        # When X[i,j] == 0, this constrains the final time to be greater than a large negative
        # number (meaning that this is a trivial constraint)
        for i in 1:N+M
            @constraint(model, tor[j] - (tr0[i] + Drs[i,j]) >= -Mm*(1 - X[i,j]))
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
                @constraint(model, T[i] >= tof[v] + Δt[v])
            end
        end
        @objective(model, Min, sum(map(i->T[i]*get(weights,i,0.0), 1:length(root_nodes))))
        # @objective(model, Min, sum(map(v->tof[v]*get(weights,v,0.0), root_nodes)))
    elseif cost_model == :MakeSpan
        @variable(model, T)
        @constraint(model, T .>= tof .+ Δt)
        @objective(model, Min, T)
    end
    model;
end
function formulate_optimization_problem(spec::T,optimizer;
    kwargs...
    ) where {T<:TaskGraphProblemSpec}
    formulate_optimization_problem(
        spec.graph,
        spec.Drs,
        spec.Dss,
        spec.Δt,
        spec.Δt_collect,
        spec.Δt_deliver,
        spec.to0_,
        spec.tr0_,
        spec.root_nodes,
        spec.weights,
        spec.s0,
        spec.sF,
        spec.nR,
        optimizer;
        kwargs...
        )
end
function get_assignment_matrix(model::M) where {M<:JuMP.Model}
    Matrix{Int}(value.(model[:X]))
end
# function get_station_precedence_dict(model::M) where {M<:JuMP.Model} end

export
    exclude_solutions!

"""
    `exclude_solutions!(model::JuMP.Model,forbidden_solutions::Vector{Matrix{Int}})`

    This is the key utility for finding the next best solution to the MILP
    problem. It simply excludes every specific solution passed to it.
"""
function exclude_solutions!(model::JuMP.Model,M::Int,forbidden_solutions::Vector{Matrix{Int}})
    for Xf in forbidden_solutions
        @constraint(model, sum(model[:X] .* Xf) <= M-1)
    end
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
    r₀ = [r₀;sₜ]
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
    `construct_task_graphs_problem`
"""
function construct_task_graphs_problem(
        project_spec::P,
        r0::Vector{Int},
        s0::Vector{Int},
        sF::Vector{Int},
        dist_matrix,
        Δt_collect=zeros(length(s0)),
        Δt_deliver=zeros(length(sF))
        ) where {P<:ProjectSpec}
    # select subset of pickup, dropoff and free locations to instantiate objects and robots
    # r0,s0,sF        = get_random_problem_instantiation(N,M,pickup_vtxs,dropoff_vtxs,free_vtxs)
    # project_spec    = construct_random_project_spec(M,s0,sF;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    N = length(r0)
    M = length(s0)
    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial object conditions
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final object conditions
    # M = length(object_ICs)
    # object_ICs = project_spec.initial_conditions
    # object_FCs = project_spec.final_conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N) # initial robot conditions
    for r in 1:M # dummy robots
        robot_ICs[r+N] = ROBOT_AT(r+N,sF[r])
    end

    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])

    delivery_graph = construct_delivery_graph(project_spec,M)
    # display(delivery_graph.tasks)
    G = delivery_graph.graph
    Δt = get_duration_vector(project_spec) # initialize vector of operation times
    # set initial conditions
    to0_ = Dict{Int,Float64}()
    for v in vertices(G)
        if is_root_node(G,v)
            to0_[v] = 0.0
        end
    end
    tr0_ = Dict{Int,Float64}()
    for i in 1:N
        tr0_[i] = 0.0
    end
    root_node_groups = map(v->get_input_ids(project_spec.operations[v]),collect(project_spec.root_nodes))
    problem_spec = TaskGraphProblemSpec(N=N,M=M,graph=G,D=dist_matrix,Drs=Drs,
        Dss=Dss,Δt=Δt,tr0_=tr0_,to0_=to0_,root_nodes=root_node_groups,
        Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,s0=s0,sF=sF)
    # @show problem_spec.root_nodes
    return project_spec, problem_spec, object_ICs, object_FCs, robot_ICs
end
function construct_task_graphs_problem(
        operations::Vector{Operation},
        robot_ICs::Vector{ROBOT_AT},
        object_ICs::Vector{OBJECT_AT},
        object_FCs::Vector{OBJECT_AT},
        dist_function,
        Δt_collect=zeros(length(object_ICs)),
        Δt_deliver=zeros(length(object_ICs)),
        Δt_process=zeros(length(operations))
        ) where {P<:ProjectSpec}
    # select subset of pickup, dropoff and free locations to instantiate objects and robots
    # r0,s0,sF        = get_random_problem_instantiation(N,M,pickup_vtxs,dropoff_vtxs,free_vtxs)
    # project_spec    = construct_random_project_spec(M,s0,sF;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
    N = length(robot_ICs)
    M = length(object_ICs)
    r0 = map(pred->get_id(get_initial_location_id(pred)),robot_ICs)
    s0 = map(pred->get_id(get_initial_location_id(pred)),object_ICs)
    sF = map(pred->get_id(get_initial_location_id(pred)),object_FCs)
    # object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial object conditions
    # object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final object conditions
    # M = length(object_ICs)
    # object_ICs = project_spec.initial_conditions
    # object_FCs = project_spec.final_conditions
    # robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N) # initial robot conditions
    for r in 1:M # dummy robots
        robot_ICs[r+N] = ROBOT_AT(r+N,sF[r])
    end

    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,dist_function)

    delivery_graph = construct_delivery_graph(project_spec,M)
    # display(delivery_graph.tasks)
    G = delivery_graph.graph
    # Δt = get_duration_vector(project_spec) # initialize vector of operation times
    # set initial conditions
    to0_ = Dict{Int,Float64}()
    for v in vertices(G)
        if is_root_node(G,v)
            to0_[v] = 0.0
        end
    end
    tr0_ = Dict{Int,Float64}()
    for i in 1:N
        tr0_[i] = 0.0
    end
    root_node_groups = map(v->get_input_ids(project_spec.operations[v]),collect(project_spec.root_nodes))
    problem_spec = TaskGraphProblemSpec(N=N,M=M,graph=G,D=dist_matrix,Drs=Drs,
        Dss=Dss,Δt=Δt_process,tr0_=tr0_,to0_=to0_,root_nodes=root_node_groups,
        Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,s0=s0,sF=sF)
    # @show problem_spec.root_nodes
    return project_spec, problem_spec, object_ICs, object_FCs, robot_ICs
end

export
    construct_random_project_spec,
    get_random_problem_instantiation,
    construct_random_task_graphs_problem

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
function construct_random_project_spec(M::Int,object_ICs::Vector{OBJECT_AT},object_FCs::Vector{OBJECT_AT};
    max_parents=1,depth_bias=1.0,Δt_min=0,Δt_max=0)
    project_spec = ProjectSpec(
        M=M,
        initial_conditions=object_ICs,
        final_conditions=object_FCs
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
        output_idx = pairs[end].first
        station_id = output_idx
        input_idxs = collect(max(1,1+i-rand(1:max_parents)):i)
        i = i - length(input_idxs)
        # Δt = Δt_min + (Δt_max-Δt_min)*rand()
        Δt=rand(Δt_min:Δt_max)
        # add_operation!(project_spec,construct_operation(station_id, input_ids, [output_id], Δt))
        input_ids = map(idx->get_id(get_object_id(project_spec.initial_conditions[idx])), input_idxs)
        output_ids = map(idx->get_id(get_object_id(project_spec.initial_conditions[idx])), [output_idx])
        # @show input_ids, output_ids
        # @show project_spec.initial_conditions
        # @show project_spec.object_id_to_idx
        # @show map(id->project_spec.object_id_to_idx[id], input_ids)
        # @show map(id->project_spec.object_id_to_idx[id], output_ids)
        add_operation!(project_spec,construct_operation(project_spec, station_id, input_ids, output_ids, Δt))
        for idx in input_idxs
            enqueue!(frontier, idx=>M-i)
        end
    end
    Δt=0
    final_idx = get_id(get_object_id(project_spec.initial_conditions[M]))
    add_operation!(project_spec,construct_operation(project_spec, -1, [final_idx], [], Δt))
    project_spec
end
# function construct_random_project_spec(M::Int,object_ICs::Vector{OBJECT_AT},object_FCs::Vector{OBJECT_AT};
#     kwargs...)
#     object_IC_dict = Vector{OBJECT_AT}([pred for pred in object_ICs])
#     object_FC_dict = Vector{OBJECT_AT}([pred for pred in object_FCs])
#     construct_random_project_spec(M,object_IC_dict,object_FC_dict;
#         kwargs...)
# end
function construct_random_project_spec(M::Int,s0::Vector{Int},sF::Vector{Int};
    kwargs...)
    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s) for (o,s) in enumerate(s0)])
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,s) for (o,s) in enumerate(sF)])
    construct_random_project_spec(M,object_ICs,object_FCs;
        kwargs...)
end

"""
    `get_random_problem_instantiation`

    Args:
    - `N`: number of robots
    - `M`: number of delivery tasks
    - `robot_zones`: list of possible start locations for robots
    - `pickup_zones`: list of possible start locations for objects
    - `dropoff_zones`: list of possible destinations for objects
"""
function get_random_problem_instantiation(N::Int,M::Int,pickup_zones,dropoff_zones,robot_zones)
    ##### Random Problem Initialization #####
    r0 = robot_zones[sortperm(rand(length(robot_zones)))][1:N]
    s0 = pickup_zones[sortperm(rand(length(pickup_zones)))][1:M]
    sF = dropoff_zones[sortperm(rand(length(dropoff_zones)))][1:M]
    return r0,s0,sF
end

"""
    `construct_randomd_task_graphs_problem`
"""
function construct_random_task_graphs_problem(N::Int,M::Int,
    pickup_vtxs::Vector{Int},dropoff_vtxs::Vector{Int},free_vtxs::Vector{Int},dist_matrix,
    Δt_collect::Vector{Float64}=zeros(M),
    Δt_deliver::Vector{Float64}=zeros(M)
    )
    # select subset of pickup, dropoff and free locations to instantiate objects and robots
    r0,s0,sF        = get_random_problem_instantiation(N,M,pickup_vtxs,dropoff_vtxs,free_vtxs)
    project_spec    = construct_random_project_spec(M,s0,sF;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)

    construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix,Δt_collect,Δt_deliver)
end

export
    initialize_test_problem

function initialize_test_problem(N=8,M=12;env_id=2)
    # experiment_dir = joinpath(dirname(pathof(WebotsSim)),"..","experiments")
    filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml");
    factory_env = read_env(filename);

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_random_task_graphs_problem(
        N,M,
        get_pickup_zones(factory_env),
        get_dropoff_zones(factory_env),
        get_free_zones(factory_env),
        get_dist_matrix(factory_env.graph));

    project_spec, problem_spec, robot_ICs, factory_env.graph
end


export
    combine_project_specs

"""
    `combine_project_specs(specs::Vector{ProjectSpec})`

    A helper for combining multiple `ProjectSpec`s into a single
    ProjectSpec.
"""
function combine_project_specs(specs::Vector{P}) where {P<:ProjectSpec}
    M = 0
    new_spec = ProjectSpec(
        initial_conditions=vcat(map(spec->spec.initial_conditions, specs)...),
        final_conditions=vcat(map(spec->spec.final_conditions, specs)...)
        )
    for spec in specs
        # spec_M = length(spec.initial_conditions)
        for op in spec.operations
            new_op = Operation(station_id=op.station_id, Δt = op.Δt)
            for pred in preconditions(op)
                push!(new_op.pre, OBJECT_AT(get_object_id(pred), get_location_id(pred)))
            end
            for pred in postconditions(op)
                push!(new_op.post, OBJECT_AT(get_object_id(pred), get_location_id(pred)))
            end
            add_operation!(new_spec, new_op)
        end
        # M = M + spec_M
    end
    new_spec
end

################################################################################
################################### Rendering ##################################
################################################################################

export
    title_string,
    get_display_metagraph

title_string(pred::OBJECT_AT,verbose=true) = verbose ? string("O",get_id(get_object_id(pred)),"-",get_id(get_location_id(pred))) : string("O",get_id(get_object_id(pred)));
title_string(pred::ROBOT_AT,verbose=true)  = verbose ? string("R",get_id(get_robot_id(pred)),"-",get_id(get_location_id(pred))) : string("R",get_id(get_robot_id(pred)));
title_string(a::GO,verbose=true)        = verbose ? string("go\n",get_id(get_robot_id(a)),",",get_id(get_initial_location_id(a)),",",get_id(get_destination_location_id(a))) : "go";
title_string(a::COLLECT,verbose=true)   = verbose ? string("collect\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a))) : "collect";
title_string(a::CARRY,verbose=true)     = verbose ? string("carry\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_destination_location_id(a))) : "carry";
title_string(a::DEPOSIT,verbose=true)   = verbose ? string("deposit\n",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a))) : "deposit";
title_string(op::Operation,verbose=true)= verbose ? "op" : "op";

function get_display_metagraph(project_schedule::ProjectSchedule;
    verbose=true,
    f=(v,p)->title_string(p,verbose),
    object_color="orange",
    robot_color="lime",
    action_color="cyan",
    operation_color="red",
    remove_leaf_robots=false
    )
    graph = MetaDiGraph(deepcopy(project_schedule.graph))
    for (id,pred) in get_object_ICs(project_schedule)
        v = get_vtx(project_schedule, get_object_id(pred))
        set_prop!(graph, v, :vtype, :object_ic)
        set_prop!(graph, v, :text, f(v,pred))
        set_prop!(graph, v, :color, object_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,op) in get_operations(project_schedule)
        v = get_vtx(project_schedule, OperationID(id))
        set_prop!(graph, v, :vtype, :operation)
        set_prop!(graph, v, :text, f(v,op))
        set_prop!(graph, v, :color, operation_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,a) in get_actions(project_schedule)
        v = get_vtx(project_schedule, ActionID(id))
        set_prop!(graph, v, :vtype, :action)
        set_prop!(graph, v, :text, f(v,a))
        set_prop!(graph, v, :color, action_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    for (id,pred) in get_robot_ICs(project_schedule)
        v = get_vtx(project_schedule, get_robot_id(pred))
        set_prop!(graph, v, :vtype, :robot_ic)
        set_prop!(graph, v, :text, f(v,pred))
        set_prop!(graph, v, :color, robot_color)
        set_prop!(graph, v, :vtx_id, v)
    end
    if remove_leaf_robots == true
        for v in reverse(vertices(graph))
            # if get_prop(graph,v,:vtype) ==:robot_ic
            if typeof(get_vtx_id(project_schedule,v)) <: RobotID
                if length(outneighbors(graph,v)) == 0
                    rem_vertex!(graph,v)
                end
            end
        end
    end
    graph
end

################################################################################
######################### COMPARISONS AND VERIFICATION #########################
################################################################################
function Base.:(==)(op1::Operation,op2::Operation)
    try
        @assert(op1.pre == op2.pre)
        @assert(op1.post == op2.post)
        @assert(op1.Δt == op2.Δt)
        @assert(op1.station_id == op2.station_id)
    catch e
        println(e.msg)
        # throw(e)
        return false
    end
    return true
end
function Base.:(==)(spec1::ProjectSpec,spec2::ProjectSpec)
    try
        @assert(spec1.initial_conditions == spec2.initial_conditions)
        @assert(spec1.final_conditions == spec2.final_conditions)
        @assert(spec1.operations == spec2.operations)
        @assert(spec1.pre_deps == spec2.pre_deps)
        @assert(spec1.graph == spec2.graph)
        @assert(spec1.root_nodes == spec2.root_nodes)
        @assert(spec1.weights == spec2.weights)
        @assert(spec1.weight == spec2.weight)
        @assert(spec1.object_id_to_idx == spec2.object_id_to_idx)
    catch e
        println(e.msg)
        # throw(e)
        return false
    end
    return true
end

end # module TaskGraphsUtils
