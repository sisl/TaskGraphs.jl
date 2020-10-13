# module TaskGraphsUtils
#
# using Parameters
# using LightGraphs, MetaGraphs
# using GraphUtils
# using LinearAlgebra
# using DataStructures
# using JuMP, MathOptInterface, Gurobi
# using Random
#
# using ..TaskGraphs
# using CRCBS
# using ..FactoryWorlds
# using ..PlanningPredicates
# using ..TaskGraphsCore


export to_string_dict

to_string_dict(dict::Dict{K,V}) where {K,V} = Dict{String,V}(string(k)=>v for (k,v) in dict)
TOML.print(io::IO,dict::Dict{Symbol,V}) where {V} = TOML.print(io,to_string_dict(dict))

export
    remap_object_id,
    remap_object_ids!

# utilities for remappingg object ids
remap_object_id(x,args...) = x
remap_object_id(id::ObjectID,max_obj_id)    = ObjectID(get_id(id) + max_obj_id)
remap_object_id(spec::PathSpec,max_obj_id)  = PathSpec(spec,object_id=spec.object_id + max_obj_id)
remap_object_id(node::OBJECT_AT,args...)    = OBJECT_AT(remap_object_id(get_object_id(node),args...), get_initial_location_id(node))
remap_object_id(node::A,args...) where {A<:Union{CARRY,COLLECT,DEPOSIT}} = A(node,o=remap_object_id(get_object_id(node),args...))
remap_object_id(node::TEAM_ACTION,args...) = TEAM_ACTION(node,instructions=map(i->remap_object_id(i,args...),node.instructions))
function remap_object_ids!(node::Operation,args...)
    new_pre = Set([remap_object_id(o,args...) for o in node.pre])
    empty!(node.pre)
    union!(node.pre, new_pre)
    new_post = Set([remap_object_id(o,args...) for o in node.post])
    empty!(node.post)
    union!(node.post, new_post)
    node
end
remap_object_id(node::Operation,args...)    = remap_object_ids!(deepcopy(node),args...)
function remap_object_ids!(project_schedule::OperatingSchedule,args...)
    for i in 1:length(project_schedule.vtx_ids)
        project_schedule.vtx_ids[i] = remap_object_id(project_schedule.vtx_ids[i],args...)
    end
    for dict in (project_schedule.vtx_map, project_schedule.planning_nodes)
        for k in collect(keys(dict))
            new_node = remap_object_id(dict[k],args...)
            delete!(dict, k)
            dict[remap_object_id(k,args...)] = new_node
        end
    end
    for v in vertices(get_graph(project_schedule))
        project_schedule.path_specs[v] = remap_object_id(project_schedule.path_specs[v],args...)
    end
    @assert sanity_check(project_schedule," after remap_object_ids!()")
    project_schedule
end
function remap_object_ids!(new_schedule::OperatingSchedule,old_schedule::OperatingSchedule)
    max_obj_id = 0
    for id in get_vtx_ids(old_schedule)
        if typeof(id) <: ObjectID
            # max_obj_id = maximum([get_id(id) for id in get_vtx_ids(old_schedule) if typeof(id) <: ObjectID])
            max_obj_id = max(get_id(id),max_obj_id)
        end
    end
    remap_object_ids!(new_schedule,max_obj_id)
end

export get_robot_ids

"""
    get_robot_ids(sched::OperatingSchedule,node_id::AbstractID)

Returns vector of all robot ids associated with the schedule node referenced by
node_id.
"""
function get_robot_ids(sched::OperatingSchedule,node_id::A,v=get_vtx(sched,node_id)) where {A<:Union{ActionID,RobotID}}
    ids = Vector{RobotID}()
    robot_id = RobotID(get_path_spec(sched,v).agent_id)
    node = get_node_from_id(sched,node_id)
    if isa(node,TEAM_ACTION)
        for n in node.instructions
            r = get_robot_id(n)
            if get_id(r) != -1
                push!(ids,r)
            end
        end
    else
        push!(ids,robot_id)
    end
    return ids
end
function get_robot_ids(sched::OperatingSchedule,node_id::A,v=get_vtx(sched,node_id)) where {A<:Union{ObjectID,OperationID}}
    return Vector{RobotID}()
end
get_robot_ids(s::OperatingSchedule,v::Int) = get_robot_ids(s,get_vtx_id(s,v),v)
get_robot_ids(node) = RobotID[]
get_robot_ids(node::AbstractRobotAction) = [get_robot_id(node)]
get_robot_ids(node::TEAM_ACTION) = map(n->get_robot_id(n),node.instructions)

export robot_tip_map

"""
    robot_tip_map(sched::OperatingSchedule)

Returns a `Dict{RobotID,AbstractID}` mapping `RobotID` to the terminal node of
the `sched` corresponding to the robot's last assigned task.
"""
function robot_tip_map(sched::OperatingSchedule,vtxs=get_all_terminal_nodes(sched))
    robot_tips = Dict{RobotID,AbstractID}()
    for v in vtxs
        node_id = get_vtx_id(sched,v)
        for robot_id in get_robot_ids(sched,node_id,v)
            if get_id(robot_id) != -1
                @assert !haskey(robot_tips,robot_id)
                robot_tips[robot_id] = node_id
            end
        end
    end
    # @assert length(robot_tips) == length(get_robot_ICs(sched)) "length(robot_tips) == $(length(robot_tips)), but should be $(length(get_robot_ICs(sched)))"
    robot_tips
end

export
    # validate,
    cached_pickup_and_delivery_distances,
    construct_task_graphs_problem


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
        dist_matrix;
        Δt_collect=zeros(length(s0)),
        Δt_deliver=zeros(length(sF)),
        cost_function=SumOfMakeSpans(),
        task_shapes=map(o->(1,1),s0),
        shape_dict=Dict{Int,Dict{Tuple{Int,Int},Vector{Int}}}(s=>Dict{Tuple{Int,Int},Vector{Int}}() for s in vcat(s0,sF))
        ) where {P<:ProjectSpec}
    N = length(r0)
    M = length(s0)
    for j in 1:M
        @assert haskey(shape_dict, s0[j]) "shape_dict has no key for s0[$j] = $(s0[j])"
        @assert haskey(shape_dict, sF[j]) "shape_dict has no key for sF[$j] = $(sF[j])"
        @assert length(task_shapes) >= j "task_shapes has no key for j = $j"
    end
    robot_ICs = [ROBOT_AT(r,x) for (r,x) in enumerate(r0)] # initial robot conditions
    object_ICs = [OBJECT_AT(j, get(shape_dict[x], s, x), s) for (j,(x,s)) in enumerate(zip(s0,task_shapes))] # initial object conditions
    object_FCs = [OBJECT_AT(j, get(shape_dict[x], s, x), s) for (j,(x,s)) in enumerate(zip(sF,task_shapes))] # initial object conditions
    new_project_spec = ProjectSpec(object_ICs,object_FCs)
    for op in project_spec.operations
        add_operation!(
            new_project_spec,
            construct_operation(
                new_project_spec,
                op.station_id,
                map(o->get_id(get_object_id(o)), collect(op.pre)),
                map(o->get_id(get_object_id(o)), collect(op.post)),
                op.Δt
            )
        )
    end

    delivery_graph = construct_delivery_graph(new_project_spec,M)
    G = delivery_graph.graph
    Δt = get_duration_vector(project_spec) # initialize vector of operation times
    # set initial conditions
    to0_ = Dict{Int,Float64}(v=>0.0 for v in get_all_root_nodes(G))
    tr0_ = Dict{Int,Float64}(i=>0.0 for i in 1:N)
    root_node_groups = map(v->Set(get_input_ids(new_project_spec.operations[v])),collect(new_project_spec.terminal_vtxs))
    problem_spec = ProblemSpec(N=N,M=M,graph=G,D=dist_matrix,
        Δt=Δt,tr0_=tr0_,to0_=to0_,terminal_vtxs=root_node_groups,
        cost_function=cost_function,
        Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,r0=r0,s0=s0,sF=sF)
    # @show problem_spec.terminal_vtxs
    return new_project_spec, problem_spec, object_ICs, object_FCs, robot_ICs
end
function construct_task_graphs_problem(
    def::SimpleProblemDef,
    dist_matrix;
    task_shapes = def.shapes,
    kwargs...,
)
    construct_task_graphs_problem(
        def.project_spec,
        def.r0,
        def.s0,
        def.sF,
        dist_matrix;
        task_shapes = task_shapes,
        kwargs...,
    )
end
function construct_task_graphs_problem(
    def::SimpleProblemDef,
    env::GridFactoryEnvironment;
    kwargs...,
)
    construct_task_graphs_problem(
        def,
        get_dist_matrix(env);
        task_shapes = def.shapes,
        shape_dict = env.expanded_zones,
        kwargs...,
    )
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
        max_parents=1,
        depth_bias=1.0,
        Δt_min=0,
        Δt_max=0,
        zone_map=Dict{Int,Vector{Int}}(), # maps vtx to vector of vtxs for collaborative tasks
    )
    project_spec = ProjectSpec(object_ICs,object_FCs)
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
            push!(pairs, dequeue_pair!(frontier))
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
        input_ids = map(idx->get_id(get_object_id(project_spec.initial_conditions[idx])), input_idxs)
        output_ids = map(idx->get_id(get_object_id(project_spec.initial_conditions[idx])), [output_idx])
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
# function construct_random_project_spec(M::Int,s0::Vector{Int},sF::Vector{Int};
function construct_random_project_spec(M::Int,s0::Vector{V},sF::Vector{V};kwargs...) where {V<:Union{Int,Vector{Int}}}
    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s) for (o,s) in enumerate(s0)])
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,s) for (o,s) in enumerate(sF)])
    construct_random_project_spec(M,object_ICs,object_FCs;
        kwargs...)
end

export
    choose_random_object_sizes,
    choose_and_shuffle_object_sizes,
    convert_to_collaborative

"""
    Tool for randomly selecting how many robots (and in what configuration)
        should deliver each task.
"""
function choose_random_object_sizes(M,probs::Dict{Tuple{Int,Int},Float64})
    k = sort(collect(keys(probs)))
    v = cumsum(map(key->probs[key], k))
    v = v / sum(collect(values(probs)))
    sizes = Vector{Tuple{Int,Int}}()
    for j in 1:M
        n = rand()
        i = 1
        while n > v[i]
            i += 1
        end
        push!(sizes,k[i])
    end
    sizes
end
function choose_and_shuffle_object_sizes(M,probs::Dict{Tuple{Int,Int},Float64})
    k = sort(collect(keys(probs)),by=i->i[1]*i[2])
    v = map(key->probs[key], k)
    v = M * v / sum(v)
    v_rounded = Int.(round.(v))
    while sum(v_rounded) < sum(v)
        idx = sort(collect(1:length(v)),by=i->abs(v[i]-v_rounded[i]))[end]
        v_rounded[idx] = v_rounded[idx] + 1
    end
    while sum(v_rounded) > sum(v)
        idx = sort(collect(1:length(v)),by=i->abs(v[i]-v_rounded[i]))[end]
        v_rounded[idx] = v_rounded[idx] - 1
    end
    sizes = Vector{Tuple{Int,Int}}()
    for (idx,n) in enumerate(v_rounded)
        for i in 1:n
            push!(sizes,k[idx])
        end
    end
    shuffle(sizes)
end
function choose_random_object_sizes(M,probs::Dict{Int,Float64}=Dict(1=>1.0,2=>0.0,4=>0.0),choices=[(1,1),(2,1),(1,2),(2,2)])
    k = sort(collect(keys(probs)))
    size_probs = Dict{Tuple{Int,Int},Float64}(s=>probs[s[1]*s[2]] for s in choices)
    for ksize in k
        counts = sum(map(s->s[1]*s[2]==ksize, choices))
        for s in choices
            size_probs[s] = size_probs[s] / counts
        end
    end
    choose_and_shuffle_object_sizes(M,size_probs)
end

"""
    convert_to_collaborative(project_spec)
"""
function convert_to_collaborative(project_spec::ProjectSpec,new_starts::Dict{ObjectID,Vector{Int}},new_goals::Dict{ObjectID,Vector{Int}})
    new_spec = ProjectSpec(
        map(o->OBJECT_AT(get_object_id(o), new_starts[get_object_id(o)]), project_spec.initial_conditions),
        map(o->OBJECT_AT(get_object_id(o), new_goals[get_object_id(o)]), project_spec.final_conditions),
    )
    for op in project_spec.operations
        add_operation!(
            new_spec,
            construct_operation(
                new_spec,
                op.station_id,
                map(o->get_id(get_object_id(o)), collect(op.pre)),
                map(o->get_id(get_object_id(o)), collect(op.post)),
                op.Δt
            )
        )
    end
    new_spec
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

    construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;Δt_collect=Δt_collect,Δt_deliver=Δt_deliver)
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

    project_spec, problem_spec, robot_ICs, factory_env
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
        vcat(map(spec->spec.initial_conditions, specs)...),
        vcat(map(spec->spec.final_conditions, specs)...)
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
title_string(op::Operation,verbose=true)= verbose ? string("op",get_id(get_operation_id(op))) : "op";
title_string(a::TEAM_ACTION{A},verbose=true) where {A} = verbose ? string("team", A, "\n","r: (",map(i->string(get_id(get_robot_id(i)), ","), a.instructions)...,")") : string("team", A)

Base.string(pred::OBJECT_AT) =  string("O(",get_id(get_object_id(pred)),",",get_id(get_location_id(pred)),")")
Base.string(pred::ROBOT_AT)  =  string("R(",get_id(get_robot_id(pred)),",",get_id(get_location_id(pred)),")")
Base.string(a::GO)        =  string("GO(",get_id(get_robot_id(a)),",",get_id(get_initial_location_id(a)),"->",get_id(get_destination_location_id(a)),")")
Base.string(a::COLLECT)   =  string("COLLECT(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a)),")")
Base.string(a::CARRY)     =  string("CARRY(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_initial_location_id(a)),"->",get_id(get_destination_location_id(a)),")")
Base.string(a::DEPOSIT)   =  string("DEPOSIT(",get_id(get_robot_id(a)),",",get_id(get_object_id(a)),",",get_id(get_location_id(a)),")")
Base.string(op::Operation)=  string("OP(",get_id(get_operation_id(op)),")")
Base.string(a::TEAM_ACTION{A}) where {A} =  string("TEAM_ACTION( ",map(i->string(string(i), ","), a.instructions)...," )")

function get_display_metagraph(project_schedule::OperatingSchedule;
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
function Base.:(==)(o1::OBJECT_AT,o2::OBJECT_AT)
    try
        @assert(o1.o == o2.o)
        @assert(o1.x == o2.x)
    catch e
        if isa(e,AssertionError)
            # println(e.msg)
        else
            throw(e)
        end
        return false
    end
    return true
end
function Base.:(==)(op1::Operation,op2::Operation)
    try
        @assert(op1.pre == op2.pre)
        @assert(op1.post == op2.post)
        @assert(op1.Δt == op2.Δt)
        @assert(op1.station_id == op2.station_id)
    catch e
        if isa(e,AssertionError)
            # println(e.msg)
        else
            throw(e)
        end
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
        @assert(spec1.terminal_vtxs == spec2.terminal_vtxs)
        @assert(spec1.weights == spec2.weights)
        @assert(spec1.weight == spec2.weight)
        @assert(spec1.object_id_to_idx == spec2.object_id_to_idx)
    catch e
        if isa(e,AssertionError)
            # println(e.msg)
        else
            throw(e)
        end
        return false
    end
    return true
end


export
    get_object_paths

function get_object_paths(solution,schedule,cache)
    robot_paths = convert_to_vertex_lists(solution)
    tF = maximum(map(length, robot_paths))
    object_paths = Vector{Vector{Int}}()
    object_intervals = Vector{Vector{Int}}()
    object_ids = Int[]
    path_idxs = Int[]
    for v in vertices(schedule.graph)
        node = get_node_from_id(schedule,get_vtx_id(schedule,v))
        if isa(node, Union{CARRY,TEAM_ACTION{CARRY}})
            if isa(node, CARRY)
                object_id = get_object_id(node)
                agent_id_list = [get_id(get_robot_id(node))]
                s0_list = [get_id(get_initial_location_id(node))]
                sF_list = [get_id(get_destination_location_id(node))]
            else
                object_id = get_object_id(node.instructions[1])
                agent_id_list = map(n->get_id(get_robot_id(n)), node.instructions)
                s0_list = map(n->get_id(get_initial_location_id(n)), node.instructions)
                sF_list = map(n->get_id(get_destination_location_id(n)), node.instructions)
            end
            object_vtx = get_vtx(schedule,object_id)
            for (idx,(agent_id,s0,sF)) in enumerate(zip(agent_id_list,s0_list,sF_list))
                if get_id(agent_id) != -1
                    push!(object_paths,[
                        map(t->s0,0:cache.t0[v]-1)...,
                        map(t->robot_paths[agent_id][t],min(cache.t0[v]+1,tF):min(cache.tF[v]+1,tF,length(robot_paths[agent_id])))...,
                        map(t->sF,min(cache.tF[v]+1,tF):tF)...
                    ])
                    push!(object_intervals,[cache.t0[object_vtx],cache.tF[v]+1])
                    push!(object_ids, get_id(object_id))
                    push!(path_idxs, idx)
                end
            end
        end
    end
    object_paths, object_intervals, object_ids, path_idxs
end
function get_object_paths(solution,env)
    get_object_paths(solution,env.schedule,env.cache)
end

export
    fill_object_path_dicts!,
    convert_to_path_vectors

function fill_object_path_dicts!(solution,project_schedule,cache,
        object_path_dict = Dict{Int,Vector{Vector{Int}}}(),
        object_interval_dict = Dict{Int,Vector{Int}}()
    )
    object_paths, object_intervals, object_ids, path_idxs = get_object_paths(solution,project_schedule,cache)
    for (path,interval,id,idx) in zip(object_paths, object_intervals, object_ids, path_idxs)
        if !haskey(object_path_dict,id)
            object_path_dict[id] = Vector{Vector{Int}}()
        end
        if length(object_path_dict[id]) >= idx
            object_path_dict[id][idx] = path
        else
            push!(object_path_dict[id], path)
        end
        object_interval_dict[id] = interval
    end
    object_path_dict, object_interval_dict
end

function convert_to_path_vectors(object_path_dict, object_interval_dict)
    object_paths = Vector{Vector{Int}}()
    object_intervals = Vector{Vector{Int}}()
    for id in sort(collect(keys(object_path_dict)))
        paths = object_path_dict[id]
        for path in paths
            push!(object_intervals, object_interval_dict[id])
            push!(object_paths, path)
        end
    end
    object_paths, object_intervals
end

# end # module TaskGraphsUtils
